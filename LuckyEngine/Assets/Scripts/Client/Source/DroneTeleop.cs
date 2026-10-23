using System;
using System.Collections.Generic;
using System.IO;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Fly the Skydio X2 and move its SO-100 arm with an Xbox controller.
	/// The bindings are in ControllerBindings.txt in the project folder.
	/// The only key is Y: fly to the next corner of the test square (blue dots on the ground), for tuning.
	/// Attach to the entity that holds dronstein.xml (its MujocoScene).
	///
	/// The sticks move a target point; a cascaded PID (position -> velocity -> tilt) flies the drone to it.
	/// All math is in MuJoCo's frame (z up), straight from qpos/qvel.
	/// </summary>
	public class DroneTeleop : Entity
	{
		public float MoveSpeed = 1.0f;  // m/s
		public float TurnSpeed = 1.0f;  // rad/s
		public float ArmSpeed = 1.0f;   // rad/s
		public float PreciseScale = 0.25f;  // speed factor in precise mode

		// Position loop: error -> desired velocity, capped by MaxSpeed and by what BrakeAcc can stop in time.
		public float KpPos = 2.0f, MaxSpeed = 5.0f, BrakeAcc = 2.0f;
		// Velocity loop + integral of the position error (cancels steady offsets, e.g. from the rough COM below).
		public float KpVel = 6.0f, KiPos = 8.0f, IntegralRange = 0.3f;  // integrate only within IntegralRange m
		public float StickSmooth = 3.5f;           // 1/s, how quickly stick speed builds up and dies down (higher = snappier, more tilt)
		public float MaxTiltAcc = 4.0f;            // m/s^2, caps horizontal acceleration (~22 deg tilt)
		public float KpAtt = 10.0f, KdAtt = 1.5f;  // attitude loop

		static readonly string[] Motors = { "thrust1", "thrust2", "thrust3", "thrust4" };
		static readonly string[] ArmJoints = { "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper" };
		// Rotor positions in the drone body frame and yaw torque per newton, from x2.xml.
		static readonly float[,] Rotors = { { -0.14f, -0.18f, -0.0201f }, { -0.14f, 0.18f, 0.0201f }, { 0.14f, 0.18f, -0.0201f }, { 0.14f, -0.18f, 0.0201f } };
		const float MaxThrust = 13f;
		static readonly float[,] Waypoints = { { 10, 10 }, { 10, -10 }, { -10, -10 }, { -10, 10 } };  // (x, y), MuJoCo frame
		// Arm poses for the deploy button, without the gripper (left as is).
		static readonly float[] Stowed = { 0f, -3.32f, 3.11f, 1.18f, 0f };   // folded up
		static readonly float[] Deployed = { 0f, -2.05f, 0.38f, 0f, 0f };    // gripper pointing down, 0.53 m below

		MujocoSceneComponent m_Mj;
		MujocoActuator[] m_Motors = new MujocoActuator[4], m_Arm = new MujocoActuator[6];
		float[] m_ArmLow = new float[6], m_ArmHigh = new float[6];
		uint m_Drone, m_GraspSite;
		float[] m_Qpos, m_Qvel;
		Entity m_Camera, m_Body, m_GripDot, m_CenterDot;
		Vector3 m_CameraOffset;

		// Flight
		Vector3 m_Target;                  // the point the drone chases
		Vector3 m_TargetVel, m_TargetAcc;  // how fast the sticks move it (smoothed), both fed forward
		Vector3 m_ErrorIntegral;
		float m_Yaw = MathF.PI;  // heading of the nose; the X2 spawns with its nose (camera, body -x) along world -x
		int m_Waypoint = -1;
		bool m_Precise;

		// Arm
		float[] m_ArmTarget = { 0f, -3.32f, 3.11f, 1.18f, 0f, -0.174f };
		float[] m_ArmGoal;  // pose the arm is moving to after deploy, null when the sticks drive it
		bool m_Deployed;
		bool m_RollWrist;   // right stick left/right: false = shoulder_pan, true = wrist_roll
		int m_Joint = 1;    // the joint on right stick up/down

		// Controller
		Dictionary<string, string> m_Bindings = new();  // ControllerBindings.txt: name -> axis or button
		float m_Deadzone = 0.1f;
		int m_Pad = -1;  // controller ID, -1 when none is connected

		float m_LogTimer, m_MaxTilt;

		protected override void OnCreate()
		{
			m_Mj = GetComponent<MujocoSceneComponent>();
			m_Mj.SetControlledManually(true);  // otherwise Lucky rewrites ctrl every step to hold the pose
			for (int i = 0; i < 4; i++)
			{
				m_Motors[i] = m_Mj.GetActuator(Motors[i]);
				m_Motors[i].IK = false;
			}
			for (int i = 0; i < 6; i++)
			{
				m_Arm[i] = m_Mj.GetActuator(ArmJoints[i]);
				m_Arm[i].IK = false;
				m_Mj.GetActuatorCtrlRange(m_Mj.GetActuatorIndex(ArmJoints[i]), out m_ArmLow[i], out m_ArmHigh[i]);
			}
			m_Drone = m_Mj.GetBodyID("x2");
			m_Qpos = new float[m_Mj.ModelNq];
			m_Qvel = new float[m_Mj.ModelNv];

			m_Mj.GetFullQpos(m_Qpos);
			m_Target = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2] + 0.5f);  // take off 0.5 m
			// Chase camera: keep the scene camera at its authored offset from the drone.
			m_Camera = FindEntityByTag("Camera");
			m_Body = FindEntityByTag("x2");
			m_CameraOffset = m_Camera.Translation - m_Body.Translation;
			// Red dot on the ground straight below the gripper.
			m_GripDot = FindEntityByTag("GripDot");
			// Green dot on the ground straight below the drone's center.
			m_CenterDot = FindEntityByTag("CenterDot");
			m_GraspSite = m_Mj.GetSiteID("grasp_site");

			Log.Info($"DroneTeleop: start at {m_Target}, nq={m_Mj.ModelNq} nu={m_Mj.ModelNu}");
			LoadBindings();
		}

		// OnPhysicsUpdate doesn't fire for MuJoCo robots, so control runs every frame from here.
		protected override void OnUpdate(float dt)
		{
			int[] pads = Input.GetConnectedControllerIDs();
			m_Pad = pads.Length > 0 ? pads[0] : -1;
			if (Pressed("precise"))
			{
				m_Precise = !m_Precise;
				Log.Info($"Precise mode: {(m_Precise ? "on" : "off")}");
			}
			float scale = m_Precise ? PreciseScale : 1f;

			FlightInput(dt, scale);
			ArmInput(dt, ArmSpeed * scale);

			float[] thrust = MotorThrusts(dt);
			for (int i = 0; i < 4; i++) m_Motors[i].Value = thrust[i];
			for (int i = 0; i < 6; i++) m_Arm[i].Value = m_ArmTarget[i];

			m_Camera.Translation = m_Body.Translation + m_CameraOffset;
			Vector3 grasp = m_Mj.GetSitePositionMj(m_GraspSite);  // MuJoCo (x, y) on the ground is Lucky (x, -z)
			m_GripDot.Translation = new Vector3(grasp.X, m_GripDot.Translation.Y, -grasp.Y);
			m_CenterDot.Translation = new Vector3(m_Body.Translation.X, m_CenterDot.Translation.Y, m_Body.Translation.Z);
			LogStatus(dt, thrust);
		}

		// Left stick, triggers and bumpers move the target point and heading; Y jumps it to the next waypoint.
		void FlightInput(float dt, float scale)
		{
			float move = MoveSpeed * scale;
			var forward = new Vector3(MathF.Cos(m_Yaw), MathF.Sin(m_Yaw), 0f);
			var left = new Vector3(-MathF.Sin(m_Yaw), MathF.Cos(m_Yaw), 0f);
			Vector3 wantVel = (forward * Axis("forward_back") + left * Axis("left_right")) * move;
			wantVel.Z = Pair("up", "down") * move;
			m_Yaw += Pair("turn_left", "turn_right") * TurnSpeed * scale * dt;

			// Ease the speed in and out (critically damped), so acceleration and tilt rise and fall smoothly.
			m_TargetAcc += ((wantVel - m_TargetVel) * (StickSmooth * StickSmooth) - m_TargetAcc * (2f * StickSmooth)) * dt;
			m_TargetVel += m_TargetAcc * dt;
			m_Target += m_TargetVel * dt;

			if (Input.IsKeyPressed(KeyCode.Y))
			{
				m_Waypoint = (m_Waypoint + 1) % 4;
				m_Target.X = Waypoints[m_Waypoint, 0];
				m_Target.Y = Waypoints[m_Waypoint, 1];
				Log.Info($"DroneTeleop: going to waypoint {m_Waypoint} ({m_Target.X}, {m_Target.Y})");
			}
		}

		// Right stick moves the selected joints, the d-pad the gripper; deploy swings the arm to a pose.
		void ArmInput(float dt, float speed)
		{
			if (Pressed("roll_wrist"))
			{
				m_RollWrist = !m_RollWrist;
				Log.Info($"Right stick left/right: {(m_RollWrist ? "wrist_roll" : "shoulder_pan")}");
			}
			string[] selects = { "select_shoulder", "select_elbow", "select_wrist" };
			for (int i = 0; i < 3; i++)
				if (Pressed(selects[i]))
				{
					m_Joint = i + 1;  // shoulder_lift, elbow_flex, wrist_flex
					Log.Info($"Right stick up/down: {ArmJoints[m_Joint]}");
				}
			if (Pressed("deploy"))
			{
				m_Deployed = !m_Deployed;
				m_ArmGoal = m_Deployed ? Deployed : Stowed;
				Log.Info($"Arm: {(m_Deployed ? "deploying" : "stowing")}");
			}

			float turn = Axis("arm_turn"), bend = Axis("arm_joint"), grip = Pair("gripper_open", "gripper_close");
			if (turn != 0f || bend != 0f || grip != 0f) m_ArmGoal = null;  // the sticks take over from deploy
			MoveArmJoint(m_RollWrist ? 4 : 0, turn * speed * dt);
			MoveArmJoint(m_Joint, bend * speed * dt);
			MoveArmJoint(5, grip * speed * dt);

			// After deploy, move the joints to the pose at arm speed, so the swing doesn't knock the drone.
			if (m_ArmGoal == null) return;
			bool done = true;
			for (int i = 0; i < m_ArmGoal.Length; i++)
			{
				MoveArmJoint(i, Math.Clamp(m_ArmGoal[i] - m_ArmTarget[i], -speed * dt, speed * dt));
				done &= MathF.Abs(m_ArmGoal[i] - m_ArmTarget[i]) < 1e-3f;
			}
			if (done) m_ArmGoal = null;
		}

		void MoveArmJoint(int joint, float delta)
		{
			m_ArmTarget[joint] = Math.Clamp(m_ArmTarget[joint] + delta, m_ArmLow[joint], m_ArmHigh[joint]);
		}

		void LoadBindings()
		{
			string path = Path.GetFullPath(Path.Combine(Application.DataDirectoryPath, "..", "ControllerBindings.txt"));
			if (!File.Exists(path))
			{
				Log.Info($"DroneTeleop: no {path}, controller not used");
				return;
			}
			foreach (string raw in File.ReadAllLines(path))
			{
				string line = raw.Split('#')[0].Trim();
				int eq = line.IndexOf('=');
				if (eq > 0) m_Bindings[line[..eq].Trim()] = line[(eq + 1)..].Trim();
			}
			if (m_Bindings.TryGetValue("deadzone", out string? d) && float.TryParse(d, out float dz)) m_Deadzone = dz;
			Log.Info($"DroneTeleop: controller bindings from {path}");
		}

		// What `name` is bound to, or null when it isn't bound or no controller is connected.
		string? Binding(string name) => m_Pad >= 0 && m_Bindings.TryGetValue(name, out string? b) ? b : null;

		// A number is an axis ("-1" = axis 1 flipped): its value, or null for a button binding.
		float? AxisValue(string b) =>
			int.TryParse(b.TrimStart('-'), out int axis) ? Input.GetControllerAxis(m_Pad, axis) * (b.StartsWith('-') ? -1f : 1f) : null;

		// A button is a name (A, LeftBumper, ...) or bN for raw button N. `pressed`: went down this frame, else held.
		bool Button(string b, bool pressed)
		{
			if (b.StartsWith('b') && int.TryParse(b[1..], out int raw))
				return pressed ? Input.IsControllerButtonPressed(m_Pad, raw) : Input.IsControllerButtonDown(m_Pad, raw);
			return Enum.TryParse(b, out GamepadButton button)
				&& (pressed ? Input.IsControllerButtonPressed(m_Pad, button) : Input.IsControllerButtonDown(m_Pad, button));
		}

		// Analog axis with the deadzone applied, 0 when unbound.
		float Axis(string name)
		{
			float v = Binding(name) is string b ? AxisValue(b) ?? 0f : 0f;
			return MathF.Abs(v) < m_Deadzone ? 0f : v;
		}

		// Held: a button down, or an axis (e.g. a trigger) pushed past halfway.
		bool Held(string name) => Binding(name) is string b && (AxisValue(b) is float v ? v > 0.5f : Button(b, pressed: false));

		// Went down this frame. Only buttons count, never axes.
		bool Pressed(string name) => Binding(name) is string b && AxisValue(b) == null && Button(b, pressed: true);

		// +1 while `plus` is held, -1 while `minus` is, 0 for both or neither.
		float Pair(string plus, string minus) => (Held(plus) ? 1f : 0f) - (Held(minus) ? 1f : 0f);

		// Once a second: position, target, thrust and peak tilt; with log_axes = true also every controller axis and button.
		void LogStatus(float dt, float[] thrust)
		{
			m_LogTimer += dt;
			if (m_LogTimer <= 1f) return;
			if (m_Pad >= 0 && m_Bindings.GetValueOrDefault("log_axes") == "true")
			{
				var axes = new float[6];
				for (int i = 0; i < 6; i++) axes[i] = Input.GetControllerAxis(m_Pad, i);
				var held = new List<string>();
				for (int i = 0; i < 16; i++)
					if (Input.IsControllerButtonDown(m_Pad, i)) held.Add($"b{i}");
				foreach (GamepadButton button in Enum.GetValues<GamepadButton>())
					if (Input.IsControllerButtonDown(m_Pad, button)) held.Add(button.ToString());
				Log.Info($"Controller '{Input.GetControllerName(m_Pad)}': axes [{string.Join(" ", Array.ConvertAll(axes, v => v.ToString("F2")))}] held [{string.Join(" ", held)}]");
			}
			Log.Info($"DroneTeleop: pos=({m_Qpos[0]:F3}, {m_Qpos[1]:F3}, {m_Qpos[2]:F3}) target=({m_Target.X:F2}, {m_Target.Y:F2}, {m_Target.Z:F2}) thrust=[{thrust[0]:F1} {thrust[1]:F1} {thrust[2]:F1} {thrust[3]:F1}] maxTilt={m_MaxTilt:F0}deg");
			m_LogTimer = 0f;
			m_MaxTilt = 0f;
		}

		/// <summary>Cascaded PID: target point + yaw -> 4 rotor thrusts in newtons.</summary>
		float[] MotorThrusts(float dt)
		{
			m_Mj.GetFullQpos(m_Qpos);
			m_Mj.GetFullQvel(m_Qvel);
			var pos = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			var vel = new Vector3(m_Qvel[0], m_Qvel[1], m_Qvel[2]);     // world frame
			var angVel = new Vector3(m_Qvel[3], m_Qvel[4], m_Qvel[5]);  // body frame
			float[,] R = QuatToMatrix(m_Qpos[3], m_Qpos[4], m_Qpos[5], m_Qpos[6]);  // body -> world
			m_MaxTilt = MathF.Max(m_MaxTilt, MathF.Acos(Math.Clamp(R[2, 2], -1f, 1f)) * 180f / MathF.PI);

			// Mass and center of mass of the drone + arm.
			float mass = 0f;
			var com = Vector3.Zero;
			for (uint b = 0; b < m_Mj.ModelNbody; b++)
			{
				if (m_Mj.GetBodyRootID(b) != m_Drone) continue;
				m_Mj.GetBodyPoseMj(b, out Vector3 p, out Quaternion _, out Vector3 _, out Vector3 _);
				float m = m_Mj.GetBodyMass(b);
				mass += m;
				com += p * m;
			}
			com = com * (1f / mass);

			// Position loop: fly at a speed we can still brake from (sqrt profile), plus the sticks' velocity.
			Vector3 err = m_Target - pos;
			float dist = err.Length();
			float speed = MathF.Min(MathF.Min(KpPos * dist, MathF.Sqrt(2f * BrakeAcc * dist)), MaxSpeed);
			Vector3 velTarget = (dist > 1e-6f ? err * (speed / dist) : Vector3.Zero) + m_TargetVel;
			if (dist < IntegralRange) m_ErrorIntegral += err * dt;

			// Velocity loop: desired acceleration in world frame, horizontal part capped by the tilt limit.
			Vector3 acc = (velTarget - vel) * KpVel + m_ErrorIntegral * KiPos + m_TargetAcc;
			float horiz = MathF.Sqrt(acc.X * acc.X + acc.Y * acc.Y);
			if (horiz > MaxTiltAcc) { acc.X *= MaxTiltAcc / horiz; acc.Y *= MaxTiltAcc / horiz; }
			Vector3 force = (acc + new Vector3(0f, 0f, 9.81f)) * mass;
			float totalThrust = force.X * R[0, 2] + force.Y * R[1, 2] + force.Z * R[2, 2];

			// Desired orientation: body z along the force, nose pointing at m_Yaw.
			// The X2's nose (gimbal camera) is its -x side, so body +x points the opposite way.
			Vector3 z = force.Normalized();
			Vector3 y = Vector3.Cross(z, new Vector3(-MathF.Cos(m_Yaw), -MathF.Sin(m_Yaw), 0f)).Normalized();
			Vector3 x = Vector3.Cross(y, z);
			float[,] Rt = { { x.X, y.X, z.X }, { x.Y, y.Y, z.Y }, { x.Z, y.Z, z.Z } };

			// Attitude loop: torque in body frame. E = Rt^T R - R^T Rt.
			float E(int i, int j)
			{
				float e = 0f;
				for (int k = 0; k < 3; k++) e += Rt[k, i] * R[k, j] - R[k, i] * Rt[k, j];
				return e;
			}
			var attErr = new Vector3(E(2, 1), E(0, 2), E(1, 0)) * 0.5f;
			Vector3 torque = attErr * -KpAtt - angVel * KdAtt;

			// Mixer: which rotor thrusts give this total thrust and torque about the center of mass.
			Vector3 d = com - pos;
			float comX = R[0, 0] * d.X + R[1, 0] * d.Y + R[2, 0] * d.Z;  // com in body frame
			float comY = R[0, 1] * d.X + R[1, 1] * d.Y + R[2, 1] * d.Z;
			var mix = new float[4, 5];
			for (int i = 0; i < 4; i++)
			{
				float rx = Rotors[i, 0] - comX, ry = Rotors[i, 1] - comY;
				mix[0, i] = 1f; mix[1, i] = ry; mix[2, i] = -rx; mix[3, i] = Rotors[i, 2];
			}
			mix[0, 4] = totalThrust; mix[1, 4] = torque.X; mix[2, 4] = torque.Y; mix[3, 4] = torque.Z;
			float[] f = Solve4(mix);
			for (int i = 0; i < 4; i++) f[i] = Math.Clamp(f[i], 0f, MaxThrust);
			return f;
		}

		static float[,] QuatToMatrix(float w, float x, float y, float z)
		{
			return new float[,]
			{
				{ 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y) },
				{ 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x) },
				{ 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y) },
			};
		}

		/// <summary>Solve a 4x4 system given as an augmented 4x5 matrix (Gaussian elimination).</summary>
		static float[] Solve4(float[,] a)
		{
			for (int c = 0; c < 4; c++)
			{
				int pivot = c;
				for (int r = c + 1; r < 4; r++)
					if (MathF.Abs(a[r, c]) > MathF.Abs(a[pivot, c])) pivot = r;
				for (int k = 0; k < 5; k++) (a[c, k], a[pivot, k]) = (a[pivot, k], a[c, k]);
				for (int r = 0; r < 4; r++)
				{
					if (r == c) continue;
					float factor = a[r, c] / a[c, c];
					for (int k = c; k < 5; k++) a[r, k] -= factor * a[c, k];
				}
			}
			return new[] { a[0, 4] / a[0, 0], a[1, 4] / a[1, 1], a[2, 4] / a[2, 2], a[3, 4] / a[3, 3] };
		}
	}
}
