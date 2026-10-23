using System;
using System.Collections.Generic;
using System.IO;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Fly the Skydio X2 and move its SO-100 arm from the keyboard or an Xbox controller.
	/// Controller bindings are in ControllerBindings.txt in the project folder.
	/// Attach to the entity that holds dronstein.xml (its MujocoScene).
	///
	///   No WASD/QE/Shift/Ctrl: the editor's free camera uses those.
	///   N/M up/down   U/O turn (always, in both modes)
	///   F switches IJKL between the arm and the drone
	///   Drone: I/K forward/back   J/L left/right
	///   Arm:   J/L turn the arm (1 switches J/L to rolling the wrist and back)   2-5 pick shoulder/elbow/wrist/gripper   I/K move it
	///   G deploys the arm (gripper straight down) or stows it again   hold Space: slow, precise
	///   Y fly to the next corner of the test square (blue dots on the ground), for tuning
	///
	/// The keys move a target point; a cascaded PID (position -> velocity -> tilt) flies the drone to it.
	/// All math is in MuJoCo's frame (z up), straight from qpos/qvel.
	/// </summary>
	public class DroneTeleop : Entity
	{
		public float MoveSpeed = 1.0f;  // m/s
		public float TurnSpeed = 1.0f;  // rad/s
		public float ArmSpeed = 1.0f;   // rad/s
		public float PreciseScale = 0.25f;  // speed factor while Space is held

		// Position loop: error -> desired velocity, capped by MaxSpeed and by what BrakeAcc can stop in time.
		public float KpPos = 2.0f, MaxSpeed = 5.0f, BrakeAcc = 2.0f;
		// Velocity loop + integral of the position error (cancels steady offsets, e.g. from the rough COM below).
		public float KpVel = 6.0f, KiPos = 8.0f, IntegralRange = 0.3f;  // integrate only within IntegralRange m
		public float StickSmooth = 3.5f;           // 1/s, how quickly stick/key speed builds up and dies down (higher = snappier, more tilt)
		public float MaxTiltAcc = 4.0f;            // m/s^2, caps horizontal acceleration (~22 deg tilt)
		public float KpAtt = 10.0f, KdAtt = 1.5f;  // attitude loop

		static readonly string[] Motors = { "thrust1", "thrust2", "thrust3", "thrust4" };
		static readonly string[] ArmJoints = { "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper" };
		// Rotor positions in the drone body frame and yaw torque per newton, from x2.xml.
		static readonly float[,] Rotors = { { -0.14f, -0.18f, -0.0201f }, { -0.14f, 0.18f, 0.0201f }, { 0.14f, 0.18f, -0.0201f }, { 0.14f, -0.18f, 0.0201f } };
		const float MaxThrust = 13f;
		static readonly float[,] Waypoints = { { 10, 10 }, { 10, -10 }, { -10, -10 }, { -10, 10 } };  // (x, y), MuJoCo frame

		MujocoSceneComponent m_Mj;
		MujocoActuator[] m_Motors = new MujocoActuator[4];
		MujocoActuator[] m_Arm = new MujocoActuator[6];
		static readonly float[] Stowed = { 0f, -3.32f, 3.11f, 1.18f, 0f };   // arm folded up (gripper left as is)
		static readonly float[] Deployed = { 0f, -2.05f, 0.38f, 0f, 0f };    // gripper pointing down, 0.53 m below
		float[] m_ArmTarget = { 0f, -3.32f, 3.11f, 1.18f, 0f, -0.174f };
		float[] m_ArmGoal;  // pose the arm is moving to after G, null when the keys drive it
		float[] m_ArmLow = new float[6], m_ArmHigh = new float[6];
		uint m_Drone;
		float[] m_Qpos, m_Qvel;

		Entity m_Camera, m_Body, m_GripDot;
		uint m_GraspSite;
		Vector3 m_CameraOffset;

		Vector3 m_Target, m_TargetVel;  // the point the drone chases, and how fast the keys move it
		Vector3 m_TargetAcc;            // m_TargetVel change per second, fed forward
		Vector3 m_ErrorIntegral;
		float m_Yaw = MathF.PI;  // heading of the nose; the X2 spawns with its nose (camera, body -x) along world -x
		int m_Waypoint = -1;
		bool m_RollWrist;  // J/L: false = turn the arm, true = roll the wrist
		bool m_FlyOnIJKL;  // F: IJKL fly the drone instead of moving the arm
		bool m_Deployed;
		int m_Joint = 1;   // the arm joint on I/K
		Dictionary<string, string> m_Bindings = new();  // ControllerBindings.txt: name -> axis or button
		int m_Pad = -1;    // controller ID, -1 when none is connected
		bool m_PadPrecise; // precise mode switched on from the controller

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
			m_GraspSite = m_Mj.GetSiteID("grasp_site");

			Log.Info($"DroneTeleop: start at {m_Target}, nq={m_Mj.ModelNq} nu={m_Mj.ModelNu}");
			LoadBindings();
		}

		float m_LogTimer, m_MaxTilt;

		protected override void OnUpdate(float dt)
		{
			var forward = new Vector3(MathF.Cos(m_Yaw), MathF.Sin(m_Yaw), 0f);
			var left = new Vector3(-MathF.Sin(m_Yaw), MathF.Cos(m_Yaw), 0f);
			int[] pads = Input.GetConnectedControllerIDs();
			m_Pad = pads.Length > 0 ? pads[0] : -1;
			if (Pressed("precise"))
			{
				m_PadPrecise = !m_PadPrecise;
				Log.Info($"Precise mode: {(m_PadPrecise ? "on" : "off")}");
			}
			float scale = Input.IsKeyDown(KeyCode.Space) || m_PadPrecise ? PreciseScale : 1f;
			float move = MoveSpeed * scale, turnSpeed = TurnSpeed * scale, armSpeed = ArmSpeed * scale;
			Vector3 wantVel = Vector3.Zero;
			if (Input.IsKeyPressed(KeyCode.F))
			{
				m_FlyOnIJKL = !m_FlyOnIJKL;
				Log.Info($"IJKL: {(m_FlyOnIJKL ? "drone" : "arm")}");
			}
			if (m_FlyOnIJKL)
			{
				if (Input.IsKeyDown(KeyCode.I)) wantVel += forward * move;
				if (Input.IsKeyDown(KeyCode.K)) wantVel -= forward * move;
				if (Input.IsKeyDown(KeyCode.J)) wantVel += left * move;
				if (Input.IsKeyDown(KeyCode.L)) wantVel -= left * move;
			}
			if (Input.IsKeyDown(KeyCode.N)) wantVel.Z += move;
			if (Input.IsKeyDown(KeyCode.M)) wantVel.Z -= move;
			if (Input.IsKeyDown(KeyCode.U)) m_Yaw += turnSpeed * dt;
			if (Input.IsKeyDown(KeyCode.O)) m_Yaw -= turnSpeed * dt;
			wantVel += (forward * Axis("forward_back") + left * Axis("left_right")) * move;
			wantVel.Z += ((Held("up") ? 1f : 0f) - (Held("down") ? 1f : 0f)) * move;
			m_Yaw += ((Held("turn_left") ? 1f : 0f) - (Held("turn_right") ? 1f : 0f)) * turnSpeed * dt;
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

			// Rotating joints go on J/L, the rest on I/K (arrow keys would also drive the editor).
			if (Input.IsKeyPressed(KeyCode.D1))
			{
				m_RollWrist = !m_RollWrist;
				Log.Info($"J/L: {(m_RollWrist ? "wrist_roll" : "shoulder_pan")}");
			}
			KeyCode[] keys = { KeyCode.D2, KeyCode.D3, KeyCode.D4, KeyCode.D5 };
			int[] joints = { 1, 2, 3, 5 };  // shoulder_lift, elbow_flex, wrist_flex, gripper
			for (int i = 0; i < 4; i++)
				if (Input.IsKeyPressed(keys[i]))
				{
					m_Joint = joints[i];
					Log.Info($"I/K: {ArmJoints[m_Joint]}");
				}

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

			if (Input.IsKeyPressed(KeyCode.G) || Pressed("deploy"))
			{
				m_Deployed = !m_Deployed;
				m_ArmGoal = m_Deployed ? Deployed : Stowed;
				Log.Info($"Arm: {(m_Deployed ? "deploying" : "stowing")}");
			}

			if (!m_FlyOnIJKL)
			{
				float turn = 0f;
				if (Input.IsKeyDown(KeyCode.J)) turn += armSpeed * dt;
				if (Input.IsKeyDown(KeyCode.L)) turn -= armSpeed * dt;
				MoveArmJoint(m_RollWrist ? 4 : 0, turn);

				float armStep = 0f;
				if (Input.IsKeyDown(KeyCode.I)) armStep += armSpeed * dt;
				if (Input.IsKeyDown(KeyCode.K)) armStep -= armSpeed * dt;
				MoveArmJoint(m_Joint, armStep);
				if (turn != 0f || armStep != 0f) m_ArmGoal = null;  // the keys take over from G
			}
			float padJoint = Axis("arm_joint"), padTurn = Axis("arm_turn");
			float padGrip = (Held("gripper_open") ? 1f : 0f) - (Held("gripper_close") ? 1f : 0f);
			MoveArmJoint(m_Joint, padJoint * armSpeed * dt);
			MoveArmJoint(m_RollWrist ? 4 : 0, padTurn * armSpeed * dt);
			MoveArmJoint(5, padGrip * armSpeed * dt);
			if (padJoint != 0f || padTurn != 0f || padGrip != 0f) m_ArmGoal = null;

			// After G, move the arm joints to the pose at ArmSpeed, so the swing doesn't knock the drone.
			if (m_ArmGoal != null)
			{
				bool done = true;
				for (int i = 0; i < m_ArmGoal.Length; i++)
				{
					float step = Math.Clamp(m_ArmGoal[i] - m_ArmTarget[i], -armSpeed * dt, armSpeed * dt);
					MoveArmJoint(i, step);
					done &= MathF.Abs(m_ArmGoal[i] - m_ArmTarget[i]) < 1e-3f;
				}
				if (done) m_ArmGoal = null;
			}

			Control(dt);
			m_Camera.Translation = m_Body.Translation + m_CameraOffset;
			Vector3 grasp = m_Mj.GetSitePositionMj(m_GraspSite);  // MuJoCo (x, y) on the ground is Lucky (x, -z)
			m_GripDot.Translation = new Vector3(grasp.X, m_GripDot.Translation.Y, -grasp.Y);
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
			Log.Info($"DroneTeleop: controller bindings from {path}");
		}

		// Axis bound to `name` ("-1" = axis 1 flipped), with the deadzone applied. 0 without a controller.
		float Axis(string name)
		{
			if (m_Pad < 0 || !m_Bindings.TryGetValue(name, out string? b) || !int.TryParse(b.TrimStart('-'), out int axis)) return 0f;
			float v = Input.GetControllerAxis(m_Pad, axis) * (b.StartsWith("-") ? -1f : 1f);
			float deadzone = m_Bindings.TryGetValue("deadzone", out string? d) && float.TryParse(d, out float dz) ? dz : 0.1f;
			return MathF.Abs(v) < deadzone ? 0f : v;
		}

		// Binding held: a button held down, or an axis (e.g. a trigger) pushed past halfway.
		bool Held(string name)
		{
			if (m_Pad < 0 || !m_Bindings.TryGetValue(name, out string? b)) return false;
			if (int.TryParse(b.TrimStart('-'), out int axis)) return Input.GetControllerAxis(m_Pad, axis) * (b.StartsWith("-") ? -1f : 1f) > 0.5f;
			if (b.StartsWith("b") && int.TryParse(b[1..], out int raw)) return Input.IsControllerButtonDown(m_Pad, raw);
			return Enum.TryParse(b, out GamepadButton button) && Input.IsControllerButtonDown(m_Pad, button);
		}

		// Button bound to `name` went down this frame. Numbers are axes, so they never count as buttons.
		bool Pressed(string name)
		{
			if (m_Pad < 0 || !m_Bindings.TryGetValue(name, out string? b) || int.TryParse(b.TrimStart('-'), out _)) return false;
			if (b.StartsWith("b") && int.TryParse(b[1..], out int raw)) return Input.IsControllerButtonPressed(m_Pad, raw);
			return Enum.TryParse(b, out GamepadButton button) && Input.IsControllerButtonPressed(m_Pad, button);
		}

		void MoveArmJoint(int joint, float delta)
		{
			m_ArmTarget[joint] = Math.Clamp(m_ArmTarget[joint] + delta, m_ArmLow[joint], m_ArmHigh[joint]);
		}

		// OnPhysicsUpdate doesn't fire for MuJoCo robots, so this runs every frame from OnUpdate.
		void Control(float dt)
		{
			float[] thrust = MotorThrusts(dt);
			for (int i = 0; i < 4; i++) m_Motors[i].Value = thrust[i];
			for (int i = 0; i < 6; i++) m_Arm[i].Value = m_ArmTarget[i];

			m_LogTimer += dt;
			if (m_LogTimer > 1f)
			{
				if (m_Pad >= 0 && m_Bindings.GetValueOrDefault("log_axes") == "true")
				{
					var axes = new float[6];
					for (int i = 0; i < 6; i++) axes[i] = Input.GetControllerAxis(m_Pad, i);
					var held = new List<string>();
					for (int i = 0; i < 16; i++)
						if (Input.IsControllerButtonDown(m_Pad, i)) held.Add($"b{i}");
					foreach (GamepadButton button in Enum.GetValues<GamepadButton>())
						if (Input.IsControllerButtonDown(m_Pad, button) && !held.Contains(button.ToString())) held.Add(button.ToString());
					Log.Info($"Controller '{Input.GetControllerName(m_Pad)}': axes [{string.Join(" ", Array.ConvertAll(axes, v => v.ToString("F2")))}] held [{string.Join(" ", held)}]");
				}
				Log.Info($"DroneTeleop: pos=({m_Qpos[0]:F3}, {m_Qpos[1]:F3}, {m_Qpos[2]:F3}) target=({m_Target.X:F2}, {m_Target.Y:F2}, {m_Target.Z:F2}) thrust=[{thrust[0]:F1} {thrust[1]:F1} {thrust[2]:F1} {thrust[3]:F1}] maxTilt={m_MaxTilt:F0}deg");
				m_LogTimer = 0f; m_MaxTilt = 0f;
			}
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

			// Position loop: fly at a speed we can still brake from (sqrt profile), plus the keys' velocity.
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
			m_MaxTilt = MathF.Max(m_MaxTilt, MathF.Acos(Math.Clamp(R[2, 2], -1f, 1f)) * 180f / MathF.PI);
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
