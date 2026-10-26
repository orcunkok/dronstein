using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Fly the Skydio X2 and move its SO-100 arm with an Xbox controller.
	/// The bindings are in ControllerBindings.txt in the project folder.
	/// Keys: Y flies to the next corner of the test square (blue dots on the ground), for tuning;
	/// R shows/hides a wire sphere of how far the gripper can reach; F switches the camera between the drone and the gripper, W/S zoom in/out. Drag with the right mouse button to orbit the camera.
	/// Attach to the entity that holds dronstein.xml (its MujocoScene).
	///
	/// The sticks move a target point; a cascaded PID (position -> velocity -> tilt) flies the drone to it.
	/// All math is in MuJoCo's frame (z up), straight from qpos/qvel.
	/// </summary>
	public class DroneTeleop : Entity
	{
		public float MoveSpeed = 0.1f;  // m/s
		public float TurnSpeed = 1.0f;  // rad/s
		public float ArmSpeed = 1.0f;   // rad/s
		public float PreciseScale = 0.25f;  // speed factor in precise mode
		public float MinHeight = 0.269f;    // m, the target point never goes below this
		public float OrbitSpeed = 0.005f;   // rad per pixel of mouse drag
		public float GripperViewDistance = 1.232f;  // m, starting camera distance when it looks at the gripper
		public float ZoomSpeed = 1.427f;    // m/s while W or S is held

		// Position loop: error -> desired velocity, capped by Tuning.MaxSpeed and by what BrakeAcc can stop in time.
		public float KpPos = 2.0f, BrakeAcc = 8f;
		// Velocity loop + integral of the position error (cancels steady offsets the P terms leave).
		public float KpVel = 6.0f, KiPos = 8.0f, IntegralRange = 0.3f;  // integrate only within IntegralRange m
		public float StickSmooth = 3.5f;           // 1/s, how quickly stick speed builds up and dies down (higher = snappier, more tilt)
		public float MaxTiltAcc = 4.0f;            // m/s^2, caps horizontal acceleration (~22 deg tilt)
		public float KpAtt = 21.258f, KdAtt = 1.5f;  // attitude loop

		static readonly string[] Motors = { "thrust1", "thrust2", "thrust3", "thrust4" };
		static readonly string[] ArmJoints = { "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper" };
		static readonly string[] JointSelects = { "select_shoulder", "select_elbow", "select_wrist" };  // bindings for shoulder_lift, elbow_flex, wrist_flex
		// Rotor positions in the drone body frame and yaw torque per newton, from dronstein.xml.
		static readonly float[,] Rotors = { { -0.14f, -0.18f, -0.0201f }, { -0.14f, 0.18f, 0.0201f }, { 0.14f, 0.18f, -0.0201f }, { 0.14f, -0.18f, 0.0201f } };
		const float MaxThrust = 13f, Gravity = 9.81f;
		static readonly float[,] Waypoints = { { 10, 10 }, { 10, -10 }, { -10, -10 }, { -10, 10 } };  // (x, y), MuJoCo frame
		const float MaxDt = 0.05f;  // s, longer frames (hitches) are treated as this long, so one slow frame can't kick the drone
		// Arm poses for the deploy button, without the gripper (left as is).
		static readonly float[] Stowed = { 0f, -3.32f, 3.11f, 1.18f, 0f };   // folded up
		static readonly float[] Deployed = { 0f, -2.05f, 0.38f, 0f, 0f };    // gripper pointing down, 0.53 m below

		// Set in OnCreate, hence `null!`.
		MujocoSceneComponent m_Mj = null!;
		MujocoActuator[] m_Motors = new MujocoActuator[4], m_Arm = new MujocoActuator[6];
		float[] m_ArmLow = new float[6], m_ArmHigh = new float[6];
		uint m_Drone, m_GraspSite;
		uint m_Shoulder, m_Elbow, m_Wrist;  // Upper_Arm, Lower_Arm, Wrist_Pitch_Roll: their origins are the lift, elbow and wrist pivots
		uint[] m_DroneBodies = null!;       // the drone's and arm's bodies, with their masses: they never change, so found once
		float[] m_DroneBodyMass = null!;
		float m_Mass;
		float[] m_Qpos = null!, m_Qvel = null!;
		// Reused every frame instead of allocating new arrays.
		float[,] m_R = new float[3, 3], m_Rt = new float[3, 3], m_Mix = new float[4, 5];
		float[] m_Thrust = new float[4];
		Entity m_Camera = null!, m_Body = null!, m_GripDot = null!, m_CenterDot = null!;
		float m_GripDotY, m_CenterDotY;  // the dots' fixed height above the ground
		Tuning m_Tuning = null!;  // the knobs on the entity tagged "Tuning"
		int m_SelectedBall;          // the ball shown purple (0 = none)
		Material? m_BallMaterial;    // its own material, given back when another ball is selected
		bool m_Missing;  // a tagged entity wasn't found: the script does nothing
		// Orbit camera: the camera sits on a sphere around the drone (or the gripper) and looks at its center.
		float m_OrbitYaw, m_OrbitPitch, m_OrbitDistance, m_GripperDistance;
		bool m_WatchGripper;
		Vector2 m_LastMouse;

		// Flight
		Vector3 m_Target;                  // the point the drone chases
		Vector3 m_TargetVel, m_TargetAcc;  // how fast the sticks move it (smoothed), both fed forward
		Vector3 m_ErrorIntegral;
		float m_Yaw = MathF.PI;  // heading of the nose; the X2 spawns with its nose (camera, body -x) along world -x
		int m_Waypoint = -1;
		bool m_Precise;

		// Arm
		float[] m_ArmTarget = [.. Stowed, -0.174f];  // starts stowed, gripper slightly closed
		float[]? m_ArmGoal;  // pose the arm is moving to after deploy, null when the sticks drive it
		bool m_Deployed;
		bool m_RollWrist;   // right stick left/right: false = shoulder_pan, true = wrist_roll
		int m_Joint = 1;    // the joint on right stick up/down

		// Controller: ControllerBindings.txt, parsed once when Play starts.
		readonly record struct Bind(int Axis, float Sign, int Raw, GamepadButton Button);  // an axis (Axis >= 0), raw button bN (Raw >= 0) or named button
		Dictionary<string, Bind> m_Bindings = new();
		float m_Deadzone = 0.1f;
		bool m_LogAxes;
		int m_Pad = -1;  // controller ID, -1 when none is connected

		float m_LogTimer, m_MaxTilt;

		protected override void OnCreate()
		{
			m_Mj = GetComponent<MujocoSceneComponent>()!;  // the script sits on the MuJoCo entity
			m_Mj.SetControlledManually(true);  // otherwise Lucky rewrites ctrl every step to hold the pose
			for (int i = 0; i < 4; i++)
			{
				m_Motors[i] = m_Mj.GetActuator(Motors[i])!;
				m_Motors[i].IK = false;
			}
			for (int i = 0; i < 6; i++)
			{
				m_Arm[i] = m_Mj.GetActuator(ArmJoints[i])!;
				m_Arm[i].IK = false;
				m_Mj.GetActuatorCtrlRange(m_Mj.GetActuatorIndex(ArmJoints[i]), out m_ArmLow[i], out m_ArmHigh[i]);
			}
			m_Drone = m_Mj.GetBodyID("x2");
			var bodies = new List<uint>();
			for (uint b = 0; b < m_Mj.ModelNbody; b++)
				if (m_Mj.GetBodyRootID(b) == m_Drone) bodies.Add(b);
			m_DroneBodies = bodies.ToArray();
			m_DroneBodyMass = Array.ConvertAll(m_DroneBodies, m_Mj.GetBodyMass);
			foreach (float m in m_DroneBodyMass) m_Mass += m;
			m_Qpos = new float[m_Mj.ModelNq];
			m_Qvel = new float[m_Mj.ModelNv];

			m_Mj.GetFullQpos(m_Qpos);
			m_Target = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2] + 0.5f);  // take off 0.5 m
			m_Tuning = Find("Tuning")?.As<Tuning>()!;
			m_Camera = Find("Camera");
			m_Body = Find("x2");
			m_GripDot = Find("GripDot");      // red dot on the ground straight below the gripper
			m_CenterDot = Find("CenterDot");  // green dot on the ground straight below the drone's center
			if (m_Missing) return;
			m_GripDotY = m_GripDot.Translation.Y;
			m_CenterDotY = m_CenterDot.Translation.Y;
			// Orbit camera: start from the scene camera's authored offset from the drone.
			Vector3 offset = m_Camera.Translation - m_Body.Translation;
			m_OrbitDistance = MathF.Max(offset.Length(), 0.3f);
			m_OrbitYaw = MathF.Atan2(offset.X, offset.Z);
			m_OrbitPitch = MathF.Asin(Math.Clamp(offset.Y / m_OrbitDistance, -1f, 1f));
			m_GripperDistance = GripperViewDistance;
			m_GraspSite = m_Mj.GetSiteID("grasp_site");
			m_Shoulder = m_Mj.GetBodyID("Upper_Arm");
			m_Elbow = m_Mj.GetBodyID("Lower_Arm");
			m_Wrist = m_Mj.GetBodyID("Wrist_Pitch_Roll");

			Log.Info($"DroneTeleop: start at {m_Target}, nq={m_Mj.ModelNq} nu={m_Mj.ModelNu}, {m_DroneBodies.Length} bodies, {m_Mass:F2} kg");
			LoadBindings();
		}

		// The entity with this tag; logs and sets m_Missing when there is none.
		Entity Find(string tag)
		{
			Entity? e = FindEntityByTag(tag);
			if (e == null)
			{
				Log.Error($"DroneTeleop: no entity tagged '{tag}' in the scene, the script is off");
				m_Missing = true;
			}
			return e!;
		}

		// OnPhysicsUpdate doesn't fire for MuJoCo robots, so control runs every frame from here.
		protected override void OnUpdate(float dt)
		{
			if (m_Missing) return;
			dt = MathF.Min(dt, MaxDt);
			if (m_Pad < 0 || !Input.IsControllerPresent(m_Pad))
			{
				int[] pads = Input.GetConnectedControllerIDs();  // allocates, so only while no controller is in use
				m_Pad = pads.Length > 0 ? pads[0] : -1;
			}
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

			Vector3 grasp = m_Mj.GetSitePositionMj(m_GraspSite);  // MuJoCo (x, y) on the ground is Lucky (x, -z)
			Vector3 body = m_Body.Translation;
			OrbitCamera(dt, body, new Vector3(grasp.X, grasp.Z, -grasp.Y));
			m_GripDot.Translation = new Vector3(grasp.X, m_GripDotY, -grasp.Y);
			m_CenterDot.Translation = new Vector3(body.X, m_CenterDotY, body.Z);
			if (Input.IsKeyPressed(KeyCode.R)) m_Tuning.ShowReach = !m_Tuning.ShowReach;
			if (m_Tuning.ShowReach) DrawReach(grasp);
			if (m_Tuning.Ball != m_SelectedBall && m_Tuning.SelectedBallMaterial != null) SelectBall(m_Tuning.Ball);
			LogStatus(dt, thrust);
		}

		// Paints ball n with Tuning's glowing purple material and gives the previously selected ball its own back.
		void SelectBall(int n)
		{
			BallMesh(m_SelectedBall)?.SetMaterial(0, m_BallMaterial!);
			m_SelectedBall = n;
			StaticMeshComponent? mesh = BallMesh(n);
			if (mesh == null) return;
			m_BallMaterial = mesh.GetMaterial(0);
			mesh.SetMaterial(0, m_Tuning.SelectedBallMaterial!);
		}

		// Ball n is tagged GolfBall_01, GolfBall_02...
		StaticMeshComponent? BallMesh(int n) =>
			n > 0 ? FindEntityByTag($"GolfBall_{n:00}")?.GetComponent<StaticMeshComponent>() : null;

		// The farthest the fingertip (grasp_site) can get below the drone: the lower half of a sphere around the
		// shoulder-lift pivot, with the arm's three links laid end to end as radius. Ignores the joint limits.
		void DrawReach(Vector3 tip)
		{
			Vector3 shoulder = BodyPos(m_Shoulder), elbow = BodyPos(m_Elbow), wrist = BodyPos(m_Wrist);
			float radius = (elbow - shoulder).Length() + (wrist - elbow).Length() + (tip - wrist).Length();
			Vector3 center = new(shoulder.X, shoulder.Z, -shoulder.Y);  // to Lucky's frame (y up)
			Vector4 color = new(0.2f, 0.9f, 1f, 1f);
			const int Rings = 6, Meridians = 8, Segments = 32;
			for (int r = 1; r <= Rings / 2; r++)  // latitude rings, up to the equator (the cut)
			{
				float lat = MathF.PI * r / Rings - MathF.PI / 2f, y = MathF.Sin(lat) * radius, ring = MathF.Cos(lat) * radius;
				for (int i = 0; i < Segments; i++)
				{
					float a = 2f * MathF.PI * i / Segments, b = 2f * MathF.PI * (i + 1) / Segments;
					DebugRenderer.DrawLine(center + new Vector3(MathF.Cos(a) * ring, y, MathF.Sin(a) * ring),
						center + new Vector3(MathF.Cos(b) * ring, y, MathF.Sin(b) * ring), color);
				}
			}
			for (int m = 0; m < Meridians; m++)  // quarter circles from the bottom pole up to the equator
			{
				float lon = 2f * MathF.PI * m / Meridians;
				Vector3 dir = new(MathF.Cos(lon), 0f, MathF.Sin(lon));
				for (int i = 0; i < Segments / 4; i++)
				{
					float a = MathF.PI * i / (Segments / 2) - MathF.PI / 2f, b = MathF.PI * (i + 1) / (Segments / 2) - MathF.PI / 2f;
					DebugRenderer.DrawLine(center + (dir * MathF.Cos(a) + new Vector3(0f, MathF.Sin(a), 0f)) * radius,
						center + (dir * MathF.Cos(b) + new Vector3(0f, MathF.Sin(b), 0f)) * radius, color);
				}
			}
		}

		Vector3 BodyPos(uint body)
		{
			m_Mj.GetBodyPoseMj(body, out Vector3 p, out Quaternion _, out Vector3 _, out Vector3 _);
			return p;
		}

		// Drag with the right mouse button to orbit, W/S to zoom; F switches between the drone and the gripper.
		void OrbitCamera(float dt, Vector3 drone, Vector3 gripper)
		{
			if (Input.IsKeyPressed(KeyCode.F))
			{
				m_WatchGripper = !m_WatchGripper;
				Log.Info($"Camera: {(m_WatchGripper ? "gripper" : "drone")}");
			}
			Vector2 mouse = Input.GetMousePosition();
			if (Input.IsMouseButtonDown(MouseButton.Right))
			{
				m_OrbitYaw -= (mouse.X - m_LastMouse.X) * OrbitSpeed;
				m_OrbitPitch = Math.Clamp(m_OrbitPitch + (mouse.Y - m_LastMouse.Y) * OrbitSpeed, -1.4f, 1.4f);
			}
			m_LastMouse = mouse;

			// Each view keeps its own zoom.
			float zoom = ((Input.IsKeyDown(KeyCode.S) ? 1f : 0f) - (Input.IsKeyDown(KeyCode.W) ? 1f : 0f)) * ZoomSpeed * dt;
			if (m_WatchGripper) m_GripperDistance = Math.Clamp(m_GripperDistance + zoom, 0.3f, 20f);
			else m_OrbitDistance = Math.Clamp(m_OrbitDistance + zoom, 0.3f, 20f);

			Vector3 center = m_WatchGripper ? gripper : drone;
			float dist = m_WatchGripper ? m_GripperDistance : m_OrbitDistance;
			var dir = new Vector3(MathF.Sin(m_OrbitYaw) * MathF.Cos(m_OrbitPitch), MathF.Sin(m_OrbitPitch), MathF.Cos(m_OrbitYaw) * MathF.Cos(m_OrbitPitch));
			m_Camera.Translation = center + dir * dist;
			m_Camera.Rotation = new Vector3(-m_OrbitPitch, m_OrbitYaw, 0f);  // the camera looks down its -z, so this faces the center
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

			// Keep the target above the ground, so holding "down" on the ground can't bury it.
			if (m_Target.Z < MinHeight)
			{
				m_Target.Z = MinHeight;
				m_TargetVel.Z = MathF.Max(m_TargetVel.Z, 0f);
				m_TargetAcc.Z = MathF.Max(m_TargetAcc.Z, 0f);
			}

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
			for (int i = 0; i < 3; i++)
				if (Pressed(JointSelects[i]))
				{
					m_Joint = i + 1;
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
			string path = Path.GetFullPath(Path.Combine(Application.DataDirectoryPath!,"..", "ControllerBindings.txt"));
			if (!File.Exists(path))
			{
				Log.Info($"DroneTeleop: no {path}, controller not used");
				return;
			}
			foreach (string raw in File.ReadAllLines(path))
			{
				string line = raw.Split('#')[0].Trim();
				int eq = line.IndexOf('=');
				if (eq <= 0) continue;
				string name = line[..eq].Trim(), value = line[(eq + 1)..].Trim();
				if (name == "deadzone") { if (float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out float dz)) m_Deadzone = dz; }
				else if (name == "log_axes") m_LogAxes = value == "true";
				else if (ParseBind(value) is Bind bind) m_Bindings[name] = bind;
				else Log.Info($"DroneTeleop: '{value}' for {name} is not an axis or a button, ignored");
			}
			Log.Info($"DroneTeleop: controller bindings from {path}");
		}

		// A number is an axis ("-1" = axis 1 flipped), bN is raw button N, anything else a button name (A, LeftBumper, ...).
		static Bind? ParseBind(string b)
		{
			if (int.TryParse(b.TrimStart('-'), out int axis)) return new Bind(axis, b.StartsWith('-') ? -1f : 1f, -1, default);
			if (b.StartsWith('b') && int.TryParse(b[1..], out int raw)) return new Bind(-1, 1f, raw, default);
			if (Enum.TryParse(b, out GamepadButton button)) return new Bind(-1, 1f, -1, button);
			return null;
		}

		// The binding for `name`; false when it isn't bound or no controller is connected.
		bool Bound(string name, out Bind b)
		{
			b = default;
			return m_Pad >= 0 && m_Bindings.TryGetValue(name, out b);
		}

		float AxisValue(Bind b) => Input.GetControllerAxis(m_Pad, b.Axis) * b.Sign;

		// `pressed`: went down this frame, else held.
		bool Button(Bind b, bool pressed) => b.Raw >= 0
			? (pressed ? Input.IsControllerButtonPressed(m_Pad, b.Raw) : Input.IsControllerButtonDown(m_Pad, b.Raw))
			: (pressed ? Input.IsControllerButtonPressed(m_Pad, b.Button) : Input.IsControllerButtonDown(m_Pad, b.Button));

		// Analog axis with the deadzone applied, 0 when unbound.
		float Axis(string name)
		{
			float v = Bound(name, out Bind b) && b.Axis >= 0 ? AxisValue(b) : 0f;
			return MathF.Abs(v) < m_Deadzone ? 0f : v;
		}

		// Held: a button down, or an axis (e.g. a trigger) pushed past halfway.
		bool Held(string name) => Bound(name, out Bind b) && (b.Axis >= 0 ? AxisValue(b) > 0.5f : Button(b, pressed: false));

		// Went down this frame. Only buttons count, never axes.
		bool Pressed(string name) => Bound(name, out Bind b) && b.Axis < 0 && Button(b, pressed: true);

		// +1 while `plus` is held, -1 while `minus` is, 0 for both or neither.
		float Pair(string plus, string minus) => (Held(plus) ? 1f : 0f) - (Held(minus) ? 1f : 0f);

		// Once a second: position, target, thrust and peak tilt; with log_axes = true also every controller axis and button.
		void LogStatus(float dt, float[] thrust)
		{
			m_LogTimer += dt;
			if (m_LogTimer <= 1f) return;
			if (m_Pad >= 0 && m_LogAxes)
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

		/// <summary>Cascaded PID: target point + yaw -> 4 rotor thrusts in newtons (in m_Thrust).</summary>
		float[] MotorThrusts(float dt)
		{
			m_Mj.GetFullQpos(m_Qpos);
			m_Mj.GetFullQvel(m_Qvel);
			var pos = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			var vel = new Vector3(m_Qvel[0], m_Qvel[1], m_Qvel[2]);     // world frame
			var angVel = new Vector3(m_Qvel[3], m_Qvel[4], m_Qvel[5]);  // body frame
			m_Tuning.Speed = vel.Length();
			float[,] R = m_R;  // body -> world
			QuatToMatrix(m_Qpos[3], m_Qpos[4], m_Qpos[5], m_Qpos[6], R);
			m_MaxTilt = MathF.Max(m_MaxTilt, MathF.Acos(Math.Clamp(R[2, 2], -1f, 1f)) * 180f / MathF.PI);

			// Center of mass of the drone + arm.
			var com = Vector3.Zero;
			for (int i = 0; i < m_DroneBodies.Length; i++)
			{
				m_Mj.GetBodyPoseMj(m_DroneBodies[i], out Vector3 p, out Quaternion _, out Vector3 _, out Vector3 _);
				com += p * m_DroneBodyMass[i];
			}
			com = com * (1f / m_Mass);

			// Position loop: fly at a speed we can still brake from (sqrt profile), plus the sticks' velocity.
			Vector3 err = m_Target - pos;
			float dist = err.Length();
			float speed = MathF.Min(MathF.Min(KpPos * dist, MathF.Sqrt(2f * BrakeAcc * dist)), m_Tuning.MaxSpeed);
			Vector3 velTarget = (dist > 1e-6f ? err * (speed / dist) : Vector3.Zero) + m_TargetVel;
			if (dist < IntegralRange) m_ErrorIntegral += err * dt;

			// Velocity loop: desired acceleration in world frame, horizontal part capped by the tilt limit.
			Vector3 acc = (velTarget - vel) * KpVel + m_ErrorIntegral * KiPos + m_TargetAcc;
			float horiz = MathF.Sqrt(acc.X * acc.X + acc.Y * acc.Y);
			if (horiz > MaxTiltAcc) { acc.X *= MaxTiltAcc / horiz; acc.Y *= MaxTiltAcc / horiz; }
			// Sink at most at half g: asking for more than g would point the thrust down and flip the drone.
			acc.Z = MathF.Max(acc.Z, -0.5f * Gravity);
			Vector3 force = (acc + new Vector3(0f, 0f, Gravity)) * m_Mass;
			float totalThrust = force.X * R[0, 2] + force.Y * R[1, 2] + force.Z * R[2, 2];

			// Desired orientation: body z along the force, nose pointing at m_Yaw.
			// The X2's nose (gimbal camera) is its -x side, so body +x points the opposite way.
			Vector3 z = force.Normalized();
			Vector3 y = Vector3.Cross(z, new Vector3(-MathF.Cos(m_Yaw), -MathF.Sin(m_Yaw), 0f)).Normalized();
			Vector3 x = Vector3.Cross(y, z);
			float[,] Rt = m_Rt;
			Rt[0, 0] = x.X; Rt[0, 1] = y.X; Rt[0, 2] = z.X;
			Rt[1, 0] = x.Y; Rt[1, 1] = y.Y; Rt[1, 2] = z.Y;
			Rt[2, 0] = x.Z; Rt[2, 1] = y.Z; Rt[2, 2] = z.Z;

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
			float[,] mix = m_Mix;  // every entry is rewritten here, Solve4 scrambles it
			for (int i = 0; i < 4; i++)
			{
				float rx = Rotors[i, 0] - comX, ry = Rotors[i, 1] - comY;
				mix[0, i] = 1f; mix[1, i] = ry; mix[2, i] = -rx; mix[3, i] = Rotors[i, 2];
			}
			mix[0, 4] = totalThrust; mix[1, 4] = torque.X; mix[2, 4] = torque.Y; mix[3, 4] = torque.Z;
			Solve4(mix, m_Thrust);
			for (int i = 0; i < 4; i++) m_Thrust[i] = Math.Clamp(m_Thrust[i], 0f, MaxThrust);
			return m_Thrust;
		}

		static void QuatToMatrix(float w, float x, float y, float z, float[,] R)
		{
			R[0, 0] = 1 - 2 * (y * y + z * z); R[0, 1] = 2 * (x * y - w * z);     R[0, 2] = 2 * (x * z + w * y);
			R[1, 0] = 2 * (x * y + w * z);     R[1, 1] = 1 - 2 * (x * x + z * z); R[1, 2] = 2 * (y * z - w * x);
			R[2, 0] = 2 * (x * z - w * y);     R[2, 1] = 2 * (y * z + w * x);     R[2, 2] = 1 - 2 * (x * x + y * y);
		}

		/// <summary>Solve a 4x4 system given as an augmented 4x5 matrix (Gaussian elimination) into `result`.</summary>
		static void Solve4(float[,] a, float[] result)
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
			for (int i = 0; i < 4; i++) result[i] = a[i, 4] / a[i, i];
		}
	}
}
