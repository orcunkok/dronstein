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
	/// Keys: Y flies to the next corner of the test square (blue dots on the ground), for tuning; G folds/unfolds the arm; B, a ball number and Enter fly to that ball;
	/// R resets the scene; T shows/hides a wire sphere of how far the gripper can reach; C switches the camera between the drone, the gripper and the
	/// selected ball (Tuning.Ball), Q/E zoom in/out. F starts/stops the Autonomous Mode Action (starting runs Pick all balls).
	/// Tuning's Set button flies in behind and above the selected ball, with the ball inside the reach. Drag with the right mouse button to orbit the camera.
	/// The top right of the screen shows the camera view and the mode: Manual, or Calibration while Y's test square is flown.
	/// Attach to the entity that holds dronstein.xml (its MujocoScene).
	///
	/// The sticks move a target point; a cascaded PID (position -> velocity -> tilt) flies the drone to it.
	/// All math is in MuJoCo's frame (z up), straight from qpos/qvel.
	/// </summary>
	public partial class DroneTeleop : Entity
	{
		public float ArmSpeed = 1.0f;   // rad/s
		public float PreciseScale = 0.25f;  // speed factor in precise mode
		public float MinHeight = 0.269f;    // m, the target point never goes below this
		public float OrbitSpeed = 0.005f;   // rad per pixel of mouse drag
		public float GripperViewDistance = 1.232f;  // m, starting camera distance when it looks at the gripper
		public float ZoomSpeed = 1.427f;    // m/s while W or S is held
		public Material? SelectedBallMaterial;  // glowing material the selected ball (Tuning.Ball) switches to, none = not highlighted

		// Position loop: error to the reference -> velocity correction (on top of the reference velocity).
		float KpPos = 2.0f;  // not public: public fields get overwritten by values saved in the scene
		// Velocity loop + integral of the position error (cancels steady offsets the P terms leave).
		float KpVel = 6.0f, KiPos = 8.0f, IntegralRange = 0.1f;  // integrate only within IntegralRange m, and only while (nearly) hovering
		float KpAtt = 40f, KdAtt = 2.5f;  // attitude loop

		// Speed modes (Tuning.SpeedMode): Slow, Medium, Fast. A flight is a minimum-jerk quintic from the reference state to the goal, so
		// it never overshoots; the limits set its duration (peak speed = 1.875 D/T, peak acceleration = 5.774 D/T^2).
		static readonly string[] ModeNames = { "slow", "medium", "fast" };
		static readonly float[] ModeVmax = { 0.8f, 2.5f, 6f };        // m/s, horizontal
		static readonly float[] ModeVz = { 0.4f, 1.0f, 2.5f };        // m/s, vertical (flights and sticks)
		static readonly float[] ModeAmax = { 0.6f, 2.0f, 4f };        // m/s^2, the reference's peak acceleration (4 m/s^2 = 22 deg tilt)
		static readonly float[] ModeYawRate = { 0.6f, 1.5f, 2.5f };   // rad/s (flights and sticks)
		static readonly float[] ModeYawAcc = { 0.6f, 1.5f, 2.5f };    // rad/s^2 (yaw torque is limited by what the rotors can twist, ~0.3 Nm)
		static readonly float[] ModeTmin = { 1.5f, 1f, 0.8f };        // s, shortest flight
		static readonly float[] ModeStick = { 0.5f, 1.5f, 4f };       // m/s at full stick
		static readonly float[] ModeSmooth = { 3f, 2.5f, 2f };        // 1/s, how quickly stick speed builds up and dies down (higher = snappier, more tilt)
		static readonly float[] ModeTilt = { 1.2f, 3f, 6.5f };        // m/s^2, caps the commanded horizontal acceleration (reference + feedback)

		static readonly string[] Motors = { "thrust1", "thrust2", "thrust3", "thrust4" };
		static readonly string[] ArmJoints = { "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper" };
		static readonly string[] JointSelects = { "select_shoulder", "select_elbow", "select_wrist" };  // bindings for shoulder_lift, elbow_flex, wrist_flex
		// Rotor positions in the drone body frame and yaw torque per newton, from dronstein.xml.
		static readonly float[,] Rotors = { { -0.14f, -0.18f, -0.0201f }, { -0.14f, 0.18f, 0.0201f }, { 0.14f, 0.18f, -0.0201f }, { 0.14f, -0.18f, 0.0201f } };
		const float MaxThrust = 13f, Gravity = 9.81f, MaxYawTorque = 0.3f;
		static readonly float[,] Waypoints = { { 10, 10 }, { 10, -10 }, { -10, -10 }, { -10, 10 } };  // (x, y), MuJoCo frame
		static readonly string[] Views = { "Drone", "Gripper", "Ball" };  // what the camera can watch; C steps through them
		// HUD, in pixels: the labels' x from the right edge, the first baseline from the top, the row height, the values' x after the labels.
		const float HudRight = 320f, HudTop = 60f, HudRow = 44f, HudValueX = 115f;
		const float HudModeLabelDx = 33f, HudTaskLabelDx = 37f;  // right-aligns MODE and TASK with CAMERA (text has no alignment; widths measured on screen: 96, 62, 60 px)
		// Rendered pixels per Application.Width unit: the monitor scale (Hyprland, 1.6) times Lucky's editor UI scale
		// (UiScale in ~/.config/LuckyEngine/LuckyEditor/EditorApplicationSettings.yaml, 0.75). Change it if either changes.
		const float HudPixelScale = 1.6f * 0.75f;
		static readonly Vector4 Purple = new(0.8f, 0.6f, 1f, 1f), Yellow = new(1f, 0.847f, 0f, 1f);  // ball view, calibrating (#FFD800)
		const float MaxDt = 0.05f;  // s, longer frames (hitches) are treated as this long, so one slow frame can't kick the drone
		// Arm poses for the deploy button, without the gripper (left as is).
		static readonly float[] Stowed = { 0f, -3.32f, 3.11f, 1.18f, 0f };   // folded up
		static readonly float[] Deployed = { 0f, -2.01f, 1.85f, -0.35f, 1.5708f }; // upper arm ~12 deg back from straight down, forearm forward ~12 deg below level, wrist ~20 deg further down and rolled 90 deg

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
		Entity? m_Ball;              // its entity, null when there is none
		uint m_BallBody;             // and its MuJoCo body, for the ball camera
		Material? m_BallMaterial;    // its own material, given back when another ball is selected
		bool m_Missing;  // a tagged entity wasn't found: the script does nothing
		// Orbit camera: the camera sits on a sphere around what it watches (Views[m_View]) and looks at its center.
		float m_OrbitYaw, m_OrbitPitch;
		Vector3 m_OrbitCenter;  // the point it looks at, which is always the middle of the screen
		float[] m_ViewDistance = new float[3];  // each view keeps its own zoom
		int m_View;
		// HUD texts, moved only when the screen is resized and rewritten only when what they show changes.
		Entity m_HudCameraLabel = null!, m_HudCamera = null!, m_HudModeLabel = null!, m_HudMode = null!, m_HudTaskLabel = null!, m_HudTask = null!;
		Vector4 m_HudCameraColor, m_HudModeColor;  // the values' colours set in the scene, used when not purple/yellow
		(uint, uint) m_HudSize;
		(int, int, int, int, int, int) m_HudShown = (-1, -1, -2, -1, -1, -1);  // (m_View, m_SelectedBall, m_Waypoint, m_ToBall, m_PickPhase, m_PaState)
		Vector2 m_LastMouse;

		// Flight
		Vector3 m_Target;                  // the point the drone chases
		Vector3 m_TargetVel, m_TargetAcc;  // its velocity and acceleration (from the sticks, smoothed, or from the trajectory), both fed forward
		Quintic m_PosQ, m_YawQ;            // the flight to a goal (yaw uses X only)
		float m_PosT, m_YawT;              // s into them
		bool m_PosOn, m_YawOn;
		float m_YawRate;                   // rad/s, the yaw reference's rate, fed forward
		Vector3 m_AccFF;                   // the reference acceleration actually fed forward: a little ahead of m_TargetAcc, since the tilt takes AccLead to build up
		const float AccLead = 0.05f;       // s
		Vector3 m_ErrorIntegral;
		float m_Yaw = MathF.PI;  // heading of the nose; the X2 spawns with its nose (camera, body -x) along world -x
		int m_Waypoint = -1;
		int m_ToBall;         // the ball being flown to (Tuning's Navigate to ball or B), 0 = none; until the sticks take over
		bool m_AutoWas;       // Tuning.AutoOn as last seen
		int m_AutoMode;       // the Tuning.AutoMode that was started
		static readonly string[] AutoModeNames = { "navigate to ball", "pick selected ball", "pick all balls" };
		string? m_BallEntry;  // digits typed after B, null when not typing
		readonly HashSet<KeyCode> m_KeysDown = new();  // keys down last frame, for Tapped
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
		float m_PadScan;  // s until the next look for a controller while none is in use

		float m_LimV, m_LimVz, m_LimA;     // a flight's own limits instead of the mode's (the pick's slow descent), 0 = the mode's
		float m_LogTimer, m_MaxTilt;
		string m_CmdPath = "";
		float m_CmdTimer;

		// A flight: a quintic polynomial p(t) from (p0, v0, a0) to (pf, 0, 0) in time T (continuous acceleration, no overshoot), or, for a long
		// flight from rest, a straight line with a cruise: a smootherstep speed ramp up to Vl over Ta, constant Vl, the mirrored ramp down.
		struct Quintic
		{
			public Vector3 P0, V0, A0, C3, C4, C5;
			public float T;
			public bool Cruise;
			public Vector3 Dir;
			public float D, Vl, Ta;
			public void SetCruise(Vector3 p0, Vector3 dir, float d, float vl, float ta)
			{
				P0 = p0; Dir = dir; D = d; Vl = vl; Ta = ta; T = d / vl + ta;
				Cruise = true;
			}
			// Speed ramp from rest to Vl (smootherstep): distance, speed, acceleration after t.
			void Ramp(float t, out float s, out float v, out float a)
			{
				float u = t / Ta, u2 = u * u, u3 = u2 * u;
				s = Vl * Ta * u2 * u2 * (u2 - 3f * u + 2.5f);
				v = Vl * u3 * (6f * u2 - 15f * u + 10f);
				a = Vl / Ta * 30f * u2 * (u2 - 2f * u + 1f);
			}
			public void Set(Vector3 p0, Vector3 v0, Vector3 a0, Vector3 pf, float t)
			{
				P0 = p0; V0 = v0; A0 = a0; T = t;
				Cruise = false;
				Vector3 d = pf - p0;
				float t2 = t * t;
				C3 = (d * 20f - v0 * (12f * t) - a0 * (3f * t2)) * (1f / (2f * t2 * t));
				C4 = (d * -30f + v0 * (16f * t) + a0 * (3f * t2)) * (1f / (2f * t2 * t2));
				C5 = (d * 12f - v0 * (6f * t) - a0 * t2) * (1f / (2f * t2 * t2 * t));
			}
			public void Eval(float t, out Vector3 p, out Vector3 v, out Vector3 a)
			{
				if (Cruise)
				{
					float ds, dv, da;
					if (t < Ta) Ramp(t, out ds, out dv, out da);
					else if (t > T - Ta) { Ramp(T - t, out ds, out dv, out da); ds = D - ds; da = -da; }
					else { ds = Vl * (t - 0.5f * Ta); dv = Vl; da = 0f; }
					p = P0 + Dir * ds; v = Dir * dv; a = Dir * da;
					return;
				}
				float t2 = t * t, t3 = t2 * t, t4 = t3 * t;
				p = P0 + V0 * t + A0 * (0.5f * t2) + C3 * t3 + C4 * t4 + C5 * (t4 * t);
				v = V0 + A0 * t + C3 * (3f * t2) + C4 * (4f * t3) + C5 * (5f * t4);
				a = A0 + C3 * (6f * t) + C4 * (12f * t2) + C5 * (20f * t3);
			}
		}

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
			m_Target = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			m_Tuning = Find("Tuning")?.As<Tuning>()!;
			m_Camera = Find("Camera");
			m_Body = Find("x2");
			m_GripDot = Find("GripDot");      // red dot on the ground straight below the gripper
			m_CenterDot = Find("CenterDot");  // green dot on the ground straight below the drone's center
			m_HudCameraLabel = Find("HudCameraLabel");
			m_HudCamera = Find("HudCamera");
			m_HudModeLabel = Find("HudModeLabel");
			m_HudMode = Find("HudMode");
			m_HudTaskLabel = Find("HudTaskLabel");
			m_HudTask = Find("HudTask");
			if (m_Missing) return;
			m_HudCameraColor = m_HudCamera.GetComponent<TextComponent>()!.Color;
			m_HudModeColor = m_HudMode.GetComponent<TextComponent>()!.Color;
			m_GripDotY = m_GripDot.Translation.Y;
			m_CenterDotY = m_CenterDot.Translation.Y;
			// Orbit camera: start from the scene camera's authored offset from the drone.
			Vector3 offset = m_Camera.Translation - m_Body.Translation;
			m_ViewDistance[0] = MathF.Max(offset.Length(), 0.3f);
			m_OrbitYaw = MathF.Atan2(offset.X, offset.Z);
			m_OrbitPitch = MathF.Asin(Math.Clamp(offset.Y / m_ViewDistance[0], -1f, 1f));
			m_ViewDistance[1] = m_ViewDistance[2] = GripperViewDistance;
			m_GraspSite = m_Mj.GetSiteID("grasp_site");
			m_Shoulder = m_Mj.GetBodyID("Upper_Arm");
			m_Elbow = m_Mj.GetBodyID("Lower_Arm");
			m_Wrist = m_Mj.GetBodyID("Wrist_Pitch_Roll");
			PickInit();
			KeepOutInit();

			Log.Info($"DroneTeleop: start at {m_Target}, nq={m_Mj.ModelNq} nu={m_Mj.ModelNu}, {m_DroneBodies.Length} bodies, {m_Mass:F2} kg");
			LoadBindings();
			m_CmdPath = Path.GetFullPath(Path.Combine(Application.DataDirectoryPath!, "..", "Cache", "nightrun_cmd.txt"));
			Log.Info($"DroneTeleop: command file {m_CmdPath}, KpAtt={KpAtt} KdAtt={KdAtt} KpPos={KpPos} KpVel={KpVel} KiPos={KiPos}");
			FlyTo(m_Target + new Vector3(0f, 0f, 0.5f));  // take off 0.5 m
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
			if (m_Pad >= 0 && !Input.IsControllerPresent(m_Pad)) m_Pad = -1;
			if (m_Pad < 0 && (m_PadScan -= dt) <= 0f)
			{
				m_PadScan = 1f;  // GetConnectedControllerIDs allocates, so once a second rather than every frame
				int[] pads = Input.GetConnectedControllerIDs();
				m_Pad = pads.Length > 0 ? pads[0] : -1;
			}
			if (Pressed("precise"))
			{
				m_Precise = !m_Precise;
				Log.Info($"Precise mode: {(m_Precise ? "on" : "off")}");
			}
			float scale = m_Precise ? PreciseScale : 1f;

			m_Tuning.SpeedMode = Mode;
			m_Tuning.TopSpeed = ModeVmax[Mode];
			PollCommands(dt);
			if (Tapped(KeyCode.R)) ResetScene();
			if (Tapped(KeyCode.F))  // start / stop; starting always runs Pick all balls
			{
				m_Tuning.AutoOn = !m_Tuning.AutoOn;
				if (m_Tuning.AutoOn) m_Tuning.AutoMode = 2;
			}
			FlightInput(dt, scale);
			ArmInput(dt, ArmSpeed * scale);
			PickUpdate(dt);
			PickAllUpdate(dt);
			AutoUpdate();
			NavUpdate(dt);
			ScriptUpdate(dt);

			float[] thrust = MotorThrusts(dt);
			for (int i = 0; i < 4; i++) m_Motors[i].Value = thrust[i];
			TestUpdate(dt);
			for (int i = 0; i < 6; i++) m_Arm[i].Value = m_ArmTarget[i];

			Vector3 grasp = m_Mj.GetSitePositionMj(m_GraspSite);  // MuJoCo (x, y) on the ground is Lucky (x, -z)
			Vector3 body = m_Body.Translation;
			BallEntry();
			if (m_Tuning.Ball != m_SelectedBall) SelectBall(m_Tuning.Ball);
			if (m_Tuning.GoToBall) GoToBall(grasp);
			Vector3 ball = m_Ball != null ? BodyPos(m_BallBody) : Vector3.Zero;
			OrbitCamera(dt, body, new Vector3(grasp.X, grasp.Z, -grasp.Y), new Vector3(ball.X, ball.Z, -ball.Y));
			UpdateHud();
			m_GripDot.Translation = new Vector3(grasp.X, m_GripDotY, -grasp.Y);
			m_CenterDot.Translation = new Vector3(body.X, m_CenterDotY, body.Z);
			if (Tapped(KeyCode.T)) m_Tuning.ShowReach = !m_Tuning.ShowReach;
			if (m_Tuning.ShowReach) DrawReach(grasp);
			LogStatus(dt, thrust);
		}

		int Mode => Math.Clamp(m_Tuning.SpeedMode, 0, 2);

		// Starts a flight from the reference's current state (position, velocity, acceleration: no jump) to `goal`, ending at rest.
		// yaw (rad, a heading to turn to by the shortest way) is optional, null = keep the heading. The duration is the longest the limits of the mode need.
		void FlyTo(Vector3 goal, float? yaw = null) => RawFlyTo(KeepOut(goal), yaw);  // KeepOut splits it into legs near the bin (DroneTeleopKeepOut.cs)

		// One straight flight, no bin check.
		void RawFlyTo(Vector3 goal, float? yaw = null)
		{
			int m = Mode;
			float vmax = m_LimV > 0f ? m_LimV : ModeVmax[m], vz = m_LimVz > 0f ? m_LimVz : ModeVz[m], amax = m_LimA > 0f ? m_LimA : ModeAmax[m];
			goal.Z = MathF.Max(goal.Z, MinHeight);
			Vector3 d = goal - m_Target;
			float dh = MathF.Sqrt(d.X * d.X + d.Y * d.Y), dz = MathF.Abs(d.Z);
			float T = MathF.Max(MathF.Max(Duration(dh, vmax, amax), Duration(dz, vz, amax)), m_LimV > 0f ? 0.5f : ModeTmin[m]);
			float D = d.Length(), vl = MathF.Min(dh > 1e-4f ? vmax * D / dh : 1e9f, dz > 1e-4f ? vz * D / dz : 1e9f);  // top speed along the line
			float ta = 1.875f * vl / amax;
			if (D > 1e-3f || m_TargetVel.Length() > 1e-2f)
			{
				if (D > vl * ta && m_TargetVel.Length() < 0.05f && m_TargetAcc.Length() < 0.2f) m_PosQ.SetCruise(m_Target, d * (1f / D), D, vl, ta);  // long enough to reach vl, from rest
				else m_PosQ.Set(m_Target, m_TargetVel, m_TargetAcc, goal, T);
				m_PosT = 0f;
				m_PosOn = true;
			}
			if (yaw is float y) TurnTo(m_Yaw + MathF.IEEERemainder(y - m_Yaw, 2f * MathF.PI));
		}

		// Turns the heading to `yaw` exactly (rad, not wrapped: pass m_Yaw + delta for a given turn).
		void TurnTo(float yaw)
		{
			int m = Mode;
			float d = MathF.Abs(yaw - m_Yaw);
			if (d < 1e-3f) return;
			float T = MathF.Max(Duration(d, ModeYawRate[m], ModeYawAcc[m]), ModeTmin[m]);
			m_YawQ.Set(new Vector3(m_Yaw, 0, 0), new Vector3(m_YawRate, 0, 0), Vector3.Zero, new Vector3(yaw, 0, 0), T);
			m_YawT = 0f;
			m_YawOn = true;
		}

		// Time the quintic needs for distance d: its peak speed is 1.875 d/T, its peak acceleration 5.774 d/T^2.
		static float Duration(float d, float vmax, float amax) => d < 1e-4f ? 0f : MathF.Max(1.875f * d / vmax, MathF.Sqrt(5.774f * d / amax));

		// Paints ball n with the glowing purple SelectedBallMaterial and gives the previously selected ball its own back.
		// Ball n is tagged GolfBall_01, GolfBall_02...
		void SelectBall(int n)
		{
			if (m_BallMaterial != null) m_Ball?.GetComponent<StaticMeshComponent>()!.SetMaterial(0, m_BallMaterial);
			m_BallMaterial = null;
			m_SelectedBall = n;
			m_Ball = n > 0 ? FindEntityByTag($"GolfBall_{n:00}") : null;
			if (m_Ball == null) return;
			m_BallBody = m_Mj.GetBodyID($"lucky_body_{m_Ball.ID}");
			if (SelectedBallMaterial == null) return;
			var mesh = m_Ball.GetComponent<StaticMeshComponent>()!;
			m_BallMaterial = mesh.GetMaterial(0);
			mesh.SetMaterial(0, SelectedBallMaterial);
		}

		// R: everything back as when Play started (drone, arm, balls, trash can), and the script's flight and arm state with it.
		void ResetScene()
		{
			m_PickPhase = PickPhase.Off;
			m_PaState = PaState.Off;
			m_QN = m_QI = 0;
			m_NavOn = false;
			m_NavIgnore = true;
			m_Mj.ResetToInitialState(false);  // false: the simulation clock restarts too
			m_Mj.GetFullQpos(m_Qpos);
			m_Target = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			m_TargetVel = m_TargetAcc = m_ErrorIntegral = Vector3.Zero;
			m_PosOn = m_YawOn = false;
			m_YawRate = 0f;
			m_TestStep = -1;
			m_Yaw = MathF.PI;
			FlyTo(m_Target + new Vector3(0f, 0f, 0.5f));  // take off 0.5 m, as in OnCreate
			m_Waypoint = -1;
			m_ToBall = 0;
			m_ArmTarget = [.. Stowed, -0.174f];
			m_ArmGoal = null;
			m_Deployed = false;
			Log.Info("DroneTeleop: scene reset");
		}

		// B, a ball number and Enter (e.g. B 1 1 Enter) selects that ball in Tuning and flies there, like Navigate to ball.
		// A number with no ball is ignored.
		void BallEntry()
		{
			// Every key is checked every frame (single |), so Tapped always knows whether it was down last frame.
			bool b = Tapped(KeyCode.B), enter = Tapped(KeyCode.Enter) | Tapped(KeyCode.KPEnter);
			int digit = -1;
			for (int d = 0; d <= 9; d++)
				if (Tapped(KeyCode.D0 + d) | Tapped(KeyCode.KP0 + d)) digit = d;
			if (b) { m_BallEntry = ""; return; }
			if (m_BallEntry == null) return;
			if (digit >= 0) m_BallEntry += digit;
			if (!enter) return;
			if (int.TryParse(m_BallEntry, out int n) && n > 0 && FindEntityByTag($"GolfBall_{n:00}") != null)
			{
				m_Tuning.Ball = n;
				m_Tuning.GoToBall = true;
			}
			m_BallEntry = null;
		}

		// Went down this frame (down now, up last frame), however long it is held. Call once per key per frame.
		bool Tapped(KeyCode key)
		{
			bool down = Input.IsKeyDown(key), tapped = down && !m_KeysDown.Contains(key);
			if (down) m_KeysDown.Add(key);
			else m_KeysDown.Remove(key);
			return tapped;
		}

		// Tuning's Navigate to ball mode (and B): puts the reach's center (the shoulder-lift pivot) behind the selected ball along the heading
		// and above it, so the ball sits Tuning.GraspReach of the reach away, Tuning.GraspAngle below level: inside the
		// half-sphere, where the forward-reaching arm can grab it. If that would take the drone below MinHeight, it stays
		// at MinHeight and backs off instead, keeping the same distance. The pivot's offset from the drone is taken in the
		// body frame and turned to the heading, level, as the drone will be when it hovers there.
		void GoToBall(Vector3 tip)
		{
			m_Tuning.GoToBall = false;
			if (m_Ball == null) return;
			Vector3 ball = BodyPos(m_BallBody), d = BodyPos(m_Shoulder) - new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			float[,] R = m_R;  // body -> world, from this frame's MotorThrusts
			float bx = R[0, 0] * d.X + R[1, 0] * d.Y + R[2, 0] * d.Z;
			float by = R[0, 1] * d.X + R[1, 1] * d.Y + R[2, 1] * d.Z;
			float bz = R[0, 2] * d.X + R[1, 2] * d.Y + R[2, 2] * d.Z;
			// Level at heading m_Yaw, body x is (-cos, -sin, 0) and body y (sin, -cos, 0), as in MotorThrusts.
			float c = MathF.Cos(m_Yaw), s = MathF.Sin(m_Yaw);
			var offset = new Vector3(-c * bx + s * by, -s * bx - c * by, bz);
			float dist = m_Tuning.GraspReach * ReachRadius(tip), angle = m_Tuning.GraspAngle * MathF.PI / 180f;
			float up = MathF.Max(dist * MathF.Sin(angle), MinHeight + offset.Z - ball.Z);  // pivot above the ball
			float back = MathF.Sqrt(MathF.Max(dist * dist - up * up, 0f));                  // pivot behind it
			var goal = new Vector3(ball.X - c * back - offset.X, ball.Y - s * back - offset.Y, ball.Z + up - offset.Z);
			FlyTo(goal);
			m_Waypoint = -1;
			m_ToBall = m_SelectedBall;
			Log.Info($"DroneTeleop: going to ball {m_SelectedBall:00}, goal ({goal.X:F2}, {goal.Y:F2}, {goal.Z:F2})");
		}

		// Tuning's Autonomous Mode Action switch: turning it on starts the Select Mode action, turning it off stops it and
		// gives the drone back to manual (as the sticks do). Changing the Select Mode while it runs restarts it in the new mode.
		// It is turned off here once the action is over, by itself or by the sticks.
		void AutoUpdate()
		{
			int mode = Math.Clamp(m_Tuning.AutoMode, 0, 2);
			if (m_Tuning.AutoOn != m_AutoWas || (m_AutoWas && mode != m_AutoMode))
			{
				if (m_AutoWas) AutoStop();
				m_AutoWas = m_Tuning.AutoOn;
				if (m_AutoWas) AutoStart(mode);
				return;
			}
			if (!m_AutoWas) return;
			bool busy = m_AutoMode == 0 ? m_ToBall > 0 && Flying : m_AutoMode == 1 ? m_PickPhase != PickPhase.Off : m_PaState != PaState.Off;
			if (!busy) m_Tuning.AutoOn = m_AutoWas = false;
		}

		void AutoStart(int mode)
		{
			m_AutoMode = mode;
			Log.Info($"DroneTeleop: autonomous {AutoModeNames[m_AutoMode]} started");
			if (m_AutoMode == 0) m_Tuning.GoToBall = true;  // flown later this frame
			else if (m_AutoMode == 1) PickStart(m_Tuning.Ball, 'c');
			else PickAllStart(Environment.TickCount & 0xffff);
		}

		void AutoStop()
		{
			Log.Info("DroneTeleop: autonomous action stopped, manual");
			m_Waypoint = -1;
			m_ToBall = 0;
			m_PosOn = m_YawOn = false;
			PickEnd(false, "stopped");
			if (m_PaState != PaState.Off) PickAllEnd("stopped");
			m_QN = m_QI = 0;
		}

		// Top right of the screen: the camera view (purple on a ball), the mode (yellow while calibrating) and the task, the step of a pick.
		// Screen-space text is placed in rendered pixels from the bottom left. Application.Width/Height are smaller than that
		// by HudPixelScale, which scripts can't read, so it is a constant.
		void UpdateHud()
		{
			(uint Width, uint Height) size = ((uint)MathF.Round(Application.Width * HudPixelScale), (uint)MathF.Round(Application.Height * HudPixelScale));
			if (size != m_HudSize)
			{
				m_HudSize = size;
				float x = size.Width - HudRight, y = size.Height - HudTop;
				m_HudCameraLabel.Translation = new Vector3(x, y, 0f);
				m_HudCamera.Translation = new Vector3(x + HudValueX, y, 0f);
				m_HudModeLabel.Translation = new Vector3(x + HudModeLabelDx, y - HudRow, 0f);
				m_HudMode.Translation = new Vector3(x + HudValueX, y - HudRow, 0f);
				m_HudTaskLabel.Translation = new Vector3(x + HudTaskLabelDx, y - 2f * HudRow, 0f);
				m_HudTask.Translation = new Vector3(x + HudValueX, y - 2f * HudRow, 0f);
			}
			var shown = (m_View, m_SelectedBall, m_Waypoint, m_ToBall, (int)m_PickPhase, (int)m_PaState);
			if (shown == m_HudShown) return;
			m_HudShown = shown;
			SetText(m_HudCamera, m_View == 2 ? $"Ball {m_SelectedBall:00}" : Views[m_View], m_View == 2 ? Purple : m_HudCameraColor);
			bool pick = m_PickPhase != PickPhase.Off, pickAll = m_PaState != PaState.Off;  // Pick All flies to its start point before the first pick
			SetText(m_HudMode, pick ? $"Pick {m_PickN:00}" : pickAll ? "Pick All" : m_Waypoint >= 0 ? $"Calibration {m_Waypoint + 1}/4" : m_ToBall > 0 ? $"ToBall {m_ToBall:00}" : "Manual",
				pick || pickAll ? Purple : m_Waypoint >= 0 ? Yellow : m_ToBall > 0 ? Purple : m_HudModeColor);
			SetText(m_HudTask, pick ? (m_PickPhase == PickPhase.HoldB ? "Hold" : m_PickPhase.ToString()) : m_PaState == PaState.Start ? "Go to start" : pickAll ? "Next ball" : "-",
				pick || pickAll ? Purple : m_HudModeColor);
		}

		static void SetText(Entity entity, string text, Vector4 color)
		{
			var t = entity.GetComponent<TextComponent>()!;
			t.Text = text;
			t.Color = color;
		}

		// The farthest the fingertip (grasp_site) can get below the drone: the lower half of a sphere around the
		// shoulder-lift pivot, with the arm's three links laid end to end as radius. Ignores the joint limits.
		void DrawReach(Vector3 tip)
		{
			Vector3 shoulder = BodyPos(m_Shoulder);
			float radius = ReachRadius(tip);
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

		// The arm's three links laid end to end, from the shoulder-lift pivot to the fingertip `tip`.
		float ReachRadius(Vector3 tip)
		{
			Vector3 shoulder = BodyPos(m_Shoulder), elbow = BodyPos(m_Elbow), wrist = BodyPos(m_Wrist);
			return (elbow - shoulder).Length() + (wrist - elbow).Length() + (tip - wrist).Length();
		}

		Vector3 BodyPos(uint body)
		{
			m_Mj.GetBodyPoseMj(body, out Vector3 p, out Quaternion _, out Vector3 _, out Vector3 _);
			return p;
		}

		// Drag with the right mouse button to orbit, Q/E to zoom; C steps through the drone, the gripper and the selected ball.
		void OrbitCamera(float dt, Vector3 drone, Vector3 gripper, Vector3 ball)
		{
			if (Tapped(KeyCode.C)) m_View = (m_View + 1) % Views.Length;
			if (m_View == 2 && m_Ball == null) m_View = 0;  // no ball selected: skip the ball view
			Vector2 mouse = Input.GetMousePosition();
			if (Input.IsMouseButtonDown(MouseButton.Right))
			{
				m_OrbitYaw -= (mouse.X - m_LastMouse.X) * OrbitSpeed;
				m_OrbitPitch = Math.Clamp(m_OrbitPitch + (mouse.Y - m_LastMouse.Y) * OrbitSpeed, -1.4f, 1.4f);
			}
			m_LastMouse = mouse;

			float zoom = ((Input.IsKeyDown(KeyCode.E) ? 1f : 0f) - (Input.IsKeyDown(KeyCode.Q) ? 1f : 0f)) * ZoomSpeed * dt;
			float dist = m_ViewDistance[m_View] = Math.Clamp(m_ViewDistance[m_View] + zoom, 0.3f, 20f);

			Vector3 center = m_OrbitCenter = m_View == 0 ? drone : m_View == 1 ? gripper : ball;
			var dir = new Vector3(MathF.Sin(m_OrbitYaw) * MathF.Cos(m_OrbitPitch), MathF.Sin(m_OrbitPitch), MathF.Cos(m_OrbitYaw) * MathF.Cos(m_OrbitPitch));
			m_Camera.Translation = center + dir * dist;
			m_Camera.Rotation = new Vector3(-m_OrbitPitch, m_OrbitYaw, 0f);  // the camera looks down its -z, so this faces the center
		}

		// Left stick, triggers and bumpers move the target point and heading; Y jumps it to the next waypoint (calibration
		// mode, until the sticks are used again).
		void FlightInput(float dt, float scale)
		{
			int mode = Mode;
			var forward = new Vector3(MathF.Cos(m_Yaw), MathF.Sin(m_Yaw), 0f);
			var left = new Vector3(-MathF.Sin(m_Yaw), MathF.Cos(m_Yaw), 0f);
			Vector3 wantVel = (forward * Axis("forward_back") + left * Axis("left_right")) * (ModeStick[mode] * scale);
			wantVel.Z = Pair("up", "down") * ModeVz[mode] * scale;
			float turn = Pair("turn_left", "turn_right");
			if (wantVel.Length() > 0f || turn != 0f)  // the sticks take over: back to manual mode, continuing from the reference state
			{
				m_Waypoint = -1;
				m_ToBall = 0;
				m_PosOn = m_YawOn = false;
				PickEnd(false, "sticks");
				m_QN = m_QI = 0;
			}

			if (m_YawOn)
			{
				m_YawQ.Eval(m_YawT += dt, out Vector3 p, out Vector3 v, out Vector3 _);
				if (m_YawT >= m_YawQ.T) { m_YawQ.Eval(m_YawQ.T, out p, out v, out Vector3 _); v = Vector3.Zero; m_YawOn = false; }
				m_Yaw = p.X;
				m_YawRate = v.X;
			}
			else
			{
				m_YawRate = 0f;
				m_Yaw += turn * ModeYawRate[mode] * scale * dt;
			}

			if (m_PosOn)
			{
				m_PosT += dt;
				if (m_PosT >= m_PosQ.T)
				{
					m_PosQ.Eval(m_PosQ.T, out m_Target, out Vector3 _, out Vector3 _);
					m_TargetVel = m_TargetAcc = m_AccFF = Vector3.Zero;
					m_PosOn = false;
					while (!m_PosOn && m_QI < m_QN) RawFlyTo(m_Q[m_QI++]);  // the next leg of a flight split by the keep-out
				}
				else m_PosQ.Eval(m_PosT, out m_Target, out m_TargetVel, out m_TargetAcc);
				m_PosQ.Eval(MathF.Min(m_PosT + AccLead, m_PosQ.T), out Vector3 _, out Vector3 _, out m_AccFF);
			}
			else
			{
				// Ease the speed in and out (critically damped), so acceleration and tilt rise and fall smoothly.
				float w = ModeSmooth[mode];
				m_TargetAcc += ((wantVel - m_TargetVel) * (w * w) - m_TargetAcc * (2f * w)) * dt;
				m_TargetVel += m_TargetAcc * dt;
				m_Target += m_TargetVel * dt;
				m_AccFF = m_TargetAcc;
			}

			// Keep the target above the ground, so holding "down" on the ground can't bury it.
			if (m_Target.Z < MinHeight)
			{
				m_Target.Z = MinHeight;
				m_TargetVel.Z = MathF.Max(m_TargetVel.Z, 0f);
				m_TargetAcc.Z = MathF.Max(m_TargetAcc.Z, 0f);
			}

			if (Tapped(KeyCode.Y))
			{
				m_Waypoint = (m_Waypoint + 1) % 4;
				m_ToBall = 0;
				FlyTo(new Vector3(Waypoints[m_Waypoint, 0], Waypoints[m_Waypoint, 1], m_Target.Z));
				Log.Info($"DroneTeleop: going to waypoint {m_Waypoint} ({Waypoints[m_Waypoint, 0]}, {Waypoints[m_Waypoint, 1]})");
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
			if (Tapped(KeyCode.G) | Pressed("deploy"))
			{
				m_Deployed = !m_Deployed;
				m_ArmGoal = m_Deployed ? Deployed : Stowed;
				Log.Info($"Arm: {(m_Deployed ? "deploying" : "stowing")}");
			}

			float turn = Axis("arm_turn"), bend = Axis("arm_joint"), grip = Pair("gripper_open", "gripper_close");
			if (turn != 0f || bend != 0f || grip != 0f) { m_ArmGoal = null; PickEnd(false, "sticks"); }  // the sticks take over from deploy
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

		// ---- Dev tools (see .claude/nightrun/BRIEF.md): the command file, and the flight test it can start.

		// Cache/nightrun_cmd.txt, polled 4 times a second: each line is a command, then the file is emptied.
		void PollCommands(float dt)
		{
			if ((m_CmdTimer -= dt) > 0f) return;
			m_CmdTimer = 0.25f;
			if (!File.Exists(m_CmdPath)) return;
			string text = File.ReadAllText(m_CmdPath);
			if (text.Length == 0) return;
			File.WriteAllText(m_CmdPath, "");
			foreach (string raw in text.Split('\n'))
				if (raw.Trim() is { Length: > 0 } line) Command(line);
		}

		// mode slow|medium|fast, reset, goto x y z [yaw_deg], ball N, set, pick N [a|b], pickall [seed], view drone|gripper|ball [distance], test flight|bin; dev: label X, rawgoto x y z, arm deploy|stow, grip v, tool x y z [dz], binc, log.
		void Command(string line)
		{
			Log.Info($"CMD {line}");
			string[] w = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
			float Num(int i) => float.Parse(w[i], CultureInfo.InvariantCulture);
			try
			{
				switch (w[0])
				{
					case "mode": { int i = Array.IndexOf(ModeNames, w[1]); if (i >= 0) m_Tuning.SpeedMode = i; break; }
					case "reset": ResetScene(); break;
					case "trace": m_Trace = w[1] == "on"; break;
					case "goto":
						m_Waypoint = -1;
						m_ToBall = 0;
						FlyTo(new Vector3(Num(1), Num(2), Num(3)), w.Length > 4 ? Num(4) * MathF.PI / 180f : null);
						break;
					case "ball": m_Tuning.Ball = int.Parse(w[1]); break;
					case "set": m_Tuning.GoToBall = true; break;
					case "pickall": PickAllStart(w.Length > 1 ? int.Parse(w[1]) : Environment.TickCount & 0xffff); break;
					case "pick": m_Tuning.Ball = int.Parse(w[1]); PickStart(m_Tuning.Ball, w.Length > 2 ? w[2][0] : 'c'); break;
					case "grip": m_ArmTarget[5] = Num(1); m_ArmGoal = null; break;  // dev: gripper command (rad)
					case "log": Log.Info($"STATE grip={Grip():F3} cmd={m_ArmTarget[5]:F3}"); break;
					case "tool": m_Tool = new Vector3(Num(1), Num(2), Num(3)); if (w.Length > 4) m_ToolDz = Num(4); break;  // dev: ball centre in the jaw frame, and its height above the ball
					case "geoms": { for (uint gi = 0; gi < m_Mj.ModelNgeom; gi++) { string gn = m_Mj.GetGeomNameByIndex(gi); if (gn.Contains(w[1])) Log.Info($"GEOM {gi} {gn}"); } break; }  // dev
					case "label": m_Case = line[6..].Trim(); break;  // dev: names the next flights in the METRIC lines
					case "rawgoto": m_Waypoint = -1; m_ToBall = 0; RawFlyTo(new Vector3(Num(1), Num(2), Num(3))); m_QN = m_QI = 0; m_NavIgnore = true; break;  // dev: a flight without the bin keep-out (to park low next to the can)
					case "arm": m_ArmGoal = w[1] == "deploy" ? Deployed : Stowed; m_Deployed = w[1] == "deploy"; break;
					case "binc": { string r = ""; for (int i = 0; i < m_BinGeoms.Length; i++) r += m_Mj.IsGeomInContact(m_BinGeoms[i]) ? $" {i}:{m_Mj.GetGeomContactForce(m_BinGeoms[i]):F2}" : ""; Log.Info($"BINC rim={m_Rim:F3} contacts[{r} ]"); break; }
					case "view": m_View = Array.FindIndex(Views, v => v.ToLowerInvariant() == w[1]); if (m_View < 0) m_View = 0; if (w.Length > 2) m_ViewDistance[m_View] = Num(2); break;
					case "test": if (w[1] == "flight") { m_TestStep = 0; m_TestNew = true; } else if (w[1] == "bin") StartBinTest(); break;
					default: Log.Info($"CMD unknown: {line}"); break;
				}
			}
			catch (Exception e) when (e is FormatException or IndexOutOfRangeException) { Log.Info($"CMD bad arguments: {line}"); }
		}

		// `test flight`: from the current point, in the current mode, climb to 1.8 m (clear of balls and bin), then these moves in turn
		// (relative to where the last one ended, yaw in degrees), 4 s of hover after each. One METRIC line per move, then `METRIC done`.
		static readonly (string Name, float X, float Y, float Z, float Yaw)[] TestMoves =
		{
			("1m", 1, 0, 0, 0), ("5m", -5, 0, 0, 0), ("20m", 20, 0, 0, 0), ("up1m", 0, 0, 1, 0), ("down1m", 0, 0, -1, 0),
			("yaw90", 0, 0, 0, 90), ("yaw180+5m", 0, 5, 0, 180), ("return", 0, 0, 0, 0),
		};
		const float TestHeight = 1.8f, TestHold = 4f, TestWindow = 3f;  // m, s hovering after each move, s of it that is measured
		bool m_Trace;                     // `trace on`: 20 TRACE lines a second during the test (along-track reference vs true state)
		float m_TraceT;
		Vector3 m_TestP0;
		int m_TestStep = -1;              // -1 = off, 0 = the climb (not measured), n = TestMoves[n - 1]
		bool m_TestNew;                   // the step has not begun yet
		float m_TestTime, m_TestHold;     // s since the step began, s hovering since the flight ended
		Vector3 m_TestStart, m_TestGoal, m_TestDir;  // where the test began, this step's goal, and its direction of travel
		float m_TestStartYaw, m_TestGoalYaw, m_TestYawDelta;
		float m_TestPeak, m_TestOver, m_TestYawOver, m_TestTilt, m_TestArrive, m_TestOut, m_TestFlight;
		int m_TestFlips, m_TestSign, m_TestN;
		double m_TestSx, m_TestSy, m_TestSz, m_TestSq, m_TestSyaw;  // sums of the error to the goal over the measured window

		// Measures from the true MuJoCo state (qpos, qvel), against the final goal, not the reference.
		void TestUpdate(float dt)
		{
			if (m_TestStep < 0) return;
			var pos = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			var vel = new Vector3(m_Qvel[0], m_Qvel[1], m_Qvel[2]);
			if (m_TestNew) BeginTestStep();
			m_TestTime += dt;
			Vector3 e = pos - m_TestGoal;
			// Heading error against the goal: the reference m_Yaw is unbounded, the true heading is it plus a small wrapped difference (no 180 deg ambiguity).
			float eYaw = MathF.IEEERemainder(MathF.Atan2(-m_R[1, 0], -m_R[0, 0]) - m_Yaw, 2f * MathF.PI) + m_Yaw - m_TestGoalYaw;
			if (m_Trace && (m_TraceT -= dt) <= 0f)
			{
				m_TraceT = 0.05f;
				Log.Info($"TRACE t={m_TestTime:F2} ref={Vector3.Dot(m_Target - m_TestP0, m_TestDir):F3} act={Vector3.Dot(pos - m_TestP0, m_TestDir):F3} " +
					$"vref={Vector3.Dot(m_TargetVel, m_TestDir):F2} vact={Vector3.Dot(vel, m_TestDir):F2} aref={Vector3.Dot(m_TargetAcc, m_TestDir):F2} tilt={m_TestTilt:F1} " +
					$"thr={MathF.Min(MathF.Min(m_Thrust[0], m_Thrust[1]), MathF.Min(m_Thrust[2], m_Thrust[3])):F1}..{MathF.Max(MathF.Max(m_Thrust[0], m_Thrust[1]), MathF.Max(m_Thrust[2], m_Thrust[3])):F1} yaw={eYaw * 57.3f:F1}");
			}
			m_TestPeak = MathF.Max(m_TestPeak, vel.Length());
			m_TestOver = MathF.Max(m_TestOver, Vector3.Dot(e, m_TestDir));
			m_TestYawOver = MathF.Max(m_TestYawOver, MathF.Sign(m_TestYawDelta) * eYaw);
			if (e.Length() > 0.02f || MathF.Abs(eYaw) > 0.01745f) m_TestOut = m_TestTime;
			else if (m_TestArrive < 0f) m_TestArrive = m_TestTime;
			{
				// Oscillation: how often the along-track error changes sign by more than 3 mm either way (1 = it overshot once and came back).
				float ea = Vector3.Dot(e, m_TestDir);
				int sign = ea > 0.003f ? 1 : ea < -0.003f ? -1 : 0;
				if (sign != 0) { if (m_TestSign != 0 && sign != m_TestSign) m_TestFlips++; m_TestSign = sign; }
			}
			if (m_TestTime > 240f) { Log.Info($"METRIC aborted mode={ModeNames[Mode]} step={m_TestStep}: too slow"); m_TestStep = -1; return; }
			if (Flying || m_YawOn) return;

			if (m_TestFlight < 0f) m_TestFlight = m_TestTime;  // the reference arrived
			m_TestHold += dt;
			if (m_TestHold > TestHold - TestWindow)
			{
				m_TestN++;
				m_TestSx += e.X; m_TestSy += e.Y; m_TestSz += e.Z;
				m_TestSq += (double)e.X * e.X + (double)e.Y * e.Y + (double)e.Z * e.Z;
				m_TestSyaw += eYaw;
			}
			if (m_TestHold < TestHold) return;

			if (m_TestStep > 0)
			{
				double n = m_TestN, mx = m_TestSx / n, my = m_TestSy / n, mz = m_TestSz / n;
				double rms = Math.Sqrt(Math.Max(m_TestSq / n - (mx * mx + my * my + mz * mz), 0.0));  // spread around the mean
				Log.Info($"METRIC mode={ModeNames[Mode]} move={TestMoves[m_TestStep - 1].Name} time={m_TestFlight:F2} peakSpeed={m_TestPeak:F2} " +
					$"overshoot={MathF.Max(m_TestOver, 0f) * 100f:F2} finalErr={Math.Sqrt(mx * mx + my * my + mz * mz) * 100:F2} settle={m_TestOut:F2} " +
					$"hoverRms={rms * 1000:F2} maxTilt={m_TestTilt:F1} yawErr={Math.Abs(m_TestSyaw / n) * 180 / Math.PI:F2} " +
					$"yawOver={MathF.Max(m_TestYawOver, 0f) * 180f / MathF.PI:F2} osc={m_TestFlips}");
			}
			else m_TestStart = m_TestGoal;
			if (++m_TestStep > TestMoves.Length) { Log.Info($"METRIC done mode={ModeNames[Mode]}"); m_TestStep = -1; }
			else m_TestNew = true;
		}

		void BeginTestStep()
		{
			m_TestNew = false;
			m_TestTime = m_TestHold = 0f;
			m_TestPeak = m_TestOver = m_TestYawOver = m_TestTilt = 0f;
			m_TestArrive = m_TestFlight = -1f;
			m_TestOut = 0f;
			m_TestFlips = m_TestSign = m_TestN = 0;
			m_TestSx = m_TestSy = m_TestSz = m_TestSq = m_TestSyaw = 0.0;
			if (m_TestStep == 0)
			{
				m_TestGoal = new Vector3(m_Target.X, m_Target.Y, MathF.Max(m_Target.Z, TestHeight));
				m_TestStartYaw = m_Yaw;
				m_TestYawDelta = 0f;
			}
			else
			{
				var mv = TestMoves[m_TestStep - 1];
				bool back = mv.Name == "return";
				m_TestGoal = back ? m_TestStart : m_Target + new Vector3(mv.X, mv.Y, mv.Z);
				m_TestYawDelta = back ? MathF.IEEERemainder(m_TestStartYaw - m_Yaw, 2f * MathF.PI) : mv.Yaw * MathF.PI / 180f;
			}
			Vector3 d = m_TestGoal - m_Target;
			m_TestP0 = m_Target;
			m_TestDir = d.Length() > 1e-3f ? d.Normalized() : Vector3.Zero;
			m_TestGoalYaw = m_Yaw + m_TestYawDelta;
			FlyTo(m_TestGoal);
			TurnTo(m_TestGoalYaw);
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
			float tilt = MathF.Acos(Math.Clamp(R[2, 2], -1f, 1f)) * 180f / MathF.PI;
			m_MaxTilt = MathF.Max(m_MaxTilt, tilt);
			m_TestTilt = MathF.Max(m_TestTilt, tilt);

			// Center of mass of the drone + arm.
			var com = Vector3.Zero;
			for (int i = 0; i < m_DroneBodies.Length; i++)
			{
				m_Mj.GetBodyPoseMj(m_DroneBodies[i], out Vector3 p, out Quaternion _, out Vector3 _, out Vector3 _);
				com += p * m_DroneBodyMass[i];
			}
			com = com * (1f / m_Mass);

			// Position loop: velocity = the reference's + a correction for the error to the reference (linear: the reference never asks for more than it can do).
			Vector3 err = m_Target - pos;
			Vector3 corr = err * KpPos;
			float cl = corr.Length(), cmax = ModeVmax[Mode];
			if (cl > cmax) corr *= cmax / cl;
			Vector3 velTarget = corr + m_TargetVel;
			// Integral only while hovering near the reference, so it can't wind up during a flight; clamped.
			if (err.Length() < IntegralRange && m_TargetVel.Length() < 0.2f)
			{
				m_ErrorIntegral += err * dt;
				m_ErrorIntegral = new Vector3(Math.Clamp(m_ErrorIntegral.X, -0.25f, 0.25f), Math.Clamp(m_ErrorIntegral.Y, -0.25f, 0.25f), Math.Clamp(m_ErrorIntegral.Z, -0.25f, 0.25f));
			}

			// Velocity loop: desired acceleration in world frame (reference acceleration fed forward), horizontal part capped by the tilt limit.
			Vector3 acc = (velTarget - vel) * KpVel + m_ErrorIntegral * KiPos + m_AccFF;
			float horiz = MathF.Sqrt(acc.X * acc.X + acc.Y * acc.Y);
			float tiltAcc = ModeTilt[Mode];
			if (horiz > tiltAcc) { acc.X *= tiltAcc / horiz; acc.Y *= tiltAcc / horiz; }
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
			var yawRef = new Vector3(R[2, 0], R[2, 1], R[2, 2]) * m_YawRate;  // world z in the body frame: the yaw rate reference as a body rate
			Vector3 torque = attErr * -KpAtt - (angVel - yawRef) * KdAtt;
			torque.Z = Math.Clamp(torque.Z, -MaxYawTorque, MaxYawTorque);  // a bigger twist would saturate the rotors and upset roll/pitch

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
