using System;
using System.Collections.Generic;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// The X500's flight controller in the sim, standing in for ArduPilot. The ground station (src/dronstein.py) flies it
	/// over GcsLink, as a GCS flies ArduPilot: RC sticks (roll, pitch, throttle = climb rate, yaw) and commands
	/// (arm, mode, takeoff, gripper). It decides what a command does, in one of these modes (ArduPilot's numbers):
	/// - LOITER (5, default): the stick sets a speed (up to LOIT_SPEED); let go and it brakes and holds its spot.
	/// - ALT_HOLD (2): the stick sets the lean (up to ANGLE_MAX); let go and it levels but coasts.
	/// - GUIDED (4): takeoff (on the ground and armed) to the commanded height, or keyboard Y's calibration square.
	/// - RTL (6, in the air): home (where it was armed): up to RTL_ALT, over, down at LAND_SPEED, then it disarms.
	/// All hold the altitude. It starts disarmed and arms or disarms on the ground only. A LOITER or ALT_HOLD command, or
	/// any stick, takes GUIDED and RTL back. In the air keyboard Y flies the next corner of the test square (the blue dots).
	/// The camera, the HUD and R (reset) are X500View's; the controller's camera bindings are read here for it.
	///
	/// Two halves, both tuned by the constants at the top of this class (ArduPilot's parameter names):
	/// - Motors(): the drone. Each motor's ESC command (0..1, as ArduPilot outputs it) becomes thrust through spin min/max,
	///   the thrust expo, max thrust and a time constant; with rotor yaw torque and drag they push the X500's MuJoCo body
	///   through six motors on its free joint. Tune SIM_* against the real drone's logs.
	/// - Control(): a stand-in autopilot (angle P -> rate PID -> quad-X mix, climb rate -> altitude), with the gains from
	///   ArduPilot's X500v2 frame file. An ArduPilot connection later only has to replace it and feed Motors() its commands.
	/// The physics body is MJCF (Assets/Robots/X500V2/x500v2.mjcf, included by the World entity's dronstein.xml): Lucky can't
	/// push a procedural body (docs/needs-AddBodyForceWorld.md). This script's entity, the procedural mesh, is parented to
	/// that body's entity.
	/// Frames: MuJoCo world (z up); the body frame is FLU (x forward, y left, z up).
	/// Attach to the HolybroX500V2 entity.
	/// </summary>
	public class X500Flight : Entity
	{
		/// LOITER (true) or ALT_HOLD; the GCS's mode command switches it.
		public static bool Loiter = true;
		/// Armed (motors spin, it can fly) or disarmed; the GCS's arm command switches it on the ground. The HUD shows it.
		public static bool Armed;
		/// The mode as ArduPilot numbers it, for the GCS.
		public static int Mode => s_Rtl ? 6 : s_Takeoff || Calibrating ? 4 : Loiter ? 5 : 2;
		/// What MODE shows on the HUD: ArduPilot's name for Mode, as the GCS shows it.
		public static string ModeName => Mode switch { 6 => "RTL", 4 => "GUIDED", 5 => "LOITER", _ => "ALT_HOLD" };
		/// Flying keyboard Y's test square; the HUD shows it in yellow.
		public static bool Calibrating => s_Calib >= 0;
		static bool s_Rtl, s_Takeoff;  // the GCS's takeoff and return home, until done or taken back
		static int s_Calib = -1;       // the test square's corner being flown to, -1 = none
		static readonly float[,] Waypoints = { { 14.3f, 1.3f }, { 14.3f, -6.3f }, { 6.7f, -6.3f }, { 6.7f, 1.3f } };  // (x, y), MuJoCo frame: the blue dots, 0.2 m in from the corners of the test mat (north-east corner)

		const float G = 9.81f, Rho = 1.225f, Deg = MathF.PI / 180f;
		const float LandTime = 1f, LandVel = 0.2f;  // ArduPilot's land detector: still this long (s), slower than this (m/s)

		MujocoSceneComponent m_Mj = null!;
		uint m_Qpos, m_Qvel;  // the free joint's places in qpos and qvel
		// The motors on the free joint (x500v2.mjcf): force along world x/y/z, torque about body x/y/z.
		static readonly string[] WrenchNames = { "x500v2_fx", "x500v2_fy", "x500v2_fz", "x500v2_tx", "x500v2_ty", "x500v2_tz" };
		readonly MujocoActuator[] m_Wrench = new MujocoActuator[6];
		float[] m_Q = null!, m_V = null!;
		float m_Mass;
		bool m_Off;

		// The tuning, in ArduPilot's names (the parameter names are in the comments). Angles and rates in radians here.
		// The drone itself (SIM_*): change these to close the sim2real gap with logs of the real one.
		float m_ThstMax = 11.3f;      // N per motor at full command (2216 KV920 + 1045 on 4S: ~1.15 kgf)
		float m_ThstExpo = 0.65f;     // thrust curve: thrust = (1-e)*u + e*u^2 of the command above spin min
		float m_MotTau = 0.05f;       // s, motor/prop time constant
		float m_YawCoef = 0.016f;     // m, rotor drag torque per newton of thrust (yaw authority)
		float m_DragCda = 0.04f;      // m^2, body drag area (Cd*A): sets top speed at full lean
		float m_DragLin = 0.48f;      // N per m/s, rotor drag (blade flapping/induced drag): 0.35/s at 1.36 kg, an estimate
		                              // inside ArduPilot's EK3_DRAG_MCOEF range (0.1-1.0); set it from the real drone's logs
		float m_ArmLen = 0.25f;       // m, centre to motor (500 mm wheelbase)
		// Motors (MOT_*): the ESC command fraction, as ArduPilot outputs it.
		float m_SpinMin = 0.09f, m_SpinMax = 0.95f;
		// Attitude (ANGLE_MAX, ATC_*), from the X500v2 frame file.
		float m_AngleMax = 30f * Deg, m_InputTc = 0.15f, m_AccYawMax = 359.55f * Deg, m_RatImax = 0.5f, m_RatFltd = 21f;
		readonly float[] m_AngP = { 26.35878f, 21.2236f, 8.53887f };  // ATC_ANG_{RLL,PIT,YAW}_P: roll, pitch, yaw
		readonly float[] m_RatP = { 0.1255951f, 0.1466683f, 1.204564f }, m_RatI = { 0.1255951f, 0.1466683f, 0.1204564f };
		readonly float[] m_RatD = { 0.004905871f, 0.005201788f, 0.02224096f };
		// Altitude hold (PSC_D_*, PILOT_*) and loiter / RTL (LOIT_*, PSC_NE_*, RTL_*, LAND_SPEED).
		float m_PosZP = 1f, m_VelZP = 5f, m_SpdUp = 2.5f, m_SpdDn = 2.5f, m_AccZ = 2.5f, m_YawRateMax = 90f * Deg;
		float m_LoitSpeed = 5f, m_LoitAcc = 2.5f, m_PosXyP = 1f, m_VelXyP = 2f, m_RtlAlt = 1f, m_RtlSpeed = 5f, m_LandSpeed = 0.5f;
		float m_TkoffAlt;  // the takeoff command's height

		GcsLink m_Gcs = null!;
		Claw? m_Claw;  // the gripper, driven by the GCS's gripper command
		readonly float[] m_Pos = new float[3], m_Vel = new float[3], m_Att = new float[3];  // NED and roll, pitch, yaw, to the GCS
		uint m_BinBody;  // the TrashCan's MuJoCo body, found in OnCreate; SendState reports it
		uint[] m_BallBodies = Array.Empty<uint>();  // the tennis balls' MuJoCo bodies (TennisBall_*), found in OnCreate; SendState reports them
		const string DebugBall = "TennisBall_01";  // the helipad's ball, for debugging: not a pick-up ball (delete it with the debug)

		int m_Pad = -1;
		float m_PadScan;
		// The camera's controller bindings (the Xbox pad as Lucky reports it): an axis (Axis >= 0, Sign flips it) or a raw
		// button (Raw). Axes: 0 left stick X, 1 left stick Y, 2 right stick X, 3 right stick Y, 4 left trigger, 5 right trigger.
		readonly Dictionary<string, (int Axis, float Sign, int Raw)> m_Bindings = new()
		{
			["camera_yaw"] = (2, -1f, -1),    // right stick left/right: orbit around
			["camera_pitch"] = (3, -1f, -1),  // right stick up/down: orbit up/down
			["zoom_in"] = (-1, 1f, 15),       // D-pad up
			["zoom_out"] = (-1, 1f, 17),      // D-pad down
			["camera_view"] = (-1, 1f, 10),   // View: next camera
		};
		const float Deadzone = 0.15f;  // stick values below this count as 0

		// Motors in FLU (x forward, y left), ArduPilot quad X order: 1 front-right CCW, 2 back-left CCW, 3 front-left CW, 4 back-right CW.
		static readonly float[] MotorX = { 1, -1, 1, -1 }, MotorY = { -1, 1, 1, -1 }, Spin = { 1, 1, -1, -1 };  // Spin +1 = CCW from above
		readonly float[] m_Cmd = new float[4], m_Thrust = new float[4];

		// Controller state
		float m_LogTimer;
		double m_LastTime = -1;
		float m_RollT, m_PitchT, m_YawT, m_YawRateT;  // shaped targets (rad, rad/s)
		float m_AltT, m_ClimbT;                        // altitude target and its climb rate
		float m_AltMin = float.NaN;                    // resting height: the altitude target never goes below it
		float m_LandT;                                 // seconds the land test has held
		bool Landed => !Armed || m_LandT >= LandTime;  // disarmed, or still on the ground for LandTime
		float m_PosX, m_PosY, m_VelX, m_VelY;          // Loiter's target position and velocity (world x/y)
		readonly float[] m_I = new float[3], m_DErr = new float[3], m_D = new float[3];
		float m_Thr;
		float m_HomeX, m_HomeY;                        // where it was armed: RTL flies back here
		readonly HashSet<string> m_ButtonsDown = new();  // buttons down last frame, for Pressed
		// Reused every frame instead of allocated: body rates, rate targets and rate PID outputs (roll, pitch, yaw), the mix.
		readonly float[] m_W = new float[3], m_RateT = new float[3], m_Out = new float[3], m_Rp = new float[4];
		readonly float[,] m_R = new float[3, 3];       // world <- body rotation, from BodyRotation
		bool m_KeyY;                                   // keyboard Y down last frame
		bool m_KeyF;                                   // keyboard F down last frame
		int m_PickUps;                                 // F presses here, to the GCS: each one starts or stops its pick-up

		// Props: entities tagged X500Prop1..4 (children of this entity, motor order), spun about their local +Y.
		readonly Entity?[] m_Props = new Entity?[4];
		readonly float[] m_PropAngle = new float[4];
		Entity? m_CenterDot;  // red dot on the ground below the drone's center (CG reference), shown while armed
		const float PropSpinMax = 60f, PropSpinIdle = 10f;  // rad/s on screen at full thrust and at idle (real rpm would alias)

		protected override void OnCreate()
		{
			m_Gcs = new GcsLink();
			Loiter = true;
			Armed = s_Rtl = s_Takeoff = false;
			s_Calib = -1;
			for (int i = 0; i < 4; i++) m_Props[i] = FindEntityByTag($"X500Prop{i + 1}");  // optional: no tag, no spin
			m_CenterDot = FindEntityByTag("CenterDot");  // optional: no tag, no dot
			Entity? scene = FindEntityByTag("World");
			if (scene == null)
			{
				Log.Error("X500Flight: needs the entity tagged World (its MuJoCo scene holds the X500); off");
				m_Off = true;
				return;
			}
			m_Mj = scene.GetComponent<MujocoSceneComponent>()!;
			m_Mj.SetControlledManually(true);  // otherwise Lucky writes its hold-pose targets into the motors' ctrl
			uint joint = m_Mj.GetJointID("x500v2_free");
			m_Qpos = m_Mj.GetJointQposAdr(joint);
			m_Qvel = m_Mj.GetJointQvelAdr(joint);
			m_Mass = m_Mj.GetBodyMass(m_Mj.GetBodyID("x500v2"));
			var balls = new List<uint>();
			for (uint i = 0; i < m_Mj.ModelNbody; i++)
			{
				string name = m_Mj.GetBodyNameByIndex(i);
				if (name.StartsWith("TennisBall_", StringComparison.Ordinal) && name != DebugBall) balls.Add(i);
			}
			m_BallBodies = balls.ToArray();
			m_BinBody = m_Mj.GetBodyID("TrashCan");
			for (int k = 0; k < 6; k++)
			{
				m_Wrench[k] = m_Mj.GetActuator(WrenchNames[k])!;
				m_Wrench[k].IK = false;
			}
			StopMotors();
			m_Q = new float[m_Mj.ModelNq];
			m_V = new float[m_Mj.ModelNv];
			Log.Info($"X500Flight: start, {m_Mass:F3} kg, hover throttle {Hover:F3}, {m_BallBodies.Length} balls");
		}

		protected override void OnDestroy() => m_Gcs?.Dispose();

		// No thrust, and ctrl zeroed: until it is written, Lucky's initial ctrl (the joint's own qpos) would push the X500.
		void StopMotors()
		{
			Array.Clear(m_Thrust);
			foreach (MujocoActuator a in m_Wrench) a.Value = 0;
		}

		float Hover => m_Mass * G / (4f * m_ThstMax);

		protected override void OnUpdate(float frameDt)
		{
			if (m_Off) return;
			if (m_Pad >= 0 && !Input.IsControllerPresent(m_Pad)) m_Pad = -1;
			if (m_Pad < 0 && (m_PadScan -= frameDt) <= 0f)
			{
				m_PadScan = 1f;  // GetConnectedControllerIDs allocates, so once a second rather than every frame
				int[] pads = Input.GetConnectedControllerIDs();
				m_Pad = pads.Length > 0 ? pads[0] : -1;
			}

			// Physics time, not frame time: the forces hold until the next frame, over however many MuJoCo steps that is.
			double now = m_Mj.MujocoTime;
			float dt = (float)(now - m_LastTime);
			if (m_LastTime < 0 || dt < 0)  // first frame, or the scene was reset (R)
			{
				m_LastTime = now;
				m_AltMin = float.NaN;
				Armed = s_Rtl = s_Takeoff = false;
				s_Calib = -1;
				StopMotors();
				return;
			}
			m_LastTime = now;
			if (dt <= 0f) return;  // paused
			dt = MathF.Min(dt, 0.05f);

			m_Mj.GetFullQpos(m_Q);
			m_Mj.GetFullQvel(m_V);
			Control(dt);
			Motors(dt);
			SendState();
			SpinProps(dt);
			CenterDot();
			Status(dt);
		}

		// ---------------------------------------------------------------- the drone

		// World <- FLU rotation of the body, from its quaternion in qpos.
		void BodyRotation(float[,] r)
		{
			float w = m_Q[m_Qpos + 3], x = m_Q[m_Qpos + 4], y = m_Q[m_Qpos + 5], z = m_Q[m_Qpos + 6];
			r[0, 0] = 1 - 2 * (y * y + z * z); r[0, 1] = 2 * (x * y - w * z); r[0, 2] = 2 * (x * z + w * y);
			r[1, 0] = 2 * (x * y + w * z); r[1, 1] = 1 - 2 * (x * x + z * z); r[1, 2] = 2 * (y * z - w * x);
			r[2, 0] = 2 * (x * z - w * y); r[2, 1] = 2 * (y * z + w * x); r[2, 2] = 1 - 2 * (x * x + y * y);
		}

		// ESC commands -> thrust (with the motors' lag) -> force and torque on the body, plus drag.
		void Motors(float dt)
		{
			float e = m_ThstExpo, lag = 1f - MathF.Exp(-dt / m_MotTau);
			float arm = m_ArmLen * 0.70710678f, kq = m_YawCoef;
			float total = 0f, tx = 0f, ty = 0f, tz = 0f;
			for (int i = 0; i < 4; i++)
			{
				float u = Math.Clamp((m_Cmd[i] - m_SpinMin) / (m_SpinMax - m_SpinMin), 0f, 1f);
				m_Thrust[i] += (m_ThstMax * ((1f - e) * u + e * u * u) - m_Thrust[i]) * lag;
				float t = m_Thrust[i];
				total += t;
				tx += MotorY[i] * arm * t;      // r x F with F along +z: (y F, -x F, 0)
				ty -= MotorX[i] * arm * t;
				tz -= Spin[i] * kq * t;         // a CCW prop twists the body CW
			}
			BodyRotation(m_R);
			float vx = m_V[m_Qvel], vy = m_V[m_Qvel + 1], vz = m_V[m_Qvel + 2];
			float speed = MathF.Sqrt(vx * vx + vy * vy + vz * vz);
			float drag = 0.5f * Rho * m_DragCda * speed + m_DragLin;
			for (int k = 0; k < 3; k++)
				m_Wrench[k].Value = m_R[k, 2] * total - drag * m_V[m_Qvel + k];
			m_Wrench[3].Value = tx;  // free joint torques are in the body frame (FLU)
			m_Wrench[4].Value = ty;
			m_Wrench[5].Value = tz;
		}

		// ---------------------------------------------------------------- the stand-in autopilot

		void Control(float dt)
		{
			// The GCS's sticks, ArduPilot's way: RC1 roll right, RC2 pitch (forward is low), RC3 climb, RC4 yaw right.
			var cmd = m_Gcs.Receive();
			if (cmd?.Cmd == "reset")  // the GCS's reset: the same as R, the scene back to its start (the next frame sees it)
			{
				m_Mj.ResetToInitialState(false);
				Log.Info("X500Flight: reset from the GCS");
				return;
			}
			float roll = m_Gcs.Stick(0), pitch = -m_Gcs.Stick(1), throttle = m_Gcs.Stick(2), turn = -m_Gcs.Stick(3);
			float climb = throttle * (throttle > 0f ? m_SpdUp : m_SpdDn);
			BodyRotation(m_R);
			float phi = MathF.Atan2(m_R[2, 1], m_R[2, 2]), theta = -MathF.Asin(Math.Clamp(m_R[2, 0], -1f, 1f));
			float psi = MathF.Atan2(m_R[1, 0], m_R[0, 0]);
			float x = m_Q[m_Qpos], y = m_Q[m_Qpos + 1], alt = m_Q[m_Qpos + 2];
			float vx = m_V[m_Qvel], vy = m_V[m_Qvel + 1], vz = m_V[m_Qvel + 2];
			(m_W[0], m_W[1], m_W[2]) = (m_V[m_Qvel + 3], m_V[m_Qvel + 4], m_V[m_Qvel + 5]);  // body rates (FLU)

			// ArduPilot's land detector, from the autopilot's own altitude target, not the sim's ground truth: the target is on
			// the floor (it only creeps there, hence the 1 cm) and the drone isn't moving up or down, for LandTime.
			m_LandT = m_AltT <= m_AltMin + 0.01f && MathF.Abs(vz) < LandVel ? m_LandT + dt : 0f;
			bool grounded = Landed;
			if (cmd?.Cmd == "arm" && grounded && (cmd.Value.Value != 0f) != Armed)  // arm / disarm, on the ground only
			{
				Armed = !Armed;
				if (Armed) (m_HomeX, m_HomeY, m_LandT) = (x, y, LandTime);  // landed until it moves: no spool-up on the ground
				Log.Info($"X500Flight: {(Armed ? "armed" : "disarmed")}");
			}
			bool keyY = Input.IsKeyDown(KeyCode.Y);
			if (keyY && !m_KeyY && Armed && !grounded)  // keyboard Y, in the air: the next corner of the test square
			{
				s_Calib = (s_Calib + 1) % 4;
				s_Rtl = s_Takeoff = false;
				(m_PosX, m_PosY, m_VelX, m_VelY) = (x, y, vx, vy);
				Log.Info($"X500Flight: going to waypoint {s_Calib} ({Waypoints[s_Calib, 0]}, {Waypoints[s_Calib, 1]})");
			}
			m_KeyY = keyY;
			bool keyF = Input.IsKeyDown(KeyCode.F);
			if (keyF && !m_KeyF) m_PickUps++;
			m_KeyF = keyF;
			int mode = cmd?.Cmd == "mode" ? (int)cmd.Value.Value : -1;
			if (mode == 5 || mode == 2 || (mode == 6 && Armed && !grounded))  // LOITER, ALT_HOLD, or RTL in the air
			{
				Loiter = mode == 5 || (mode == 6 && Loiter);
				s_Rtl = mode == 6;
				s_Takeoff = false;
				s_Calib = -1;
				(m_PosX, m_PosY, m_VelX, m_VelY) = (x, y, vx, vy);  // it starts from where it is, at the speed it has
				Log.Info($"X500Flight: {ModeName}");
			}
			if (cmd?.Cmd == "takeoff" && Armed && grounded)
			{
				m_TkoffAlt = cmd.Value.Value;
				s_Takeoff = true;
				Log.Info($"X500Flight: takeoff to {m_TkoffAlt:F2} m");
			}
			if (cmd?.Cmd == "gripper")
			{
				if (cmd.Value.Value != 0f) m_Claw?.Close();
				else m_Claw?.Open();
			}
			if (pitch != 0f || roll != 0f || climb != 0f || turn != 0f)  // the pilot takes over
			{
				s_Rtl = s_Takeoff = false;
				s_Calib = -1;
			}
			if (!Armed) climb = 0f;

			// Disarmed, or on the ground and not climbing (and at the start): no thrust, every target follows the drone.
			if (!Armed || (grounded && climb <= 0f && !s_Takeoff))
			{
				if (float.IsNaN(m_AltMin)) m_AltMin = alt;
				if (s_Rtl) { Armed = s_Rtl = false; Log.Info("X500Flight: home, landed, disarmed"); }
				m_AltT = m_AltMin;
				m_RollT = m_PitchT = m_YawRateT = m_ClimbT = m_Thr = 0f;
				m_YawT = psi;
				(m_PosX, m_PosY, m_VelX, m_VelY) = (x, y, 0f, 0f);
				Array.Clear(m_I);
				Array.Clear(m_D);
				Array.Clear(m_DErr);
				for (int i = 0; i < 4; i++) m_Cmd[i] = Armed ? m_SpinMin : 0f;  // armed: idle (no thrust yet)
				return;
			}

			// Takeoff: up to the commanded height above the ground, holding its spot. RTL, from where its target is: up to RTL_ALT
			// (unless higher), over home at RTL_SPEED braking at LOIT_ACC_MAX, then down at LAND_SPEED (it disarms on landing).
			// Calibration: over the corner the same way, at its altitude, and holds there.
			float wantSpeed = m_LoitSpeed, wantDx = 0f, wantDy = 0f;  // auto: the speed to fly and its direction (world)
			if (s_Takeoff)
			{
				float top = m_AltMin + m_TkoffAlt;
				climb = Math.Clamp(2f * (top - m_AltT), 0f, m_SpdUp);
				if (m_AltT >= top - 0.01f) s_Takeoff = false;
			}
			else if (s_Rtl)
			{
				float hx = m_HomeX - m_PosX, hy = m_HomeY - m_PosY, dist = MathF.Sqrt(hx * hx + hy * hy), top = m_AltMin + m_RtlAlt;
				if (dist < 0.05f) { (m_PosX, m_PosY) = (m_HomeX, m_HomeY); climb = -m_LandSpeed; }  // over home: land
				else if (alt < top - 0.3f) climb = Math.Clamp(2f * (top - m_AltT), 0f, m_SpdUp);  // too low: climb first
				else
				{
					wantSpeed = MathF.Min(m_RtlSpeed, MathF.Sqrt(2f * m_LoitAcc * dist));  // fast enough to stop at home
					(wantDx, wantDy) = (hx / dist, hy / dist);
				}
			}
			else if (Calibrating)
			{
				float cx = Waypoints[s_Calib, 0], cy = Waypoints[s_Calib, 1];
				float hx = cx - m_PosX, hy = cy - m_PosY, dist = MathF.Sqrt(hx * hx + hy * hy);
				if (dist < 0.05f) (m_PosX, m_PosY) = (cx, cy);  // there: hold
				else
				{
					wantSpeed = MathF.Min(m_RtlSpeed, MathF.Sqrt(2f * m_LoitAcc * dist));
					(wantDx, wantDy) = (hx / dist, hy / dist);
				}
			}
			else if (m_Gcs.Setpoint is float[] goal)  // the GCS's position setpoint (ArduPilot's GUIDED): fly there and hold it
			{
				float tx = goal[0], ty = -goal[1];  // NED -> MuJoCo: north = x, east = -y
				float hx = tx - m_PosX, hy = ty - m_PosY, dist = MathF.Sqrt(hx * hx + hy * hy);
				if (dist < 0.05f) (m_PosX, m_PosY) = (tx, ty);  // there: hold
				else
				{
					wantSpeed = MathF.Min(m_RtlSpeed, MathF.Sqrt(2f * m_LoitAcc * dist));
					(wantDx, wantDy) = (hx / dist, hy / dist);
				}
				m_YawT = -goal[3];  // the NED heading (clockwise from north) as the FLU yaw the controller holds
				float top = m_AltMin - goal[2];  // down -> the altitude above where it started
				climb = Math.Clamp(2f * (top - m_AltT), -m_LandSpeed, m_SpdUp);  // down no faster than LAND_SPEED
			}
			bool auto = s_Takeoff || s_Rtl || Calibrating || m_Gcs.Setpoint != null;

			float maxAngle = m_AngleMax, tc = m_InputTc;
			float rollIn = roll * maxAngle, pitchIn = pitch * maxAngle;  // Manual: the stick is the lean
			if (Loiter || auto)
			{
				// The stick is a velocity in the heading frame; the target velocity moves toward it at LOIT_ACC_MAX, so
				// letting go brakes at that rate. The target position follows it, and holds when it stops.
				float c = MathF.Cos(psi), s = MathF.Sin(psi), speed = wantSpeed;
				float wantX = (c * pitch + s * roll + wantDx) * speed, wantY = (s * pitch - c * roll + wantDy) * speed;  // roll is positive to the right
				float dvx = wantX - m_VelX, dvy = wantY - m_VelY, dv = MathF.Sqrt(dvx * dvx + dvy * dvy), step = m_LoitAcc * dt;
				if (dv > step) { dvx *= step / dv; dvy *= step / dv; }
				m_VelX += dvx; m_VelY += dvy;
				m_PosX += m_VelX * dt; m_PosY += m_VelY * dt;
				// Position -> velocity -> acceleration (plus the target's own), capped at what ANGLE_MAX can give.
				float posP = m_PosXyP, velP = m_VelXyP;
				float ax = velP * (m_VelX + posP * (m_PosX - x) - vx) + dvx / dt, ay = velP * (m_VelY + posP * (m_PosY - y) - vy) + dvy / dt;
				float a = MathF.Sqrt(ax * ax + ay * ay), aMax = G * MathF.Tan(maxAngle);
				if (a > aMax) { ax *= aMax / a; ay *= aMax / a; }
				// To the heading frame, then lean: nose down accelerates forward, a roll to the right accelerates right.
				pitchIn = MathF.Atan((c * ax + s * ay) / G);
				rollIn = -MathF.Atan((-s * ax + c * ay) / G);
			}

			// Lean targets, shaped by ATC_INPUT_TC; the yaw rate ramps at ATC_ACC_Y_MAX.
			float rollRate = (rollIn - m_RollT) / tc, pitchRate = (pitchIn - m_PitchT) / tc;
			m_RollT += rollRate * dt;
			m_PitchT += pitchRate * dt;
			float yawStep = m_AccYawMax * dt;
			m_YawRateT += Math.Clamp(turn * m_YawRateMax - m_YawRateT, -yawStep, yawStep);
			m_YawT += m_YawRateT * dt;

			// Angle P plus the targets' own rates (feedforward) -> Euler rates -> body rates.
			float dPhi = m_AngP[0] * (m_RollT - phi) + rollRate;
			float dTheta = m_AngP[1] * (m_PitchT - theta) + pitchRate;
			float dPsi = m_AngP[2] * MathF.IEEERemainder(m_YawT - psi, 2f * MathF.PI) + m_YawRateT;
			float sp = MathF.Sin(phi), cp = MathF.Cos(phi), st = MathF.Sin(theta), ct = MathF.Cos(theta);
			(m_RateT[0], m_RateT[1], m_RateT[2]) = (dPhi - st * dPsi, cp * dTheta + sp * ct * dPsi, -sp * dTheta + cp * ct * dPsi);

			// Rate PIDs, outputs in ArduPilot's normalised units.
			float dAlpha = dt / (dt + 1f / (2f * MathF.PI * m_RatFltd));
			for (int k = 0; k < 3; k++)
			{
				float err = m_RateT[k] - m_W[k];
				m_D[k] += ((err - m_DErr[k]) / dt - m_D[k]) * dAlpha;
				m_DErr[k] = err;
				m_I[k] = Math.Clamp(m_I[k] + m_RatI[k] * err * dt, -m_RatImax, m_RatImax);
				m_Out[k] = m_RatP[k] * err + m_I[k] + m_RatD[k] * m_D[k];
			}

			// Altitude: climb rate -> altitude target -> velocity -> acceleration -> throttle (hover feedforward, tilt boost).
			climb = MathF.Max(climb, -2f * (m_AltT - m_AltMin));  // set it down gently: descent slows over the last metre
			m_ClimbT += Math.Clamp(climb - m_ClimbT, -m_AccZ * dt, m_AccZ * dt);
			m_AltT += m_ClimbT * dt;
			if (m_AltT < m_AltMin)  // on the ground: holding the triggers down can't bury the target
			{
				m_AltT = m_AltMin;
				m_ClimbT = MathF.Max(m_ClimbT, 0f);
			}
			float acc = m_VelZP * (m_ClimbT + m_PosZP * (m_AltT - alt) - vz);
			m_Thr = Math.Clamp(Hover * (1f + acc / G) / MathF.Max(cp * ct, 0.5f), 0f, 1f);

			Mix(m_Out[0], m_Out[1], m_Out[2]);
		}

		// To the GCS, ArduPilot's frames: position and velocity NED (north = MuJoCo x, east = -y, down = -z), roll, pitch
		// and yaw FRD (the body's FLU Euler angles with pitch and yaw flipped). Down is from where it rests on the ground
		// (m_AltMin), as ArduPilot's altitude is from where it started, so landed reads 0, not the body's height.
		void SendState()
		{
			for (int k = 0; k < 3; k++)
			{
				float sign = k == 0 ? 1f : -1f;
				m_Pos[k] = sign * m_Q[m_Qpos + k];
				m_Vel[k] = sign * m_V[m_Qvel + k];
			}
			if (!float.IsNaN(m_AltMin)) m_Pos[2] += m_AltMin;
			m_Att[0] = MathF.Atan2(m_R[2, 1], m_R[2, 2]);
			m_Att[1] = MathF.Asin(Math.Clamp(m_R[2, 0], -1f, 1f));
			m_Att[2] = -MathF.Atan2(m_R[1, 0], m_R[0, 0]);
			m_Claw ??= FindEntityByTag("Claw")?.As<Claw>();  // here, not in OnCreate: its script may start later
			var balls = new float[m_BallBodies.Length][];
			for (int i = 0; i < balls.Length; i++)
			{
				m_Mj.GetBodyPoseMj(m_BallBodies[i], out Vector3 p, out _, out _, out _);
				balls[i] = new[] { p.X, -p.Y };  // MuJoCo x = north, -y = east
			}
			m_Mj.GetBodyPoseMj(m_BinBody, out Vector3 b, out _, out _, out _);
			m_Gcs.Send(Armed, Mode, Landed, m_Pos, m_Vel, m_Att, m_Claw?.Closed ?? false, balls, new[] { b.X, -b.Y }, m_PickUps);
		}

		// Quad X mix: thrust fraction per motor, roll and pitch first (throttle shifted to fit them), then yaw in whatever
		// room the motors have left, so a yaw never changes the total thrust; then back through the thrust curve to ESC
		// commands, as ArduPilot's motor library does.
		void Mix(float roll, float pitch, float yaw)
		{
			float[] rp = m_Rp;
			float lo = 0f, hi = 0f;
			for (int i = 0; i < 4; i++)
			{
				rp[i] = 0.5f * (MotorY[i] * roll - MotorX[i] * pitch);  // left motors lift a roll right, front motors drop for nose down
				lo = MathF.Min(lo, rp[i]); hi = MathF.Max(hi, rp[i]);
			}
			float rpScale = hi - lo > 1f ? 1f / (hi - lo) : 1f;
			float thr = Math.Clamp(m_Thr, -lo * rpScale, 1f - hi * rpScale);
			float yawMax = 1f;  // a motor that yaw speeds up must stay under 1, one it slows down above 0
			for (int i = 0; i < 4; i++)
			{
				rp[i] = thr + rp[i] * rpScale;
				yawMax = MathF.Min(yawMax, -Spin[i] * yaw > 0f ? 2f * (1f - rp[i]) : 2f * rp[i]);
			}
			yawMax = MathF.Max(yawMax, 0f);
			yaw = Math.Clamp(yaw, -yawMax, yawMax);
			float e = m_ThstExpo;
			for (int i = 0; i < 4; i++)
			{
				float t = Math.Clamp(rp[i] - 0.5f * Spin[i] * yaw, 0f, 1f);  // CW motors speed up to yaw left
				float u = (-(1f - e) + MathF.Sqrt((1f - e) * (1f - e) + 4f * e * t)) / (2f * e);  // inverse thrust curve
				m_Cmd[i] = m_SpinMin + (m_SpinMax - m_SpinMin) * u;
			}
		}

		// ---------------------------------------------------------------- pilot

		// The binding for `name`, while a controller is connected.
		bool Bound(string name, out (int Axis, float Sign, int Raw) b)
		{
			b = default;
			return m_Pad >= 0 && m_Bindings.TryGetValue(name, out b);
		}

		// An analog axis (-1..1) with the deadzone, 0 when unbound. X500View reads the camera's with it too.
		public float Axis(string name)
		{
			float v = Bound(name, out var b) && b.Axis >= 0 ? Input.GetControllerAxis(m_Pad, b.Axis) * b.Sign : 0f;
			return MathF.Abs(v) < Deadzone ? 0f : Math.Clamp(v, -1f, 1f);
		}

		// A button down, or an axis (a trigger) pushed past halfway.
		public bool Held(string name) => Bound(name, out var b)
			&& (b.Axis >= 0 ? Input.GetControllerAxis(m_Pad, b.Axis) * b.Sign > 0.5f : Input.IsControllerButtonDown(m_Pad, b.Raw));

		// Went down this frame (down now, up last frame). Call once per name per frame.
		public bool Pressed(string name)
		{
			bool down = Held(name), pressed = down && !m_ButtonsDown.Contains(name);
			if (down) m_ButtonsDown.Add(name);
			else m_ButtonsDown.Remove(name);
			return pressed;
		}

		// Props turn with their motor's thrust (a slow idle when armed on the ground, still when disarmed). Rotation is about
		// the prop's local +Y (up); in a right-handed Y-up frame a positive angle turns +Z toward +X, which seen from above
		// is counterclockwise, so a CCW motor (Spin +1) adds and a CW one (Spin -1) subtracts.
		void SpinProps(float dt)
		{
			for (int i = 0; i < 4; i++)
			{
				if (m_Props[i] is not Entity prop) continue;
				float w = Armed ? MathF.Max(PropSpinIdle, PropSpinMax * MathF.Sqrt(m_Thrust[i] / m_ThstMax)) : 0f;
				m_PropAngle[i] = MathF.IEEERemainder(m_PropAngle[i] + Spin[i] * w * dt, 2f * MathF.PI);
				prop.Rotation = new Vector3(0f, m_PropAngle[i], 0f);
			}
		}

		// The dot is shown while armed, straight below the drone's center: MuJoCo (x, y) on the ground is Lucky (x, -z).
		void CenterDot()
		{
			if (m_CenterDot is not Entity dot) return;
			dot.GetComponent<StaticMeshComponent>()!.Visible = Armed;
			dot.Translation = new Vector3(m_Q[m_Qpos], dot.Translation.Y, -m_Q[m_Qpos + 1]);
		}

		void Status(float dt)
		{
			if ((m_LogTimer += dt) < 1f) return;
			m_LogTimer = 0f;
			float tilt = MathF.Acos(Math.Clamp(m_R[2, 2], -1f, 1f)) * 180f / MathF.PI;
			Log.Info($"X500Flight: {ModeName}, {(Armed ? "armed" : "disarmed")} x={m_Q[m_Qpos]:F2} y={m_Q[m_Qpos + 1]:F2} " +
				$"v={MathF.Sqrt(m_V[m_Qvel] * m_V[m_Qvel] + m_V[m_Qvel + 1] * m_V[m_Qvel + 1]):F2} alt={m_Q[m_Qpos + 2]:F2} vz={m_V[m_Qvel + 2]:F2} " +
				$"tilt={tilt:F1} thr={m_Thr:F2} thrust=[{m_Thrust[0]:F1} {m_Thrust[1]:F1} {m_Thrust[2]:F1} {m_Thrust[3]:F1}] dt={dt * 1000f:F1}ms");
		}
	}
}
