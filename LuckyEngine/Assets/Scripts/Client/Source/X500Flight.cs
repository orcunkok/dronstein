using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Flies the Holybro X500 V2 with the controller (ControllerBindings.txt): LT climbs and RT descends, the bumpers turn,
	/// and the left stick moves it, in one of two modes (x500_mode, B, switches):
	/// - Loiter (default, ArduPilot's Loiter): the stick sets a speed (up to LOIT_SPEED); let go and it brakes and holds its spot.
	/// - Manual (ArduPilot's AltHold): the stick sets the lean (up to ANGLE_MAX); let go and it levels but coasts.
	/// Both hold the altitude. It starts disarmed: x500_arm (Y) arms and disarms it on the ground. x500_takeoff_rtl (Share)
	/// takes off to TKOFF_ALT on the ground, and in the air returns home (where it was armed): up to RTL_ALT, over, down at
	/// LAND_SPEED, then it disarms. Share again, B or any stick or trigger takes RTL back.
	/// The camera, the HUD and R (reset) are X500View's.
	///
	/// Two halves, both set by X500.param in the project folder:
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
		/// Loiter (true) or Manual; B switches it.
		public static bool Loiter = true;
		/// Armed (motors spin, it can fly) or disarmed; Y switches it on the ground. The HUD shows it.
		public static bool Armed;
		/// What MODE shows on the HUD: Takeoff or RTL while those run, Loiter or Manual otherwise.
		public static string ModeName => s_Rtl ? "RTL" : s_Takeoff ? "Takeoff" : Loiter ? "Loiter" : "Manual";
		static bool s_Rtl, s_Takeoff;  // Share's takeoff and return home, until done or taken back

		const float G = 9.81f, Rho = 1.225f;

		MujocoSceneComponent m_Mj = null!;
		uint m_Qpos, m_Qvel;  // the free joint's places in qpos and qvel
		// The motors on the free joint (x500v2.mjcf): force along world x/y/z, torque about body x/y/z.
		static readonly string[] WrenchNames = { "x500v2_fx", "x500v2_fy", "x500v2_fz", "x500v2_tx", "x500v2_ty", "x500v2_tz" };
		readonly MujocoActuator[] m_Wrench = new MujocoActuator[6];
		float[] m_Q = null!, m_V = null!;
		float m_Mass;
		bool m_Off;
		readonly Dictionary<string, float> P = new();
		int m_Pad = -1;
		float m_PadScan;
		// ControllerBindings.txt: an axis (Axis >= 0, Sign flips it) or a raw button bN.
		readonly Dictionary<string, (int Axis, float Sign, int Raw)> m_Bindings = new();
		float m_Deadzone = 0.1f;

		// Motors in FLU (x forward, y left), ArduPilot quad X order: 1 front-right CCW, 2 back-left CCW, 3 front-left CW, 4 back-right CW.
		static readonly float[] MotorX = { 1, -1, 1, -1 }, MotorY = { -1, 1, 1, -1 }, Spin = { 1, 1, -1, -1 };  // Spin +1 = CCW from above
		readonly float[] m_Cmd = new float[4], m_Thrust = new float[4];

		// Controller state
		float m_LogTimer;
		double m_LastTime = -1;
		float m_RollT, m_PitchT, m_YawT, m_YawRateT;  // shaped targets (rad, rad/s)
		float m_AltT, m_ClimbT;                        // altitude target and its climb rate
		float m_AltMin = float.NaN;                    // resting height: the altitude target never goes below it
		float m_PosX, m_PosY, m_VelX, m_VelY;          // Loiter's target position and velocity (world x/y)
		readonly float[] m_I = new float[3], m_DErr = new float[3], m_D = new float[3];
		float m_Thr;
		float m_HomeX, m_HomeY;                        // where it was armed: RTL flies back here
		readonly HashSet<string> m_ButtonsDown = new();  // buttons down last frame, for Pressed

		// Props: entities tagged X500Prop1..4 (children of this entity, motor order), spun about their local +Y.
		readonly Entity?[] m_Props = new Entity?[4];
		readonly float[] m_PropAngle = new float[4];
		const float PropSpinMax = 60f, PropSpinIdle = 10f;  // rad/s on screen at full thrust and at idle (real rpm would alias)

		protected override void OnCreate()
		{
			LoadParams();
			LoadBindings();
			Loiter = true;
			Armed = s_Rtl = s_Takeoff = false;
			for (int i = 0; i < 4; i++) m_Props[i] = FindEntityByTag($"X500Prop{i + 1}");  // optional: no tag, no spin
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
			for (int k = 0; k < 6; k++)
			{
				m_Wrench[k] = m_Mj.GetActuator(WrenchNames[k])!;
				m_Wrench[k].IK = false;
			}
			StopMotors();
			m_Q = new float[m_Mj.ModelNq];
			m_V = new float[m_Mj.ModelNv];
			Log.Info($"X500Flight: start, {m_Mass:F3} kg, hover throttle {Hover:F3}, params {P.Count}");
		}

		// No thrust, and ctrl zeroed: until it is written, Lucky's initial ctrl (the joint's own qpos) would push the X500.
		void StopMotors()
		{
			Array.Clear(m_Thrust);
			foreach (MujocoActuator a in m_Wrench) a.Value = 0;
		}

		float Hover => m_Mass * G / (4f * P["SIM_THST_MAX"]);

		void LoadParams()
		{
			string path = Path.GetFullPath(Path.Combine(Application.DataDirectoryPath!, "..", "X500.param"));
			foreach (string raw in File.ReadAllLines(path))
			{
				string[] f = raw.Split('#')[0].Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
				if (f.Length == 2 && float.TryParse(f[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float v)) P[f[0]] = v;
			}
		}

		// The names it reads: forward_back, left_right, up, down, turn_left, turn_right, x500_*, camera_*, zoom_*, deadzone.
		void LoadBindings()
		{
			string path = Path.GetFullPath(Path.Combine(Application.DataDirectoryPath!, "..", "ControllerBindings.txt"));
			if (!File.Exists(path)) return;
			foreach (string raw in File.ReadAllLines(path))
			{
				string[] f = raw.Split('#')[0].Split('=');
				if (f.Length != 2) continue;
				string name = f[0].Trim(), value = f[1].Trim();
				if (name == "deadzone") float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out m_Deadzone);
				else if (int.TryParse(value.TrimStart('-'), out int axis)) m_Bindings[name] = (axis, value.StartsWith('-') ? -1f : 1f, -1);
				else if (value.StartsWith('b') && int.TryParse(value[1..], out int button)) m_Bindings[name] = (-1, 1f, button);
			}
		}

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
			SpinProps(dt);
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

		readonly float[,] m_R = new float[3, 3];

		// ESC commands -> thrust (with the motors' lag) -> force and torque on the body, plus drag.
		void Motors(float dt)
		{
			float spinMin = P["MOT_SPIN_MIN"], spinMax = P["MOT_SPIN_MAX"], e = P["SIM_THST_EXPO"], tmax = P["SIM_THST_MAX"];
			float lag = 1f - MathF.Exp(-dt / P["SIM_MOT_TAU"]);
			float arm = P["SIM_ARM_LEN"] * 0.70710678f, kq = P["SIM_YAW_COEF"];
			float total = 0f, tx = 0f, ty = 0f, tz = 0f;
			for (int i = 0; i < 4; i++)
			{
				float u = Math.Clamp((m_Cmd[i] - spinMin) / (spinMax - spinMin), 0f, 1f);
				m_Thrust[i] += (tmax * ((1f - e) * u + e * u * u) - m_Thrust[i]) * lag;
				float t = m_Thrust[i];
				total += t;
				tx += MotorY[i] * arm * t;      // r x F with F along +z: (y F, -x F, 0)
				ty -= MotorX[i] * arm * t;
				tz -= Spin[i] * kq * t;         // a CCW prop twists the body CW
			}
			BodyRotation(m_R);
			float vx = m_V[m_Qvel], vy = m_V[m_Qvel + 1], vz = m_V[m_Qvel + 2];
			float speed = MathF.Sqrt(vx * vx + vy * vy + vz * vz);
			float drag = 0.5f * Rho * P["SIM_DRAG_CDA"] * speed + P["SIM_DRAG_LIN"];
			for (int k = 0; k < 3; k++)
				m_Wrench[k].Value = m_R[k, 2] * total - drag * m_V[m_Qvel + k];
			m_Wrench[3].Value = tx;  // free joint torques are in the body frame (FLU)
			m_Wrench[4].Value = ty;
			m_Wrench[5].Value = tz;
		}

		// ---------------------------------------------------------------- the stand-in autopilot

		void Control(float dt)
		{
			float pitch = Axis("forward_back"), roll = -Axis("left_right");  // left_right is positive to the left
			float climb = (Held("up") ? P["PILOT_SPD_UP"] : 0f) - (Held("down") ? P["PILOT_SPD_DN"] : 0f);
			float turn = (Held("turn_left") ? 1f : 0f) - (Held("turn_right") ? 1f : 0f);
			BodyRotation(m_R);
			float phi = MathF.Atan2(m_R[2, 1], m_R[2, 2]), theta = -MathF.Asin(Math.Clamp(m_R[2, 0], -1f, 1f));
			float psi = MathF.Atan2(m_R[1, 0], m_R[0, 0]);
			float x = m_Q[m_Qpos], y = m_Q[m_Qpos + 1], alt = m_Q[m_Qpos + 2];
			float vx = m_V[m_Qvel], vy = m_V[m_Qvel + 1], vz = m_V[m_Qvel + 2];
			float[] w = { m_V[m_Qvel + 3], m_V[m_Qvel + 4], m_V[m_Qvel + 5] };  // body rates (FLU)

			// The target only creeps to the floor (the descent slows near it), hence the 1 cm.
			bool grounded = float.IsNaN(m_AltMin) || (m_AltT <= m_AltMin + 0.01f && alt < m_AltMin + 0.05f);
			if (Pressed("x500_arm") && grounded)  // Y: arm / disarm, on the ground only
			{
				Armed = !Armed;
				(m_HomeX, m_HomeY) = (x, y);
				Log.Info($"X500Flight: {(Armed ? "armed" : "disarmed")}");
			}
			if (Pressed("x500_mode"))  // B: Loiter <-> Manual, and takes RTL / takeoff back
			{
				Loiter = !Loiter;
				s_Rtl = s_Takeoff = false;
				(m_PosX, m_PosY, m_VelX, m_VelY) = (x, y, vx, vy);  // Loiter starts from where it is, at the speed it has
			}
			if (Pressed("x500_takeoff_rtl") && Armed)  // Share: takeoff on the ground, return home (or take it back) in the air
			{
				if (grounded) s_Takeoff = true;
				else
				{
					s_Rtl = !s_Rtl;
					s_Takeoff = false;
					(m_PosX, m_PosY, m_VelX, m_VelY) = (x, y, vx, vy);
				}
				Log.Info($"X500Flight: {ModeName}");
			}
			if (pitch != 0f || roll != 0f || climb != 0f || turn != 0f) s_Rtl = s_Takeoff = false;  // the pilot takes over
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
				for (int i = 0; i < 4; i++) m_Cmd[i] = Armed ? P["MOT_SPIN_MIN"] : 0f;  // armed: idle (no thrust yet)
				return;
			}

			// Takeoff: up to TKOFF_ALT above the ground, holding its spot. RTL, from where its target is: up to RTL_ALT
			// (unless higher), over home at RTL_SPEED braking at LOIT_ACC_MAX, then down at LAND_SPEED (it disarms on landing).
			float wantSpeed = P["LOIT_SPEED"], wantDx = 0f, wantDy = 0f;  // auto: the speed to fly and its direction (world)
			if (s_Takeoff)
			{
				float top = m_AltMin + P["TKOFF_ALT"];
				climb = Math.Clamp(2f * (top - m_AltT), 0f, P["PILOT_SPD_UP"]);
				if (m_AltT >= top - 0.01f) s_Takeoff = false;
			}
			else if (s_Rtl)
			{
				float hx = m_HomeX - m_PosX, hy = m_HomeY - m_PosY, dist = MathF.Sqrt(hx * hx + hy * hy), top = m_AltMin + P["RTL_ALT"];
				if (dist < 0.05f) { (m_PosX, m_PosY) = (m_HomeX, m_HomeY); climb = -P["LAND_SPEED"]; }  // over home: land
				else if (alt < top - 0.3f) climb = Math.Clamp(2f * (top - m_AltT), 0f, P["PILOT_SPD_UP"]);  // too low: climb first
				else
				{
					wantSpeed = MathF.Min(P["RTL_SPEED"], MathF.Sqrt(2f * P["LOIT_ACC_MAX"] * dist));  // fast enough to stop at home
					(wantDx, wantDy) = (hx / dist, hy / dist);
				}
			}
			bool auto = s_Takeoff || s_Rtl;

			float maxAngle = P["ANGLE_MAX"] * MathF.PI / 180f, tc = P["ATC_INPUT_TC"];
			float rollIn = roll * maxAngle, pitchIn = pitch * maxAngle;  // Manual: the stick is the lean
			if (Loiter || auto)
			{
				// The stick is a velocity in the heading frame; the target velocity moves toward it at LOIT_ACC_MAX, so
				// letting go brakes at that rate. The target position follows it, and holds when it stops.
				float c = MathF.Cos(psi), s = MathF.Sin(psi), speed = wantSpeed;
				float wantX = (c * pitch + s * roll + wantDx) * speed, wantY = (s * pitch - c * roll + wantDy) * speed;  // roll is positive to the right
				float dvx = wantX - m_VelX, dvy = wantY - m_VelY, dv = MathF.Sqrt(dvx * dvx + dvy * dvy), step = P["LOIT_ACC_MAX"] * dt;
				if (dv > step) { dvx *= step / dv; dvy *= step / dv; }
				m_VelX += dvx; m_VelY += dvy;
				m_PosX += m_VelX * dt; m_PosY += m_VelY * dt;
				// Position -> velocity -> acceleration (plus the target's own), capped at what ANGLE_MAX can give.
				float posP = P["PSC_NE_POS_P"], velP = P["PSC_NE_VEL_P"];
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
			float yawStep = P["ATC_ACC_Y_MAX"] * MathF.PI / 180f * dt;
			m_YawRateT += Math.Clamp(turn * P["PILOT_Y_RATE"] * MathF.PI / 180f - m_YawRateT, -yawStep, yawStep);
			m_YawT += m_YawRateT * dt;

			// Angle P plus the targets' own rates (feedforward) -> Euler rates -> body rates.
			float dPhi = P["ATC_ANG_RLL_P"] * (m_RollT - phi) + rollRate;
			float dTheta = P["ATC_ANG_PIT_P"] * (m_PitchT - theta) + pitchRate;
			float dPsi = P["ATC_ANG_YAW_P"] * MathF.IEEERemainder(m_YawT - psi, 2f * MathF.PI) + m_YawRateT;
			float sp = MathF.Sin(phi), cp = MathF.Cos(phi), st = MathF.Sin(theta), ct = MathF.Cos(theta);
			float[] rateT = { dPhi - st * dPsi, cp * dTheta + sp * ct * dPsi, -sp * dTheta + cp * ct * dPsi };

			// Rate PIDs, outputs in ArduPilot's normalised units.
			string[] axis = { "RLL", "PIT", "YAW" };
			float[] o = new float[3];
			float dAlpha = dt / (dt + 1f / (2f * MathF.PI * P["ATC_RAT_FLTD"])), imax = P["ATC_RAT_IMAX"];
			for (int k = 0; k < 3; k++)
			{
				float err = rateT[k] - w[k];
				m_D[k] += ((err - m_DErr[k]) / dt - m_D[k]) * dAlpha;
				m_DErr[k] = err;
				m_I[k] = Math.Clamp(m_I[k] + P[$"ATC_RAT_{axis[k]}_I"] * err * dt, -imax, imax);
				o[k] = P[$"ATC_RAT_{axis[k]}_P"] * err + m_I[k] + P[$"ATC_RAT_{axis[k]}_D"] * m_D[k];
			}

			// Altitude: climb rate -> altitude target -> velocity -> acceleration -> throttle (hover feedforward, tilt boost).
			climb = MathF.Max(climb, -2f * (m_AltT - m_AltMin));  // set it down gently: descent slows over the last metre
			m_ClimbT +=Math.Clamp(climb - m_ClimbT, -P["PILOT_ACC_Z"] * dt, P["PILOT_ACC_Z"] * dt);
			m_AltT += m_ClimbT * dt;
			if (m_AltT < m_AltMin)  // on the ground: holding the triggers down can't bury the target
			{
				m_AltT = m_AltMin;
				m_ClimbT = MathF.Max(m_ClimbT, 0f);
			}
			float acc = P["PSC_D_VEL_P"] * (m_ClimbT + P["PSC_D_POS_P"] * (m_AltT - alt) - vz);
			m_Thr = Math.Clamp(Hover * (1f + acc / G) / MathF.Max(cp * ct, 0.5f), 0f, 1f);

			Mix(o[0], o[1], o[2]);
		}

		// Quad X mix: thrust fraction per motor, roll and pitch first (throttle shifted to fit them), then yaw in whatever
		// room the motors have left, so a yaw never changes the total thrust; then back through the thrust curve to ESC
		// commands, as ArduPilot's motor library does.
		void Mix(float roll, float pitch, float yaw)
		{
			float[] rp = new float[4];
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
			float e = P["SIM_THST_EXPO"], spinMin = P["MOT_SPIN_MIN"], spinMax = P["MOT_SPIN_MAX"];
			for (int i = 0; i < 4; i++)
			{
				float t = Math.Clamp(rp[i] - 0.5f * Spin[i] * yaw, 0f, 1f);  // CW motors speed up to yaw left
				float u = (-(1f - e) + MathF.Sqrt((1f - e) * (1f - e) + 4f * e * t)) / (2f * e);  // inverse thrust curve
				m_Cmd[i] = spinMin + (spinMax - spinMin) * u;
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
			return MathF.Abs(v) < m_Deadzone ? 0f : Math.Clamp(v, -1f, 1f);
		}

		// A button down, or an axis (a trigger) pushed past halfway.
		public bool Held(string name) => Bound(name, out var b)
			&& (b.Axis >= 0 ? Input.GetControllerAxis(m_Pad, b.Axis) * b.Sign > 0.5f : Input.IsControllerButtonDown(m_Pad, b.Raw));

		// Went down this frame (down now, up last frame). Call once per name per frame.
		bool Pressed(string name)
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
				float w = Armed ? MathF.Max(PropSpinIdle, PropSpinMax * MathF.Sqrt(m_Thrust[i] / P["SIM_THST_MAX"])) : 0f;
				m_PropAngle[i] = MathF.IEEERemainder(m_PropAngle[i] + Spin[i] * w * dt, 2f * MathF.PI);
				prop.Rotation = new Vector3(0f, m_PropAngle[i], 0f);
			}
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
