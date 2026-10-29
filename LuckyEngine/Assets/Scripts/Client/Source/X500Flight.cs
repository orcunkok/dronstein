using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Flies the Holybro X500 V2 by hand, like ArduCopter's AltHold: the right stick leans it (up to ANGLE_MAX), the left stick
	/// turns it and climbs/descends, centred throttle holds the altitude. Arm with the left stick down and right for 2 s, disarm
	/// on the ground with down and left. V switches the controller and camera between the X500 and Dronstein.
	///
	/// Two halves, both set by X500.param in the project folder:
	/// - Motors(): the drone. Each motor's ESC command (0..1, as ArduPilot outputs it) becomes thrust through spin min/max,
	///   the thrust expo, max thrust and a time constant; with rotor yaw torque and drag they are applied to the X500's MuJoCo
	///   body. Tune SIM_* against the real drone's logs. Waits on Lucky for a working body force (docs/needs-AddBodyForceWorld.md).
	/// - Control(): a stand-in for ArduPilot (angle P -> rate PID -> quad-X mix, climb rate -> altitude), with the X500v2 frame
	///   file's gains. An ArduPilot connection later only has to replace it and feed Motors() its commands.
	/// Frames: MuJoCo world (z up); the body frame is FLU (x forward, y left, z up).
	/// Attach to the HolybroX500V2 entity.
	/// </summary>
	public class X500Flight : Entity
	{
		/// True while the X500 has the controller and the camera (DroneTeleop then leaves them alone).
		public static bool Piloted;

		const float G = 9.81f, Rho = 1.225f, ArmHold = 2f;

		MujocoSceneComponent m_Mj = null!;
		Entity m_Camera = null!;
		uint m_Qpos, m_Qvel;  // the free joint's places in qpos and qvel
		float[] m_Q = null!, m_V = null!;
		float m_Mass;
		bool m_Off;
		readonly Dictionary<string, float> P = new();
		int m_Pad = -1;
		float m_PadScan;
		bool m_KeyV;

		// Motors in FLU (x forward, y left), ArduPilot quad X order: 1 front-right CCW, 2 back-left CCW, 3 front-left CW, 4 back-right CW.
		static readonly float[] MotorX = { 1, -1, 1, -1 }, MotorY = { -1, 1, 1, -1 }, Spin = { 1, 1, -1, -1 };  // Spin +1 = CCW from above
		readonly float[] m_Cmd = new float[4], m_Thrust = new float[4];

		// Controller state
		bool m_Armed, m_Landed = true;
		float m_ArmTimer, m_LandTimer, m_LogTimer;
		double m_LastTime = -1;
		float m_RollT, m_PitchT, m_YawT, m_YawRateT;  // shaped targets (rad, rad/s)
		float m_AltT, m_ClimbT;                        // altitude target and its climb rate
		readonly float[] m_I = new float[3], m_DErr = new float[3], m_D = new float[3];
		float m_Thr;

		protected override void OnCreate()
		{
			LoadParams();
			Entity? scene = FindEntityByTag("Dronstein"), body = FindEntityByTag("x500v2"), cam = FindEntityByTag("Camera");
			if (scene == null || body == null || cam == null)
			{
				Log.Error("X500Flight: needs the entities tagged Dronstein (MuJoCo scene), x500v2 (the X500's body) and Camera; off");
				m_Off = true;
				return;
			}
			m_Mj = scene.GetComponent<MujocoSceneComponent>()!;
			m_Camera = cam;
			uint b = m_Mj.GetBodyID($"lucky_body_{body.ID}");
			uint joint = m_Mj.GetJointID($"lucky_joint_{body.ID}");  // Lucky names a body's free joint after its entity
			m_Qpos = m_Mj.GetJointQposAdr(joint);
			m_Qvel = m_Mj.GetJointQvelAdr(joint);
			m_Mass = m_Mj.GetBodyMass(b);
			m_Q = new float[m_Mj.ModelNq];
			m_V = new float[m_Mj.ModelNv];
			Log.Info($"X500Flight: start, {m_Mass:F3} kg, hover throttle {Hover:F3}, params {P.Count}");
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

		protected override void OnUpdate(float frameDt)
		{
			if (m_Off) return;
			if (Input.IsKeyDown(KeyCode.V) && !m_KeyV) Piloted = !Piloted;
			m_KeyV = Input.IsKeyDown(KeyCode.V);
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
				m_Armed = false;
				m_Landed = true;
				Array.Clear(m_Thrust);
				return;
			}
			m_LastTime = now;
			if (dt <= 0f) return;  // paused
			dt = MathF.Min(dt, 0.05f);

			m_Mj.GetFullQpos(m_Q);
			m_Mj.GetFullQvel(m_V);
			Control(dt);
			Motors(dt);
			if (Piloted) ChaseCamera(dt);
			Status(dt);
		}

		// ---------------------------------------------------------------- the drone

		// World <- FLU rotation of the body. qpos holds the body's quaternion; the body frame is Pod's (x forward, y up, z right).
		void Rotation(float[,] r)
		{
			float w = m_Q[m_Qpos + 3], x = m_Q[m_Qpos + 4], y = m_Q[m_Qpos + 5], z = m_Q[m_Qpos + 6];
			float[,] p = {
				{ 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y) },
				{ 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x) },
				{ 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y) } };
			for (int i = 0; i < 3; i++)
			{
				r[i, 0] = p[i, 0];   // forward = body x
				r[i, 1] = -p[i, 2];  // left = -body z
				r[i, 2] = p[i, 1];   // up = body y
			}
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
			Rotation(m_R);
			float vx = m_V[m_Qvel], vy = m_V[m_Qvel + 1], vz = m_V[m_Qvel + 2];
			float speed = MathF.Sqrt(vx * vx + vy * vy + vz * vz);
			float drag = 0.5f * Rho * P["SIM_DRAG_CDA"] * speed + P["SIM_DRAG_LIN"];
			// NEEDS LUCKY: qfrc_applied is overwritten every substep by Lucky's policy layer, so this does nothing yet.
			// Swap for MujocoSceneInstance::AddBodyForceWorld once it is exposed (see docs/needs-AddBodyForceWorld.md).
			for (int k = 0; k < 3; k++)
				m_Mj.SetQfrcApplied(m_Qvel + (uint)k, m_R[k, 2] * total - drag * m_V[m_Qvel + k]);
			// Free joint torques are in the body frame (Pod: x forward, y up, z right).
			m_Mj.SetQfrcApplied(m_Qvel + 3, tx);
			m_Mj.SetQfrcApplied(m_Qvel + 4, tz);
			m_Mj.SetQfrcApplied(m_Qvel + 5, -ty);
		}

		// ---------------------------------------------------------------- the stand-in autopilot

		void Control(float dt)
		{
			float thr = Stick("RC_THR"), yaw = Stick("RC_YAW"), pitch = Stick("RC_PIT"), roll = Stick("RC_RLL");
			Rotation(m_R);
			float phi = MathF.Atan2(m_R[2, 1], m_R[2, 2]), theta = -MathF.Asin(Math.Clamp(m_R[2, 0], -1f, 1f));
			float psi = MathF.Atan2(m_R[1, 0], m_R[0, 0]);
			float alt = m_Q[m_Qpos + 2], vz = m_V[m_Qvel + 2];
			// Body rates, FLU, from the body-frame angular velocity (Pod axes).
			float[] w = { m_V[m_Qvel + 3], -m_V[m_Qvel + 5], m_V[m_Qvel + 4] };

			// Arming: left stick down and right (arm) or left (disarm, on the ground) for 2 s, like ArduPilot's rudder arming.
			bool armGesture = thr < -0.9f && yaw > 0.9f && !m_Armed, disarmGesture = thr < -0.9f && yaw < -0.9f && m_Armed && m_Landed;
			m_ArmTimer = armGesture || disarmGesture ? m_ArmTimer + dt : 0f;
			if (m_ArmTimer > ArmHold)
			{
				m_Armed = !m_Armed;
				m_ArmTimer = 0f;
				m_Landed = true;
				Log.Info($"X500Flight: {(m_Armed ? "armed" : "disarmed")}");
			}

			// Pilot climb rate with a deadband round mid stick, ramped at PILOT_ACC_Z.
			float dz = P["THR_DZ"], climb = 0f;
			if (thr > dz) climb = (thr - dz) / (1f - dz) * P["PILOT_SPD_UP"];
			else if (thr < -dz) climb = (thr + dz) / (1f - dz) * P["PILOT_SPD_DN"];

			if (!m_Armed || m_Landed)
			{
				// On the ground: motors idle (or stopped), targets follow the drone, integrators cleared.
				float idle = m_Armed ? P["MOT_SPIN_ARM"] : 0f;
				for (int i = 0; i < 4; i++) m_Cmd[i] = idle;
				m_RollT = m_PitchT = m_YawRateT = m_ClimbT = 0f;
				m_YawT = psi;
				m_AltT = alt;
				Array.Clear(m_I);
				Array.Clear(m_D);
				m_Thr = 0f;
				if (m_Armed && climb > 0f) m_Landed = false;  // takeoff
				return;
			}

			// Lean targets, shaped by ATC_INPUT_TC; the yaw rate ramps at ATC_ACC_Y_MAX.
			float maxAngle = P["ANGLE_MAX"] * MathF.PI / 180f, tc = P["ATC_INPUT_TC"];
			float rollRate = (roll * maxAngle - m_RollT) / tc, pitchRate = (pitch * maxAngle - m_PitchT) / tc;
			m_RollT += rollRate * dt;
			m_PitchT += pitchRate * dt;
			float yawStep = P["ATC_ACC_Y_MAX"] * MathF.PI / 180f * dt;
			m_YawRateT += Math.Clamp(-yaw * P["PILOT_Y_RATE"] * MathF.PI / 180f - m_YawRateT, -yawStep, yawStep);
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
			m_ClimbT += Math.Clamp(climb - m_ClimbT, -P["PILOT_ACC_Z"] * dt, P["PILOT_ACC_Z"] * dt);
			m_AltT += m_ClimbT * dt;
			float acc = P["PSC_D_VEL_P"] * (m_ClimbT + P["PSC_D_POS_P"] * (m_AltT - alt) - vz);
			m_Thr = Math.Clamp(Hover * (1f + acc / G) / MathF.Max(cp * ct, 0.5f), 0f, 1f);

			// Landed: pushing down, the throttle is at its floor and nothing moves for a second.
			m_LandTimer = climb < 0f && m_Thr < 0.5f * Hover && MathF.Abs(vz) < 0.2f ? m_LandTimer + dt : 0f;
			if (m_LandTimer > 1f) { m_Landed = true; m_LandTimer = 0f; Log.Info("X500Flight: landed"); }

			Mix(o[0], o[1], o[2]);
		}

		// Quad X mix: thrust fraction per motor (roll and pitch first, then yaw, then throttle shifted to fit), then
		// back through the thrust curve to ESC commands, as ArduPilot's motor library does.
		void Mix(float roll, float pitch, float yaw)
		{
			float[] rp = new float[4], y = new float[4];
			float lo = 0f, hi = 0f;
			for (int i = 0; i < 4; i++)
			{
				rp[i] = 0.5f * (MotorY[i] * roll - MotorX[i] * pitch);  // left motors lift a roll right, front motors drop for nose down
				y[i] = -0.5f * Spin[i] * yaw;                             // CW motors speed up to yaw left
				lo = MathF.Min(lo, rp[i]); hi = MathF.Max(hi, rp[i]);
			}
			float rpScale = hi - lo > 1f ? 1f / (hi - lo) : 1f;
			float room = MathF.Max(0f, 1f - (hi - lo) * rpScale);       // what roll and pitch leave for yaw
			float yawScale = MathF.Abs(yaw) > room ? room / MathF.Abs(yaw) : 1f;  // yaw spreads the motors by |yaw|
			lo = 1f; hi = -1f;
			for (int i = 0; i < 4; i++)
			{
				rp[i] = rp[i] * rpScale + y[i] * yawScale;
				lo = MathF.Min(lo, rp[i]); hi = MathF.Max(hi, rp[i]);
			}
			float thr = Math.Clamp(m_Thr, -lo, 1f - hi);
			float e = P["SIM_THST_EXPO"], spinMin = P["MOT_SPIN_MIN"], spinMax = P["MOT_SPIN_MAX"];
			for (int i = 0; i < 4; i++)
			{
				float t = Math.Clamp(thr + rp[i], 0f, 1f);
				float u = (-(1f - e) + MathF.Sqrt((1f - e) * (1f - e) + 4f * e * t)) / (2f * e);  // inverse thrust curve
				m_Cmd[i] = spinMin + (spinMax - spinMin) * u;
			}
		}

		// ---------------------------------------------------------------- pilot

		// A stick (-1..1, up and right positive) from X500.param's RC_* axis, 0 when V hasn't given the X500 the controller.
		float Stick(string name)
		{
			if (!Piloted || m_Pad < 0) return 0f;
			float a = P[name];  // the axis number, negative (including -0) to flip it
			float v = Input.GetControllerAxis(m_Pad, (int)MathF.Abs(a)) * (float.IsNegative(a) ? -1f : 1f);
			return MathF.Abs(v) < P["RC_DZ"] ? 0f : Math.Clamp(v, -1f, 1f);
		}

		// Behind and above the X500, looking along its heading.
		void ChaseCamera(float dt)
		{
			var at = new Vector3(m_Q[m_Qpos], m_Q[m_Qpos + 2], -m_Q[m_Qpos + 1]);  // MuJoCo (x, y, z) is Lucky (x, -z, y)
			float psi = MathF.Atan2(m_R[1, 0], m_R[0, 0]);
			// MuJoCo heading psi -> Lucky: forward (cos, sin) in MuJoCo (x, y) is (cos, -sin) in Lucky (x, z).
			var back = new Vector3(-MathF.Cos(psi), 0f, MathF.Sin(psi));
			Vector3 want = at + back * 2.2f + new Vector3(0f, 0.9f, 0f);
			m_Camera.Translation += (want - m_Camera.Translation) * MathF.Min(1f, 4f * dt);
			Vector3 d = m_Camera.Translation - at;
			float dist = d.Length();
			m_Camera.Rotation = new Vector3(-MathF.Asin(d.Y / dist), MathF.Atan2(d.X, d.Z), 0f);
		}

		void Status(float dt)
		{
			if ((m_LogTimer += dt) < 1f) return;
			m_LogTimer = 0f;
			float tilt = MathF.Acos(Math.Clamp(m_R[2, 2], -1f, 1f)) * 180f / MathF.PI;
			Log.Info($"X500Flight: {(m_Armed ? m_Landed ? "armed, landed" : "flying" : "disarmed")} alt={m_Q[m_Qpos + 2]:F2} vz={m_V[m_Qvel + 2]:F2} " +
				$"tilt={tilt:F1} thr={m_Thr:F2} thrust=[{m_Thrust[0]:F1} {m_Thrust[1]:F1} {m_Thrust[2]:F1} {m_Thrust[3]:F1}] dt={dt * 1000f:F1}ms");
		}
	}
}
