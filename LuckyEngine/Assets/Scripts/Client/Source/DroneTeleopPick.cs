using System;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Autonomous pick of one golf ball and drop into the TrashCan (Tuning's Pick selected ball mode, or `pick N [a|b]` in the command file).
	/// The design and the numbers behind it are in .claude/nightrun/sim/PICK_DESIGN.md (tried offline in Python MuJoCo first).
	///
	/// The drone hovers 0.24 m behind the ball (nose pointing from the ball toward the bin), holds still, and the arm reaches the ball:
	/// every frame the joint targets come from an analytic IK of the ball's position in the *measured* drone frame, so the small hover
	/// errors are absorbed by the arm. The drone alone provides the vertical travel (the arm keeps its fingers vertical).
	/// Phases: Transit (one straight flight to the hover point behind the ball, the arm unfolding and the gripper opening near its end),
	/// Grab (the arm aligns on the ball, the drone descends a little to the pick height, force-limited close), Lift, Transit (carries the
	/// ball to above the bin rim), Drop. Grab checks the grasp itself (Grasped); no grasp means open and retry.
	/// </summary>
	public partial class DroneTeleop
	{
		enum PickPhase { Off, Transit, Grab, Retry, Lift, HoldB, Drop }

		// Design constants (PICK_DESIGN.md). Jaw-frame offsets and the gripper numbers were measured again in Lucky.
		const float HPick = 0.42f;          // m, drone origin height while the fingertips are at ball level
		const float BallD = 0.0428f;        // m, golf ball diameter
		const float HHov = HPick + 2f * BallD;  // m, drone origin height at the hover point: the lowest fingertip 2 ball heights above the ground
		const float Standoff = 0.24f;       // m, drone origin to ball along the nose
		const float DescentV = 0.04f, DescentA = 0.3f;  // m/s, m/s^2: Grab's descent from HHov to HPick
		const float GrabPress = 0.005f;     // m, Grab commands this far below HPick: the fixed jaw's tip is on the ground, so the drone presses on it and doesn't lift in the close
		const float PickRoll0 = 1.5708f;    // wrist_roll the IK constants below were derived at: jaws open sideways
		float PickRoll = PickRoll0;         // wrist_roll for the pick (dev: `gp roll`)
		float GripOpen = 0.90f;             // dev: `gp open`
		const float GripClosed = -0.174f;
		const float PickArmSpeed = 0.8f, GripOpenSpeed = 1.5f;  // rad/s
		const float Rim = 0.46f, SafeLow = Rim + 0.10f + 0.03f; // bin rim; lowest point near the bin must stay above SafeLow (3 cm margin)
		const float ApproachR = 1.0f;       // m, the arm unfolds this close to the hover point (~3 s from folded at PickArmSpeed)
		const float StowedLow = 0.133f;     // m, lowest point of the stowed arm below the drone origin
		float SqueezeLead = 0.02f;          // rad, close command leads the measured jaw angle: torque = kp * lead = 1 Nm (dev: `gp lead`). More seats the ball out on the pads: it slips
		const float FastLead = 0.1f;        // rad: the close leads by this (full torque) while the jaw is above GripSlowAt
		float GripSlowAt = 0.65f;           // rad, ~0.2 rad before the ball; 0 = no fast part (dev: `gp fast`). Closer fails: the jaw hits the ball too fast
		int StallFrames = 15;               // frames the jaw must be still to end the close (dev: `gp stall`)
		float m_PickStopG, m_PickStopT;     // jaw angle and time (since the close began) when the jaw last came to a stop in Grab
		const float BinFloorR = 0.165f;
		// Ball centre in the Fixed_Jaw frame (x, y, z) and its height above the ball centre.
		static readonly Vector3 Tool0 = new(-0.030f, -0.0857f, -0.006f);   // the offsets the IK constants below were derived with
		Vector3 m_Tool = new(-0.030f, -0.0857f, -0.006f);
		float m_ToolDz = 0.003f;
		// IK constants in the drone body frame (PICK_DESIGN.md): pan axis point, lift pivot, links, wrist-to-tool at Tool0.
		static readonly Vector3 IkC = new(-0.1102f, 0f, 0.0235f), IkS = new(-0.1408f, 0f, -0.0790f);
		static readonly Vector3 IkE = new(-0.11257f, 0f, 0.0280f), IkW = new(-0.13490f, 0f, -0.0052f);  // E0 - S, W0 - E0
		static readonly Vector3 IkWoff0 = new(0.0060f, 0.0300f, -0.1458f);

		PickPhase m_PickPhase;
		int m_PickN, m_PickRetry;
		char m_PickStop;                    // 'a' or 'b': stop after that sub-milestone, 'c' = the whole pick
		float m_PickT, m_PickClock, m_PickSub;  // s in this phase, s since the pick began, a phase-internal timer (settling)
		bool m_PickSettling;               // Grab: the close has begun. Retry: the gripper is open, flying back to the hover point
		uint m_PickBallBody, m_PickBinBody, m_Jaw, m_GripJoint;
		Vector3 m_PickBall0, m_PickBin0, m_PickHov, m_PickCarryOffset, m_PickGoal;
		float m_PickYaw, m_PickTilt, m_PickMaxBallSite, m_PickMinZ;
		int m_PickStall, m_PickReplans;
		bool m_PickGrasped;                // the ball is in the gripper (set by Grab): Transit carries it to the bin
		Vector3 m_PickBallPrev;
		float m_PickHc;
		int m_GripQ, m_GripV;               // the gripper joint's places in qpos and qvel
		float[] m_PickQ = new float[4];
		float[,] m_Rj = new float[3, 3];
		Quaternion m_PickBinQ0;

		// Once, from OnCreate: the jaw body and the gripper joint's place in qpos/qvel.
		void PickInit()
		{
			m_Jaw = m_Mj.GetBodyID("Fixed_Jaw");
			m_GripJoint = m_Mj.GetJointID("gripper");
			m_GripQ = (int)m_Mj.GetJointQposAdr(m_GripJoint);
			m_GripV = (int)m_Mj.GetJointQvelAdr(m_GripJoint);
		}

		void PickStart(int n, char stop)
		{
			Entity? ball = FindEntityByTag($"GolfBall_{n:00}"), bin = FindEntityByTag("TrashCan");
			if (ball == null || bin == null) { Log.Info($"METRIC pick ball={n} result=fail reason=no_ball_or_bin"); return; }
			m_PickBallBody = m_Mj.GetBodyID($"lucky_body_{ball.ID}");
			m_PickBinBody = m_Mj.GetBodyID($"lucky_body_{bin.ID}");
			m_Tuning.SpeedMode = 0;  // slow
			m_Tuning.Ball = n;
			m_PickN = n; m_PickStop = stop; m_PickRetry = 0; m_PickClock = 0f; m_PickReplans = 0; m_PickGrasped = false; m_PickDone = false;
			m_PickBall0 = BodyPos(m_PickBallBody);
			m_PickBin0 = BodyPos(m_PickBinBody);
			m_Mj.GetBodyPoseMj(m_PickBinBody, out Vector3 _, out m_PickBinQ0, out Vector3 _, out Vector3 _);
			m_PickTilt = 0f; m_PickMaxBallSite = 0f; m_PickMinZ = 9f;
			NavBegin();
			m_Waypoint = -1; m_ToBall = 0; m_ArmGoal = null;
			m_PickBallPrev = m_PickBall0;
			PickIkSelfTest();
			PickPlanTransit();
		}

		// Heading from the ball toward the bin and the hover point behind the ball at HHov: one straight flight there (KeepOut still splits it near the bin).
		void PickPlanTransit()
		{
			Vector3 ball = m_PickBall0, bin = m_PickBin0;
			m_PickYaw = MathF.Atan2(bin.Y - ball.Y, bin.X - ball.X);
			m_PickHov = new Vector3(ball.X - Standoff * MathF.Cos(m_PickYaw), ball.Y - Standoff * MathF.Sin(m_PickYaw), HHov);
			PickEnter(PickPhase.Transit, $"yaw={m_PickYaw * 57.2958f:F0} hov=({m_PickHov.X:F3},{m_PickHov.Y:F3})");
			FlyTo(m_PickHov, m_PickYaw);
		}

		// HUD timers: the last whole pick, and the previous step.
		float m_PickLast = -1f, m_StepLast = -1f;

		void PickEnter(PickPhase p, string extra = "")
		{
			if (m_PickPhase != PickPhase.Off) m_StepLast = m_PickT;
			m_PickPhase = p; m_PickT = 0f; m_PickSub = 0f; m_PickSettling = false; m_PickStall = 0;
			Vector3 ball = BodyPos(m_PickBallBody), site = m_Mj.GetSitePositionMj(m_GraspSite);
			Log.Info($"METRIC pick ball={m_PickN} phase={p} t={m_PickClock:F1} pos=({m_Qpos[0]:F3},{m_Qpos[1]:F3},{m_Qpos[2]:F3}) ball=({ball.X:F3},{ball.Y:F3},{ball.Z:F3}) " +
				$"site=({site.X:F3},{site.Y:F3},{site.Z:F3}) grip={Grip():F2} retry={m_PickRetry} {extra}");
		}

		void PickEnd(bool ok, string reason)
		{
			if (m_PickPhase == PickPhase.Off) return;
			Vector3 ball = BodyPos(m_PickBallBody), bin = BodyPos(m_PickBinBody);
			float rx = ball.X - bin.X, ry = ball.Y - bin.Y;
			bool inBin = MathF.Sqrt(rx * rx + ry * ry) < BinFloorR - 0.0213f && ball.Z < Rim && ball.Z > 0f;
			float moved = MathF.Sqrt((bin.X - m_PickBin0.X) * (bin.X - m_PickBin0.X) + (bin.Y - m_PickBin0.Y) * (bin.Y - m_PickBin0.Y)) * 100f;
			m_Mj.GetBodyPoseMj(m_PickBinBody, out Vector3 _, out Quaternion q, out Vector3 _, out Vector3 _);
			float binTilt = 2f * MathF.Acos(Math.Clamp(MathF.Abs(q.X * m_PickBinQ0.X + q.Y * m_PickBinQ0.Y + q.Z * m_PickBinQ0.Z + q.W * m_PickBinQ0.W), 0f, 1f)) * 57.2958f;  // angle from the starting orientation
			m_PickDone = true; m_PickOkLast = ok; m_PickInBin = inBin; m_PickReason = reason;
			m_StepLast = m_PickT; m_PickLast = m_PickClock;
			if (!ok && PickCarrying) m_ArmTarget[5] = GripOpen;  // let go of whatever is left in the hand
			Log.Info($"METRIC pick ball={m_PickN} result={(ok ? "ok" : "fail")} reason={reason} stop={m_PickStop} ballInBin={inBin} binMoved={moved:F2} binTilt={binTilt:F1} " +
				$"maxTilt={m_PickTilt:F1} ballSiteMax={m_PickMaxBallSite * 100f:F1} minZ={m_PickMinZ:F3} {NavFields()} retries={m_PickRetry} time={m_PickClock:F1}");
			m_PickPhase = PickPhase.Off;
		}

		float Grip() => m_Qpos[m_GripQ];

		// Fixed_Jaw frame: origin and rotation (m_Rj, jaw -> world).
		Vector3 JawFrame()
		{
			m_Mj.GetBodyPoseMj(m_Jaw, out Vector3 p, out Quaternion q, out Vector3 _, out Vector3 _);
			QuatToMatrix(q.W, q.X, q.Y, q.Z, m_Rj);
			return p;
		}

		// A point of the Fixed_Jaw frame in the world.
		Vector3 JawPoint(Vector3 pj)
		{
			Vector3 o = JawFrame();
			return new Vector3(o.X + m_Rj[0, 0] * pj.X + m_Rj[0, 1] * pj.Y + m_Rj[0, 2] * pj.Z,
				o.Y + m_Rj[1, 0] * pj.X + m_Rj[1, 1] * pj.Y + m_Rj[1, 2] * pj.Z,
				o.Z + m_Rj[2, 0] * pj.X + m_Rj[2, 1] * pj.Y + m_Rj[2, 2] * pj.Z);
		}

		// A world point in the Fixed_Jaw frame (the ball's place between the jaws).
		Vector3 InJaw(Vector3 w)
		{
			Vector3 o = JawFrame(), d = w - o;
			return new Vector3(m_Rj[0, 0] * d.X + m_Rj[1, 0] * d.Y + m_Rj[2, 0] * d.Z,
				m_Rj[0, 1] * d.X + m_Rj[1, 1] * d.Y + m_Rj[2, 1] * d.Z,
				m_Rj[0, 2] * d.X + m_Rj[1, 2] * d.Y + m_Rj[2, 2] * d.Z);
		}

		// Analytic IK: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex (into q) that put the tool point (world) on `tool`
		// with the finger vertical (lift + elbow + wrist = -PI/2), through the drone's measured pose. False when out of reach or limits.
		bool SolveIk(Vector3 tool, float[] q)
		{
			Vector3 d = tool - new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			float[,] R = m_R;
			return SolveIkBody(new Vector3(R[0, 0] * d.X + R[1, 0] * d.Y + R[2, 0] * d.Z, R[0, 1] * d.X + R[1, 1] * d.Y + R[2, 1] * d.Z, R[0, 2] * d.X + R[1, 2] * d.Y + R[2, 2] * d.Z), q);
		}

		// The same, for a tool point in the drone body frame (x points back, z up).
		bool SolveIkBody(Vector3 t, float[] q)
		{
			float tx = t.X, ty = t.Y, tz = t.Z;
			// Wrist-to-tool with the finger vertical: the tool offset moves it by jaw (-dz, -dx, +dy) in the body frame.
			Vector3 dt = m_Tool - Tool0, wo = new(IkWoff0.X - dt.Z, IkWoff0.Y - dt.X, IkWoff0.Z + dt.Y);
			float rr = PickRoll - PickRoll0, cr = MathF.Cos(rr), sr = MathF.Sin(rr);
			wo = new Vector3(cr * wo.X - sr * wo.Y, sr * wo.X + cr * wo.Y, wo.Z);  // the wrist roll turns the tool offset about the (vertical) finger
			float l1 = MathF.Sqrt(IkE.X * IkE.X + IkE.Z * IkE.Z), l2 = MathF.Sqrt(IkW.X * IkW.X + IkW.Z * IkW.Z);
			float a = wo.Y, dx = tx - IkC.X, dy = ty - IkC.Y, rr2 = dx * dx + dy * dy - a * a;
			if (rr2 < 0f) return false;
			float px = IkC.X - MathF.Sqrt(rr2);
			float pan = Wrap(MathF.Atan2(a, px - IkC.X) - MathF.Atan2(dy, dx));
			float vx = px - wo.X - IkS.X, vz = tz - wo.Z - IkS.Z, r = MathF.Sqrt(vx * vx + vz * vz);
			if (r > l1 + l2 || r < MathF.Abs(l1 - l2)) return false;
			float beta = -MathF.Acos(Math.Clamp((r * r - l1 * l1 - l2 * l2) / (2f * l1 * l2), -1f, 1f));  // the elbow branch inside the limits
			float a10 = MathF.Atan2(IkE.Z, IkE.X), a20 = MathF.Atan2(IkW.Z, IkW.X);
			float elbow = Wrap(a20 - a10 - beta);
			float a1 = MathF.Atan2(vz, vx) - MathF.Atan2(l2 * MathF.Sin(beta), l1 + l2 * MathF.Cos(beta));
			float lift = Wrap(a10 - a1);
			float wrist = Wrap(-MathF.PI / 2f - lift - elbow);
			q[0] = pan; q[1] = lift; q[2] = elbow; q[3] = wrist;
			for (int i = 0; i < 4; i++)
				if (q[i] < m_ArmLow[i] - 1e-3f || q[i] > m_ArmHigh[i] + 1e-3f) return false;
			return true;
		}

		static float Wrap(float a) => MathF.IEEERemainder(a, 2f * MathF.PI);

		// Slews the arm joints to the IK of `tool` at `speed` rad/s. False when unreachable.
		bool ArmServo(Vector3 tool, float speed, float dt, out bool arrived)
		{
			arrived = false;
			if (!SolveIk(tool, m_PickQ)) return false;
			arrived = SlewArm(speed, dt);
			return true;
		}

		// Moves the joints toward m_PickQ (wrist roll to PickRoll) at `speed` rad/s. True when there.
		bool SlewArm(float speed, float dt)
		{
			bool arrived = true;
			for (int i = 0; i < 5; i++)
			{
				float goal = i < 4 ? m_PickQ[i] : PickRoll;
				MoveArmJoint(i, Math.Clamp(goal - m_ArmTarget[i], -speed * dt, speed * dt));
				arrived &= MathF.Abs(goal - m_ArmTarget[i]) < 0.01f;
			}
			return arrived;
		}

		// On the way to the hover point: the gripper opens and the arm goes to the pre-grasp pose for the planned hover (drone level, the ball
		// Standoff ahead), so Grab only corrects the rest there.
		void PickPrePose(Vector3 ball, float dt)
		{
			m_ArmTarget[5] = MoveToward(m_ArmTarget[5], GripOpen, GripOpenSpeed * dt);
			if (SolveIkBody(new Vector3(-Standoff, 0f, ball.Z + m_ToolDz - HPick), m_PickQ)) SlewArm(PickArmSpeed, dt);
		}

		void PickIkSelfTest()
		{
			// Design example: drone level at the origin, tool at (-0.24, 0, -0.3957): (-0.2332, -1.9950, 1.5998, -1.1756).
			// m_R is the identity here only if the drone is level at yaw PI, so this just checks for NaN and logs; the real check is the jaw-frame error at the ball (atGround, in Grab).
			m_Mj.GetActuatorGainPrms(m_Mj.GetActuatorIndex("gripper"), out float kp, out float kv);
			m_Mj.GetActuatorCtrlRange(m_Mj.GetActuatorIndex("gripper"), out float lo, out float hi);
			Log.Info($"PICK gripper actuator kp={kp} kv={kv} ctrlrange=({lo},{hi})");
			Log.Info($"PICK jaw check: site vs jaw-frame point (0,-0.103,0) err={(m_Mj.GetSitePositionMj(m_GraspSite) - JawPoint(new Vector3(0f, -0.103f, 0f))).Length() * 1000f:F2} mm");
		}

		// Settled: within tol of the goal and slower than vtol, for `hold` s (m_PickSub counts it).
		bool PickSettled(Vector3 goal, float tol, float vtol, float hold, float dt)
		{
			var pos = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			var vel = new Vector3(m_Qvel[0], m_Qvel[1], m_Qvel[2]);
			bool ok = !Flying && !m_YawOn && (pos - goal).Length() < tol && vel.Length() < vtol;
			if (ok) m_PickSub += dt; else m_PickSub = 0f;
			return m_PickSub >= hold;
		}

		float YawError() => MathF.IEEERemainder(MathF.Atan2(-m_R[1, 0], -m_R[0, 0]) - m_Yaw, 2f * MathF.PI);

		// A drone flight with its own limits (the mode's when 0).
		void PickFly(Vector3 goal, float vmax, float vz, float amax)
		{
			m_LimV = vmax; m_LimVz = vz; m_LimA = amax;
			FlyTo(goal);
			m_LimV = m_LimVz = m_LimA = 0f;
		}

		// Called every frame after the sticks and before the control law.
		void PickUpdate(float dt)
		{
			if (m_PickPhase == PickPhase.Off) return;
			m_PickClock += dt; m_PickT += dt;
			if (m_PickPhase != PickPhase.Transit || m_PickGrasped) m_ArmGoal = null;  // Transit folds the arm by itself, never while carrying
			m_PickTilt = MathF.Max(m_PickTilt, MathF.Acos(Math.Clamp(m_R[2, 2], -1f, 1f)) * 57.2958f);
			m_PickMinZ = MathF.Min(m_PickMinZ, m_Qpos[2]);
			if (m_PickClock > 240f) { PickEnd(false, "timeout"); return; }

			Vector3 ball = BodyPos(m_PickBallBody), site = m_Mj.GetSitePositionMj(m_GraspSite);
			var pos = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			float g = Grip();
			bool carrying = PickCarrying;
			if (carrying)
			{
				float bs = (ball - site).Length();
				m_PickMaxBallSite = MathF.Max(m_PickMaxBallSite, bs);
				if (bs > 0.04f) { PickEnd(false, "ball_slipped"); return; }
				m_ArmTarget[5] = Math.Clamp(MathF.Max(GripClosed, g - SqueezeLead), m_ArmLow[5], m_ArmHigh[5]);  // keep the light squeeze, don't re-latch lower
			}

			switch (m_PickPhase)
			{
				case PickPhase.Transit:
				{
					if (m_PickT > 120f) { PickEnd(false, $"transit_timeout flying={Flying} yawOn={m_YawOn} yawErr={YawError():F2}"); return; }
					if (m_PickGrasped)  // the carry: to above the bin with the ball, then drop it
					{
						if (!PickSettled(m_PickGoal, 0.015f, 0.03f, 0.5f, dt)) break;
						PickEnter(PickPhase.Drop, $"ballToBin={new Vector2(ball.X - m_PickBin0.X, ball.Y - m_PickBin0.Y).Length() * 100f:F1}cm");
						break;
					}
					// the arm folds for the flight (once the fingertips are clear of the ground: a pick can follow a failed one) and, within
					// ApproachR of the hover point, unfolds to the pre-grasp pose with the gripper open (Grab fine-aligns it there)
					if (HDist(pos, m_PickHov) < ApproachR) { m_ArmGoal = null; PickPrePose(ball, dt); }
					else if (site.Z > 0.2f) { m_ArmGoal = Stowed; m_ArmTarget[5] = MoveToward(m_ArmTarget[5], GripClosed, GripOpenSpeed * dt); }
					if (Flying) break;
					// the ball may have rolled since the plan (an earlier drop, a nudge): once it is still, plan again from where it is
					float bsp = HDist(ball, m_PickBallPrev) / dt;
					m_PickBallPrev = ball;
					if (HDist(ball, m_PickBall0) > 0.02f && m_PickReplans < 4)
					{
						if (bsp > 0.02f) { m_PickSub = 0f; break; }
						m_PickReplans++;
						m_PickBall0 = ball;
						PickPlanTransit();
						break;
					}
					// the heading must have converged; then 0.5 s of calm
					var vel = new Vector3(m_Qvel[0], m_Qvel[1], m_Qvel[2]);
					if (!m_YawOn && (pos - m_PickHov).Length() < 0.015f && vel.Length() < 0.03f && MathF.Abs(YawError()) < 0.02f) m_PickSub += dt; else m_PickSub = 0f;
					if (m_PickSub >= 0.5f) PickEnter(PickPhase.Grab, $"driftXY={new Vector2(pos.X - m_PickHov.X, pos.Y - m_PickHov.Y).Length() * 100f:F1}cm");
					break;
				}
				case PickPhase.Grab:
				{
					if (!m_PickSettling)
					{
						// the arm aligns on the ball (0.5 s held at HHov), the drone descends with it and presses on the fixed jaw's tip, 1 s of calm
						// (the arm holds its pose: servoing on the tilting drone would chase it), then the close
						if (m_PickT > 20f) { PickEnd(false, "grab_timeout"); return; }
						float moved = new Vector2(ball.X - m_PickBall0.X, ball.Y - m_PickBall0.Y).Length() * 100f;
						if (moved > 1f) { PickEnd(false, $"ball_moved_by_approach {moved:F1}cm"); return; }
						bool high = m_Target.Z > HPick + 1e-3f;
						if ((high || Flying) && (!PickDeploy(ball, dt) || Flying)) { m_PickSub = 0f; break; }
						if ((m_PickSub += dt) < (high ? 0.5f : 1f)) break;
						m_PickSub = 0f;
						if (high) { PickFly(new Vector3(pos.X, pos.Y, HPick - GrabPress), DescentV, DescentV, DescentA); break; }
						Vector3 inJaw0 = InJaw(ball);
						Log.Info($"METRIC pick ball={m_PickN} atGround ballMoved={(ball - m_PickBall0).Length() * 100f:F2}cm ballInJaw=({inJaw0.X:F4},{inJaw0.Y:F4},{inJaw0.Z:F4}) " +
							$"tool=({m_Tool.X:F4},{m_Tool.Y:F4},{m_Tool.Z:F4}) siteZ={site.Z:F4} droneZ={pos.Z:F3} minZ={m_PickMinZ:F3} grip={g:F3} gripCmd={m_ArmTarget[5]:F3}");
						if (m_PickStop == 'a') { PickEnd(true, "stage_a"); return; }  // the drone and the arm stay as they are
						m_PickSettling = true;
						break;
					}
					// Force-limited close: full torque (FastLead) down to GripSlowAt, then the command leads the measured angle by SqueezeLead, so the jaw meets the ball with ~1 Nm instead of the saturated 3.5 Nm.
					m_ArmTarget[5] = Math.Clamp(MathF.Max(GripClosed, g - (GripSlowAt > 0f && g > GripSlowAt ? FastLead : SqueezeLead)), m_ArmLow[5], m_ArmHigh[5]);
					float gv = m_Qvel[m_GripV];
					m_PickStall = MathF.Abs(gv) < 0.01f ? m_PickStall + 1 : 0;
					m_PickSub += dt;  // time since the close began
					if (m_PickStall == 1) { m_PickStopG = g; m_PickStopT = m_PickSub; }
					if (m_PickStall <= StallFrames && m_PickSub <= 4f) break;
					Log.Info($"METRIC close ball={m_PickN} time={m_PickSub:F2} stopT={m_PickStopT:F2} stopG={m_PickStopG:F3} open={GripOpen:F2} lead={SqueezeLead:F3} roll={PickRoll * 57.2958f:F0} fast={GripSlowAt:F2} stall={StallFrames}");
					// the close is over: the grasp is checked here, with the jaws still on the ball
					float dist = (ball - site).Length();
					Vector3 inJaw = InJaw(ball);
					Log.Info($"METRIC pick ball={m_PickN} afterClose grip={g:F3} ballSite={dist * 100f:F1}cm ballMoved={(ball - m_PickBall0).Length() * 100f:F2}cm ballInJaw=({inJaw.X:F4},{inJaw.Y:F4},{inJaw.Z:F4})");
					if (Grasped(g, dist, inJaw))
					{
						m_PickHc = pos.Z + (SafeLow + 0.02f - MathF.Min(site.Z - 0.004f, ball.Z - 0.0214f));
						m_PickGrasped = true; PickEnter(PickPhase.Lift, $"hc={m_PickHc:F2}");
						PickFly(new Vector3(pos.X, pos.Y, m_PickHc), 0f, 0f, 0f);
					}
					else if (m_PickRetry >= 2) PickEnd(false, $"grasp_not_detected grip={g:F2} ballSite={dist * 100f:F1}cm");
					else { m_PickRetry++; PickEnter(PickPhase.Retry); }
					break;
				}
				case PickPhase.Retry:
				{
					// open, fly back to the hover point behind the ball's new place (the arm keeps the fingertips over the ball on the way), then grab again
					if (m_PickT < 0.05f) { m_PickBall0 = ball; m_PickHov = new Vector3(ball.X - Standoff * MathF.Cos(m_PickYaw), ball.Y - Standoff * MathF.Sin(m_PickYaw), HHov); }
					if (!m_PickSettling)
					{
						m_ArmTarget[5] = MoveToward(m_ArmTarget[5], GripOpen, GripOpenSpeed * dt);
						if (MathF.Abs(m_ArmTarget[5] - GripOpen) < 0.01f) { m_PickSettling = true; FlyTo(m_PickHov); }
						break;
					}
					PickDeploy(ball, dt);
					if (PickSettled(m_PickHov, 0.015f, 0.03f, 0.5f, dt)) PickEnter(PickPhase.Grab);
					break;
				}
				case PickPhase.Lift:
				{
					if (m_PickT > 60f) { PickEnd(false, "lift_timeout"); return; }
					if (!PickSettled(new Vector3(pos.X, pos.Y, m_PickHc), 0.02f, 0.05f, 0.33f, dt)) break;
					if (ball.Z < 0.3f) { PickEnd(false, "ball_lost_in_lift"); return; }
					m_PickCarryOffset = new Vector3(ball.X - pos.X, ball.Y - pos.Y, 0f);
					Log.Info($"METRIC pick ball={m_PickN} liftDone ballZ={ball.Z:F3} ballSite={(ball - site).Length() * 100f:F1}cm");
					if (m_PickStop == 'b') { PickEnter(PickPhase.HoldB); break; }
					m_PickGoal = new Vector3(m_PickBin0.X - m_PickCarryOffset.X, m_PickBin0.Y - m_PickCarryOffset.Y, m_PickHc);
					PickEnter(PickPhase.Transit, $"goal=({m_PickGoal.X:F3},{m_PickGoal.Y:F3})");
					FlyTo(m_PickGoal);
					break;
				}
				case PickPhase.HoldB:
					if (m_PickT >= 10f) PickEnd(m_PickMaxBallSite < 0.03f, "stage_b");
					break;
				case PickPhase.Drop:
					m_ArmTarget[5] = MoveToward(m_ArmTarget[5], GripOpen, GripOpenSpeed * dt);
					if (BallInBin(m_PickBallBody) || m_PickT >= 1.8f) PickEnd(true, "dropped");  // done once the ball is in the can
					break;
			}
		}

		// The gripper opens and the arm goes to the pre-grasp pose for the drone's commanded height: the same drone-frame pose as at ground level,
		// so the arm holds it while the drone descends. The commanded, not the measured, height: at the ground the tool point stays put, so the
		// fingertips press the ground and lift the drone ~5 mm, which pins the jaws for the close (with the measured height the drone rises
		// during the close instead and drags the ball to the pad ends: no grasp). Called every frame by Grab and Retry. True when the arm is there with the gripper open.
		bool PickDeploy(Vector3 ball, float dt)
		{
			m_ArmTarget[5] = MoveToward(m_ArmTarget[5], GripOpen, GripOpenSpeed * dt);
			ArmServo(ball + new Vector3(0f, 0f, m_ToolDz + (m_Target.Z - HPick)), PickArmSpeed, dt, out bool arrived);
			return arrived && MathF.Abs(m_ArmTarget[5] - GripOpen) < 0.01f;
		}

		// Dev (`gp name value`): grasp knobs for tests: open (rad), lead (rad), roll (deg), fast (GripSlowAt rad, 0 = off), stall (frames).
		void GraspParam(string name, float v)
		{
			switch (name)
			{
				case "open": GripOpen = v; break;
				case "lead": SqueezeLead = v; break;
				case "roll": PickRoll = v * MathF.PI / 180f; break;
				case "fast": GripSlowAt = v; break;
				case "stall": StallFrames = (int)v; break;
			}
			Log.Info($"GP open={GripOpen:F2} lead={SqueezeLead:F3} roll={PickRoll * 57.2958f:F0} fast={GripSlowAt:F2} stall={StallFrames}");
		}

		// The ball is in the gripper and the drone is lifting it or carrying it to the bin (the squeeze and the slip check run then).
		bool PickCarrying => m_PickGrasped && m_PickPhase is PickPhase.Lift or PickPhase.HoldB or PickPhase.Transit;

		// The grasp is held when the jaw angle says a ball is in, it is at the grip site, and it is not pushed toward the pad ends (it would slip out in the lift).
		static bool Grasped(float g, float dist, Vector3 inJaw) => g > 0.30f && g < 0.60f && dist < 0.03f && inJaw.Y > -0.093f && MathF.Abs(inJaw.Z + 0.003f) < 0.006f;

		static float MoveToward(float v, float goal, float step) => v + Math.Clamp(goal - v, -step, step);
	}
}
