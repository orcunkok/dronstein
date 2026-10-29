using System;
using System.Collections.Generic;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Bin keep-out for every flight the script plans (FlyTo: goto, Set/B, Y, the pick, the test moves). The bin is a vertical cylinder
	/// up to its rim, read live (it can be pushed). Rule: whenever any part of the drone's footprint (rotors, the arm as it is now,
	/// the ball in hand) is within 0.5 m of the bin's edge, its lowest point must be at least 10 cm above the rim.
	/// A straight (diagonal) flight that would break the rule is split into a climb along it, cruise (high) over the zone, then straight to the goal; a goal that is itself too low
	/// inside the zone is raised (METRIC keepout clamp). The same extent is measured from the true state every frame while a
	/// flight or a pick runs and logged when it ends (METRIC flight minClear=... nearBin=...).
	/// </summary>
	public partial class DroneTeleop
	{
		const float BinR = 0.185f;          // m, outer radius of the can (floor 0.165) with a little margin
		const float BinWall = 0.454f;       // m, wall height above the can's origin (fallback; measured when the geoms say so)
		const float NearBin = 0.5f;         // m from the bin's edge: the rim rule applies
		const float RimAbove = 0.10f, PlanMargin = 0.04f;  // required clearance over the rim, and extra the planner adds on top
		const float FootR = 0.30f;          // m, rotors (0.23 from the centre) plus their size: the footprint of the body alone

		uint m_BinBody;
		bool m_HasBin;
		float m_Rim = 0.46f;                // m, the rim's height, from the bin's live pose
		uint[] m_BinGeoms = Array.Empty<uint>();  // the can's wall geoms (not the floor: the ball lands on it)
		float m_ExLow, m_ExReach;           // Extent's results
		Vector3[] m_Q = new Vector3[3];     // legs still to fly after the current one
		int m_QN, m_QI;
		bool Flying => m_PosOn || m_QI < m_QN;

		// A flight session (from FlyTo until the drone is still again, or a whole pick): what was measured from the true state.
		bool m_NavOn;
		float m_NavClear, m_NavT;
		bool m_NavNear, m_NavCompliant;     // m_NavCompliant: the rule has held at least once (a drone parked low next to the can can't satisfy it before it climbs)
		float m_NavStartClear;
		bool m_NavIgnore;                   // the take-off after a reset and `rawgoto` flights are not measured
		int m_NavContacts;
		Vector3 m_NavBin0;
		string m_Case = "";                 // label of the test case, logged with each flight
		readonly Queue<string> m_Script = new();   // commands of a test suite, one per idle moment
		float m_ScriptWait;

		void KeepOutInit()
		{
			Entity? bin = FindEntityByTag("TrashCan");
			if (bin == null) return;
			m_BinBody = m_Mj.GetBodyID($"lucky_body_{bin.ID}");
			m_HasBin = true;
			var ids = new List<uint>();
			string[] tags = new string[17];
			tags[0] = "TrashCan";
			for (int i = 1; i < 17; i++) tags[i] = i == 1 ? "TrashCan_Wall" : $"TrashCan_Wall_{i - 1:00}";
			foreach (string t in tags)
				if (FindEntityByTag(t) is Entity e) ids.Add(m_Mj.GetGeomID($"lucky_geom_{e.ID}"));
			m_BinGeoms = ids.ToArray();
		}

		// How low and how far out the drone reaches right now: m_ExLow = lowest z of body, arm (as posed) and ball in hand,
		// m_ExReach = farthest horizontal distance of any of it from the drone's origin.
		void Extent()
		{
			var pos = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			m_ExLow = pos.Z - StowedLow;
			m_ExReach = FootR;
			Part(pos, m_Mj.GetSitePositionMj(m_GraspSite), 0.005f);   // fingertip
			Part(pos, BodyPos(m_Jaw), 0.05f);
			Part(pos, BodyPos(m_Wrist), 0.05f);
			Part(pos, BodyPos(m_Elbow), 0.04f);
			Part(pos, BodyPos(m_Shoulder), 0.04f);
			if (PickCarrying) Part(pos, BodyPos(m_PickBallBody), 0.0214f);
		}

		void Part(Vector3 pos, Vector3 p, float radius)
		{
			m_ExLow = MathF.Min(m_ExLow, p.Z - radius);
			m_ExReach = MathF.Max(m_ExReach, MathF.Sqrt((p.X - pos.X) * (p.X - pos.X) + (p.Y - pos.Y) * (p.Y - pos.Y)) + radius + 0.03f);
		}

		void RefreshRim()
		{
			m_Rim = BodyPos(m_BinBody).Z + BinWall;
		}

		// The flight to `goal` the rule allows. Returns the first leg to fly; the rest wait in m_Q. Cancels legs still queued.
		Vector3 KeepOut(Vector3 goal)
		{
			m_QN = m_QI = 0;
			if (!m_HasBin) return goal;
			Vector3 bin = BodyPos(m_BinBody);
			RefreshRim();
			Extent();
			var pos = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			float below = pos.Z - m_ExLow, zone = BinR + NearBin + m_ExReach;
			float zsafe = m_Rim + RimAbove + PlanMargin + below;      // origin height at which the lowest point is rim + 10 cm + margin
			Vector3 p0 = m_Target;
			goal.Z = MathF.Max(goal.Z, MinHeight);
			float dGoal = HDist(goal, bin);
			if (dGoal < zone && goal.Z < zsafe)
			{
				Log.Info($"METRIC keepout clamp goal=({goal.X:F2},{goal.Y:F2},{goal.Z:F2}) bin=({bin.X:F2},{bin.Y:F2}) dist={dGoal:F2} zone={zone:F2} z->{zsafe:F2}");
				goal.Z = zsafe;
			}
			float tu = LastUnsafe(p0, goal, bin, zone, zsafe - 0.01f);
			if (tu < 0f) return goal;  // straight, diagonal when the heights differ
			bool high = goal.Z >= zsafe - 0.01f;                                    // a high goal: the cruise ends at it
			float ts = MathF.Max(FirstUnsafe(p0, goal, bin, zone, zsafe - 0.01f) - 0.06f, 0f), te = MathF.Min(tu + 0.02f, 1f);
			Vector3 s = p0 + (goal - p0) * ts, e = p0 + (goal - p0) * te;          // the straight track is unsafe between these
			var legs = new List<Vector3>(3);
			if (p0.Z < zsafe) legs.Add(new Vector3(s.X, s.Y, zsafe));              // climb along the track (straight up when it starts unsafe)
			legs.Add(high ? goal : new Vector3(e.X, e.Y, MathF.Max(e.Z, zsafe)));   // cruise high over the zone, along the track
			if (!high) legs.Add(goal);                                              // then straight (diagonal) down to the goal
			for (int i = 1; i < legs.Count; i++) m_Q[m_QN++] = legs[i];
			Log.Info($"METRIC keepout plan legs={legs.Count} zsafe={zsafe:F2} zone={zone:F2} from=({p0.X:F2},{p0.Y:F2},{p0.Z:F2}) to=({goal.X:F2},{goal.Y:F2},{goal.Z:F2})");
			return legs[0];
		}

		// The first fraction of the straight flight a -> b where the origin is below zsafe with its ground track within `zone` of the bin.
		static float FirstUnsafe(Vector3 a, Vector3 b, Vector3 bin, float zone, float zsafe)
		{
			for (int i = 0; i <= 50; i++)
			{
				Vector3 p = a + (b - a) * (i / 50f);
				if (HDist(p, bin) < zone && p.Z < zsafe) return i / 50f;
			}
			return 1f;
		}

		// The last fraction of the straight flight a -> b where the origin is below zsafe with its ground track within `zone` of the bin, -1 when none.
		static float LastUnsafe(Vector3 a, Vector3 b, Vector3 bin, float zone, float zsafe)
		{
			for (int i = 50; i >= 0; i--)
			{
				Vector3 p = a + (b - a) * (i / 50f);
				if (HDist(p, bin) < zone && p.Z < zsafe) return i / 50f;
			}
			return -1f;
		}

		static float HDist(Vector3 a, Vector3 b) => MathF.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

		// Every frame: the true clearance (lowest part over the rim) while any part of the footprint is within NearBin of the bin's edge,
		// contacts of the can's walls, and the end of a flight session.
		void NavUpdate(float dt)
		{
			if (!m_HasBin) return;
			if (m_NavIgnore) { if (!Flying) m_NavIgnore = false; return; }
			if (!m_NavOn && (Flying || m_PickPhase != PickPhase.Off)) NavBegin();
			if (!m_NavOn) return;
			m_NavT += dt;
			Vector3 bin = BodyPos(m_BinBody);
			RefreshRim();
			Extent();
			var pos = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			if (HDist(pos, bin) - BinR - m_ExReach < NearBin)
			{
				float clear = m_ExLow - m_Rim;
				if (!m_NavCompliant && clear < RimAbove) m_NavStartClear = MathF.Min(m_NavStartClear, clear);
				else { m_NavCompliant = true; m_NavNear = true; m_NavClear = MathF.Min(m_NavClear, clear); }
			}
			else m_NavCompliant = true;
			// Hard hits on the can's walls (balls resting inside it push with < 0.5 N each, so only more than 1 N counts).
			for (int i = 1; i < m_BinGeoms.Length; i++)  // 0 is the can's own base collider, always touching the ground
				if (m_Mj.IsGeomInContact(m_BinGeoms[i]) && m_Mj.GetGeomContactForce(m_BinGeoms[i]) > 1f) { m_NavContacts++; break; }
			if (Flying || m_PickPhase != PickPhase.Off) { m_NavT = 0f; return; }
			if (new Vector3(m_Qvel[0], m_Qvel[1], m_Qvel[2]).Length() > 0.05f || m_NavT < 1f) return;  // still settling
			NavLog("flight");
			m_NavOn = false;
		}

		void NavBegin()
		{
			m_NavOn = true;
			m_NavClear = 9f; m_NavNear = m_NavCompliant = false; m_NavContacts = 0; m_NavT = 0f; m_NavStartClear = 9f;
			m_NavBin0 = BodyPos(m_BinBody);
		}

		// METRIC flight / pick tail: clearance in cm (na when never near), binMoved in cm, frames with a wall contact.
		string NavFields() => $"minClear={(m_NavNear ? (m_NavClear * 100f).ToString("F1") : "na")} nearBin={m_NavNear} startClear={(m_NavStartClear < 8f ? (m_NavStartClear * 100f).ToString("F1") : "ok")} binMoved={HDist(BodyPos(m_BinBody), m_NavBin0) * 100f:F2} binHits={m_NavContacts}";

		void NavLog(string what) => Log.Info($"METRIC {what} case={(m_Case.Length > 0 ? m_Case : "-")} {NavFields()}");

		// A test suite: its commands run one at a time, each when the drone is still, nothing is flying and no pick runs.
		void ScriptUpdate(float dt)
		{
			if (m_Script.Count == 0) return;
			m_ScriptWait -= dt;
			if (m_ScriptWait > 0f || Flying || m_YawOn || m_PickPhase != PickPhase.Off || m_NavOn) return;
			if (new Vector3(m_Qvel[0], m_Qvel[1], m_Qvel[2]).Length() > 0.08f) return;
			string line = m_Script.Dequeue();
			m_ScriptWait = line.StartsWith("reset") ? 6f : 1.5f;  // after a reset the drone takes off first
			if (line == "end") { Log.Info("METRIC done bin"); return; }
			Command(line);
		}

		// `test bin`: adversarial flights around the can, one METRIC flight line each (case label = the last `label`).
		void StartBinTest()
		{
			m_Script.Clear();
			string[] cases =
			{
				"reset", "mode slow",
				"label cross_x+ z0.4", "goto 2.5 0 0.4", "goto -2.5 0 0.4",
				"label cross_x- z0.3", "goto 2.5 0 0.3",
				"label cross_y+ z0.5", "goto 0 2.5 0.5", "goto 0 -2.5 0.5",
				"label cross_y- z0.6", "goto 0 2.5 0.6",
				"label cross_diag z0.35", "goto 2 2 0.35", "goto -2 -2 0.35",
				"label chord_edge z0.3", "goto 1.5 0.6 0.3", "goto -1.5 0.6 0.3",
				"label goal_next_to_bin", "goto 0.55 0 0.3", "goto -0.6 0.1 0.3", "goto 0 0.7 0.3",
				"label deployed_cross", "goto 3 3 1.5", "arm deploy", "goto 2.5 0 0.6", "goto -2.5 0 0.6", "goto 0 2.5 0.4", "arm stow",
				"reset", "mode slow",
				"label parked_low_fly_away", "rawgoto 0.55 0 0.4", "label parked_low_fly_away2", "goto 3 0 0.4",
				"label parked_low_then_pick", "rawgoto 0.55 0.2 0.4", "label parked_low_pick", "pick 14",
				"label set_balls", "ball 3", "set", "ball 4", "set", "ball 5", "set", "ball 7", "set", "ball 24", "set",
				"label pick_far_side_after_drop", "pick 24", "pick 3", "pick 7",
				"end",
			};
			foreach (string c in cases) m_Script.Enqueue(c);
			m_ScriptWait = 0f;
			Log.Info("METRIC test bin started");
		}
	}
}
