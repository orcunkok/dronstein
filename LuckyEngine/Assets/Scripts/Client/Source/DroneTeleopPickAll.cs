using System;
using System.Collections.Generic;
using System.Text;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Pick All: every golf ball not yet in the can, in a seeded random order, from a random start heading (Tuning's Pick All button,
	/// or `pickall [seed]` in the command file). Each ball is a normal pick (DroneTeleopPick.cs) planned from the live ball position;
	/// between balls the drone goes straight from the can to the next ball. A failed pick (after its own 2 grasp retries) is put at the
	/// end of the queue once if the ball is still on the ground outside the can, then skipped with its reason.
	/// </summary>
	public partial class DroneTeleop
	{
		enum PaState { Off, Start, Next, Picking }

		PaState m_PaState;
		int m_PaSeed, m_PaTotal, m_PaOk, m_PaIdx;
		readonly List<int> m_PaQueue = new();
		readonly HashSet<int> m_PaRetried = new();
		readonly List<string> m_PaFails = new();
		readonly uint[] m_PaBodies = new uint[26];
		float m_PaClock, m_PaWait;
		int m_PaHits, m_PaCur;
		Vector3 m_PaBin0;

		// Last pick's outcome, written by PickEnd.
		bool m_PickDone, m_PickOkLast, m_PickInBin;
		string m_PickReason = "";

		bool BallInBin(uint body)
		{
			Vector3 ball = BodyPos(body), bin = BodyPos(m_PickBinBody);
			float rx = ball.X - bin.X, ry = ball.Y - bin.Y;
			return MathF.Sqrt(rx * rx + ry * ry) < BinFloorR - 0.0213f && ball.Z < Rim && ball.Z > 0f;
		}

		void PickAllStart(int seed)
		{
			Entity? bin = FindEntityByTag("TrashCan");
			if (bin == null) { Log.Info("METRIC pickall result=fail reason=no_bin"); return; }
			m_PickBinBody = m_Mj.GetBodyID($"lucky_body_{bin.ID}");
			var rng = new System.Random(seed);
			m_PaQueue.Clear(); m_PaRetried.Clear(); m_PaFails.Clear();
			for (int n = 1; n <= 25; n++)
			{
				if (FindEntityByTag($"GolfBall_{n:00}") is not Entity e) continue;
				m_PaBodies[n] = m_Mj.GetBodyID($"lucky_body_{e.ID}");
				if (!BallInBin(m_PaBodies[n])) m_PaQueue.Add(n);
			}
			for (int i = m_PaQueue.Count - 1; i > 0; i--) { int j = rng.Next(i + 1); (m_PaQueue[i], m_PaQueue[j]) = (m_PaQueue[j], m_PaQueue[i]); }
			m_PaSeed = seed; m_PaTotal = m_PaQueue.Count; m_PaOk = 0; m_PaIdx = 0; m_PaClock = 0f; m_PaHits = 0; m_PaWait = 0f;
			m_PaBin0 = BodyPos(m_PickBinBody);
			m_Tuning.SpeedMode = 0;
			m_Waypoint = -1; m_ToBall = 0;
			// random start: a point 1..3 m out at 0.8..1.5 m height and a random heading (turned to with the arm stowed)
			float ang = (float)(rng.NextDouble() * 2 * Math.PI), rad = 1.5f + (float)rng.NextDouble() * 2f, yaw = (float)((rng.NextDouble() * 2 - 1) * Math.PI);
			var start = new Vector3(m_PaBin0.X + rad * MathF.Cos(ang), m_PaBin0.Y + rad * MathF.Sin(ang), 0.8f + (float)rng.NextDouble() * 0.7f);
			Log.Info($"METRIC pickall seed={seed} start order=[{string.Join(",", m_PaQueue)}] n={m_PaTotal} startPos=({start.X:F2},{start.Y:F2},{start.Z:F2}) startYaw={yaw * 57.2958f:F0}");
			FlyTo(start, yaw);
			m_PaState = PaState.Start;
		}

		void PickAllEnd(string why)
		{
			Vector3 bin = BodyPos(m_PickBinBody);
			Log.Info($"METRIC pickall seed={m_PaSeed} ok={m_PaOk}/{m_PaTotal} fail=[{string.Join(" ", m_PaFails)}] binMoved={HDist(bin, m_PaBin0) * 100f:F2}cm binHits={m_PaHits} time={m_PaClock:F0} end={why}");
			Log.Info("METRIC done pickall");
			m_PaState = PaState.Off;
		}

		void PickAllUpdate(float dt)
		{
			if (m_Tuning.DoPickAll) { m_Tuning.DoPickAll = false; PickAllStart(Environment.TickCount & 0xffff); }
			if (m_PaState == PaState.Off) return;
			m_PaClock += dt;
			if (m_PaClock > 3600f) { if (m_PickPhase != PickPhase.Off) PickEnd(false, "pickall_timeout"); PickAllEnd("timeout"); return; }
			switch (m_PaState)
			{
				case PaState.Start:
					if (!Flying && !m_YawOn && new Vector3(m_Qvel[0], m_Qvel[1], m_Qvel[2]).Length() < 0.05f && (m_PaWait += dt) > 1f) m_PaState = PaState.Next;
					break;
				case PaState.Next:
					if (m_PaQueue.Count == 0) { PickAllEnd("done"); return; }
					m_PaCur = m_PaQueue[0]; m_PaQueue.RemoveAt(0); m_PaIdx++;
					m_PickDone = false;
					Log.Info($"METRIC pickall next ball={m_PaCur} idx={m_PaIdx} queueLeft={m_PaQueue.Count} t={m_PaClock:F0}");
					PickStart(m_PaCur, 'c');
					if (m_PickPhase == PickPhase.Off && !m_PickDone) { m_PaFails.Add($"{m_PaCur}:start_failed"); }
					else m_PaState = PaState.Picking;
					break;
				case PaState.Picking:
					if (m_PickPhase != PickPhase.Off || !m_PickDone) break;
					m_PaHits = Math.Max(m_PaHits, m_NavContacts);
					if (m_PickReason == "sticks") { PickAllEnd("sticks"); return; }
					bool inBin = BallInBin(m_PaBodies[m_PaCur]);
					Vector3 b = BodyPos(m_PaBodies[m_PaCur]);
					if (inBin) { m_PaOk++; Log.Info($"METRIC pickall ball={m_PaCur} inBin=True reason={m_PickReason}"); }
					else
					{
						string why = m_PickOkLast ? "not_in_bin_after_drop" : m_PickReason;
						bool requeue = m_PaRetried.Add(m_PaCur) && b.Z < 0.06f && b.Z > 0f;
						if (requeue) m_PaQueue.Add(m_PaCur); else m_PaFails.Add($"{m_PaCur}:{why.Split(' ')[0]}");
						Log.Info($"METRIC pickall ball={m_PaCur} inBin=False reason={why} requeued={requeue} ballPos=({b.X:F2},{b.Y:F2},{b.Z:F3})");
					}
					m_PaState = PaState.Next;
					break;
			}
		}
	}
}
