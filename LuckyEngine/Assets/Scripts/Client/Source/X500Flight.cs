using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// The X500 flown by PX4 through the sim (sim/src --lucky). The drone is this scene's MuJoCo body; the sim computes
	/// its forces (rotors, drag) at 250 Hz with PX4 and this script applies them, so every contact is Lucky's own.
	/// Each tick (sim/src/lucky.py, UDP 14600, one JSON line each way): send the body's free-joint qpos/qvel and the tick's
	/// length, wait for the sim's answer (force in the world, torque in the body frame, PX4's armed state and mode), set
	/// the six motors on the free joint (x500v2.mjcf), and MuJoCo steps. No answer for a second: motors off, "No PX4".
	/// The HUD (X500View) reads Armed and ModeName; the controller's camera bindings are read here for it.
	/// Attach to the HolybroX500V2 entity.
	/// </summary>
	public class X500Flight : Entity
	{
		/// For the HUD: the sim answering, and PX4's arming and mode.
		public static bool Connected => Environment.TickCount64 - s_LastRx < TimeoutMs;
		public static bool Armed;
		public static string ModeName = "No PX4";
		public static bool Calibrating => false;

		const int SimPort = 14600;
		const long TimeoutMs = 1000;
		const int WaitUs = 100_000;  // longest wait for the sim's answer each tick, while it is connected

		MujocoSceneComponent m_Mj = null!;
		uint m_Qpos, m_Qvel;
		static readonly string[] WrenchNames = { "x500v2_fx", "x500v2_fy", "x500v2_fz", "x500v2_tx", "x500v2_ty", "x500v2_tz" };
		readonly MujocoActuator[] m_Wrench = new MujocoActuator[6];
		float[] m_Q = null!, m_V = null!;
		double m_LastTime = -1;
		bool m_Off;

		UdpClient m_Udp = null!;  // made in OnCreate: Lucky also constructs scripts outside Play
		static long s_LastRx = -TimeoutMs;

		int m_Pad = -1;
		float m_PadScan;
		// The camera's controller bindings: an axis (Axis >= 0, Sign flips it) or a raw button (Raw).
		readonly Dictionary<string, (int Axis, float Sign, int Raw)> m_Bindings = new()
		{
			["camera_yaw"] = (2, -1f, -1),    // right stick left/right: orbit around
			["camera_pitch"] = (3, -1f, -1),  // right stick up/down: orbit up/down
			["zoom_in"] = (-1, 1f, 15),       // D-pad up
			["zoom_out"] = (-1, 1f, 17),      // D-pad down
			["camera_view"] = (-1, 1f, 10),   // View: next camera
		};
		const float Deadzone = 0.15f;
		readonly HashSet<string> m_ButtonsDown = new();

		// Props: entities tagged X500Prop1..4, spun about their local +Y while armed (cosmetic).
		static readonly float[] Spin = { 1, 1, -1, -1 };  // +1 = CCW from above
		readonly Entity?[] m_Props = new Entity?[4];
		readonly float[] m_PropAngle = new float[4];
		const float PropSpin = 40f;  // rad/s on screen

		protected override void OnCreate()
		{
			(Armed, ModeName, s_LastRx) = (false, "No PX4", -TimeoutMs);
			for (int i = 0; i < 4; i++) m_Props[i] = FindEntityByTag($"X500Prop{i + 1}");
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
			for (int k = 0; k < 6; k++)
			{
				m_Wrench[k] = m_Mj.GetActuator(WrenchNames[k])!;
				m_Wrench[k].IK = false;
			}
			StopMotors();
			m_Q = new float[m_Mj.ModelNq];
			m_V = new float[m_Mj.ModelNv];
			m_Udp = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0));
			m_Udp.Connect(IPAddress.Loopback, SimPort);
			Log.Info($"X500Flight: start, the sim on udp {SimPort}");
		}

		protected override void OnDestroy() => m_Udp?.Dispose();

		void StopMotors()
		{
			foreach (MujocoActuator a in m_Wrench) a.Value = 0;
		}

		protected override void OnUpdate(float frameDt)
		{
			if (m_Off) return;
			ScanPad(frameDt);
			double now = m_Mj.MujocoTime;
			float dt = (float)(now - m_LastTime);
			m_LastTime = now;
			if (dt <= 0f || dt > 0.1f) dt = 0.02f;  // first tick, or the scene was reset (R)
			bool was = Connected;
			Exchange(dt);
			if (Connected != was) Log.Info($"X500Flight: sim {(Connected ? "connected" : "lost")}");
			if (!Connected)
			{
				StopMotors();
				(Armed, ModeName) = (false, "No PX4");
			}
			SpinProps(frameDt);
		}

		// The body's state to the sim, its force and torque back onto the free joint's motors.
		void Exchange(float dt)
		{
			m_Mj.GetFullQpos(m_Q);
			m_Mj.GetFullQvel(m_V);
			var state = new { dt, qpos = m_Q[(int)m_Qpos..((int)m_Qpos + 7)], qvel = m_V[(int)m_Qvel..((int)m_Qvel + 6)] };
			byte[] line = Encoding.UTF8.GetBytes(JsonSerializer.Serialize(state));
			try
			{
				m_Udp.Send(line, line.Length);
				if (!m_Udp.Client.Poll(Connected ? WaitUs : 0, SelectMode.SelectRead)) return;
				byte[]? answer = null;
				IPEndPoint from = new(IPAddress.Any, 0);
				while (m_Udp.Available > 0) answer = m_Udp.Receive(ref from);  // the newest
				if (answer == null) return;
				using JsonDocument doc = JsonDocument.Parse(answer);
				JsonElement m = doc.RootElement;
				int k = 0;
				foreach (JsonElement x in m.GetProperty("force").EnumerateArray()) m_Wrench[k++].Value = x.GetSingle();
				foreach (JsonElement x in m.GetProperty("torque").EnumerateArray()) m_Wrench[k++].Value = x.GetSingle();
				(Armed, ModeName) = (m.GetProperty("armed").GetBoolean(), m.GetProperty("mode").GetString() ?? "");
				s_LastRx = Environment.TickCount64;
			}
			catch (Exception e) when (e is SocketException or JsonException or InvalidOperationException or KeyNotFoundException or IndexOutOfRangeException)
			{
				// the sim isn't there (yet), or a bad packet: the next tick tries again
			}
		}

		void SpinProps(float dt)
		{
			for (int i = 0; i < 4; i++)
			{
				if (m_Props[i] is not Entity prop) continue;
				m_PropAngle[i] = MathF.IEEERemainder(m_PropAngle[i] + (Armed ? Spin[i] * PropSpin * dt : 0f), 2f * MathF.PI);
				prop.Rotation = new Vector3(0f, m_PropAngle[i], 0f);
			}
		}

		void ScanPad(float dt)
		{
			if (m_Pad >= 0 && !Input.IsControllerPresent(m_Pad)) m_Pad = -1;
			if (m_Pad < 0 && (m_PadScan -= dt) <= 0f)
			{
				m_PadScan = 1f;  // GetConnectedControllerIDs allocates, so once a second rather than every frame
				int[] pads = Input.GetConnectedControllerIDs();
				m_Pad = pads.Length > 0 ? pads[0] : -1;
			}
		}

		bool Bound(string name, out (int Axis, float Sign, int Raw) b)
		{
			b = default;
			return m_Pad >= 0 && m_Bindings.TryGetValue(name, out b);
		}

		// An analog axis (-1..1) with the deadzone, 0 when unbound. X500View reads the camera's with it.
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
	}
}
