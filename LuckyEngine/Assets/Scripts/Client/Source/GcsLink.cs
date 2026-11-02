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
	/// The flight controller's end of the link to the ground station (src/lucky/link.py), the way ArduPilot SITL talks to
	/// a GCS: every frame it sends its state as one JSON line to 127.0.0.1:14550, whether or not anyone listens, and takes
	/// the GCS's sticks and commands from whatever comes back. So either side can start first, and they find each other.
	/// GCS -> FC: {"rc": [roll, pitch, throttle, yaw], "seq": n, "cmd": "arm", "value": 1}, rc in PWM (1500 centred,
	/// pitch forward is low, as ArduPilot's RC1-4). A command runs once, when its seq is new; "ack" sends the seq back.
	/// No GCS packet for a second: the sticks read centred.
	/// </summary>
	public class GcsLink : IDisposable
	{
		const int GcsPort = 14550;
		const long TimeoutMs = 1000;

		readonly UdpClient m_Udp = new(new IPEndPoint(IPAddress.Loopback, 0));
		readonly IPEndPoint m_Gcs = new(IPAddress.Loopback, GcsPort);
		readonly int[] m_Rc = { 1500, 1500, 1500, 1500 };
		long m_LastRx = -TimeoutMs;
		long m_Seq;
		string m_Cmd = "";
		float m_Value;

		public bool Connected => Environment.TickCount64 - m_LastRx < TimeoutMs;

		/// RC channel 0..3 (roll, pitch, throttle, yaw) as -1..1, 0 when the GCS is gone.
		public float Stick(int channel) => Connected ? Math.Clamp((m_Rc[channel] - 1500) / 500f, -1f, 1f) : 0f;

		/// Reads what the GCS sent since last frame; returns its command once, when it is new (else null).
		public (string Cmd, float Value)? Receive()
		{
			bool was = Connected;
			long seq = m_Seq;
			try
			{
				while (m_Udp.Available > 0)
				{
					IPEndPoint from = new(IPAddress.Any, 0);
					using JsonDocument doc = JsonDocument.Parse(m_Udp.Receive(ref from));
					JsonElement m = doc.RootElement;
					int i = 0;
					foreach (JsonElement ch in m.GetProperty("rc").EnumerateArray()) m_Rc[i++] = ch.GetInt32();
					seq = m.GetProperty("seq").GetInt64();
					m_Cmd = m.GetProperty("cmd").GetString() ?? "";
					m_Value = m.GetProperty("value").GetSingle();
					m_LastRx = Environment.TickCount64;
				}
			}
			catch (Exception e) when (e is SocketException or JsonException or InvalidOperationException or KeyNotFoundException or IndexOutOfRangeException)
			{
				Log.Info($"GcsLink: bad packet ignored ({e.Message})");
			}
			if (Connected != was) Log.Info($"GcsLink: GCS {(Connected ? "connected" : "lost")}");
			if (seq == m_Seq || m_Cmd == "") return null;
			m_Seq = seq;
			return (m_Cmd, m_Value);
		}

		/// The FC's state to the GCS (NED, metres, radians), with the last command's seq.
		public void Send(bool armed, int mode, bool landed, float[] pos, float[] vel, float[] att, bool gripperClosed)
		{
			var state = new { armed, mode, landed, pos, vel, att, gripper = gripperClosed, ack = m_Seq };
			byte[] line = Encoding.UTF8.GetBytes(JsonSerializer.Serialize(state));
			try { m_Udp.Send(line, line.Length, m_Gcs); }
			catch (SocketException) { }  // nobody listening yet: fine, keep sending
		}

		public void Dispose() => m_Udp.Dispose();
	}
}
