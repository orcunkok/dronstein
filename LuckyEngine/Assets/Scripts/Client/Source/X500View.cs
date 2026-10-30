using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using System.Text.Json;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// The view of the Holybro X500 V2: an orbit camera that follows it, the HUD at the top right (CAMERA, MODE and
	/// Armed / Disarmed) and R, which puts everything back as when Play started (X500, golf balls, trash can).
	/// Camera: drag with the right mouse button or move the right stick (camera_yaw / camera_pitch) to orbit; Q/E or
	/// D-pad down/up (zoom_out / zoom_in) to zoom. C or camera_view (X) steps through the cameras: orbit the X500, follow
	/// it (glued behind its tail), orbit the gripper (the entity tagged Claw, if there is one), and back. Each orbit keeps
	/// its own zoom.
	/// Attach to the entity tagged World (its MuJoCo scene holds the X500).
	/// </summary>
	public class X500View : Entity
	{
		public float OrbitSpeed = 0.005f;  // rad per pixel of mouse drag
		public float ZoomSpeed = 1.5f;     // m/s while Q, E or the D-pad is held

		const float CameraStickSpeed = 2f;  // rad/s at full right stick
		static readonly Vector3 ClawCenter = new(0f, -0.06f, 0f);  // the gripper camera looks here, between the fingers
		const float TailBack = 1.2f, TailUp = 0.35f;  // follow camera: m behind the X500's center and above it, in its frame
		// MuJoCo world -> Lucky world (-90 deg about x), and the camera's frame (looks down -z, y up) -> the X500's FLU.
		static readonly Quaternion MjToLucky = new(-0.70710678f, 0f, 0f, 0.70710678f), CameraToFlu = new(0.5f, -0.5f, -0.5f, 0.5f);
		static readonly string[] CameraNames = { "X500 V2", "Follow", "Gripper" };  // the HUD's CAMERA, per m_View
		// HUD layout in Application.Width/Height units (scaled to rendered pixels by PixelScale): the labels' x from the
		// right edge, the first baseline from the top, the row height, the values' x after the labels, the text sizes.
		const float HudRight = 242.5f, HudTop = 41f, HudRow = 36.5f, HudValueX = 96f;
		const float HudLabelSize = 17.5f, HudValueSize = 23.3f, HudArmSize = 13.3f;
		const float HudModeLabelDx = 27.5f;  // right-aligns MODE with CAMERA (text has no alignment; widths measured on screen)
		// Armed / Disarmed ends just left of the MODE label: text has no alignment, so each word starts its own width
		// (measured on screen) plus a gap left of the label.
		const float HudArmedDx = 55f, HudDisarmedDx = 73f;
		static readonly Vector4 Green = new(0.30f, 0.80f, 0.52f, 1f), Red = new(0.94f, 0.33f, 0.36f, 1f);  // Armed, Disarmed

		MujocoSceneComponent m_Mj = null!;
		X500Flight? m_Flight;  // the X500's controller reading (camera sticks); null: mouse and keys only
		uint m_Body;
		Entity m_Camera = null!, m_HudCameraLabel = null!, m_HudCamera = null!, m_HudModeLabel = null!, m_HudMode = null!;
		Entity? m_HudArm, m_Claw;  // optional
		bool m_Missing;  // a tagged entity wasn't found: the script does nothing
		// Orbit camera: the camera sits on a sphere around the X500 and looks at its center.
		float m_OrbitYaw, m_OrbitPitch, m_Distance;
		int m_View;                      // 0 orbit the X500, 1 follow it, 2 orbit the gripper
		float m_OtherDistance = 0.4f;    // the zoom of the orbit not shown (X500 or gripper), swapped in when it shows
		Vector2 m_LastMouse;
		bool m_KeyR, m_KeyC;
		(uint, uint) m_HudSize;
		float m_PixelScale = 1f;  // rendered pixels per Application.Width unit, see PixelScale
		string m_HudShown = "";  // the mode and arming last written, so the texts change only when they do

		protected override void OnCreate()
		{
			m_Mj = GetComponent<MujocoSceneComponent>()!;
			m_Body = m_Mj.GetBodyID("x500v2");
			m_Camera = Find("Camera");
			m_HudCameraLabel = Find("HudCameraLabel");
			m_HudCamera = Find("HudCamera");
			m_HudModeLabel = Find("HudModeLabel");
			m_HudMode = Find("HudMode");
			m_HudArm = FindEntityByTag("HudArm");
			m_Claw = FindEntityByTag("Claw");
			if (m_Missing) return;
			// Start from the scene camera's authored offset from the X500.
			Vector3 offset = m_Camera.Translation - X500Position();
			m_Distance = MathF.Max(offset.Length(), 0.3f);
			m_OrbitYaw = MathF.Atan2(offset.X, offset.Z);
			m_OrbitPitch = MathF.Asin(Math.Clamp(offset.Y / m_Distance, -1f, 1f));
			m_HudCamera.GetComponent<TextComponent>()!.Text = "X500 V2";
		}

		// The entity with this tag; logs and sets m_Missing when there is none.
		Entity Find(string tag)
		{
			Entity? e = FindEntityByTag(tag);
			if (e == null)
			{
				Log.Error($"X500View: no entity tagged '{tag}' in the scene, the script is off");
				m_Missing = true;
			}
			return e!;
		}

		protected override void OnUpdate(float dt)
		{
			if (m_Missing) return;
			bool r = Input.IsKeyDown(KeyCode.R);
			if (r && !m_KeyR)
			{
				m_Mj.ResetToInitialState(false);  // false: the simulation clock restarts too (X500Flight sees it and disarms)
				Log.Info("X500View: scene reset");
			}
			m_KeyR = r;
			bool c = Input.IsKeyDown(KeyCode.C);
			m_Flight ??= FindEntityByTag("HolybroX500V2")?.As<X500Flight>();  // here, not in OnCreate: its script may start later
			if ((c && !m_KeyC) | (m_Flight?.Pressed("camera_view") ?? false)) NextView();  // | : Pressed runs every frame
			m_KeyC = c;
			if (m_View == 1) FollowCamera();
			else OrbitCamera(MathF.Min(dt, 0.05f));
			UpdateHud();
		}

		// C / camera_view: the next camera; the gripper's orbit is skipped when there is no gripper.
		void NextView()
		{
			m_View = (m_View + 1) % (m_Claw == null ? 2 : 3);
			if (m_View != 1 && m_Claw != null)  // into one orbit from the other (through follow): swap in its zoom
				(m_Distance, m_OtherDistance) = (m_OtherDistance, m_Distance);
			m_HudCamera.GetComponent<TextComponent>()!.Text = CameraNames[m_View];
		}

		Vector3 Target() => m_View == 2 ? m_Claw!.Translation + ClawCenter : X500Position();

		// Glued to the X500: TailBack behind and TailUp above it, turning, pitching and rolling with it.
		void FollowCamera()
		{
			m_Mj.GetBodyPoseMj(m_Body, out Vector3 _, out Quaternion q, out Vector3 _, out Vector3 _);
			Quaternion rot = Mul(Mul(MjToLucky, q), CameraToFlu);
			m_Camera.RotationQuat = rot;
			m_Camera.Translation = X500Position() + Rotate(rot, new Vector3(0f, TailUp, TailBack));
		}

		static Quaternion Mul(Quaternion a, Quaternion b) => new(
			a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
			a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
			a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
			a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);

		static Vector3 Rotate(Quaternion q, Vector3 v)
		{
			Vector3 t = Vector3.Cross(q.XYZ, v) * 2f;
			return v + t * q.W + Vector3.Cross(q.XYZ, t);
		}

		// The X500's position in Lucky's frame (MuJoCo (x, y, z) is Lucky (x, -z, y)).
		Vector3 X500Position()
		{
			m_Mj.GetBodyPoseMj(m_Body, out Vector3 p, out Quaternion _, out Vector3 _, out Vector3 _);
			return new Vector3(p.X, p.Z, -p.Y);
		}

		void OrbitCamera(float dt)
		{
			Vector2 mouse = Input.GetMousePosition();
			if (Input.IsMouseButtonDown(MouseButton.Right))
			{
				m_OrbitYaw -= (mouse.X - m_LastMouse.X) * OrbitSpeed;
				m_OrbitPitch += (mouse.Y - m_LastMouse.Y) * OrbitSpeed;
			}
			m_LastMouse = mouse;
			float zoom = (Input.IsKeyDown(KeyCode.E) ? 1f : 0f) - (Input.IsKeyDown(KeyCode.Q) ? 1f : 0f);
			if (m_Flight != null)
			{
				m_OrbitYaw -= m_Flight.Axis("camera_yaw") * CameraStickSpeed * dt;
				m_OrbitPitch += m_Flight.Axis("camera_pitch") * CameraStickSpeed * dt;
				zoom += (m_Flight.Held("zoom_out") ? 1f : 0f) - (m_Flight.Held("zoom_in") ? 1f : 0f);
			}
			m_OrbitPitch = Math.Clamp(m_OrbitPitch, -1.4f, 1.4f);
			m_Distance = Math.Clamp(m_Distance + zoom * ZoomSpeed * dt, 0.3f, 20f);

			var dir = new Vector3(MathF.Sin(m_OrbitYaw) * MathF.Cos(m_OrbitPitch), MathF.Sin(m_OrbitPitch), MathF.Cos(m_OrbitYaw) * MathF.Cos(m_OrbitPitch));
			m_Camera.Translation = Target() + dir * m_Distance;
			m_Camera.Rotation = new Vector3(-m_OrbitPitch, m_OrbitYaw, 0f);  // the camera looks down its -z, so this faces the center
		}

		// Top right of the screen: CAMERA, MODE (X500Flight.ModeName) and Armed / Disarmed left of MODE.
		// Screen-space text is placed and sized in rendered pixels from the bottom left; Application.Width/Height are
		// smaller than that by PixelScale. Laid out again whenever the view's size changes (a resize or another monitor).
		void UpdateHud()
		{
			(uint Width, uint Height) size = (Application.Width, Application.Height);
			if (size != m_HudSize)
			{
				m_HudSize = size;
				float s = m_PixelScale = PixelScale(), x = (size.Width - HudRight) * s, y = (size.Height - HudTop) * s;
				Place(m_HudCameraLabel, x, y, HudLabelSize * s);
				Place(m_HudCamera, x + HudValueX * s, y, HudValueSize * s);
				Place(m_HudModeLabel, x + HudModeLabelDx * s, y - HudRow * s, HudLabelSize * s);
				Place(m_HudMode, x + HudValueX * s, y - HudRow * s, HudValueSize * s);
				if (m_HudArm != null) m_HudArm.Scale = new Vector3(HudArmSize * s, HudArmSize * s, 1f);
			}
			if (m_HudArm != null)
				m_HudArm.Translation = m_HudModeLabel.Translation - new Vector3((X500Flight.Armed ? HudArmedDx : HudDisarmedDx) * m_PixelScale, 0f, 0f);
			string shown = X500Flight.ModeName + (X500Flight.Armed ? "+" : "-");
			if (shown == m_HudShown) return;
			m_HudShown = shown;
			m_HudMode.GetComponent<TextComponent>()!.Text = X500Flight.ModeName;
			if (m_HudArm == null) return;
			var arm = m_HudArm.GetComponent<TextComponent>()!;
			arm.Text = X500Flight.Armed ? "Armed" : "Disarmed";
			arm.Color = X500Flight.Armed ? Green : Red;
		}

		static void Place(Entity text, float x, float y, float size)
		{
			text.Translation = new Vector3(x, y, 0f);
			text.Scale = new Vector3(size, size, 1f);
		}

		// Rendered pixels per Application.Width unit: Lucky's UI scale times the scale of the monitor its window is on.
		// Scripts can't read either from Lucky, so: UiScale from Lucky's editor settings, the monitor's scale from
		// macOS (the window's backing scale, 2 on Retina) or Hyprland (hyprctl). Either missing counts as 1 (no scaling).
		static float PixelScale() => UiScale() * MonitorScale();

		static float UiScale()
		{
			try
			{
				string path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
					"LuckyEngine", "LuckyEditor", "EditorApplicationSettings.yaml");
				foreach (string line in File.ReadLines(path))
				{
					string t = line.Trim();
					if (t.StartsWith("UiScale:") && float.TryParse(t[8..], NumberStyles.Float, CultureInfo.InvariantCulture, out float v) && v > 0f)
						return v;
				}
			}
			catch (Exception) { }  // no settings file: Lucky's default
			return 1f;
		}

		static float MonitorScale()
		{
			if (OperatingSystem.IsMacOS()) return MacScale();
			try
			{
				using JsonDocument clients = JsonDocument.Parse(Hyprctl("clients -j"));
				int pid = Environment.ProcessId, monitor = -1;
				foreach (JsonElement c in clients.RootElement.EnumerateArray())
					if (c.GetProperty("pid").GetInt32() == pid) { monitor = c.GetProperty("monitor").GetInt32(); break; }
				using JsonDocument monitors = JsonDocument.Parse(Hyprctl("monitors -j"));
				foreach (JsonElement m in monitors.RootElement.EnumerateArray())
					if (m.GetProperty("id").GetInt32() == monitor) return m.GetProperty("scale").GetSingle();
			}
			catch (Exception) { }  // not Hyprland
			return 1f;
		}

		// [[NSApp mainWindow] backingScaleFactor], or the main screen's when no Lucky window is main (another app is active).
		static float MacScale()
		{
			try
			{
				IntPtr app = Msg(objc_getClass("NSApplication"), "sharedApplication"), window = Msg(app, "mainWindow");
				IntPtr target = window != IntPtr.Zero ? window : Msg(objc_getClass("NSScreen"), "mainScreen");
				if (target != IntPtr.Zero) return (float)MsgDouble(target, sel_registerName("backingScaleFactor"));
			}
			catch (Exception) { }
			return 1f;
		}

		static IntPtr Msg(IntPtr obj, string sel) => obj == IntPtr.Zero ? IntPtr.Zero : MsgPtr(obj, sel_registerName(sel));
		const string ObjC = "/usr/lib/libobjc.A.dylib";
		[DllImport(ObjC)] static extern IntPtr objc_getClass(string name);
		[DllImport(ObjC)] static extern IntPtr sel_registerName(string name);
		[DllImport(ObjC, EntryPoint = "objc_msgSend")] static extern IntPtr MsgPtr(IntPtr obj, IntPtr sel);
		[DllImport(ObjC, EntryPoint = "objc_msgSend")] static extern double MsgDouble(IntPtr obj, IntPtr sel);

		static string Hyprctl(string args)
		{
			using var p = Process.Start(new ProcessStartInfo("hyprctl", args) { RedirectStandardOutput = true, UseShellExecute = false })!;
			string output = p.StandardOutput.ReadToEnd();
			p.WaitForExit(1000);
			return output;
		}
	}
}
