using System;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// The view of the Holybro X500 V2: an orbit camera that follows it, the HUD at the top right (CAMERA, MODE and
	/// Armed / Disarmed) and R, which puts everything back as when Play started (X500, golf balls, trash can).
	/// Camera: drag with the right mouse button or move the right stick (camera_yaw / camera_pitch) to orbit; Q/E or
	/// D-pad down/up (zoom_out / zoom_in) to zoom.
	/// Attach to the entity tagged World (its MuJoCo scene holds the X500).
	/// </summary>
	public class X500View : Entity
	{
		public float OrbitSpeed = 0.005f;  // rad per pixel of mouse drag
		public float ZoomSpeed = 1.5f;     // m/s while Q, E or the D-pad is held

		const float CameraStickSpeed = 2f;  // rad/s at full right stick
		// HUD, in pixels: the labels' x from the right edge, the first baseline from the top, the row height, the values' x after the labels.
		const float HudRight = 291f, HudTop = 49f, HudRow = 44f, HudValueX = 115f;
		const float HudModeLabelDx = 33f;  // right-aligns MODE with CAMERA (text has no alignment; widths measured on screen: 96, 62 px)
		// Armed / Disarmed ends just left of the MODE label: text has no alignment, so each word starts its own width (at
		// scale 16, estimated) plus a gap left of the label.
		const float HudArmedDx = 66f, HudDisarmedDx = 88f;
		// Rendered pixels per Application.Width unit: the monitor scale (Hyprland, 1.6) times Lucky's editor UI scale
		// (UiScale in ~/.config/LuckyEngine/LuckyEditor/EditorApplicationSettings.yaml, 0.75). Change it if either changes.
		const float HudPixelScale = 1.6f * 0.75f;
		static readonly Vector4 Green = new(0.30f, 0.80f, 0.52f, 1f), Red = new(0.94f, 0.33f, 0.36f, 1f);  // Armed, Disarmed

		MujocoSceneComponent m_Mj = null!;
		X500Flight? m_Flight;  // the X500's controller reading (camera sticks); null: mouse and keys only
		uint m_Body;
		Entity m_Camera = null!, m_HudCameraLabel = null!, m_HudCamera = null!, m_HudModeLabel = null!, m_HudMode = null!;
		Entity? m_HudArm;  // optional
		bool m_Missing;  // a tagged entity wasn't found: the script does nothing
		// Orbit camera: the camera sits on a sphere around the X500 and looks at its center.
		float m_OrbitYaw, m_OrbitPitch, m_Distance;
		Vector2 m_LastMouse;
		bool m_KeyR;
		(uint, uint) m_HudSize;
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
			OrbitCamera(MathF.Min(dt, 0.05f));
			UpdateHud();
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
			m_Flight ??= FindEntityByTag("HolybroX500V2")?.As<X500Flight>();  // here, not in OnCreate: its script may start later
			if (m_Flight != null)
			{
				m_OrbitYaw -= m_Flight.Axis("camera_yaw") * CameraStickSpeed * dt;
				m_OrbitPitch += m_Flight.Axis("camera_pitch") * CameraStickSpeed * dt;
				zoom += (m_Flight.Held("zoom_out") ? 1f : 0f) - (m_Flight.Held("zoom_in") ? 1f : 0f);
			}
			m_OrbitPitch = Math.Clamp(m_OrbitPitch, -1.4f, 1.4f);
			m_Distance = Math.Clamp(m_Distance + zoom * ZoomSpeed * dt, 0.3f, 20f);

			var dir = new Vector3(MathF.Sin(m_OrbitYaw) * MathF.Cos(m_OrbitPitch), MathF.Sin(m_OrbitPitch), MathF.Cos(m_OrbitYaw) * MathF.Cos(m_OrbitPitch));
			m_Camera.Translation = X500Position() + dir * m_Distance;
			m_Camera.Rotation = new Vector3(-m_OrbitPitch, m_OrbitYaw, 0f);  // the camera looks down its -z, so this faces the center
		}

		// Top right of the screen: CAMERA, MODE (X500Flight.ModeName) and Armed / Disarmed left of MODE.
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
			}
			if (m_HudArm != null) m_HudArm.Translation = m_HudModeLabel.Translation - new Vector3(X500Flight.Armed ? HudArmedDx : HudDisarmedDx, 0f, 0f);
			string shown = X500Flight.ModeName + (X500Flight.Armed ? "+" : "-");
			if (shown == m_HudShown) return;
			m_HudShown = shown;
			m_HudMode.GetComponent<TextComponent>()!.Text = X500Flight.ModeName;
			if (m_HudArm == null) return;
			var arm = m_HudArm.GetComponent<TextComponent>()!;
			arm.Text = X500Flight.Armed ? "Armed" : "Disarmed";
			arm.Color = X500Flight.Armed ? Green : Red;
		}
	}
}
