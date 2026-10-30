using System;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// The Hiwonder wave-edge claw (tools/claw/build_claw.py), on its own for now: O (or the Open and Close buttons)
	/// swings both fingers fully open or fully closed about their pivots, the way the geared pair does. Visual only (no
	/// colliders yet).
	/// Attach to the Claw entity; its children tagged ClawFingerL and ClawFingerR are the fingers, mirror images of
	/// each other, so both turn by the same angle with opposite signs.
	/// </summary>
	public class Claw : Entity
	{
		[Tooltip("Each finger's swing from closed to fully open; 72.3 deg is the open pose measured on the Hiwonder drawing.")]
		[Units("deg")] [Slider(0f, 90f)]
		public float OpenAngle = 72.3f;

		[Tooltip("How fast the fingers swing.")]
		[Units("deg/s")] [Slider(10f, 720f)]
		public float Speed = 180f;

		Entity? m_Left, m_Right;
		float m_Angle, m_Target;  // degrees open: 0 closed; starts fully open
		bool m_KeyO;

		[Button("Open")]
		void Open() => m_Target = OpenAngle;

		[Button("Close")]
		void Close() => m_Target = 0f;

		// The gripper stick (X500View): stick > 0 opens, < 0 closes, at Speed at full stick.
		public void Drive(float stick, float dt) => m_Target = Math.Clamp(m_Target + stick * Speed * dt, 0f, OpenAngle);

		protected override void OnCreate()
		{
			m_Left = FindEntityByTag("ClawFingerL");
			m_Right = FindEntityByTag("ClawFingerR");
			m_Angle = m_Target = OpenAngle;
			if (m_Left == null || m_Right == null)
				Log.Error("Claw: needs children tagged ClawFingerL and ClawFingerR");
		}

		protected override void OnUpdate(float dt)
		{
			if (m_Left == null || m_Right == null) return;
			bool o = Input.IsKeyDown(KeyCode.O);
			if (o && !m_KeyO) m_Target = m_Target > 0f ? 0f : OpenAngle;  // O: fully open <-> fully closed
			m_KeyO = o;
			float step = Speed * dt;
			m_Angle += Math.Clamp(m_Target - m_Angle, -step, step);
			float a = m_Angle * MathF.PI / 180f;
			m_Left.Rotation = new Vector3(0f, 0f, -a);  // left tip swings out to -x
			m_Right.Rotation = new Vector3(0f, 0f, a);
		}
	}
}
