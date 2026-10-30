using System;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// The Hiwonder wave-edge claw (tools/claw): O (or the Open and Close buttons, or the gripper stick, X500View) opens or
	/// closes it. It is physical: two hinged fingers in the X500's MuJoCo body (x500v2.mjcf, made by
	/// tools/x500v2/build_x500v2.py with tools/claw/claw_physics.py), each pushed to the target by a servo with a torque
	/// limit, so a ball stops the fingers and is squeezed with that torque. The fingers drawn here follow the physics.
	/// Attach to the Claw entity; its children tagged ClawFingerL and ClawFingerR are the fingers, mirror images of
	/// each other, so both turn by the same angle with opposite signs.
	/// </summary>
	public class Claw : Entity
	{
		[Tooltip("Each finger's swing from closed to fully open; 72.3 deg is the open pose measured on the Hiwonder drawing (and the MJCF joint's range).")]
		[Units("deg")] [Slider(0f, 90f)]
		public float OpenAngle = 72.3f;

		[Tooltip("How fast the servos' target swings.")]
		[Units("deg/s")] [Slider(10f, 720f)]
		public float Speed = 180f;

		const float Deg = MathF.PI / 180f;

		Entity? m_Left, m_Right;
		MujocoSceneComponent? m_Mj;
		bool m_Found;  // the servos and joints below are looked up
		uint m_ServoL, m_ServoR, m_QposL, m_QposR;  // the servos' actuator numbers and the joints' places in the position array
		float[] m_Q = Array.Empty<float>();
		float m_Angle, m_Target;  // degrees open, the servos' target and where it is going: 0 closed; starts fully open
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
			m_Mj = FindEntityByTag("World")?.GetComponent<MujocoSceneComponent>();
			if (m_Left == null || m_Right == null || m_Mj == null)
			{
				Log.Error("Claw: needs children tagged ClawFingerL and ClawFingerR and the entity tagged World (its MuJoCo scene holds the claw); off");
				return;
			}
			m_Angle = m_Target = OpenAngle;
		}

		// The servos and joints, once the MuJoCo world has them (it can come up after this script starts).
		bool FindServos()
		{
			if (m_Found) return true;
			(m_ServoL, m_ServoR) = (m_Mj!.GetActuatorIndex("x500v2_claw_l"), m_Mj.GetActuatorIndex("x500v2_claw_r"));
			if (m_ServoL >= m_Mj.ModelNu || m_ServoR >= m_Mj.ModelNu) return false;
			m_QposL = m_Mj.GetJointQposAdr(m_Mj.GetJointID("x500v2_claw_l_hinge"));
			m_QposR = m_Mj.GetJointQposAdr(m_Mj.GetJointID("x500v2_claw_r_hinge"));
			m_Q = new float[m_Mj.ModelNq];
			Log.Info("Claw: servos found");
			return m_Found = true;
		}

		protected override void OnUpdate(float dt)
		{
			if (m_Mj == null || !FindServos()) return;
			bool o = Input.IsKeyDown(KeyCode.O);
			if (o && !m_KeyO) m_Target = m_Target > 0f ? 0f : OpenAngle;  // O: fully open <-> fully closed
			m_KeyO = o;
			float step = Speed * dt;
			m_Angle += Math.Clamp(m_Target - m_Angle, -step, step);
			// The joints' zero is fully open and they turn negative to close, so the servos' target is the angle short of open.
			m_Mj.SetCtrlByIndex(m_ServoL, (m_Angle - OpenAngle) * Deg);
			m_Mj.SetCtrlByIndex(m_ServoR, (m_Angle - OpenAngle) * Deg);
			m_Mj.GetFullQpos(m_Q);
			float left = OpenAngle * Deg + m_Q[m_QposL], right = OpenAngle * Deg + m_Q[m_QposR];
			m_Left!.Rotation = new Vector3(0f, 0f, -left);  // left tip swings out to -x
			m_Right!.Rotation = new Vector3(0f, 0f, right);
		}
	}
}
