using System;
using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Fly the Skydio X2 and move its SO-100 arm from the keyboard.
	/// Attach to the entity that holds dronstein.xml (its MujocoScene).
	///
	///   W/S forward/back   A/D left/right   LeftShift/LeftControl up/down   Q/E turn
	///   J/L turn the arm   1 switches J/L to rolling the wrist and back
	///   2-5 pick shoulder/elbow/wrist/gripper   I/K move it
	///
	/// The keys move a target point; a PD controller tilts the drone towards it.
	/// All math is in MuJoCo's frame (z up), straight from qpos/qvel.
	/// </summary>
	public class DroneTeleop : Entity
	{
		public float MoveSpeed = 1.0f;  // m/s
		public float TurnSpeed = 1.0f;  // rad/s
		public float ArmSpeed = 1.0f;   // rad/s

		public float KpPos = 2.0f, KdPos = 2.5f;  // position loop
		public float MaxTiltAcc = 4.0f;           // m/s^2, caps horizontal acceleration (~22 deg tilt)
		public float KpAtt = 3.0f, KdAtt = 0.6f;  // attitude loop

		static readonly string[] Motors = { "thrust1", "thrust2", "thrust3", "thrust4" };
		static readonly string[] ArmJoints = { "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper" };
		// Rotor positions in the drone body frame and yaw torque per newton, from x2.xml.
		static readonly float[,] Rotors = { { -0.14f, -0.18f, -0.0201f }, { -0.14f, 0.18f, 0.0201f }, { 0.14f, 0.18f, -0.0201f }, { 0.14f, -0.18f, 0.0201f } };
		const float MaxThrust = 13f;

		MujocoSceneComponent m_Mj;
		MujocoActuator[] m_Motors = new MujocoActuator[4];
		MujocoActuator[] m_Arm = new MujocoActuator[6];
		float[] m_ArmTarget = { 0f, -3.32f, 3.11f, 1.18f, 0f, -0.174f };  // the arm's "rest" pose: folded up
		float[] m_ArmLow = new float[6], m_ArmHigh = new float[6];
		uint m_Drone;
		float[] m_Qpos, m_Qvel;

		Entity m_Camera, m_Body, m_GripDot;
		uint m_GraspSite;
		Vector3 m_CameraOffset;

		Vector3 m_Target;
		float m_Yaw;
		bool m_RollWrist;  // J/L: false = turn the arm, true = roll the wrist
		int m_Joint = 1;   // the arm joint on I/K

		protected override void OnCreate()
		{
			m_Mj = GetComponent<MujocoSceneComponent>();
			m_Mj.SetControlledManually(true);  // otherwise Lucky rewrites ctrl every step to hold the pose
			for (int i = 0; i < 4; i++)
			{
				m_Motors[i] = m_Mj.GetActuator(Motors[i]);
				m_Motors[i].IK = false;
			}
			for (int i = 0; i < 6; i++)
			{
				m_Arm[i] = m_Mj.GetActuator(ArmJoints[i]);
				m_Arm[i].IK = false;
				m_Mj.GetActuatorCtrlRange(m_Mj.GetActuatorIndex(ArmJoints[i]), out m_ArmLow[i], out m_ArmHigh[i]);
			}
			m_Drone = m_Mj.GetBodyID("x2");
			m_Qpos = new float[m_Mj.ModelNq];
			m_Qvel = new float[m_Mj.ModelNv];

			m_Mj.GetFullQpos(m_Qpos);
			m_Target = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2] + 0.5f);  // take off 0.5 m
			// Chase camera: keep the scene camera at its authored offset from the drone.
			m_Camera = FindEntityByTag("Camera");
			m_Body = FindEntityByTag("x2");
			m_CameraOffset = m_Camera.Translation - m_Body.Translation;
			// Red dot on the ground straight below the gripper.
			m_GripDot = FindEntityByTag("GripDot");
			m_GraspSite = m_Mj.GetSiteID("grasp_site");

			Log.Info($"DroneTeleop: start at {m_Target}, nq={m_Mj.ModelNq} nu={m_Mj.ModelNu}");
		}

		float m_LogTimer;

		protected override void OnUpdate(float dt)
		{
			var forward = new Vector3(MathF.Cos(m_Yaw), MathF.Sin(m_Yaw), 0f);
			var left = new Vector3(-MathF.Sin(m_Yaw), MathF.Cos(m_Yaw), 0f);
			float step = MoveSpeed * dt;
			if (Input.IsKeyDown(KeyCode.W)) m_Target += forward * step;
			if (Input.IsKeyDown(KeyCode.S)) m_Target -= forward * step;
			if (Input.IsKeyDown(KeyCode.A)) m_Target += left * step;
			if (Input.IsKeyDown(KeyCode.D)) m_Target -= left * step;
			if (Input.IsKeyDown(KeyCode.LeftShift)) m_Target.Z += step;
			if (Input.IsKeyDown(KeyCode.LeftControl)) m_Target.Z -= step;
			if (Input.IsKeyDown(KeyCode.Q)) m_Yaw += TurnSpeed * dt;
			if (Input.IsKeyDown(KeyCode.E)) m_Yaw -= TurnSpeed * dt;

			// Rotating joints go on J/L, the rest on I/K (arrow keys would also drive the editor).
			if (Input.IsKeyPressed(KeyCode.D1))
			{
				m_RollWrist = !m_RollWrist;
				Log.Info($"J/L: {(m_RollWrist ? "wrist_roll" : "shoulder_pan")}");
			}
			KeyCode[] keys = { KeyCode.D2, KeyCode.D3, KeyCode.D4, KeyCode.D5 };
			int[] joints = { 1, 2, 3, 5 };  // shoulder_lift, elbow_flex, wrist_flex, gripper
			for (int i = 0; i < 4; i++)
				if (Input.IsKeyPressed(keys[i]))
				{
					m_Joint = joints[i];
					Log.Info($"I/K: {ArmJoints[m_Joint]}");
				}

			float turn = 0f;
			if (Input.IsKeyDown(KeyCode.J)) turn += ArmSpeed * dt;
			if (Input.IsKeyDown(KeyCode.L)) turn -= ArmSpeed * dt;
			MoveArmJoint(m_RollWrist ? 4 : 0, turn);

			float armStep = 0f;
			if (Input.IsKeyDown(KeyCode.I)) armStep += ArmSpeed * dt;
			if (Input.IsKeyDown(KeyCode.K)) armStep -= ArmSpeed * dt;
			MoveArmJoint(m_Joint, armStep);

			Control(dt);
			m_Camera.Translation = m_Body.Translation + m_CameraOffset;
			Vector3 grasp = m_Mj.GetSitePositionMj(m_GraspSite);  // MuJoCo (x, y) on the ground is Lucky (x, -z)
			m_GripDot.Translation = new Vector3(grasp.X, m_GripDot.Translation.Y, -grasp.Y);
		}

		void MoveArmJoint(int joint, float delta)
		{
			m_ArmTarget[joint] = Math.Clamp(m_ArmTarget[joint] + delta, m_ArmLow[joint], m_ArmHigh[joint]);
		}

		// OnPhysicsUpdate doesn't fire for MuJoCo robots, so this runs every frame from OnUpdate.
		void Control(float dt)
		{
			float[] thrust = MotorThrusts();
			for (int i = 0; i < 4; i++) m_Motors[i].Value = thrust[i];
			for (int i = 0; i < 6; i++) m_Arm[i].Value = m_ArmTarget[i];

			m_LogTimer += dt;
			if (m_LogTimer > 1f)
			{
				m_LogTimer = 0f;
				Log.Info($"DroneTeleop: pos=({m_Qpos[0]:F2}, {m_Qpos[1]:F2}, {m_Qpos[2]:F2}) thrust=[{thrust[0]:F1} {thrust[1]:F1} {thrust[2]:F1} {thrust[3]:F1}]");
			}
		}

		/// <summary>Port of the Python PD controller: target point + yaw -> 4 rotor thrusts in newtons.</summary>
		float[] MotorThrusts()
		{
			m_Mj.GetFullQpos(m_Qpos);
			m_Mj.GetFullQvel(m_Qvel);
			var pos = new Vector3(m_Qpos[0], m_Qpos[1], m_Qpos[2]);
			var vel = new Vector3(m_Qvel[0], m_Qvel[1], m_Qvel[2]);     // world frame
			var angVel = new Vector3(m_Qvel[3], m_Qvel[4], m_Qvel[5]);  // body frame
			float[,] R = QuatToMatrix(m_Qpos[3], m_Qpos[4], m_Qpos[5], m_Qpos[6]);  // body -> world

			// Mass and center of mass of the drone + arm.
			float mass = 0f;
			var com = Vector3.Zero;
			for (uint b = 0; b < m_Mj.ModelNbody; b++)
			{
				if (m_Mj.GetBodyRootID(b) != m_Drone) continue;
				m_Mj.GetBodyPoseMj(b, out Vector3 p, out Quaternion _, out Vector3 _, out Vector3 _);
				float m = m_Mj.GetBodyMass(b);
				mass += m;
				com += p * m;
			}
			com = com * (1f / mass);

			// Position loop: desired force in world frame.
			Vector3 acc = (m_Target - pos) * KpPos - vel * KdPos;
			acc.X = Math.Clamp(acc.X, -MaxTiltAcc, MaxTiltAcc);
			acc.Y = Math.Clamp(acc.Y, -MaxTiltAcc, MaxTiltAcc);
			Vector3 force = (acc + new Vector3(0f, 0f, 9.81f)) * mass;
			float totalThrust = force.X * R[0, 2] + force.Y * R[1, 2] + force.Z * R[2, 2];

			// Desired orientation: body z along the force, nose pointing at m_Yaw.
			Vector3 z = force.Normalized();
			Vector3 y = Vector3.Cross(z, new Vector3(MathF.Cos(m_Yaw), MathF.Sin(m_Yaw), 0f)).Normalized();
			Vector3 x = Vector3.Cross(y, z);
			float[,] Rt = { { x.X, y.X, z.X }, { x.Y, y.Y, z.Y }, { x.Z, y.Z, z.Z } };

			// Attitude loop: torque in body frame. E = Rt^T R - R^T Rt.
			float E(int i, int j)
			{
				float e = 0f;
				for (int k = 0; k < 3; k++) e += Rt[k, i] * R[k, j] - R[k, i] * Rt[k, j];
				return e;
			}
			var attErr = new Vector3(E(2, 1), E(0, 2), E(1, 0)) * 0.5f;
			Vector3 torque = attErr * -KpAtt - angVel * KdAtt;

			// Mixer: which rotor thrusts give this total thrust and torque about the center of mass.
			Vector3 d = com - pos;
			float comX = R[0, 0] * d.X + R[1, 0] * d.Y + R[2, 0] * d.Z;  // com in body frame
			float comY = R[0, 1] * d.X + R[1, 1] * d.Y + R[2, 1] * d.Z;
			var mix = new float[4, 5];
			for (int i = 0; i < 4; i++)
			{
				float rx = Rotors[i, 0] - comX, ry = Rotors[i, 1] - comY;
				mix[0, i] = 1f; mix[1, i] = ry; mix[2, i] = -rx; mix[3, i] = Rotors[i, 2];
			}
			mix[0, 4] = totalThrust; mix[1, 4] = torque.X; mix[2, 4] = torque.Y; mix[3, 4] = torque.Z;
			float[] f = Solve4(mix);
			for (int i = 0; i < 4; i++) f[i] = Math.Clamp(f[i], 0f, MaxThrust);
			return f;
		}

		static float[,] QuatToMatrix(float w, float x, float y, float z)
		{
			return new float[,]
			{
				{ 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y) },
				{ 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x) },
				{ 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y) },
			};
		}

		/// <summary>Solve a 4x4 system given as an augmented 4x5 matrix (Gaussian elimination).</summary>
		static float[] Solve4(float[,] a)
		{
			for (int c = 0; c < 4; c++)
			{
				int pivot = c;
				for (int r = c + 1; r < 4; r++)
					if (MathF.Abs(a[r, c]) > MathF.Abs(a[pivot, c])) pivot = r;
				for (int k = 0; k < 5; k++) (a[c, k], a[pivot, k]) = (a[pivot, k], a[c, k]);
				for (int r = 0; r < 4; r++)
				{
					if (r == c) continue;
					float factor = a[r, c] / a[c, c];
					for (int k = c; k < 5; k++) a[r, k] -= factor * a[c, k];
				}
			}
			return new[] { a[0, 4] / a[0, 0], a[1, 4] / a[1, 1], a[2, 4] / a[2, 2], a[3, 4] / a[3, 3] };
		}
	}
}
