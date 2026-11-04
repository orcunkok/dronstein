using System;
using System.Collections.Generic;
using Hazel;

namespace Robots
{
	public class UnitreeGo2Agent : RobotAgent
	{
		public UnitreeGo2Agent(float dt)
			: base(dt)
		{ }

		private VelocityCommand m_VelCommand = new VelocityCommand();
		public float[] VelCommand => m_VelCommand.Values;

		// UnitreeGo2 does not require calibration - it uses external gRPC control
		protected override bool m_NeedsCalibration => false;

			// Foot configuration for privileged observations
		private const int m_NumFeet = 4;
		private static readonly string[] m_FootNames = { "FL", "FR", "RL", "RR" };
		private static readonly string[] m_FootCollisionGeomNames = {"FL", "FR", "RL", "RR"};

		private List<JointConfig>? _jointSpec;
		protected override List<JointConfig> JointSpec => _jointSpec ??= new List<JointConfig>
		{
			// Go2 has 12 joints: 4 legs × 3 joints each (hip abduction, hip pitch, knee)
			new JointConfig("FL_hip_joint",   0.0f,  0.3727530387083568f),
			new JointConfig("FL_thigh_joint", 0.9f,  0.3727530387083568f),
			new JointConfig("FL_calf_joint", -1.8f,  0.24850202580557115f),

			new JointConfig("FR_hip_joint",   0.0f,  0.3727530387083568f),
			new JointConfig("FR_thigh_joint", 0.9f,  0.3727530387083568f),
			new JointConfig("FR_calf_joint", -1.8f,  0.24850202580557115f),

			new JointConfig("RL_hip_joint",   0.0f,  0.3727530387083568f),
			new JointConfig("RL_thigh_joint", 0.9f,  0.3727530387083568f),
			new JointConfig("RL_calf_joint", -1.8f,  0.24850202580557115f),

			new JointConfig("RR_hip_joint",   0.0f,  0.3727530387083568f),
			new JointConfig("RR_thigh_joint", 0.9f,  0.3727530387083568f),
			new JointConfig("RR_calf_joint", -1.8f,  0.24850202580557115f),
		};

		private List<StateConfig>? _stateSpec;
		protected override List<StateConfig> StateSpec => _stateSpec ??= new List<StateConfig>
		{
			// Actor observations
			new StateConfig("base_lin_vel",     3),
			new StateConfig("base_ang_vel",     3),
			new StateConfig("base_quat",        4),
			new StateConfig("proj_grav",        3),
			new StateConfig("joint_pos",        m_NumJoints),
			new StateConfig("joint_vel",        m_NumJoints),
			new StateConfig("vel_command",      3),

			// Critic observations
			new StateConfig("foot_contacts",    m_NumFeet),
			new StateConfig("foot_heights",     m_NumFeet),
			new StateConfig("foot_forces",      m_NumFeet),
			new StateConfig("foot_air_time",    m_NumFeet),
			new StateConfig("foot_velocity",    m_NumFeet * 3),
			new StateConfig("illegal_contact",  1),
		};

		// --- State ---
		private float[] m_DefaultPos = null!;

		// --- Buffers ---
		private float[] m_JointPositions = null!;
		private float[] m_JointVelocities = null!;

		// --- Privileged observation state ---
		private uint[] m_FootGeomIds = null!;
		private uint[] m_FootSiteIds = null!;
		private uint[] m_FootCollisionGeomIds = null!;
		private float[] m_FootAirTime = null!;

		public override void OnSetup()
		{
			CommandManager.Add(m_VelCommand);

			m_PelvisBodyId = m_MujocoScene.GetBodyID("base");
			if (m_PelvisBodyId == uint.MaxValue)
			{
				Log.Error("[UnitreeGo2Agent] Could not find 'base' body in MuJoCo scene.");
			}

			m_DefaultPos = new float[m_NumJoints];

			m_JointPositions = new float[m_NumJoints];
			m_JointVelocities = new float[m_NumJoints];

			for (int i = 0; i < m_NumJoints; i++)
			{
				m_DefaultPos[i] = JointSpec[i].DefaultPosition;
			}

				// Cache foot geom/site IDs for privileged observations
			m_FootGeomIds = new uint[m_NumFeet];
			m_FootSiteIds = new uint[m_NumFeet];
			m_FootCollisionGeomIds = new uint[m_NumFeet];
			m_FootAirTime = new float[m_NumFeet];

			for (int i = 0; i < m_NumFeet; i++)
			{
				m_FootGeomIds[i] = m_MujocoScene.GetGeomID(m_FootNames[i]);
				m_FootSiteIds[i] = m_MujocoScene.GetSiteID(m_FootNames[i]);
				m_FootCollisionGeomIds[i] = m_MujocoScene.GetGeomID(m_FootCollisionGeomNames[i]);

				if (m_FootGeomIds[i] == uint.MaxValue)
					Log.Warn($"[UnitreeGo2Agent] Foot geom '{m_FootNames[i]}' not found");
				if (m_FootSiteIds[i] == uint.MaxValue)
					Log.Warn($"[UnitreeGo2Agent] Foot site '{m_FootNames[i]}' not found");
				if (m_FootCollisionGeomIds[i] == uint.MaxValue)
					Log.Warn($"[UnitreeGo2Agent] Foot collision geom '{m_FootCollisionGeomNames[i]}' not found");
			}
		}

		public override void OnReset()
		{
			Array.Clear(m_JointPositions, 0, m_NumJoints);
			Array.Clear(m_JointVelocities, 0, m_NumJoints);
			Array.Clear(m_FootAirTime, 0, m_NumFeet);
		}

		public override void OnState()
		{
			float[] state = m_StateBuffer;
			int idx = m_StateOffset;

			// Base velocities in body frame
			Quaternion trunkQuat = m_PelvisBodyId != uint.MaxValue
				? m_MujocoScene.GetOrientation(m_PelvisBodyId, hazelSpace: false)
				: Quaternion.Identity;

			if (m_PelvisBodyId == uint.MaxValue)
			{
				throw new Exception("[UnitreeGo2Agent] Cannot compute base velocities: 'base' body not found.");
			}

			Vector3 linVelWorld = m_MujocoScene.GetVelocity(m_PelvisBodyId, hazelSpace: false);
			Vector3 linVelBody = HzObservations.BaseLinVelB(trunkQuat, linVelWorld);
			state[idx++] = linVelBody.X;
			state[idx++] = linVelBody.Y;
			state[idx++] = linVelBody.Z;

			Vector3 angVelWorld = m_MujocoScene.GetAngularVelocity(m_PelvisBodyId, hazelSpace: false);
			Vector3 angVelBody = HzObservations.BaseAngVelB(trunkQuat, angVelWorld);
			state[idx++] = angVelBody.X;
			state[idx++] = angVelBody.Y;
			state[idx++] = angVelBody.Z;

			state[idx++] = trunkQuat.W;
			state[idx++] = trunkQuat.X;
			state[idx++] = trunkQuat.Y;
			state[idx++] = trunkQuat.Z;

			// Projected gravity in body frame (upright robot → (0, 0, -1))
			Vector3 gravWorld = new Vector3(0, 0, -9.81f);
			Vector3 gravBody = HzMath.QuatApplyInverse(trunkQuat, gravWorld);
			float len = gravBody.Length();
			if (len > 1e-6f) gravBody /= len;

			state[idx++] = gravBody.X;
			state[idx++] = gravBody.Y;
			state[idx++] = gravBody.Z;

			// Joint positions (relative to default) and velocities
			m_MujocoScene.GetJointPositions(m_JointQposAdrs, m_JointPositions);
			m_MujocoScene.GetJointVelocities(m_JointQvelAdrs, m_JointVelocities);

			for (int i = 0; i < m_NumJoints; i++)
				state[idx++] = m_JointPositions[i];

			for (int i = 0; i < m_NumJoints; i++)
				state[idx++] = m_JointVelocities[i];

			// Velocity command
			state[idx++] = VelCommand[0];
			state[idx++] = VelCommand[1];
			state[idx++] = VelCommand[2];

			int contactIdx = idx;
			int heightIdx  = idx + m_NumFeet;
			int forceIdx   = idx + m_NumFeet * 2;
			int airTimeIdx = idx + m_NumFeet * 3;

			int footVelIdx = idx + m_NumFeet * 4;

			for (int i = 0; i < m_NumFeet; i++)
			{
				bool hasGeom = m_FootGeomIds[i] != uint.MaxValue;
				bool hasSite = m_FootSiteIds[i] != uint.MaxValue;

				bool inContact = hasGeom && m_MujocoScene.IsGeomInContact(m_FootGeomIds[i]);

				state[contactIdx + i] = inContact ? 1f : 0f;
				state[heightIdx + i]  = hasSite ? m_MujocoScene.GetSitePositionMj(m_FootSiteIds[i]).Z : 0f;
				state[forceIdx + i]   = hasGeom ? m_MujocoScene.GetGeomContactForce(m_FootGeomIds[i]) : 0f;

				m_FootAirTime[i] = inContact ? 0f : m_FootAirTime[i] + m_dt;
				state[airTimeIdx + i] = m_FootAirTime[i];

				if (hasSite)
				{
					Vector3 vel = m_MujocoScene.GetSiteLinearVelocityMj(m_FootSiteIds[i]);
					state[footVelIdx + i * 3 + 0] = vel.X;
					state[footVelIdx + i * 3 + 1] = vel.Y;
					state[footVelIdx + i * 3 + 2] = vel.Z;
				}
				else
				{
					state[footVelIdx + i * 3 + 0] = 0f;
					state[footVelIdx + i * 3 + 1] = 0f;
					state[footVelIdx + i * 3 + 2] = 0f;
				}
			}

			// Illegal contact: 1.0 if any non-foot robot geom touches the ground
			int illegalIdx = footVelIdx + m_NumFeet * 3;
			state[illegalIdx] = m_MujocoScene.HasNonFootGroundContact(m_FootCollisionGeomIds) ? 1f : 0f;
		}

		public override void OnAction()
		{
			// Pass-through: actions are processed joint positions from LuckyLab
			for (int i = 0; i < m_NumJoints; i++)
			{
				m_CtrlBuffer[m_CtrlOffset + i] = m_ActionBuffer[m_ActionOffset + i];
			}
		}
	}
}
