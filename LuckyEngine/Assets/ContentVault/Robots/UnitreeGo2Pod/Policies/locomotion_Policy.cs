using System;
using Hazel;

namespace locomotion_Policy
{
	public class locomotion_Policy : Entity
	{
		// Match this to the PolicySlot.Id you see for 'locomotion' in the RobotControllerComponent inspector.
		private const uint k_SlotId = 1;

		// --- Policy commands (tweak live in the Properties panel) ---
		[Slider(-1.0f, 1.0f)]
		public float SetVx = 0.0f;
		[Slider(-1.0f, 1.0f)]
		public float SetVy = 0.0f;
		[Slider(-0.5f, 0.5f)]
		public float SetYawRate = 0.0f;
		[Slider(0.14f, 0.32f)]
		public float SetBaseHeight = 0.27f;

		protected override void OnCreate()
		{
			RobotControllerComponent robot = GetComponent<RobotControllerComponent>();
			robot.SetPolicyActive(k_SlotId, true);
		}

		protected override void OnUpdate(float deltaTime)
		{
			RobotControllerComponent robot = GetComponent<RobotControllerComponent>();

			// --- Drive the policy from the fields above ---
			robot.SetFloat(k_SlotId, 1u, SetVx);
			robot.SetFloat(k_SlotId, 2u, SetVy);
			robot.SetFloat(k_SlotId, 3u, SetYawRate);
			robot.SetFloat(k_SlotId, 4u, SetBaseHeight);
		}
	}
}
