using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Live knobs for development: select the entity tagged "Tuning" and drag them in the Properties panel while playing.
	/// To tune something new, add a public field here and read it from the other script.
	/// </summary>
	public class Tuning : Entity
	{
		[ToggleButton("Visible", "Hidden")] [Tooltip("Wire half-sphere of the gripper's reach (T toggles)")]
		public bool ShowReach = true;

		[Group("Speed")] [ReadOnly] [Dropdown("Slow", "Medium", "Fast")] [Tooltip("Current speed mode (set with the buttons below)")]
		public int SpeedMode = 0;

		[Group("Speed")] [ReadOnly] [Units("m/s")] [Tooltip("Top speed of the current mode, written by DroneTeleop")]
		public float TopSpeed;

		[Group("Speed")] [ReadOnly] [Units("m/s")] [Tooltip("The drone's current speed, written by DroneTeleop")]
		public float Speed;

		[Button("Slow")] [Tooltip("Precise: ~0.8 m/s")]
		internal void SetSlow() => SpeedMode = 0;

		[Button("Medium")] [Tooltip("~2.5 m/s")]
		internal void SetMedium() => SpeedMode = 1;

		[Button("Fast")] [Tooltip("~6 m/s")]
		internal void SetFast() => SpeedMode = 2;

		[Slider(0, 25)] [Tooltip("Golf ball to highlight, 1 to 25 for GolfBall_01 to GolfBall_25 (0 = none)")]
		public int Ball = 14;

		internal bool GoToBall;  // Set was clicked; DroneTeleop flies there and clears it

		[Slider(0.3f, 1f)] [Tooltip("How far the ball sits from the shoulder-lift pivot when Set arrives, as a fraction of the reach (1 = on the sphere)")]
		public float GraspReach = 0.65f;

		[Slider(10f, 90f)] [Units("deg")] [Tooltip("Angle below level from the shoulder-lift pivot down to the ball when Set arrives (90 = straight over it)")]
		public float GraspAngle = 45f;

		[Button("Set")] [Tooltip("Fly in behind the selected ball and low, so the ball sits inside the reach (GraspReach, GraspAngle)")]
		internal void SetBall() => GoToBall = true;

		internal bool DoPick;  // Pick was clicked; DroneTeleop starts the pick and clears it

		[Button("Pick")] [Tooltip("Pick the selected ball by itself and drop it in the trash can (slow mode); the sticks stop it")]
		internal void PickBall() => DoPick = true;

		internal bool DoPickAll;  // Pick All was clicked; DroneTeleop starts it and clears it

		[Button("Pick All")] [Tooltip("Start from a random heading and pick every ball that is not in the trash can yet, in a random order (slow mode); the sticks stop it")]
		internal void PickAll() => DoPickAll = true;
	}
}
