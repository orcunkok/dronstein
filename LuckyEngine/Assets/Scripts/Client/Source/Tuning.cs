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

		[Slider(1f, 50f)] [Units("m/s")] [Tooltip("Top speed of the drone")]
		public float MaxSpeed = 50f;

		[ReadOnly] [Units("m/s")] [Tooltip("The drone's current speed, written by DroneTeleop")]
		public float Speed;

		[Slider(0, 25)] [Tooltip("Golf ball to highlight, 1 to 25 for GolfBall_01 to GolfBall_25 (0 = none)")]
		public int Ball = 14;

		internal bool GoToBall;  // Set was clicked; DroneTeleop flies there and clears it

		[Slider(0.3f, 1f)] [Tooltip("How far the ball sits from the shoulder-lift pivot when Set arrives, as a fraction of the reach (1 = on the sphere)")]
		public float GraspReach = 0.65f;

		[Slider(10f, 90f)] [Units("deg")] [Tooltip("Angle below level from the shoulder-lift pivot down to the ball when Set arrives (90 = straight over it)")]
		public float GraspAngle = 45f;

		[Button("Set")] [Tooltip("Fly in behind the selected ball and low, so the ball sits inside the reach (GraspReach, GraspAngle)")]
		internal void SetBall() => GoToBall = true;
	}
}
