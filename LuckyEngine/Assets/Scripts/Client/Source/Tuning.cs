using Hazel;

namespace Dronstein
{
	/// <summary>
	/// Live knobs for development: select the entity tagged "Tuning" and drag them in the Properties panel while playing.
	/// To tune something new, add a public field here and read it from the other script.
	/// </summary>
	public class Tuning : Entity
	{
		[ToggleButton("Visible", "Hidden")] [Tooltip("Wire half-sphere of the gripper's reach (R toggles)")]
		public bool ShowReach = true;

		[Slider(1f, 50f)] [Units("m/s")] [Tooltip("Top speed of the drone")]
		public float MaxSpeed = 50f;

		[ReadOnly] [Units("m/s")] [Tooltip("The drone's current speed, written by DroneTeleop")]
		public float Speed;

		[Slider(0, 25)] [Tooltip("Golf ball to highlight, 1 to 25 for GolfBall_01 to GolfBall_25 (0 = none)")]
		public int Ball = 4;

		[Tooltip("Glowing material the selected ball switches to")]
		public Material? SelectedBallMaterial;
	}
}
