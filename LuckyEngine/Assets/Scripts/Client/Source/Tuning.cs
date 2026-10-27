using System.Globalization;
using System.IO;
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

		[Group("Speed")] [Dropdown("Slow", "Medium", "Fast")] [Tooltip("Speed mode, kept between runs (saved in SpeedMode.txt)")]
		public int SpeedMode = 2;

		[Group("Speed")] [ReadOnly] [Units("m/s")] [Tooltip("Top speed of the current mode, written by DroneTeleop")]
		public float TopSpeed;

		[HideFromEditor]
		public float Speed;  // the drone's current speed, written by DroneTeleop

		[Slider(0.3f, 1f)] [Tooltip("How far the ball sits from the shoulder-lift pivot when Navigate to ball arrives, as a fraction of the reach (1 = on the sphere)")]
		public float GraspReach = 1f;

		[Slider(10f, 90f)] [Units("deg")] [Tooltip("Angle below level from the shoulder-lift pivot down to the ball when Navigate to ball arrives (90 = straight over it)")]
		public float GraspAngle = 90f;

		[Slider(0, 25)] [Tooltip("Golf ball to highlight, 1 to 25 for GolfBall_01 to GolfBall_25 (0 = none)")]
		public int Ball = 9;

		[DisplayName("Select Mode")] [Dropdown("Navigate to ball", "Pick selected ball", "Pick all balls")]
		[Tooltip("What Autonomous Mode Action runs. Navigate to ball: fly in behind the selected ball and low, so it sits inside the reach (GraspReach, GraspAngle). " +
			"Pick selected ball: pick it and drop it in the trash can (slow mode). Pick all balls: from a random heading, pick every ball not in the can yet, in a random order (slow mode)")]
		public int AutoMode = 1;

		[DisplayName("Autonomous Mode Action")] [ToggleButton("Stop Autonomous Action", "Start Autonomous Action")]
		[Tooltip("Starts the selected mode; Stop (or the sticks) gives the drone back to manual. Switches back to Start when the action is over")]
		public bool AutoOn = true;  // DroneTeleop starts AutoMode when it turns on, stops it when it is turned off, and turns it off when the action ends

		internal bool GoToBall;  // B + ball number + Enter, or Navigate to ball; DroneTeleop flies there and clears it

		string m_SpeedModePath = "";
		int m_SavedSpeedMode;

		// Loads the saved speed mode, so Play starts in the mode that was last picked.
		protected override void OnCreate()
		{
			m_SpeedModePath = Path.GetFullPath(Path.Combine(Application.DataDirectoryPath!, "..", "SpeedMode.txt"));
			if (File.Exists(m_SpeedModePath) && int.TryParse(File.ReadAllText(m_SpeedModePath).Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int mode) && mode is >= 0 and <= 2)
				SpeedMode = mode;
			m_SavedSpeedMode = SpeedMode;
		}

		// Saves the speed mode whenever it changes, from the dropdown or the mode command.
		protected override void OnUpdate(float dt)
		{
			if (SpeedMode == m_SavedSpeedMode) return;
			m_SavedSpeedMode = SpeedMode;
			File.WriteAllText(m_SpeedModePath, SpeedMode.ToString(CultureInfo.InvariantCulture));
		}
	}
}
