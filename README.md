# dronstein
Lucky Robots: a drone with a robotic arm.

A Skydio X2 drone carries an SO-100 arm in Lucky Engine. The goal is to pick up golf balls: the drone flies over to a ball and holds still, like a moving table, then the arm reaches down and grabs it.

## How to run
1. Open the `LuckyEngine` project in Lucky Engine.
2. Press **Play**.
3. Fly with an Xbox controller. A few extra keys are on the keyboard.
4. To send the drone to a ball, use the Tuning panel (see below).

## Controller
| Button | What it does |
|---|---|
| Left stick | Fly forward/back and left/right |
| Right trigger / left trigger | Up / down |
| LB / RB | Turn left / right |
| Menu | Slow, precise mode on/off |
| View | Unfold / fold the arm |
| Right stick left/right | Turn the arm (or roll the wrist after pressing A) |
| Right stick up/down | Move the chosen joint |
| X / Y / B | Choose the shoulder / elbow / wrist joint |
| A | Switch the right stick between turning the arm and rolling the wrist |
| D-pad up / down | Open / close the gripper |

To change buttons, edit `LuckyEngine/ControllerBindings.txt`, save it, and press Play again.

## Keyboard
| Key | What it does |
|---|---|
| F | Camera follows the drone, the gripper or the selected ball |
| W / S | Zoom in / out |
| Right mouse drag | Rotate the camera |
| T | Show / hide the arm's reach (the blue half-sphere) |
| R | Reset the scene: drone, arm, balls and trash can go back to where they started |
| G | Unfold / fold the arm (same as View on the controller) |
| B, number, Enter | Fly to that ball, e.g. B 1 1 Enter for ball 11 (same as Tuning's Set). A number with no ball is ignored |
| Y | Fly to the next corner of the test square (blue dots on the ground) |

## Tuning panel
Select the **Tuning** entity in the scene. Its Properties panel has live knobs you can change while playing:

| Knob | What it does |
|---|---|
| Show Reach | Show / hide the arm's reach (same as T) |
| Max Speed | Top speed of the drone |
| Speed | The drone's current speed (read only) |
| Ball | Pick a golf ball, 1 to 25. It glows purple. 0 = none |
| **Set** | Fly to the chosen ball. The drone stops right above it, at the height where the bottom of the reach half-sphere touches the ground |

## On screen
Top right: what the camera follows (purple when it's a ball) and the mode: **Manual**, **Calibration** (yellow) while flying the test square, or **ToBall 11** (purple, with the ball number) while going to a ball. Moving the sticks switches back to Manual.

## On the ground
- Green dot: the point right under the drone's center.
- Red dot: the point right under the gripper.
- Checkered ground: each tile is 1 m.
- Golf balls and a trash can: the things to pick up and where they go.

## Latest changes
- **Trash can can be moved:** it's now a free, light (~1.1 kg) aluminium can you can push with the drone or drag with the mouse (Force Tool).
- **R resets the scene** (drone, arm, balls, trash can back to the start). The reach half-sphere moved to **T**.
- **B key:** type B, a ball number and Enter to fly to that ball. The screen shows ToBall and the ball number.
- **G key:** unfolds / folds the arm. Unfolded, the upper arm leans slightly back and the forearm reaches forward almost level (about 12° below) instead of hanging straight down.
- **Fly to a ball:** pick a ball in Tuning and press **Set**. The drone flies over it and lowers until its reach just touches the ground.
- **Ball selection:** the chosen ball glows purple, and F can point the camera at it.
- **Tuning panel:** live knobs for reach, speed and ball while playing.
- **Screen labels:** camera and mode shown at the top right.
- **New look:** checkered 1 m ground tiles and higher sunlight.
- **Controller:** reconnects on its own if unplugged.
