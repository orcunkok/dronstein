# dronstein
Lucky Robots: a drone with a robotic arm.

A Skydio X2 drone carries an SO-100 arm in Lucky Engine. The goal is to pick up golf balls: the drone flies over to a ball and holds still, like a moving table, then the arm reaches down and grabs it.

## How to run
1. Open the `LuckyEngine` project in Lucky Engine.
2. Press **Play**.
3. Fly with an Xbox controller. A few extra keys are on the keyboard.

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
| F | Camera follows the drone or the gripper |
| W / S | Zoom in / out |
| Right mouse drag | Rotate the camera |
| R | Show / hide the arm's reach (the blue half-sphere) |
| Y | Fly to the next corner of the test square (blue dots on the ground) |

## On the ground
- Green dot: the point right under the drone's center.
- Gripper dot: the point right under the gripper.
- Golf balls and a trash can: the things to pick up and where they go.
