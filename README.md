# dronstein
Lucky Robots: a drone with a robotic arm.

A Skydio X2 drone carries an SO-100 arm in Lucky Engine. The goal is to pick up golf balls: the drone flies over to a ball and holds still, like a moving table, then the arm reaches down and grabs it.

## How to run
1. Open the `LuckyEngine` project in Lucky Engine.
2. Press **Play**.
3. Fly with an Xbox controller. A few extra keys are on the keyboard.
4. To send the drone to a ball or let it pick balls by itself, use the Tuning panel (see below).

## Controller
| Button | What it does |
|---|---|
| Left stick | Fly forward/back and left/right |
| Right trigger / left trigger | Up / down |
| LB / RB | Turn left / right |
| Menu | Slow, precise mode on/off |
| View | Unfold / fold the arm |
| Right stick left/right | Turn the arm |
| Right stick up/down | Move the chosen joint |
| X / Y / B | Choose the shoulder / elbow / wrist joint |
| D-pad up / down | Open / close the gripper |
| D-pad left / right | Roll the wrist |

To change buttons, edit `LuckyEngine/ControllerBindings.txt`, save it, and press Play again.

## Keyboard
| Key | What it does |
|---|---|
| C | Camera follows the drone, the gripper or the selected ball |
| Q / E | Zoom in / out |
| Right mouse drag | Rotate the camera |
| T | Show / hide the arm's reach (the blue half-sphere) |
| R | Reset the scene: drone, arm, balls and trash can go back to where they started |
| G | Unfold / fold the arm (same as View on the controller) |
| B, number, Enter | Fly to that ball, e.g. B 1 1 Enter for ball 11 (same as Tuning's Navigate to ball). The keypad digits and keypad Enter work too. A number with no ball is ignored |
| Y | Fly to the next corner of the test square (blue dots on the ground) |
| F | Start / stop the Autonomous Mode Action. Starting it sets Select Mode to Pick all balls |

## Tuning panel
Select the **Tuning** entity in the scene. Its Properties panel has live knobs you can change while playing:

| Knob | What it does |
|---|---|
| **Show Reach** | Show / hide the arm's reach (same as T) |
| Speed Mode | Slow, Medium or Fast: sets top speed, acceleration, turn rate and how the sticks feel (about 0.8 / 2.5 / 6 m/s). Kept between runs (saved in `LuckyEngine/SpeedMode.txt`). Flights (Navigate to ball, B, Y) follow a smooth path that does not overshoot |
| Top Speed | Top speed of the current mode (read only) |
| Grasp Reach | How far the ball sits from the arm's shoulder when Navigate to ball arrives, as a fraction of the reach (1 = on the half-sphere) |
| Grasp Angle | How far below level the ball sits from the shoulder when Navigate to ball arrives (90° = straight under it) |
| Ball | Pick a golf ball, 1 to 25. It glows purple. 0 = none |
| Select Mode | What the button below runs: **Navigate to ball**, **Pick selected ball** or **Pick all balls** (see below) |
| **Autonomous Mode Action** | **Start Autonomous Action** runs the selected mode and the button turns into **Stop Autonomous Action**. Stop (or moving a stick) gives the drone back to manual. It switches back to Start by itself when the action is over |

The modes:
- **Navigate to ball:** fly in behind the chosen ball and low, so the ball sits inside the reach (Grasp Reach, Grasp Angle).
- **Pick selected ball:** the drone picks the chosen ball by itself and drops it in the trash can (switches to Slow).
- **Pick all balls:** starts from a random point and heading, then picks every ball that is not in the trash can yet, in a random order (Slow).

## On screen
Top right, three rows:
- **Camera:** what the camera follows (purple when it's a ball).
- **Mode:** **Manual**, **Calibration 1/4** (yellow) while flying the test square, **ToBall 11** (purple, with the ball number) while going to a ball, **Pick 14** (purple, with the ball number) during an autonomous pick, or **Pick All** while Pick all balls flies to its start point. Moving the sticks switches back to Manual.
- **Task:** the step the pick is on, `-` when there is none.

Tasks of a pick, in order:

| Task | What the drone does |
|---|---|
| Go to start | Pick all balls only: flying to its random start point and heading, before the first ball |
| Transit | Flies to a point behind the ball (high over the trash can if the way passes near it), turning its nose toward the can, arm folded; the arm unfolds during the last metre |
| Deploy | Holds still and corrects the unfolded arm to the ball (a small step) |
| Descend | Slowly lowers until the open jaws are around the ball |
| Close | Closes the gripper gently (force limited) |
| Check | Checks that the ball is held. If not, Retry |
| Retry | Opens, climbs a little, moves behind the ball's new place and deploys again (up to 2 retries) |
| Lift | Climbs until the ball is above the trash can's rim |
| Carry | Flies the ball over the trash can |
| Drop | Opens the gripper; the ball falls in |
| Retreat | Climbs 15 cm away from the can |
| Settle | Waits a moment for the ball to settle, then the pick is done |
| Next ball | Pick all balls only: between two picks |
| Hold | Dev only (`pick N b`): holds the lifted ball for 10 s instead of carrying it |

## On the ground
- Green dot: the point right under the drone's center.
- Red dot: the point right under the gripper.
- Checkered ground: each tile is 1 m.
- Golf balls and a trash can: the things to pick up and where they go.

## Latest changes
- **Autonomous Mode Action:** the Navigate to ball (was Set), Pick and Pick All buttons in Tuning are replaced by a **Select Mode** dropdown and one **Start / Stop Autonomous Action** switch. Stop gives the drone back to manual.
- **Task on screen:** a third row at the top right shows the pick's step (Transit, Descend, Lift…); the Mode row shows only **Pick** and the ball number. Pick all balls now shows **Pick All** / **Go to start** while it flies to its start point, instead of Manual.
- **Speed Mode dropdown:** the Slow / Medium / Fast buttons are now a dropdown in Tuning, kept between runs in `LuckyEngine/SpeedMode.txt`.
- **Pick All:** the **Pick All** button in Tuning (Slow mode) sends the drone to a random point with a random heading, then picks every ball that is not in the trash can yet, in a random order, one after the other, straight from the can to the next ball. Each pick plans from the ball's live position (it re-plans if the ball rolled), folds the arm and turns to the new heading in transit, and unfolds it during the last metre. A failed pick is retried twice, then put back in the queue once if the ball is on the ground, then skipped. About 40 s per ball. (Dev: `pickall [seed]`; the end line is `METRIC pickall seed= ok=N/M fail=[...] binMoved= binHits= time=`.)
- **Bin keep-out:** every flight the script plans (goto, Set, B, Y, the pick) keeps the drone, its arm and a carried ball at least 10 cm above the trash can's rim whenever any part of it is within 0.5 m of the can's edge. A flight that would cross the can low is split into climb, high cruise and descent; a goal next to the can is raised (logged as `METRIC keepout clamp`). Dev: `test bin` runs the adversarial flights; every flight logs `METRIC flight minClear=... nearBin=... binMoved=...`. Sticks are not guarded.
- **Autonomous pick:** the **Pick** button in Tuning picks the selected ball and drops it in the trash can: the drone flies behind the ball, unfolds the arm, lowers over the ball, closes the gripper gently, climbs above the can's rim, carries the ball over the can and lets go (about 35 s in Slow). A missed grasp is retried up to 2 times. (Dev: `pick N` in `Cache/nightrun_cmd.txt`, `pick N a` stops after lowering over the ball, `pick N b` after lifting it; results are `METRIC pick ...` lines in the log.)
- **Speed modes:** Slow / Medium / Fast buttons in Tuning. Flights to a ball or corner now follow a smooth, planned path (no overshoot) at the speed of the mode; the sticks scale with the mode too. (Dev: `Cache/nightrun_cmd.txt` takes test commands, e.g. `test flight`.)
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
