# Grasp findings: making the gripper close faster

Tests run 2026-10-28 in Lucky, on the Pick All flow (`DroneTeleopPick.cs`). The goal was a close of about 1.5 s with every grasp holding. We got it down from 4.0 s to about 2.1 s with every grasp holding. Anything faster dropped balls in a real Pick All.

## What we found

**The close was slow because the jaw crept the whole way.** The close pushes the jaw with a light, fixed force (the command leads the measured jaw angle by 0.02 rad, about 1 N·m). At that force the jaw moves at only about 0.15 rad/s. From fully open (0.90 rad) it reached the ball (about 0.45 rad) after about 3 s, then waited about 0.8 s for the jaw to be still. Opening width was never the main cost; the slow travel was.

**The jaw has to meet the ball gently, and that's the whole story.** Every failure looked the same: the ball ended up a few mm toward the end of the pads. In the jaw frame, `ballInJaw` y is about −0.0867 for a good seat. Balls seated beyond about −0.0905 slipped out during the carry, and beyond −0.093 the grasp check rejected them. What pushes the ball out is the jaw arriving too fast or too hard:

- full torque down to 0.55 rad, then gentle: the jaw still hit the ball hard, and balls were seated out;
- opening only to 0.65: the jaw touched the ball during the descent and pushed it about 2 cm;
- a stronger squeeze (lead 0.04, about 2 N·m) together with a narrower opening (0.70): the jaw reached the ball in 0.8 s, and both grasps failed;
- pre-closing the jaw during the descent's bottom wait: every grasp failed, with or without the arm still tracking.

**Speed is free before the last ~0.2 rad.** Closing at full torque from 0.90 down to 0.65 rad, then switching to the gentle push, seated every ball perfectly. So the fast part of the close is safe as long as the jaw has slowed down well before the ball.

**Each speed-up works alone, but they don't add up.** In the screening, a 2× squeeze, a narrower opening (0.70), the fast part to 0.65, and a shorter stop check (15 still frames instead of 30) each kept perfect grasps. Combining a 2× squeeze with the narrower opening failed.

**The faster settings looked fine in isolated tests and failed in real use.** With a reset before every pick, a 1.5× or 2× squeeze passed all 16 trials and closed in 1.2–1.9 s. In a continuous Pick All the same settings dropped 2 of 6 balls (balls 13 and 9, every time). With a stronger squeeze the close falls into one of two outcomes: either the jaw meets the ball early (about 1.0–1.2 s) and seats it 3–5 mm out, or it meets it later (about 1.6 s) and seats it perfectly. Only the gentle 1 N·m push lands in the good outcome every time. So reset-based tests are good for screening, but a setting has to pass a real Pick All before it becomes the default.

**The stop check can be shorter.** Ending the close after 15 still frames instead of 30 saves 0.3 s and made no grasp worse. 10 frames looked fine too, but it was only tested together with the stronger squeezes, so it isn't proven on its own.

**Wrist roll 90° (jaws open sideways) is right; 0° doesn't work as-is.** With the jaws front/back, no pick succeeded: the ball was pushed about 1.5 cm and several trials timed out. The arm's IK constants were measured at 90°. Rotating the tool offset for 0° still leaves the ball 4 mm off in the jaws before the close even starts. Making 0° work would need its own measurement and tuning.

**Result.** The default close is now: full torque to 0.65 rad, then the 1 N·m push, ended after 15 still frames. That takes 2.1–2.4 s instead of 4.0 s, and in a real Pick All 6 of 6 balls went in the can, all seated perfectly.

**To go faster than ~2 s,** the slow part would have to start based on where the ball actually is between the jaws, instead of at a fixed angle. Other options are a different jaw approach, or pads that tolerate a ball seated further out.

## How the tests were run

- `gp open|lead|roll|fast|stall <value>` (in `Cache/nightrun_cmd.txt`) changes the grasp settings live, so one Play session runs many trials without rebuilding.
- Each close logs `METRIC close` with its time, when and at what jaw angle the jaw stopped on the ball, and the settings. `METRIC pick ball= afterClose` gives `ballInJaw`.
- Screening and confirmation trials: `reset`, set the knobs, then one full pick (`pick N`) with carry and drop. The result is whether the ball ended in the can.
- Real-use checks: `pickall 7` (seed 7, balls 12, 11, 13, 9, 4, 19 first), no resets in between.

Settings: **lead** is how far the close command leads the jaw angle, in rad (0.02 = about 1 N·m, 0.04 = about 2 N·m). **open** is the open jaw angle, in rad. **fast** is the angle the full-torque part of the close goes down to (off = gentle the whole way). **stall** is the number of still frames that end the close. **roll** is the wrist roll.

## Test results

### Early tries (Pick All, seed 7)

| Change from the original | Close time | Balls in the can | Ball seat (`ballInJaw` y) |
|---|---|---|---|
| none (lead 0.02, open 0.90, gentle all the way, stall 30) | ~4.0 s | 4/4 | −0.0866 |
| open 0.65 | ~4 s | 0/3 | pushed ~1.9 cm, then `ball_moved_by_approach` |
| full torque to 0.55, then gentle, stall 15 | ~1.4 s | 1/3 | −0.090 to −0.096, slips |
| same, stall 30 | ~1.7 s | 2/4 | −0.090 to −0.096, slips |
| same, descend from 10 cm instead of 5 cm | ~1.7 s | 0/4 | −0.0964, grasp not detected |
| full torque to 0.70, then gentle, stall 30 | 2.7 s | 4/4 | −0.0866 |
| pre-close to 0.55 during the descent's bottom wait | 1.7–2.0 s | 0/6 | −0.0965, grasp not detected |
| pre-close to 0.65, arm frozen during the wait | 2.1–3.2 s | 0/4 | −0.091 to −0.097 |

### Screening: 2^(5−2) fractional factorial

Five factors at two levels, 8 runs plus the original as run 0, balls 12 and 13, reset before each pick. Fast and stall were set from the others (D = AB, E = AC), so main effects are aliased with two-factor interactions: good for screening, not for fine tuning.

| Run | Lead | Open | Roll | Fast to | Stall | Close time | Balls in the can | Ball seat |
|---|---|---|---|---|---|---|---|---|
| 0 | 0.02 | 0.90 | 90° | off | 30 | 3.9 s | 1/1 (one result line not captured) | −0.0866 |
| 1 | 0.02 | 0.90 | 90° | 0.65 | 15 | 2.1 s | 2/2 | −0.0868 |
| 2 | 0.04 | 0.90 | 90° | off | 30 | 2.4 s | 2/2 | −0.0867 |
| 3 | 0.02 | 0.70 | 90° | off | 15 | 2.3 s | 2/2 | −0.0868 |
| 4 | 0.04 | 0.70 | 90° | 0.65 (no effect) | 30 | 1.4–2.4 s | 0/2 | −0.0909, −0.0976 |
| 5–8 | mixed | mixed | **0°** | mixed | mixed | — | 0/8 | ball pushed ~1.5 cm, timeouts |

### Confirmation (open 0.90, roll 90°, fast to 0.65; balls 12, 13, 11, 9; reset before each)

| Candidate | Lead | Stall | Close time | Balls in the can | Ball seat |
|---|---|---|---|---|---|
| K1 | 0.03 | 15 | 1.9–2.0 s | 4/4 | −0.0868 |
| K2 | 0.04 | 15 | 1.3 s (3), 1.9 s (1) | 4/4 | −0.0868 to −0.0898 |
| K3 | 0.04 | 10 | 1.22 s | 4/4 | −0.0893 to −0.0901 |
| K4 | 0.035 | 10 | 1.8–1.9 s (3), 1.3 s (1) | 4/4 | −0.0869 to −0.0893 |

### Real use (Pick All seed 7, first 6 balls, no resets; open 0.90, roll 90°, fast to 0.65)

| Lead | Stall | Close time | Balls in the can | Failures |
|---|---|---|---|---|
| 0.04 | 10 | 1.2–1.9 s | 4/6 | 13 and 9 slipped in the carry (seat −0.0923, −0.0915) |
| 0.03 | 10 | 1.4–1.9 s | 4/6 | 13 and 9 slipped in the carry (seat −0.0907, −0.0910) |
| **0.02 (default)** | **15** | **2.1–2.4 s** | **6/6** | none, every ball seated at −0.0868 |
