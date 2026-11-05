# Pick one golf ball -> drop in bin: offline prototype and design (for DroneTeleop.cs)

Everything here was measured in Python MuJoCo 3.16, with the original `MotorThrusts` ported 1:1 (`dronelib.py`,
controller at 60 Hz zero-order hold, physics dt 0.005 s, reference = quintic with feed-forward, slow mode vmax/amax 0.7).
Re-run: `cd sim; ../../../.venv/bin/python pick_proto.py 12 -v` (one scenario/phase log) and `run_batch.py 30 20` (table).
Files: `scene.xml` (robot + ground + ball + bin; `assets` is a symlink to the robot meshes), `dronelib.py` (controller port,
Sim, quintic), `ik.py` (FK, analytic IK, calibration), `pick_proto.py` (state machine = this doc), `run_batch.py`,
`exp_servo.py`, `exp_misc.py` (extra experiments), `scratch/` (debug scripts, ignore).

## Results (Python sim; ball friction / mass randomised, start pose + heading random, measurement noise 2 mm / 1 cm/s)

| set | n | (a) approach: ball moved < 1 cm, no ground touch | (b) grasp detected, ball within 3 cm of grasp_site through lift+carry | (c) ball in bin, bin moved < 1 cm, tilt < 2 deg | lowest point >= rim+10 cm near bin |
|---|---|---|---|---|---|
| nominal: ball x -1.6..-0.5, y +-0.8, friction 0.5-1.5, mass 0.5-1.5x | 30 | 30/30 | 30/30 | 30/30 | 30/30 |
| ring: ball 1.1-1.6 m from the bin on a random side (10/20 needed the high-transit rule) | 20 | 20/20 | 20/20 | 20/20 | 20/20 |
| corners: friction {0.5,1.5} x mass {0.5,1.5}x | 8 | 8/8 | 8/8 | 8/8 | 8/8 |

Typical numbers: ball moved by approach 0.02-0.05 cm (max 0.05); max |ball - grasp_site| during lift/carry 2.2 cm; max tilt 8.3 deg
(from the transit, not the arm); min clearance of lowest point over the rim 14.2 cm; ball ends 0.3-1.6 cm from bin centre line
at release, bin never moved; total time ~44 s (transit 10, deploy 4.5, descend 6.8, close 3.8, lift 4.3, carry 8.1, drop 1.8).

Extra stress (see "Failure modes"): white force noise on the drone 0.5 N rms per 5 ms step: 15/15; 1.0 N: ~6/8; 1.5 N: ~3/8.
Miss tests (ball 3 / 5 cm off across the jaws): both caught by the grasp check, nothing carried.

## Conventions
- MuJoCo frame, z up, ground top z = 0. Drone nose = body -x; heading `m_Yaw`: nose direction in the world = (cos yaw, sin yaw);
  identity quaternion = yaw PI. All as in the C#.
- Arm IK is done in the **drone body frame** (x forward-negative: the arm reaches to body -x). Joint axes: shoulder_pan = body -z
  (vertical), lift/elbow/wrist_flex = body +y (all three in the same plane), wrist_roll = along the forearm, gripper = jaw hinge.
  Pitch rule: finger direction angle from straight-down, toward the nose = 90 deg + deg(lift + elbow + wrist_flex). So
  **finger vertical <=> lift + elbow + wrist_flex = -PI/2**. Pan keeps the finger vertical (pan axis is vertical).
- Fixed_Jaw frame (`mj_xpos/xmat` of body `Fixed_Jaw`): -y = finger axis toward the tip, x = jaw opening direction, z = pad width
  direction. `grasp_site` = (0, -0.103, 0) in it. Pads (box geoms `fixed_jaw_pad_1..4`) cover y in [-0.1064, -0.047], 2 mm thick;
  lowest points of the hand are the pad_1 bottom edge (y = -0.1064) and the Fixed_Jaw collision mesh at about the same height.

## Decisions and the numbers behind them

### 1. Finger vertical, drone high, standoff 0.24 m  (H_PICK = 0.42, STANDOFF = 0.24)
With the finger vertical the wrist limit (-1.66) is the binding constraint: the tool point can only be >= ~0.34 m below the drone
origin (measured reachable band of tool z in drone frame is -0.45..-0.17, x -0.39..+0.1). So the drone hovers at H = 0.42 m
(tool/ball centre 0.0214 m above ground -> tool z_d = -0.40) and **the drone, not the arm, provides the vertical travel**
(arm lift range with vertical finger is only ~10 cm). Margin to joint limits at (H 0.42, D 0.24): >= 0.35 rad on all joints.
Grasp pose at the reference ball height (pan, lift, elbow, wrist) = (-0.233, -1.995, 1.600, -1.176) rad, roll +1.5708.
Pre-grasp pose = the SAME arm pose with the drone 10 cm higher (H 0.52); the descent is a drone move, the arm only trims.

### 2. Drone holds still, arm reaches and trims (closed loop)  -- vs drone fine positioning
Numbers (16 seeds, noise on): hover noise with arm deployed: std 0.8/1.1/0.6 mm (xyz), peak 1.5/2.9/1.5 mm. Drone displacement
caused by slewing the arm stowed -> grasp pose: 1.3-1.6 cm peak, 1-2.5 deg tilt, **independent of slew speed 0.5-2.5 rad/s**
(0.5: 1.4 cm/1.2 deg; 1.0: 1.4/1.5; 2.5: 1.6/2.5). The residual error of the ball in the jaw frame at ground level:
- arm re-solves IK every frame against the *actual* drone pose (what `pick_proto.py` does): x sd 0.9 mm (max 2.2), y sd 0.2, z sd 1.1 (max 4.2).
- arm frozen at the pose solved before the descent, drone alone positions: x sd 3.0 mm (max 6.7), y max 3.3, z max 7.0.
Both worked here (jaw clearance is +-19 mm in x), but the pads are only 8-16 mm wide in jaw z and the ball must end within ~5 mm of
the pad centre in z (see grasp), so the closed-loop trim is 3x better. Decision: drone flies to the hover point and holds;
**every frame the arm joint targets are recomputed (analytic IK) from the ball position expressed in the measured drone frame.**
Use `ArmSpeed` 1.0 rad/s for these motions (no benefit from slower; the 0.8 used in the prototype is also fine).

### 3. Gripper values
Jaw gap vs `gripper` joint (between pad faces near pad_3, 5 cm up from the tip): g = 0.30 -> 4.0 cm, 0.50 -> 5.1, 0.90 -> 7.1 cm,
1.75 -> ~10 cm. Ball is 4.27 cm. **Open = 0.90** (margin ~1.9 cm per side). Ball sits closed at jaw angle **0.40 (0.396-0.402)**, an empty close ends at -0.174.
The pads are tilted (jaw gap is 3 mm narrower toward the palm than at the tip): a closing jaw wedges the ball toward the tip.

**Do not command -0.174 and let the +-3.5 Nm force limit squeeze (kp 50 saturates at 0.07 rad error).** That jammed the ball out of the
wedge: with the saturated squeeze the ball crept out in 5-10 s and dropped in ~40% of runs (also each close pushed the whole hand 9 mm up).
Instead **force-limited close**: every frame command `ctrl = max(-0.174, g_measured - 0.02)` (torque = 50*0.02 = 1 Nm), until
`|dg/dt| < 0.01` for 0.5 s (about 3.8 s total). Then keep commanding `g_measured - 0.02` (do not re-latch a lower value). With that the ball creeps
only ~2-3 mm per 10 s (tested with 12 s extra hold) and survives everything in the table. Ball friction does not matter in MuJoCo
(pad friction 2 and combined by max), mass +-50% does not matter.

### 4. Where the ball must be in the jaw (three small offsets that mattered)
Tool point = point of the Fixed_Jaw frame that we place on the ball centre: **TOOL = (x -0.030, y -0.0857, z -0.006)** m (jaw frame)
plus `TOOL_DZ = +0.003` above the ball centre height:
- x -0.030: ball is centred in the open gap (fixed pad face at +0.008..0.012, moving pad -0.07 at g=0.9). x = -0.0146 (first guess, 3 mm from the fixed
  pad) made the descent touch the ball (0.4 cm, then jammed the drone); -0.030 moved the ball 0.0004 cm.
- y -0.0857 and TOOL_DZ 0.003: pad bottom edge sits 3.5 mm above ground (pad bottom is 0.0207 below the tool point; the contact margin of the jaw
  geoms is 3 mm). TOOL_DZ 0.007 let the close push the hand up and the ball ended at y = -0.100 (6 mm from the pad end) and slid out; 0.003 ends at -0.090. 0.0 or lower starts pressing the
  pads into the ground and fails often. 0.002..0.003 is the window.
- z -0.006: during the close the ball is pushed +4..+9 mm along jaw z (pad width direction); starting 6 mm negative ends it at -0.004..+0.003 (centred on the pads).
  With z = 0 or -0.010 the ball ended off-centre and the 10 s hold test dropped it in 1/12 and 7/12.

### 5. Carry, bin approach, safe height
- After the grasp the arm is frozen (same joint targets); **only the drone moves**. Lift = vertical drone move to `hc = z + (SAFE_LOW + 0.02 - lowest_point_with_ball)`,
  `SAFE_LOW = rim 0.45 + 0.10 + 0.03 = 0.58`. In practice hc = 1.02 m (lowest = pad bottom, ball centre ends at z 0.60).
- Then fly horizontally at hc (heading kept) to `drone_xy = bin_xy - (ball_xy - drone_xy)_measured at grasp time` so the ball is centred over the bin. Ends within 0.5-1.6 cm.
- Release by opening to 0.90 at 1.5 rad/s, wait 1.8 s. The ball falls 0.57 m and lands inside (ends against a wall in this sim; fine).
- Heading rule (important for the bin rule): nose points **from the ball toward the bin** (`yaw = atan2(bin - ball)`): the drone hovers D behind the ball on the
  side AWAY from the bin, the arm reaches toward the bin.
- Transit with stowed arm: if the straight line start -> hover point passes closer than 1.0 m to the bin centre, cruise at origin z >= 0.58 + 0.133 + 0.02 = 0.733
  (stowed arm lowest point is 0.133 below the origin; deployed hand 0.309), as 3 legs: climb, cruise, descend at the hover point. Triggered in 10/20 ring runs, never hit the bin.
- Picking a ball closer than ~0.5 m (hand) / ~0.9 m (drone body radius incl. arm) to the bin violates "lowest >= rim+10 cm within 0.5 m" by construction. Not handled; 1.1 m worked.

### 6. Servo/rate numbers
Descent over the ball: drone reference quintic from H_PICK+0.10 to H_PICK, vmax 0.04 m/s, amax 0.3 (T = 4.7 s). Drone tracking lag 2 mm.
Slow-mode reference for all other drone moves: vmax 0.7, amax 0.7 (min-jerk 5th order: T = max(1.875 D/vmax, sqrt(5.7735 D/amax)), start from the current reference
state so legs can be chained). Yaw is slewed at 1 rad/s and must have converged before DEPLOY.

## State machine (exact conditions)

| phase | action | exit | timeout / failure |
|---|---|---|---|
| TRANSIT | arm stowed, gripper -0.174. Legs: climb(if high), cruise, descend to hover point `hov = ball - 0.24*(cos y, sin y)`, z = 0.52 (y per heading rule) | last leg done, `|pos-hov|<1.5 cm`, `|v|<3 cm/s`, yaw error < 0.02, then 0.5 s | 120 s -> fail |
| DEPLOY | gripper -> 0.90 at 1.5 rad/s; arm joints -> IK of (ball + (0,0,dz)) with dz = 0.52-0.42 (+TOOL_DZ), slew 0.8-1.0 rad/s | all joints within 0.01 rad and gripper at 0.90, >0.5 s; then 0.75 s settle | 12 s; IK unreachable -> fail |
| DESCEND | drone reference z 0.52 -> 0.42 (vmax 0.04). Each frame: tool target = ball + (0,0,(z_ref-0.42)+0.003) -> IK in measured drone frame -> arm targets | trajectory done, + 1 s | ball moved > 1 cm -> abort (not hit in sim) |
| CLOSE | force-limited close (command g_meas-0.02 every frame) | jaw velocity < 0.01 rad/s for 30 frames | 4 s |
| GRASP CHECK | `0.30 < g < 0.50` **and** `|ball - grasp_site| < 3 cm` | -> LIFT | else: open (0.90), go back to DEPLOY (retry up to 2). Not implemented in the proto: it returns fail |
| LIFT | arm frozen; drone vertical to hc (~1.02) | settled (2 cm, 5 cm/s) + 0.33 s | 60 s; ball z < 0.3 -> fail |
| CARRY | fly to over-bin point at hc, same heading | settled (1.5 cm, 3 cm/s) + 0.5 s | 90 s |
| DROP | gripper 0.90, wait 1.8 s | - | - |
| RETREAT | climb +0.15, hold | 1.7 s | - |

Monitor during LIFT/CARRY/DROP-before: if `g` leaves [0.30, 0.50] or |ball - grasp_site| > 3 cm: ball is slipping -> abort (fly back/land; not prototyped).

## IK (code-ready; verified against MuJoCo FK to 1e-6 m)
Constants, drone body frame, all joints 0 (from `ik.analytic_consts()` for roll = +1.5708, tool point above):
`c` (pan axis point) = (-0.1102, 0, 0.0235); `S` (lift pivot) = (-0.1408, 0, -0.0790); `E0 - S` = (-0.11257, 0, 0.0280) (|.| = l1 = 0.116);
`W0 - E0` = (-0.13490, 0, -0.0052) (l2 = 0.135); `w_off` (tool minus wrist pivot with the finger vertical) = (0.0060, 0.0300, -0.1458).
Note the rest vector `T0 - W0` depends on the tool point and the roll sign; in C# use `GetBodyPoseMj` of `Wrist_Pitch_Roll` at joints 0, or just keep these numbers
(they come from the tool point above; roll = +1.5708 puts the tool lateral offset +0.03 on body +y).

    Input: tool = target in the drone frame (R^T (p_world - pos)).      Ry(t) = [[c,0,s],[0,1,0],[-s,0,c]]
    a   = w_off.y;  d = (tool.x - c.x, tool.y - c.y);  px = c.x - sqrt(d.d - a^2)           (nose side is -x)
    pan = atan2(a, px - c.x) - atan2(d.y, d.x)
    W   = (px, tool.z) - (w_off.x, w_off.z)                                                (wrist target, x-z plane)
    v   = W - (S.x, S.z); r = |v|;  cosb = (r^2 - l1^2 - l2^2)/(2 l1 l2);  beta = -acos(cosb)    (elbow branch: sign -1 is the one in limits)
    a1_0 = atan2((E0-S).z, (E0-S).x) ; a2_0 = atan2((W0-E0).z, (W0-E0).x)                   (angles in the x-z plane, atan2(z,x))
    elbow = (a2_0 - a1_0) - beta;  A1 = atan2(v.z, v.x) - atan2(l2 sin beta, l1 + l2 cos beta);  lift = a1_0 - A1
    wrist = -PI/2 - lift - elbow                                                              (finger vertical)
    reject if r > l1+l2 or any joint outside its limit; roll = +1.5708

Solved at the pre-computed grasp pose: tool (-0.24, 0, -0.3957) -> (-0.2332, -1.9950, 1.5998, -1.1756). Reachable band at roll +PI/2: tool z_d between
about -0.45 and -0.34 for x = -0.20..-0.30; outside it the wrist hits +-1.66.

## Failure modes found, and handling
1. **Saturated gripper squeeze** (ctrl -0.174): wedge squeezes the ball out of the jaw tip in seconds. -> force-limited close (above). Biggest finding.
2. **Gripper accidentally opens while carrying**: bug in my first version (arm slew vector included the roll value in the gripper slot). Reminder for C#: in the
   frozen-arm state do not write the gripper slot from any slew loop.
3. **Hand rises 9 mm during close** (wedge reaction) -> ball ends too close to the pad end. -> TOOL_DZ 0.003, tool z -0.006, force-limited close.
4. **Pads inside contact margin of the ground** (<3 mm) push the whole drone up: servo wound up and IK became unreachable. -> keep pad bottom >= 3.5 mm above the ground (TOOL_DZ >= 0.002).
5. **Thin bin floor**: with a 10 mm thick floor the ball falling 0.57 m (3.3 m/s, dt 0.005 s) penetrated to 9 mm and moved the bin 1.1-1.7 cm in 2/12 runs. The final
   `scene.xml` floor is 30 mm thick. **Check the real TrashCan floor collider thickness**; if thin, the "bin moves < 1 cm" metric may fail from ball impact alone (only mitigation
   under the rim+10 cm rule: smaller release height is not possible; maybe slow-mo the ball by releasing with the hand tilted, untested).
6. Finger-vertical reach limit at low H: with H < 0.36 the wrist cannot reach (see 1). Don't lower the drone further.
7. Disturbances: white 1 N rms force noise on the drone loses the ball in ~25% (creep + shake), 1.5 N loses most. Lucky has no turbulence, but real noise in the tracking
   (tilt jitter) shows up the same way. The grip-angle window / ball-vs-site monitor is the guard.
8. Detection false positive (first version `g > -0.07`): a 5 cm miss still read 0.16 (a half-closed jaw). The window [0.30, 0.50] fixes it.

## What was NOT done / honest caveats
- Retry logic for a failed grasp is not implemented in the prototype (it logs `grasp_not_detected` and stops); the doc table gives the intended behaviour.
- The ball is not added to the CoM/mass used by the mixer (46 g = 1.8% of the drone mass; no visible effect).
- Aerodynamics: MuJoCo viscosity only; no rotor downwash/ground effect (real Lucky has none either).
- Lucky's ball friction/restitution/solref, the bin collider thickness and the exact contact margins of the jaws may differ from this model. Most sensitive to those: items 1, 3, 4, 5.
- Control rate emulation: 60 Hz zero-order hold with physics at the model dt (0.005); real frame jitter / dt capping not simulated.
- Heading randomised only for the start; the pick heading is fixed by the rule above (nose from ball to bin). Balls within ~1 m of the bin are not supported.
