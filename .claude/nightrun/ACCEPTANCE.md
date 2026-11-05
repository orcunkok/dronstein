# Acceptance criteria (numbers, not vibes)

## Step 1 — speed modes
Test suite per mode (slow, medium, fast): moves of 1 m, 5 m, 20 m horizontal, 1 m vertical up and down, a 90° yaw-in-place, and a 180° yaw combined with a 5 m move. Hover 4 s after each.
Per move, log: `METRIC mode=<m> move=<name> time=<s> peakSpeed=<m/s> overshoot=<cm> finalErr=<cm> settle=<s> hoverRms=<mm> maxTilt=<deg> yawErr=<deg>`.
- finalErr (after settle) < 2 cm in every mode
- overshoot < 3 cm (along the move direction) in every mode
- hoverRms (position RMS over the last 3 s of hover) < 5 mm
- yawErr < 1°, no yaw overshoot > 3°
- No oscillation: velocity sign of the along-track error changes at most once during settle
- Speeds clearly distinct: slow ≈ 0.5–1 m/s top, medium ≈ 2–3 m/s, fast ≈ 5–8 m/s (lead may adjust); fast tilt ≤ 30°
- Manual stick flight still works and feels the same character in each mode (acceleration scaled)
- Ball Set / B-key arrival uses the same smooth trajectory
- Tuning panel: Slow / Medium / Fast buttons visible (screenshot), current mode visible

## Step 2 — one ball into bin (Slow mode, fixed ball e.g. 14)
Log `METRIC pick ball=N phase=<phase> t=<s> ...` at each phase change and `METRIC pick ball=N result=ok|fail reason=... ` at the end.
- (a) gripper open, fingertips at ground level straddling the ball, ball moved < 1 cm by the approach
- (b) after close+lift, ball z rises with grasp_site (ball within 3 cm of grasp_site for the whole carry)
- (c) ball ends inside the bin footprint and below the rim
- 9/10 successes from `reset` + `pick N`
- Drone never touches the ground; no tilt > 20° during pick; no visible jitter in screenshots

## Step 2.5 — bin-safe transit
- Lowest point of drone + arm + ball ≥ bin rim + 10 cm whenever horizontally within 0.5 m of the bin
- Bin displacement after a run < 1 cm, tilt < 2°

## Step 3 — random balls
- `pickall` (all remaining balls in a random order, random start yaw) ≥ 90% success, 0 bin knockovers, no crashes
- Failed grasps detected and retried (max 2 retries) or skipped with a logged reason
