# Autonomy: Pick All balls

Single source of truth for how the drone's Pick All action works. Behaviour is described at a high level; the code has the detail (see the [code map](#code-map) at the bottom).

Last checked against the code: 2026-10-27.

## 1. What it does

The drone flies to a random start point with a random heading. Then it picks up every golf ball not yet in the trash can, one at a time, in random order, and drops each into the can. Between balls it flies straight from the can to the next ball. It stops when the queue is empty, when the user takes over, or after a 1 hour timeout. A full run takes about 40 s per ball.

Each ball is one **pick**: a state machine that grabs, lifts, carries and drops one ball. Pick All is a queue around it.

## 2. Flow

```
  Start (F key or Tuning switch)
        |
        v
  Go to start: fly to a random point and heading, wait 1 s still
        |
        v
  Build queue: every ball 1..25 not in the can, shuffled (seeded)
        |
        v
  +-->  Queue empty? --yes--> Done (summary line)
  |          | no
  |          v
  |     Pick the head ball (section 3)
  |          |
  |          v
  |     In the can?--------------------+
  |        | yes                       | no
  |        v                           v
  |     count ok              on the floor and first failure?
  |        |                    | yes                | no
  |        |                    v                    v
  |        |              requeue at the end     skip, log reason
  |        |                    |                    |
  +--------+--------------------+--------------------+
```

Stopped at any point by: the sticks, the Stop switch, the 1 hour timeout, or a `sticks` end inside a pick.

## 3. One pick, phase by phase

The drone hovers 24 cm behind the ball, nose toward the can. The arm reaches the ball, and every frame its joints come from an inverse-kinematics solve of the ball's measured position, so small hover errors are absorbed by the arm.

Times are the wait or limit in the code. "Flight" means the time the drone takes to get there, which varies.

| Phase | Drone | Gripper and arm | Moves on when | Time | Limit |
|---|---|---|---|---|---|
| **Transit** | Flies to the hover point in 3 legs (climb, cruise, descend). Cruises high if the line passes near the can. | Arm folds to stowed. Gripper closes. | Arrived, heading turned, still for 0.5 s. Re-plans if the ball rolled (up to 4 times). | Flight + 0.5 s | 120 s |
| **Deploy** | Holds. | Gripper opens; arm moves to the pre-grasp pose. | Open and arrived, then 0.75 s settle. | Arm move + 0.75 s | 12 s |
| **Descend** | Descends 10 cm at 4 cm/s to fingertip-at-ball height. | Arm tracks the ball. | Reached the height, then 1 s hold. Fails if the ball moved more than 1 cm. | ~3.5 s | none |
| **Close** | Holds. | Closes with a light, fixed squeeze (about 1 N·m). | Gripper stopped, or 4 s passed. | up to 4 s | 4 s |
| **Check** | Holds. | Reads jaw angle, grip distance, ball position between the jaws. | Ball in the jaws: **Lift**. Not: **Retry** (2 times max), then fail. | instant | none |
| **Retry** | Climbs back to pre-grasp height, moves behind the ball's new spot. | Opens, then re-deploys. | Back at the pre-grasp pose. | climb + settle | counted |
| **Lift** | Climbs to carry height, above the can rim with margin. | Holds the ball lightly. | Settled for 0.33 s. | climb + 0.33 s | 60 s |
| **Carry** | Flies to above the can, keeping the ball's offset. | Holds the ball. | Settled over the can for 0.5 s. | flight + 0.5 s | 90 s |
| **Drop** | Holds. | Opens. | Fixed time. | 1.8 s | none |
| **Retreat** | Climbs 15 cm. | Stays open. | Fixed time. | 1.7 s | none |
| **Settle** | Holds. | — | Fixed time, then the pick is marked ok. | 2 s | none |

The whole pick has a 240 s limit.

The fixed waits add up to about 8.6 s per ball (0.5 + 0.75 + 1 + 0.33 + 0.5 + 1.8 + 1.7 + 2). The rest of the 40 s per ball is flight and the slow descent.

Failures inside a pick end it with a reason. The gripper opens if it was carrying the ball.

## 4. Pick All: the queue

The Pick All driver has three states:

1. **Start**: fly to the random start point and heading (set by the seed). Wait until still and not turning for 1 s.
2. **Next**: take the head of the queue. If it is empty, finish with the summary. Otherwise start one pick.
3. **Picking**: wait for the pick to end, then record the result. A `sticks` end ends Pick All. Otherwise return to Next.

Queue rules:

- Built once at the start from live ball positions. Balls already in the can are left out.
- Shuffled with the seed, so a run can be repeated.
- A ball that fails is requeued once, and only if it is still on the floor (height above 0 and under 6 cm). A second failure skips it and logs why.
- Between balls, each pick plans its own route from the drone's current position.

## 5. How to start and stop

| Action | Effect |
|---|---|
| **F** key, or Tuning → Select Mode = Pick all balls + Start | Starts Pick All. |
| Tuning → Stop Autonomous Action, or moving the sticks | Stops and returns to manual. |
| Run ends by itself | The Tuning switch goes back to Start. |
| Changing Select Mode while running | Stops and restarts in the new mode. |

The Tuning fields `AutoMode` (Select Mode) and `AutoOn` (the switch) are the only state. Keys and the panel both write them, and `AutoUpdate` in `DroneTeleop.cs` acts on the change.

## 6. Limitations and considerations

Read this before changing the logic.

- **Perception is simulated.** Ball and can positions come straight from the simulator's body positions. There is no camera or detector. Real hardware will need a perception step, and the rest of the logic already works from positions, so only the source changes.
- **Grasp success uses sim-tuned thresholds.** Jaw angle, grip distance and the ball's position between the jaws were tuned in simulation (`.claude/nightrun/sim/PICK_DESIGN.md`). Real grippers need their own checks.
- **Fixed waits.** The drop, retreat and settle waits are fixed times. Condition-based ends would be faster and more robust.
- **No overlap between picks.** The next transit starts only after the retreat has finished.
- **Random order.** Balls are picked in random order, so transit is not optimised. Nearest-neighbour would shorten it. The seed keeps runs reproducible.
- **Simple recovery.** A failed ball is retried once, then skipped. The drone never repositions the ball or tries a different one first.
- **Keep-out limits.** The bin keep-out (`DroneTeleopKeepOut.cs`) applies to every flight the script plans, so it covers Pick All and single picks. It does not guard the sticks.
- **Timeouts are spread out.** Phase limits live in `DroneTeleopPick.cs`. Named constants at the top would make tuning easier.
- **Results go only to the log.** There is no return value for callers.

## 6a. Debugging

Grep `/tmp/LuckyEngine/Logs/APP.log` for:

- `METRIC pickall seed=` — the start of a run, with the seed and queue order.
- `METRIC pickall ball=` — each ball's result, with `inBin` and `reason`.
- `METRIC pickall seed= ok=` — the run summary.
- `METRIC pick ball= phase=` — each phase change in a pick.
- `METRIC pick ball= result=` — each pick's end and reason.
- `METRIC keepout clamp` / `METRIC keepout plan` — the keep-out changed a goal or split a route.

## 7. Code map

| File | Responsibility |
|---|---|
| `Client/Source/Tuning.cs` | Tuning panel fields: Select Mode, the Start/Stop switch, Ball selection. |
| `Client/Source/DroneTeleop.cs` | Keys, the start/stop logic (`AutoUpdate`, `AutoStart`, `AutoStop`), the HUD. |
| `Client/Source/DroneTeleopPickAll.cs` | Pick All: the queue, the start point, the summary, the driver. |
| `Client/Source/DroneTeleopPick.cs` | One pick: the phase state machine and the arm IK. |
| `Client/Source/DroneTeleopKeepOut.cs` | The bin keep-out that splits and lifts flights near the can. |

Design notes and offline experiments: `.claude/nightrun/sim/PICK_DESIGN.md`.
