# Night run brief (dronstein) — read fully before doing anything

Lead: Opus (tech lead). Workers: Sonnet agents, one at a time on the simulator.
Repo: `/home/orcun/dev/dronstein`, branch `own-reach-grab`. Lucky Engine project: `LuckyEngine/`.
Also read: `AGENTS.md` (repo root), `README.md`, `.claude/nightrun/ACCEPTANCE.md`, `.claude/nightrun/PROGRESS.md`.

## Goal of the night (lead decides which step you work on)
1. Speed modes Slow/Medium/Fast (Tuning panel buttons), equally good quality at all speeds.
2. Autonomous pick of ONE fixed ball → drop in TrashCan, in Slow mode. Sub-steps: (a) gripper opened and lowered to the ground straddling the ball, (b) close + lift ball, (c) carry + drop into bin.
2.5. Bin-safe transit: drone + arm (+ carried ball) always above the bin rim when near/over it; never knock the bin.
3. Any/random ball 1..25, any start heading.

## Hard rules
- **No git at all.** The lead commits and pushes. Don't run git add/commit/stash/checkout/branch/reset.
- Don't touch anything outside `/home/orcun/dev/dronstein` except reading Lucky logs. No drivers, packages, system config.
- Don't hand-edit `.hscene`, `AssetRegistry.hzr`, `.hmaterial`. Use the Lucky MCP tools (see AGENTS.md for saving scenes).
- **Anti tail-chasing:** if 3 consecutive attempts don't improve your metric, STOP and report back to the lead with numbers and your hypothesis. Don't stack hacks. Don't randomly twiddle gains — reason from the physics/logs.
- Match code style of `DroneTeleop.cs`: short comments explaining why, terse naming (`m_` fields), no allocation per frame.
- Append a short entry to `.claude/nightrun/PROGRESS.md` when you finish (what you did, numbers, open issues).
- Final report to the lead: files changed, metric table, known issues. Be honest: say what failed.

## Lucky Engine workflow
- MCP tools `mcp__lucky-engine__*` (load schemas with ToolSearch, e.g. `select:mcp__lucky-engine__build_script_assembly,mcp__lucky-engine__harness_enter_play_mode`).
- Edit `LuckyEngine/Assets/Scripts/Client/Source/*.cs` → `build_script_assembly` (must be 0 errors) → `harness_exit_play_mode` / `harness_enter_play_mode` to reload.
- Logs: `/tmp/LuckyEngine/Logs/APP.log` (script `Log.Info` lines land here). Use `grep`/`tail`, never dump the whole file.
- Screenshots: `capture_viewport_screenshot` (works in Play). Use it at key moments to catch jank the numbers miss.
- `harness_get_scene_state` output is huge (67k chars) — avoid it, or grep the saved file.
- **If Lucky crashes / MCP stops answering:** relaunch with
  `cd ~ && MESA_VK_WSI_PRESENT_MODE=mailbox nohup /home/orcun/Desktop/LuckyEngine-2026.3-Linux-x64.AppImage --server >/home/orcun/dev/dronstein/LuckyEngine/Cache/lucky_stdout.log 2>&1 &`
  (the `mailbox` env var is REQUIRED: the desktop is locked overnight and the default FIFO present times out → Vulkan crash `AcquireNextImage2KHR ... result = 2`. Don't use XWayland, it hangs.)
  Run the launch as its own Bash call (never put `pkill -f Lucky...` in the same command — it kills your own shell). Kill only Lucky PIDs from `pgrep -f "LuckyEngine --server"`.
  Poll `curl -s -o /dev/null -w "%{http_code}" http://127.0.0.1:19331/mcp` until it's not 000, then open the project (`harness_open_project` with `/home/orcun/dev/dronstein/LuckyEngine/dronstein.hproj`, scene `Scenes/Main.hscene`), continue. Never "fix" the environment.
- Simulation runs in real time in Play; a 20 m flight takes real seconds. Plan test durations; poll the log with `sleep` in a Bash loop (e.g. `for i in $(seq 60); do grep -q 'METRIC done' ... && break; sleep 2; done`).

## Driving tests without clicking (command file)
Agents can't click inspector buttons. Use (or build, if not yet there) the **command file**: DroneTeleop polls
`LuckyEngine/Cache/nightrun_cmd.txt` (Cache/ is gitignored) ~4×/s; when it's non-empty it executes each line and truncates the file. Commands (extend as needed, keep them documented here):
- `mode slow|medium|fast` — speed mode (same as the Tuning buttons)
- `reset` — same as R
- `goto x y z [yaw_deg]` — fly to a MuJoCo-frame point
- `ball N` / `set` — select ball N / fly to it (same as Tuning Set)
- `test flight` — run the flight test suite, logs `METRIC ...` lines, ends with `METRIC done`
- `pick N [a|b]` — autonomous pick of ball N and drop in the can (a: stop after lowering over it, b: after lifting it); `test bin` — bin keep-out suite
- dev: `label X` (name for METRIC flight lines), `rawgoto x y z` (no keep-out), `arm deploy|stow`, `grip v`, `tool x y z [dz]` (ball centre in the jaw frame), `view drone|gripper|ball [dist]`, `binc`, `log`, `geoms substr`
- `pickall [seed]` — pick every ball not in the bin, seeded random order, random start heading; ends with `METRIC pickall seed= ok=N/M fail=[...] binMoved= binHits= time=` and `METRIC done pickall` (do `reset` first for a full 25)
Path from script: `Path.Combine(Application.DataDirectoryPath!, "..", "Cache", "nightrun_cmd.txt")` (same base as ControllerBindings.txt). Verify the resolved path in a log line once.
Every command logs `CMD <line>` when executed. Every test result line starts with `METRIC` so it's greppable.

## Project facts
- Control code: `DroneTeleop.cs` (target point → cascaded PID → 4 thrusts; arm via position actuators `m_ArmTarget`). Knobs: `Tuning.cs`.
- MuJoCo frame z up; Lucky y up: MuJoCo (x,y,z) = Lucky (x,-z,y). Ground top z=0.
- Drone heading `m_Yaw`; nose is body −x. Drone + arm mass ~ printed at start in log (`DroneTeleop: start ...`).
- Golf balls: tags `GolfBall_01..25`, diameter 0.0427 m, MuJoCo body `lucky_body_<entityID>`. TrashCan: tag `TrashCan`, free body ~1.1 kg, floor 0.33×0.33 m, walls ~0.45 m tall (verify from live body/geom data, don't trust this).
- Arm: SO-100, joints shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper (range −0.174 closed … 1.75 open). Position actuators kp=50, forcerange ±3.5. `grasp_site` on the fixed jaw (fingertip). Jaw geoms friction 2, condim 6. See `LuckyEngine/Assets/Robots/Dronstein/dronstein.xml`.
- `Deployed` / `Stowed` poses exist; `ArmSpeed` slews joints so swings don't kick the drone.
