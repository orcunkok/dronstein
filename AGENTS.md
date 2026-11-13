# Agent rules for dronstein

## Git
- Never run git on the user's checkout or remote: no add, commit, branch, push, stash or merge. The user handles git.
- Never commit, anywhere, not even in a worktree. Worktrees under `.claude/worktrees/` are scratch copies, fully isolated from the user's repo and remote: edit files there, copy them into the user's checkout as plain files, and remove the worktree when done (no commits, no branches left behind).

## Lucky Engine
- Work through the Lucky MCP (`lucky`, http://127.0.0.1:19331/mcp) from the start. Always prioritze using lucky mcp tools for edits. If no tool fits, stop and ask.
- Always build when you're done: after any script change reaches the user's checkout (edit, copy or branch switch), exit Play and `build_script_assembly`, so Lucky never runs a stale script.
- While you are working, Lucky is yours: stop and start Play as much as you need, don't wait for the user. Entities can only be edited in Edit mode.
- The user's sim (`sim/src`, TCP 4560, UDP 14600), PX4 container (UDP 14580/18570) or `autonomy/src/dronstein.py` may hold ports while you test. You may kill it whenever you need the port, without asking. Same for the user's Play session in Lucky.
- The user may not be watching the CLI and may use Lucky at the same time (e.g. a Play session you didn't start, or edits in the editor). If that gets in your way, carry on and alert them in your reply to leave Lucky alone while you work.
- Test only a fresh build: exit Play, `build_script_assembly`, then enter Play. Before trusting any result, check `APP.log` has an `X500Flight: start` line *after* the latest `Build succeeded` line; if not, the old script is still running. Building or entering Play while Play runs doesn't restart it.
- Read only the log lines written since your run started (note `wc -l` of `APP.log` first, or check the `start` line); the log also holds older runs and is reset when Lucky restarts.
- A result that doesn't change at all after a code change (same numbers to the last digit) most likely means a stale build: rebuild and re-check before concluding the change didn't work.
- Script fields: set with `set_entity_property` (component `Script`). An empty `Material` field is not null in Play: Lucky fills it with a placeholder, so always assign it.
- Save the scene: `authoring_state` (context_id), then `scene_authoring` inspect of the edited entity (fingerprint), then `authoring_save`.
- Check visually with `capture_viewport_screenshot` (works in Play; `capture_scene` doesn't render Play). Logs: `/tmp/LuckyEngine/Logs/APP.log`.

## Scene facts (verified)
- MuJoCo frame is z up; Lucky is y up: MuJoCo (x, y, z) = Lucky (x, -z, y). The ground top is at height 0.
- DirectionalLight shines along R·(1,1,1)/√3, R = Rz·Ry·Rx (radians). Straight down: Rotation `0 0.7854 -2.1863`. The SkyLight sun is cosmetic only.
- The X500's and the claw's MuJoCo geometry is generated (`Assets/Robots/X500V2/x500v2.mjcf` by `tools/x500v2/build_x500v2.py`, the claw's parts from `tools/claw/claw_physics.py`): change those, not the file, and keep the claw's height (`CLAW_POS`) equal to the scene's Claw entity Translation. Its frame is the Pod model turned half a turn, like the scene's `HolybroX500V2`. In MJCF, hinge ranges are in degrees. `GetActuator` doesn't return the claw's position servos: drive them with `GetActuatorIndex` + `SetCtrlByIndex`, and read hinge angles from `GetFullQpos` (`GetJointPosition` returned nothing useful).
- A loaded material is cached: editing its `.hmaterial` won't reload it. Write it once under a new name before first load.

## Keep it KISS and YAGNI
- Don't add a state or phase where a return value does the job: a helper that returns true/false (or a result) is enough. Fewest states, smallest change that does what was asked, no extra modes or options, features or bloatware.
- Don't leave code or docs behind for removed states; delete them with the state.

## Talking to the user
- "wrap up" means close as done with no summary.

## Where to make changes
- When you are done, make the changes in the user's main checkout (`~/dev/dronstein`), not only in a worktree. The user runs the editor from there, so worktree-only changes never reach it.

## Done means visible
- Never say a change is done from file edits or "scene saved" alone. Exit Play, build, enter Play, take `capture_viewport_screenshot` and check the change is visible in it (compare with a screenshot from before the change when it is visual). If you can't see it, it isn't done: say so.
- Rotating the physics body `x500v2` turns the model with it, so the nose (body +x = model +x, the GPS and claw side) stays where it was relative to the model. To turn the model relative to the flight direction (and the follow camera), rotate the mesh child `HolybroX500V2` (Rotation `90 0 180` flips it), not the body.
