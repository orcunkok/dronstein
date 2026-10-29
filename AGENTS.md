# Agent rules for dronstein

## Git
- Never run git on the user's checkout or remote: no add, commit, branch, push, stash or merge. The user handles git.
- Agents may work in their own worktrees under `.claude/worktrees/` (ignored by git). Commit there if needed, but never push, and get the changes into the user's checkout as plain file edits.

## Lucky Engine
- Work through the Lucky MCP (`lucky`, http://127.0.0.1:19331/mcp) from the start. Don't hand-edit Lucky's own files (`AssetRegistry.hzr`, `.hmaterial` handles, `.hscene`) or invent asset handles. If no tool fits, stop and ask.
- Scripts: edit `LuckyEngine/Assets/Scripts/Client/Source/*.cs`, then `build_script_assembly`. Use Lucky's inspector attributes (`read_doc("patterns/inspector-attributes")`), e.g. `[Button("Label")]` methods for buttons.
- Always build when you're done: after any script change reaches the user's checkout (edit, copy or branch switch), exit Play and `build_script_assembly`, so Lucky never runs a stale script.
- You may stop and start Play as needed. Entities can only be edited in Edit mode.
- Test only a fresh build: exit Play, `build_script_assembly`, then enter Play. Before trusting any result, check `APP.log` has a `DroneTeleop: start` line *after* the latest `Build succeeded` line; if not, the old script is still running. Building or entering Play while Play runs doesn't restart it.
- Read only the log lines written since your run started (note `wc -l` of `APP.log` first, or check the `start` line); the log also holds older runs and is reset when Lucky restarts.
- Start a run with the dev command file: write e.g. `pick 14` or `pickall` to `LuckyEngine/Cache/nightrun_cmd.txt` (commands: `case "..."` in `DroneTeleop.cs`).
- A result that doesn't change at all after a code change (same numbers to the last digit) most likely means a stale build: rebuild and re-check before concluding the change didn't work.
- Script fields: set with `set_entity_property` (component `Script`). An empty `Material` field is not null in Play: Lucky fills it with a placeholder, so always assign it.
- Save the scene: `authoring_state` (context_id), then `scene_authoring` inspect of the edited entity (fingerprint), then `authoring_save`.
- Check visually with `capture_viewport_screenshot` (works in Play; `capture_scene` doesn't render Play). Logs: `/tmp/LuckyEngine/Logs/APP.log`.

## Scene facts (verified)
- MuJoCo frame is z up; Lucky is y up: MuJoCo (x, y, z) = Lucky (x, -z, y). The ground top is at height 0.
- DirectionalLight shines along R·(1,1,1)/√3, R = Rz·Ry·Rx (radians). Straight down: Rotation `0 0.7854 -2.1863`. The SkyLight sun is cosmetic only.
- Tiled floor: use a Plane (`GroundSurface`), not a Cube (bad top-face UVs). `Ground` is collider only.
- Use `create_entity` + `add_component` instead of `create_game_object` (which writes a junk `<Name>_Material.hmaterial`).
- A movable object made of several colliders (e.g. `TrashCan`): put a free `MujocoJoint` **and a collider** on an unscaled parent, then parent the pieces under it; they become one rigid body. A joint on a parent with no collider is ignored (stays world/static).
- A loaded material is cached: editing its `.hmaterial` won't reload it. Write it once under a new name before first load.

## Keep it KISS
- Don't add a state or phase where a return value does the job: a helper that returns true/false (or a result) is enough. Fewest states, smallest change that does what was asked, no extra modes or options.
- Don't leave code or docs behind for removed states; delete them with the state.

## Benchmark
- When the user says "benchmark": run `pickall` on a fresh build (exit Play, `build_script_assembly`, enter Play, write `pickall` to `LuckyEngine/Cache/nightrun_cmd.txt`). Wait until `APP.log` has the `pickall seed=... end=done` line, and check the `DroneTeleop: start` line is after the latest `Build succeeded`.
- Only record a run after it has ended. Use the `METRIC pick ball=... result=` lines of that run (`time=` field).
- Write the record to `benchmarks/benchmark-YYYY-MM-DD-HHMM.md` and `benchmarks/granular/granular-YYYY-MM-HHMM.md` in same format as them.
- Compare it with `benchmarks/benchmark_best.md`. If the new run looks better (more successful picks, or a lower average at the same success count), ask the user whether to overwrite `benchmark_best.md`. Overwrite it only after a yes.

## Talking to the user
- "wrap up" means close as done with no summary.
