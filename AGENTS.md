# Agent rules for dronstein

## Git
- Never run git on the user's checkout or remote: no add, commit, branch, push, stash or merge. The user handles git.
- Agents may work in their own worktrees under `.claude/worktrees/` (ignored by git). Commit there if needed, but never push, and get the changes into the user's checkout as plain file edits.

## Lucky Engine
- Work through the Lucky MCP (`lucky`, http://127.0.0.1:19331/mcp) from the start. Don't hand-edit Lucky's own files (`AssetRegistry.hzr`, `.hmaterial` handles, `.hscene`) or invent asset handles. If no tool fits, stop and ask.
- Scripts: edit `LuckyEngine/Assets/Scripts/Client/Source/*.cs`, then `build_script_assembly`. Use Lucky's inspector attributes (`read_doc("patterns/inspector-attributes")`), e.g. `[Button("Label")]` methods for buttons.
- You may stop and start Play as needed. Entities can only be edited in Edit mode.
- Script fields: set with `set_entity_property` (component `Script`). An empty `Material` field is not null in Play: Lucky fills it with a placeholder, so always assign it.
- Save the scene: `authoring_state` (context_id), then `scene_authoring` inspect of the edited entity (fingerprint), then `authoring_save`.
- Check visually with `capture_viewport_screenshot` (works in Play; `capture_scene` doesn't render Play). Logs: `/tmp/LuckyEngine/Logs/APP.log`.

## Scene facts (verified)
- MuJoCo frame is z up; Lucky is y up: MuJoCo (x, y, z) = Lucky (x, -z, y). The ground top is at height 0.
- DirectionalLight shines along R·(1,1,1)/√3, R = Rz·Ry·Rx (radians). Straight down: Rotation `0 0.7854 -2.1863`. The SkyLight sun is cosmetic only.
- Tiled floor: use a Plane (`GroundSurface`), not a Cube (bad top-face UVs). `Ground` is collider only.
- Use `create_entity` + `add_component` instead of `create_game_object` (which writes a junk `<Name>_Material.hmaterial`).
- A loaded material is cached: editing its `.hmaterial` won't reload it. Write it once under a new name before first load.

## Talking to the user
- "wrap up" means close as done with no summary.
