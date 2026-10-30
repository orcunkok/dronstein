# MVP plan: Holybro X500 + cheap claw + tennis balls in Lucky

## Context
The current setup (Skydio X2 + SO-100 arm, `Assets/Robots/Dronstein/dronstein.xml`, existing scenes and scripts) has a visible gap between the ball and the gripper. Likely causes: `margin="0.003" gap="0.001"`, and convex-hull mesh colliders on concave jaws. We want a fresh MVP that is separate from that setup: the real drone (Holybro X500 V2) with a real, cheap, light off-the-shelf claw and tennis balls. **Legs and claw must collide correctly**; wiring and electronics are out of scope.

Ground rules:
- Don't touch anything existing. All new work goes in new folders and files.
- No git (AGENTS.md); the user is on branch `simple-mvp`.
- Lucky changes go through the Lucky MCP only. No hand-editing `.hscene`, `.hzr` or `.hmaterial`.
- **Stop after every step and wait for the user's OK.**

## What already exists (findings)
- **Lucky:** no tennis ball, no drone and no drone gripper in the Content Vault or project assets (checked `list_content_packs`, `list_available_robots`, `search_assets`). So the ball and claw are ours to build.
- **X500 source:** PX4 official `PX4-gazebo-models/models/x500_base` (BSD-3). Provides:
  - meshes: frame `NXP-HGD-CF.dae`, `5010Base/Bell.dae`, `1345_prop_cw/ccw.stl` (props at scale 0.846)
  - mass 2.0 kg, inertia diag (0.0217, 0.0217, 0.04)
  - rotors at (±0.174, ±0.174, 0.06); rotors 0 and 1 ccw, 2 and 3 cw
  - max thrust 8.55 N per motor (k=8.54858e-6, ω_max=1000); yaw moment factor 0.016
  - leg boxes 0.015×0.015×0.21 at y=±0.098, tilted ±0.35 rad; skids 0.25×0.015×0.015 at y=±0.132, z=-0.2195
- **Pattern to reuse:** visual and collision split as in `dronstein.xml` (`group=2 contype=0` for visuals, `group=3` for collision), the `finger_collision` pad idea (`condim=6`, high friction), and thrust motors with yaw gear.
- **Repo facts:** MuJoCo z-up vs Lucky y-up conversion; multi-collider movable objects need an unscaled parent with a free joint and a collider (AGENTS.md).

## Steps (one at a time, confirm between each)

- [x] **Step 0: Plan file.** Copy this plan to `/Users/orcun/dev/dronstein/mvpPlan.md`, then track progress there (a check per step).

- [ ] **Step 1: Choose the gripper (decision, no build).**
- Requirement: a tennis ball is 67 mm across, so the claw needs ≥ ~80 mm opening. Under ~150 g including the servo, servo-driven, cheap (<$20), buyable.
- Search online for 2–3 real products. For each: link, price, weight, opening, servo, and whether a CAD or STL file exists.
- Note: the common MG90S/SG90 mini grippers (25–35 mm objects) are **too small**. Candidates to check: aluminium MG996R claw kits, and 4-finger servo "drone claw" or "payload claw" units.
- The user picks one. Record the dimensions we'll model in mvpPlan.md.
- **Picked: Hiwonder Green Robot Gripper (aluminium, with LDX-335MG servo).**
  - Buy (US): https://www.amazon.com/dp/B09TW8ZGSL; maker page: https://www.hiwonder.com/products/green-robot-gripper ($39.99)
  - Maker specs: max opening **132 mm**, **136 g with servo** (servo 64 g), body 54.5 × 20 × 47.5 mm, PWM 500–2500 µs, 0–180°, M3 mounting holes.
  - Tennis ball 67 mm → ~±32 mm landing tolerance.
  - No CAD found: model finger shapes from product photos at these dimensions, and re-measure the real part when it arrives.
  - Still to confirm on the Amazon page (couldn't fetch it): price, stock and delivery time.

- [ ] **Step 2: Get the assets (download and convert only).**
- Download the PX4 x500_base meshes into the job tmp folder. Convert `.dae` → `.obj` (Python venv with trimesh + pycollada, run with `-I` from a scripts folder; needs Bash approval).
- Gripper: use the vendor's or a community STL/STEP if one exists for the chosen product; otherwise model it from simple shapes at the real dimensions.
- Tennis ball: our own. A UV-sphere OBJ (r = 33.5 mm) plus a generated yellow-felt texture with the white seam curve. No licence issues.
- Output goes to the new folder `LuckyEngine/Assets/Robots/X500Claw/assets/`, with the PX4 LICENSE copied in.

- [ ] **Step 3: X500 model, drone only** (`Assets/Robots/X500Claw/x500_claw.xml`).
- Free-floating body, 2.0 kg plus the PX4 inertia; 4 thrust sites and motors (`ctrlrange 0 8.55`, yaw gear ±0.016). Visual meshes positioned as in the SDF.
- **Legs:** fit collision to the mesh, not to PX4's rough boxes. Measure the frame mesh's leg tubes and skids, then use capsules or cylinders matching the real tube diameter and position. Use `margin=0 gap=0`.
- Body collision: one box for the centre plate. Props: no collision (MVP).
- Check offline (MuJoCo Python): the model compiles, the drone drops and rests on its skids without bouncing or sinking, and total mass is correct.

- [ ] **Step 4: Add the claw.**
- Mount it centred under the payload rails; fingertip height ≈ 1 cm above the ground when landed (adjustable).
- Joints matching the real mechanism (e.g. 2 hinges geared together through a joint equality). One `position` actuator `gripper` with range 0 = open to 1 = closed, and a force limit from the servo's torque.
- **Collision:** finger shapes fitted to the real fingers. Boxes or capsules where possible; convex pieces where a finger is curved (one convex mesh per segment, never one hull over a concave finger). `margin=0 gap=0`, high-friction pads, contacts excluded between claw parts and the drone body.
- Check offline: open and close sweep with no self-collision; mass and centre of gravity are still balanced.

- [ ] **Step 5: Tennis ball.**
- Its own model, or a body in the same file: a true `sphere` collider r=0.0335, mass 0.058, friction suited to felt, soft `solref` for a light bounce. Visual = the textured mesh.
- Check offline: drop on the floor (it should bounce a little and settle); visual radius equals collider radius.

- [ ] **Step 6: Grasp test offline (the gap check).**
- Drone landed over the ball → close the claw → hover upward.
- Measure the pad-to-ball distance with `mj_geomDistance` and the contact penetration. Pass: gap ≤ 0.5 mm, penetration ≤ 1 mm, and the ball stays held through the lift.

- [ ] **Step 7: Bring it into Lucky.**
- New scene via MCP (`create_scene`) with ground, light and camera following the existing scene facts.
- Insert the `x500_claw.xml` model and tennis balls through the MCP. If no MCP tool inserts an arbitrary model file, stop and ask.
- Check with `capture_viewport_screenshot` that visuals and colliders line up (collision group visible), and that Play is stable on the ground.

- [ ] **Step 8 (later, separate): control script.** A minimal script to take off, land on the ball, grab and lift. Only after Steps 1–7 are approved.

## Verification summary
- Every model step ends with an offline MuJoCo check (compile, mass, drop test), and the numbers are reported to the user.
- Step 6 gives a measured answer to "is there a gap?".
- Step 7 gives a Lucky screenshot plus a Play stability check.

## Files (all new)
- `/Users/orcun/dev/dronstein/mvpPlan.md`
- `LuckyEngine/Assets/Robots/X500Claw/x500_claw.xml`, `assets/*`, `LICENSE`
- the new Lucky scene, created through MCP
- Scratch work in `$CLAUDE_JOB_DIR/tmp` (venv, downloads, check scripts)
