# Ask Lucky: expose AddBodyForceWorld to C#

The Holybro X500 V2 was first a procedural (Pod) body with a free joint. `X500Flight.cs` computes its rotor thrust, yaw
torque and drag every frame, but no script API could apply them to that body:

- `MujocoSceneComponent.SetQfrcApplied` returns true, but Lucky's policy layer overwrites `qfrc_applied` every
  substep, so nothing moves (tested on the X500 and on a golf ball).
- `RigidBodyComponent.AddForce` only drives Jolt bodies.
- MJCF actuators can't target the body: `lucky_joint_<id>` doesn't exist when `dronstein.xml` compiles.

The engine already has `Hazel::MujocoSceneInstance::AddBodyForceWorld(int body, vec3 force, vec3 point)` (`xfrc_applied`).

**Request:** expose it on `MujocoSceneComponent`, e.g. `AddBodyForceWorld(uint bodyId, Vector3 force, Vector3 point)` and
a torque variant, in MuJoCo world coordinates, kept for every substep until the next call.

Used now instead: the physics body is MJCF (`Assets/Robots/X500V2/x500v2.mjcf`) with six motors on its own free joint, and the
procedural X500 is only its visual. With this API the procedural body could carry the physics again and the motors go.
