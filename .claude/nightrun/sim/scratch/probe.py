import mujoco, numpy as np
m=mujoco.MjModel.from_xml_path('/home/orcun/dev/dronstein/.claude/nightrun/sim/scene.xml')
d=mujoco.MjData(m)
print(m.nq,m.nv,m.nu,m.opt.timestep, 'mass drone', sum(m.body_mass[b] for b in range(m.nbody) if m.body_rootid[b]==m.body('x2').id))
print('bin mass', sum(m.body_mass[m.body('bin').id:m.body('bin').id+1]))
for n in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']:
    j=m.joint(n); print(n,j.range,j.qposadr)
