import mujoco, numpy as np
m=mujoco.MjModel.from_xml_path('scene.xml'); d=mujoco.MjData(m)
J=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
def fk(q):
    d.qpos[:]=m.qpos0; 
    for n,v in zip(J,q): d.qpos[m.joint(n).qposadr[0]]=v
    mujoco.mj_forward(m,d)
x2=m.body('x2').id
def drone_frame(p):
    return d.xmat[x2].reshape(3,3).T@(p-d.xpos[x2])
for name,q in [('stowed',[0,-3.32,3.11,1.18,0,-0.174]),('deployed',[0,-2.01,1.85,-0.35,1.5708,-0.174])]:
    fk(q)
    print(name,'grasp(drone frame)',drone_frame(d.site('grasp_site').xpos), 'world',d.site('grasp_site').xpos)
fk([0,-2.01,1.85,-0.35,1.5708,-0.174])
for b in ['Upper_Arm','Lower_Arm','Wrist_Pitch_Roll','Fixed_Jaw','Moving_Jaw']:
    print(b, drone_frame(d.xpos[m.body(b).id]))
# pads
for g in range(m.ngeom):
    n=m.geom(g).name
    if 'pad' in n: print(n, drone_frame(d.geom_xpos[g]), m.geom_size[g])
# zero pose
fk([0,0,0,0,0,0]); print('zero grasp', drone_frame(d.site('grasp_site').xpos))
