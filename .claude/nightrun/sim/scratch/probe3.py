import mujoco, numpy as np
m=mujoco.MjModel.from_xml_path('scene.xml'); d=mujoco.MjData(m)
J=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
def fk(q):
    d.qpos[:]=m.qpos0
    for n,v in zip(J,q): d.qpos[m.joint(n).qposadr[0]]=v
    mujoco.mj_forward(m,d)
fj=m.body('Fixed_Jaw').id
for g in [-0.174,0,0.3,0.5,0.7,0.9,1.2,1.75]:
    fk([0,-2.01,1.85,-0.35,1.5708,g])
    R=d.xmat[fj].reshape(3,3); 
    out=[]
    for n in ['moving_jaw_pad_1','moving_jaw_pad_2','moving_jaw_pad_3','moving_jaw_pad_4']:
        gid=m.geom(n).id; p=R.T@(d.geom_xpos[gid]-d.xpos[fj]); out.append(np.round(p[[0,1]],4))
    print(g,out)
fk([0,-2.01,1.85,-0.35,1.5708,0.0])
R=d.xmat[fj].reshape(3,3)
print('site in jaw frame',R.T@(d.site('grasp_site').xpos-d.xpos[fj]))
print('jaw frame axes in world (cols)\n',R)
# moving pad orientation
for n in ['moving_jaw_pad_4']:
    gid=m.geom(n).id; print(R.T@d.geom_xmat[gid].reshape(3,3))
