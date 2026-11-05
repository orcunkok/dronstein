import mujoco, numpy as np
m=mujoco.MjModel.from_xml_path('scene.xml'); d=mujoco.MjData(m)
J=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
x2=m.body('x2').id
def fk(q):
    d.qpos[:]=m.qpos0
    for n,v in zip(J,q): d.qpos[m.joint(n).qposadr[0]]=v
    mujoco.mj_forward(m,d)
def df(p): return d.xmat[x2].reshape(3,3).T@(p-d.xpos[x2])
def B(n): return df(d.xpos[m.body(n).id])
for q in [[0,0,0,0,0,0],[0,-1,0.5,0.3,0,0],[0.5,-1,0.5,0.3,0,0]]:
    fk(q); S,E,W=B('Upper_Arm'),B('Lower_Arm'),B('Wrist_Pitch_Roll')
    print(q,'S',S.round(4),'|E-S|',np.linalg.norm(E-S).round(4),'|W-E|',np.linalg.norm(W-E).round(4), 'E',E.round(3))
# pan axis in drone frame
fk([0,0,0,0,0,0]); ax=d.xaxis[m.joint('shoulder_pan').id]; print('pan axis drone', (d.xmat[x2].reshape(3,3).T@ax).round(3))
for n in ['shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']:
    fk([0,0,0,0,0,0]); ax=d.xaxis[m.joint(n).id]; print(n,'axis drone',(d.xmat[x2].reshape(3,3).T@ax).round(3))
# planar angle vs joints: orientation of finger axis (-y of jaw) in drone x-z plane
fj=m.body('Fixed_Jaw').id
for q in [[0,0,0,0,0,0],[0,-1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,-1,0.5,0.3,0,0]]:
    fk(q); R=d.xmat[x2].reshape(3,3).T@d.xmat[fj].reshape(3,3); f=-R[:,1]
    print(q,'finger dir drone',f.round(3),'angle from -z toward -x (deg)',np.degrees(np.arctan2(-f[0],-f[2])).round(1))
