from pick_proto import *
import mujoco
s=Sim(noise=False, ball_pos=(-1.0,0.0))
sv=Servo(s)
ball=s.ball()
yaw=math.pi; nose=np.array([math.cos(yaw),math.sin(yaw),0])
hover=ball-STANDOFF*nose; hover[2]=H_HOVER
print('hover',hover)
s.fly_to(hover, yaw)
s.run(8, until=lambda: not s.flying())
s.run(2)
print('hovering at',s.pos().round(3))
s.grip_goal=G_OPEN
# pregrasp
tool=ball+np.array([0,0,PRE_DZ])
for i in range(240):
    sv.goto(tool, 0.8); s.frame()
    if i%30==0: print(round(s.t,2),'pos',s.pos().round(3),'tilt',round(s.tilt_deg(),1),'qarm',s.arm_target.round(2),'err',round(sv.err,4))
print('ball moved',np.linalg.norm(s.ball()-ball))
def jawinfo():
    p,R=s.jaw_frame(); b=s.ball(); loc=R.T@(b-p)
    out={}
    fromto=np.zeros(6)
    bg=s.m.geom('ball_geom').id
    for n in ['fixed_jaw_pad_%d'%i for i in range(1,5)]+['moving_jaw_pad_%d'%i for i in range(1,5)]:
        out[n]=round(mujoco.mj_geomDistance(s.m,s.d,bg,s.m.geom(n).id,0.2,fromto),4)
    return loc.round(4),out
for k in range(120):
    sv.goto(tool,0.8); s.frame()
print('pre-grasp: arm',s.arm_target.round(3),'ball in jaw frame / dist',jawinfo())
print('grasp site',s.grasp().round(4),'ball',s.ball().round(4))
# descend
z=PRE_DZ
while z>0:
    z=max(0,z-DESC_V/60)
    sv.goto(ball+np.array([0,0,z]),0.8); s.frame()
for k in range(90): sv.goto(ball,0.8); s.frame()
print('landed: arm',s.arm_target.round(3),'jaw frame / dist',jawinfo())
print('grasp site',s.grasp().round(4),'ball',s.ball().round(4),'moved',np.linalg.norm(s.ball()-ball), 'drone',s.pos().round(3),'tilt',s.tilt_deg())
print('lowest robot pt',s.lowest_point())
# contacts
for c in s.d.contact[:s.d.ncon]:
    print(s.m.geom(c.geom1).name or s.m.body(s.m.geom_bodyid[c.geom1]).name, s.m.geom(c.geom2).name or s.m.body(s.m.geom_bodyid[c.geom2]).name, round(c.dist,4))
