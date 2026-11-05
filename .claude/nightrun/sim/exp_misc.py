import numpy as np, math
from pick_proto import *
import pick_proto as pp
import run_batch as rb
# 1. hover precision with measurement noise and arm at grasp pose
sim=Sim(seed=5,ball_pos=(-1,0),start_xy=(-0.7,0)); P=Pick(sim)
sim.fly_to([-0.76,0,0.42],math.pi); sim.run(8)
q=tool_world_to_q(sim,np.array([-1.0,0,0.0214+0.003])); sim.arm_goal(np.array([*q,ROLL]),0.8); sim.run(6)
tr=[]
for _ in range(600): sim.frame(); tr.append(sim.pos().copy())
tr=np.array(tr); print('HOVER (noise on, arm deployed, 10 s): std mm xyz', (tr.std(0)*1000).round(1), 'peak |dev| mm', (abs(tr-tr.mean(0)).max(0)*1000).round(1))
# 2. transit_high count in ring scenario + disturbance
hi=0; n=0
for i in range(20):
    m=rb.run('ring',i); hi+=m['transit_high']; n+=1
print('ring: high transit used in %d/%d runs'%(hi,n))
# 3. stress: random disturbance forces on the drone (N, per 5 ms physics step, white), bigger sensor noise
for df in (0.5,1.5):
    ok=0; N=15
    for i in range(N):
        m=rb.run('nominal',i,dist_force=df); ok+=m['ok']
    print('disturbance sigma %.1f N white per step: %d/%d ok'%(df,ok,N),flush=True)
# 4. miss detection: ball 4 cm off in the jaw opening direction -> must be detected, not carried
import dronelib
miss=0
for off in (0.03,0.05):
    s=rb.scene('nominal',3)
    sim=Sim(seed=3,ball_pos=s['ball_pos'],bin_pos=s['bin_pos'],start_yaw=s['start_yaw'],start_xy=s['start_xy'])
    P=Pick(sim)
    orig=pp.tool_world_to_q
    def shifted(sim_,tool,roll=pp.ROLL,off=off,orig=orig):
        t=np.array(tool,float); to=np.array(s['bin_pos']+(0,))[:2]-np.array(s['ball_pos'])
        ang=math.atan2(to[1],to[0]); t[0]+=off*math.cos(ang+math.pi/2); t[1]+=off*math.sin(ang+math.pi/2)  # sideways, across the jaws
        return orig(sim_,t,roll)
    pp.tool_world_to_q=shifted
    m=P.run(); pp.tool_world_to_q=orig
    print('miss test offset %.0f cm: grasp_detected=%s fail=%s ball moved by approach %.1f cm grip angle %.2f'%(off*100,m.get('grasp_detected'),m['fail'],m.get('ball_moved_cm_at_ground',-1),m.get('grip_joint_closed',9)))
