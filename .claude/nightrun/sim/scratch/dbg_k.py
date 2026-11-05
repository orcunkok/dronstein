import sys
from pick_proto import *
import pick_proto as pp
pp.TOOL_DZ=float(sys.argv[2])
i=int(sys.argv[1])
sc=random_scene(i)
sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
P=Pick(sim); orig=P.step
def st():
    orig()
    if P.phase in('CLOSE','LIFT') and int(round(sim.t*60))%12==0:
        pj,R=sim.jaw_frame(); loc=R.T@(sim.ball()-pj)
        print(P.phase,round(sim.t,2),'drone z',round(sim.pos()[2],4),'tipz',round(float(sim.lowest_point()),4),'loc',loc.round(4),'ballz',round(sim.ball()[2],4),'g',round(sim.d.qpos[sim.arm_q['gripper']],3),'vz',round(sim.d.qvel[2],3))
P.step=st
m=P.run(); print(m['fail'])
