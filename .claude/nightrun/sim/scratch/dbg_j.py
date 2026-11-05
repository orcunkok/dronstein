import sys
from pick_proto import *
import pick_proto as pp
pp.HOLD_DELTA=0.01
i=int(sys.argv[1])
sc=random_scene(i)
sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
P=Pick(sim); orig=P.step
def st():
    orig()
    if P.phase=='CLOSE' and int(round(sim.t*60))%6==0:
        pj,R=sim.jaw_frame(); loc=R.T@(sim.ball()-pj)
        print(round(sim.t,2),'drone z',round(sim.pos()[2],4),'jaw tip z',round(float(sim.lowest_point()),4),'ball loc',loc.round(4),'ballz',round(sim.ball()[2],4),'g',round(sim.d.qpos[sim.arm_q['gripper']],3), 'ball xy',sim.ball()[:2].round(4))
P.step=st
P.run()
