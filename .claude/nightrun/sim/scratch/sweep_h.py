import sys
from pick_proto import *
import pick_proto as pp
for hd in (None, 0.07, 0.05, 0.03, 0.015):
    pp.HOLD_DELTA=hd; out=[]
    for i in (1,4,6,7,8,9):
        sc=random_scene(i)
        sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
        m=Pick(sim).run()
        out.append((m['ok'],round(float(m['max_ball_tool_cm']),1),round(m.get('grip_stall_angle',9),2)))
    print(hd,out,flush=True)
