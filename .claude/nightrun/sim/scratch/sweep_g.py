import sys
from pick_proto import *
import pick_proto as pp
for gc in (-0.174, 0.0, 0.15, 0.25, 0.30):
    pp.G_CLOSE=gc; out=[]
    for i in (1,4,6,7,8,9):
        sc=random_scene(i)
        sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
        m=Pick(sim).run()
        out.append((m['ok'],round(m['max_ball_tool_cm'],1),round(m['bin_moved_cm'],1)))
    print(gc,out,flush=True)
