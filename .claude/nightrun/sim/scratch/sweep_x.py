import sys
from pick_proto import *
import pick_proto as pp
for tx in (-0.020,-0.026,-0.032):
    IK.TOOL[0]=tx; pp.K_IK=None
    out=[]
    for i in range(3):
        sc=random_scene(i)
        sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
        m=Pick(sim).run()
        out.append((m['fail'],round(m.get('ball_moved_cm',-1),2),round(m.get('ball_moved_cm_at_ground',-1),2),m.get('grasp_detected'),m['ok']))
    print(tx,out)
