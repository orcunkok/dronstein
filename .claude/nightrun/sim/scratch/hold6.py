import sys
from pick_proto import *
import pick_proto as pp
for dz in (0.007,0.003,0.0,-0.003,-0.006):
    pp.TOOL_DZ=dz
    out=[]
    for i in (1,4,7,0,2,3):
        sc=random_scene(i)
        sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
        P=Pick(sim); P.hold_extra=12
        m=P.run()
        out.append((round(m.get('ball_in_jaw_after_close',[0,0])[1],4),round(m.get('creep_cm',99),2)))
    print(dz,out,flush=True)
