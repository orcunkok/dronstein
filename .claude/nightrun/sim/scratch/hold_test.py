import sys
from pick_proto import *
import pick_proto as pp
seeds=(1,4,6,7)
for hd in (None, 0.07, 0.05, 0.03, 0.015, 0.008):
    pp.HOLD_DELTA=hd; out=[]
    for i in seeds:
        sc=random_scene(i)
        sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
        P=Pick(sim); P.hold_extra=12
        m=P.run()
        out.append((round(m.get('creep_cm',99),2),round(m.get('creep_grip',9),2)))
    print(hd,out,flush=True)
