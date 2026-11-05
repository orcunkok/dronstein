import sys
from pick_proto import *
import pick_proto as pp
for dz in (0.002,0.003,0.004,0.005):
    pp.TOOL_DZ=dz
    out=[]
    for i in range(10):
        sc=random_scene(i)
        sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
        P=Pick(sim); P.hold_extra=10
        m=P.run()
        out.append('%s/%.3f/%.1f'%('ok' if m['fail'] is None else m['fail'][:8], m.get('ball_in_jaw_after_close',[0,0])[1], m.get('creep_cm',99)))
    print(dz,out,flush=True)
