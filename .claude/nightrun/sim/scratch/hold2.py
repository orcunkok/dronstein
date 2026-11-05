from pick_proto import *
import pick_proto as pp
pp.HOLD_DELTA=0.015
for i in (1,4,6,7,0,2,3,5):
    sc=random_scene(i)
    sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
    P=Pick(sim); P.hold_extra=10
    m=P.run()
    print(i,'ground',m['ball_in_jaw_at_ground'],'after close',m['ball_in_jaw_after_close'],'creep',round(m.get('creep_cm',99),2),'yaw',round(sim.yaw,2),'arm',sim.arm_target.round(2))
