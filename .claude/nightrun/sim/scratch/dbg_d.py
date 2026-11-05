from pick_proto import *
import pick_proto as pp
sc=random_scene(0)
sim=Sim(seed=0,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
p=Pick(sim,True)
orig=p.step
def st():
    orig()
    if p.phase=='DESCEND' and int(sim.t*60)%30==0: print(round(sim.t,2),'z',sim.pos()[2].round(3),'ref',sim.target[2].round(3),'vz',sim.d.qvel[2].round(3),'tilt',round(sim.tilt_deg(),1),'arm',sim.arm_target.round(2))
p.step=st
m=p.run(); print(m['fail'])
