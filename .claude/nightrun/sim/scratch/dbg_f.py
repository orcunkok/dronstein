from pick_proto import *
sc=random_scene(0)
sim=Sim(seed=0,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
p=Pick(sim,False)
orig=p.step
def info():
    pj,R=sim.jaw_frame(); loc=R.T@(sim.ball()-pj)
    fromto=np.zeros(6); bg=sim.m.geom('ball_geom').id
    d={n[:3]+n[-1]:round(mujoco.mj_geomDistance(sim.m,sim.d,bg,sim.m.geom(n).id,0.3,fromto),4) for n in ['fixed_jaw_pad_%d'%i for i in range(1,5)]+['moving_jaw_pad_%d'%i for i in range(1,5)]}
    return loc.round(4),d
def st():
    orig()
    if p.phase=='DESCEND' and int(sim.t*60)%24==0: print(round(sim.t,2),'z',sim.pos()[2].round(3),*info())
p.step=st
m=p.run()
