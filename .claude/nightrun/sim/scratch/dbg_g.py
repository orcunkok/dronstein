from pick_proto import *
IK.TOOL[0]=-0.026
sc=random_scene(1)
sim=Sim(seed=1,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
p=Pick(sim,True)
def nm(g):
    n=sim.m.geom(g).name
    return n if n else sim.m.body(sim.m.geom_bodyid[g]).name+'#%d'%g
orig=p.step
def st():
    orig()
    if p.phase=='DESCEND' and int(sim.t*60)%20==0:
        cs=[(nm(c.geom1),nm(c.geom2),round(c.dist,4)) for c in sim.d.contact[:sim.d.ncon] if 'bin' not in nm(c.geom1)+nm(c.geom2) and not ({nm(c.geom1),nm(c.geom2)}=={'ground','ball_geom'})]
        print(round(sim.t,2),'z',sim.pos()[2].round(3),'ref',sim.target[2].round(3),'lowest',round(sim.lowest_point(),4),cs)
p.step=st
m=p.run(); print(m['fail'])
