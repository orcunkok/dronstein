from pick_proto import *
sc=random_scene(0)
sim=Sim(seed=0,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
p=Pick(sim,False)
orig=p.step; seen=[0]
def nm(g):
    n=sim.m.geom(g).name
    return n if n else sim.m.body(sim.m.geom_bodyid[g]).name+'#%d'%g
def st():
    orig()
    if p.phase=='DESCEND' and True:
        for c in sim.d.contact[:sim.d.ncon]:
            a,b=nm(c.geom1),nm(c.geom2)
            if 'bin' in a+b: continue
            if a=='ground' and b=='ball_geom': continue
            if a=='ground' and b.startswith('bin'): continue
            print(round(sim.t,2),a,b,round(c.dist,4),'z',sim.pos()[2].round(3)); seen[0]+=1
p.step=st
m=p.run()
