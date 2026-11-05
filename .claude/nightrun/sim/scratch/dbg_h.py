import sys
from pick_proto import *
i=int(sys.argv[1])
sc=random_scene(i)
sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
p=Pick(sim,True)
orig=p.step
def nm(g):
    n=sim.m.geom(g).name
    return n if n else sim.m.body(sim.m.geom_bodyid[g]).name+'#%d'%g
last=[0]
def st():
    orig()
    mv=np.linalg.norm(sim.bin()[:2]-p.bin0[:2])
    if mv>0.002 and last[0]<6:
        last[0]+=1
        cs=set((nm(c.geom1),nm(c.geom2)) for c in sim.d.contact[:sim.d.ncon] if 'bin' in nm(c.geom1)+nm(c.geom2) and 'ground' not in (nm(c.geom1),nm(c.geom2)))
        print(round(sim.t,2),p.phase,'bin moved',round(mv*100,2),cs,'ball',sim.ball().round(3),'grasp',sim.grasp().round(3),'drone',sim.pos().round(3))
    if p.phase in('LIFT','CARRY') and int(sim.t*60)%60==0:
        print(round(sim.t,2),p.phase,'ball-site',round(float(np.linalg.norm(sim.ball()-sim.grasp()))*100,1),'ball',sim.ball().round(3),'grip',sim.d.qpos[sim.arm_q['gripper']].round(2),'tilt',round(sim.tilt_deg(),1))
p.step=st
m=p.run(); print(m['phase_t'])
