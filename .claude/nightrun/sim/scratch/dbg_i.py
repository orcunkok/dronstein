import sys
from pick_proto import *
i=int(sys.argv[1])
sc=random_scene(i)
sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
p=Pick(sim,False)
orig=p.step
def nm(g):
    n=sim.m.geom(g).name
    return n if n else sim.m.body(sim.m.geom_bodyid[g]).name+'#%d'%g
bg=sim.m.geom('ball_geom').id
def st():
    orig()
    if p.phase in("CLOSE","LIFT","CARRY") and int(sim.t*60)%20==0:
        pj,R=sim.jaw_frame(); loc=R.T@(sim.ball()-pj)
        fs=[]
        for k in range(sim.d.ncon):
            c=sim.d.contact[k]
            if bg in (c.geom1,c.geom2):
                f=np.zeros(6); mujoco.mj_contactForce(sim.m,sim.d,k,f)
                other=c.geom2 if c.geom1==bg else c.geom1
                if nm(other)!='ground': fs.append((nm(other),round(f[0],1),round(c.dist,4)))
        print(round(sim.t,2),p.phase,'loc',loc.round(3),'grip',sim.d.qpos[sim.arm_q['gripper']].round(3),'ctrl',round(sim.d.ctrl[sim.act_arm[5]],2),'tq',round(sim.d.actuator_force[sim.act_arm[5]],2),fs)
p.step=st
m=p.run()
