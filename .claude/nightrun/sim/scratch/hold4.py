import sys
from pick_proto import *
import pick_proto as pp
for hd in (None,0.07,0.04,0.02,0.01,0.005):
    pp.HOLD_DELTA=hd
    out=[]
    for i in (1,4,7,0,2,3):
        sc=random_scene(i)
        sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
        P=Pick(sim); P.hold_extra=0
        def enter(nm, P=P, orig=P.enter):
            orig(nm)
            if nm=='CARRY' and not hasattr(P,'done'):
                P.done=1
                q=P.q_hold.copy(); q[3]+=1.57; P.q_hold=q
                for _ in range(60*3): P.carry_step()
                y0=P.ball_in_jaw()
                for _ in range(60*15): P.carry_step()
                y1=P.ball_in_jaw()
                P.res=(round(float(-1000*(y1-y0)[1]),1), round(float(np.linalg.norm(sim.ball()-sim.grasp()))*100,1))
                raise StopIteration
        P.enter=enter
        try: P.run()
        except StopIteration: pass
        out.append(getattr(P,'res',None))
    print(hd,out,flush=True)
