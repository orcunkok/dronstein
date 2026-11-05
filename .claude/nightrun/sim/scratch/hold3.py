import sys
from pick_proto import *
import pick_proto as pp
pp.HOLD_DELTA=float(sys.argv[1]) if len(sys.argv)>1 and sys.argv[1]!='None' else None
variants=[('vertical finger (baseline)',0,0),('wrist+0.8',0.8,0),('wrist+1.57',1.57,0),('wrist+1.57 roll->0',1.57,-1.5708),('wrist+1.57 roll->pi',1.57,1.5708),('roll->0 only',0,-1.5708),('roll->pi only',0,1.5708)]
for name,dw,dr in variants:
    out=[]
    for i in (1,4,7):
        sc=random_scene(i)
        sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
        P=Pick(sim); P.hold_extra=0
        orig_finish=P.finish
        # run until just before CARRY by hooking enter
        def enter(nm, P=P, orig=P.enter, dw=dw, dr=dr):
            orig(nm)
            if nm=='CARRY' and not hasattr(P,'done'):
                P.done=1
                q=P.q_hold.copy(); q[3]+=dw; q[4]+=dr
                P.q_hold=q
                for _ in range(60*3): P.carry_step()
                y0=P.ball_in_jaw(); 
                for _ in range(60*10): P.carry_step()
                y1=P.ball_in_jaw()
                P.res=(np.round(1000*(y1-y0),1), round(float(np.linalg.norm(sim.ball()-sim.grasp()))*100,1), round(float(sim.d.qpos[sim.arm_q['gripper']]),2))
                raise StopIteration
        P.enter=enter
        try: P.run()
        except StopIteration: pass
        out.append(getattr(P,'res',None))
    print(name,out,flush=True)
