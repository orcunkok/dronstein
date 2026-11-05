"""Drone-moves-table vs arm-servo precision: ball position error in the jaw frame at ground level."""
import numpy as np, sys
from pick_proto import *
import run_batch as rb
def jaw_err(noservo, n=16, arm_v=ARM_V, **kw):
    ex=[];dev=[];tilt=[]
    for i in range(n):
        s=rb.scene('nominal',i)
        sim=Sim(seed=i,ball_pos=s['ball_pos'],bin_pos=s['bin_pos'],start_yaw=s['start_yaw'],start_xy=s['start_xy'],ball_fric=s['ball_fric'],ball_mass=s['ball_mass'],**kw)
        P=Pick(sim); P.noservo=noservo; P.arm_v=arm_v
        m=P.run()
        j=np.array(m['ball_in_jaw_at_ground']); nom=np.array([-0.030,-0.0857-0.003,-0.006])
        ex.append(j-nom); dev.append(m.get('deploy_dev_cm',0)); tilt.append(m.get('deploy_tilt_deg',0))
        ex[-1]=np.append(ex[-1], 1.0 if m['ok'] else 0.0)
    ex=np.array(ex)*np.array([1000,1000,1000,1])
    return ex
if __name__=='__main__':
    for ns in (False,True):
        e=jaw_err(ns)
        print('noservo' if ns else 'servo  ', 'jaw-frame ball error mm: x mean %.1f sd %.1f max|%.1f|  y mean %.1f sd %.1f max|%.1f|  z mean %.1f sd %.1f max|%.1f|   success %d/%d'%(e[:,0].mean(),e[:,0].std(),abs(e[:,0]).max(),e[:,1].mean(),e[:,1].std(),abs(e[:,1]).max(),e[:,2].mean(),e[:,2].std(),abs(e[:,2]).max(),e[:,3].sum(),len(e)),flush=True)
    for av in (0.5,0.8,1.0,1.5,2.5):
        devs=[];tl=[]
        for i in range(6):
            s=rb.scene('nominal',i)
            sim=Sim(seed=i,ball_pos=s['ball_pos'],bin_pos=s['bin_pos'],start_yaw=s['start_yaw'],start_xy=s['start_xy'],ball_fric=s['ball_fric'],ball_mass=s['ball_mass'])
            P=Pick(sim); P.arm_v=av; m=P.run(); devs.append(m.get('deploy_dev_cm',0)); tl.append(m.get('deploy_tilt_deg',0))
        print('ARM slew %.1f rad/s: drone peak displacement during stowed->grasp deploy %.1f cm (max over 6 runs), peak tilt %.1f deg'%(av,max(devs),max(tl)),flush=True)
