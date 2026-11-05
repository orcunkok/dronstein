from pick_proto import *
sc=random_scene(0)
sim=Sim(seed=0,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
import pick_proto as pp
orig=pp.tool_world_to_q
def dbg(sim_,tool,roll=ROLL):
    q=orig(sim_,tool,roll)
    if q is None:
        R=quat_to_mat(sim_.d.qpos[3:7]); print('FAIL tool_d',(R.T@(np.asarray(tool)-sim_.d.qpos[0:3])).round(3),'t',round(sim_.t,2),'pos',sim_.pos().round(3),'tilt',sim_.tilt_deg())
    return q
pp.tool_world_to_q=dbg
p=Pick(sim,True); m=p.run(); print(m['fail'])
