import sys
from pick_proto import *
import pick_proto as pp
pp.TOOL_DZ=0.003
IK.TOOL[2]=float(sys.argv[1]) if len(sys.argv)>1 else 0.0; pp.K_IK=None
out=[]
for i in range(12):
    sc=random_scene(i)
    sim=Sim(seed=i,ball_pos=sc['ball_pos'],bin_pos=sc['bin_pos'],start_yaw=sc['start_yaw'],start_xy=sc['start_xy'],ball_fric=sc['ball_fric'],ball_mass=sc['ball_mass'])
    P=Pick(sim); P.hold_extra=9
    m=P.run()
    out.append('%.4f->%.4f c%.1f%s'%(m['ball_in_jaw_at_ground'][2],m['ball_in_jaw_after_close'][2],m.get('creep_cm',99),'' if m['fail'] is None else '!'))
print(IK.TOOL,out)
