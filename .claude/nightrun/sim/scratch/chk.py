from ik import *
K=analytic_consts(); H=0.45; r=0.02135
tp=np.array([-0.030,-0.074,0.0])
q=ik_analytic(np.array([-0.26,0,r-H]),1.5708,-1,K,0.0,tp)
p,f=fk(q,1.5708,tp); print('q',q.round(3),'tool',p.round(4),'target',[-0.26,0,r-H],'finger',f.round(3))
print('lowest',robot_lowest([*q,1.5708])+H)
# pad bottoms
for n in ['fixed_jaw_pad_1','moving_jaw_pad_1','fixed_jaw_pad_4']:
    g=M.geom(n).id; print(n,(D.geom_xpos[g]+H).round(3)[2])
for b in ['Fixed_Jaw','Moving_Jaw','Wrist_Pitch_Roll','Lower_Arm']: print(b,(D.xpos[M.body(b).id][2]+H).round(3))
