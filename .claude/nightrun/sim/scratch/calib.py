from ik import *
K=analytic_consts()
H=0.45; r=0.02135
for alpha in (0,0.4,0.7,0.9,1.1):
    for D_ in (0.24,0.30):
        # find tool y (jaw frame) so that the lowest robot point sits at ground + 0.006 when the ball centre is at z=r
        lo_y,hi_y=-0.14,-0.03
        res=None
        for it in range(30):
            ty=(lo_y+hi_y)/2
            tp=np.array([-0.030,ty,0.0])
            q=ik_analytic(np.array([-D_,0,r-H]),1.5708,-1,K,alpha,tp)
            if q is None: res=None; break
            low=robot_lowest([*q,1.5708])+H
            # deeper ball (ty closer to 0) -> lower pads -> lower 'low'
            if low>0.006: lo_y=ty   # too high: pads can go lower, move ball deeper (ty up toward -0.03)
            else: hi_y=ty
            res=(ty,low,q)
        if res: print('alpha',alpha,'D',D_,'tool y %.4f low %.4f'%(res[0],res[1]),'q',res[2].round(2),'in limits',bool(np.all(res[2]>=LO[:4])&np.all(res[2]<=HI[:4])))
        else: print('alpha',alpha,'D',D_,'unreachable')
