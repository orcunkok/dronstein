from ik import *
K=analytic_consts()
H=0.45; r=0.02135
for alpha in (-0.6,-0.3,0,0.2,0.4,0.6):
    for D_ in (0.20,0.26,0.32):
        best=None
        for ty in np.arange(-0.03,-0.14,-0.002):
            tp=np.array([-0.030,ty,0.0])
            q=ik_analytic(np.array([-D_,0,r-H]),1.5708,-1,K,alpha,tp)
            if q is None or np.any(q<LO[:4]+0.02) or np.any(q>HI[:4]-0.02): continue
            low=robot_lowest([*q,1.5708])+H
            if low>=0.006: best=(ty,low,q); break
        print('alpha %.1f D %.2f'%(alpha,D_), 'none' if best is None else 'ball y_jaw %.3f (tip-edge %.3f deeper by %.0f mm) low %.4f q %s'%(best[0],-0.1064,(best[0]+0.1064)*1000,best[1],best[2].round(2)))
