from ik import *
import numpy as np
pts=[]
for lift in np.arange(-3.3,0.17,0.03):
    for el in np.arange(-0.17,3.14,0.03):
        wr=-np.pi/2-lift-el
        if abs(wr)>1.66: continue
        p,f=fk([0,lift,el,wr],1.5708)
        pts.append((p[0],p[2],lift,el,wr))
pts=np.array(pts)
print(len(pts))
for z in np.arange(-0.45,-0.0,0.04):
    sel=pts[np.abs(pts[:,1]-z)<0.01]
    if len(sel): print('z=%.2f'%z,'x range %.3f..%.3f'%(sel[:,0].min(),sel[:,0].max()), len(sel))
