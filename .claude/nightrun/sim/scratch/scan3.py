from ik import *
K=analytic_consts()
def feas(tgt):
    q=ik_analytic(np.array(tgt),1.5708,-1,K)
    if q is None: return None
    m=np.minimum(q-LO[:4],HI[:4]-q)[1:].min()
    return m,q
for H in (0.36,0.40,0.44):
  print('H',H)
  for dz in (0,0.04,0.08,0.12):
    row=[]
    for Dx in (0.18,0.22,0.26,0.30,0.34,0.38):
        r=feas([-Dx,0,0.02135+dz-H]); row.append('  -- ' if r is None else '%5.2f'%r[0])
    print(' dz',dz,row)
