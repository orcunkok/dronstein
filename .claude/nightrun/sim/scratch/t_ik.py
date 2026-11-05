from ik import *
K=analytic_consts()
print({k:np.round(v,4) for k,v in K.items()})
rng=np.random.default_rng(1)
for tgt in [(-0.28,0,-0.379),(-0.28,0.05,-0.279),(-0.25,-0.04,-0.35),(-0.33,0,-0.35)]:
  for es in (1,-1):
    q=ik_analytic(np.array(tgt),1.5708,es,K)
    if q is None: print(tgt,es,'none'); continue
    p,f=fk(q,1.5708)
    ok=np.all(q>=LO[:4])&np.all(q<=HI[:4])
    print(tgt,es,q.round(3),'fk err',np.linalg.norm(p-tgt).round(6),'finger',f.round(3),'in limits',ok)
