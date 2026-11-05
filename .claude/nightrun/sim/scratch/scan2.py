from ik import *
for H in (0.28,0.30,0.33,0.36,0.40):
    row=[]
    for Dx in (0.20,0.25,0.30,0.33,0.36,0.39):
        s=ik3(np.array([-Dx,0,0.02135-H]),1.5708)
        row.append('--' if s is None else '%.2f'%s[1])
    print(H,row)
s=ik3(np.array([-0.30,0,0.02135-0.36]),1.5708); print(s)
