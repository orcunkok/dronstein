import numpy as np
import run_batch as rb
for df in (0.0,1.0,1.5):
    out=[]
    for i in range(8):
        m=rb.run('nominal',i,dist_force=df)
        out.append((round(m.get('grip_joint_closed',9),2), round(m.get('ball_moved_cm_after_close',-1),1), m['ok']))
    print(df,out)
