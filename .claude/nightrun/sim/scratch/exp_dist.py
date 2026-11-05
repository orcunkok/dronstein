import numpy as np
import run_batch as rb
for df in (1.0,1.5):
    for i in range(10):
        m=rb.run('nominal',i,dist_force=df)
        if not m['ok']: print(df,i,m['fail'],'tilt',round(m['max_tilt_deg'],1),'moved',m.get('ball_moved_cm_at_ground'),'ball-site',round(float(m['max_ball_tool_cm']),1),'ground',m['drone_ground_contact'],'inbin',m['ball_in_bin'],'binmv',round(m['bin_moved_cm'],1),'phase_t',list(m['phase_t'].items())[-1])
