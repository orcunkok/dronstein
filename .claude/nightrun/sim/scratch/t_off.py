from pick_proto import *
s=Sim(noise=False,start_xy=(0,0))
s.fly_to([0,0,1.0]); s.run(5)
print('stowed: origin z',s.pos()[2].round(3),'lowest',s.lowest_point().round(3),'offset',(s.pos()[2]-s.lowest_point()).round(3))
# lowest relative to origin for DEPLOYED
s.arm_goal(DEPLOYED,1.0); s.run(4); print('deployed offset',(s.pos()[2]-s.lowest_point()).round(3))
# horizontal extent of the robot from origin (for zone radius): max distance of any collision geom vertex
import numpy as np
