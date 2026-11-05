from dronelib import *
s=Sim(noise=False, start_xy=(0,0))
s.fly_to([0,0,0.52]); s.run(5)
s.arm_goal(np.array([*DEPLOYED]),0.8); s.grip_goal=0.9
s.run(6)
print('hover',s.pos().round(4))
s.fly_to([0,0,0.42],vmax=0.04,amax=0.3)
for i in range(10):
    s.run(0.5); print(round(s.t,1),'z',s.pos()[2].round(4),'ref',s.target[2].round(4),'vz',s.d.qvel[2].round(3),'eint',s.eint[2].round(3))
