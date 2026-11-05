from dronelib import *
s=Sim(noise=False)
s.fly_to([1.0,0,0.8])
s.run(3)
print(s.pos(), s.tilt_deg(), s.max_tilt, 'mass',s.mass)
s.fly_to([0.0,1.0,0.8]); 
for i in range(8):
    s.run(1); print(round(s.t,1), s.pos().round(3), 'ref',s.target.round(3), round(s.tilt_deg(),1))
