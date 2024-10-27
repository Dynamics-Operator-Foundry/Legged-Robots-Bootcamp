import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator

m = 1
c = 1
k = 10
wall = -2

t_step = 1/1000

ground = 0

x0 = 1.2
dx0 = 0.0
t = 0

x0_all_rk4 = []
dx0_all_rk4 = []
t_all = []

xstart = np.array([x0, dx0])
x_rk4 = xstart

event_thres = 1e-2

sample_factor = 10

def f_spring_mass_damper(x):
    return np.array([
        x[1], 
        -c / m * x[1] - k / m * x[0]
        ])

def draw_anime(success):
    print('INTEGRATION END')
    print('TIME NOW: ', t)
    print()
    if success:
        save_name = "spring_mass_damper"
    else:
        save_name = "spring_mass_damper" + "_failed"
    Simulation2D().anime(
        t=t_all[::sample_factor], 
        x_states=[
            x0_all_rk4[::sample_factor]
        ], 
        ms=1000 * t_step * sample_factor,
        mission="Spring-Mass-Damper System", 
        sim_object="spring_mass_damper",
        sim_info={'ground':ground, 'wall': wall, 'ball_radius':0.4},
        save=False,
        save_name=save_name
    )
    exit()

while True:

    x_new_rk4 = Integrator().rk4(f_spring_mass_damper, x=x_rk4, h=t_step)
    x0_all_rk4.append(x_new_rk4[0])
    
    dx0_all_rk4.append(x_new_rk4[1])
    
    t = t + t_step
    t_all.append(t)

    x_rk4 = x_new_rk4
    
    if np.abs(x_rk4[0] - 0) < event_thres and np.abs(x_rk4[1] - 0) < event_thres:
        break
    
print('SYSTEM INTEGRATION SUCCEEDED...')
draw_anime(True)


exit()