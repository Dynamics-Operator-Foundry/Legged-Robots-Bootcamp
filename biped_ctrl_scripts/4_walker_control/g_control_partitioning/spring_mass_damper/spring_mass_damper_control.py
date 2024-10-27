import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator

m = 1
c = 1
k = 1
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

Kp = 1

def force_control(x):
    Kd = -c + 2 * np.sqrt(m * (k + Kp))
    return Kp * (-x[0]) + Kd * (-x[1])

def f_spring_mass_damper(x,u):    
    return np.array([
        x[1], 
        (-c * x[1] - k * x[0] + u) / m
        ])

def draw_anime(success):
    print('INTEGRATION END')
    print('TIME NOW: ', t)
    print()
    if success:
        save_name = "spring_mass_damper_control"
    else:
        save_name = "spring_mass_damper_control" + "_failed"
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
    u = force_control(x_rk4)
    x_new_rk4 = Integrator().rk4(f_spring_mass_damper, x=x_rk4, u=u, h=t_step, ctrl_on=True)
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