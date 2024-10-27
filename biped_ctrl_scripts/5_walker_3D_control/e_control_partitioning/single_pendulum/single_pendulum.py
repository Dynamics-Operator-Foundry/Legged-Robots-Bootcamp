import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation2D as sim2D, RobotUtils as util
g = 9.81
l1 = 1.0
l2 = 0.5
m1 = 1.0
m2 = 0.5
M = 1.0
I1 = 0.02
I2 = 0.02 / 8

t = 0
sample_factor = 10

# simulation environment
q0_all_rk4 = []

t_step = 1e-3
t_all = []

q0=-0.001
u0=-0.0
x_rk4 = np.array([q0, u0])

def draw_anime(success):
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "single_pendulum"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "single_pendulum" + "_failed"
    
    sim2D().anime(
        t=t_all[::sample_factor], 
        x_states=[
            q0_all_rk4[::sample_factor]
        ], 
        ms=1000 * t_step * sample_factor,
        mission="Swing", 
        sim_object="single_pendulum",
        sim_info={'l1': l1},
        save=False,
        save_name=save_name
    )
    exit()

def f_single_pendulum(x):
    
    theta0 = x[0]
    omega0 = x[1]
    
    A = 1.0*I1 + 0.25*l1**2*m1
    b = g*l1*m1*np.cos(theta0)/2
    
    alpha0 = -b/A
    
    return np.array([
        omega0, 
        alpha0
        ])
    
t_lim = 10.0

while True:
    x_rk4_new = inte().rk4(f_single_pendulum, x=x_rk4, h=t_step)
            
    q0_all_rk4.append(x_rk4_new[0])
    
    t = t + t_step
    t_all.append(t)

    x_rk4 = x_rk4_new
    
    if t > t_lim:
        break
    
draw_anime(True)