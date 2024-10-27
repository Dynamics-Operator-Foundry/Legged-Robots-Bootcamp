import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation2D as sim2D, RobotUtils as util
g = 9.81
l1 = 1.0
l2 = 0.5
m1 = 1.0
m2 = 0.5
I1 = 0.02
I2 = 0.02 / 8

t = 0
sample_factor = 10

# simulation environment
q0_all_rk4 = []
q1_all_rk4 = []

t_step = 1e-3
t_all = []

q0=np.pi/2+0.01
q1=0.0
u0=-0.0
u1=-0.0
x_rk4 = np.array([q0, q1, u0, u1])

def draw_anime(success):
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "double_pendulum"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "double_pendulum" + "_failed"
    
    sim2D().anime(
        t=t_all[::sample_factor], 
        x_states=[
            q0_all_rk4[::sample_factor], 
            q1_all_rk4[::sample_factor]
        ], 
        ms=1000 * t_step * sample_factor,
        mission="Swing", 
        sim_object="double_pendulum",
        sim_info={'l1': l1, 'l2': l2},
        save=False,
        save_name=save_name
    )
    exit()

def f_double_pendulum(x):
    
    theta0 = x[0]
    theta1 = x[1]
    omega0 = x[2]
    omega1 = x[3]
    
    M00 =  1.0*I1 + 1.0*I2 + 0.25*l1**2*m1 + 1.0*l1**2*m2 + 1.0*l1*l2*m2*np.cos(theta1) + 0.25*l2**2*m2
    M01 =  1.0*I2 + 0.5*l1*l2*m2*np.cos(theta1) + 0.25*l2**2*m2
    M10 =  1.0*I2 + 0.5*l1*l2*m2*np.cos(theta1) + 0.25*l2**2*m2
    M11 =  1.0*I2 + 0.25*l2**2*m2
    
    b_0 =  0.5*g*l1*m1*np.cos(theta0) + 1.0*g*l1*m2*np.cos(theta0) + 0.5*g*l2*m2*np.cos(theta0 + theta1) - 1.0*l1*l2*m2*omega0*omega1*np.sin(theta1) - 0.5*l1*l2*m2*omega1**2*np.sin(theta1)
    b_1 =  0.5*l2*m2*(g*np.cos(theta0 + theta1) + l1*omega0**2*np.sin(theta1))
    
    A = np.array([[M00, M01],[M10, M11]])
    b = np.array([-b_0, -b_1])
    x_new = np.linalg.solve(A,b)
    
    return np.array([
        omega0, 
        omega1, 
        x_new[0], 
        x_new[1] 
        ])
    
t_lim = 10.0

while True:
    x_rk4_new = inte().rk4(f_double_pendulum, x=x_rk4, h=t_step)
            
    q0_all_rk4.append(x_rk4_new[0])
    q1_all_rk4.append(x_rk4_new[1])
    
    t = t + t_step
    t_all.append(t)

    x_rk4 = x_rk4_new
    
    if t > t_lim:
        break
    
draw_anime(True)