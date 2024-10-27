import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import importlib
import time as time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util


l_hip = 0.4
l_thigh = 1.4
l_knee = 0.6

q = np.zeros(6)

phi0 = 0
theta1 = 0
theta2 = 0

phi0, theta1, theta2, dphi0, dtheta1, dtheta2 = q

t_all = []
t_all.append(0)

x0_all_rk4 = []
x1_all_rk4 = []
x2_all_rk4 = []
x3_all_rk4 = []
x4_all_rk4 = []
x5_all_rk4 = []

def save_data(q):
    x0_all_rk4.append(q[0])
    x1_all_rk4.append(q[1])
    x2_all_rk4.append(q[2])
    x3_all_rk4.append(q[3])
    x4_all_rk4.append(q[4])
    x5_all_rk4.append(q[5])
    
    return

save_data(q)

sim3D().anime(
    t=t_all,
    x_states=[
        x0_all_rk4,
        x1_all_rk4,
        x2_all_rk4,
        x3_all_rk4,
        x4_all_rk4,
        x5_all_rk4
    ],
    ms=10,
    mission='3D Leg',
    sim_object='3Dleg',
    sim_info={'l_hip': l_hip, 'l_thigh': l_thigh, 'l_knee': l_knee},
    save=True,
    save_name='3Dleg'
)

