import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import importlib
import time as time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util, Leg3DModelling as leg3D


l_hip = 0.08
l_thigh = 0.213
l_knee = 0.213

q = np.zeros(6)

phi0 = 0.2
theta1 = 0.9
theta2 = -1.50

q = np.array([phi0, theta1, theta2, 0, 0, 0])

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
    
    return

save_data(q)

p = leg3D().get_p_foot_3Dleg(q, sim_info={'l_hip': l_hip, 'l_thigh': l_thigh, 'l_knee': l_knee})
print(p)
exit()

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
    save=False,
    save_name='3Dleg'
)

