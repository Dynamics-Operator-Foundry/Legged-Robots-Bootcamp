import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import importlib
import time as time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util

folder_path = os.path.abspath(os.path.join(os.path.dirname(__file__), './dynamics/funcs'))
sys.path.append(folder_path)

# List all Python files in the folder
module_names = [f[:-3] for f in os.listdir(folder_path) if f.endswith('.py')]

# Dynamically import all files and functions
for module_name in module_names:
    module = importlib.import_module(module_name)
    # Import all functions from each module dynamically
    for func in dir(module):
        if callable(getattr(module, func)):
            globals()[func] = getattr(module, func)

l0 = 1;
l1 = 0.5
l2 = 0.5
w = 0.1

q = np.zeros(14)
param = np.array([w, l0, l1, l2])

x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh, roll_rh,pitch_rh,yaw_rh, pitch_lk, pitch_rk = q

t_now = time.time()
p_H = sim3D().get_p_H(*q,*param) # head
print(p_H)
p_B = sim3D().get_p_B(*q,*param)
print(p_B)

p_LH = sim3D().get_p_LH(*q,*param)
print(p_LH)
p_RH = sim3D().get_p_RH(*q,*param)
print(p_RH)

p_LK = sim3D().get_p_LK(*q,*param)
print(p_LK)
p_RK = sim3D().get_p_RK(*q,*param)
print(p_RK)

p_LA = sim3D().get_p_LA(*q,*param)
print(p_LA)
p_RA = sim3D().get_p_RA(*q,*param)
print(p_RA)

p_Torso = sim3D().get_p_Torso(*q,*param)
print(p_Torso)

p_Thigh_L = sim3D().get_p_Thigh_L(*q,*param)
print(p_Thigh_L)
p_Thigh_R = sim3D().get_p_Thigh_R(*q,*param)
print(p_Thigh_R)

p_Calf_L = sim3D().get_p_Calf_L(*q,*param)
print(p_Calf_L)
p_Calf_R = sim3D().get_p_Calf_R(*q,*param)
print(p_Calf_R)

print("==========")
p_Hip_I = sim3D().get_p_Hip_L_init(0,0,0,*q[3:],*param)
print(p_Hip_I)

q[0:3] = p_Hip_I

print(q[0:3])

t_all = []
t_all.append(0)

x0_all_rk4 = []
x1_all_rk4 = []
x2_all_rk4 = []
x3_all_rk4 = []
x4_all_rk4 = []
x5_all_rk4 = []
x6_all_rk4 = []
x7_all_rk4 = []
x8_all_rk4 = []
x9_all_rk4 = []
x10_all_rk4 = []
x11_all_rk4 = []
x12_all_rk4 = []
x13_all_rk4 = []

def save_data(q):
    x0_all_rk4.append(q[0])
    x1_all_rk4.append(q[1])
    x2_all_rk4.append(q[2])
    x3_all_rk4.append(q[3])
    x4_all_rk4.append(q[4])
    x5_all_rk4.append(q[5])
    x6_all_rk4.append(q[6])
    x7_all_rk4.append(q[7])
    x8_all_rk4.append(q[8])
    x9_all_rk4.append(q[9])
    x10_all_rk4.append(q[10])
    x11_all_rk4.append(q[11])
    x12_all_rk4.append(q[12])
    x13_all_rk4.append(q[13])
    
    return
save_data(q)

print('time:', (time.time() - t_now) * 500)

# exit()

sim3D().anime(
    t=t_all,
    x_states=[
        x0_all_rk4,
        x1_all_rk4,
        x2_all_rk4,
        x3_all_rk4,
        x4_all_rk4,
        x5_all_rk4,
        x6_all_rk4,
        x7_all_rk4,
        x8_all_rk4,
        x9_all_rk4,
        x10_all_rk4,
        x11_all_rk4,
        x12_all_rk4,
        x13_all_rk4,
    ],
    ms=10,
    mission='Walk',
    sim_object='3Dwalker',
    sim_info={'w': w, 'l0': l0, 'l1': l1, 'l2': l2},
    save=False,
    save_name='3Dwalker'
)

