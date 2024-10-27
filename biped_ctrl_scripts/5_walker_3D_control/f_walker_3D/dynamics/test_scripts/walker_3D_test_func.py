import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import importlib
import time as time
import pickle
from sympy.utilities.autowrap import autowrap
import sympy as sp

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util

folder_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../funcs'))
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

# l0 = 1;
# l1 = 0.5
# l2 = 0.5
# w = 0.1
# g=9.8
# mb=70
# mt=30
# mc=20
# Ibx=1 
# Iby=1 
# Ibz=1 
# Itx=1 
# Ity=1
# Itz=1
# Icx=1
# Icy=1 
# Icz=1

# var_global = {'yaw':0}
# q = np.zeros(14)
# qdot = np.zeros(14)
# qddot = np.zeros(14)
# param = np.array([w, l0, l1, l2])
# param_more = np.array([g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz])

# x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk = q

# dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk = qdot

# ddx,ddy,ddz,ddroll,ddpitch,ddyaw, ddroll_lh,ddpitch_lh,ddyaw_lh,ddpitch_lk, ddroll_rh,ddpitch_rh,ddyaw_rh,ddpitch_rk = qddot


# lala = get_B_ss_matrix(*q,*qdot,*qddot,*param,*param_more)
with open(folder_path + "/Mass_matrix.pkl", "rb") as file:
        mass_mat = pickle.load(file)
print(mass_mat.shape)

with open(folder_path + "/EOM_equations.pkl", "rb") as file:
        eom = pickle.load(file)
print(eom.shape)

# exit()

x, y, z = sp.symbols('x y z', real=True)
roll, pitch, yaw = sp.symbols('roll pitch yaw',real=True)
roll_lh, pitch_lh, yaw_lh = sp.symbols('roll_lh pitch_lh yaw_lh',real=True)
roll_rh, pitch_rh, yaw_rh = sp.symbols('roll_rh pitch_rh yaw_rh',real=True)
pitch_lk, pitch_rk = sp.symbols('pitch_lk pitch_rk',real=True)

w, l0, l1, l2 = sp.symbols('w l0 l1 l2', real=True)

dx, dy, dz = sp.symbols('dx dy dz', real=True)
droll, dpitch, dyaw = sp.symbols('droll dpitch dyaw',real=True)
droll_lh, dpitch_lh, dyaw_lh = sp.symbols('droll_lh dpitch_lh dyaw_lh',real=True)
droll_rh, dpitch_rh, dyaw_rh = sp.symbols('droll_rh dpitch_rh dyaw_rh',real=True)
dpitch_lk, dpitch_rk = sp.symbols('dpitch_lk dpitch_rk',real=True)

ddx, ddy, ddz = sp.symbols('ddx ddy ddz', real=True)
ddroll, ddpitch, ddyaw = sp.symbols('ddroll ddpitch ddyaw',real=True)
ddroll_lh, ddpitch_lh, ddyaw_lh = sp.symbols('ddroll_lh ddpitch_lh ddyaw_lh',real=True)
ddroll_rh, ddpitch_rh, ddyaw_rh = sp.symbols('ddroll_rh ddpitch_rh ddyaw_rh',real=True)
ddpitch_lk, ddpitch_rk = sp.symbols('ddpitch_lk ddpitch_rk',real=True)

g = sp.symbols('g', real=True)
Ibx, Iby, Ibz = sp.symbols('Ibx, Iby, Ibz', real=True)
Itx, Ity, Itz = sp.symbols('Itx, Ity, Itz', real=True)
Icx, Icy, Icz = sp.symbols('Icx, Icy, Icz', real=True)
mb, mt, mc = sp.symbols('mb, mt, mc', real=True) 

q = [x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk,]
qdot = [dx, dy, dz, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk]
qddot = [ddx, ddy, ddz, ddroll, ddpitch, ddyaw, ddroll_lh, ddpitch_lh, ddyaw_lh, ddpitch_lk, ddroll_rh, ddpitch_rh, ddyaw_rh, ddpitch_rk]

param = [w, l0, l1, l2, g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
all_variables = [*q, *qdot, *qddot, *param ]


# Extract the used variables from mass_mat

mass_mat_variable = mass_mat.free_symbols

mass_mat_variable = sorted(mass_mat_variable, key=lambda x: all_variables.index(x))

print(mass_mat_variable)


eom_variable = eom.free_symbols

eom_variable = sorted(eom_variable, key=lambda x: all_variables.index(x))

print(eom_variable)
# exit()


# # for what in all_variables:
#     # print(what)
    
# for what in mass_mat_variable:
#     var_global[str(what)] = 0
    
# print(var_global)

# # Check which of the 54 variables are actually used
# used_vars_list = [var for var in all_variables if var in used_variables]

# print(f"Used variables are: {used_vars_list}")


# # print(mass)
# print("Mass loaded successfully.")

# test_var = [*q,*qdot,*qddot,*param,*param_more]

t_now = time.time()
auto_get_A_ss_matrix = autowrap(mass_mat, args=mass_mat_variable, backend='cython', tempdir=folder_path + '/compiled_func',)
print("mass_mat OK")
auto_get_eom_matrix = autowrap(eom, args=eom_variable, backend='cython', tempdir=folder_path + '/compiled_func',)
print("eom OK")

print("COMPILED TIME: ",time.time() - t_now)
exit()



print(len(mass_mat_variable))
lala = np.zeros(len(mass_mat_variable))
t_now = time.time()
auto_get_B_ss_matrix(*lala)

# print(lala)
print(time.time()-t_now)