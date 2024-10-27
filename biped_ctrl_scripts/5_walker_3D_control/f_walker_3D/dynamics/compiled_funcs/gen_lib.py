import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import importlib
import time as time
import pickle
from sympy.utilities.autowrap import autowrap
import sympy as sp
import shutil


func_name = 'P_R'

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
F = sp.symbols('F', real=True) 

q = [x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk]
qdot = [dx, dy, dz, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk]
qddot = [ddx, ddy, ddz, ddroll, ddpitch, ddyaw, ddroll_lh, ddpitch_lh, ddyaw_lh, ddpitch_lk, ddroll_rh, ddpitch_rh, ddyaw_rh, ddpitch_rk]

param = [w, l0, l1, l2, g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz, F]
all_variables = [*q, *qdot, *qddot, *param ]


# HERE LOAD AND COMPILE
folder_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ''))
with open(folder_path + "/binaries/"+func_name+".pkl", "rb") as file:
        func = pickle.load(file)
# print(func.shape)

func_variable = func.free_symbols
func_variable = sorted(func_variable, key=lambda x: all_variables.index(x))
print(func_variable)


t_now = time.time()
folder_name = folder_path + "/"+func_name

if not os.path.exists(folder_name):
    os.makedirs(folder_name)
else:
    shutil.rmtree(folder_name)
    os.makedirs(folder_name)
    
auto_get_A_ss_matrix = autowrap(func, args=func_variable, backend='cython', tempdir=folder_path + '/' + func_name + '/',)
print("COMPILATION END")

print("COMPILED TIME: ",time.time() - t_now)
file_name = folder_name+'/'+func_name+'.py'
with open(file_name, "w") as f:
    f.write(str(func_variable))  # Use .strip() to remove leading/trailing whitespace
exit()