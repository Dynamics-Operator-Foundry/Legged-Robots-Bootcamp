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

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../funcs/compiled_func')))

# import wrapper_module_0
from wrapper_module_0 import autofunc_c as lala
from wrapper_module_1 import autofunc_c as lala1
t_now = time.time()
temp_var = np.zeros(27)
lala(*temp_var)
temp_var = np.zeros(56)
lala1(*temp_var)

print(time.time() - t_now)

class testhaha():
    def __init__(self):
        return
    
    def get_A(self):
        temp_var = np.zeros(56)
        test = lala1(*temp_var)
        
        return test
    

exit()


# autofunc_c(temp_var)
