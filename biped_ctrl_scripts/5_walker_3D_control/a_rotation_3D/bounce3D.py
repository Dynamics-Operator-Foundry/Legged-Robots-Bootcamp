import matplotlib.pyplot as plt
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util

x0 = 0
x1 = 0
x2 = 4
dx0 = 0.1
dx1 = -0.4
dx2 = 1

xstart = np.array([x0, x1, x2, dx0, dx1, dx2])

c = 0.02
m = 1
g = 9.81
e = 0.9

t_start = 0
t_end = 5
t_step = 1/1000

ground = 0


# x = [x0, x1, dx0, dx1]
# dx = f(x) = [dx0, dx1, ddx0, ddx1]

def f(x):
    v_norm = np.sqrt(x[3]**2 + x[4]**2 + x[5]**2)
    return np.array([
        x[3], 
        x[4],
        x[5], 
        - c / m * v_norm * x[3], 
        - c / m * v_norm * x[4], 
        - c / m * v_norm * x[5] - g
        ])

t = np.arange(t_start, t_end, t_step)


x0_all_rk4 = []
x1_all_rk4 = []
x2_all_rk4 = []
dx0_all_rk4 = []
dx1_all_rk4 = []
dx2_all_rk4 = []

event_thres = 1e-3

sample_factor = 10

x_euler = xstart
x_rk4 = xstart

for t_ in t:

    x_new_rk4 = inte().rk4(func=f,x=x_rk4, h=t_step)

    x0_all_rk4.append(x_new_rk4[0])
    x1_all_rk4.append(x_new_rk4[1])
    x2_all_rk4.append(x_new_rk4[2])
    dx0_all_rk4.append(x_new_rk4[3])
    dx1_all_rk4.append(x_new_rk4[4])
    dx2_all_rk4.append(x_new_rk4[5])
    
    x_rk4 = x_new_rk4
    
    if x_rk4[2] - ground < 0 and x_rk4[5] < 0:
        x_rk4[5] = (-1) * e * x_rk4[5]

# exit()

sim3D().anime(
    t=t[::sample_factor], 
    x_states=[x0_all_rk4[::sample_factor], x1_all_rk4[::sample_factor], x2_all_rk4[::sample_factor]], 
    ms=1000 * t_step * sample_factor,
    mission="Bounce", 
    sim_object="ball",
    sim_info={'ground':ground},
    save=False,
    save_name='bounce_3D'
)

exit()

# Create a new figure with specified size
plt.figure(figsize=(8, 10))  # Adjust size to accommodate both subplots

# Subplot 1: x0 vs x1 for both Euler and RK4 methods
plt.subplot(5, 1, 1)  # 2 rows, 1 column, 1st subplot
plt.plot(x0_all_euler, x1_all_euler, label='Euler Method')
plt.plot(x0_all_rk4, x1_all_rk4, label='RK4 Method')
plt.xlabel('x0')
plt.ylabel('x1')
plt.title('x0 vs x1')
plt.grid(True)
plt.legend()

# Subplot 2: t vs x0 for RK4 method
plt.subplot(5, 1, 2)  # 2 rows, 1 column, 2nd subplot
plt.plot(t, x0_all_rk4, label='RK4 Method')
plt.xlabel('t')
plt.ylabel('x0')
plt.title('t vs x0')
plt.grid(True)
plt.legend()

plt.subplot(5, 1, 3)  # 2 rows, 1 column, 2nd subplot
plt.plot(t, x1_all_rk4, label='RK4 Method')
plt.xlabel('t')
plt.ylabel('x1')
plt.title('t vs x1')
plt.grid(True)
plt.legend()

plt.subplot(5, 1, 4)  # 2 rows, 1 column, 2nd subplot
plt.plot(t, dx0_all_rk4, label='RK4 Method')
plt.xlabel('t')
plt.ylabel('dx0')
plt.title('t vs dx0')
plt.grid(True)
plt.legend()

plt.subplot(5, 1, 5)  # 2 rows, 1 column, 2nd subplot
plt.plot(t, dx1_all_rk4, label='RK4 Method')
plt.xlabel('t')
plt.ylabel('dx1')
plt.title('t vs dx1')
plt.grid(True)
plt.legend()

# Adjust layout to prevent overlap
plt.tight_layout()
plt.show()
