import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator

m = 1
c = 1
k = 1
wall = -2

t_step = 1/1000

ground = 0

x0 = 0.0
x1 = 0.0
dx0 = 0.0
dx1 = 0.0
t = 0

x0_all_rk4 = []
x1_all_rk4 = []
dx0_all_rk4 = []
dx1_all_rk4 = []

t_all = []

xstart = np.array([x0, x1, dx0, dx1])
x_rk4 = xstart

event_thres = 1e-2

sample_factor = 10

Kp = 1 * np.identity(2)
Kd = 2 * np.sqrt(Kp)

M = np.array([[1, 0.1],[0.1, 2]])
C = np.array([[0.2, 0],[0, 0.1]])
K = np.array([[5, 1],[1, 10]])

M_d = np.array([[0.04, 0.01],[0.01, 0.04]])
C_d = np.array([[0.04, 0.01],[0.01, 0.01]])
K_d = np.array([[0.04, 0.01],[0.01, 0.01]])

M_hat = M + 4 * M_d
C_hat = C + C_d
K_hat = K + K_d

x0_d = 0.5
x1_d = 0.1

def tau_control(x):
    x_d = np.array([x0_d, x1_d])
    q_d = np.array([x_d[0], x_d[1]])
    q = np.array([x[0], x[1]])
    qdot = np.array([x[2], x[3]])
    
    tau = M_hat @ (-Kp @ (q - q_d) - Kd @ qdot) + C_hat @ qdot + K_hat @ q
    return tau

def f_fictitious(x, tau):
    x_d = np.array([x0_d, x1_d])
    q = np.array([x[0], x[1]])
    qdot = np.array([x[2], x[3]])
    
    b = tau - C @ qdot - K @ q
    A = M
    
    qddot = np.linalg.solve(A, b)
    
    return np.array([
        qdot[0], 
        qdot[1],
        qddot[0],
        qddot[1]
        ])

while True:
    tau = tau_control(x_rk4)
    x_new_rk4 = Integrator().rk4(f_fictitious, x=x_rk4, u=tau, h=t_step, ctrl_on=True)
    
    x0_all_rk4.append(x_new_rk4[0])
    x1_all_rk4.append(x_new_rk4[1])
    
    
    t = t + t_step
    t_all.append(t)

    x_rk4 = x_new_rk4
    
    if (np.abs(x_rk4[0] - x0_d) < event_thres and np.abs(x_rk4[1] - x1_d) < event_thres) or t > 40.0 :
        break
    
print('SYSTEM INTEGRATION SUCCEEDED...')

# Create a new figure with specified size
plt.figure(figsize=(8, 10))  # Adjust size to accommodate both subplots

plt.subplot(2, 1, 1)  # 2 rows, 1 column, 1st subplot
plt.plot(t_all, x0_all_rk4)
plt.xlabel('t')
plt.ylabel('x0')
plt.title('t vs x0')
plt.grid(True)

plt.subplot(2, 1, 2)  # 2 rows, 1 column, 1st subplot
plt.plot(t_all, x1_all_rk4)
plt.xlabel('t')
plt.ylabel('x1')
plt.title('t vs x1')
plt.grid(True)

# Adjust layout to prevent overlap
plt.tight_layout()
plt.show()


exit()