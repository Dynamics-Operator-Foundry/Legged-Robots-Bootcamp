import sympy as sp, numpy as np
import matplotlib.pyplot as plt

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation2D as sim2D, RobotUtils as util

A = 0.5
B = A
a = 2
b = 1
r_x0 = 1
r_y0 = 0

t = np.arange(0,5,1e-3)
tf = 5

r_x =  A*np.sin(2*np.pi*a*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3)) + r_x0
r_y =  B*np.cos(2*np.pi*b*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3)) + r_y0
r_xdot =  2*np.pi*A*a*(30*t**4/tf**5 - 60*t**3/tf**4 + 30*t**2/tf**3)*np.cos(2*np.pi*a*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3))
r_ydot =  -2*np.pi*B*b*(30*t**4/tf**5 - 60*t**3/tf**4 + 30*t**2/tf**3)*np.sin(2*np.pi*b*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3))
r_xddot =  -2*np.pi*A*a**2*(30*t**4/tf**5 - 60*t**3/tf**4 + 30*t**2/tf**3)*np.sin(2*np.pi*a*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3)) + 2*np.pi*A*a*(120*t**3/tf**5 - 180*t**2/tf**4 + 60*t/tf**3)*np.cos(2*np.pi*a*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3))
r_yddot =  -2*np.pi*B*b**2*(30*t**4/tf**5 - 60*t**3/tf**4 + 30*t**2/tf**3)*np.sin(2*np.pi*b*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3)) - 2*np.pi*B*b*(120*t**3/tf**5 - 180*t**2/tf**4 + 60*t/tf**3)*np.sin(2*np.pi*b*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3))

# # Create a new figure with specified size
# plt.figure(figsize=(8, 10))  # Adjust size to accommodate both subplots

# # Subplot 1: x0 vs x1 for both Euler and RK4 methods
# plt.subplot(2, 1, 1)  # 2 rows, 1 column, 1st subplot
# plt.plot(t, r_xdot, label='lemniscate')

# plt.subplot(2, 1, 2)  # 2 rows, 1 column, 1st subplot
# plt.plot(t, r_ydot, label='lemniscate')


# plt.xlabel('r_x0')
# plt.ylabel('r_y0')
# plt.title('r_x0 vs r_y0')
# plt.grid(True)
# plt.legend()
# plt.show()

# exit()

r_all_ref = np.vstack([r_x, r_y, r_xdot, r_ydot, r_xddot, r_yddot])

r_ref = np.vstack([r_x, r_y])
rdot_ref = np.vstack([r_xdot, r_ydot])
rddot_ref = np.vstack([r_xddot, r_yddot])

# Now transform them into joint space

l1 = 1.0
l2 = 1.0

def forward_kinematics(q):
    theta0 = q[0]
    theta1 = q[1]
    R0 =  l1*np.cos(theta0) + l2*(-np.sin(theta0)*np.sin(theta1) + np.cos(theta0)*np.cos(theta1))
    R1 =  l1*np.sin(theta0) + l2*(np.sin(theta0)*np.cos(theta1) + np.sin(theta1)*np.cos(theta0))
    
    return np.array([R0, R1])

def solve_inverse_kinematics(q0, r_ref):
    q_k = q0
    dq_norm = np.inf
    
    # dr = J dq
    while dq_norm > 1e-6:
        theta0 = q_k[0]
        theta1 = q_k[1]
        
        J00 =  -l1*np.sin(theta0) - l2*np.sin(theta0 + theta1)
        J01 =  -l2*np.sin(theta0 + theta1)
        J10 =  l1*np.cos(theta0) + l2*np.cos(theta0 + theta1)
        J11 =  l2*np.cos(theta0 + theta1)
        J = np.array([[J00, J01], [J10, J11]])
        
        dr = r_ref - forward_kinematics(q_k)
        dq = np.linalg.pinv(J) @ dr
        q_k = q_k + dq
        # save_for_viz(q_k)
        
        dq_norm = np.linalg.norm(dq)
        
    return q_k

q0_all = []
q1_all = []
def get_q():
    r_ref_len = r_ref.shape[1]
    
    q_i = np.array([-0.5, 2.0])
    for i in range(r_ref_len):
        q_i = solve_inverse_kinematics(q0=q_i, r_ref=r_ref[:,i])

        q0_all.append(util().rad_2_pi_range(q_i[0]))
        q1_all.append(util().rad_2_pi_range(q_i[1]))
    
    return
get_q()

def get_Jacobian(q):
    theta0 = q[0]
    theta1 = q[1]
    
    J00 =  -l1*np.sin(theta0) - l2*np.sin(theta0 + theta1)
    J01 =  -l2*np.sin(theta0 + theta1)
    J10 =  l1*np.cos(theta0) + l2*np.cos(theta0 + theta1)
    J11 =  l2*np.cos(theta0 + theta1)
    J = np.array([[J00, J01], [J10, J11]])
    
    return J

def get_Jacobian_dot(q, qdot):
    theta0 = q[0]
    theta1 = q[1]
    theta0dot = qdot[0]
    theta1dot = qdot[1]
    
    Jdot00 =  -l1*theta0dot*np.cos(theta0) - l2*theta0dot*np.cos(theta0 + theta1) - l2*theta1dot*np.cos(theta0 + theta1)
    Jdot01 =  l2*(-theta0dot - theta1dot)*np.cos(theta0 + theta1)
    Jdot10 =  -l1*theta0dot*np.sin(theta0) - l2*theta0dot*np.sin(theta0 + theta1) - l2*theta1dot*np.sin(theta0 + theta1)
    Jdot11 =  l2*(-theta0dot - theta1dot)*np.sin(theta0 + theta1)
    Jdot = np.array([[Jdot00, Jdot01], [Jdot10, Jdot11]])
    
    return Jdot

q0dot_all = []
q1dot_all = []
q0ddot_all = []
q1ddot_all = []
def get_qdot():
    rdot_ref_len = rdot_ref.shape[1]
    for i in range(rdot_ref_len):
        q = [q0_all[i], q1_all[i]]
        J = get_Jacobian(q)
        
        # r_dot = J * q_dot
        qdot_i = np.linalg.pinv(J) @ rdot_ref[:,i]
        
        # r_ddot = J_dot * q_dot + J * q_ddot
        Jdot = get_Jacobian_dot(q=q,qdot=qdot_i)
        qddot_i = np.linalg.pinv(J) @ (rddot_ref[:,i] - Jdot @ np.linalg.pinv(J) @ rdot_ref[:,i])
        
        q0dot_all.append(qdot_i[0])
        q1dot_all.append(qdot_i[1])
        q0ddot_all.append(qddot_i[0])
        q1ddot_all.append(qddot_i[1])
    
    return
get_qdot()


q0ddot_all_test = []
q1ddot_all_test = []

def get_qddot():
    rddot_ref_len = rddot_ref.shape[1]
    qdot_previous_x = 0
    qdot_previous_y = 0
    for i in range(rddot_ref_len):
        q = [q0_all[i], q1_all[i]]
        qdot = [q0dot_all[i], q1dot_all[i]]
        
        J = get_Jacobian(q)
        
        qddot_x_test = (qdot[0] - qdot_previous_x) / 1e-3
        qddot_y_test = (qdot[1] - qdot_previous_y) / 1e-3
        qdot_previous_x = qdot[0]
        qdot_previous_y = qdot[1]
        q0ddot_all_test.append(qddot_x_test)
        q1ddot_all_test.append(qddot_y_test)
        
    return
    
# get_qddot()


# # Create a new figure with specified size
plt.figure(figsize=(8, 10))  # Adjust size to accommodate both subplots

# # Subplot 1: x0 vs x1 for both Euler and RK4 methods
plt.subplot(4, 1, 1)  # 2 rows, 1 column, 1st subplot
plt.plot(t, q0ddot_all, label='lemniscate')

plt.subplot(4, 1, 2)  # 2 rows, 1 column, 1st subplot
plt.plot(t, q1ddot_all, label='lemniscate')


# plt.subplot(4, 1, 3)  # 2 rows, 1 column, 1st subplot
# plt.plot(t, r_xddot, label='lemniscate')

# plt.subplot(4, 1, 4)  # 2 rows, 1 column, 1st subplot
# plt.plot(t, r_yddot, label='lemniscate')


# plt.subplot(4, 1, 3)  # 2 rows, 1 column, 1st subplot
# plt.plot(t, rddot_ref[0,:], label='lemniscate')

# plt.subplot(4, 1, 4)  # 2 rows, 1 column, 1st subplot
# plt.plot(t, rddot_ref[1,:], label='lemniscate')

plt.subplot(4, 1, 3)  # 2 rows, 1 column, 1st subplot
plt.plot(t, q0_all, label='lemniscate')

plt.subplot(4, 1, 4)  # 2 rows, 1 column, 1st subplot
plt.plot(t, q1_all, label='lemniscate')

plt.tight_layout()
plt.show()