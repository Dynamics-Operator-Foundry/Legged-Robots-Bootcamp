import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util, Leg3DModelling as leg3D
# from sympy import *


t = 0
sample_factor = 10

# simulation environment
q0_all_rk4 = []
q1_all_rk4 = []
q2_all_rk4 = []

t_step = 1e-3
t_all = []

q0=0.0
q1=0.67
q2=-1.3

q_i = np.array([q0, q1, q2])

l_hip = 0.4
l_thigh = 1.4
l_knee = 1.2

def forward_kinematics(q):
    phi0 = q[0]
    theta1 = q[1]
    theta2 = q[2]
    
    r_E = np.array([[-l_knee*np.sin(theta1 + theta2) - l_thigh*np.sin(theta1)], [l_hip*np.cos(phi0) + l_knee*np.sin(phi0)*np.cos(theta1 + theta2) + l_thigh*np.sin(phi0)*np.cos(theta1)], [l_hip*np.sin(phi0) - l_knee*np.cos(phi0)*np.cos(theta1 + theta2) - l_thigh*np.cos(phi0)*np.cos(theta1)]])
    
    return r_E.flatten()


def get_Jacobian(q):
    phi0 = q[0]
    theta1 = q[1]
    theta2 = q[2]
    
    # print(q.shape)
    
    J = np.array([
        [0, -l_knee*np.cos(theta1 + theta2) - l_thigh*np.cos(theta1), -l_knee*np.cos(theta1 + theta2)], 
        [-l_hip*np.sin(phi0) + l_knee*np.cos(phi0)*np.cos(theta1 + theta2) + l_thigh*np.cos(phi0)*np.cos(theta1), -(l_knee*np.sin(theta1 + theta2) + l_thigh*np.sin(theta1))*np.sin(phi0), -l_knee*np.sin(phi0)*np.sin(theta1 + theta2)], 
        [l_hip*np.cos(phi0) + l_knee*np.sin(phi0)*np.cos(theta1 + theta2) + l_thigh*np.sin(phi0)*np.cos(theta1), (l_knee*np.sin(theta1 + theta2) + l_thigh*np.sin(theta1))*np.cos(phi0), l_knee*np.sin(theta1 + theta2)*np.cos(phi0)]
        ])
    
    return J

# q = np.zeros(3)
# get_Jacobian(q)
# exit()

def save_for_viz(q):
    t = 0
    for i in range(500):
        q0_all_rk4.append(q[0])
        q1_all_rk4.append(q[1])
        q2_all_rk4.append(q[2])
        
        t_all.append(t)
    
    return

def solve_inverse_kinematics(q0, r_ref):
    q_k = q0
    dq_norm = np.inf
    
    # dr = J dq
    while dq_norm > 1e-6:
        
        J = get_Jacobian(q_k)
        
        dr = r_ref - forward_kinematics(q_k)
        print(np.linalg.norm(dr))
        print(dq_norm)
        print()
                
        dq = np.linalg.pinv(J) @ dr
        q_k = q_k + dq
        save_for_viz(q_k)
                
        dq_norm = np.linalg.norm(dq)
        
    return q_k

r = np.array([0, 0, -1])
save_for_viz(q_i)
solve_inverse_kinematics(q_i, r)

def draw_anime(success):
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "inverse_kinematics_numerical"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "inverse_kinematics_numerical" + "_failed"
    
    sim3D().anime(
        t=t_all[::sample_factor],
        x_states=[
            q0_all_rk4[::sample_factor],
            q1_all_rk4[::sample_factor],
            q2_all_rk4[::sample_factor]
        ],
        ms=10,
        mission='3D Leg',
        sim_object='3Dleg',
        sim_info={'l_hip': l_hip, 'l_thigh': l_thigh, 'l_knee': l_knee},
        save=False,
        save_name='3Dleg_inverse_kinematics'
    )
    exit()
    
draw_anime(True)