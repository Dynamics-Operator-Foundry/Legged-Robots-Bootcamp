import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation2D as sim2D, RobotUtils as util

l1 = 1.0
l2 = 1.0


t = 0
sample_factor = 10

# simulation environment
q0_all_rk4 = []
q1_all_rk4 = []

t_step = 1e-3
t_all = []

q0=np.pi/2
q1=0.0

q_i = np.array([q0, q1])

def forward_kinematics(q):
    theta0 = q[0]
    theta1 = q[1]
    R0 =  l1*np.cos(theta0) + l2*(-np.sin(theta0)*np.sin(theta1) + np.cos(theta0)*np.cos(theta1))
    R1 =  l1*np.sin(theta0) + l2*(np.sin(theta0)*np.cos(theta1) + np.sin(theta1)*np.cos(theta0))
    
    return np.array([R0, R1])

t = 0
def save_for_viz(q):

    for i in range(500):
        q0_all_rk4.append(q[0])
        q1_all_rk4.append(q[1])
        
        t_all.append(t)
    
    return

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
        save_for_viz(q_k)
        
        dq_norm = np.linalg.norm(dq)
        
    return q_k

r = np.array([1, 0.5])
save_for_viz(q_i)
solve_inverse_kinematics(q_i, r)

def draw_anime(success):
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "inverse_kinematics_numerical"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "inverse_kinematics_numerical" + "_failed"
    
    sim2D().anime(
        t=t_all[::sample_factor], 
        x_states=[
            q0_all_rk4[::sample_factor], 
            q1_all_rk4[::sample_factor]
        ], 
        ms=1000 * t_step * sample_factor,
        mission="Swing", 
        sim_object="double_pendulum",
        sim_info={'l1': l1, 'l2': l2},
        save=False,
        save_name=save_name
    )
    exit()
    
draw_anime(True)