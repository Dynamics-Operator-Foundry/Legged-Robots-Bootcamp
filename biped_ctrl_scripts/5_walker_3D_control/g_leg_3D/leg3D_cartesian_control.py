import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation2D as sim2D, RobotUtils as util

g = 9.81
l1 = 1.0
l2 = 1.0
m1 = 1.0
m2 = 1.0
I1 = 0.5
I2 = 0.5

t = 0
sample_factor = 10

# simulation environment
q0_all_rk4 = []
q1_all_rk4 = []

t_step = 1e-3
t_all = []
u_all = []

q0=-0.5
q1=2.0
u0=-0.0
u1=-0.0

x_rk4 = np.array([q0, q1, u0, u1])

Kp = 100 * np.identity(2)
Kd = 2 * np.sqrt(Kp)

event_thres = 1e-2

t0 = 0
tf = 5

def cartesian_traj_setting():
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
    r_xddot =  -4*np.pi**2*A*a**2*(30*t**4/tf**5 - 60*t**3/tf**4 + 30*t**2/tf**3)**2*np.sin(2*np.pi*a*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3)) + 2*np.pi*A*a*(120*t**3/tf**5 - 180*t**2/tf**4 + 60*t/tf**3)*np.cos(2*np.pi*a*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3))
    r_yddot =  -4*np.pi**2*B*b**2*(30*t**4/tf**5 - 60*t**3/tf**4 + 30*t**2/tf**3)**2*np.cos(2*np.pi*b*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3)) - 2*np.pi*B*b*(120*t**3/tf**5 - 180*t**2/tf**4 + 60*t/tf**3)*np.sin(2*np.pi*b*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3))
    
    # r_ref = np.vstack([r_x, r_y, r_xdot, r_ydot, r_xddot, r_yddot])
    r_ref = np.vstack([r_x, r_y])
    rdot_ref = np.vstack([r_xdot, r_ydot])
    rddot_ref = np.vstack([r_xddot, r_yddot])
    
    return r_ref, rdot_ref, rddot_ref

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

def get_q(r_ref):
    r_ref_len = r_ref.shape[1]
    q_i = np.array([-0.5, 2.0])
    
    q0_all = []
    q1_all = []
    
    for i in range(r_ref_len):
        q_i = solve_inverse_kinematics(q0=q_i, r_ref=r_ref[:,i])

        q0_all.append(util().rad_2_pi_range(q_i[0]))
        q1_all.append(util().rad_2_pi_range(q_i[1]))
    
    return q0_all, q1_all

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

def get_qdot(rdot_ref, rddot_ref, q0_all, q1_all):
    rdot_ref_len = rdot_ref.shape[1]
    
    q0dot_all = []
    q1dot_all = []
    q0ddot_all = []
    q1ddot_all = []
    
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
    
    return q0dot_all, q1dot_all, q0ddot_all, q1ddot_all

def traj_setting():

    r_ref, rdot_ref, rddot_ref = cartesian_traj_setting()
    q0_all, q1_all = get_q(r_ref)
    q0dot_all, q1dot_all, q0ddot_all, q1ddot_all = get_qdot(rdot_ref, rddot_ref, q0_all, q1_all)
    
    q_ref = np.vstack([q0_all, q1_all, q0dot_all, q1dot_all, q0ddot_all, q1ddot_all])
    
    return q_ref

# exit()

def draw_anime(success):
    print('INTEGRATION END')
    print('TIME NOW: ', t)
    print()
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "double_pendulum_cartesian_control"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "double_pendulum_cartesian_control" + "_failed"
    
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

def generate_noise():
    mean = 0
    std_dev = 0.1

    return np.random.normal(mean, std_dev, 1)[0]

def generate_noise_matrix(n, m):
    mean = 0
    std_dev = 0.1
    return np.random.normal(mean, std_dev, (n, m))

def tau_control(x, x_ref):
    
    theta0_ref = x_ref[0]
    theta1_ref = x_ref[1]
    qq_ref = np.array([theta0_ref, theta1_ref])
    
    theta0dot_ref = x_ref[2]
    theta1dot_ref = x_ref[3]
    qqdot_ref = np.array([theta0dot_ref, theta1dot_ref])
    
    theta0ddot_ref = x_ref[4]
    theta1ddot_ref = x_ref[5]
    qqddot_ref = np.array([theta0ddot_ref, theta1ddot_ref])
    
    theta0 = util().rad_2_pi_range(x[0] + generate_noise()) 
    theta1 = util().rad_2_pi_range(x[1] + generate_noise()) 
    
    omega0 = x[2] + generate_noise()
    omega1 = x[3] + generate_noise()
    
    qq = np.array([theta0, theta1])
    qqdot = np.array([omega0, omega1])
    
    # M * qddot + C(qdot) + G(q)
    # = tau
    # = M^ * (-Kp * (q - q_ref) - Kd * qdot) + C^ * qdot + G^ * q 
    
    M00 =  1.0*I1 + 1.0*I2 + 0.25*l1**2*m1 + 1.0*l1**2*m2 + 1.0*l1*l2*m2*np.cos(theta1) + 0.25*l2**2*m2
    M01 =  1.0*I2 + 0.5*l1*l2*m2*np.cos(theta1) + 0.25*l2**2*m2
    M10 =  1.0*I2 + 0.5*l1*l2*m2*np.cos(theta1) + 0.25*l2**2*m2
    M11 =  1.0*I2 + 0.25*l2**2*m2
    
    M = np.array([[M00, M01],[M10, M11]])
    M_hat = M + generate_noise_matrix(2,2)
    
    C = np.zeros([2,2])
    C_hat = C + generate_noise_matrix(2,2)
    
    g_0 = 0.5*g*l1*m1*np.cos(theta0) + 1.0*g*l1*m2*np.cos(theta0) + 0.5*g*l2*m2*np.cos(theta0 + theta1) - 1.0*l1*l2*m2*omega0*omega1*np.sin(theta1) - 0.5*l1*l2*m2*omega1**2*np.sin(theta1)
    g_1 =  0.5*l2*m2*(g*np.cos(theta0 + theta1) + l1*omega0**2*np.sin(theta1))
    
    G = np.array([g_0, g_1])
    G_hat = G + generate_noise_matrix(2,1).transpose().flatten()
    
    tau = M_hat @ (qqddot_ref - Kp @ (qq-qq_ref) - Kd @ (qqdot - qqdot_ref)) + C_hat @ qqdot + G_hat
    
    return tau

def f_double_pendulum(x, tau):
    
    theta0 = util().rad_2_pi_range(x[0] + generate_noise()) 
    theta1 = util().rad_2_pi_range(x[1] + generate_noise()) 
    
    omega0 = x[2] + generate_noise()
    omega1 = x[3] + generate_noise()
    
    q = np.array([theta0, theta1])
    qdot = np.array([omega0, omega1])
    
    M00 =  1.0*I1 + 1.0*I2 + 0.25*l1**2*m1 + 1.0*l1**2*m2 + 1.0*l1*l2*m2*np.cos(theta1) + 0.25*l2**2*m2
    M01 =  1.0*I2 + 0.5*l1*l2*m2*np.cos(theta1) + 0.25*l2**2*m2
    M10 =  1.0*I2 + 0.5*l1*l2*m2*np.cos(theta1) + 0.25*l2**2*m2
    M11 =  1.0*I2 + 0.25*l2**2*m2
    
    M = np.array([[M00, M01],[M10, M11]])
    
    g_0 = 0.5*g*l1*m1*np.cos(theta0) + 1.0*g*l1*m2*np.cos(theta0) + 0.5*g*l2*m2*np.cos(theta0 + theta1) - 1.0*l1*l2*m2*omega0*omega1*np.sin(theta1) - 0.5*l1*l2*m2*omega1**2*np.sin(theta1)
    g_1 =  0.5*l2*m2*(g*np.cos(theta0 + theta1) + l1*omega0**2*np.sin(theta1))
    
    C = np.zeros([2,2])
    G = np.array([g_0, g_1])
    
    # M * qddot + C(qdot) + G(q)
    # = tau
    # = M^ * (-Kp * (q - q_ref) - Kd * qdot) + C^ * qdot + G^ * q
    b = tau - C @ qdot - G
    A = M
    qddot = np.linalg.solve(A,b)
 
    return np.array([
        omega0, 
        omega1, 
        qddot[0], 
        qddot[1] 
        ])
    
t_lim = 10.0

q_ref = traj_setting()
no_all_steps = q_ref.shape[1]

# print(no_all_steps)
# exit()

for i in range(no_all_steps):
    
    tau = tau_control(x_rk4, q_ref[:,i])
    x_rk4_new = inte().rk4(f_double_pendulum, x=x_rk4, u=tau, h=t_step, ctrl_on=True)
    
    q0_all_rk4.append(x_rk4_new[0])
    q1_all_rk4.append(x_rk4_new[1])
    
    t = t + t_step
    t_all.append(t)
    u_all.append(tau)

    x_rk4 = x_rk4_new
    theta0_current = util().rad_2_pi_range(x_rk4[0])
    theta1_current = util().rad_2_pi_range(x_rk4[1])
    
draw_anime(True)

print('SYSTEM INTEGRATION SUCCEEDED...')

# Create a new figure with specified size
plt.figure(figsize=(8, 10))  # Adjust size to accommodate both subplots

plt.subplot(1, 1, 1)  # 2 rows, 1 column, 1st subplot
plt.plot(t_all, u_all)
plt.xlabel('t')
plt.ylabel('u')
plt.title('t vs u')
plt.grid(True)

# Adjust layout to prevent overlap
plt.tight_layout()
plt.show()