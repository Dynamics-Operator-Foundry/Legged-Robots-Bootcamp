import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
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

q0=-np.pi/2 - 0.5
q1=0.0
u0=-0.0
u1=-0.0

x_rk4 = np.array([q0, q1, u0, u1])

Kp = 100 * np.identity(2)
Kd = 2 * np.sqrt(Kp)

event_thres = 1e-2

t0 = 0
tf = 3
t0_p0 = 0
tf_p0 = 1.5
t0_p1 = 1.5
tf_p1 = 3.0

def traj_setting():
    
    # theta0
    al0_0 =  -np.pi/2 - 0.5
    al0_1 =  0
    al0_2 =  0.333333333333333
    al0_3 =  -0.0740740740740741
    
    t_local = np.arange(t0, tf, t_step)
    q0_ref = al0_0 + al0_1*t_local + al0_2*t_local**2 + al0_3*t_local**3
    q0dot_ref = al0_1 + 2*al0_2*t_local + 3*al0_3*t_local**2
    q0ddot_ref = 2*al0_2 + 6*al0_3*t_local
    
    # theta1
    al1_p0_0 =  0
    al1_p0_1 =  0
    al1_p0_2 =  1.33333333333333 - 0.666666666666667*np.pi
    al1_p0_3 =  -0.592592592592593 + 0.296296296296296*np.pi
    al1_p1_0 =  -4.0 + 2.0*np.pi
    al1_p1_1 =  8.0 - 4.0*np.pi
    al1_p1_2 =  -4.0 + 2.0*np.pi
    al1_p1_3 =  0.592592592592593 - 0.296296296296296*np.pi
    
    t_local = np.arange(t0_p0, tf_p0, t_step)
    q1_ref_p0 = al1_p0_0 + al1_p0_1*t_local + al1_p0_2*t_local**2 + al1_p0_3*t_local**3
    q1dot_ref_p0 = al1_p0_1 + 2*al1_p0_2*t_local + 3*al1_p0_3*t_local**2
    q1ddot_ref_p0 = 2*al1_p0_2 + 6*al1_p0_3*t_local
    
    t_local = np.arange(t0_p1, tf_p1, t_step)
    q1_ref_p1 = al1_p1_0 + al1_p1_1*t_local + al1_p1_2*t_local**2 + al1_p1_3*t_local**3
    q1dot_ref_p1 = al1_p1_1 + 2*al1_p1_2*t_local + 3*al1_p1_3*t_local**2
    q1ddot_ref_p1 = 2*al1_p1_2 + 6*al1_p1_3*t_local
    
    q1_ref = np.hstack([q1_ref_p0, q1_ref_p1])
    q1dot_ref = np.hstack([q1dot_ref_p0, q1dot_ref_p1])
    q1ddot_ref = np.hstack([q1ddot_ref_p0, q1ddot_ref_p1])
    
    q_ref = np.vstack([q0_ref, q1_ref, q0dot_ref, q1dot_ref, q0ddot_ref, q1ddot_ref])
    
    # print(q_ref.shape)
    # print(q_ref[:,1])
    
    return q_ref

# exit()

def draw_anime(success):
    print('INTEGRATION END')
    print('TIME NOW: ', t)
    print()
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "double_pendulum_traj_tracking"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "double_pendulum_traj_tracking" + "_failed"
    
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