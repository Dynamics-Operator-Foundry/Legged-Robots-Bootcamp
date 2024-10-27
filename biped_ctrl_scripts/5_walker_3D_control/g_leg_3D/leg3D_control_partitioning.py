import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation2D as sim2D, RobotUtils as util

g = 9.81
l1 = 1.0
l2 = 0.5
m1 = 1.0
m2 = 0.5
I1 = 0.02
I2 = 0.02 / 8

t = 0
sample_factor = 10

# simulation environment
q0_all_rk4 = []
q1_all_rk4 = []

t_step = 1e-3
t_all = []
u_all = []

q0=-np.pi
q1=0.0
u0=-0.0
u1=-0.0

x_rk4 = np.array([q0, q1, u0, u1])

q0_ref = np.pi/2
q1_ref = 0

q_ref = np.array([q0_ref, q1_ref])

Kp = 100 * np.identity(2)
Kd = 2 * np.sqrt(Kp)

event_thres = 1e-2

def draw_anime(success):
    print('INTEGRATION END')
    print('TIME NOW: ', t)
    print()
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "double_pendulum_control_partitioning"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "double_pendulum_control_partitioning" + "_failed"
    
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

def tau_control(x):
    
    theta0 = util().rad_2_pi_range(x[0] + generate_noise()) 
    theta1 = util().rad_2_pi_range(x[1] + generate_noise()) 
    
    omega0 = x[2] + generate_noise()
    omega1 = x[3] + generate_noise()
    
    q = np.array([theta0, theta1])
    qdot = np.array([omega0, omega1])
    
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
    
    tau = M_hat @ (- Kp @ (q-q_ref) - Kd @ qdot) + C_hat @ qdot + G_hat
    
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

while True:
    tau = tau_control(x_rk4)
    x_rk4_new = inte().rk4(f_double_pendulum, x=x_rk4, u=tau, h=t_step, ctrl_on=True)
    
    q0_all_rk4.append(x_rk4_new[0])
    q1_all_rk4.append(x_rk4_new[1])
    
    t = t + t_step
    t_all.append(t)
    u_all.append(tau)

    x_rk4 = x_rk4_new
    theta0_current = util().rad_2_pi_range(x_rk4[0])
    theta1_current = util().rad_2_pi_range(x_rk4[1])
    
    # print(np.abs(theta0_current - q0_ref))
    if np.abs(theta0_current - q0_ref) < event_thres and np.abs(theta1_current - q1_ref) < event_thres or t > t_lim:
        break
    
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