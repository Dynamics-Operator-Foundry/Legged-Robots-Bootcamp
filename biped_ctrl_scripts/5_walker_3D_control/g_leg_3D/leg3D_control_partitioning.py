import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util, Leg3DModelling as leg3D

g = 9.81

l_hip = 0.4
l_thigh = 1.4
l_knee = 1.2
m_foot = 0.1
m_calf = 0.3 
m_knee = 0.1 
m_thigh = 0.4 
m_hip = 0.1
Ihx = 0.1
Ihy = 0.1
Ihz = 0.2
Itx = 0.2
Ity = 0.2
Itz = 0.4
Icx = 0.1
Icy = 0.1
Icz = 0.3

param = [l_hip, l_thigh, l_knee, m_foot, m_calf, m_knee, m_thigh, m_hip, Ihx, Ihy, Ihz, Itx, Ity, Itz, Icx, Icy, Icz, g]

t = 0
sample_factor = 10

# simulation environment
x0_all_rk4 = []
x1_all_rk4 = []
x2_all_rk4 = []
x3_all_rk4 = []
x4_all_rk4 = []
x5_all_rk4 = []

t_all = []

Kp = 100 * np.identity(3)
Kd = 2 * np.sqrt(Kp)

event_thres = 1e-2
q_ref = np.array([0.1,0.67,-1.3])

def save_data(q):
    x0_all_rk4.append(q[0])
    x1_all_rk4.append(q[1])
    x2_all_rk4.append(q[2])
    x3_all_rk4.append(q[3])
    x4_all_rk4.append(q[4])
    x5_all_rk4.append(q[5])
    
    return

def generate_noise():
    mean = 0
    std_dev = 0.1

    return np.random.normal(mean, std_dev, 1)[0]

def generate_noise_matrix(n, m):
    mean = 0
    std_dev = 0.1
    return np.random.normal(mean, std_dev, (n, m))

def tau_control(x):
    
    phi0 = x[0]
    theta1 = x[1]
    theta2 = x[2]
    omega0 = x[3]
    omega1 = x[4]
    omega2 = x[5]
    
    q = np.array([phi0, theta1, theta2])
    qdot = np.array([omega0, omega1, omega2])
    
    M = leg3D().get_mass_mat(*q, *qdot, *param)    
    N = leg3D().get_N_vec(*q, *qdot, *param)
    N = N.flatten()
        
    # M * qddot + C(qdot) + G(q)
    # = tau
    # = M^ * (-Kp * (q - q_ref) - Kd * qdot) + C^ * qdot + G^ * q 
    
    M_hat = M
    N_hat = N
    # + generate_noise_matrix(3,1).transpose().flatten()
    
    # print("herererererere")
    # print(M_hat.shape)
    # print(q.shape)
    # print(q_ref.shape)
    # print(qdot.shape)
    # print(N_hat.shape)
    
    tau = M_hat @ (- Kp @ (q-q_ref) - Kd @ qdot) + N_hat
    # print(tau.shape)
    # exit()
    return tau

def f_double_pendulum(x, tau):
    
    phi0 = x[0]
    theta1 = x[1]
    theta2 = x[2]
    omega0 = x[3]
    omega1 = x[4]
    omega2 = x[5]
    
    q = np.array([phi0, theta1, theta2])
    qdot = np.array([omega0, omega1, omega2])
    
    M = leg3D().get_mass_mat(*q, *qdot, *param)    
    N = leg3D().get_N_vec(*q, *qdot, *param)
    N = N.flatten()
    
    # M * qddot + N
    # = tau
    # = M^ * (-Kp * (q - q_ref) - Kd * qdot) + C^ * qdot + G^ * q
    # print('gan')
    # print(tau.shape)
    # print(N.shape)
    b = tau - N
    A = M
    
    # print("jererere")
    # print(A)
    # print("jererere")
    # print(b)
    # print("jererere")
    
    # exit()
    A = np.array(A, dtype=np.float64)
    b = np.array(b, dtype=np.float64)
    qddot = np.linalg.solve(A,b)
 
    return np.array([
        omega0, 
        omega1, 
        omega2, 
        qddot[0], 
        qddot[1],
        qddot[2] 
        ])

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
    
    
    sim3D().anime(
        t=t_all[::sample_factor],
        x_states=[
            x0_all_rk4[::sample_factor],
            x1_all_rk4[::sample_factor],
            x2_all_rk4[::sample_factor],
            x3_all_rk4[::sample_factor],
            x4_all_rk4[::sample_factor],
            x5_all_rk4[::sample_factor]
        ],
        ms=10,
        mission='3D Leg',
        sim_object='3Dleg',
        sim_info={'l_hip': l_hip, 'l_thigh': l_thigh, 'l_knee': l_knee},
        save=False,
        save_name='3Dleg_partition_control'
    )
    exit()


    
t_lim = 10.0

x_rk4 = np.array([0,0,0,0,0,0])
t_step = 1e-3

while True:
    tau = tau_control(x_rk4)
    x_rk4_new = inte().rk4(f_double_pendulum, x=x_rk4, u=tau, h=t_step, ctrl_on=True)
    
    t = t + t_step
    t_all.append(t)
    save_data(x_rk4_new)

    x_rk4 = x_rk4_new
    # theta0_current = util().rad_2_pi_range(x_rk4[0])
    # theta1_current = util().rad_2_pi_range(x_rk4[1])
    
    print(t)
    print(np.linalg.norm(x_rk4[0:3] - q_ref))
    
    # print(np.abs(theta0_current - q0_ref))
    if np.linalg.norm(x_rk4[0:3] - q_ref) < event_thres or t > t_lim:
        print('h')
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