import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation2D as sim2D, RobotUtils as util

g = 9.81
l1 = 1.0
m1 = 1.0
I1 = 0.02


t = 0
sample_factor = 10

# simulation environment
q0_all_rk4 = []

t_step = 1e-3
t_all = []
u_all = []

q0=-np.pi/2
u0=-0.0
x_rk4 = np.array([q0, u0])

q0_ref = np.pi/2
Kp = 100
Kd = 2*np.sqrt(Kp)

event_thres = 1e-2

def draw_anime(success):
    print('INTEGRATION END')
    print('TIME NOW: ', t)
    print()
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "single_pendulum_control_partitioning"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "single_pendulum_control_partitioning" + "_failed"
    
    sim2D().anime(
        t=t_all[::sample_factor], 
        x_states=[
            q0_all_rk4[::sample_factor]
        ], 
        ms=1000 * t_step * sample_factor,
        mission="Swing", 
        sim_object="single_pendulum",
        sim_info={'l1': l1},
        save=False,
        save_name=save_name
    )
    exit()

def generate_noise():
    mean = 0
    std_dev = 0.1
    
    return np.random.normal(mean, std_dev, 1)[0]

def tau_control(x):
    theta0 = x[0] + generate_noise()
    omega0 = x[1] + generate_noise()
    
    theta0 = util().rad_2_pi_range(theta0)
    
    # M * qddot + C(qdot) + G(q)
    # = tau
    # = M^ * (-Kp * (q - q_ref) - Kd * qdot) + C^ * qdot + G^ * q 
    M = 1.0*I1 + 0.25*l1**2*m1
    C = 0
    G = g*l1*m1*np.cos(theta0)/2
    
    M_hat = M + generate_noise()
    C_hat = C + generate_noise()
    G_hat = G + generate_noise()
    
    tau = M_hat*(-Kp * (theta0 - q0_ref) - Kd * omega0) + C_hat * omega0 + G_hat
    
    return tau

def f_single_pendulum(x, tau):
    
    theta0 = x[0] + generate_noise()
    omega0 = x[1] + generate_noise()
    
    theta0 = util().rad_2_pi_range(theta0)
    
    # M * qddot + C(qdot) + G(q)
    # = tau
    # = M^ * (-Kp * (q - q_ref) - Kd * qdot) + C^ * qdot + G^ * q 
    
    M = 1.0*I1 + 0.25*l1**2*m1
    C = 0
    G = g*l1*m1*np.cos(theta0)/2
    
    alpha0 = (tau - G - C)/M
    
    return np.array([
        omega0, 
        alpha0
        ])
    
t_lim = 10.0

while True:
    tau = tau_control(x_rk4)
    x_rk4_new = inte().rk4(f_single_pendulum, x=x_rk4, u=tau, h=t_step, ctrl_on=True)
    
    q0_all_rk4.append(x_rk4_new[0])
    
    t = t + t_step
    t_all.append(t)
    u_all.append(tau)

    x_rk4 = x_rk4_new
    theta0_current = util().rad_2_pi_range(x_rk4[0])
    
    # print(np.abs(theta0_current - q0_ref))
    if np.abs(theta0_current - q0_ref) < event_thres or t > t_lim:
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