import numpy as np
from scipy.interpolate import griddata
import sys
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
import time
import psutil
from tqdm import tqdm


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation2D as sim2D, RobotUtils as util


# basic parameters for a walker
hip_m = 1.0 # kg, mass of hip
leg_m = 0.5 # kg, mass of leg
leg_I = 0.02 # kg x m^2, moment of inertia of leg
leg_l = 1.0 # kg x m^2, length of 
leg_c = 0.5 # m, CoM of the leg
g = 10.0 # gravity
slope_angle = 0.01

# initial states in ground plane {G}
# initial state of q = [q0, q1, u0, u1, x0, x1]
# which are {q0, q1} = {theta_leg0, theta_leg1}
# which are {u0, u1} = {omega_leg0, omega_leg1}
# which are {x0, x1} = {xc_leg0, xc_leg1}
q0 = 0.0
q1 = 0.4

u0 = -0.5
u1 = 0.0

x_rk4 = np.array([q0, q1, u0, u1])
foot_on_ground_now = 1
t = 0

# integration environs
t_step = 1e-3
ground = 0
no_of_walk = 6
walk_i = 0
event_thres = 1e-2

# controller related
Kp_v = 0.05
Kp_phi = 4.0
Kp_phidot = 8.0

# LUT-related
grid_size = 16
q1_vals = np.linspace(-np.pi/2, np.pi/2, grid_size)
u0_vals = np.linspace(-np.pi, np.pi, grid_size)
u1_vals = np.linspace(-np.pi, np.pi, grid_size)
U_vals = np.linspace(0.0, np.pi/2, grid_size)

def f_single_stance(x,u):
    I = leg_I
    M = hip_m
    c = leg_c
    l = leg_l
    m = leg_m
    
    theta0 = x[0]
    theta1 = x[1]
    omega0 = x[2]
    omega1 = x[3]
    gam = slope_angle
    
    M00 =  2.0*I + 1.0*M*l**2 + 2.0*c**2*m - 2.0*c*l*m*np.cos(theta1) - 2.0*c*l*m + 2.0*l**2*m
    M01 =  1.0*I + 1.0*c**2*m - 1.0*c*l*m*np.cos(theta1)
    M10 =  1.0*I + 1.0*c**2*m - 1.0*c*l*m*np.cos(theta1)
    M11 =  1.0*I + 1.0*c**2*m
    
    b_0 =  1.0*M*g*l*np.sin(gam - theta0) - 1.0*c*g*m*np.sin(gam - theta0) + 1.0*c*g*m*np.sin(-gam + theta0 + theta1) + 2.0*c*l*m*omega0*omega1*np.sin(theta1) + 1.0*c*l*m*omega1**2*np.sin(theta1) + 2.0*g*l*m*np.sin(gam - theta0)
    b_1 =  1.0*c*m*(g*np.sin(-gam + theta0 + theta1) - l*omega0**2*np.sin(theta1))
    
    A = np.array([[M00, M01],[M10, M11]])
    b = np.array([-b_0, -b_1])
    x_new = np.linalg.solve(A,b)
    
    return np.array([
        omega0, 
        omega1, 
        x_new[0], 
        x_new[1] + u
        ])

def f_foot_strike(x):
    
    I = leg_I
    M = hip_m
    c = leg_c
    l = leg_l
    m = leg_m
    
    theta0_n = x[0]
    theta1_n = x[1]
    omega0_n = x[2]
    omega1_n = x[3]
    
    M00 = 1.0*M + 2.0*m
    M01 = 0
    M02 = -1.0*M*l*np.cos(theta0_n) + 1.0*m*(c*np.cos(theta0_n) + c*np.cos(theta0_n + theta1_n) - 2*l*np.cos(theta0_n))
    M03 = 1.0*c*m*np.cos(theta0_n + theta1_n)
    M10 = 0
    M11 = 1.0*M + 2.0*m
    M12 = -1.0*M*l*np.sin(theta0_n) + 1.0*m*(c*np.sin(theta0_n) + c*np.sin(theta0_n + theta1_n) - 2*l*np.sin(theta0_n))
    M13 = 1.0*c*m*np.sin(theta0_n + theta1_n)
    M20 = -1.0*M*l*np.cos(theta0_n) + 1.0*c*m*np.cos(theta0_n) + 1.0*c*m*np.cos(theta0_n + theta1_n) - 2.0*l*m*np.cos(theta0_n)
    M21 = -1.0*M*l*np.sin(theta0_n) + 1.0*c*m*np.sin(theta0_n) + 1.0*c*m*np.sin(theta0_n + theta1_n) - 2.0*l*m*np.sin(theta0_n)
    M22 = 2.0*I + 1.0*M*l**2 + 2.0*c**2*m - 2.0*c*l*m*np.cos(theta1_n) - 2.0*c*l*m + 2.0*l**2*m
    M23 = 1.0*I + 1.0*c**2*m - 1.0*c*l*m*np.cos(theta1_n)
    M30 = 1.0*c*m*np.cos(theta0_n + theta1_n)
    M31 = 1.0*c*m*np.sin(theta0_n + theta1_n)
    M32 = 1.0*I + 1.0*c**2*m - 1.0*c*l*m*np.cos(theta1_n)
    M33 = 1.0*I + 1.0*c**2*m
    
    M_fs = np.array([[M00, M01, M02, M03], [M10, M11, M12, M13], [M20, M21, M22, M23], [M30, M31, M32, M33]])
    
    J00 = 1
    J01 = 0
    J02 = l*(-np.cos(theta0_n) + np.cos(theta0_n + theta1_n))
    J03 = l*np.cos(theta0_n + theta1_n)
    J10 = 0
    J11 = 1
    J12 = l*(-np.sin(theta0_n) + np.sin(theta0_n + theta1_n))
    J13 = l*np.sin(theta0_n + theta1_n)
    
    J_C2 = np.array([[J00, J01, J02, J03], [J10, J11, J12, J13]])

    A = np.zeros((6,6))
    A[0:4,0:4] = M_fs
    A[0:4,4:6] = -J_C2.transpose()
    A[4:6,0:4] = J_C2
    # A = np.array([[M_fs, -J_C2.transpose()],[J_C2, np.zeros([2,2])]])
    
    omega0_o = x[2]
    omega1_o = x[2]
    qdot_ = np.array([0, 0, omega0_o, omega1_o])
    b = np.zeros([6])
    b[0:4] = M_fs @ qdot_
    # b = np.array([M_fs @ qdot_,0,0])
    
    x_new = np.linalg.solve(A,b)
    
    omega0_n = x_new[2]
    omega1_n = x_new[3]
    
    # change parameter expression: leg1 -> leg2, leg2 -> leg1
    theta0 = x[0] + x[1]
    theta1 = -x[1]
    omega0 = omega0_n + omega1_n
    omega1 = -omega1_n
    
    return np.array([
        theta0, 
        theta1, 
        omega0, 
        omega1 
        ])

def check_sys(x1):
    if x1 < - 1 * event_thres:
        return True
    else:
        return False

def get_foot_in_air(x, x_current_stance):
    T_B1_2_G = util().homo2D(
        psi=np.pi/2+x[0], 
        trans=np.array([x_current_stance[0],0])
    )
    T_B2_2_B1 = util().homo2D(
        psi=np.pi + x[1], 
        trans=np.array([leg_l,0])
    )
    foot_in_air_B1 = np.dot(T_B2_2_B1, np.array([leg_l, 0, 1]))
    foot_in_air_G = np.dot(T_B1_2_G, foot_in_air_B1)
    
    return foot_in_air_G[0:2]

def get_hip(x, x_current_stance):
    T_B1_2_G = util().homo2D(
        psi=np.pi/2+x[0], 
        trans=np.array([x_current_stance[0],0])
    )
    
    hip_G = T_B1_2_G @ np.array([leg_l, 0, 1])
    
    return hip_G[0:2]

def swing_control(phi_d, x):
    # cascaded P-control here
    current_phidot = f_single_stance(x=x,u=0)[1]
    current_phiddot = f_single_stance(x=x,u=0)[3]
    phidot_d = Kp_phi * (phi_d - x[1]) 
    
    u = Kp_phidot * (phidot_d - x[3]) + current_phiddot
    
    return u

# states after all integration
q0_all_rk4 = []
q1_all_rk4 = []
u0_all_rk4 = []
u1_all_rk4 = []
x0_all_rk4 = []
x1_all_rk4 = []
foot_on_ground_now_all = [] # foot 1, foot 2
dx0_all_rk4 = []
dx1_all_rk4 = []

t = 0
t_all = []
sample_factor = 10

def draw_anime(success):
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "walker_control"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "walker_control_" + "_failed"
    
    sim2D().anime(
        t=t_all[::sample_factor], 
        x_states=[
            q0_all_rk4[::sample_factor], 
            q1_all_rk4[::sample_factor], 
            x0_all_rk4[::sample_factor], 
            x1_all_rk4[::sample_factor],
            foot_on_ground_now_all[::sample_factor]
        ], 
        ms=1000 * t_step * sample_factor,
        mission="Walker Control", 
        sim_object="walker",
        sim_info={'ground': ground,'slope_angle':slope_angle, 'leg_l':leg_l},
        save=False,
        save_name=save_name
    )
    exit()

# integration function
def P(q1, u0, u1, U):
    q0_all_rk4.clear()
    q1_all_rk4.clear()
    u0_all_rk4.clear()
    u1_all_rk4.clear()
    
    x0_all_rk4.clear()
    x1_all_rk4.clear()
    t_all.clear()
    
    foot_on_ground_now_all.clear()
    foot_on_ground_now = 1
    
    x_rk4 = np.array([0, q1, u0, u1])
    x_start_lala = np.array([0, q1, u0, u1])
    fsm = 'single_stance'
    x_current_stance = [0,0]
    after_foot_strike = False
    t = 0
    while True:
        if fsm == 'single_stance':
            # integrate throughout single stance
            while True:
                u = swing_control(phi_d=U, x=x_rk4)

                x_rk4_new = inte().rk4(f_single_stance, x=x_rk4, u=u, h=t_step, ctrl_on=True)
                
                q0_all_rk4.append(x_rk4_new[0])
                q1_all_rk4.append(x_rk4_new[1])
                u0_all_rk4.append(x_rk4_new[2])
                u1_all_rk4.append(x_rk4_new[3])
                
                x0_all_rk4.append(x_current_stance[0])
                x1_all_rk4.append(x_current_stance[1])
                foot_on_ground_now_all.append(foot_on_ground_now)
                
                t = t + t_step
                t_all.append(t)

                x_rk4 = x_rk4_new
                
                foot_in_air = get_foot_in_air(x_rk4, x_current_stance)
                hip = get_hip(x_rk4, x_current_stance)
                
                fail = check_sys(hip[1])
                if fail:
                    print('SYSTEM FAILED...')
                    # print()
                    return False, 0
                                
                if np.abs(x_rk4[0]) < 0.1 * event_thres and after_foot_strike:
                    print('SYSTEM SUCCEEDED...')
                    # print()
                    # print(x_start_lala, x_rk4[2])                    
                    # draw_anime(False)
                    # print
                    return True, x_rk4[2] # qdot_plus
            
                if np.abs(foot_in_air[1] - x_current_stance[1]) < event_thres and np.abs(x_rk4[1] + 2 * x_rk4[0]) < event_thres and np.abs(x_rk4[0]) > 1 * event_thres and np.abs(x_rk4[1]) > 1 * event_thres and x_rk4[0] < 0:
                    fsm = 'foot_strike'
                    # print("SWITCH TO FOOT STRIKE")
                    break
        
        elif fsm == 'foot_strike':
            # bounce state
            x_current_stance = [get_foot_in_air(x_rk4, x_current_stance)[0], 0]
            theta0, theta1, omega0, omega1 = f_foot_strike(x_rk4)
            x_rk4 = np.array([theta0, theta1, omega0, omega1])
            # print(fsm)
            fsm = 'single_stance'
            
            if foot_on_ground_now == 1:
                foot_on_ground_now = 2
            else:
                foot_on_ground_now = 1
            
            after_foot_strike = True

output_values_vector = []

tasks = [(q1, u0, u1, U) for q1 in q1_vals for u0 in u0_vals for u1 in u1_vals for U in U_vals]

# Process a point
def process_point(q1, u0, u1, U):
    try:
        success, q0dot_plus = P(q1, u0, u1, U)  
        if success:
            # print([q1, u0, u1, q0dot_plus], U)
            return [q1, u0, u1, q0dot_plus], U
        else:
            return None
    except Exception as e:
        print(f"Error processing point ({q1}, {u0}, {u1}, {U}): {e}")
        return None 

def save_data(input_v, output_v):
    # Convert results to a NumPy array for easier handling
    input_array = np.array(input_v)
    output_array = np.array(output_v)
    # Save the arrays
    np.save('input_array_'+ str(grid_size) +'.npy', input_array)
    np.save('output_array_'+ str(grid_size) +'.npy', output_array)

    print("Arrays saved successfully.")
    
# Function to parallelize execution of tasks
def parallel_lookup(tasks, num_workers=1):
    input_v = []
    output_v = []
    # Use tqdm for progress tracking
    with ProcessPoolExecutor(max_workers=num_workers) as executor:
        # Submit tasks to the executor
        future_to_task = {executor.submit(process_point, *task): task for task in tasks}
        
        # Create tqdm progress bar
        with tqdm(total=len(future_to_task), desc="Processing tasks", unit="task") as pbar:
            # Collect results as they complete
            for future in as_completed(future_to_task):
                result = future.result()  # Get result from future
                # Check available RAM
                ram_info = psutil.virtual_memory()
                available_ram_gb = ram_info.available / (1024 ** 3)
                
                if result is not None:
                    input_, output_ = result
                    input_v.append(input_)
                    output_v.append(output_)

                    
                        
                if pbar.n % 1000 == 0:
                    print(available_ram_gb, "GB LEFT")
                    print()
                if available_ram_gb < 1.0:
                    print("WARNING! LOW RAM!")
                    save_data(input_v=input_v, output_v=output_v)
                    os._exit(0)
                # Update the progress bar
                pbar.update(1)
    save_data(input_v=input_v, output_v=output_v)
    return 

def test():
    # lala = process_point(0.5, -0.5, 0.0, 1.5)
    input_array = np.load('input_array_16.npy')
    output_array = np.load('output_array_16.npy')

    indi = 13000
    
    print(input_array.shape)
    print(output_array.shape)
    print('=============================================')
    print("q1, u0, u1, u0_des: ")
    print(input_array[indi,:])
    print("phi:")
    print(output_array[indi])
    print()
    
    lala1 = input_array[indi,:]
    lala2 = output_array[indi]
    state, U = process_point(lala1[0], lala1[1], lala1[2], lala2)
    
    print('=============================================')
    print('u0_des:')
    print(lala1[3])
    print('u0_real:')
    print(state[3])
    # print(lala_end)
    
    # draw_anime(True)
    exit()
    
if __name__ == '__main__':
    test()
    start_time = time.time()
    output_values_vector = parallel_lookup(tasks, num_workers=1)  # Adjust num_workers as needed
    end_time = time.time()
    print('TIME USED: ',end_time-start_time)
