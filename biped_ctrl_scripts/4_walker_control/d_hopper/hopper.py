import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator

c = 0.02
m = 100
g = 9.81
e = 0.9
k = 10000
l = 1

t_step = 1/1000

ground = 0

x0 = 0
x1 = 1.2
dx0 = 1.0
dx1 = 0
no_of_jump = 10
# cannot set too high, as the current fsm switching condition is ideal

xstart = np.array([x0, x1, dx0, dx1])
xc_stance = 0

# apex, flight, touchdown, stance, takeoff
fsm = 'apex'

x0_all_rk4 = []
x1_all_rk4 = []
lx0_all_rk4 = []
lx1_all_rk4 = []
dx0_all_rk4 = []
dx1_all_rk4 = []

t_all = []

theta = 10

jump_i = 0
x_rk4 = xstart

event_thres = 1e-2
t = 0

sample_factor = 10

def f_flight(x):
    return np.array([
        x[2], 
        x[3], 
        0.0, 
        - g
        ])

def f_stance(x):
    l_now = np.sqrt(
                (x_rk4[0] - xc_stance) ** 2 + x_rk4[1] ** 2
            )
            
    return np.array([
        x[2], 
        x[3], 
        k * (l - l_now) * (x[0] - xc_stance) / l_now / m, 
        k * (l - l_now) * x[1] / l_now / m - g
        ])

def check_sys(x1_body,x1_leg):
    if (x1_body - ground) < -10 * event_thres or (x1_leg - ground) < -10 * event_thres:
        print('SYSTEM FAILED...')
        draw_anime(False)

def draw_anime(success):
    if success:
        save_name = "hopper_" + str(no_of_jump)
    else:
        save_name = "hopper_" + str(no_of_jump) + "_failed"
    Simulation2D().anime(
        t=t_all[::sample_factor], 
        x_states=[
            x0_all_rk4[::sample_factor], 
            x1_all_rk4[::sample_factor], 
            lx0_all_rk4[::sample_factor], 
            lx1_all_rk4[::sample_factor]
        ], 
        ms=1000 * t_step * sample_factor,
        mission="Hop", 
        sim_object="hopper",
        sim_info={'ground':ground},
        save=False,
        save_name=save_name
    )
    exit()

while True:
    if fsm == 'apex':
        print()
        print(jump_i)
        print(fsm)
        fsm = 'flight_down'
        jump_i = jump_i + 1
        
    elif fsm == 'flight_down':
        print(fsm)
        while True:
            x_new_rk4 = Integrator().rk4(f_flight, x=x_rk4, h=t_step)
            x0_all_rk4.append(x_new_rk4[0])
            x1_all_rk4.append(x_new_rk4[1])
            lx0_all_rk4.append(x_new_rk4[0] + l * np.sin(theta / 180 * np.pi))
            lx1_all_rk4.append(x_new_rk4[1] - l * np.cos(theta / 180 * np.pi))
            dx0_all_rk4.append(x_new_rk4[2])
            dx1_all_rk4.append(x_new_rk4[3])
            
            t = t + t_step
            t_all.append(t)
            
            check_sys(x_new_rk4[1], x_new_rk4[1] - l * np.cos(theta / 180 * np.pi))
            x_rk4 = x_new_rk4
            
            if np.abs(x_rk4[1] - l * np.cos(theta / 180 *np.pi)) < event_thres:
                #  @ touchdown
                xc_stance = x_rk4[0] + l * np.sin(theta / 180 *np.pi)
                fsm = 'stance'
                
                break 
        
    elif fsm == 'stance':
        print(fsm)
        while True:
            x_new_rk4 = Integrator().rk4(f_stance, x=x_rk4, h=t_step)
            x0_all_rk4.append(x_new_rk4[0])
            x1_all_rk4.append(x_new_rk4[1])
            lx0_all_rk4.append(xc_stance)
            lx1_all_rk4.append(0)
            dx0_all_rk4.append(x_new_rk4[2])
            dx1_all_rk4.append(x_new_rk4[3])
            
            t = t + t_step
            t_all.append(t)
            
            check_sys(x_new_rk4[1],0)
            x_rk4 = x_new_rk4
            
            if np.abs(
                np.sqrt(
                    (x_rk4[0] - xc_stance) ** 2 + x_rk4[1] ** 2
                    )  - l
                ) < event_thres and x_rk4[3] > 0:
                #  @ takeoff
                fsm = 'flight_up'
                
                break         
        
    elif fsm == 'flight_up':
        print(fsm)
        while True:
            x_new_rk4 = Integrator().rk4(f_flight, x=x_rk4, h=t_step)
            x0_all_rk4.append(x_new_rk4[0])
            x1_all_rk4.append(x_new_rk4[1])
            lx0_all_rk4.append(x_new_rk4[0] + l * np.sin(theta / 180 * np.pi))
            lx1_all_rk4.append(x_new_rk4[1] - l * np.cos(theta / 180 * np.pi))
            dx0_all_rk4.append(x_new_rk4[2])
            dx1_all_rk4.append(x_new_rk4[3])
            
            t = t + t_step
            t_all.append(t)
            
            check_sys(x_new_rk4[1], x_new_rk4[1] - l * np.cos(theta / 180 * np.pi))
            x_rk4 = x_new_rk4
            
            if np.abs(x_rk4[3] - 0) < event_thres:
                #  @ apex
                fsm = 'apex'
                break 
            
    # print('end once')
    if jump_i == no_of_jump:
        break
    
print('SYSTEM INTEGRATION SUCCEEDED...')
draw_anime(True)


exit()