from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
from sympy import *

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './5_walker_3D_Control/f_walker_3D/')))
from walker3D_model import Walker3DModelling
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './5_walker_3D_Control/g_leg_3D/')))
from leg3D_model import Leg3DModelling

class RobotUtils:
    def __init__(self):
        pass
    
    def rot2D(self, psi):
        return np.array(
            [[cos(psi), -sin(psi)], [sin(psi), cos(psi)]]
        )
        
    def homo2D(self, psi, trans):
        homo2D_mat = np.zeros((3, 3))
        homo2D_mat[:2, :2] = self.rot2D(psi)
        homo2D_mat[0:2, 2] = trans
        homo2D_mat[2, 0:3] = np.array([0,0,1])
        
        return homo2D_mat
    
    def rot3D(self, phi, theta, psi):
        R_x = np.array([
            [1, 0, 0],
            [0, cos(phi), -sin(phi)],
            [0, sin(phi), cos(phi)]
        ])
    
        R_y = np.array([
            [cos(theta), 0, sin(theta)],
            [0, 1, 0],
            [-sin(theta), 0, cos(theta)]
        ])
    
        R_z = np.array([
            [cos(psi), -sin(psi), 0],
            [sin(psi), cos(psi), 0],
            [0, 0, 1]
        ])
    
        return R_z @ R_y @ R_x
    
    def rad_2_pi_range(self, angle):
        while np.abs(angle) > np.pi:
            if angle > 0:
                angle = angle - 2*np.pi
            else:
                angle = angle + 2*np.pi
            
        return angle
    
    def generate_noise(self, mean, std_dev):

        return np.random.normal(mean, std_dev, 1)[0]

    def generate_noise_matrix(self, n, m, mean, std_dev):

        if n == 1:
            return np.random.normal(mean, std_dev, (m,))
        elif m == 1:
            return np.random.normal(mean, std_dev, (n,))
        else:
            np.random.normal(mean, std_dev, (n, m))

class Integrator():
    def __init__(self):
        pass
    
    def rk4(self, func, x, h, u=0, ctrl_on=False):
        
        if ctrl_on:
            k1 = func(x,u) * h
            k2 = func(x + k1 / 2, u) * h
            k3 = func(x + k2 / 2, u) * h  
            k4 = func(x + k3, u) * h
        else:
            k1 = func(x) * h
            k2 = func(x + k1 / 2) * h
            k3 = func(x + k2 / 2) * h  
            k4 = func(x + k3) * h
            
            
        return x + (k1 + 2 * k2 + 2 * k3 + k4) / 6
   
    def rkdp(self, func, x, h, u=0, args=[], ctrl_on=False):
        a21 = 1 / 5
        a31, a32 = 3 / 40, 9 / 40
        a41, a42, a43 = 44 / 55, -56 / 15, 32 / 9
        a51, a52, a53, a54 = 19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729
        a61, a62, a63, a64, a65 = 9017 / 3186, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656

        c2, c3, c4, c5, c6 = 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1

        B = np.array((35 / 384, 0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84))
        
        if ctrl_on:
            k1 = func(x, u, *args)
            k2 = func(x + h * a21 * k1, u, *args)
            k3 = func(x + h * (a31 * k1 + a32 * k2), u, *args)
            k4 = func(x + h * (a41 * k1 + a42 * k2 + a43 * k3), u, *args)
            k5 = func(x + h * (a51 * k1 + a52 * k2 + a53 * k3 + a54 * k4), u, *args)
            k6 = func(x + h * (a61 * k1 + a62 * k2 + a63 * k3 + a64 * k4 + a65 * k5), u, *args)

            k = np.array((k1, k2, k3, k4, k5, k6))
        else:
            k1 = func(x, *args)
            k2 = func(x + h * a21 * k1, *args)
            k3 = func(x + h * (a31 * k1 + a32 * k2), *args)
            k4 = func(x + h * (a41 * k1 + a42 * k2 + a43 * k3), *args)
            k5 = func(x + h * (a51 * k1 + a52 * k2 + a53 * k3 + a54 * k4), *args)
            k6 = func(x + h * (a61 * k1 + a62 * k2 + a63 * k3 + a64 * k4 + a65 * k5), *args)

            k = np.array((k1, k2, k3, k4, k5, k6))
            
        return x + h * B @ k

    def euler_forward(self, func, x, h):
        return x + h * func(x)
         
class Simulation2D(RobotUtils):
    def __init__(self):
        super().__init__()   
        self.ball = plt.Circle((0, 0), 0.2, color='red', fill=True)
        self.foot_circle = plt.Circle((0, 0), 0.05, color='green', fill=True)
        self.leg = None
        
        self.head = plt.Circle((0, 0), 0.2, color='black', fill=True)
        self.leg1 = None
        self.leg2 = None
        self.hand1 = None
        self.hand2 = None
        self.neck = None

        self.foot1 = plt.Circle((0, 0), 0.05, color='green', fill=True)
        self.foot2 = plt.Circle((0, 0), 0.05, color='green', fill=True)
        
        self.x_states = []
        
        self.sim_object = 'ball'
        self.sim_object = 'hopper'
        self.sim_object = 'walker'
        self.sim_info = {'ground':0, 'slope_angle':0, 'leg_l':0}
        
        self.fig = None
        self.ax = None
        
    def anime_init(self):
        self.fig, self.ax = plt.subplots()
    
        if self.sim_object == 'ball':
            # draw ball
            self.ball = plt.Circle((self.x_states[0][0], self.x_states[1][0]), 0.2, color='red', fill=True)
            self.ax.add_patch(self.ball)
            
            # draw ground
            self.ax.plot(
                [
                    min(self.x_states[0]) - 2.0, max(self.x_states[0]) + 2.0 
                ], 
                [
                    self.sim_info['ground'], 
                    self.sim_info['ground']
                ], 
                color='black', linewidth=2
            )
            
            self.ax.plot(
                [
                    min(self.x_states[0]) - 2.0, min(self.x_states[0]) - 2.0 
                ], 
                [
                    min(self.x_states[1]) - 2.0, max(self.x_states[1]) + 2.0 
                ], 
                color='black', linewidth=2
            )
            
        elif self.sim_object == 'hopper':
            # draw hopper
            self.ball = plt.Circle((self.x_states[0][0], self.x_states[1][0]), 0.1, color='red', fill=True)

            self.leg, = self.ax.plot(
                [self.x_states[0][0], self.x_states[2][0]], 
                [self.x_states[1][0], self.x_states[3][0]], 
                color='blue', 
                linewidth=3)
            # body_x x_states[0]
            # body_y x_states[1]
            # leg_x x_states[2]
            # leg_g x_states[3]
            
            self.foot_circle = plt.Circle(
                (self.x_states[2][0], self.x_states[3][0]), 
                0.05, color='green', fill=True
            )
            
            self.ax.add_patch(self.ball)
            self.ax.add_patch(self.foot_circle)
            
            # draw ground
            self.ax.plot(
                [
                    min(np.min(self.x_states[0]), np.min(self.x_states[2])) - 2, 
                    max(np.max(self.x_states[0]), np.max(self.x_states[2])) + 2
                ], [self.sim_info['ground'], self.sim_info['ground']], color='black', linewidth=2)
            
        elif self.sim_object == 'walker':
            initial_state = [
                self.x_states[0][0], 
                self.x_states[1][0], 
                self.x_states[2][0], 
                self.x_states[3][0]
            ]
            # leg1_theta x_states[0]
            # leg2_theta x_states[1]
            # leg_xc x_states[2]
            # leg_yc x_states[3] in {G}
            
            # transform from {G} to {I}
            foot_on_ground, hip, foot_in_air = self.draw_walker(x=initial_state)
            
            # draw walker
            self.ball = plt.Circle((hip[0], hip[1]), 0.2, color='red', fill=True)
            self.head = plt.Circle((hip[0], hip[1]+0.4), 0.12, color='black', fill=True)
            self.foot1 = plt.Circle((foot_on_ground[0], foot_on_ground[1]), 0.05, color='green', fill=True)
            self.foot2 = plt.Circle((foot_in_air[0], foot_in_air[1]), 0.05, color='green', fill=True)
            self.leg1, = self.ax.plot(
                [hip[0], foot_on_ground[0]], 
                [hip[1], foot_on_ground[1]], 
                color='cyan', 
                linewidth=4)
            self.leg2, = self.ax.plot(
                [hip[0], foot_in_air[0]], 
                [hip[1], foot_in_air[1]], 
                color='pink', 
                linewidth=4)
            self.hand1, = self.ax.plot(
                [hip[0], hip[0]-0.6], 
                [hip[1], hip[1]+0.1], 
                color='orange', 
                linewidth=2
            )
            
            self.hand2, = self.ax.plot(
                [hip[0], hip[0]+0.6], 
                [hip[1], hip[1]+0.1], 
                color='orange', 
                linewidth=2
            )
            
            self.neck, = self.ax.plot(
                [hip[0], hip[0]], 
                [hip[1], hip[1]+0.6], 
                color='black', 
                linewidth=4)
            
            self.ax.add_patch(self.ball)
            self.ax.add_patch(self.foot1)
            self.ax.add_patch(self.foot2)
            self.ax.add_patch(self.head)
            
            # draw ground (slope)
            min_x = np.min(self.x_states[2]) - 2
            max_x = np.max(self.x_states[2]) + 2
            
            T_G_2_I = self.homo2D(
                psi=-self.sim_info['slope_angle'], 
                trans=np.zeros([2])
            )
            
            min_xy_I = T_G_2_I @ np.array([min_x, self.sim_info['ground'], 1])
            max_xy_I = T_G_2_I @ np.array([max_x, self.sim_info['ground'], 1])
            
            self.ax.plot(
                [min_xy_I[0], max_xy_I[0]], 
                [min_xy_I[1], max_xy_I[1]], 
                color='black', linewidth=2
            )
        
        elif self.sim_object == 'double_pendulum':
            initial_state = [
                self.x_states[0][0], 
                self.x_states[1][0]
            ]
            fixation, hinge, end_effector = self.draw_double_pendulum(x=initial_state)
            
            # draw points
            self.foot1 = plt.Circle((fixation[0], fixation[1]), 0.05, color='green', fill=True)
            self.foot2 = plt.Circle((hinge[0], hinge[1]), 0.05, color='green', fill=True)
            self.ball = plt.Circle((end_effector[0], end_effector[1]), 0.05, color='green', fill=True)
            self.ax.add_patch(self.foot1)
            self.ax.add_patch(self.foot2)
            self.ax.add_patch(self.ball)
            
            # draw arms
            self.leg1, = self.ax.plot(
                [fixation[0], hinge[0]], 
                [fixation[1], hinge[1]], 
                color='cyan', 
                linewidth=4)
            self.leg2, = self.ax.plot(
                [hinge[0], end_effector[0]], 
                [hinge[1], end_effector[1]], 
                color='pink', 
                linewidth=4)
            
            # draw plane cross
            self.ax.plot(
                [-2, 2], 
                [0, 0], 
                color='black', linewidth=2
            )
            self.ax.plot(
                [0, 0], 
                [-2, 2], 
                color='black', linewidth=2
            )
        
        elif self.sim_object == 'single_pendulum':
            initial_state = self.x_states[0][0]
            
            fixation, end_effector = self.draw_single_pendulum(x=initial_state)
            
            # draw points
            self.foot1 = plt.Circle((fixation[0], fixation[1]), 0.05, color='green', fill=True)
            self.ball = plt.Circle((end_effector[0], end_effector[1]), 0.05, color='green', fill=True)
            self.ax.add_patch(self.foot1)
            self.ax.add_patch(self.ball)
            
            # draw arms
            self.leg1, = self.ax.plot(
                [fixation[0], end_effector[0]], 
                [fixation[1], end_effector[1]], 
                color='cyan', 
                linewidth=4)
            
            # draw plane cross
            self.ax.plot(
                [-2, 2], 
                [0, 0], 
                color='black', linewidth=2
            )
            self.ax.plot(
                [0, 0], 
                [-2, 2], 
                color='black', linewidth=2
            )
        
        elif self.sim_object == 'spring_mass_damper':
            # draw hopper
            self.ball = plt.Circle((self.x_states[0][0], self.sim_info['ball_radius']), self.sim_info['ball_radius'], color='red', fill=True)

            self.leg, = self.ax.plot(
                [self.x_states[0][0], self.sim_info['wall']], 
                [self.sim_info['ball_radius'], self.sim_info['ball_radius']], 
                color='blue', 
                linewidth=3)
            # ball_x x_states[0]
            
            self.foot_circle = plt.Circle(
                (self.sim_info['wall'], self.sim_info['ball_radius']), 
                0.05, color='green', fill=True
            )
            
            self.ax.add_patch(self.ball)
            self.ax.add_patch(self.foot_circle)
            
            # draw ground
            self.ax.plot(
                [
                    min(np.min(self.x_states[0][0]), self.sim_info['wall']) - 0.5, 
                    max(np.max(self.x_states[0][0]), self.sim_info['wall']) + 0.5
                ], [self.sim_info['ground'], self.sim_info['ground']], color='black', linewidth=2)
            self.ax.plot(
                [self.sim_info['wall'], self.sim_info['wall']], 
                [1.5, -0.0], color='black', linewidth=2)
            
        else:
            print("GOT ERROR CHOOSING OBJECT")
            exit()
            
        self.ax.set_aspect('equal')
            
    def anime(
        self, 
        t, 
        x_states, 
        ms = 10,
        mission='lala', 
        sim_object='ball', 
        sim_info={}, 
        save=False, 
        save_name='obj_sim'
        ):
        
        self.sim_object = sim_object
        self.x_states = x_states
        self.sim_info = sim_info
        self.anime_init()    
            
        ani = FuncAnimation(self.fig, self.update, frames=len(t), interval=ms, blit=True)
        if save:
            ani.save( save_name + '.mp4', writer='ffmpeg', fps=1000/ms)  # Match to real-time playback speed
            
        plt.title(mission)
        plt.show()
           
    def update(self, frame):
        if self.sim_object == 'ball':
            self.ball.center = (self.x_states[0][frame], self.x_states[1][frame])
            
            return (self.ball,)
        elif self.sim_object == 'hopper':
            self.ball.center = (self.x_states[0][frame], self.x_states[1][frame])
            self.leg.set_data(
                [self.x_states[0][frame], self.x_states[2][frame]], 
                [self.x_states[1][frame], self.x_states[3][frame]]
                )
            self.foot_circle.center = (self.x_states[2][frame], self.x_states[3][frame])

            return (self.ball, self.leg, self.foot_circle,)
        elif self.sim_object == 'walker':
            
            current_state = [
                self.x_states[0][frame], 
                self.x_states[1][frame], 
                self.x_states[2][frame],
                self.x_states[3][frame]
            ]
            foot_on_ground, hip, foot_in_air = self.draw_walker(x=current_state)
            
            if self.x_states[4][frame] == 1:
                self.leg1.set_data(
                    [hip[0], foot_on_ground[0]], 
                    [hip[1], foot_on_ground[1]]
                )
                self.leg2.set_data(
                    [hip[0], foot_in_air[0]], 
                    [hip[1], foot_in_air[1]]
                )
            else:
                self.leg1.set_data(
                    [hip[0], foot_in_air[0]], 
                    [hip[1], foot_in_air[1]]
                )
                self.leg2.set_data(
                    [hip[0], foot_on_ground[0]], 
                    [hip[1], foot_on_ground[1]]                    
                )
            self.hand1.set_data(
                    [hip[0], hip[0]-0.6], 
                    [hip[1], hip[1]+0.1]                    
                )
            self.hand2.set_data(
                    [hip[0], hip[0]+0.6], 
                    [hip[1], hip[1]+0.1]                    
                )
            
            # draw walker
            self.ball.center = (hip[0], hip[1])
            self.head.center = (hip[0], hip[1]+0.4)
            self.neck.set_data(
                [hip[0], hip[0]],
                [hip[1], hip[1]+0.4]
            )
            self.foot1.center = (foot_on_ground[0], foot_on_ground[1])
            self.foot2.center = (foot_in_air[0], foot_in_air[1])
            
            
            return (self.ball, self.foot1, self.foot2, self.leg1, self.leg2, self.hand1, self.hand2, self.head, self.neck)
                    
        elif self.sim_object == 'double_pendulum':
            current_state = [
                self.x_states[0][frame], 
                self.x_states[1][frame]
            ]
            fixation, hinge, end_effector = self.draw_double_pendulum(x=current_state)
            self.foot1.center = (fixation[0], fixation[1])
            self.foot2.center = (hinge[0], hinge[1])
            self.ball.center = (end_effector[0], end_effector[1])
            self.leg1.set_data(
                    [fixation[0], hinge[0]], 
                    [fixation[1], hinge[1]]                    
                )
            self.leg2.set_data(
                    [hinge[0], end_effector[0]], 
                    [hinge[1], end_effector[1]]                    
                )
            return (self.foot1, self.foot2, self.ball, self.leg1, self.leg2)
        
        elif self.sim_object == 'single_pendulum':
            current_state = self.x_states[0][frame]
            
            fixation, end_effector = self.draw_single_pendulum(x=current_state)
            self.foot1.center = (fixation[0], fixation[1])
            self.ball.center = (end_effector[0], end_effector[1])
            self.leg1.set_data(
                    [fixation[0], end_effector[0]], 
                    [fixation[1], end_effector[1]]                    
                )
            
            return (self.foot1, self.ball, self.leg1,)
        
        elif self.sim_object == 'spring_mass_damper':
            self.ball.center = (self.x_states[0][frame], self.sim_info['ball_radius'])
            self.leg.set_data(
                [self.x_states[0][frame], self.sim_info['wall']], 
                [self.sim_info['ball_radius'], self.sim_info['ball_radius']]
                )
            # self.foot_circle.center = (self.x_states[2][frame], self.x_states[3][frame])

            return (self.ball, self.leg,)
        
        else:
            exit()
            
    def draw_walker(self, x):
        # draw the walker in inertial frame {I}
        # q = [q0, q1, x0, x1] (in ground frame {G})
        T_G_2_I = self.homo2D(
            psi=-self.sim_info['slope_angle'], 
            trans=np.zeros([2])
        )
        T_B1_2_G = self.homo2D(
            psi=np.pi/2+x[0], 
            trans=np.array([x[2],0])
        )
        T_B2_2_B1 = self.homo2D(
            psi=np.pi + x[1], 
            trans=np.array([self.sim_info['leg_l'],0])
        )
        
        # foot on ground in {I}
        foot_on_ground = T_G_2_I @ np.array([x[2], 0, 1])
        
        # hip in {I}
        hip_G = T_B1_2_G @ np.array([self.sim_info['leg_l'], 0, 1])
        hip = T_G_2_I @ hip_G
        
        # foot in air in {I}
        foot_in_air_B1 = T_B2_2_B1 @ np.array([self.sim_info['leg_l'], 0, 1])
        foot_in_air_G = T_B1_2_G @ foot_in_air_B1
        foot_in_air = T_G_2_I @ foot_in_air_G
        
        return foot_on_ground[0:2], hip[0:2], foot_in_air[0:2]
    
    def draw_double_pendulum(self, x):
        # draw the double pendulum in inertial frame {I}
        # q = [q0, q1] (in inertial frame {I})
        T_B1_2_I = self.homo2D(
            psi=x[0], 
            trans=np.array([0,0])
        )
        T_B2_2_B1 = self.homo2D(
            psi=x[1], 
            trans=np.array([self.sim_info['l1'],0])
        )
        
        # foot on ground in {I}
        fixation = np.array([0,0])
        
        # hip in {I}
        hinge = T_B1_2_I @ np.array([self.sim_info['l1'], 0, 1])
        
        # foot in air in {I}
        end_effector_B1 = T_B2_2_B1 @ np.array([self.sim_info['l2'], 0, 1])
        end_effector = T_B1_2_I @ end_effector_B1
        
        return fixation, hinge[0:2], end_effector[0:2]
    
    def draw_single_pendulum(self, x):
        # draw the double pendulum in inertial frame {I}
        # q = [q0, q1] (in inertial frame {I})
        T_B1_2_I = self.homo2D(
            psi=x, 
            trans=np.array([0,0])
        )
        
        # foot on ground in {I}
        fixation = np.array([0,0])
        
        # foot in air in {I}
        end_effector = T_B1_2_I  @ np.array([self.sim_info['l1'], 0, 1])
        
        return fixation, end_effector[0:2]


class Simulation3D(RobotUtils, Walker3DModelling, Leg3DModelling):
    def __init__(self):
        super().__init__() 
        self.ball = None
        self.ball1 = None
        self.ball2 = None
        
        self.rod = None
        self.rod1 = None
        self.rod2 = None

        self.foot1 = plt.Circle((0, 0), 0.05, color='green', fill=True)
        self.foot2 = plt.Circle((0, 0), 0.05, color='green', fill=True)
        
        self.foot = None 
        self.knee = None 
        self.hip = None 
        self.body = None 
        self.l_hip = None 
        self.l_thigh = None
        self.l_calf = None
        
        self.x_states = []
        
        self.sim_object = 'ball'
        self.sim_object = 'hopper'
        self.sim_object = 'walker'
        self.sim_info = {'ground':0, 'slope_angle':0, 'leg_l':0}
        
        self.fig = None
        self.ax = None
        
    def anime_init(self):
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': '3d'})
        
        if self.sim_object == 'ball':
            # draw ball
            self.ball, = self.ax.plot([], [], [], 'o', color='red', markersize=10)
            
            self.set_sim_range(
                min(self.x_states[0]) - 2.0, 
                max(self.x_states[0]) + 2.0,
                min(self.x_states[1]) - 2.0, 
                max(self.x_states[1]) + 2.0,
                min(self.x_states[2]) - 0.1, 
                max(self.x_states[2]) + 0.1
            )
            
        elif self.sim_object == 'rod':
            
            self.ball1, = self.ax.plot([],[],[], 'o', color='red', markersize=10)
            self.ball2, = self.ax.plot([],[],[], 'o', color='green', markersize=10)
            self.rod, = self.ax.plot([], [], [], color='blue', linewidth=5)
            
            self.set_sim_range(
                min(self.x_states[0])-self.sim_info['rod_length']-0.5,
                max(self.x_states[0])+self.sim_info['rod_length']+0.5,
                min(self.x_states[1])-self.sim_info['rod_length']-0.5,
                max(self.x_states[1])+self.sim_info['rod_length']+0.5,
                min(self.x_states[2])-self.sim_info['rod_length']-0.5,
                max(self.x_states[2])+self.sim_info['rod_length']+0.5,
            )
            
        elif self.sim_object == '3Dwalker':
            self.head, = self.ax.plot([],[],[], 'o', color='red', markersize=10)
            self.hip, = self.ax.plot([],[],[], 'o', color='red', markersize=10)
            self.hipl, = self.ax.plot([],[],[], 'o', color='cyan', markersize=10)
            self.hipr, = self.ax.plot([],[],[], 'o', color='orange', markersize=10)
            self.kneel, = self.ax.plot([],[],[], 'o', color='cyan', markersize=10)
            self.kneer, = self.ax.plot([],[],[], 'o', color='orange', markersize=10)
            self.anklel, = self.ax.plot([],[],[], 'o', color='cyan', markersize=10)
            self.ankler, = self.ax.plot([],[],[], 'o', color='orange', markersize=10)
            
            self.torso_com, = self.ax.plot([],[],[], 'o', color='black', markersize=12)
            self.thighl_com, = self.ax.plot([],[],[], 'o', color='black', markersize=12)
            self.thighr_com, = self.ax.plot([],[],[], 'o', color='black', markersize=12)
            self.calfl_com, = self.ax.plot([],[],[], 'o', color='black', markersize=12)
            self.calfr_com, = self.ax.plot([],[],[], 'o', color='black', markersize=12)
            
            self.torso, = self.ax.plot([], [], [], color='gray', linewidth=5)
            self.hip, = self.ax.plot([], [], [], color='gray', linewidth=5)
            self.thighl, = self.ax.plot([], [], [], color='blue', linewidth=5)
            self.thighr, = self.ax.plot([], [], [], color='blue', linewidth=5)
            self.calfl, = self.ax.plot([], [], [], color='pink', linewidth=5)
            self.calfr, = self.ax.plot([], [], [], color='pink', linewidth=5)
            
            self.set_sim_range(
                min(self.x_states[0])-self.sim_info['w']-0.5,
                max(self.x_states[0])+self.sim_info['w']+0.5,
                min(self.x_states[1])-self.sim_info['w']-0.5,
                max(self.x_states[1])+self.sim_info['w']+0.5,
                0.0-0.05,
                max(self.x_states[2])+self.sim_info['l0']+0.5,
            )
            
        elif self.sim_object == '3Dleg':
            self.body, = self.ax.plot([],[],[], 'o', color='red', markersize=10)
            self.hip, = self.ax.plot([],[],[], 'o', color='cyan', markersize=10)
            self.knee, = self.ax.plot([],[],[], 'o', color='orange', markersize=10)
            self.foot, = self.ax.plot([],[],[], 'o', color='cyan', markersize=10)
            
            self.l_hip, = self.ax.plot([], [], [], color='black', linewidth=5)
            self.l_thigh, = self.ax.plot([], [], [], color='gray', linewidth=5)
            self.l_calf, = self.ax.plot([], [], [], color='blue', linewidth=5)
            
            self.set_sim_range(
                -2.0,
                2.0,
                -2.0,
                2.0,
                -2.0,
                2.0
            )
            
    def set_sim_range(self, xmin, xmax, ymin, ymax, zmin, zmax):
        
        # Set plot limits and labels
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylim(ymin, ymax)
        self.ax.set_zlim(zmin, zmax)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_aspect('equal')
        
    def anime(
        self, 
        t, 
        x_states, 
        ms = 10,
        mission='lala', 
        sim_object='ball', 
        sim_info={}, 
        save=False, 
        save_name='obj_sim'
        ):
        
        self.sim_object = sim_object
        self.x_states = x_states
        self.sim_info = sim_info
        self.anime_init()    
            
        ani = FuncAnimation(
            self.fig, 
            self.update, 
            frames=len(t), 
            interval=ms, 
            blit=True
        )
        
        print(len(t))
        print(1000/ms)
        
        if save:
            ani.save( save_name + '.mp4', writer='ffmpeg', fps=1000/ms)  # Match to real-time playback speed
        
        plt.title(mission)
        plt.show()
        
    def update(self, frame):
        if self.sim_object == 'ball':
            # Update ball's position
            self.ball.set_data([self.x_states[0][frame]], [self.x_states[1][frame]])
            self.ball.set_3d_properties([self.x_states[2][frame]])  # Set z position
            
            return self.ball,
            
        elif self.sim_object == 'rod':
            # self.ax.collections.remove(self.rod_surface)
            
            rod_start_now = [self.x_states[0][frame], self.x_states[1][frame], self.x_states[2][frame]]
            pitch_now = self.x_states[3][frame]
            yaw_now = self.x_states[4][frame]
            
            rod_start_now, rod_end_now = self.get_rod_cartesian(rod_start_now, pitch_now, yaw_now)            
            
            self.rod.set_data([rod_start_now[0], rod_end_now[0]], [rod_start_now[1], rod_end_now[1]])
            self.rod.set_3d_properties([rod_start_now[2], rod_end_now[2]]) 
            
            self.ball1.set_data([rod_start_now[0]], [rod_start_now[1]])
            self.ball1.set_3d_properties([rod_start_now[2]])  
            
            self.ball2.set_data([rod_end_now[0]], [rod_end_now[1]])
            self.ball2.set_3d_properties([rod_end_now[2]])  
            
            return self.ball1, self.ball2, self.rod,
        
        elif self.sim_object == '3Dwalker':
            q_now = np.array([self.x_states[0][frame], self.x_states[1][frame], self.x_states[2][frame], self.x_states[3][frame], self.x_states[4][frame], self.x_states[5][frame], self.x_states[6][frame], self.x_states[7][frame], self.x_states[8][frame], self.x_states[9][frame], self.x_states[10][frame], self.x_states[11][frame], self.x_states[12][frame], self.x_states[13][frame]])
            param = np.array([self.sim_info['w'], self.sim_info['l0'], self.sim_info['l1'], self.sim_info['l2']])
            
            # get points
            p_H = self.get_p_H(*q_now, *param)
            p_B = self.get_p_B(*q_now, *param)
            
            p_LH = self.get_p_LH(*q_now, *param)
            p_RH = self.get_p_RH(*q_now, *param)
            
            p_LK = self.get_p_LK(*q_now, *param)
            p_RK = self.get_p_RK(*q_now, *param)
            
            p_LA = self.get_p_LA(*q_now, *param)
            p_RA = self.get_p_RA(*q_now, *param)
            
            p_Torso = self.get_p_Torso(*q_now, *param)
            p_Thigh_L = self.get_p_Thigh_L(*q_now, *param)
            p_Thigh_R = self.get_p_Thigh_R(*q_now, *param)
            p_Calf_L = self.get_p_Calf_L(*q_now, *param)
            p_Calf_R = self.get_p_Calf_R(*q_now, *param)
            
            # set dots
            self.head.set_data([p_H[0]],[p_H[1]])
            self.head.set_3d_properties([p_H[2]])
            
            self.hip.set_data([p_B[0]],[p_B[1]])
            self.hip.set_3d_properties([p_B[2]])
            
            self.hipl.set_data([p_LH[0]],[p_LH[1]])
            self.hipl.set_3d_properties([p_LH[2]])
            self.hipr.set_data([p_RH[0]],[p_RH[1]])
            self.hipr.set_3d_properties([p_RH[2]])
            
            self.kneel.set_data([p_LK[0]],[p_LK[1]])
            self.kneel.set_3d_properties([p_LK[2]])
            self.kneer.set_data([p_RK[0]],[p_RK[1]])
            self.kneer.set_3d_properties([p_RK[2]])
            
            self.anklel.set_data([p_LA[0]],[p_LA[1]])
            self.anklel.set_3d_properties([p_LA[2]])
            self.ankler.set_data([p_RA[0]],[p_RA[1]])
            self.ankler.set_3d_properties([p_RA[2]])
            
            self.torso_com.set_data([p_Torso[0]],[p_Torso[1]])
            self.torso_com.set_3d_properties([p_Torso[2]])
            self.thighl_com.set_data([p_Thigh_L[0]],[p_Thigh_L[1]])
            self.thighl_com.set_3d_properties([p_Thigh_L[2]])
            self.thighr_com.set_data([p_Thigh_R[0]],[p_Thigh_R[1]])
            self.thighr_com.set_3d_properties([p_Thigh_R[2]])
            self.calfl_com.set_data([p_Calf_L[0]],[p_Calf_L[1]])
            self.calfl_com.set_3d_properties([p_Calf_L[2]])
            self.calfr_com.set_data([p_Calf_R[0]],[p_Calf_R[1]])
            self.calfr_com.set_3d_properties([p_Calf_R[2]])
            
            # set lines
            self.torso.set_data([p_H[0], p_B[0]], [p_H[1], p_B[1]])
            self.torso.set_3d_properties([p_H[2], p_B[2]])
            self.hip.set_data([p_LH[0], p_RH[0]], [p_LH[1], p_RH[1]])
            self.hip.set_3d_properties([p_LH[2], p_RH[2]])
            self.thighl.set_data([p_LH[0], p_LK[0]], [p_LH[1], p_LK[1]])
            self.thighl.set_3d_properties([p_LH[2], p_LK[2]])
            self.thighr.set_data([p_RH[0], p_RK[0]], [p_RH[1], p_RK[1]])
            self.thighr.set_3d_properties([p_RH[2], p_RK[2]])
            self.calfl.set_data([p_LK[0], p_LA[0]], [p_LK[1], p_LA[1]])
            self.calfl.set_3d_properties([p_LK[2], p_LA[2]])
            self.calfr.set_data([p_RK[0], p_RA[0]], [p_RK[1], p_RA[1]])
            self.calfr.set_3d_properties([p_RK[2], p_RA[2]])
            
            return self.head, self.hip, self.hipl, self.hipr, self.kneel, self.kneer, self.anklel, self.ankler, self.torso_com, self.thighl_com, self.thighr_com, self.calfl_com, self.calfr_com, self.torso, self.hip, self.thighl, self.thighr, self.calfl, self.calfr,
        
        elif self.sim_object == '3Dleg':
            q_now = np.array([
                self.x_states[0][frame], 
                self.x_states[1][frame], 
                self.x_states[2][frame]
            ])
            
            p_foot = self.get_p_foot_3Dleg(q_now)
            p_knee = self.get_p_knee_3Dleg(q_now)
            p_hip  = self.get_p_hip_3Dleg(q_now)
            
            # set dots
            self.foot.set_data([p_foot[0]], [p_foot[1]])
            self.foot.set_3d_properties([p_foot[2]])
            self.knee.set_data([p_knee[0]], [p_knee[1]])
            self.knee.set_3d_properties([p_knee[2]])
            self.hip.set_data([p_hip[0]], [p_hip[1]])
            self.hip.set_3d_properties([p_hip[2]])
            self.body.set_data([0], [0])
            self.body.set_3d_properties([0])
            
            # set lines
            self.l_hip.set_data([0, p_hip[0]], [0, p_hip[1]])
            self.l_hip.set_3d_properties([0, p_hip[2]])
            self.l_thigh.set_data([p_hip[0], p_knee[0]], [p_hip[1], p_knee[1]])
            self.l_thigh.set_3d_properties([p_hip[2], p_knee[2]])
            self.l_calf.set_data([p_knee[0], p_foot[0]], [p_knee[1], p_foot[1]])
            self.l_calf.set_3d_properties([p_knee[2], p_foot[2]])
            
            return self.foot, self.knee, self.hip, self.body, self.l_hip, self.l_thigh, self.l_calf        
            
    def get_rod_cartesian(self, x_rod_start, pitch, yaw):
        x_rod_end = self.get_rod_end(pitch, yaw, self.sim_info['rod_length']) + x_rod_start
        return x_rod_start, x_rod_end
    
    def get_rod_end(self, pitch, yaw, l):
    
        x_end_B = np.array([l,0,0])
        R = self.rot3D(phi=0, theta=pitch, psi=yaw)
        x_end_I = R @ x_end_B
        
        return x_end_I
    
    