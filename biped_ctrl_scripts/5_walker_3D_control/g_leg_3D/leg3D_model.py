import sympy as sp
from sympy import *
import numpy as np
from time import strftime, localtime
import time 

class Leg3DModelling():
    def __init__(self):
        pass
    
    def get_p_foot_3Dleg(self, q):
        phi0 = q[0]
        theta1 = q[1]
        theta2 = q[2]
        
        l_hip = self.sim_info['l_hip']
        l_thigh = self.sim_info['l_thigh']
        l_knee = self.sim_info['l_knee']
        
        p_foot = np.array([
            -l_knee*(sin(theta1)*cos(theta2) + sin(theta2)*cos(theta1)) - l_thigh*sin(theta1), 
            l_hip*cos(phi0) - l_knee*(sin(phi0)*sin(theta1)*sin(theta2) - sin(phi0)*cos(theta1)*cos(theta2)) + l_thigh*sin(phi0)*cos(theta1),
            l_hip*sin(phi0) - l_knee*(-sin(theta1)*sin(theta2)*cos(phi0) + cos(phi0)*cos(theta1)*cos(theta2)) - l_thigh*cos(phi0)*cos(theta1)
        ])
        
        return p_foot
    
    def get_p_knee_3Dleg(self, q):
        phi0 = q[0]
        theta1 = q[1]
        theta2 = q[2]
        
        l_hip = self.sim_info['l_hip']
        l_thigh = self.sim_info['l_thigh']
        l_knee = self.sim_info['l_knee']
        
        p_calf = np.array([
            -l_thigh*sin(theta1),
            l_hip*cos(phi0) + l_thigh*sin(phi0)*cos(theta1),
            l_hip*sin(phi0) - l_thigh*cos(phi0)*cos(theta1)
        ])
        
        return p_calf    
    
    def get_p_hip_3Dleg(self, q):
        phi0 = q[0]
        theta1 = q[1]
        theta2 = q[2]
        
        l_hip = self.sim_info['l_hip']
        l_thigh = self.sim_info['l_thigh']
        l_knee = self.sim_info['l_knee']
        
        p_hip = np.array([
            0,
            l_hip*cos(phi0),
            l_hip*sin(phi0)
        ])
        
        return p_hip
    
    def get_mass_mat(self, phi0, theta1, theta2, omega0, omega1, omega2, l_hip, l_thigh, l_knee, m_foot, m_calf, m_knee, m_thigh, m_hip, Ihx, Ihy, Ihz, Itx, Ity, Itz, Icx, Icy, Icz, g):
        
        mass_mat = np.array([[1.0*Icx + 1.0*Ihx + 1.0*Itx + 0.25*l_hip**2*m_hip + 0.25*m_calf*((-2*l_hip*sin(phi0) + l_knee*cos(phi0)*cos(theta1 + theta2) + 2*l_thigh*cos(phi0)*cos(theta1))**2 + (2*l_hip*cos(phi0) + l_knee*sin(phi0)*cos(theta1 + theta2) + 2*l_thigh*sin(phi0)*cos(theta1))**2) + 1.0*m_foot*((-l_hip*sin(phi0) + l_knee*cos(phi0)*cos(theta1 + theta2) + l_thigh*cos(phi0)*cos(theta1))**2 + (l_hip*cos(phi0) + l_knee*sin(phi0)*cos(theta1 + theta2) + l_thigh*sin(phi0)*cos(theta1))**2) + 1.0*m_knee*(l_hip**2 + l_thigh**2*cos(theta1)**2) + 0.25*m_thigh*(4*l_hip**2 + l_thigh**2*cos(theta1)**2), l_hip*(0.5*l_knee*m_calf*sin(theta1 + theta2) + 1.0*l_knee*m_foot*sin(theta1 + theta2) + 1.0*l_thigh*m_calf*sin(theta1) + 1.0*l_thigh*m_foot*sin(theta1) + 1.0*l_thigh*m_knee*sin(theta1) + 0.5*l_thigh*m_thigh*sin(theta1)), l_hip*l_knee*(0.5*m_calf + 1.0*m_foot)*sin(theta1 + theta2)], [l_hip*(0.5*l_knee*m_calf*sin(theta1 + theta2) + 1.0*l_knee*m_foot*sin(theta1 + theta2) + 1.0*l_thigh*m_calf*sin(theta1) + 1.0*l_thigh*m_foot*sin(theta1) + 1.0*l_thigh*m_knee*sin(theta1) + 0.5*l_thigh*m_thigh*sin(theta1)), 1.0*Ihy*cos(phi0)**2 + 1.0*Ihz*sin(phi0)**2 + 1.0*Ity*cos(phi0)**2 + 1.0*Itz*sin(phi0)**2 + 1.0*l_thigh**2*m_knee + 0.25*l_thigh**2*m_thigh + 0.25*m_calf*(l_knee**2 + 4*l_knee*l_thigh*cos(theta2) + 4*l_thigh**2) + 1.0*m_foot*(l_knee**2 + 2*l_knee*l_thigh*cos(theta2) + l_thigh**2), l_knee*(0.25*m_calf*(l_knee + 2*l_thigh*cos(theta2)) + 1.0*m_foot*(l_knee + l_thigh*cos(theta2)))], [l_hip*l_knee*(0.5*m_calf + 1.0*m_foot)*sin(theta1 + theta2), l_knee*(0.25*m_calf*(l_knee + 2*l_thigh*cos(theta2)) + 1.0*m_foot*(l_knee + l_thigh*cos(theta2))), 1.0*Icy*cos(phi0)**2 + 1.0*Icz*sin(phi0)**2 + 0.25*l_knee**2*m_calf + 1.0*l_knee**2*m_foot]])
        
        return mass_mat
    
    def get_N_vec(self, phi0, theta1, theta2, omega0, omega1, omega2, l_hip, l_thigh, l_knee, m_foot, m_calf, m_knee, m_thigh, m_hip, Ihx, Ihy, Ihz, Itx, Ity, Itz, Icx, Icy, Icz, g):
        N_vec = np.array([[0.5*Icy*omega2**2*sin(2*phi0) - 0.5*Icz*omega2**2*sin(2*phi0) + 0.5*Ihy*omega1**2*sin(2*phi0) - 0.5*Ihz*omega1**2*sin(2*phi0) + 0.5*Ity*omega1**2*sin(2*phi0) - 0.5*Itz*omega1**2*sin(2*phi0) + 1.0*g*l_hip*m_calf*cos(phi0) + 1.0*g*l_hip*m_foot*cos(phi0) + 0.5*g*l_hip*m_hip*cos(phi0) + 1.0*g*l_hip*m_knee*cos(phi0) + 1.0*g*l_hip*m_thigh*cos(phi0) - 0.25*g*l_knee*m_calf*sin(-phi0 + theta1 + theta2) + 0.25*g*l_knee*m_calf*sin(phi0 + theta1 + theta2) - 0.5*g*l_knee*m_foot*sin(-phi0 + theta1 + theta2) + 0.5*g*l_knee*m_foot*sin(phi0 + theta1 + theta2) + 0.5*g*l_thigh*m_calf*sin(phi0 - theta1) + 0.5*g*l_thigh*m_calf*sin(phi0 + theta1) + 0.5*g*l_thigh*m_foot*sin(phi0 - theta1) + 0.5*g*l_thigh*m_foot*sin(phi0 + theta1) + 0.5*g*l_thigh*m_knee*sin(phi0 - theta1) + 0.5*g*l_thigh*m_knee*sin(phi0 + theta1) + 0.25*g*l_thigh*m_thigh*sin(phi0 - theta1) + 0.25*g*l_thigh*m_thigh*sin(phi0 + theta1) + 0.5*l_hip*l_knee*m_calf*omega1**2*cos(theta1 + theta2) + 1.0*l_hip*l_knee*m_calf*omega1*omega2*cos(theta1 + theta2) + 0.5*l_hip*l_knee*m_calf*omega2**2*cos(theta1 + theta2) + 1.0*l_hip*l_knee*m_foot*omega1**2*cos(theta1 + theta2) + 2.0*l_hip*l_knee*m_foot*omega1*omega2*cos(theta1 + theta2) + 1.0*l_hip*l_knee*m_foot*omega2**2*cos(theta1 + theta2) + 1.0*l_hip*l_thigh*m_calf*omega1**2*cos(theta1) + 1.0*l_hip*l_thigh*m_foot*omega1**2*cos(theta1) + 1.0*l_hip*l_thigh*m_knee*omega1**2*cos(theta1) + 0.5*l_hip*l_thigh*m_thigh*omega1**2*cos(theta1) - 0.25*l_knee**2*m_calf*omega0*omega1*sin(2*theta1 + 2*theta2) - 0.25*l_knee**2*m_calf*omega0*omega2*sin(2*theta1 + 2*theta2) - 1.0*l_knee**2*m_foot*omega0*omega1*sin(2*theta1 + 2*theta2) - 1.0*l_knee**2*m_foot*omega0*omega2*sin(2*theta1 + 2*theta2) - 1.0*l_knee*l_thigh*m_calf*omega0*omega1*sin(2*theta1 + theta2) - 0.5*l_knee*l_thigh*m_calf*omega0*omega2*sin(theta2) - 0.5*l_knee*l_thigh*m_calf*omega0*omega2*sin(2*theta1 + theta2) - 2.0*l_knee*l_thigh*m_foot*omega0*omega1*sin(2*theta1 + theta2) - 1.0*l_knee*l_thigh*m_foot*omega0*omega2*sin(theta2) - 1.0*l_knee*l_thigh*m_foot*omega0*omega2*sin(2*theta1 + theta2) - 1.0*l_thigh**2*m_calf*omega0*omega1*sin(2*theta1) - 1.0*l_thigh**2*m_foot*omega0*omega1*sin(2*theta1) - 1.0*l_thigh**2*m_knee*omega0*omega1*sin(2*theta1) - 0.25*l_thigh**2*m_thigh*omega0*omega1*sin(2*theta1)], [-1.0*Ihy*omega0*omega1*sin(2*phi0) + 1.0*Ihz*omega0*omega1*sin(2*phi0) - 1.0*Ity*omega0*omega1*sin(2*phi0) + 1.0*Itz*omega0*omega1*sin(2*phi0) + 0.25*g*l_knee*m_calf*sin(-phi0 + theta1 + theta2) + 0.25*g*l_knee*m_calf*sin(phi0 + theta1 + theta2) + 0.5*g*l_knee*m_foot*sin(-phi0 + theta1 + theta2) + 0.5*g*l_knee*m_foot*sin(phi0 + theta1 + theta2) - 0.5*g*l_thigh*m_calf*sin(phi0 - theta1) + 0.5*g*l_thigh*m_calf*sin(phi0 + theta1) - 0.5*g*l_thigh*m_foot*sin(phi0 - theta1) + 0.5*g*l_thigh*m_foot*sin(phi0 + theta1) - 0.5*g*l_thigh*m_knee*sin(phi0 - theta1) + 0.5*g*l_thigh*m_knee*sin(phi0 + theta1) - 0.25*g*l_thigh*m_thigh*sin(phi0 - theta1) + 0.25*g*l_thigh*m_thigh*sin(phi0 + theta1) + 0.125*l_knee**2*m_calf*omega0**2*sin(2*theta1 + 2*theta2) + 0.5*l_knee**2*m_foot*omega0**2*sin(2*theta1 + 2*theta2) + 0.5*l_knee*l_thigh*m_calf*omega0**2*sin(2*theta1 + theta2) - 1.0*l_knee*l_thigh*m_calf*omega1*omega2*sin(theta2) - 0.5*l_knee*l_thigh*m_calf*omega2**2*sin(theta2) + 1.0*l_knee*l_thigh*m_foot*omega0**2*sin(2*theta1 + theta2) - 2.0*l_knee*l_thigh*m_foot*omega1*omega2*sin(theta2) - 1.0*l_knee*l_thigh*m_foot*omega2**2*sin(theta2) + 0.5*l_thigh**2*m_calf*omega0**2*sin(2*theta1) + 0.5*l_thigh**2*m_foot*omega0**2*sin(2*theta1) + 0.5*l_thigh**2*m_knee*omega0**2*sin(2*theta1) + 0.125*l_thigh**2*m_thigh*omega0**2*sin(2*theta1)], [-1.0*Icy*omega0*omega2*sin(2*phi0) + 1.0*Icz*omega0*omega2*sin(2*phi0) + 0.25*g*l_knee*m_calf*sin(-phi0 + theta1 + theta2) + 0.25*g*l_knee*m_calf*sin(phi0 + theta1 + theta2) + 0.5*g*l_knee*m_foot*sin(-phi0 + theta1 + theta2) + 0.5*g*l_knee*m_foot*sin(phi0 + theta1 + theta2) + 0.125*l_knee**2*m_calf*omega0**2*sin(2*theta1 + 2*theta2) + 0.5*l_knee**2*m_foot*omega0**2*sin(2*theta1 + 2*theta2) + 0.25*l_knee*l_thigh*m_calf*omega0**2*sin(theta2) + 0.25*l_knee*l_thigh*m_calf*omega0**2*sin(2*theta1 + theta2) + 0.5*l_knee*l_thigh*m_calf*omega1**2*sin(theta2) + 0.5*l_knee*l_thigh*m_foot*omega0**2*sin(theta2) + 0.5*l_knee*l_thigh*m_foot*omega0**2*sin(2*theta1 + theta2) + 1.0*l_knee*l_thigh*m_foot*omega1**2*sin(theta2)]])
        
        return N_vec