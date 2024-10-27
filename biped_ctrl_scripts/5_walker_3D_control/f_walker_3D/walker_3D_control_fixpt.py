import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import time as time
import sympy as sp
from sympy import *
import scipy.optimize as opt


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './dynamics/compiled_funcs')))

# IMPORT FROM CYTHON
from B_matrix.wrapper_module_0 import autofunc_c as get_B_matrix_cy
from Mass_matrix.wrapper_module_0 import autofunc_c as get_Mass_matrix_cy
from J_l_ss.wrapper_module_0 import autofunc_c as get_J_l_ss_cy
from J_r_ss.wrapper_module_0 import autofunc_c as get_J_r_ss_cy
from Jdot_l_ss.wrapper_module_0 import autofunc_c as get_Jdot_l_ss_cy
from Jdot_r_ss.wrapper_module_0 import autofunc_c as get_Jdot_r_ss_cy
from collision.wrapper_module_0 import autofunc_c as get_collision_cy
from P_L.wrapper_module_0 import autofunc_c as get_P_L_cy
from P_R.wrapper_module_0 import autofunc_c as get_P_R_cy

# print('gan')
# exit()

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util

# SAVE DATA FOR VISUALIZATION
t_all = []
x0_all_rk4 = []
x1_all_rk4 = []
x2_all_rk4 = []
x3_all_rk4 = []
x4_all_rk4 = []
x5_all_rk4 = []
x6_all_rk4 = []
x7_all_rk4 = []
x8_all_rk4 = []
x9_all_rk4 = []
x10_all_rk4 = []
x11_all_rk4 = []
x12_all_rk4 = []
x13_all_rk4 = []

# FOR VISUALIZATION
def save_data(q):
    x0_all_rk4.append(q[0])
    x1_all_rk4.append(q[1])
    x2_all_rk4.append(q[2])
    x3_all_rk4.append(q[3])
    x4_all_rk4.append(q[4])
    x5_all_rk4.append(q[5])
    x6_all_rk4.append(q[6])
    x7_all_rk4.append(q[7])
    x8_all_rk4.append(q[8])
    x9_all_rk4.append(q[9])
    x10_all_rk4.append(q[10])
    x11_all_rk4.append(q[11])
    x12_all_rk4.append(q[12])
    x13_all_rk4.append(q[13])
    
    return

def traj_setting(t0, tf, p0, pf, v0, vf, a0, af):
    
    A = np.array([
        [1, t0, t0**2, t0**3, t0**4, t0**5],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
        ])
    b = np.array([p0, pf, v0, vf, a0, af])
    
    return np.linalg.solve(A,b)

def get_ref(pp, tt, tf_one_step):
    if tt>tf_one_step:
        tt = tf_one_step
    # print(tt)
    x_ref_return = np.zeros((8,3))
    for i in range(8):
        p0 = pp[i,0]
        p1 = pp[i,1]
        p2 = pp[i,2]
        p3 = pp[i,3]
        p4 = pp[i,4]
        p5 = pp[i,5]
        
        x_ref_return[i,0] = p5*tt**5 + p4*tt**4 + p3*tt**3 + p2*tt**2 + p1*tt + p0
        x_ref_return[i,1] = 5*p5*tt**4 + 4*p4*tt**3 + 3*p3*tt**2 + 2*p2*tt + p1
        x_ref_return[i,2] = 20*p5*tt**3 + 12*p4*tt**2 + 6*p3*tt + 2*p2
        
        x_ref_return[i,0]
    
    return x_ref_return

def f_single_stance(x,u,param_kine, param_dyna, which_leg):
    q = x[0:14]
    qdot = x[14:28]
    
    x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk = q
    dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk = qdot
    
    ddx,ddy,ddz,ddroll,ddpitch,ddyaw, ddroll_lh,ddpitch_lh,ddyaw_lh,ddpitch_lk, ddroll_rh,ddpitch_rh,ddyaw_rh,ddpitch_rk = np.zeros(14)
    
    w, l0, l1, l2 = param_kine
    g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz, F = param_dyna
    
    mass_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    M_mat = get_Mass_matrix_cy(*mass_matrix_argu)
    
    N_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, dx, dy, dz, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, ddx, ddy, ddz, ddroll, ddpitch, ddyaw, ddroll_lh, ddpitch_lh, ddyaw_lh, ddpitch_lk, ddroll_rh, ddpitch_rh, ddyaw_rh, ddpitch_rk, w, l0, l1, l2, g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    N_vec = get_B_matrix_cy(*N_matrix_argu)
    
    B_ctrl_mat = np.zeros((17,8))
    B_ctrl_mat[6:14,0:8] = np.identity(8)
    
    if which_leg == 'l':
        J_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, w, l1, l2]
        Jdot_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, w, l1, l2]
        
        J_l_ss = get_J_l_ss_cy(*J_l_argu)
        Jdot_l_ss = get_Jdot_l_ss_cy(*Jdot_l_argu)
        
        AA = np.zeros((17,17))
        bb = np.zeros((17,1))
        
        AA[0:14, 0:14] = M_mat
        AA[0:14, 14:17] = -J_l_ss.transpose()
        AA[14:17, 0:14] = J_l_ss
        
        bb[0:14] = N_vec
        # if which_leg == 'l':
        #     pass
        #     print()
        #     print('here')
        #     print(qdot)
        #     print()
            # exit()
        bb[14:17] = -np.array([Jdot_l_ss @ qdot]).transpose() 
        # u = np.zeros((8,1))
        bb = bb + B_ctrl_mat @ u
        
        x_new = np.linalg.solve(AA,bb)
        
    elif which_leg == 'r':
        J_r_argu = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l1, l2]
        Jdot_r_argu = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, droll, dpitch, dyaw, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, w, l1, l2]    
        
        J_r_ss = get_J_r_ss_cy(*J_r_argu)
        Jdot_r_ss = get_Jdot_r_ss_cy(*Jdot_r_argu)
        
        AA = np.zeros((17,17))
        bb = np.zeros((17,1))
        
        AA[0:14, 0:14] = M_mat
        AA[0:14, 14:17] = -J_r_ss.transpose()
        AA[14:17, 0:14] = J_r_ss
        
        bb[0:14] = N_vec
        bb[14:17] = -np.array([Jdot_r_ss @ qdot]).transpose()
        bb = bb + B_ctrl_mat @ u
        
        x_new = np.linalg.solve(AA,bb)
    else:   
        print("CHECK WHICH_LEG")
        exit()
        
    xdot = np.array([*qdot, *x_new[0:14].flatten()])
    
    return xdot

def gen_control_partitioning(x, traj_coeff, t_now, tf_one_step, param_kine, param_dyna, which_leg):
    q = x[0:14]
    qdot = x[14:28]
    
    x_ref = get_ref(pp=traj_coeff, tt=t_now, tf_one_step=tf_one_step)
    s_ref = x_ref[:,0]
    v_ref = x_ref[:,1]
    a_ref = x_ref[:,2]
    
    x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk = q
    dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk = qdot
    
    ddx,ddy,ddz,ddroll,ddpitch,ddyaw, ddroll_lh,ddpitch_lh,ddyaw_lh,ddpitch_lk, ddroll_rh,ddpitch_rh,ddyaw_rh,ddpitch_rk = np.zeros(14)
    
    w, l0, l1, l2 = param_kine
    g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz, F = param_dyna
    
    mass_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    M = get_Mass_matrix_cy(*mass_matrix_argu)
    
    N_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, dx, dy, dz, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, ddx, ddy, ddz, ddroll, ddpitch, ddyaw, ddroll_lh, ddpitch_lh, ddyaw_lh, ddpitch_lk, ddroll_rh, ddpitch_rh, ddyaw_rh, ddpitch_rk, w, l0, l1, l2, g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    
    N = get_B_matrix_cy(*N_matrix_argu)
    
    B_ctrl_mat = np.zeros((17,8))
    B_ctrl_mat[6:14,0:8] = np.identity(8)
    
    S = None
    AA = np.zeros((17,17))
    bb = np.zeros((17,1))
    
    if which_leg == 'l':
        J_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, w, l1, l2]
        Jdot_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, w, l1, l2]
        
        J_l_ss = get_J_l_ss_cy(*J_l_argu)
        Jdot_l_ss = get_Jdot_l_ss_cy(*Jdot_l_argu)
    
        AA[0:14, 0:14] = M
        AA[0:14, 14:17] = -J_l_ss.transpose()
        AA[14:17, 0:14] = J_l_ss
        
        bb[0:14] = N
        bb[14:17] = -np.array([Jdot_l_ss @ qdot]).transpose() 
        
        S_L = np.array([
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # roll
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # pitch
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # yaw
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], # roll_rh
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], # pitch_rh
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], # yaw_rh
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0], # pitch_lk
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]  # pitch_rk
        ])
        # when left leg on ground, we are desired to control roll, pitch, yaw of the revolution between torso and hip
        # and also the roll, pitch, yaw of r-hip, and pitch of l- & r- knee
        # the controller, yet, it based on the revolusion of hips and knees
        S = S_L
        
        s = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_lk, pitch_rk]
        v = [droll, dpitch, dyaw, droll_rh, dpitch_rh, dyaw_rh, dpitch_lk, dpitch_rk]
        
    elif which_leg == 'r':
        J_r_argu = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l1, l2]
        Jdot_r_argu = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, droll, dpitch, dyaw, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, w, l1, l2] 
           
        J_r_ss = get_J_r_ss_cy(*J_r_argu)
        Jdot_r_ss = get_Jdot_r_ss_cy(*Jdot_r_argu)
        
        AA[0:14, 0:14] = M
        AA[0:14, 14:17] = -J_r_ss.transpose()
        AA[14:17, 0:14] = J_r_ss
        
        bb[0:14] = N
        bb[14:17] = -np.array([Jdot_r_ss @ qdot]).transpose()
        
        S_R = np.array([
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # roll
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # pitch
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # yaw
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # roll_lh
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], # pitch_rlh
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0], # yaw_lh
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0], # pitch_lk
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]  # pitch_rk
        ])
        # when right leg on ground, we are desired to control roll, pitch, yaw of the revolution between torso and hip
        # and also the roll, pitch, yaw of l-hip, and pitch of l- & r- knee
        # the controller, yet, it based on the revolusion of hips and knees
        S = S_R
        
        s = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, pitch_rk]
        v = [droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, dpitch_rk]
        
    else:   
        print("CHECK WHICH_LEG")
        exit()
        
    Kp = 100
    Kd = 2 * np.sqrt(Kp)
    AAinv = np.linalg.inv(AA)
    
    
    q_ddot_c = a_ref + Kd*(v_ref-v) + Kp*(s_ref-s);
    
    # print()
    
    # SAinvB = S @ AAinv @ B_ctrl_mat
    # SAinvB_inv = np.linalg.inv(SAinvB)
    # tau = SAinvB_inv @ (q_ddot_c - S @ AAinv @ bb);

    A_lin = S @ AAinv @ B_ctrl_mat    
    q_ddot_c = np.array([q_ddot_c]).transpose()
    b_lin = (q_ddot_c - S @ AAinv @ bb)
    
    tau = np.linalg.solve(A_lin,b_lin)
    # print('tau')
    # print(tau.shape)
    return tau

def get_collision(x, param_kine):
    q = x[0:14]
    x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk = q
    w, l0, l1, l2 = param_kine
    
    colli_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l1, l2]
    colli = get_collision_cy(*colli_argu)
    
    return colli

def f_foot_strike(x, param_kine, param_dyna, which_leg):
    q = x[0:14]
    qdot_minus = x[14:28]
    
    x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk = q
    dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk = qdot_minus
    
    w, l0, l1, l2 = param_kine
    g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz, F = param_dyna
    
    J_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, w, l1, l2]
    J_r_argu = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l1, l2]
    
    J_l_fs = get_J_l_ss_cy(*J_l_argu)
    J_r_fs = get_J_r_ss_cy(*J_r_argu)
    
    P_L_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, l1, l2, F]
    P_R_argu = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, l1, l2, F]
    
    P_L = get_P_L_cy(*P_L_argu)
    P_R = get_P_R_cy(*P_R_argu)
    
    mass_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    M = get_Mass_matrix_cy(*mass_matrix_argu)
    
    A_fs = np.zeros((17,17))
    b_fs = np.zeros((17,1))
    
    if which_leg == 'r':
        J_st = J_r_fs # stance
        J_sw = J_l_fs # swing
        
        # P_R = np.zeros((3,1))
        b_fs[0:14] = J_st.transpose() @ P_R + np.array([M @ qdot_minus]).T
        
        A_fs[0:14, 0:14] = M
        A_fs[0:14, 14:17] = -J_sw.transpose()
        A_fs[14:17, 0:14] = J_sw
                
        x_new = np.linalg.solve(A_fs, b_fs)
        
        # print("HEREREERERE")
        # print(F)
        # print()
        # # print(J_sw.transpose() @ P_R)
        # # print(np.array([M @ qdot_minus]))
        # print(J_st)
        # # print(P_R)
        
        # print(b_fs)
        # print()
        # print('here')
        # print()
        # print(x_new)
        # exit()
#    1.422403916953874
#    0.168988534317824
#    0.500620129819512

#   -0.012972034380743
#   -0.058312984939275
#    1.538845716650764

#   -0.144273084817784
#   -0.472876768116811
#   -1.600741574501896
#   -1.897382814466572

#    0.439880719465252
#   -1.006952358253448
#    1.992826876177882
#   -0.656080499837396

#  -21.627344275122404
#   -0.852207633085055
#   49.543701408820830
        # exit()
        
    elif which_leg == 'l':
        J_st = J_l_fs # stance
        J_sw = J_r_fs # swing
        
        # print(P_R.shape)
        lala = np.array([M @ qdot_minus])
        # print(lala.shape)
        b_fs[0:14] = J_st.transpose() @ P_L + np.array([M @ qdot_minus]).T
        
        A_fs[0:14, 0:14] = M
        # print()
        # print(J_sw.shape)
        A_fs[0:14, 14:17] = -J_sw.transpose()
        A_fs[14:17, 0:14] = J_sw
                
        x_new = np.linalg.solve(A_fs, b_fs)
    else:
        print("CHECK WHICH_LEG")
        exit()
    
    
    x = np.array([*q, *x_new[0:14].flatten()])
    # print(x[14:28])
    # exit()
    return x

# START HERE!
##################################################
# 1. INITIALIZATION
def init(q_fix, qdot_fix):
    
    l0 = 1;
    l1 = 0.5;
    l2 = 0.5;
    w = 0.1;

    g = 9.8
    mb = 70
    mt = 10
    mc = 5
    Ibx = 5
    Iby = 3
    Ibz = 2
    Itx = 1
    Ity = 0.3
    Itz = 2
    Icx = 0.5
    Icy = 0.15
    Icz = 1

    F = 0.18 * (mb + 2 * mt + 2 * mc) * np.sqrt(g * (l1+l2))

    param_kine = [w, l0, l1, l2]
    param_dyna = [g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz, F]

    # INITIAL STATE
    roll = q_fix[0]
    droll = qdot_fix[0]
    pitch = q_fix[1]
    dpitch = qdot_fix[1]
    yaw = q_fix[2]
    dyaw = qdot_fix[2]
    roll_lh = q_fix[3]
    droll_lh = qdot_fix[3]
    pitch_lh = q_fix[4]
    dpitch_lh = qdot_fix[4]
    yaw_lh = q_fix[5]
    dyaw_lh = qdot_fix[5]
    pitch_lk = q_fix[6]
    dpitch_lk = qdot_fix[6]
    roll_rh = q_fix[7]
    droll_rh = qdot_fix[7]
    pitch_rh = q_fix[8]
    dpitch_rh = qdot_fix[8]
    yaw_rh = q_fix[9]
    dyaw_rh = qdot_fix[9]
    pitch_rk = q_fix[10]
    dpitch_rk = qdot_fix[10]

    x = 0
    y = 0
    z = 0
    dx = 0
    dy = 0
    dz = 0

    q = np.array([x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk])

    qdot = np.array([dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk])

    which_leg = 'r' # we start with left leg

    if which_leg == 'l':
        p_Hip_I = sim3D().get_p_Hip_L_init(0,0,0,*q[3:],*param_kine)
        v_Hip_I = sim3D().get_v_Hip_L_init(0,0,0,*q[3:],0,0,0,*qdot[3:],*param_kine)
    elif which_leg == 'r':
        p_Hip_I = sim3D().get_p_Hip_R_init(0,0,0,*q[3:],*param_kine)
        v_Hip_I = sim3D().get_v_Hip_R_init(0,0,0,*q[3:],0,0,0,*qdot[3:],*param_kine)
        pass
    else:
        print("CHECK WHICH_LEG")
        exit()

    q[0:3] = p_Hip_I
    qdot[0:3] = v_Hip_I

    x_rk4 = np.array([*q,*qdot])
    
    return x_rk4, param_kine, param_dyna, which_leg
# exit()

# TESTING

# test_funcs()

##################################################
# 2. FIND FIX POINT FOR POINCARE MAP
# temporarily omitted here
roll = 0.000000000000000
droll = 0.000000000000000
pitch = -0.000000000000000
dpitch = 0.000000000000001
yaw = -0.000000000000006
dyaw = -0.000000000000003

roll_lh = 0.000000000000006
droll_lh = 0.000000000000043
pitch_lh = -0.000000000000002
dpitch_lh = 0.000000000000070
yaw_lh = -0.000000000000013
dyaw_lh = -0.000000000000057
pitch_lk = -0.999999999999981
dpitch_lk = 0.000000000000061
roll_rh = -0.029247773468329
droll_rh = 0.054577886084082
pitch_rh = -0.001551040824746
dpitch_rh = -1.029460778006675
yaw_rh = 0.054690088280553
dyaw_rh = 0.559306810759302
pitch_rk = -0.000000000000001
dpitch_rk = -0.000000000000031

q_fix = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk]
qdot_fix = [droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk]

##################################################
# 3. SET TRAJECTORY
def get_traj_coeff(x_rk4, which_leg):
    # print("GET TRAJ")
    x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh, pitch_rk = x_rk4[0:14]
    dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk = x_rk4[14:28]
    
    if which_leg == 'r':
        x_ref_0 = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, pitch_rk]
        # x_ref_0 = [0.0, -0.0, -6e-15, 6e-15, -2e-15, -1.3e-14, -0.999999999999981, -1e-15]
        v_ref_0 = [droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, dpitch_rk]
    elif which_leg == 'l':
        x_ref_0 = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_lk, pitch_rk]
        # x_ref_0 = [0.0, -0.0, -6e-15, 6e-15, -2e-15, -1.3e-14, -0.999999999999981, -1e-15]
        v_ref_0 = [droll, dpitch, dyaw, droll_rh, dpitch_rh, dyaw_rh, dpitch_lk, dpitch_rk]
    else:
        print("CHECK WHICH_LEG")
        exit()

    x_ref_f = [0,0,0,0,0.375,0,0,0]

    # if which_leg == 'l':
        # exit()
    v_ref_f = [0,0,0,0,0,0,0,0]
    a_ref_0 = [0,0,0,0,0,0,0,0]
    a_ref_f = [0,0,0,0,0,0,0,0]

    ctrller_dof = 8
    traj_coeffs = np.zeros((8,6))

    tf = 0.2
    
    for i in range(ctrller_dof):    
            traj_coeffs[i,:] = traj_setting(
                t0=0,
                tf=tf,
                p0=x_ref_0[i],
                pf=x_ref_f[i],
                v0=v_ref_0[i],
                vf=v_ref_f[i],
                a0=a_ref_0[i],
                af=a_ref_f[i],
            )
            
    # print(traj_coeffs)
    # print()

    return traj_coeffs, tf

sample_factor = 10
acc_factor = 1
def draw_anime(success, param_kine):
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "passive_walker_control_partition"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "passive_walker_control_partition" + "_failed"
    print('FPS:', 1000 / (1000 * t_step * sample_factor))
    print('ACC:', acc_factor)
    w, l0, l1, l2 = param_kine
    sim3D().anime(
        t=t_all[::sample_factor * acc_factor],
        x_states=[
            x0_all_rk4[::sample_factor * acc_factor],
            x1_all_rk4[::sample_factor * acc_factor],
            x2_all_rk4[::sample_factor * acc_factor],
            x3_all_rk4[::sample_factor * acc_factor],
            x4_all_rk4[::sample_factor * acc_factor],
            x5_all_rk4[::sample_factor * acc_factor],
            x6_all_rk4[::sample_factor * acc_factor],
            x7_all_rk4[::sample_factor * acc_factor],
            x8_all_rk4[::sample_factor * acc_factor],
            x9_all_rk4[::sample_factor * acc_factor],
            x10_all_rk4[::sample_factor * acc_factor],
            x11_all_rk4[::sample_factor * acc_factor],
            x12_all_rk4[::sample_factor * acc_factor],
            x13_all_rk4[::sample_factor * acc_factor],
        ],
        ms=10,
        mission='Walk',
        sim_object='3Dwalker',
        sim_info={'w': w, 'l0': l0, 'l1': l1, 'l2': l2},
        save=False,
        save_name=save_name
    )
    exit()

# t_all.append(0)
# q = x_rk4[0:14]
# save_data(q)
# draw_anime(True, param_kine)

x_rk4, param_kine, param_dyna, which_leg = init(q_fix, qdot_fix)
# print(x_rk4)
# exit()
no_of_steps = 2
fsm = 'single_stance'
t_now = 0
t_step = 1e-3
last_event = 0
walk_i = 0

t_step_start = t_now

def P(x):

    roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk = x[0:11]
    
    droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk = x[11:22]
    
    x = 0
    y = 0
    z = 0
    dx = 0
    dy = 0
    dz = 0
    
    q = np.array([x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk])
    qdot = np.array([dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk])
    p_Hip_I = sim3D().get_p_Hip_R_init(0,0,0,*q[3:],*param_kine)
    v_Hip_I = sim3D().get_v_Hip_R_init(0,0,0,*q[3:],0,0,0,*qdot[3:],*param_kine)
    
    q[0:3] = p_Hip_I
    qdot[0:3] = v_Hip_I

    x_rk4 = np.array([*q,*qdot])
    
    fsm = 'single_stance'
    t_now = 0
    t_step = 1e-3
    last_event = 0
    t_step_start = t_now
    while True:
        if fsm == 'single_stance':
            # print(which_leg)
            traj_coeff, tf_one_step = get_traj_coeff(x_rk4, which_leg)
            while True:           
                # print(t_now-t_step_start) 
                u = gen_control_partitioning(
                    x=x_rk4,
                    traj_coeff=traj_coeff,
                    t_now=t_now-t_step_start,
                    tf_one_step=tf_one_step,
                    param_kine=param_kine,
                    param_dyna=param_dyna,
                    which_leg=which_leg
                )
                
                x_rk4_new = inte().rkdp(
                    f_single_stance,
                    x=x_rk4,
                    u=u,
                    h=t_step,
                    args=[param_kine, param_dyna, which_leg],
                    ctrl_on=True
                )
                
                if x_rk4[2] < -1e-4 or x_rk4[2] > 2 + 1e-4:
                    return np.nan * np.ones_like(x_rk4)
                    # print(x_rk4[2])
                    # draw_anime(False, param_kine=param_kine)
            
                t_now = t_now + t_step    
                x_rk4 = x_rk4_new
                
                current_event = get_collision(x=x_rk4, param_kine=param_kine)
            
                if current_event * last_event < 0:
                    # print()
                    # print(current_event)
                    # print(last_event)
                    # print(current_event * last_event)
                    # print()
                    fsm = 'foot_strike'
                    # print("FOOT STRIKE!!!!!!!!!!!!!!!!!!!")
                    
                    break
                last_event = current_event
                # print(t_now)
        elif fsm == 'foot_strike':  
            x_rk4 = f_foot_strike(
                x=x_rk4,
                param_kine=param_kine,
                param_dyna=param_dyna,
                which_leg=which_leg
            )
            
            return x_rk4

def f_lala(x):

    result = P(x)
    if np.any(np.isnan(result)):  # Check for NaN in the output
        return np.full_like(result, np.inf)  # Return infinities for failed cases

    x_obj_init = np.array([
        *x[0:3], *x[11:14], 
        *x[3:7], *x[14:18],   
        *x[7:11], *x[18:22]]) # l, r
    
    print(x_obj_init.shape)
    
    x_obj_end = np.array([
        *result[3:6], *result[17:20], 
        *result[10:14], *result[24:28],
        *result[6:10], *result[20:24],]) # r, l
    
    print(x_obj_end.shape)

    print("HERE AFTER ONE ITERATION IN OPTIMIZATION")
    # print(x)
    print(np.linalg.norm(x_obj_end - x_obj_init))
    print()

    return x_obj_init - x_obj_end

# x_obj = np.array([-8.55352329e-03, -1.13522480e-03,  7.61806003e-03,  8.52454327e-02,-4.64678981e-02,  6.88216266e-03, -4.56955923e-02, -3.86937936e-04,2.11474667e-02, -9.42578850e-02,  4.35923378e-03, -3.35324985e-01, 1.70532805e-01, -1.54311993e-01,  6.41431781e-01,  8.75178710e-01, 1.86190894e-01, -1.22362330e+00, -4.86739622e-01,  2.74367696e-01,-4.17924127e-01, -3.80963886e-02])

# x_obj = np.array([
#  -0.04389255, -0.06571901, -0.00175553,  0.07567546,  0.02671035,  0.00323625,
#  -0.03994828, -0.02345805, -0.05655095, -0.2925124,  -0.01527097, -0.53886243,
#  -0.19287034, -0.23827584,  0.54603717,  0.33599716,  0.32457124, -0.86617886,
#  -0.650402,    0.0638597,  -0.31429277, -0.13284889
#  ])

x_obj = np.zeros(22)
print(x_obj)
# exit()

# x_lala = opt.fsolve(f_lala, x_obj, xtol=1e-6,)
x_lala = opt.root(f_lala, x_obj,method='hybr', tol=1e-6,)

print(x_lala)
print("END FIXPOINT SEARCH")
# x_rk4 = x_lala
exit()

while True:
    if fsm == 'single_stance':
        print(which_leg)
        traj_coeff, tf_one_step = get_traj_coeff(x_rk4, which_leg)
        while True:           
            # print(t_now-t_step_start) 
            u = gen_control_partitioning(
                x=x_rk4,
                traj_coeff=traj_coeff,
                t_now=t_now-t_step_start,
                tf_one_step=tf_one_step,
                param_kine=param_kine,
                param_dyna=param_dyna,
                which_leg=which_leg
            )
            
            x_rk4_new = inte().rkdp(
                f_single_stance,
                x=x_rk4,
                u=u,
                h=t_step,
                args=[param_kine, param_dyna, which_leg],
                ctrl_on=True
            )
            
            if x_rk4[2] < -1e-4 or x_rk4[2] > 2 + 1e-4:
                print(x_rk4[2])
                draw_anime(False, param_kine=param_kine)
            
            save_data(x_rk4_new[0:14])
            t_now = t_now + t_step
            t_all.append(t_now)
            
            x_rk4 = x_rk4_new
            
            current_event = get_collision(x=x_rk4, param_kine=param_kine)
            # print(current_event)
            if current_event * last_event < 0:
                print()
                print(current_event)
                print(last_event)
                print(current_event * last_event)
                print()
                fsm = 'foot_strike'
                print("FOOT STRIKE!!!!!!!!!!!!!!!!!!!")
                
                break
            last_event = current_event
            # print(t_now)
    elif fsm == 'foot_strike':  
        x_rk4 = f_foot_strike(
            x=x_rk4,
            param_kine=param_kine,
            param_dyna=param_dyna,
            which_leg=which_leg
        )
        
        if which_leg == 'l':
            which_leg = 'r'
        elif which_leg == 'r':
            which_leg = 'l'
        else:
            print("CHECK WHICH_LEG")
            exit()
        
        fsm = 'single_stance'
        last_event = 0
        t_step_start = t_now
        
        walk_i = walk_i + 1
        
        if walk_i > no_of_steps:
            draw_anime(True, param_kine=param_kine)
            break
        
        print("ONE STEP END,", walk_i)
        print("============\n")
        
        
        
        