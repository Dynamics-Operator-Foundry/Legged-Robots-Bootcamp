import sympy as sp
import time as time
import numpy as np
import pickle

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './libs')))

from cython_acc_func import cython_acc_func as cy

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Walker3DModelling as model

def gen_func_file(expr_name, expr, var, datatype):
    file_name = 'get_'+expr_name + '.py'
    func_name = 'get_'+expr_name
    print(func_name)
    folder_des = os.path.abspath(os.path.join(os.path.dirname(__file__), './funcs'))
    
    model().gen_func(
        file_name=file_name,
        func_name=func_name,
        arguments=(*var,),
        expression=expr,
        return_object=expr_name,
        folder_name=folder_des,
        datatype=datatype
    )
    
    return

def main():
    t_now = time.time()
    
    x, y, z = sp.symbols('x y z', real=True)
    roll, pitch, yaw = sp.symbols('roll pitch yaw',real=True)
    roll_lh, pitch_lh, yaw_lh = sp.symbols('roll_lh pitch_lh yaw_lh',real=True)
    roll_rh, pitch_rh, yaw_rh = sp.symbols('roll_rh pitch_rh yaw_rh',real=True)
    pitch_lk, pitch_rk = sp.symbols('pitch_lk pitch_rk',real=True)

    w, l0, l1, l2 = sp.symbols('w l0 l1 l2', real=True)

    dx, dy, dz = sp.symbols('dx dy dz', real=True)
    droll, dpitch, dyaw = sp.symbols('droll dpitch dyaw',real=True)
    droll_lh, dpitch_lh, dyaw_lh = sp.symbols('droll_lh dpitch_lh dyaw_lh',real=True)
    droll_rh, dpitch_rh, dyaw_rh = sp.symbols('droll_rh dpitch_rh dyaw_rh',real=True)
    dpitch_lk, dpitch_rk = sp.symbols('dpitch_lk dpitch_rk',real=True)

    u = sp.symbols('u0 u1 u2', real=True)
    r = sp.symbols('r0 r1 r2', real=True)

    I = sp.Matrix([[1,0,0],
                [0,1,0],
                [0,0,1]])

    # TRANSFORMATION
    hip_mid = sp.Matrix([x,y,z])
    T_O_2_I = sp.Matrix([[I,hip_mid],
                        [0,0,0,1]]) # from ZRM {O} to {I}
    R_O_2_I = I # from ZRM {O} to {I}

    # torso
    # roll -> pitch -> yaw
        # roll
    r = sp.Matrix([0,0,0])
    u = sp.Matrix([1,0,0])
    T_UR_2_O = cy().T_ZRM_F(u,r,roll) 
    # from upperbody {U} (roll) to ZRM {O}
    R_UR_2_O = T_UR_2_O[0:3,0:3] 
    # exit()
    # from upperbody {U} (roll) to ZRM {O}
    omega_UR_2_O = droll * u

        # pitch
    r = sp.Matrix([0,0,0])
    u = sp.Matrix([0,1,0])
    T_UP_2_UR = cy().T_ZRM_F(u,r,pitch) 
    # from upperbody {U} (pitch) to from upperbody {U} (roll)
    R_UP_2_UR = T_UP_2_UR[0:3,0:3] 
    
    # from upperbody {U} (pitch) to from upperbody {U} (roll)
    omega_UP_2_UR = dpitch * u 

        # yaw
    r = sp.Matrix([0,0,0])
    u = sp.Matrix([0,0,1])
    T_UY_2_UP = cy().T_ZRM_F(u,r,yaw) 
    # from upperbody {U} (yaw) to upperbody {U} (pitch)
    R_UY_2_UP = T_UY_2_UP[0:3,0:3] 
    # from upperbody {U} (yaw) to upperbody {U} (pitch)
    omega_UY_2_UP = dyaw * u # omega_12

    # left-side
    # yaw -> roll -> pitch
        # yaw
    r = sp.Matrix([0,w,0])
    u = sp.Matrix([0,0,1])
    T_LHY_2_UY = cy().T_ZRM_F(u,r,yaw_lh) 
    # from left-hip {LH} (yaw) to upperbody {U} (yaw)
    R_LHY_2_UY = T_LHY_2_UY[0:3,0:3] 
    # from left-hip {LH} (yaw) to upperbody {U} (yaw)
    omega_LHY_2_UY = dyaw_lh * u

        # roll
    r = sp.Matrix([0,w,0])
    u = sp.Matrix([1,0,0])
    T_LHR_2_LHY = cy().T_ZRM_F(u,r,roll_lh) 
    # from left-hip {LH} (roll) to left-hip {LH} (yaw)
    R_LHR_2_LHY = T_LHR_2_LHY[0:3,0:3]
    # from left-hip {LH} (roll) to left-hip {LH} (yaw)
    omega_LHR_2_LHY = droll_lh * u

        # pitch
    r = sp.Matrix([0,w,0])
    u = sp.Matrix([0,-1,0])
    T_LHP_2_LHR = cy().T_ZRM_F(u,r,pitch_lh) 
    # from left-hip {LH} (pitch) to left-hip {LH} (roll)
    R_LHP_2_LHR = T_LHP_2_LHR[0:3,0:3]
    # from left-hip {LH} (pitch) to left-hip {LH} (roll)
    omega_LHP_2_LHR = dpitch_lh * u

        # knee
    r = sp.Matrix([0,w,-l1])
    u = sp.Matrix([0,-1,0])
    T_LK_2_LHP = cy().T_ZRM_F(u,r,pitch_lk) 
    R_LK_2_LHP = T_LK_2_LHP[0:3,0:3]
    # from left-knee {LK} to left-hip {LH} (pitch)
    omega_LK_2_LHP = dpitch_lk * u
    

    # right-side
    # yaw -> roll -> pitch
    # yaw -> roll -> pitch
        # yaw
    r = sp.Matrix([0,-w,0])
    u = sp.Matrix([0,0,-1])
    T_RHY_2_UY = cy().T_ZRM_F(u,r,yaw_rh) 
    # from right-hip {RH} (yaw) to upperbody {U} (yaw)
    R_RHY_2_UY = T_RHY_2_UY[0:3,0:3] 
    # from right-hip {RH} (yaw) to upperbody {U} (yaw)
    omega_RHY_2_UY = dyaw_rh * u

        # roll
    r = sp.Matrix([0,-w,0])
    u = sp.Matrix([-1,0,0])
    T_RHR_2_RHY = cy().T_ZRM_F(u,r,roll_rh) 
    # from right-hip {RH} (roll) to right-hip {RH} (yaw)
    R_RHR_2_RHY = T_RHR_2_RHY[0:3,0:3]
    # from right-hip {RH} (roll) to right-hip {RH} (yaw)
    omega_RHR_2_RHY = droll_rh * u

        # pitch
    r = sp.Matrix([0,-w,0])
    u = sp.Matrix([0,-1,0])
    T_RHP_2_RHR = cy().T_ZRM_F(u,r,pitch_rh) 
    # from right-hip {RH} (pitch) to right-hip {RH} (roll)
    R_RHP_2_RHR = T_RHP_2_RHR[0:3,0:3]
    # from right-hip {RH} (pitch) to right-hip {RH} (roll)
    omega_RHP_2_RHR = pitch_rh * u

        # knee
    r = sp.Matrix([0,-w,-l1])
    u = sp.Matrix([0,-1,0])
    T_RK_2_RHP = cy().T_ZRM_F(u,r,pitch_rk) 
    R_RK_2_RHP = T_RK_2_RHP[0:3,0:3] 
    # from right-knee {RK} to right-hip {RH} (pitch)
    omega_RK_2_RHP = pitch_rk * u
    print("GOT ALL TRANSFORMATIONS")
    
    ##################################

    # POSITION VECTORS
    p_B = hip_mid # position of base

    T_UY_2_I = T_O_2_I @ T_UR_2_O @ T_UP_2_UR @ T_UY_2_UP
    # from upper body (yaw) {U} (yaw) to {I}
    p_H = T_UY_2_I @ sp.Matrix([0,0,l0,1]) 
    # position of head

    # left
    T_LHP_2_I = T_UY_2_I @ T_LHY_2_UY @ T_LHR_2_LHY @ T_LHP_2_LHR 
    # from left-hip (pitch) {LH} (yaw) to {I}
    p_LH = T_LHP_2_I @ sp.Matrix([0,w,0,1]) 
    # position of left hip

    T_LK_2_I = T_LHP_2_I @ T_LK_2_LHP
    # from left-knee (pitch) {LK} (yaw) to {I}
    p_LK = T_LK_2_I @ sp.Matrix([0,w,-l1,1])
    # position of left knee (topology after joint knee)
    p_LA = T_LK_2_I @ sp.Matrix([0,w,-(l1+l2),1])
    # position of left ankle

    # right
    T_RHP_2_I = T_UY_2_I @ T_RHY_2_UY @ T_RHR_2_RHY @ T_RHP_2_RHR 
    # from right-hip (pitch) {RH} (yaw) to {I}
    p_RH = T_RHP_2_I @ sp.Matrix([0,-w,0,1]) 
    # position of right hip

    T_RK_2_I = T_RHP_2_I @ T_RK_2_RHP
    # from right-knee (pitch) {RK} (yaw) to {I}
    p_RK = T_RK_2_I @ sp.Matrix([0,-w,-l1,1])
    # position of right knee (topology after joint knee)
    p_RA = T_RK_2_I @ sp.Matrix([0,-w,-(l1+l2),1])
    # position of right ankle

    # center of masses
    p_Torso = T_UY_2_I @ sp.Matrix([0,0,l0/2,1]) 
    # COM of torso
    p_Thigh_L = T_LHP_2_I @ sp.Matrix([0,w,-l1/2,1]) 
    p_Calf_L = T_LK_2_I @ sp.Matrix([0,w,-(l1+l2/2),1]) 
    p_Thigh_R = T_RHP_2_I @ sp.Matrix([0,-w,-l1/2,1])
    p_Calf_R = T_RK_2_I @ sp.Matrix([0,-w,-(l1+l2/2),1]) 


    # hip_mid @ init
    p_Hip_L_init = -p_LA # with x = 0, y = 0, z = 0
    p_Hip_R_init = -p_RA # with x = 0, y = 0, z = 0
    # hip_mid + p_LA = ankle position
    # when start, ankle position is on ground, and set this as the starting point and infer the rest
    
    print("GOT ALL POSITIONS")
    ##################################
    
    # DYNAMICS STARTS HERE (drop annotation->too much)
    
    # VELOCITIES VECTORS
    omega_13 = omega_UR_2_O + R_UR_2_O@omega_UP_2_UR
    R13 = R_UR_2_O@R_UP_2_UR
    omega_14 = omega_13 + R13@omega_UY_2_UP
    R14 = R13@R_UY_2_UP
    
    omega_15l = omega_14 + R14@omega_LHY_2_UY
    omega_15r = omega_14 + R14@omega_RHY_2_UY
    
    R15l = R14@R_LHY_2_UY
    omega_16l = omega_15l + R15l@omega_LHR_2_LHY
    R15r = R14@R_RHY_2_UY
    omega_16r = omega_15r + R15r@omega_RHR_2_RHY
    R16l = R15l@R_LHR_2_LHY
    omega_17l = omega_16l + R16l@omega_LHP_2_LHR
    R16r = R15r@R_RHR_2_RHY
    omega_17r = omega_16r + R16r@omega_RHP_2_RHR
    R17l = R16l@R_LHP_2_LHR
    omega_18l = omega_17l + R17l@omega_LK_2_LHP;
    R17r = R16r@R_RHP_2_RHR
    omega_18r = omega_17r + R17r@omega_RK_2_RHP;
    
    R18l = R17l@R_LK_2_LHP
    R18r = R17r@R_RK_2_RHP
    
    omegaB_2 = omega_UR_2_O
    omegaB_3 = omega_UP_2_UR + R_UP_2_UR@omegaB_2
    omegaB_4 = omega_UY_2_UP + R_UY_2_UP@omegaB_3;
    
    omegaB_5l = omega_LHY_2_UY + R_LHY_2_UY@omegaB_4;
    omegaB_6l = omega_LHR_2_LHY + R_LHR_2_LHY@omegaB_5l;
    omegaB_7l = omega_LHP_2_LHR + R_LHP_2_LHR@omegaB_6l;
    omegaB_8l = omega_LK_2_LHP + R_LK_2_LHP@omegaB_7l;
    omegaB_5r = omega_RHY_2_UY + R_RHY_2_UY@omegaB_4;
    omegaB_6r = omega_RHR_2_RHY + R_RHR_2_RHY@omegaB_5r;
    omegaB_7r = omega_RHP_2_RHR + R_RHP_2_RHR@omegaB_6r;
    omegaB_8r = omega_RK_2_RHP + R_RK_2_RHP@omegaB_7r;
    
    
    # foot impulse
    P = sp.symbols('P', real=True)
    I_LA = sp.zeros(3,3)
    I_RA = sp.zeros(3,3)
    
    I_LA[0] = P*(p_LK[0]-p_LA[0])/l2;
    I_LA[1] = P*(p_LK[1]-p_LA[1])/l2;
    I_LA[2] = P*(p_LK[2]-p_LA[2])/l2;
    I_RA[0] = P*(p_RK[0]-p_RA[0])/l2;
    I_RA[1] = P*(p_RK[1]-p_RA[1])/l2;
    I_RA[2] = P*(p_RK[2]-p_RA[2])/l2;

    ddx, ddy, ddz = sp.symbols('ddx ddy ddz', real=True)
    ddroll, ddpitch, ddyaw = sp.symbols('ddroll ddpitch ddyaw',real=True)
    ddroll_lh, ddpitch_lh, ddyaw_lh = sp.symbols('ddroll_lh ddpitch_lh ddyaw_lh',real=True)
    ddroll_rh, ddpitch_rh, ddyaw_rh = sp.symbols('ddroll_rh ddpitch_rh ddyaw_rh',real=True)
    ddpitch_lk, ddpitch_rk = sp.symbols('ddpitch_lk ddpitch_rk',real=True)
    
    g = sp.symbols('g', real=True)
    Ibx, Iby, Ibz = sp.symbols('Ibx, Iby, Ibz', real=True)
    Itx, Ity, Itz = sp.symbols('Itx, Ity, Itz', real=True)
    Icx, Icy, Icz = sp.symbols('Icx, Icy, Icz', real=True)
    mb, mt, mc = sp.symbols('mb, mt, mc', real=True) 
    
    # x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2
    q = [x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk,]
    qdot = [dx, dy, dz, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk]
    qddot = [ddx, ddy, ddz, ddroll, ddpitch, ddyaw, ddroll_lh, ddpitch_lh, ddyaw_lh, ddpitch_lk, ddroll_rh, ddpitch_rh, ddyaw_rh, ddpitch_rk]
    
    v_Torso = p_Torso.jacobian(q) @ sp.Matrix(qdot)
    v_Thigh_L = p_Thigh_L.jacobian(q) @ sp.Matrix(qdot)
    v_Thigh_R = p_Thigh_R.jacobian(q) @ sp.Matrix(qdot)
    v_Calf_L = p_Calf_L.jacobian(q) @ sp.Matrix(qdot)
    v_Calf_R = p_Calf_R.jacobian(q) @ sp.Matrix(qdot)
    
    print("GOT ALL VELOCITIES")
    ##################################
    
    # GET LAGRAGIAN
    
    I_torso = sp.diag(Ibx, Iby, Ibz)
    I_thigh = sp.diag(Itx, Ity, Itz)
    I_calf = sp.diag(Icx, Icy, Icz)
    
    T = 0.5 * (mb * v_Torso.dot(v_Torso) + mt * v_Thigh_L.dot(v_Thigh_L) + mc * v_Calf_L.dot(v_Calf_L) + mt * v_Thigh_R.dot(v_Thigh_R) + mc * v_Calf_R.dot(v_Calf_R)) + 0.5 * (omegaB_4.dot(I_torso @ omegaB_4) + omegaB_7l.dot(I_thigh @ omegaB_7l) + omegaB_8l.dot(I_calf @ omegaB_8l) + omegaB_7r.dot(I_thigh @ omegaB_7r) + omegaB_8r.dot(I_calf @ omegaB_8r))
    V = mb * g * p_Torso[2] + mt * g * p_Thigh_L[2] + mc * g * p_Calf_L[2] + mt * g * p_Thigh_R[2] + mc * g * p_Calf_R[2]
    
    L = T - V
    print("GOT LAGRAGIAN")
    ##################################
    dof = 14 # dimension of q
    
    folder_des = os.path.abspath(os.path.join(os.path.dirname(__file__), './compiled_funcs'))

    # Loading the saved equations
    # with open(folder_des + "/EOM_equations.pkl", "rb") as file:
    #     EOM_vec = pickle.load(file)
    # print(EOM_vec.shape)
    # print("Equations loaded successfully.")
    
    
    J_l_ss = sp.Matrix(p_LA[0:3]).jacobian(q)
    J_r_ss = sp.Matrix(p_RA[0:3]).jacobian(q)
    
    Jdot_l_ss = np.zeros([J_l_ss.shape[0], J_l_ss.shape[1]])
    Jdot_l_ss = sp.Matrix(Jdot_l_ss)
    
    Jdot_r_ss = np.zeros([J_r_ss.shape[0], J_r_ss.shape[1]])
    Jdot_r_ss = sp.Matrix(Jdot_r_ss)
    
    for i in range(Jdot_l_ss.shape[0]):
        for j in range(Jdot_l_ss.shape[1]):
            Jdot_l_ss[i, j] = sp.Matrix([J_l_ss[i,j]]).jacobian(q) @ sp.Matrix(qdot)
            Jdot_r_ss[i, j] = sp.Matrix([J_r_ss[i,j]]).jacobian(q) @ sp.Matrix(qdot)
                
    
    folder_des = os.path.abspath(os.path.join(os.path.dirname(__file__), './compiled_funcs'))
    with open(folder_des + "/J_l_ss.pkl", "wb") as file:
        pickle.dump(J_l_ss, file)
    with open(folder_des + "/J_r_ss.pkl", "wb") as file:
        pickle.dump(J_r_ss, file)
        
    with open(folder_des + "/Jdot_l_ss.pkl", "wb") as file:
        pickle.dump(Jdot_l_ss, file)
    with open(folder_des + "/Jdot_r_ss.pkl", "wb") as file:
        pickle.dump(Jdot_r_ss, file)
    
    var = [*q, w, l0, l1, l2]
    gen_func_file(
        expr_name='J_l_ss',
        expr=J_l_ss,
        var=var,
        datatype='mat'
    )
    gen_func_file(
        expr_name='J_r_ss',
        expr=J_r_ss,
        var=var,
        datatype='mat'
    )
    
    var = [*q, *qdot, w, l0, l1, l2]
    gen_func_file(
        expr_name='Jdot_l_ss',
        expr=Jdot_l_ss,
        var=var,
        datatype='mat'
    )
    gen_func_file(
        expr_name='Jdot_r_ss',
        expr=Jdot_r_ss,
        var=var,
        datatype='mat'
    )
    
    
    t_end = time.time()
    print('END, TIME: ', t_end - t_now)


if __name__ =="__main__":
    main()