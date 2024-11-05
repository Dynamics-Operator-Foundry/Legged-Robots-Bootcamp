/*
    This file is part of quadruped_ctrl_ros - learning material for quadruped control

    quadruped_ctrl_ros is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    quadruped_ctrl_ros is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with quadruped_ctrl_ros.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file ctrl_trot.cpp
 * \date 25/10/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include "quadruped_ctrl_ros/ctrl_server.h"

void ctrl_server::balance_ctrl()
{
    if (!balance_track_start)
    {
        set_balance_ctrl();
        set_balance_ctrl_gain();
        balance_track_start = true;
    }   

    if (balance_fsm == "X") 
        posi_delta_B.x() = x_mag * sin(ctrl_param);
    else if (balance_fsm == "Y") 
        posi_delta_B.y() = y_mag * sin(ctrl_param);
    else if (balance_fsm == "Z") 
        posi_delta_B.z() = z_mag * sin(ctrl_param);
    else 
        yaw_base = yaw_mag * sin(ctrl_param);
    
    ctrl_param = ctrl_param + 2 * M_PI / 3.0 * 1 / ctrl_freq;

    if (ctrl_param > 2 * M_PI)
    {
        ctrl_param = 0;
        posi_delta_B.setZero();

        if (balance_fsm == "X") 
            balance_fsm = "Y";
        else if (balance_fsm == "Y") 
            balance_fsm = "Z";
        else if (balance_fsm == "Z") 
            balance_fsm = "YAW";
        else 
            balance_fsm = "X";
    }

    // in inertial frame
    Eigen::Vector3d acc_p = 
        Kp_p * (
            posi_base_I + pose_SE3_robot_base.rotationMatrix() * posi_delta_B - 
            pose_SE3_robot_base.translation()
        ) 
        +
        Kd_p * (
            Eigen::Vector3d::Zero() 
            - 
            // pose_SE3_robot_base.rotationMatrix() * 
            twist_robot_base.head(3)
        );

    Eigen::Matrix3d dR = rpy2q(Eigen::Vector3d(0.0,0.0,yaw_base)).normalized().toRotationMatrix() * pose_SE3_robot_base.rotationMatrix().inverse();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(dR, Eigen::ComputeFullU | Eigen::ComputeFullV);
    dR = svd.matrixU() * svd.matrixV().transpose();
    if (dR.determinant() < 0)
        dR = -dR;  
    
    Eigen::Vector3d acc_w = // Eigen::Vector3d::Zero();
        Kp_w * rotMatToExp(dR) 
        + 
        Kd_w * (Eigen::Vector3d::Zero() - twist_robot_base.tail(3));

    std::vector<Eigen::Vector3d> feet_posi_I;

    for (int leg_i = 0; leg_i < leg_no; leg_i ++)
        feet_posi_I.emplace_back(pose_SE3_robot_base.rotationMatrix() * get_foot_p_B(leg_i));
        
    Sophus::Vector6d acc;
    acc.head(3) = acc_p;
    acc.tail(3) = acc_w;

    f_now = (-1) * get_f(feet_posi_I, acc, contact_gait);
    f_prev = f_now;
    set_tau(f_now);

    for(int i = 0; i < DoF; i++)
        cmdSet.motorCmd[i].tau = balance_tau[i];
}

void ctrl_server::set_balance_ctrl()
{
    x_mag = 0.04;
    y_mag = 0.04;
    z_mag = 0.04;
    yaw_mag = 20 * M_PI / 180;

    Kp_p = Eigen::Vector3d(150,150,150).asDiagonal();
    Kd_p = Eigen::Vector3d(25, 25, 25).asDiagonal();

    Kp_w = 200;
    Kd_w = Eigen::Vector3d(30, 30, 30).asDiagonal();

    posi_base_I = pose_SE3_robot_base.translation();
    posi_delta_B.setZero();

    yaw_base = q2rpy(pose_SE3_robot_base.unit_quaternion())(2);

    m = 12.0;
    mI = Eigen::Vector3d(0.0792, 0.2085, 0.2265).asDiagonal();

    contact_gait.setConstant(1);

    f_prev.resize(12);
    f_prev.setZero();

    Eigen::Matrix<double, 6, 1> s;
    s << 20, 20, 50, 450, 450, 450; 
    S_w = s.asDiagonal();

    Eigen::Matrix<double, 12, 1> w;
    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
    W_w = w.asDiagonal();

    Eigen::Matrix<double, 12, 1> u;
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    U_w = u.asDiagonal();
    alpha = 0.001;
    beta = 0.1;

    ctrl_param = 0;

    balance_fsm = "X";
}

void ctrl_server::set_balance_ctrl_gain()
{
    for(int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        cmdSet.motorCmd[leg_i*3+0].mode = 10;
        cmdSet.motorCmd[leg_i*3+0].dq = 0;
        cmdSet.motorCmd[leg_i*3+0].Kp = 0.8;
        cmdSet.motorCmd[leg_i*3+0].Kd = 0.8;

        cmdSet.motorCmd[leg_i*3+1].mode = 10;
        cmdSet.motorCmd[leg_i*3+1].dq = 0;
        cmdSet.motorCmd[leg_i*3+1].Kp = 0.8;
        cmdSet.motorCmd[leg_i*3+1].Kd = 0.8;

        cmdSet.motorCmd[leg_i*3+2].mode = 10;
        cmdSet.motorCmd[leg_i*3+2].dq = 0;
        cmdSet.motorCmd[leg_i*3+2].Kp = 0.8;
        cmdSet.motorCmd[leg_i*3+2].Kd = 0.8;
    }
}

void ctrl_server::balance_ctrl_reset()
{
    balance_track_start = false;
}

Eigen::VectorXd ctrl_server::get_f(
    std::vector<Eigen::Vector3d> feet_posi, 
    Sophus::Vector6d acc,
    Eigen::Vector4i contact_i
)
{
    set_HMat(feet_posi);
    set_fVec(acc);
    set_constraints();

    qp_opt(HMat, fVec, AMat, lbVec, ubVec);
    // Q, g, A, lb, ub

    return getQpsol();
}

void ctrl_server::set_HMat(
    const std::vector<Eigen::Vector3d> feet_posi
)
{
    // A
    A_dyn.setZero();
    A_dyn.block<3,3>(0,0).setIdentity();
    A_dyn.block<3,3>(0,3).setIdentity();
    A_dyn.block<3,3>(0,6).setIdentity();
    A_dyn.block<3,3>(0,9).setIdentity();
    A_dyn.block<3,3>(3,0) = Sophus::SO3d::hat(feet_posi[0]);
    A_dyn.block<3,3>(3,3) = Sophus::SO3d::hat(feet_posi[1]);
    A_dyn.block<3,3>(3,6) = Sophus::SO3d::hat(feet_posi[2]);
    A_dyn.block<3,3>(3,9) = Sophus::SO3d::hat(feet_posi[3]);

    // HMat
    HMat = A_dyn.transpose() * S_w * A_dyn + alpha * W_w + beta * U_w;
}

void ctrl_server::set_fVec(const Sophus::Vector6d acc)
{
    // b
    Eigen::Matrix<double, 6, 1> b;
    b.head(3) = m * (acc.head(3) - Eigen::Vector3d(0,0,-9.81));
    b.tail(3) = pose_SE3_robot_base.rotationMatrix() * mI * pose_SE3_robot_base.rotationMatrix().inverse() * acc.tail(3);

    // fVec
    fVec = -1 * b.transpose() * S_w * A_dyn - f_prev.transpose() * beta * U_w;
}

void ctrl_server::set_constraints()
{   
    // std::cout<<"here in set constraints"<<std::endl;
    // std::cout<<contact_gait<<std::endl<<std::endl;

    int leg_no_in_air = 0;
    std::vector<int> which_legs_in_air, which_legs_on_ground;
 
    for (int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        if(contact_gait(leg_i) == 0)
        {
            leg_no_in_air++;
            which_legs_in_air.emplace_back(leg_i);
        }
        else
        {
            which_legs_on_ground.emplace_back(leg_i);   
        }
    }

    int leg_no_on_ground = (leg_no - leg_no_in_air);

    int row_no = leg_no_on_ground * 5 + leg_no_in_air * 3;
    int col_no = 12;
    AMat.resize(row_no, col_no);
    AMat.setZero();

    ubVec.resize(row_no, 1);
    ubVec.setZero(); 

    int inf_size = (leg_no - leg_no_in_air) * 5;
    ubVec.block(0, 0, inf_size, 1).setConstant(INFINITY);

    lbVec.resize(row_no, 1);
    lbVec.setZero();

    Eigen::Matrix<double, 5, 3> oneblock;
    oneblock <<
         1, 0, mu,
        -1, 0 ,mu,
         0, 1, mu,
         0, -1, mu,
         0, 0, 1;

    for (int i = 0; i < which_legs_on_ground.size(); i++)
    {
        AMat.block<5,3>(i*5,which_legs_on_ground[i]*3) = oneblock;
    }

    for (int i = 0; i < which_legs_in_air.size(); i++)
    {
        AMat.block<3,3>(leg_no_on_ground * 5 + i*3,which_legs_in_air[i]*3) = Eigen::Matrix3d::Identity();
    }

    // std::cout<<contact_gait<<std::endl;
    // std::cout<<AMat<<std::endl<<std::endl;
    // std::cout<<ubVec<<std::endl<<std::endl;;
    // std::cout<<lbVec<<std::endl<<std::endl;;
}

void ctrl_server::set_tau(
    Eigen::Matrix<double, 12, 1> f_I
)
{
    for (int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        balance_tau.segment(leg_i * 3, 3) = 
            get_Jacobian(leg_i).transpose() * 
            pose_SE3_robot_base.rotationMatrix().inverse() * 
            f_I.segment(leg_i * 3, 3);
    }
}

Eigen::Vector3d ctrl_server::rotMatToExp(const Eigen::Matrix3d& rm)
{
    double cosValue = rm.trace()/2.0-1/2.0;
    if(cosValue > 1.0f)
    {
        cosValue = 1.0f;
    }
    else if(cosValue < -1.0f)
    {
        cosValue = -1.0f;
    }

    double angle = acos(cosValue);
    Eigen::Vector3d exp;
    if (fabs(angle) < 1e-5)
    {
        exp=Eigen::Vector3d(0,0,0);
    }
    else if (fabs(angle - M_PI) < 1e-5)
    {
        exp = angle * Eigen::Vector3d(rm(0,0)+1, rm(0,1), rm(0,2)) / sqrt(2*(1+rm(0, 0)));
    }
    else
    {
        exp=angle/(2.0f*sin(angle))*Eigen::Vector3d(rm(2,1)-rm(1,2),rm(0,2)-rm(2,0),rm(1,0)-rm(0,1));
    }

    return exp;
}