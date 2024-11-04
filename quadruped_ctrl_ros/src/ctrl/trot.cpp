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
 * \date 25/10/20244
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include "quadruped_ctrl_ros/ctrl_server.h"

void ctrl_server::trot_ctrl()
{
    if(!trot_start)
    {
        set_gait_params();
    }
    
    calc_contact_phase();
    draw_gait(gait_viz, contact_gait);

    set_trot_vel();
    set_trot_base_desired();

    set_gait();

    set_trot_force();
    
}

void ctrl_server::set_trot_vel()
{
    trot_vel_I = Eigen::Vector3d(gait_vlim_B(0) * 0.5,0,0);
}

void ctrl_server::set_trot_base_desired()
{
    // lateral movement
        // get current posi
    trot_base_posi_desired = pose_SE3_robot_base.translation();
    
        // posi setpoint += according to current velocity
    trot_base_dposi_desired.x() = saturation_check(
        trot_vel_I.x(), 
        Eigen::Vector2d(
            trot_vel_I.x()-0.2, 
            trot_vel_I.x()+0.2
        )
    );
    trot_base_dposi_desired.y() = saturation_check(
        trot_vel_I.y(), 
        Eigen::Vector2d(
            trot_vel_I.y()-0.2, 
            trot_vel_I.y()+0.2
        )
    );
    trot_base_dposi_desired.z() = 0;

    trot_base_posi_desired.x() = saturation_check(
        trot_base_posi_desired.x() + trot_base_dposi_desired.x() * (1.0 / ctrl_freq),
        Eigen::Vector2d(
            trot_base_posi_desired.x() - 0.05,
            trot_base_posi_desired.x() + 0.05
        )
    );
    trot_base_posi_desired.y() = saturation_check(
        trot_base_posi_desired.y() + trot_base_dposi_desired.y() * (1.0 / ctrl_freq),
        Eigen::Vector2d(
            trot_base_posi_desired.y() - 0.05,
            trot_base_posi_desired.y() + 0.05
        )
    );
    trot_base_posi_desired.z() = -neutral_stance(2,0);

    // rotation
    trot_base_atti_desired.setZero();
}

void ctrl_server::set_trot_force()
{
    // in inertial frame
    Eigen::Vector3d acc_p = 
        Kp_p * (
            trot_base_posi_desired - 
            pose_SE3_robot_base.translation()
        ) 
        +
        Kd_p * (
            trot_base_dposi_desired
            - 
            pose_SE3_robot_base.rotationMatrix() * 
            twist_robot_base.head(3)
        );

    Eigen::Matrix3d dR = rpy2q(trot_base_atti_desired).normalized().toRotationMatrix() * pose_SE3_robot_base.rotationMatrix().inverse();

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

    acc(0) = saturation_check(acc(0), Eigen::Vector2d(-3,3));
    acc(1) = saturation_check(acc(1), Eigen::Vector2d(-3,3));
    acc(2) = saturation_check(acc(2), Eigen::Vector2d(-5,5));
    acc(3) = saturation_check(acc(3), Eigen::Vector2d(-40,40));
    acc(4) = saturation_check(acc(4), Eigen::Vector2d(-40,40));
    acc(5) = saturation_check(acc(5), Eigen::Vector2d(-10,10));

    // f_now = (-1) * get_f(feet_posi_I, acc);
    f_prev = f_now;
}