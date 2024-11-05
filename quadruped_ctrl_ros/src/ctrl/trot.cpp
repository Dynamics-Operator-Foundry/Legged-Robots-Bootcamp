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
        trot_start = true;
    }
    
    // 1. set phase + contact status
    calc_contact_phase();
    draw_gait(gait_viz, contact_gait);

    // 2. set lateral + yaw vel & base desired x, y, phi
    set_trot_vel();
    set_trot_base_desired();

    // 3. set foot traj
    set_foot_traj();

    // 4. set force
    set_trot_force(); // set force, force will then transformed into torque
    set_trot_swing(); // set swing, posi and velo will then transformed into q, dq

    // 5. send cmd to joints
    set_trot_cmd();
}

void ctrl_server::set_trot_vel()
{
    trot_vel_B = Eigen::Vector2d(
        0.0,
        0.0
    );

    trot_vel_yaw = -gait_vlim_B(2) * 0.5;
}

void ctrl_server::set_trot_base_desired()
{
    Eigen::Vector3d posi_base_now = pose_SE3_robot_base.translation();
    Eigen::Vector3d velo_base_now = twist_robot_base.head(3);

    double yaw_now = q2rpy(pose_SE3_robot_base.unit_quaternion())(2);
    double dyaw_now = twist_robot_base(5);

    trot_vel_I = pose_SE3_robot_base.rotationMatrix() * 
        Eigen::Vector3d(
            trot_vel_B.x(), 
            trot_vel_B.y(), 
            0
        );

    // lateral movement
        // velo setpoint
    trot_base_dposi_desired.x() = saturation_check(
        trot_vel_I.x(), 
        Eigen::Vector2d(
            velo_base_now.x()-0.2, 
            velo_base_now.x()+0.2
        )
    );
    trot_base_dposi_desired.y() = saturation_check(
        trot_vel_I.y(), 
        Eigen::Vector2d(
            velo_base_now.y()-0.2, 
            velo_base_now.y()+0.2
        )
    );
    trot_base_dposi_desired.z() = 0;

        // posi setpoint += according to current velocity setpoint
    trot_base_posi_desired.x() = saturation_check(
        posi_base_now.x() + trot_base_dposi_desired.x() * (1.0 / ctrl_freq),
        Eigen::Vector2d(
            posi_base_now.x() - 0.05,
            posi_base_now.x() + 0.05
        )
    );
    trot_base_posi_desired.y() = saturation_check(
        posi_base_now.y() + trot_base_dposi_desired.y() * (1.0 / ctrl_freq),
        Eigen::Vector2d(
            posi_base_now.y() - 0.05,
            posi_base_now.y() + 0.05
        )
    );
    trot_base_posi_desired.z() = -neutral_stance(2,0);

    // rotation
    trot_base_atti_desired.setZero();
    trot_base_datti_desired.setZero();
        
        // velo yaw setpoint
    trot_base_datti_desired.z() = saturation_check(
        trot_vel_yaw, 
        Eigen::Vector2d(
            dyaw_now-0.2, 
            dyaw_now+0.2
        )
    );
        // posi yaw setpoint
    trot_base_atti_desired(2) =saturation_check(
        yaw_now + trot_base_datti_desired.z(), 
        Eigen::Vector2d(
            yaw_now-0.2, 
            yaw_now+0.2
        )
    );
}

void ctrl_server::set_trot_force()
{
    // in inertial frame
    Eigen::Vector3d acc_p = 
        Kp_p * (
            trot_base_posi_desired 
            - 
            pose_SE3_robot_base.translation()
        ) 
        +
        Kd_p * (
            trot_base_dposi_desired
            - 
            twist_robot_base.head(3)
        );

    Eigen::Matrix3d dR = rpy2q(trot_base_atti_desired).normalized().toRotationMatrix() * pose_SE3_robot_base.rotationMatrix().inverse();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(dR, Eigen::ComputeFullU | Eigen::ComputeFullV);
    dR = svd.matrixU() * svd.matrixV().transpose();
    if (dR.determinant() < 0)
        dR = -dR;  
    
    Eigen::Vector3d acc_w =  // Eigen::Vector3d::Zero();
        Kp_w * rotMatToExp(dR) 
        + 
        Kd_w * (trot_base_datti_desired - twist_robot_base.tail(3));

    Sophus::Vector6d acc;
    acc.head(3) = acc_p;
    acc.tail(3) = acc_w;

    acc(0) = saturation_check(acc(0), Eigen::Vector2d(-3,3));
    acc(1) = saturation_check(acc(1), Eigen::Vector2d(-3,3));
    acc(2) = saturation_check(acc(2), Eigen::Vector2d(-5,5));
    acc(3) = saturation_check(acc(3), Eigen::Vector2d(-40,40));
    acc(4) = saturation_check(acc(4), Eigen::Vector2d(-40,40));
    acc(5) = saturation_check(acc(5), Eigen::Vector2d(-10,10));

    std::vector<Eigen::Vector3d> feet_rBF_I; // vector from base to foot in {I}
    std::vector<Eigen::Vector3d> feet_posi_I;
    std::vector<Eigen::Vector3d> feet_velo_I;

    for (int leg_i = 0; leg_i < leg_no; leg_i ++)
    {
        feet_posi_I.emplace_back(pose_SE3_robot_base.rotationMatrix() * get_foot_p_B(leg_i) + pose_SE3_robot_base.translation());
        feet_velo_I.emplace_back(pose_SE3_robot_base.rotationMatrix() * forward_diff_kinematics(leg_i));

        feet_rBF_I.emplace_back(pose_SE3_robot_base.rotationMatrix() * get_foot_p_B(leg_i));
    }

    f_now = (-1) * get_f(feet_rBF_I, acc, contact_gait);
    f_prev = f_now;

    for (int leg_i = 0; leg_i <leg_no; leg_i++)
    {
        
        if(contact_gait(leg_i) == 0)
        {
            f_now.segment(leg_i * 3, 3) = 
            Kps_trot * (swing_feet_posi_I[leg_i] - feet_posi_I[leg_i]) 
            + Kds_trot * (swing_feet_velo_I[leg_i] - feet_velo_I[leg_i])
            ;   
        }
    }

    set_tau(f_now);
}

void ctrl_server::set_trot_swing()
{
    Eigen::Vector3d p_swing_now_L;
    Eigen::Vector3d v_swing_now_L;
    Eigen::Vector3d temp;

    Eigen::Matrix3d rot_B2I = pose_SE3_robot_base.rotationMatrix();
    Eigen::Matrix3d rot_I2B = pose_SE3_robot_base.rotationMatrix().inverse();
    Eigen::Vector3d base_posi = pose_SE3_robot_base.translation();

    Eigen::Vector3d v_base_I = twist_robot_base.head(3);
    Eigen::Vector3d w_base_I = twist_robot_base.tail(3);
    Eigen::Vector3d base2foot;

    for (int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        p_swing_now_L = rot_I2B * (swing_feet_posi_I[leg_i] - base_posi) - r_all_base2hip[leg_i];

        q_swing_gait_target.emplace_back(
            inverse_kinematics(leg_i, p_swing_now_L)
        );

        v_swing_now_L = rot_I2B * (swing_feet_velo_I[leg_i] - twist_robot_base.head(3));
        dq_swing_gait_target.emplace_back(
            inverse_diff_kinematics(
                leg_i,
                p_swing_now_L,
                v_swing_now_L
            )
        );
    }

    for(auto what : q_swing_gait_target)
    {
        if(what.hasNaN())
            ros::shutdown();
    }
}

void ctrl_server::set_trot_cmd()
{
    // after the above functions, we now have
    // f_now;
    // q_swing_gait_target;
    // dq_swing_gait_target;

    for(int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        if(contact_gait(leg_i) == 0)
        {
            // swing -> Kp larger
            cmdSet.motorCmd[leg_i*3+0].mode = 10;
            cmdSet.motorCmd[leg_i*3+0].Kp = 3;
            cmdSet.motorCmd[leg_i*3+0].Kd = 2;
            cmdSet.motorCmd[leg_i*3+1].mode = 10;
            cmdSet.motorCmd[leg_i*3+1].Kp = 3;
            cmdSet.motorCmd[leg_i*3+1].Kd = 2;
            cmdSet.motorCmd[leg_i*3+2].mode = 10;
            cmdSet.motorCmd[leg_i*3+2].Kp = 3;
            cmdSet.motorCmd[leg_i*3+2].Kd = 2;
        }
        else
        {
            cmdSet.motorCmd[leg_i*3+0].mode = 10;
            cmdSet.motorCmd[leg_i*3+0].Kp = 0.8;
            cmdSet.motorCmd[leg_i*3+0].Kd = 0.8;
            cmdSet.motorCmd[leg_i*3+1].mode = 10;
            cmdSet.motorCmd[leg_i*3+1].Kp = 0.8;
            cmdSet.motorCmd[leg_i*3+1].Kd = 0.8;
            cmdSet.motorCmd[leg_i*3+2].mode = 10;
            cmdSet.motorCmd[leg_i*3+2].Kp = 0.8;
            cmdSet.motorCmd[leg_i*3+2].Kd = 0.8;
        }

        cmdSet.motorCmd[leg_i*3+0].tau = balance_tau[leg_i*3+0];
        cmdSet.motorCmd[leg_i*3+1].tau = balance_tau[leg_i*3+1];
        cmdSet.motorCmd[leg_i*3+2].tau = balance_tau[leg_i*3+2];

        cmdSet.motorCmd[leg_i*3+0].q = q_swing_gait_target[leg_i](0);
        cmdSet.motorCmd[leg_i*3+1].q = q_swing_gait_target[leg_i](1);
        cmdSet.motorCmd[leg_i*3+2].q = q_swing_gait_target[leg_i](2);

        cmdSet.motorCmd[leg_i*3+0].dq = dq_swing_gait_target[leg_i](0);
        cmdSet.motorCmd[leg_i*3+1].dq = dq_swing_gait_target[leg_i](1);
        cmdSet.motorCmd[leg_i*3+2].dq = dq_swing_gait_target[leg_i](2);
    }
}


void ctrl_server::trot_ctrl_reset()
{
    trot_start = false;
}
