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
 * \file ctrl_squiggle.cpp
 * \date 25/10/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include "quadruped_ctrl_ros/ctrl_server.h"

void ctrl_server::squiggle_ctrl()
{
    if(!swing_track_start)
    {
        set_squiggle_ctrl();
        set_squiggle_ctrl_gain();
    }

    if (squiggle_fsm == "ROW")
    {
        row_base = row_mag * sin(ctrl_param);
        ctrl_param = ctrl_param + 2.0 * M_PI / 6.0 * 1.0 / ctrl_freq;

        if (ctrl_param > 2 * M_PI)
        {
            ctrl_param = 0;
            squiggle_fsm = "PITCH";

            row_base = 0;
            pitch_base = 0;
            yaw_base = 0;
            height_base = 0;
        }
    }
    else if (squiggle_fsm == "PITCH")
    {
        pitch_base = pitch_mag * sin(ctrl_param);
        ctrl_param = ctrl_param + 2.0 * M_PI / 6.0 * 1.0 / ctrl_freq;

        if (ctrl_param > 2 * M_PI)
        {
            ctrl_param = 0;
            squiggle_fsm = "YAW";

            row_base = 0;
            pitch_base = 0;
            yaw_base = 0;
            height_base = 0;
        }
    }
    else if (squiggle_fsm == "YAW")
    {
        yaw_base = yaw_mag * sin(ctrl_param);
        ctrl_param = ctrl_param + 2.0 * M_PI / 6.0 * 1.0 / ctrl_freq;

        if (ctrl_param > 2 * M_PI)
        {
            ctrl_param = 0;
            squiggle_fsm = "HEIGHT";

            row_base = 0;
            pitch_base = 0;
            yaw_base = 0;
            height_base = 0;
        }
    }
    else if (squiggle_fsm == "HEIGHT")
    {
        height_base = height_mag * sin(ctrl_param);
        ctrl_param = ctrl_param + 2.0 * M_PI / 6.0 * 1.0 / ctrl_freq;

        if (ctrl_param > 2 * M_PI)
        {
            ctrl_param = 0;
            squiggle_fsm = "ROW";

            row_base = 0;
            pitch_base = 0;
            yaw_base = 0;
            height_base = 0;
        }
    }

    Eigen::Matrix3d _rot = rpy2q(
        Eigen::Vector3d(
            row_base, pitch_base, yaw_base
            )).toRotationMatrix();
    
    base_pose_S = Sophus::SE3d(_rot.normalized(), Eigen::Vector3d(0,0,height_base)); 
    // T_B_2_S
    // T_S_2_B = base_pose_S.inv()

    Sophus::SE3d T_S_2_B = base_pose_S.inverse();
    Eigen::Vector3d p_footFR_B = T_S_2_B.rotationMatrix() * p_footFR_S;
    Eigen::Vector3d p_footFL_B = T_S_2_B.rotationMatrix() * p_footFL_S;
    Eigen::Vector3d p_footRR_B = T_S_2_B.rotationMatrix() * p_footRR_S;
    Eigen::Vector3d p_footRL_B = T_S_2_B.rotationMatrix() * p_footRL_S;

    Eigen::Vector3d q_footFR_B = get_q_from_B(0, p_footFR_B);
    Eigen::Vector3d q_footFL_B = get_q_from_B(1, p_footFL_B);
    Eigen::Vector3d q_footRR_B = get_q_from_B(2, p_footRR_B);
    Eigen::Vector3d q_footRL_B = get_q_from_B(3, p_footRL_B);


}

void ctrl_server::set_squiggle_ctrl()
{
    base_S = -get_foot_p_B(0);
    base_pose_S = Sophus::SE3d(
        Eigen::Matrix3d::Identity(),
        base_S
    );

    p_footFR_S.setZero();
    p_footFL_S = base_S + get_foot_p_B(1);
    p_footRR_S = base_S + get_foot_p_B(2);
    p_footRL_S = base_S + get_foot_p_B(3);

    row_mag = 20 * M_PI / 180;
    pitch_mag = 15 * M_PI / 180;
    yaw_mag = 20 * M_PI / 180;
    height_mag = 0.04;

    squiggle_fsm = "ROW";
    row_base = 0;
    pitch_base = 0;
    yaw_base = 0;
    height_base = 0;
}

void ctrl_server::set_squiggle_ctrl_gain()
{
    for(int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        cmdSet.motorCmd[leg_i*3+0].mode = 10;
        cmdSet.motorCmd[leg_i*3+0].dq = 0;
        cmdSet.motorCmd[leg_i*3+0].Kp = 180;
        cmdSet.motorCmd[leg_i*3+0].Kd = 8;
        cmdSet.motorCmd[leg_i*3+0].tau = 0;

        cmdSet.motorCmd[leg_i*3+1].mode = 10;
        cmdSet.motorCmd[leg_i*3+1].dq = 0;
        cmdSet.motorCmd[leg_i*3+1].Kp = 180;
        cmdSet.motorCmd[leg_i*3+1].Kd = 8;
        cmdSet.motorCmd[leg_i*3+1].tau = 0;

        cmdSet.motorCmd[leg_i*3+2].mode = 10;
        cmdSet.motorCmd[leg_i*3+2].dq = 0;
        cmdSet.motorCmd[leg_i*3+2].Kp = 300;
        cmdSet.motorCmd[leg_i*3+2].Kd = 15;
        cmdSet.motorCmd[leg_i*3+2].tau = 0;
    }
}

void ctrl_server::squiggle_ctrl_reset()
{
    squiggle_track_start = false;
}