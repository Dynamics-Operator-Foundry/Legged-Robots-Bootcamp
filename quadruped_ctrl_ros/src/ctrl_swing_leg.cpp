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
 * \file ctrl_swing_leg.cpp
 * \date 25/10/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include "quadruped_ctrl_ros/ctrl_server.h"

void ctrl_server::swing_leg_ctrl()
{
    if(!swing_track_start)
    {
        set_swing_leg_ctrl();
        set_swing_leg_ctrl_gain();
        swing_track_start = true;
    }

    Eigen::Vector3d p_swing_now = forward_kinematics(0);
    Eigen::Vector3d v_swing_now = get_linear_velocity(0);
    Eigen::Vector3d accl = Kp * (p_swing_target - p_swing_now) + Kd * (Eigen::Vector3d::Zero() - v_swing_now);

    Eigen::Matrix3d J = get_Jacobian(0);

    Eigen::Vector3d tau = J.transpose() * accl;
    int which_leg = 0;

    for (int i = which_leg * 3; i < which_leg * 3 + 3; i++)
    {
        cmdSet.motorCmd[i].q = q_swing_target(i - which_leg * 3);
        cmdSet.motorCmd[i].tau = tau(i - which_leg * 3);
    }
}

void ctrl_server::set_swing_leg_ctrl()
{
    p_swing_target <<
        -0.0465797849175114, 0.139635096607956, -0.286162393820798;

    q_swing_target = inverse_kinematics(0, p_swing_target);
    
    Kp = Eigen::Vector3d(20, 20, 50).asDiagonal();
    Kd = Eigen::Vector3d( 5,  5, 20).asDiagonal();
}

void ctrl_server::set_swing_leg_ctrl_gain()
{
    for(int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        cmdSet.motorCmd[leg_i*3+0].mode = 10;
        cmdSet.motorCmd[leg_i*3+0].dq = 0;
        cmdSet.motorCmd[leg_i*3+0].Kp = 3;
        cmdSet.motorCmd[leg_i*3+0].Kd = 2;
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

void ctrl_server::swing_leg_ctrl_reset()
{
    p_swing_target.setZero();
    swing_track_start = false;
}