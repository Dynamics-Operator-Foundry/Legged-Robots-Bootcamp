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
    if(!squiggle_track_start)
    {
        set_squiggle_ctrl();
        set_squiggle_ctrl_gain();
        squiggle_track_start = true;
    }

    // std::cout<<squiggle_fsm<<std::endl;
    // std::cout<<ctrl_param<<std::endl<<std::endl;;

    if (squiggle_fsm == "ROLL") 
        roll_base = roll_mag * sin(ctrl_param);
    else if (squiggle_fsm == "PITCH") 
        pitch_base = pitch_mag * sin(ctrl_param);
    else if (squiggle_fsm == "YAW") 
        yaw_base = yaw_mag * sin(ctrl_param);
    else 
        height_base = height_mag * sin(ctrl_param);

    ctrl_param = ctrl_param + 2 * M_PI / 6.0 * 1 / ctrl_freq;

    if (ctrl_param > 2 * M_PI)
    {
        ctrl_param = 0;
        roll_base = pitch_base = yaw_base = height_base = 0;

        if (squiggle_fsm == "ROLL") 
            squiggle_fsm = "PITCH";
        else if (squiggle_fsm == "PITCH") 
            squiggle_fsm = "YAW";
        else if (squiggle_fsm == "YAW") 
            squiggle_fsm = "HEIGHT";
        else 
            squiggle_fsm = "ROLL";

        // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" << squiggle_fsm << std::endl;
    }

    Eigen::Matrix3d _rot = rpy2q(
        Eigen::Vector3d(
            roll_base, pitch_base, yaw_base
            )).normalized().toRotationMatrix();
    
    base_pose_S = Sophus::SE3d(
        _rot, 
        Eigen::Vector3d(
            base_S.x(),
            base_S.y(),
            base_S.z() + height_base
        )
    ); 
    // T_B_2_S
    // T_S_2_B = base_pose_S.inv()

    Sophus::SE3d T_S_2_B = base_pose_S.inverse();
    // std::cout<<"================================"<<std::endl;
    for (int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        p_foots_B.col(leg_i) = T_S_2_B.rotationMatrix() * p_foots_S.col(leg_i) + T_S_2_B.translation();

        // std::cout<<p_foots_B.col(leg_i)<<std::endl<<std::endl;

        q_foots.col(leg_i) = get_q_from_B(leg_i, p_foots_B.col(leg_i));

        // ros::shutdown()
        cmdSet.motorCmd[leg_i*3+0].q = q_foots.col(leg_i)[0];
        cmdSet.motorCmd[leg_i*3+1].q = q_foots.col(leg_i)[1];
        cmdSet.motorCmd[leg_i*3+2].q = q_foots.col(leg_i)[2];
    }
    // std::cout<<"================================"<<std::endl;
}

void ctrl_server::set_squiggle_ctrl()
{
    base_S = -get_foot_p_B(0);
    base_pose_S = Sophus::SE3d(
        Eigen::Matrix3d::Identity(),
        base_S
    );

    p_foots_S.col(0).setZero();
    for (int i = 1; i < leg_no; i++)
        p_foots_S.col(i) = base_S + get_foot_p_B(i);

    roll_mag = 20 * M_PI / 180;
    pitch_mag = 15 * M_PI / 180;
    yaw_mag = 20 * M_PI / 180;
    height_mag = 0.04;

    squiggle_fsm = "ROLL";
    roll_base = 0;
    pitch_base = 0;
    yaw_base = 0;
    height_base = 0;

    ctrl_param = 0;
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