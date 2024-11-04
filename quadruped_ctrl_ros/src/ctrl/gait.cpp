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
 * \file ctrl_gait.cpp
 * \date 25/10/20244
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include "quadruped_ctrl_ros/ctrl_server.h"


void ctrl_server::set_gait_params()
{
    P_gait = 0.45;
    r_gait = 0.5;
    b_gait << 0, 0.5, 0.5, 0;
    phase_gait.setConstant(0.5); 
    // all legs are at phase = 0.5, when everyone is on ground
    contact_gait << 1, 1, 1, 1;

    gait_vlim_B << 0.4, 0.3, 0.5;

    Kpb_p_trot = Eigen::Vector3d(20,20,100).asDiagonal();
    Kdb_p_trot = Eigen::Vector3d(20,20,20).asDiagonal();
    Kpb_w_trot = 400;
    Kdb_w_trot = Eigen::Vector3d(50,50,50).asDiagonal();
    Kps_trot = Eigen::Vector3d(400,400,400).asDiagonal();
    Kds_trot = Eigen::Vector3d(10,10,10).asDiagonal();


    for (int leg_i = 0; leg_i < leg_no; leg_i ++)
    {
        feet_posi_start_I.emplace_back(pose_SE3_robot_base.rotationMatrix() * get_foot_p_B(leg_i));
        feet_posi_I.emplace_back(Eigen::Vector3d::Zero());
        feet_velo_I.emplace_back(Eigen::Vector3d::Zero());
    }
}

void ctrl_server::calc_contact_phase()
{
    double t_since_start = ros::Time::now().toSec() - t_start;
    Eigen::Vector4d normalized_time;

    for (int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        normalized_time(leg_i) = fmod(t_since_start + P_gait - P_gait * b_gait(leg_i), P_gait) / P_gait;

        if (normalized_time(leg_i) < r_gait)
        {
            contact_gait(leg_i) = 1;
            phase_gait(leg_i) = normalized_time(leg_i) / r_gait;
        }
        else
        {
            contact_gait(leg_i) = 0;
            phase_gait(leg_i) = (normalized_time(leg_i) - r_gait) / (1 - r_gait);
        }
    }
}

void ctrl_server::set_gait()
{
    for(int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        if(contact_gait(leg_i) == 1)
        {
            if(phase_gait(leg_i) < 0.5)
            {
                feet_posi_start_I[leg_i] =
                    pose_SE3_robot_base.rotationMatrix() * get_foot_p_B(leg_i);
            }
            feet_posi_I[leg_i] = feet_posi_start_I[leg_i];
            feet_velo_I[leg_i] = Eigen::Vector3d::Zero();
        }
        else
        {
            // _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));

            feet_posi_I[leg_i] = get_foot_touchdown_posi(leg_i, trot_vel_I.head(2), trot_vel_I(2), phase_gait(leg_i));
            feet_velo_I[leg_i] = get_foot_touchdown_velo(leg_i, trot_vel_I.head(2), trot_vel_I(2), phase_gait(leg_i));
        }
    }
    // _pastP = feetPos;
    // _phasePast = *_phase;
}

double ctrl_server::cycloid_lateral(double start, double end, double phase)
{
    return start + (end - start) / (2 * M_PI) * (2 * M_PI * phase - sin(2 * M_PI * phase));
}

double ctrl_server::cycloid_vertical(double start, double h, double phase)
{
    return start + h / 2 * (1 - cos(2 * M_PI * phase));
}

double ctrl_server::cycloid_dlateral(double start, double end, double phase)
{
    double T  = (1 - r_gait) * P_gait; // T of swing
    return (end - start) / T * (1 - cos(2 * M_PI * phase));
}

double ctrl_server::cycloid_dvertical(double start, double h, double phase)
{
    double T  = (1 - r_gait) * P_gait; // T of swing
    return M_PI * h / T * sin(2 * M_PI * phase);
}

Eigen::Vector3d ctrl_server::get_foot_touchdown_posi(int leg_i, Eigen::Vector2d velo_I, double dw, double phase_i)
{

}

Eigen::Vector3d ctrl_server::get_foot_touchdown_velo(int leg_i, Eigen::Vector2d velo_I, double dw, double phase_i)
{

}

void ctrl_server::reset_gait()
{

}