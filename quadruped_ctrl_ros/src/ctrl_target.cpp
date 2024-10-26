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
 * \file ctrl_tip.cpp
 * \date 25/10/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include "quadruped_ctrl_ros/ctrl_server.h"

void ctrl_server::target_ctrl()
{
    if(!target_track_start)
    {
        set_target_ctrl();
        set_gain();
        target_track_start = true;
    }

    target_percent = target_percent + 1.0 / ctrl_freq;
    target_percent = target_percent > 1 ? 1 : target_percent;
    for (int i = 0; i < DoF; i++)
    {
        cmdSet.motorCmd[i].q = (1.0 - target_percent) * q_start[i] + target_percent * q_target[i];
    }
}

void ctrl_server::set_target_ctrl()
{
    
    q_target << 
        0.0, 0.67, -1.3,
        0.0, 0.67, -1.3,
        0.0, 0.67, -1.3,
        0.0, 0.67, -1.3;
    
        
    for(int i = 0; i < DoF; i++)
    {
        q_start[i] = stateNow.motorState[i].q;
    }
}

void ctrl_server::set_gain()
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


void ctrl_server::target_reset()
{
    q_target.setZero();
    target_track_start = false;
    target_percent = 0;
}

