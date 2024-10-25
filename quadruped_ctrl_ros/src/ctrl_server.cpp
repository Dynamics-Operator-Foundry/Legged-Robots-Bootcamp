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
 * \file ctrl_server.cpp
 * \date 25/10/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include "quadruped_ctrl_ros/ctrl_server.h"

ctrl_server::ctrl_server(ros::NodeHandle& _nh) : nh(_nh)
{
    config();
    register_callbacks();
    register_publishers();
    
    return;
}

ctrl_server::~ctrl_server()
{

}

void ctrl_server::config()
{
    nh.getParam("/ROBOT_NAME", ROBOT_NAME);
    nh.getParam("CTRL_MODE", CTRL_MODE);
    nh.getParam("FSM_STATE", FSM_STATE);

    geometry_msgs::PoseStamped lala;

    keyboardInitTerminal();

}


void ctrl_server::mainspinCallback(const ros::TimerEvent &e)
{
    fsm_manager();

    return;
}

void ctrl_server::fsm_manager()
{
    if(FSM_STATE == PASSIVE)
        passive_ctrl();
    else if(FSM_STATE == TIP)
        tip_ctrl();
    else if(FSM_STATE == STAND)
        stand_ctrl();
    else if(FSM_STATE == SWING_LEG)
        swing_leg_ctrl();
    else if(FSM_STATE == SQUIGGLE)
        squiggle_ctrl();
    else if(FSM_STATE == CRAWL)
        crawl_ctrl();
    else if(FSM_STATE == TROT)
        trot_ctrl();
    else    
        ROS_ERROR("Please Check System...");

    // publisher here
}


