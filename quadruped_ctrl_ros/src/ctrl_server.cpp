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
    nh.getParam("/robot_name", robot_name);
    register_callbacks();
    register_publishers();
    
    return;
}

ctrl_server::~ctrl_server()
{

}



void ctrl_server::mainspinCallback(const ros::TimerEvent &e)
{
    fsm_manager();

    return;
}

void ctrl_server::fsm_manager()
{
    if(FSM_STATE == PASSIVE)
    {
        if(print_or_not)
        {
            ROS_YELLOW_STREAM(PASSIVE);
            print_or_not = false;
        }
        passive_ctrl();
    }
    else if(FSM_STATE == TIP)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(TIP);
            print_or_not = false;
        }
        tip_ctrl();
    }
    else if(FSM_STATE == STAND)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(STAND);
            print_or_not = false;
        }
        stand_ctrl();
    }
    else if(FSM_STATE == SWING_LEG)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(SWING_LEG);
            print_or_not = false;
        }
        swing_leg_ctrl();
    }
    else if(FSM_STATE == SQUIGGLE)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(SQUIGGLE);
            print_or_not = false;
        }
        squiggle_ctrl();
    }
    else if(FSM_STATE == CRAWL)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(CRAWL);
            print_or_not = false;
        }
        crawl_ctrl();
    }
    else if(FSM_STATE == TROT)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(TROT);
            print_or_not = false;
        }
        trot_ctrl();
    }
    else
    {
        ROS_ERROR("Please Check System...");
    }
}


