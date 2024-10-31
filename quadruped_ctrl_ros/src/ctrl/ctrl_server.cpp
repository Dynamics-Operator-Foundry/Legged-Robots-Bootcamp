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

ctrl_server::ctrl_server(ros::NodeHandle& _nh) :
nh(_nh)
{
    using namespace std;

    config();

    ros::AsyncSpinner spinner(0);
    spinner.start();

    register_callbacks();
    register_publishers();

    ros::waitForShutdown();
    
    return;
}

ctrl_server::~ctrl_server()
{

}

void ctrl_server::config()
{
    nh.getParam("/robot_name", ROBOT_NAME);
    nh.getParam("CTRL_MODE", CTRL_MODE);

    q_state.resize(DoF);
    dq_state.resize(DoF);
    tau_state.resize(DoF);

    q_start.resize(DoF);
    q_target.resize(DoF);

    r_base2FRhip <<  0.1881, -0.04675, 0;
    r_base2FLhip <<  0.1881,  0.04675, 0;
    r_base2RRhip << -0.1881, -0.04675, 0;
    r_base2RLhip << -0.1881,  0.04675, 0;


    pre_FSM_STATE = FSM_STATE;
}


void ctrl_server::mainspinCallback(const ros::TimerEvent &e)
{
    using namespace std;

    fsm_manager();
    publish_servos(cmdSet);
    ////

    return;
}

void ctrl_server::fsm_manager()
{
    check_fsm_change();

    if(FSM_STATE == PASSIVE)
        passive_ctrl();
    else if(FSM_STATE == STAND)
        target_ctrl();
    else if(FSM_STATE == SWING_LEG)
        swing_leg_ctrl();
    else if(FSM_STATE == SQUIGGLE)
        squiggle_ctrl();
    else if(FSM_STATE == BALANCE)
        balance_ctrl();
    else if(FSM_STATE == CRAWL)
        crawl_ctrl();
    else if(FSM_STATE == TROT)
        trot_ctrl();
    else    
        ROS_ERROR("Please Check System...");
}

void ctrl_server::check_fsm_change()
{
    if (FSM_STATE == pre_FSM_STATE)
        return;

    pre_FSM_STATE = FSM_STATE;
    fsm_reset();
}

void ctrl_server::fsm_reset()
{
    target_ctrl_reset();
    swing_leg_ctrl_reset();
    squiggle_ctrl_reset();
}


