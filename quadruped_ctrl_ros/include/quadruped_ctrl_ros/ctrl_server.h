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

#ifndef CTRL_SERVER_H
#define CTRL_SERVER_H

#include <ros_utilities/ros_utilities.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

// FSM_STATE
#define PASSIVE "PASSIVE" // 0
#define TIP "TIP" // 1
#define STAND "STAND" // 2
#define SWING_LEG "SWING_LEG" // 3
#define SQUIGGLE "SQUIGGLE" // 4
#define CRAWL "CRAWL" // 5
#define TROT "TROT" // 6

// CTRL_MODE
// tau = tau_ff + Kp * dq + Kd * dqot
#define POS_MODE "POS_MODE"
#define VEL_MODE "VEL_MODE"
#define DAMP_MODE "DAMP_MODE"
#define TORQ_MODE "TORQ_MODE"
#define MIX_MODE "MIX_MODE"


class ctrl_server : private ros_utilities
{
    
private:
    ros::NodeHandle nh;
    
// pointers
    std::unique_ptr<ros::Subscriber[]> servo_subscribers_ptr;
    std::unique_ptr<ros::Publisher[]> servo_publishers_ptr;

// quadruped parameters & objects
    bool sys_started = false;
    double ctrl_freq = 400.0;
    std::string ROBOT_NAME;
    int DoF = 12;
    int leg_no = 4;
    unitree_legged_msgs::LowCmd cmdNow;
    unitree_legged_msgs::LowCmd cmdSet;
    unitree_legged_msgs::LowState stateNow;

// quadruped fsm and control mode
    std::string CTRL_MODE = DAMP_MODE;
    std::string FSM_STATE = PASSIVE;
    bool print_or_not = true;

    void fsm_manager();

//ros related
    // subscriber
    ros::Subscriber fsm_sub;
    void register_callbacks();
    // callbacks
        // front right
    void FRhipCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
    void FRthighCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
    void FRcalfCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
        // front left
    void FLhipCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
    void FLthighCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
    void FLcalfCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
        // rear right
    void RRhipCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
    void RRthighCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
    void RRcalfCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
        // rear left
    void RLhipCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
    void RLthighCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);
    void RLcalfCallback(const unitree_legged_msgs::MotorState::ConstPtr& msg);

    void fsmCallback(const std_msgs::String::ConstPtr& msg);

    // publisher
    void register_publishers();
    void publish_servos(unitree_legged_msgs::LowCmd& _cmdSet);

    // timer
    ros::Timer mainspin_timer;
    void mainspinCallback(const ros::TimerEvent &e);

// main functions
    void passive_ctrl();

    void target_ctrl();
    void set_target_ctrl();
    void set_gain();
    void target_reset();
    Eigen::VectorXd q_start;
    Eigen::VectorXd q_target;
    bool target_track_start = false;
    double target_percent = 0;


    void swing_leg_ctrl();
    void squiggle_ctrl();
    void crawl_ctrl();
    void trot_ctrl();

    std::string pre_FSM_STATE;
    void check_fsm_change();


// config
    void config();

// keyboard cmd
    


public:
    ctrl_server(ros::NodeHandle& _nh);
    ~ctrl_server();

};

#endif