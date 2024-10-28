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

    Eigen::VectorXd q_state;
    Eigen::VectorXd dq_state;
    Eigen::VectorXd tau_state;

    std::string ROBOT_NAME;
    int DoF = 12;
    int leg_no = 4;
    unitree_legged_msgs::LowCmd cmdNow;
    unitree_legged_msgs::LowCmd cmdSet;
    unitree_legged_msgs::LowState stateNow;

    double l_abad = 0.08;
    double l_hip = 0.213;
    double l_knee = 0.213;

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

    void push_back_state_vector();

    void fsmCallback(const std_msgs::String::ConstPtr& msg);

    // publisher
    void register_publishers();
    void publish_servos(unitree_legged_msgs::LowCmd& _cmdSet);

    // timer
    ros::Timer mainspin_timer;
    void mainspinCallback(const ros::TimerEvent &e);

// main functions
    void fsm_reset();

    void passive_ctrl();
    
    void target_ctrl();
    void set_target_ctrl();
    void set_target_ctrl_gain();
    void target_ctrl_reset();
    Eigen::VectorXd q_start;
    Eigen::VectorXd q_target;
    bool target_track_start = false;
    double target_percent = 0;

    void swing_leg_ctrl();
    void set_swing_leg_ctrl();
    void set_swing_leg_ctrl_gain();
    void swing_leg_ctrl_reset();
    Eigen::Vector3d p_swing_target, q_swing_target;
    Eigen::Matrix3d Kp, Kd;
    bool swing_track_start = false;
    
    void squiggle_ctrl();
    void crawl_ctrl();
    void trot_ctrl();

    std::string pre_FSM_STATE;
    void check_fsm_change();


// config
    void config();

// math
    Eigen::VectorXd get_leg_q(
        int leg_i,
        Eigen::VectorXd& q_state
    );
    Eigen::VectorXd get_leg_dq(
        int leg_i,
        Eigen::VectorXd& q_state
    );

    Eigen::Vector3d forward_kinematics(
        int leg_i
    );
    Eigen::Vector3d inverse_kinematics(
        int leg_i, 
        Eigen::Vector3d& r_E
    );

    Eigen::Matrix3d get_Jacobian(
        int leg_i
    );

    Eigen::Vector3d get_linear_velocity(
        int leg_i
    );


    


public:
    ctrl_server(ros::NodeHandle& _nh);
    ~ctrl_server();

};

#endif