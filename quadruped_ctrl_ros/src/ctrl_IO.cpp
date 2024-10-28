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
 * \file ctrl_IO.cpp
 * \date 25/10/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include "quadruped_ctrl_ros/ctrl_server.h"

void ctrl_server::register_callbacks()
{
    servo_subscribers_ptr = std::make_unique<ros::Subscriber[]>(DoF);
    servo_subscribers_ptr[0] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/FR_hip_controller/state", 1, &ctrl_server::FRhipCallback, this);
    servo_subscribers_ptr[1] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/FR_thigh_controller/state", 1, &ctrl_server::FRthighCallback, this);
    servo_subscribers_ptr[2] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/FR_calf_controller/state", 1, &ctrl_server::FRcalfCallback, this);
    servo_subscribers_ptr[3] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/FL_hip_controller/state", 1, &ctrl_server::FLhipCallback, this);
    servo_subscribers_ptr[4] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/FL_thigh_controller/state", 1, &ctrl_server::FLthighCallback, this);
    servo_subscribers_ptr[5] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/FL_calf_controller/state", 1, &ctrl_server::FLcalfCallback, this);
    servo_subscribers_ptr[6] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/RR_hip_controller/state", 1, &ctrl_server::RRhipCallback, this);
    servo_subscribers_ptr[7] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/RR_thigh_controller/state", 1, &ctrl_server::RRthighCallback, this);
    servo_subscribers_ptr[8] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/RR_calf_controller/state", 1, &ctrl_server::RRcalfCallback, this);
    servo_subscribers_ptr[9] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/RL_hip_controller/state", 1, &ctrl_server::RLhipCallback, this);
    servo_subscribers_ptr[10] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/RL_thigh_controller/state", 1, &ctrl_server::RLthighCallback, this);
    servo_subscribers_ptr[11] = nh.subscribe<unitree_legged_msgs::MotorState>("/" + ROBOT_NAME + "_gazebo/RL_calf_controller/state", 1, &ctrl_server::RLcalfCallback, this);
    
    fsm_sub = nh.subscribe<std_msgs::String>("/FSM", 1, &ctrl_server::fsmCallback, this);

    return;
}

void ctrl_server::FRhipCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    sys_started = true;
    stateNow.motorState[0].mode = msg->mode;
    stateNow.motorState[0].q = msg->q;
    stateNow.motorState[0].dq = msg->dq;
    stateNow.motorState[0].tauEst = msg->tauEst;
}

void ctrl_server::FRthighCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    stateNow.motorState[1].mode = msg->mode;
    stateNow.motorState[1].q = msg->q;
    stateNow.motorState[1].dq = msg->dq;
    stateNow.motorState[1].tauEst = msg->tauEst;   
}

void ctrl_server::FRcalfCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    stateNow.motorState[2].mode = msg->mode;
    stateNow.motorState[2].q = msg->q;
    stateNow.motorState[2].dq = msg->dq;
    stateNow.motorState[2].tauEst = msg->tauEst;
}

void ctrl_server::FLhipCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    stateNow.motorState[3].mode = msg->mode;
    stateNow.motorState[3].q = msg->q;
    stateNow.motorState[3].dq = msg->dq;
    stateNow.motorState[3].tauEst = msg->tauEst;
}

void ctrl_server::FLthighCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    stateNow.motorState[4].mode = msg->mode;
    stateNow.motorState[4].q = msg->q;
    stateNow.motorState[4].dq = msg->dq;
    stateNow.motorState[4].tauEst = msg->tauEst;
}

void ctrl_server::FLcalfCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    stateNow.motorState[5].mode = msg->mode;
    stateNow.motorState[5].q = msg->q;
    stateNow.motorState[5].dq = msg->dq;
    stateNow.motorState[5].tauEst = msg->tauEst;
}

void ctrl_server::RRhipCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    stateNow.motorState[6].mode = msg->mode;
    stateNow.motorState[6].q = msg->q;
    stateNow.motorState[6].dq = msg->dq;
    stateNow.motorState[6].tauEst = msg->tauEst;
}

void ctrl_server::RRthighCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    stateNow.motorState[7].mode = msg->mode;
    stateNow.motorState[7].q = msg->q;
    stateNow.motorState[7].dq = msg->dq;
    stateNow.motorState[7].tauEst = msg->tauEst;
}

void ctrl_server::RRcalfCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    stateNow.motorState[8].mode = msg->mode;
    stateNow.motorState[8].q = msg->q;
    stateNow.motorState[8].dq = msg->dq;
    stateNow.motorState[8].tauEst = msg->tauEst;
}

void ctrl_server::RLhipCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    stateNow.motorState[9].mode = msg->mode;
    stateNow.motorState[9].q = msg->q;
    stateNow.motorState[9].dq = msg->dq;
    stateNow.motorState[9].tauEst = msg->tauEst;
}

void ctrl_server::RLthighCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    stateNow.motorState[10].mode = msg->mode;
    stateNow.motorState[10].q = msg->q;
    stateNow.motorState[10].dq = msg->dq;
    stateNow.motorState[10].tauEst = msg->tauEst;
}

void ctrl_server::RLcalfCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    // std::cout<<"hu"<<std::endl;
    stateNow.motorState[11].mode = msg->mode;
    stateNow.motorState[11].q = msg->q;
    stateNow.motorState[11].dq = msg->dq;
    stateNow.motorState[11].tauEst = msg->tauEst;
     
    
    push_back_state_vector();
}

void ctrl_server::push_back_state_vector()
{
    for (int i = 0; i < DoF; i++)
    {
        q_state[i] = stateNow.motorState[i].q;
        dq_state[i] = stateNow.motorState[i].dq;
        tau_state[i] = stateNow.motorState[i].tauEst;
    }
}

void ctrl_server::fsmCallback(
    const std_msgs::String::ConstPtr& msg
)
{
    FSM_STATE = msg->data;
    // std::cout<<FSM_STATE<<"1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111"<<std::endl;
}

void ctrl_server::register_publishers()
{
    servo_publishers_ptr = std::make_unique<ros::Publisher[]>(DoF);
    servo_publishers_ptr[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/FR_hip_controller/command", 1);
    servo_publishers_ptr[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/FR_thigh_controller/command", 1);
    servo_publishers_ptr[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/FR_calf_controller/command", 1);
    servo_publishers_ptr[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/FL_hip_controller/command", 1);
    servo_publishers_ptr[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/FL_thigh_controller/command", 1);
    servo_publishers_ptr[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/FL_calf_controller/command", 1);
    servo_publishers_ptr[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/RR_hip_controller/command", 1);
    servo_publishers_ptr[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/RR_thigh_controller/command", 1);
    servo_publishers_ptr[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/RR_calf_controller/command", 1);
    servo_publishers_ptr[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/RL_hip_controller/command", 1);
    servo_publishers_ptr[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/RL_thigh_controller/command", 1);
    servo_publishers_ptr[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + ROBOT_NAME + "_gazebo/RL_calf_controller/command", 1);

    mainspin_timer = nh.createTimer(
        ros::Duration(1.0/ctrl_freq),
        &ctrl_server::mainspinCallback,
        this
    );

    return;
}

void ctrl_server::publish_servos(unitree_legged_msgs::LowCmd& _cmdSet)
{
    try {
        if (FSM_STATE == PASSIVE) {
            if (CTRL_MODE != DAMP_MODE) {
                throw std::runtime_error("CTRL_MODE in PASSIVE state");
            }
        }
    } catch (const std::runtime_error& e) {
        ROS_ERROR("@ PASSIVE, CTRL_MODE SHOULD BE %s", DAMP_MODE);
        ROS_ERROR("CURRENT MODE IS %s", CTRL_MODE.c_str());
        ROS_ERROR("Error: %s", e.what());

        ros::shutdown(); 
        std::exit(EXIT_FAILURE);  
    }

    
    for(int i = 0; i < DoF; i++)
    {
        // std::cout<<"hi"<<std::endl;
        cmdNow.motorCmd[i].mode = _cmdSet.motorCmd[i].mode;
        cmdNow.motorCmd[i].q = _cmdSet.motorCmd[i].q;
        cmdNow.motorCmd[i].dq = _cmdSet.motorCmd[i].dq;
        cmdNow.motorCmd[i].tau = _cmdSet.motorCmd[i].tau;
        cmdNow.motorCmd[i].Kd = _cmdSet.motorCmd[i].Kd;
        cmdNow.motorCmd[i].Kp = _cmdSet.motorCmd[i].Kp;

        servo_publishers_ptr[i].publish(cmdNow.motorCmd[i]);
    }
}

