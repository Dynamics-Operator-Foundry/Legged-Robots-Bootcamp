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
    servo_subscribers_ptr[0] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/FR_hip_controller/state", 1, &ctrl_server::FRhipCallback, this);
    servo_subscribers_ptr[1] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/FR_thigh_controller/state", 1, &ctrl_server::FRthighCallback, this);
    servo_subscribers_ptr[2] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/FR_calf_controller/state", 1, &ctrl_server::FRcalfCallback, this);
    servo_subscribers_ptr[3] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/FL_hip_controller/state", 1, &ctrl_server::FLhipCallback, this);
    servo_subscribers_ptr[4] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/FL_thigh_controller/state", 1, &ctrl_server::FLthighCallback, this);
    servo_subscribers_ptr[5] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/FL_calf_controller/state", 1, &ctrl_server::FLcalfCallback, this);
    servo_subscribers_ptr[6] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/RR_hip_controller/state", 1, &ctrl_server::RRhipCallback, this);
    servo_subscribers_ptr[7] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/RR_thigh_controller/state", 1, &ctrl_server::RRthighCallback, this);
    servo_subscribers_ptr[8] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/RR_calf_controller/state", 1, &ctrl_server::RRcalfCallback, this);
    servo_subscribers_ptr[9] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/RL_hip_controller/state", 1, &ctrl_server::RLhipCallback, this);
    servo_subscribers_ptr[10] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/RL_thigh_controller/state", 1, &ctrl_server::RLthighCallback, this);
    servo_subscribers_ptr[11] = nh.subscribe("/" + ROBOT_NAME + "_gazebo/RL_calf_controller/state", 1, &ctrl_server::RLcalfCallback, this);

    return;
}

void ctrl_server::FRhipCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    sys_started = true;
    lowState.motorState[0].mode = msg->mode;
    lowState.motorState[0].q = msg->q;
    lowState.motorState[0].dq = msg->dq;
    lowState.motorState[0].tauEst = msg->tauEst;
}

void ctrl_server::FRthighCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[1].mode = msg->mode;
    lowState.motorState[1].q = msg->q;
    lowState.motorState[1].dq = msg->dq;
    lowState.motorState[1].tauEst = msg->tauEst;   
}

void ctrl_server::FRcalfCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[2].mode = msg->mode;
    lowState.motorState[2].q = msg->q;
    lowState.motorState[2].dq = msg->dq;
    lowState.motorState[2].tauEst = msg->tauEst;
}

void ctrl_server::FLhipCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[3].mode = msg->mode;
    lowState.motorState[3].q = msg->q;
    lowState.motorState[3].dq = msg->dq;
    lowState.motorState[3].tauEst = msg->tauEst;
}

void ctrl_server::FLthighCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[4].mode = msg->mode;
    lowState.motorState[4].q = msg->q;
    lowState.motorState[4].dq = msg->dq;
    lowState.motorState[4].tauEst = msg->tauEst;
}

void ctrl_server::FLcalfCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[5].mode = msg->mode;
    lowState.motorState[5].q = msg->q;
    lowState.motorState[5].dq = msg->dq;
    lowState.motorState[5].tauEst = msg->tauEst;
}

void ctrl_server::RRhipCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[6].mode = msg->mode;
    lowState.motorState[6].q = msg->q;
    lowState.motorState[6].dq = msg->dq;
    lowState.motorState[6].tauEst = msg->tauEst;
}

void ctrl_server::RRthighCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[7].mode = msg->mode;
    lowState.motorState[7].q = msg->q;
    lowState.motorState[7].dq = msg->dq;
    lowState.motorState[7].tauEst = msg->tauEst;
}

void ctrl_server::RRcalfCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[8].mode = msg->mode;
    lowState.motorState[8].q = msg->q;
    lowState.motorState[8].dq = msg->dq;
    lowState.motorState[8].tauEst = msg->tauEst;
}

void ctrl_server::RLhipCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[9].mode = msg->mode;
    lowState.motorState[9].q = msg->q;
    lowState.motorState[9].dq = msg->dq;
    lowState.motorState[9].tauEst = msg->tauEst;
}

void ctrl_server::RLthighCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[10].mode = msg->mode;
    lowState.motorState[10].q = msg->q;
    lowState.motorState[10].dq = msg->dq;
    lowState.motorState[10].tauEst = msg->tauEst;
}

void ctrl_server::RLcalfCallback(
    const unitree_legged_msgs::MotorState::ConstPtr& msg
)
{
    lowState.motorState[11].mode = msg->mode;
    lowState.motorState[11].q = msg->q;
    lowState.motorState[11].dq = msg->dq;
    lowState.motorState[11].tauEst = msg->tauEst;
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
        ros::Duration(1.0/400.0),
        &ctrl_server::mainspinCallback,
        this
    );

    keyboard_timer = nh.createTimer(
        ros::Duration(1.0/10.0),
        &ctrl_server::keyboardCallback,
        this
    );

    return;
}

void ctrl_server::publish_servos(const unitree_legged_msgs::LowCmd& cmd_now)
{
    for(int i = 0; i < DoF; i++)
    {
        lowCmd.motorCmd[i].mode = cmd_now.motorCmd[i].mode;
        lowCmd.motorCmd[i].q = cmd_now.motorCmd[i].q;
        lowCmd.motorCmd[i].dq = cmd_now.motorCmd[i].dq;
        lowCmd.motorCmd[i].tau = cmd_now.motorCmd[i].tau;
        lowCmd.motorCmd[i].Kd = cmd_now.motorCmd[i].Kd;
        lowCmd.motorCmd[i].Kp = cmd_now.motorCmd[i].Kp;
    }
    for(int i = 0; i < DoF; i++)
    {   
        servo_publishers_ptr[i].publish(cmd_now.motorCmd[i]);
    }
}

