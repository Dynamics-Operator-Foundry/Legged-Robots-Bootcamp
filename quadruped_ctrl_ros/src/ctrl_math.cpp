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
 * \file ctrl_math.cpp
 * \date 25/10/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include "quadruped_ctrl_ros/ctrl_server.h"

Eigen::Vector3d ctrl_server::forward_kinematics(
    int leg_i, 
    Eigen::VectorXd& q_state
)
{
    double l0, l1, l2;
    double theta0, theta1, theta2;

    switch (leg_i)
    {
    case 0:
        l0 = -l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        theta0 = q_state[0];
        theta1 = q_state[1];
        theta2 = q_state[2];
        break;

    case 1:
        l0 = l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        theta0 = q_state[3];
        theta1 = q_state[4];
        theta2 = q_state[5];
        break;

    case 2:
        l0 = -l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        theta0 = q_state[6];
        theta1 = q_state[7];
        theta2 = q_state[8];
        break;

    case 3:
        l0 = l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        theta0 = q_state[9];
        theta1 = q_state[10];
        theta2 = q_state[11];
        break;
    
    default:
        ROS_ERROR("WRONG FOOT IN FORWARD KINEMATICS!");
        break;
    }
    
    return Eigen::Vector3d(
        l2 * sin(theta1 + theta2) + l1 * sin(theta1),
        -l2 * sin(theta0) * cos(theta1 + theta2) + l0 * cos(theta0) - l1 * cos(theta1) * sin(theta0),
        l2 * cos(theta0) * cos(theta1 + theta2) + l0 * sin(theta0) + l1 * cos(theta0) * cos(theta1)
    );
}

Eigen::Vector3d ctrl_server::inverse_kinematics(
    int leg_i,
    Eigen::VectorXd& q_state,
    Eigen::Vector3d& r_E
)
{
    double xp = r_E(0);
    double yp = r_E(1);
    double zp = r_E(2);

    // get theta0
    double L = sqrt(pow(yp,2) + pow(zp,2) - pow(l_abad,2));
    double theta0 = atan2(
        zp * l_abad + yp * L, yp * l_abad - zp * L
    );
    if (leg_i == 0 || leg_i == 2)
        theta0 = (-1) * theta0;
    

    // get theta2
    double ap = sqrt(pow(xp,2) + pow(zp,2) + pow(yp,2) - pow(l_abad,2));
    double theta2 = -M_PI + acos(
        (pow(l_hip,2) + pow(l_knee,2) - pow(ap,2)) /
        (2 * l_hip * l_knee)
    );

    // get theta1
    double a1 = yp * sin(theta0) - zp * cos(theta0);
    double a2 = xp;
    double m1 = l_knee * sin(theta2);
    double m2 = l_knee * cos(theta2) + l_hip;
    double theta1 = atan2(
        a1 * m1 + a2 * m2,
        a2 * m1 - a1 * m2
    );

    // body：-49~49°
    // thigh：-39~257°
    // shank：-161~-51°

    if (
        (theta0 < -49.0/180 * M_PI) || 
        (theta0 > 49.0/180 * M_PI) ||
        (theta1 < -39.0/180 * M_PI) ||
        (theta1 > 257.0/180 * M_PI) ||
        (theta2 > -161.0/180 * M_PI) ||
        (theta2 > -51.0/180 * M_PI) 
    )
        ROS_WARN("ANGLE HITS MECHANICAL LIMITS!");

    return Eigen::Vector3d(theta0, theta1, theta2);
}

Eigen::Matrix3d ctrl_server::get_Jacobian(
    int leg_i, 
    Eigen::VectorXd& q_state
)
{
    double l0, l1, l2;
    double theta0, theta1, theta2;

    switch (leg_i)
    {
    case 0:
        l0 = -l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        theta0 = q_state[0];
        theta1 = q_state[1];
        theta2 = q_state[2];
        break;

    case 1:
        l0 = l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        theta0 = q_state[3];
        theta1 = q_state[4];
        theta2 = q_state[5];
        break;

    case 2:
        l0 = -l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        theta0 = q_state[6];
        theta1 = q_state[7];
        theta2 = q_state[8];
        break;

    case 3:
        l0 = l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        theta0 = q_state[9];
        theta1 = q_state[10];
        theta2 = q_state[11];
        break;
    
    default:
        ROS_ERROR("WRONG FOOT IN FORWARD KINEMATICS!");
        break;
    }

    Eigen::Matrix3d J;

    J << 
        0, 
        l1*cos(theta1) + l2*cos(theta1 + theta2), 
        l2*cos(theta1 + theta2), 
        -l0*sin(theta0) - l1*cos(theta0)*cos(theta1) - l2*cos(theta0)*cos(theta1 + theta2), 
        (l1*sin(theta1) + l2*sin(theta1 + theta2))*sin(theta0), 
        l2*sin(theta0)*sin(theta1 + theta2), 
        l0*cos(theta0) - l1*sin(theta0)*cos(theta1) - l2*sin(theta0)*cos(theta1 + theta2), 
        -(l1*sin(theta1) + l2*sin(theta1 + theta2))*cos(theta0), 
        -l2*sin(theta1 + theta2)*cos(theta0);

    return J;
}