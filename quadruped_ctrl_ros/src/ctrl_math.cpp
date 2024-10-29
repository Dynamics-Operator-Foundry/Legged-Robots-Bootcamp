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

Eigen::Vector3d ctrl_server::get_leg_q(
    int leg_i,
    Eigen::VectorXd& q_state
)
{
    double theta0, theta1, theta2;

    switch (leg_i)
    {
    case 0:
        theta0 = q_state[0];
        theta1 = q_state[1];
        theta2 = q_state[2];
        break;

    case 1:
        theta0 = q_state[3];
        theta1 = q_state[4];
        theta2 = q_state[5];
        break;

    case 2:
        theta0 = q_state[6];
        theta1 = q_state[7];
        theta2 = q_state[8];
        break;

    case 3:
        theta0 = q_state[9];
        theta1 = q_state[10];
        theta2 = q_state[11];
        break;
    
    default:
        ROS_ERROR("WRONG FOOT ID IN FORWARD KINEMATICS!");
        break;
    }

    Eigen::Vector3d _q;
    _q <<
        theta0, theta1, theta2;

    return _q;
}

Eigen::Vector3d ctrl_server::get_leg_dq(
    int leg_i,
    Eigen::VectorXd& dq_state
)
{
    double dtheta0, dtheta1, dtheta2;

    switch (leg_i)
    {
    case 0:
        dtheta0 = dq_state[0];
        dtheta1 = dq_state[1];
        dtheta2 = dq_state[2];
        break;

    case 1:
        dtheta0 = dq_state[3];
        dtheta1 = dq_state[4];
        dtheta2 = dq_state[5];
        break;

    case 2:
        dtheta0 = dq_state[6];
        dtheta1 = dq_state[7];
        dtheta2 = dq_state[8];
        break;

    case 3:
        dtheta0 = dq_state[9];
        dtheta1 = dq_state[10];
        dtheta2 = dq_state[11];
        break;
    
    default:
        ROS_ERROR("WRONG FOOT ID IN FORWARD KINEMATICS!");
        break;
    }

    Eigen::Vector3d _dq;
    _dq <<
        dtheta0, dtheta1, dtheta2;

    return _dq;
}

Eigen::Vector3d ctrl_server::get_leg_kine_param(int leg_i)
{
    double l0, l1, l2;

    switch (leg_i)
    {
    case 0:
        l0 = -l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        break;

    case 1:
        l0 = l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        break;

    case 2:
        l0 = -l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        break;

    case 3:
        l0 = l_abad;
        l1 = -l_hip;
        l2 = -l_knee;
        break;
    
    default:
        ROS_ERROR("WRONG FOOT ID IN FORWARD KINEMATICS!");
        break;
    }

    Eigen::Vector3d param;
    param <<
        l0, l1, l2;

    return param;
}

Eigen::Vector3d ctrl_server::forward_kinematics(
    int leg_i
)
{
    Eigen::Vector3d leg_q = get_leg_q(leg_i, q_state);
    Eigen::Vector3d leg_param = get_leg_kine_param(leg_i);

    double theta0 = leg_q(0), theta1 = leg_q(1), theta2 = leg_q(2);
    double l0 = leg_param(0), l1 = leg_param(1), l2 = leg_param(2);

    return Eigen::Vector3d(
        l2 * sin(theta1 + theta2) + l1 * sin(theta1),
        -l2 * sin(theta0) * cos(theta1 + theta2) + l0 * cos(theta0) - l1 * cos(theta1) * sin(theta0),
        l2 * cos(theta0) * cos(theta1 + theta2) + l0 * sin(theta0) + l1 * cos(theta0) * cos(theta1)
    );
}

Eigen::Vector3d ctrl_server::inverse_kinematics(
    int leg_i,
    Eigen::Vector3d& r_E
)
{
    double xp = r_E(0);
    double yp = r_E(1);
    double zp = r_E(2);

    Eigen::Vector3d leg_param = get_leg_kine_param(leg_i);
    double l0 = leg_param(0), l1 = leg_param(1), l2 = leg_param(2);
    
    // get theta0
    double L = sqrt(pow(yp,2) + pow(zp,2) - pow(l_abad,2));
    double theta0 = atan2(
        zp * l0 + yp * L, yp * l0 - zp * L
    );
    if (leg_i == 0 || leg_i == 2)
        theta0 = (-1) * theta0;
    

    // get theta2
    double ap = sqrt(pow(xp,2) + pow(zp,2) + pow(yp,2) - pow(l_abad,2));
    double theta2 = -M_PI + acos(
        (pow(l1,2) + pow(l2,2) - pow(ap,2)) /
        (2 * l1 * l2)
    );

    // get theta1
    double a1 = yp * sin(theta0) - zp * cos(theta0);
    double a2 = xp;
    double m1 = l2 * sin(theta2);
    double m2 = l2 * cos(theta2) + l1;
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
        (theta2 < -161.0/180 * M_PI) ||
        (theta2 > -51.0/180 * M_PI) 
    )
        ROS_WARN("ANGLE HITS MECHANICAL LIMITS!");

    return Eigen::Vector3d(theta0, theta1, theta2);
}

Eigen::Matrix3d ctrl_server::get_Jacobian(
    int leg_i
)
{
    Eigen::Vector3d leg_q = get_leg_q(leg_i, q_state);
    Eigen::Vector3d leg_param = get_leg_kine_param(leg_i);

    double theta0 = leg_q(0), theta1 = leg_q(1), theta2 = leg_q(2);
    double l0 = leg_param(0), l1 = leg_param(1), l2 = leg_param(2);

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

Eigen::Vector3d ctrl_server::get_linear_velocity(
    int leg_i
)
{
    Eigen::Matrix3d J = get_Jacobian(leg_i);
    Eigen::VectorXd leg_dq = get_leg_dq(leg_i, q_state);

    return J * leg_dq;
}

Eigen::Vector3d ctrl_server::get_foot_p_B(int leg_i)
{
    Eigen::Vector3d _r_base2hip;
    switch (leg_i)
    {
    case 0:
        _r_base2hip = r_base2FRhip;
        break;
    
    case 1:
        _r_base2hip = r_base2FLhip;
        break;
        
    case 2:
        _r_base2hip = r_base2RRhip;
        break;

    case 3:
        _r_base2hip = r_base2RLhip;
        break;
    
    default:
        break;
    }

    return _r_base2hip + forward_kinematics(leg_i);
}

Eigen::Vector3d ctrl_server::get_q_from_B(
    int leg_i, 
    Eigen::Vector3d r_E_B
)
{
    Eigen::Vector3d _r_base2hip;
    
    switch (leg_i)
    {
    case 0:
        _r_base2hip = r_base2FRhip;
        break;
    
    case 1:
        _r_base2hip = r_base2FLhip;
        break;
        
    case 2:
        _r_base2hip = r_base2RRhip;
        break;

    case 3:
        _r_base2hip = r_base2RLhip;
        break;
    
    default:
        break;
    }
    Eigen::Vector3d r_E_L; // in leg frame
    return inverse_kinematics(leg_i, r_E_L);
}
