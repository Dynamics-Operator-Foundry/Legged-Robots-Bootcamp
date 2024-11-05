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
 * \file ctrl_gait.cpp
 * \date 25/10/20244
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include "quadruped_ctrl_ros/ctrl_server.h"

void ctrl_server::set_gait_params()
{
    t_start = ros::Time::now().toSec();
    P_gait = 0.45;
    r_gait = 0.5;
    b_gait << 0, 0.5, 0.5, 0;
    phase_gait.setConstant(0.5); 
    // all legs are at phase = 0.5, when everyone is on ground
    contact_gait << 1, 1, 1, 1;

    gait_vlim_B << 0.4, 0.3, 0.5;

    Kpb_p_trot = Eigen::Vector3d(70,70,70).asDiagonal();
    Kdb_p_trot = Eigen::Vector3d(10,10,10).asDiagonal();
    Kpb_w_trot = 780;
    Kdb_w_trot = Eigen::Vector3d(70,70,70).asDiagonal();
    Kps_trot = Eigen::Vector3d(400,400,400).asDiagonal();
    Kds_trot = Eigen::Vector3d(10,10,10).asDiagonal();

    f_prev.setZero();

    set_balance_ctrl();
    Kp_p = Kpb_p_trot;
    Kd_p = Kdb_p_trot;

    Kp_w = Kpb_w_trot;
    Kd_w = Kdb_w_trot;

    for (int leg_i = 0; leg_i < leg_no; leg_i ++)
    {
        swing_start_posi_I.emplace_back(
            pose_SE3_robot_base.rotationMatrix() * get_foot_p_B(leg_i) + pose_SE3_robot_base.translation()
        );
        swing_feet_posi_I.emplace_back(Eigen::Vector3d::Zero());
        swing_feet_velo_I.emplace_back(Eigen::Vector3d::Zero());
        swing_end_posi_I.emplace_back(Eigen::Vector3d::Zero());
    }
}

void ctrl_server::calc_contact_phase()
{
    // std::cout<<"time here"<<std::endl;
    double t_since_start = ros::Time::now().toSec() - t_start;
    // std::cout<<t_start<<std::endl<<std::endl;;
    Eigen::Vector4d normalized_time;

    for (int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        normalized_time(leg_i) = fmod(t_since_start + P_gait - P_gait * b_gait(leg_i), P_gait) / P_gait;

        // std::cout<<normalized_time(leg_i)<<std::endl;

        if (normalized_time(leg_i) < r_gait)
        {
            contact_gait(leg_i) = 1;
            phase_gait(leg_i) = normalized_time(leg_i) / r_gait;
        }
        else
        {
            contact_gait(leg_i) = 0;
            phase_gait(leg_i) = (normalized_time(leg_i) - r_gait) / (1 - r_gait);
        }
        // std::cout<<phase_gait(leg_i)<<std::endl;

        // std::cout<<std::endl;
    }

    // std::cout<<"=========time end========="<<std::endl;
}

void ctrl_server::set_foot_traj()
{
    // std::cout<<"set_gait"<<std::endl;

    for(int leg_i = 0; leg_i < leg_no; leg_i++)
    {
        if(contact_gait(leg_i) == 1)
        {
            // std::cout<<leg_i<<std::endl;
            if(phase_gait(leg_i) < 0.5)
            {
                swing_start_posi_I[leg_i] =
                    pose_SE3_robot_base.rotationMatrix() * get_foot_p_B(leg_i) + pose_SE3_robot_base.translation();
            }
            swing_feet_posi_I[leg_i] = swing_start_posi_I[leg_i];
            swing_feet_velo_I[leg_i] = Eigen::Vector3d::Zero();
        }
        else
        {
            swing_end_posi_I[leg_i] = get_raibert_posi(
                leg_i, 
                twist_robot_base.head(2), 
                twist_robot_base.tail(3)(2), 
                phase_gait(leg_i)
            );

            swing_feet_posi_I[leg_i] = get_swing_foot_posi(
                leg_i, 
                swing_start_posi_I[leg_i],
                swing_end_posi_I[leg_i],
                phase_gait(leg_i)
            );
            swing_feet_velo_I[leg_i] = get_swing_foot_velo(
                leg_i,
                swing_start_posi_I[leg_i],
                swing_end_posi_I[leg_i],
                phase_gait[leg_i]
            );

            // if(contact_gait[leg_i] == 0)
            // {
            //     std::cout<<contact_gait[leg_i]<<std::endl;
            //     std::cout<<phase_gait[leg_i]<<std::endl;
            //     std::cout<<std::endl<<feet_posi_start_I[leg_i]<<std::endl<<std::endl;
            //     std::cout<<end_posi_I[leg_i]<<std::endl<<std::endl;
            //     std::cout<<std::endl<<feet_posi_I[leg_i]<<std::endl<<std::endl;
            //     std::cout<<"============"<<std::endl;
            // }   
        }
    }
    // _pastP = feetPos;
    // _phasePast = *_phase;
}

double ctrl_server::cycloid_lateral(double start, double end, double phase)
{
    return start + (end - start) / (2 * M_PI) * (2 * M_PI * phase - sin(2 * M_PI * phase));
}

double ctrl_server::cycloid_vertical(double start, double h, double phase)
{
    return start + h / 2 * (1 - cos(2 * M_PI * phase));
}

double ctrl_server::cycloid_dlateral(double start, double end, double phase)
{
    double T  = (1 - r_gait) * P_gait; // T of swing
    return (end - start) / T * (1 - cos(2 * M_PI * phase));
}

double ctrl_server::cycloid_dvertical(double start, double h, double phase)
{
    double T  = (1 - r_gait) * P_gait; // T of swing
    return M_PI * h / T * sin(2 * M_PI * phase);
}

Eigen::Vector3d ctrl_server::get_raibert_posi(int leg_i, Eigen::Vector2d velo_desired_I, double dw_desired, double phase_i)
{
    double R = sqrt(
        pow(neutral_stance(0, leg_i), 2) 
            + 
        pow(neutral_stance(1, leg_i), 2) 
    );

    double init_angle = atan2(
        neutral_stance(1, leg_i), 
        neutral_stance(0, leg_i)
    );

    double t_swing = P_gait * (1 - r_gait); // r_gait stance ratio
    double t_stance = P_gait * r_gait; // r_gait stance ratio
    double kx = 0.005;
    double ky = 0.005;
    double kyaw = 0.005;

    double dw_now = twist_robot_base(5);
    Eigen::Vector3d twist_I = twist_robot_base.head(3);

    double thetaf = init_angle + q2rpy(pose_SE3_robot_base.unit_quaternion())(2) + dw_now * (1 - phase_i) * t_swing + 0.5 * dw_now * t_stance + kyaw * (dw_now - dw_desired);

    Eigen::Vector3d delta;

    delta.x() = R * cos(thetaf) + twist_I(0) * (1 - phase_i) * t_swing + 0.5 * twist_I(0) * t_stance + kx * (twist_I(0) - velo_desired_I(0));

    delta.y() = R * sin(thetaf) + twist_I(1) * (1 - phase_i) * t_swing + 0.5 * twist_I(1) * t_stance + ky * (twist_I(1) - velo_desired_I(1));

    Eigen::Vector3d raibert_touchdown = Eigen::Vector3d(
        pose_SE3_robot_base.translation().x() + delta.x(),
        pose_SE3_robot_base.translation().y() + delta.y(),
        0
    );

    return raibert_touchdown;
}

Eigen::Vector3d ctrl_server::get_swing_foot_posi(
    int leg_i, 
    Eigen::Vector3d posi_start, 
    Eigen::Vector3d posi_end, 
    double phase_i
)
{
    Eigen::Vector3d swing_posi;

    swing_posi(0) = cycloid_lateral(
        posi_start(0), 
        posi_end(0), 
        phase_gait(leg_i)
    );

    swing_posi(1) = cycloid_lateral(
        posi_start(1), 
        posi_end(1), 
        phase_gait(leg_i)
    );

    swing_posi(2) = cycloid_vertical(
        posi_start(2),
        gait_height,
        phase_gait(leg_i)
    );

    return swing_posi;
}

Eigen::Vector3d ctrl_server::get_swing_foot_velo(
    int leg_i, 
    Eigen::Vector3d posi_start, 
    Eigen::Vector3d posi_end, 
    double phase_i
)
{
    Eigen::Vector3d swing_velo;

    swing_velo(0) = cycloid_dlateral(
        posi_start(0), 
        posi_end(0), 
        phase_gait(leg_i)
    );

    swing_velo(1) = cycloid_dlateral(
        posi_start(1), 
        posi_end(1), 
        phase_gait(leg_i)
    );

    swing_velo(2) = cycloid_dvertical(
        posi_start(2),
        gait_height,
        phase_gait(leg_i)
    );

    return swing_velo;
}

void ctrl_server::draw_gait(
    cv::Mat &img, 
    const Eigen::Vector4i &contact
)
{
    int radius = 20;
    int spacing = 100;
    cv::Scalar stanceColor(255, 255, 255); // White for contact (stance)
    cv::Scalar swingColor(0, 0, 0);        // Black for no contact (swing)

    // Define positions for each leg: FR, FL, RR, RL
    std::vector<cv::Point> legPositions = {
        cv::Point(3 * spacing, spacing),         // Front Right (FR)
        cv::Point(spacing, spacing),     // Front Left (FL)
        cv::Point(3 * spacing, 3 * spacing),     // Rear Right (RR)
        cv::Point(spacing, 3 * spacing)  // Rear Left (RL)
    };

    for (int i = 0; i < 4; ++i)
    {
        // Determine if the leg is in contact (stance) or not (swing)
        cv::Scalar color = (contact[i] == 1) ? stanceColor : swingColor;

        // Draw circle for the leg
        cv::circle(img, legPositions[i], radius, color, -1);
    }

    // Add text labels for each leg
    cv::putText(img, "FR", legPositions[0] - cv::Point(20, -40), cv::FONT_HERSHEY_SIMPLEX, 0.5, stanceColor, 1);
    cv::putText(img, "FL", legPositions[1] - cv::Point(20, -40), cv::FONT_HERSHEY_SIMPLEX, 0.5, stanceColor, 1);
    cv::putText(img, "RR", legPositions[2] - cv::Point(20, -40), cv::FONT_HERSHEY_SIMPLEX, 0.5, stanceColor, 1);
    cv::putText(img, "RL", legPositions[3] - cv::Point(20, -40), cv::FONT_HERSHEY_SIMPLEX, 0.5, stanceColor, 1);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    image_pub.publish(msg);
}


void ctrl_server::reset_gait()
{

}