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
#include "quadruped_ctrl_ros/osqpwrapper.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <gazebo_msgs/ModelStates.h>

// FSM_STATE
#define PASSIVE "PASSIVE" // 0
#define STAND "STAND" // 1
#define SWING_LEG "SWING_LEG" // 2
#define SQUIGGLE "SQUIGGLE" // 3
#define BALANCE "BALANCE" // 4
#define CRAWL "CRAWL" // 5
#define TROT "TROT" // 6
#define PRONK "PRONK" // 7
#define UPRIGHT "UPRIGHT" // 7

// CTRL_MODE
// tau = tau_ff + Kp * dq + Kd * dqot
#define POS_MODE "POS_MODE"
#define VEL_MODE "VEL_MODE"
#define DAMP_MODE "DAMP_MODE"
#define TORQ_MODE "TORQ_MODE"
#define MIX_MODE "MIX_MODE"


class ctrl_server : private ros_utilities, osqpwrapper
{
    
private:
    ros::NodeHandle nh;
    
// pointers
    std::unique_ptr<ros::Subscriber[]> servo_subscribers_ptr;
    std::unique_ptr<ros::Publisher[]> servo_publishers_ptr;

// quadruped parameters & objects
    bool sys_started = false;
    double ctrl_freq = 400.0;
    double t_start;

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
    std::vector<Eigen::Vector3d> r_all_base2hip;
    Eigen::Vector3d r_base2FRhip, r_base2FLhip, r_base2RRhip, r_base2RLhip;

// quadruped fsm and control mode
    std::string CTRL_MODE = DAMP_MODE;
    std::string FSM_STATE = PASSIVE;
    bool print_or_not = true;

    void fsm_manager();

//ros related
    // subscriber
    ros::Subscriber fsm_sub, robot_base_sub, vcmd_sub;
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
    
    void robotBaseCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    Sophus::SE3d pose_SE3_robot_base;
    Sophus::Vector6d twist_robot_base;

    void vCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
    Sophus::Vector6d twist_normalized_cmd;

    // publisher
    void register_publishers();
    void publish_servos(unitree_legged_msgs::LowCmd& _cmdSet);

    // timer
    ros::Timer mainspin_timer;
    void mainspinCallback(const ros::TimerEvent &e);

// main functions
    void fsm_reset();

    // PASSIVE
    void passive_ctrl();
    
    // STAND
    void target_ctrl();
    void set_target_ctrl();
    void set_target_ctrl_gain();
    void target_ctrl_reset();
    Eigen::VectorXd q_start;
    Eigen::VectorXd q_target;
    double target_percent = 0;
    bool target_track_start = false;

    // SWING LEG
    void swing_leg_ctrl();
    void set_swing_leg_ctrl();
    void set_swing_leg_ctrl_gain(int leg_i);
    void swing_leg_ctrl_reset();
    Eigen::Vector3d p_swing_target, q_swing_target;
    Eigen::Matrix3d Kp, Kd;
    bool swing_track_start = false;
    
    // SQUIGGLE
    void squiggle_ctrl();
    void set_squiggle_ctrl();
    void set_squiggle_ctrl_gain();
    void set_squiggle_atti();
    void squiggle_ctrl_reset();
    Eigen::Vector3d base_S; 
        // s_frame: squiggle frame
        // based frame original: FR_foot
    Sophus::SE3d base_pose_S;
        // T_B_2_S
    Eigen::Vector3d p_footFR_S, p_footFL_S, p_footRR_S, p_footRL_S;
    Eigen::Matrix<double, 3, 4> p_foots_S, p_foots_B, q_foots;
    // Eigen::Matrix
    double roll_mag, pitch_mag, yaw_mag, height_mag;
    double dangle_mag, dheight_mag;
    double ctrl_param = 0;
    double roll_base, pitch_base, yaw_base, height_base;
    std::string squiggle_fsm;
    bool squiggle_track_start = false;

    // balance
    void balance_ctrl();
    void set_balance_ctrl();
    void set_balance_ctrl_gain();
    void balance_ctrl_reset();
    Eigen::VectorXd get_f(
        std::vector<Eigen::Vector3d> feet_posi, 
        Sophus::Vector6d acc,
        Eigen::Vector4i contact_i
    );
    void set_HMat(const std::vector<Eigen::Vector3d> feet_posi);
    void set_fVec(const Sophus::Vector6d acc);
    void set_constraints();
    void set_tau(Eigen::Matrix<double, 12, 1> f_I);
    Eigen::Vector3d rotMatToExp(const Eigen::Matrix3d& rm);
    Eigen::Matrix<double, 6, 6> S_w;
    Eigen::Matrix<double, 12, 12> W_w, U_w;
    double alpha, beta;
    Eigen::VectorXd f_now, f_prev;
    double x_mag, y_mag, z_mag;
    Eigen::Vector3d posi_delta_B;
    Eigen::Vector3d posi_base_I;
    Eigen::Matrix3d Kp_p, Kd_p, Kd_w;
    double Kp_w;
    double m;
    Eigen::Matrix3d mI;
    Eigen::Matrix<double, 12, 12> HMat;
    Eigen::Matrix<double, 12,  1> fVec;
    Eigen::MatrixXd AMat;
    Eigen::MatrixXd ubVec, lbVec;
    Eigen::Matrix<double, 6,  12> A_dyn;
    double mu = 0.4;
    std::string balance_fsm;
    bool balance_track_start = false;
    Eigen::Matrix<double, 12, 1> balance_tau;

    void crawl_ctrl();

    // gait_control
    void gait_ctrl(
        const double _P_gait,
        const double _r_gait,
        const Eigen::Vector4d _b_gait,
        const Eigen::Vector3d _lim
    );
    void set_gait_ctrl_vel();
    void set_gait_ctrl_base_desired();
    void set_gait_ctrl_force();
    void set_gait_ctrl_swing();
    void set_gait_ctrl_cmd();
    void gait_ctrl_reset();
    Eigen::Vector3d trot_base_posi_desired;
    Eigen::Vector3d trot_base_dposi_desired;
    Eigen::Vector3d trot_base_atti_desired;
    Eigen::Vector3d trot_base_datti_desired;
    Eigen::Vector2d trot_vel_B;
    double trot_vel_yaw;
    Eigen::Vector3d trot_vel_I;
    bool trot_start = false;

    // gait
    void set_foot_traj();
    void set_gait_params(
        const double _P_gait,
        const double _r_gait,
        const Eigen::Vector4d _b_gait,
        const Eigen::Vector3d _lim
    );
    void calc_contact_phase();
    double cycloid_lateral(double start, double end, double phase);
    double cycloid_vertical(double start, double h, double phase);
    double cycloid_dlateral(double start, double end, double phase);
    double cycloid_dvertical(double start, double h, double phase);
    Eigen::Vector3d get_raibert_posi(int leg_i, Eigen::Vector2d velo_I, double dw, double phase_i);
    Eigen::Vector3d get_swing_foot_posi(
        int leg_i, 
        Eigen::Vector3d posi_start, 
        Eigen::Vector3d posi_end, 
        double phase_i
    );
    Eigen::Vector3d get_swing_foot_velo(
        int leg_i, 
        Eigen::Vector3d posi_start, 
        Eigen::Vector3d posi_end, 
        double phase_i
    );
    void draw_gait(cv::Mat &img, const Eigen::Vector4i &contact);
    // Eigen::Vector3d get_foot_touchdown_velo(int leg_i, Eigen::Vector2d velo_I, double dw, double phase_i);
    Eigen::Vector3d gait_vlim_B;
    double P_gait, r_gait;
    Eigen::Vector4d b_gait;
    Eigen::Vector4d phase_gait;
    Eigen::Vector4i contact_gait;
    Eigen::Matrix3d Kpb_p_trot;
    Eigen::Matrix3d Kdb_p_trot;
    double Kpb_w_trot;
    Eigen::Matrix3d Kdb_w_trot;
    Eigen::Matrix3d Kps_trot;
    Eigen::Matrix3d Kds_trot;
    std::vector<Eigen::Vector3d> swing_feet_posi_I, swing_feet_velo_I, swing_start_posi_I, swing_end_posi_I;
    double gait_height = 0.08;
    cv::Mat gait_viz;
    image_transport::Publisher image_pub;
    Eigen::Matrix<double, 3, 4> neutral_stance;
    std::vector<Eigen::Vector3d> q_swing_gait_target, dq_swing_gait_target;

    // upright
    void upwright_ctrl();
    void set_upwright_ctrl();
    void set_upwright_ctrl_gain();
    void upwright_ctrl_reset();

    std::string pre_FSM_STATE;
    void check_fsm_change();

// config
    void config();

// math
    Eigen::Vector3d get_leg_q(
        int leg_i,
        Eigen::VectorXd& q_state
    );
    Eigen::Vector3d get_leg_dq(
        int leg_i,
        Eigen::VectorXd& q_state
    );
    Eigen::Vector3d get_leg_kine_param(int leg_i);

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

    Eigen::Matrix3d get_Jacobian(
        int leg_i,
        Eigen::Vector3d& r_E
    );

    Eigen::Vector3d forward_diff_kinematics(
        int leg_i
    );

    Eigen::Vector3d inverse_diff_kinematics(
        int leg_i, 
        Eigen::Vector3d& r_E
    );

    Eigen::Vector3d inverse_diff_kinematics(
        int leg_i, 
        Eigen::Vector3d& p_E,
        Eigen::Vector3d& v_E
    );

    Eigen::Vector3d get_foot_p_B(int leg_i);
    Eigen::Vector3d get_q_from_B(int leg_i, Eigen::Vector3d r_E_B);
    double saturation_check(double val, Eigen::Vector2d range);

public:
    ctrl_server(ros::NodeHandle& _nh);
    ~ctrl_server();

};

#endif