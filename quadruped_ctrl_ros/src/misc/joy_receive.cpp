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
 * \file joy_receive.cpp
 * \date 03/11/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros_utilities/ros_utilities.h>

// FSM_STATE
#define PASSIVE "PASSIVE" // 0
#define STAND "STAND" // 1
#define SWING_LEG "SWING_LEG" // 2
#define SQUIGGLE "SQUIGGLE" // 3
#define BALANCE "BALANCE" // 4
#define CRAWL "CRAWL" // 5
#define TROT "TROT" // 6
#define PRONK "PRONK" // 7
#define UPRIGHT "UPRIGHT" // 8 

static std::string FSM_STATE, pre_FSM_STATE;
static int key_no, key_no_prev;
static ros::Publisher fsm_pub, vel_pub;
static bool change_now;
static geometry_msgs::Twist vcmd;
void mainSpinCallback(const ros::TimerEvent &e);
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_receive");
    ros::NodeHandle nh("~");

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joyCallback);

    FSM_STATE = PASSIVE;
    key_no = 0;
    key_no_prev = key_no;

    fsm_pub = nh.advertise<std_msgs::String>("/FSM", 1, true);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/vcmd_normalized", 1, true);

    ros::Timer keyboard_timer = nh.createTimer(
        ros::Duration(1.0/10.0),
        mainSpinCallback
    );
    
    ros::spin();

    return 0;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{    
    vcmd.linear.x = msg->axes[1];
    vcmd.linear.y = msg->axes[0];
    vcmd.linear.z = msg->axes[4];
    vcmd.angular.x = 0;
    vcmd.angular.y = 0;
    vcmd.angular.z = msg->axes[3];

    vel_pub.publish(vcmd);

    if(msg->buttons[5] == 1) // R1
    {
        change_now = !change_now;

        if(change_now)
        {
            ROS_GREEN_STREAM("ENTER FSM SWITCHING!");
        }
        else
        {
            ROS_RED_STREAM("OUTSIDE FSM SWITCHING!");
            return;
        }
    }

    if(msg->buttons[6] == 1) // L2
    {
        key_no = 0;
        return;
    }

    if(msg->buttons[4] == 1) // L1
    {
        key_no = 1;
        return;
    }

    if(msg->buttons[0] == 1) // cross
    {
        key_no = 3;
        return;
    }

    if(msg->buttons[1] == 1) // circle
    {
        key_no = 7;
        return;
    }

    if(msg->buttons[2] == 1) // triangle
    {
        key_no = 6;
        return;
    }

    if(msg->buttons[3] == 1) // square
    {
        key_no = 5;
        return;
    }

    if(msg->buttons[7] == 1) // R2
    {
        key_no = 8;
        return;
    }
}

void mainSpinCallback(
    const ros::TimerEvent &e
)
{
    if(!change_now)
        return;

    if(key_no == key_no_prev)
        return;

    
    std::cout<<std::endl;

    if (key_no == 0)
    {
        if(!(FSM_STATE == PASSIVE))
        {
            ROS_WARN("CHANGE FSM!");
            FSM_STATE = PASSIVE;
            ROS_GREEN_STREAM(FSM_STATE);
            key_no_prev = key_no;
        }
    } 
    else if (key_no == 1) 
    {
        if(!(FSM_STATE == STAND))
        {
            ROS_WARN("CHANGE FSM!");
            FSM_STATE = STAND;
            ROS_GREEN_STREAM(FSM_STATE);
            key_no_prev = key_no;
        }
    }
    else if (key_no == 3) 
    {
        if(!(FSM_STATE == SQUIGGLE))
        {
            if (FSM_STATE != STAND)
            {
                ROS_WARN("PLEASE MAKE FSM \"STAND\" PRIOR TO SQUIGGLE!");
                key_no = key_no_prev;
            } 
            else
            {
                ROS_WARN("CHANGE FSM!");
                FSM_STATE = SQUIGGLE;
                ROS_GREEN_STREAM(FSM_STATE);
                key_no_prev = key_no;
            }
        }
    }
    else if (key_no == 5) 
    {
        if(!(FSM_STATE == CRAWL))
        {
            if (FSM_STATE != STAND)
            {
                ROS_WARN("PLEASE MAKE FSM \"STAND\" PRIOR TO CRAWL!");
                key_no = key_no_prev;
            }   
            else
            {
                ROS_WARN("CHANGE FSM!");
                FSM_STATE = CRAWL;
                ROS_GREEN_STREAM(FSM_STATE);
                key_no_prev = key_no;
            }
        }
    }
    else if (key_no == 6) 
    {
        if(!(FSM_STATE == TROT))
        {
            if (FSM_STATE != STAND)
            {
                ROS_WARN("PLEASE MAKE FSM \"STAND\" PRIOR TO TROT!");
                key_no = key_no_prev;
            }   
            else
            {
                ROS_WARN("CHANGE FSM!");
                FSM_STATE = TROT;
                ROS_GREEN_STREAM(FSM_STATE);
                key_no_prev = key_no;
            }
        }
    }
    else if (key_no == 7) 
    {
        if(!(FSM_STATE == PRONK))
        {
            if (FSM_STATE != STAND)
            {
                ROS_WARN("PLEASE MAKE FSM \"STAND\" PRIOR TO PRONK!");
                key_no = key_no_prev;
            }   
            else
            {
                ROS_WARN("CHANGE FSM!");
                FSM_STATE = PRONK;
                ROS_GREEN_STREAM(FSM_STATE);
                key_no_prev = key_no;
            }
        }
    }
    else if (key_no == 8) 
    {
        if(!(FSM_STATE == UPRIGHT))
        {
            if (FSM_STATE != STAND)
            {
                ROS_WARN("PLEASE MAKE FSM \"STAND\" PRIOR TO UPRIGHT!");
                key_no = key_no_prev;
            }   
            else
            {
                ROS_WARN("CHANGE FSM!");
                FSM_STATE = UPRIGHT;
                ROS_GREEN_STREAM(FSM_STATE);
                key_no_prev = key_no;
            }
        }
    }
    // ROS_INFO("MESSAGE RECEIVED!");
    std::string display = "CURRENT STATE: " + FSM_STATE + "!!!!!!!!";
    ROS_CYAN_STREAM(display);

    std_msgs::String fsm_pub_object;
    fsm_pub_object.data = FSM_STATE;

    fsm_pub.publish(fsm_pub_object);
}