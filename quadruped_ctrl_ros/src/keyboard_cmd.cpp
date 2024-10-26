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
 * \file keyboard_cmd.cpp
 * \date 25/10/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for quadruped_ctrl_ros_uav using airo_control_interface
 */

#include <ros_utilities/ros_utilities.h>
#include <termios.h>
#include <unistd.h>
#include <thread>

// FSM_STATE
#define PASSIVE "PASSIVE" // 0
#define TIP "TIP" // 1
#define STAND "STAND" // 2
#define SWING_LEG "SWING_LEG" // 3
#define SQUIGGLE "SQUIGGLE" // 4
#define CRAWL "CRAWL" // 5
#define TROT "TROT" // 6

static std::map<char, bool> key_states;
static std::string FSM_STATE;
static ros::Publisher fsm_pub;

void keyboardCallback(const ros::TimerEvent &e);
void keyboardInitTerminal();
void checkKeyPress();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle nh("~");

    nh.getParam("FSM_STATE", FSM_STATE);

    fsm_pub = nh.advertise<std_msgs::String>("/FSM", 1, true);

    ros::Timer keyboard_timer = nh.createTimer(
        ros::Duration(1.0/10.0),
        keyboardCallback
    );

    keyboardInitTerminal();
    
    ros::spin();

    return 0;
}

void keyboardCallback(
    const ros::TimerEvent &e
)
{
    checkKeyPress();
    std::cout<<std::endl;

    if (key_states['0'])
    {
        if(!(FSM_STATE == PASSIVE))
        {
            ROS_WARN("CHANGE FSM!");
            FSM_STATE = PASSIVE;
            ROS_GREEN_STREAM(FSM_STATE);
        }

    } 
    else if (key_states['1']) 
    {
        if(!(FSM_STATE == TIP))
        {
            ROS_WARN("CHANGE FSM!");
            FSM_STATE = TIP;
            ROS_GREEN_STREAM(FSM_STATE);
        }
    }
    else if (key_states['2']) 
    {
        if(!(FSM_STATE == STAND))
        {
            ROS_WARN("CHANGE FSM!");
            FSM_STATE = STAND;
            ROS_GREEN_STREAM(FSM_STATE);
        }
    }
    else if (key_states['3']) 
    {
        if(!(FSM_STATE == SWING_LEG))
        {
            ROS_WARN("CHANGE FSM!");
            FSM_STATE = SWING_LEG;
            ROS_GREEN_STREAM(FSM_STATE);
        }
    }
    else if (key_states['4']) 
    {
        if(!(FSM_STATE == SQUIGGLE))
        {
            ROS_WARN("CHANGE FSM!");
            FSM_STATE = SQUIGGLE;
            ROS_GREEN_STREAM(FSM_STATE);
        }
    }
    else if (key_states['5']) 
    {
        if(!(FSM_STATE == CRAWL))
        {
            ROS_WARN("CHANGE FSM!");
            FSM_STATE = CRAWL;
            ROS_GREEN_STREAM(FSM_STATE);
        }
    }
    else if (key_states['6']) 
    {
        if(!(FSM_STATE == TROT))
        {
            ROS_WARN("CHANGE FSM!");
            FSM_STATE = TROT;
            ROS_GREEN_STREAM(FSM_STATE);
        }
    }
    // ROS_INFO("MESSAGE RECEIVED!");
    std::string display = "CURRENT STATE: " + FSM_STATE + "!!!!!!!!";
    ROS_CYAN_STREAM(display);

    key_states.clear();

    std_msgs::String fsm_pub_object;
    fsm_pub_object.data = FSM_STATE;

    fsm_pub.publish(fsm_pub_object);
}



void keyboardInitTerminal() {
    termios settings;
    tcgetattr(STDIN_FILENO, &settings);
    settings.c_lflag &= ~(ICANON); // Disable canonical mode
    tcsetattr(STDIN_FILENO, TCSANOW, &settings);
}

void checkKeyPress() {
    char ch;
    std::cout<<"\n"<<std::endl;
    std::cout<<"0: PASSIVE"<<std::endl;
    std::cout<<"1: TIP"<<std::endl;
    std::cout<<"2: STAND"<<std::endl;
    std::cout<<"3: SWING_LEG"<<std::endl;
    std::cout<<"4: SQUIGGLE"<<std::endl;
    std::cout<<"5: CRAWL"<<std::endl;
    std::cout<<"6: TROT"<<std::endl;
    std::cout<<"\n"<<std::endl;
    std::cout<<"e: ENTER"<<std::endl;
    std::cout<<"q: QUIT"<<std::endl;
    std::cout<<"\n"<<std::endl;

    while (read(STDIN_FILENO, &ch, 1) == 1) {
        key_states[ch] = true;
        if (ch == 'e' || ch == 'E')
            break;

        if (ch == 'q' || ch == 'Q')
        {
            std::cout<<std::endl;
            ROS_WARN("SHUT DOWN");
            ros::shutdown(); 
            std::exit(EXIT_FAILURE);  
        }
    }
}