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

#include "quadruped_ctrl_ros/ctrl_server.h"

void ctrl_server::keyboardCallback(
    const ros::TimerEvent &e
)
{
    checkKeyPress();
    
    std::cout<<"keyboard!"<<std::endl;

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

    key_states.clear();
}

bool ctrl_server::check_fsm_change()
{
    return false;
    
}

void ctrl_server::keyboardInitTerminal() {
    termios settings;
    tcgetattr(STDIN_FILENO, &settings);
    settings.c_lflag &= ~(ICANON); // Disable canonical mode
    tcsetattr(STDIN_FILENO, TCSANOW, &settings);
}

void ctrl_server::checkKeyPress() {
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

    while (read(STDIN_FILENO, &ch, 1) == 1) {
        key_states[ch] = true;
        if (ch == 'm')
            break;
    }
}