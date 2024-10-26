cmake_minimum_required(VERSION 3.0.2)
project(quadruped_ctrl_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    joint_state_controller
    gazebo_ros
    robot_state_publisher
    unitree_legged_msgs
    ros_utilities
    roscpp
)
find_package(Boost)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    roscpp
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

add_executable(ctrl_node
    src/ctrl_node.cpp
    src/ctrl_server.cpp
    src/ctrl_math.cpp
    
    src/ctrl_IO.cpp
    src/ctrl_passive.cpp
    src/ctrl_target.cpp
    src/ctrl_swing_leg.cpp
    src/ctrl_squiggle.cpp
    src/ctrl_crawl.cpp
    src/ctrl_trot.cpp
)
target_link_libraries(ctrl_node
    ${catkin_LIBRARIES}
)
add_dependencies(ctrl_node
    unitree_legged_msgs_generate_messages_cpp
)

add_executable(keyboard_cmd
    src/keyboard_cmd.cpp
)
target_link_libraries(keyboard_cmd
    ${catkin_LIBRARIES}
)

