# Bootcamp for Legged Robots
<table align="center">
    <tr>
      <td align="center">
        <img src="/viz/quadruped_trot.gif" alt="doggie trot" width="300"/>
      </td>
      <td align="center">
        <img src="/viz/walker3D_control_partition_gif.gif" alt="walker walk" width="300"/>
      </td>
    </tr>
</table>

## Introduction
This is a learning bootcamp for legged robotics newbies. Nevertheless, fundamentals in robotics are grossly omitted; for those who have some basic knowledge, you are in the right place. In which, we have 2 main camps:
1. **3D Walker**: Naive simulator from scratch, which consists the development of:
    - [Hopper](/biped_ctrl_scripts/1_hopper_dynamics/README.md)
    - [Hopper Control](/biped_ctrl_scripts/2_hopper_control/README.md)
    - [Passive Walker](/biped_ctrl_scripts/3_passive_walker/README.md)
    - [Walker Control](/biped_ctrl_scripts/4_walker_control/README.md)
    - [3D Walker Control](/biped_ctrl_scripts/5_walker_3D_control/README.md)

    Goal:
    <p align="center">
      <img src="/viz/walker3D_control_partition_gif.gif" alt="Walker 3D Control Partition" style="width: 100%;"/>
    </p>

    via:
      - dynamics modelling (Euler-Lagrange Equations), 
      - fixed points with Poicare maps, 
      - forward and inverse kinematics, 
      - trajectory optimization, and 
      - feedback linearization.
      </br>
    <table align="center">
      <tr>
        <td align="center">
          <img src="/viz/double_pendulum_cartesian_control.gif" alt="Double Pendulum" width="300"/>
        </td>
        <td align="center">
          <img src="/viz/passive_walker_control_partition.gif" alt="Passive Walker" width="300"/>
        </td>
      </tr>
      <tr>
        <td align="center">
          <img src="/viz/3Dleg_inverse_kinematics.gif" alt="Inverse Kinematics 3D" width="300"/>
        </td>
        <td align="center">
          <img src="/viz/2Dleg_inverse_kinematics.gif" alt="Inverse Kinematics 2D" width="300"/>
        </td>
      </tr>
      <tr>
        <td align="center">
          <img src="/viz/spring_mass_damper.gif" alt="Spring Mass Damper" width="300"/>
        </td>
        <td align="center">
          <img src="/viz/raibert_hopper_200.gif" alt="Raibert Hopper" width="300"/>
        </td>
      </tr>
      <tr>
        <td align="center">
          <img src="/viz/bounce_3D.gif" alt="Bounce" width="300"/>
        </td>
        <td align="center">
          <img src="/viz/projectile.gif" alt="Projectile" width="300"/>
        </td>
      </tr>
    </table>
 
2. **Quadruped**: Basic controller design for quadruped with [unitree Go1](https://github.com/unitreerobotics/unitree_ros) in ROS/Gazebo simulation:
    - [Joints Control](/quadruped_ctrl_ros/src/ctrl/swing_leg.cpp)
    - [Base Control](/quadruped_ctrl_ros/src/ctrl/squiggle.cpp)
    - [Force Control](/quadruped_ctrl_ros/src/ctrl/balance.cpp)
    - [Gait Design](/quadruped_ctrl_ros/src/ctrl/gait.cpp)
    - [Trot/Crawl/Pronk](/quadruped_ctrl_ros/src/ctrl/gait_ctrl.cpp)
  
    Goal:
      <table align="center">
        <tr>
          <td align="center">
            <img src="/viz/quadruped_trot.gif" alt="trot" width="300"/>
          </td>
          <td align="center">
            <img src="/viz/quadruped_crawl.gif" alt="trot" width="300"/>
          </td>
        </tr>
        <tr>
          <td align="center">
            <img src="/viz/quadruped_pronk.gif" alt="pronk" width="300"/>
          </td>
          <td align="center">
            <img src="/viz/quadruped_dance.gif" alt="dance" width="300"/>
          </td>
        </tr>
      </table>

    via:
      - forward and inverse kinematics, 
      - dynamics modelling (Euler-Lagrange Equations), 
      - quadratic programming, and
      - cycloid trajectories.
      </br>
      <table align="center">
        <tr>
          <td align="center">
            <img src="/viz/quadruped_stand.gif" alt="stand" width="300"/>
          </td>
          <td align="center">
            <img src="/viz/quadruped_swing.gif" alt="swing" width="300"/>
          </td>
        </tr>
        <tr>
          <td align="center">
            <img src="/viz/quadruped_balance.gif" alt="balance" width="300"/>
          </td>
          <td align="center">
            <img src="/viz/quadruped_squiggle.gif" alt="squiggle" width="300"/>
          </td>
        </tr>
      </table>
    
## Usage
For ```biped``` scripts:
1. Please first setup ```conda``` environment.
   ```
   conda create --name leg_bootcamp
   conda activate leg_bootcamp

   cd ./biped_ctrl_scripts
   conda install --file setup.txt
   ```
2. For 3D modelling, please do the following to wrap up python scripts in C, which will take around 1 minute (tested on M1 pro):
   ```
   cd ./biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/compiled_funcs
   python gen_lib.py
   ```
3. Execute the python files.
  
For ```quadruped``` codes:
1. Please follow [this](https://github.com/Dynamics-Learning-Workshop/unitree_ros) and [that](https://github.com/Dynamics-Learning-Workshop/unitree_guide) to setup the simulation environment.
  - please note that ROS is a prerequisite.
  - we tested all the codes on ubuntu 20.04 + ROS Noetic.
  - also note that [lcm](https://github.com/lcm-proj/lcm), [osqp](https://github.com/osqp/osqp) and [osqp-eigen](https://github.com/robotology/osqp-eigen) should be installed.
  - I use ```tmux``` a lot, and this repo also used tmux. Please install tmux by ```sudo apt install tmux```.
2. Prepare a gaming controller that is compatible with ```/joy```. Setup the controller via this [instruction](./viz/joy_tutorial.pdf).
3. My practice is to install ```unitree_ros``` and ```unitree_guide``` in one workspace, and this repo in another workspace.
4. Say the workspaces are respectively ```leg_sim_ws``` and ```leg_boot_ws```, first compile them with ```catkin_make```, and then:
   ```
   cd ~ && ./leg_sim_ws/src/unitree_guide/sim.sh
   ```
5. Then
   ```
   source ~/leg_boot_ws/devel/setup.bash 
   roslaunch quadruped_ros_ctrl ctrl.launch
   ```
6. With the terminal, you will see some texts telling you the current FSM state. The control inputs via the controller are:
   <p align="center">
      <img src="/viz/controller.jpg" alt="PS4" style="width: 100%;"/>
  </p>


## Reference
Based on 
- UIC course (legged robotics) by Pranav Bhounsule, 
  - I did my own implementation in Python
- Open project by Boston Cleek and Unitree Guide., 
  - I took reference from the code structure
  - and designed my own controller that can
    - stand
    - walk
    - pronk
- Simulation platform developed by UniTree Gazebo.
  - my implementation is based in Gazebo

```
@misc{pranav,
  title        = {Legged Robotics},
  author       = {Pranav Bhounsule},
  year         = 2021,
  note         = {\url{https://pab47.github.io/legs.html} [Accessed: 17/Sep/2024]}
}

@misc{qcontrol,
  title        = {Quadruped Control},
  author       = {Boston Cleek},
  year         = 2021,
  note         = {\url{https://github.com/bostoncleek/quadruped_control} [Accessed: 17/Sep/2024]}
}

@misc{unitree,
  title        = {Unitree Ros},
  author       = {Unitree},
  year         = 2024,
  note         = {\url{https://github.com/unitreerobotics/unitree_ros} [Accessed: 17/Sep/2024]}
}
```