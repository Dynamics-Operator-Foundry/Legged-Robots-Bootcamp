# Walker 3D
## Tasks
### A. Rotation 3D
Rotation 3D revision at [here](/biped_ctrl_scripts/5_walker_3D_control/a_rotation_3D/bounce3D.py) and [here](/biped_ctrl_scripts/5_walker_3D_control/a_rotation_3D/rotation3D.py). In which, we also tried out 3D animation via $\texttt{matplotlib}$.

### B. Zero Reference Model
Here we introduce a transformation method, the zero reference model. Please refer to [this](/biped_ctrl_scripts/5_walker_3D_control/b_zero_reference_model/kinematics_zero_reference_model_walker.py).

### C. Walker
Please refer to [this](/biped_ctrl_scripts/3_passive_walker/d_walker/passive_walker.py).

### D. Trajectory
Please refer to [this](/biped_ctrl_scripts/4_walker_control/f_trajectory/).

### E. Control Partitioning
Please refer to [this](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/).


### F. Walker Control
- In this section, we first derive the kinematics of the 3D walker via zero reference model:
  - Kinematics [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/walker_3D_kinematics.py).
  - Jacobian w.r.t. ankle positions [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/walker_3D_dynamics_single_stance_jacobian.py).
  - Integration events [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/collision_detection.py).
  - Hip position initialization [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/walker_3D_dynamics_init.py).
- We then derive the dynamics for single stance and foot strike step by step.
  - Lagragian [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/walker_3D_dynamics_single_stance_Lagragian.py).
  - Equation of Motion [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/walker_3D_dynamics_single_stance_EoM_0.py).
  - Mass matrix [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/walker_3D_dynamics_single_stance_EoM_1_mass_matrix.py).
  - N matrix [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/walker_3D_dynamics_single_stance_EoM_2_bmatrix.py).
  - Impulse at foot strike [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/walker_3D_dynamics_single_stance_EoM_2_bmatrix.py).
- Since the above scripts can be slow in $\texttt{Python}$, we did the following to perform acceleration.
  - Most $\texttt{SymPy}$ expressions are stored as binaries [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/compiled_funcs/binaries/). 
  - After the $\texttt{SymPy}$ derivations, we use the function $\texttt{autowrap}$ to compile the code using $\texttt{Cython}$ backend [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/compiled_funcs/gen_lib.py).
  - We also did autogeneration and save the function script files [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/funcs/).
  - We compare the computation time in this script [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/dynamics/test_scripts/demo.py). The differences between the two can be from $1e3$ to $1e4$.
- A library [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/walker3D_model.py) for visualization. It is inherited by the integration lib [here](/dynamics_bootcamp.py)
- Observe the dynamics via [integration](/dynamics_bootcamp.py) (RK4 or Euler forward). The script is written [here](/biped_ctrl_scripts/5_walker_3D_control/f_walker_3D/walker_3D_control.py).
- The script is not perfectly done, as we are using our naive integrator (not advanced ones like $\texttt{RK4}$), so we could not really get a real good fixpoint on the Poincare map. 
  
  Remarks: 
  - We set the trajectories and initial conditions via trial-error, and therefore, the robot performance is not perfect at this moment.
  - We refer to people who are interested to [this](https://pab47.github.io/legs.html) for a better performance.

### G. Leg 3D
- As we will be visiting quadruped in the next bootcamp, [here](/biped_ctrl_scripts/5_walker_3D_control/g_leg_3D/) we revisit the inverse kinematics for the 3D leg of a quadruped robot.
  