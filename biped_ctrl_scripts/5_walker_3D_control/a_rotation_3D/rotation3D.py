import matplotlib.pyplot as plt
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util

l = 1
# q_all
rod_start = np.array([0,0,0])
q = np.array([])

rod_start_x0_all = []
rod_start_x1_all = []
rod_start_x2_all = []
q1_all = []
q2_all = []
t_all = []

sample_factor = 10

l = 1
t_step = 1e-3

# single dot
# rod_start_x0_all.append(rod_start[0])
# rod_start_x1_all.append(rod_start[1])
# rod_start_x2_all.append(rod_start[2])

# pitch = np.deg2rad(180)
# yaw = np.deg2rad(180)
# q1_all.append(pitch)
# q2_all.append(yaw)
# t_all.append(0)

# trajectory
steps = int(10/t_step)
print(steps)
# exit()
t_all = np.linspace(0, 10, steps)  

pitch_array = np.deg2rad(np.linspace(0, 4*360, len(t_all)))
# pitch_array = np.deg2rad(0 * t_all)

yaw_array = np.deg2rad(np.linspace(0, 4*360, len(t_all)))
# yaw_array = np.deg2rad(0 * t_all)

# Generate yaw array: 0 -> 360 degrees, repeating 4 times in 10 seconds
yaw_array = np.deg2rad(np.linspace(0, 4 * 360, len(t_all)))

# Generate pitch array: -90 -> 90 degrees, repeating 2 times to cover the full sphere
pitch_array = np.deg2rad(90 * np.sin(2 * np.pi * t_all / 10))


q1_all = pitch_array  # Array of pitch angles (in radians)
q2_all = yaw_array    # Array of yaw angles (in radians)
rod_start_x0_all = np.full_like(pitch_array, rod_start[0])
rod_start_x1_all = np.full_like(pitch_array, rod_start[1])
rod_start_x2_all = np.full_like(pitch_array, rod_start[2])

sim3D().anime(
    t=t_all[::sample_factor], 
    x_states=[
        rod_start_x0_all[::sample_factor], 
        rod_start_x1_all[::sample_factor], 
        rod_start_x2_all[::sample_factor], 
        q1_all[::sample_factor], 
        q2_all[::sample_factor]
    ], 
    ms=1000 * t_step * sample_factor,
    mission="Rotate", 
    sim_object="rod",
    sim_info={'rod_radius': 0.2, 'rod_length': l}, 
    save=False,
    save_name='rotate_3D'
)

exit()