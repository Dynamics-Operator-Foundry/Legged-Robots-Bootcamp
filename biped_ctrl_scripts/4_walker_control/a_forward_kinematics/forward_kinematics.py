import matplotlib.pyplot as plt
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from biped_ctrl_scripts.dynamics_bootcamp import RobotUtils

l1 = 1
l2 = 1

theta1 = 30 / 180 * np.pi
theta2 = 90 / 180 * np.pi
# l1_end_B1 = np.array([0,l1])
homo_B1_2_I = RobotUtils().homo2D(psi=theta1, trans=np.array([0,0]))

o = np.array([0,0])
# l1end_in_I = np.dot(homo_B1_2_I, np.array([l1,0,1]))
l1end_in_I = homo_B1_2_I @ np.array([l1,0,1])
l2end_in_B1 = RobotUtils().homo2D(psi=theta2, trans=np.array([l1,0])) @ np.array([l2,0,1])
# l2end_in_B1 = np.dot(
#         RobotUtils().homo2D(psi=theta2, trans=np.array([l1,0])),
#         np.array([l2,0,1])
#     )
# l2end_in_I = np.dot(homo_B1_2_I, l2end_in_B1)
l2end_in_I = homo_B1_2_I @ l2end_in_B1

print(l2end_in_B1)
print(l2end_in_I)
# exit()
l1end_in_B1 = [l1,0]

points = np.array([o,l1end_in_I[:2], l2end_in_I[:2]])
x = points[:, 0]
y = points[:, 1]

plt.scatter(x, y, color='blue', s=100)

plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Each Pts of the Manipulator')
plt.xlim(-3,3)
plt.ylim(-3,3)

plt.grid(True)
plt.show()


