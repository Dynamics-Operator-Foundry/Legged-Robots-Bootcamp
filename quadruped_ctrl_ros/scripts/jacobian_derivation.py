import sympy as sp, numpy as np
import matplotlib.pyplot as plt

l0, l1, l2 = sp.symbols('l0, l1, l2', real=True)  # length

g = sp.symbols('g', real=True)  # Slope of ramp, gravity
theta0, theta1, theta2 = sp.symbols('theta0 theta1 theta2', real=True)  # Angles
omega0, omega1, omega2 = sp.symbols('omega0 omega1 omega2', real=True)  # Angular velocity

# Rotation matrices
R_1_to_0 = sp.Matrix([
    [1, 0, 0],
    [0, sp.cos(theta0), -sp.sin(theta0)],
    [0, sp.sin(theta0), sp.cos(theta0)]
])

R_2_to_1 = sp.Matrix([
    [sp.cos(theta1), 0, sp.sin(theta1)],
    [0, 1, 0],
    [-sp.sin(theta1), 0, sp.cos(theta1)]
])

R_3_to_2 = sp.Matrix([
    [sp.cos(theta2), 0, sp.sin(theta2)],
    [0, 1, 0],
    [-sp.sin(theta2), 0, sp.cos(theta2)]
])

t_1_to_0 = sp.Matrix([0, 0, 0])
T_1_to_0 = sp.Matrix([[R_1_to_0, t_1_to_0], [0, 0, 0, 1]])

t_2_to_1 = sp.Matrix([0, 0, 0])
T_2_to_1 = sp.Matrix([[R_2_to_1, t_2_to_1], [0, 0, 0, 1]])

t_3_to_2 = sp.Matrix([0, l0, l1])
T_3_to_2 = sp.Matrix([[R_3_to_2, t_3_to_2], [0, 0, 0, 1]])



# point of foot, calf_com, knee, thigh_com, hip, hip_com
p_foot_in_3 = sp.Matrix([0, 0, l2, 1])
p_foot_in_0 = T_1_to_0 @ T_2_to_1 @ T_3_to_2 @ p_foot_in_3

# print()
# print("=======================")
# print("p_foot")
# print(sp.simplify(p_foot_in_0))

r_E = sp.Matrix([p_foot_in_0[0:3]])

# print(sp.simplify(r_E))
# exit()

# Generalized Coordinates in ground frame {I}
q = [theta0, theta1, theta2]
qdot = [omega0, omega1, omega2]

# Jacobian matrix (only the first two rows for 2D position)
J = sp.simplify(r_E.jacobian(q))

print(J)
exit()

# print()
# print('FORWARD KINEMATICS')
# # R_H = sp.simplify(R_H)
# for i in range(2):
#     print('R'+str(i)+' = ',R_E[i])
print()
print()
print('JACOBIAN')
for i in range(2):
    for j in range(2):
        print('J'+str(i)+str(j)+ ' = ' , J[i,j])
        
        

# Compute J_dot using the chain rule
qdot = [theta0dot, theta1dot]
J_dot = sp.Matrix([[sp.diff(J[i, j], theta0) * theta0dot + sp.diff(J[i, j], theta1) * theta1dot for j in range(J.shape[1])] for i in range(J.shape[0])])
print()
print('JACOBIAN DOT')
J_dot = sp.simplify(J_dot)
# print(J_dot)
for i in range(2):
    for j in range(2):
        print('J'+str(i)+str(j)+ ' = ' , J_dot[i,j])
        