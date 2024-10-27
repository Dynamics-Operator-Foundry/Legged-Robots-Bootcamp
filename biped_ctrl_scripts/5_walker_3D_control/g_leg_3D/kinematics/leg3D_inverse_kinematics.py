import sympy as sp, numpy as np
import matplotlib.pyplot as plt

r_x0, r_y0 = sp.symbols('r_x0, r_y0', real=True)
theta0, theta1 = sp.symbols('theta0 theta1', real=True)  # Angles
theta0dot, theta1dot = sp.symbols('theta0dot theta1dot', real=True)  # Angles
l1, l2 = sp.symbols('l1, l2', real=True)  # Distances

R_B1_2_I = sp.Matrix([[sp.cos(theta0), -sp.sin(theta0)],
                 [sp.sin(theta0), sp.cos(theta0)]])
R_B2_2_B1 = sp.Matrix([[sp.cos(theta1), -sp.sin(theta1)],
                 [sp.sin(theta1), sp.cos(theta1)]])

CP_I = sp.Matrix([0, 0]) # contact point in ground frame {I}
HP_B1 = sp.Matrix([l1, 0]) # hinge point in body frame 1 {B1}

T_B1_2_I = sp.Matrix([[R_B1_2_I, CP_I], [0, 0, 1]])
T_B2_2_B1 = sp.Matrix([[R_B2_2_B1, HP_B1], [0, 0, 1]])

R_H = T_B1_2_I @ sp.Matrix([l1, 0, 1]) # hinge
R_E = T_B1_2_I @ T_B2_2_B1 @ sp.Matrix([l2, 0, 1]) # CoM of leg 2


# Joint variables
q = [theta0, theta1]

# Jacobian matrix (only the first two rows for 2D position)
R_E = sp.Matrix(R_E[0:2])
J = sp.simplify(R_E.jacobian(q))

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
        