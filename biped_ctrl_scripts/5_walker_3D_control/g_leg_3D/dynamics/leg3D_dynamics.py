
import sympy as sp

# Define symbols
m_foot, m_calf, m_knee, m_thigh, m_hip = sp.symbols('m_foot, m_calf, m_knee, m_thigh, m_hip', real=True)
I_hx, I_hy, I_hz = sp.symbols('I_hx, I_hy, I_hz', real=True)
I_tx, I_ty, I_tz = sp.symbols('I_tx, I_ty, I_tz', real=True)
I_kx, I_ky, I_kz = sp.symbols('I_kx, I_ky, I_kz', real=True)
  
# Mass hinge, leg, Inertia
l_hip, l_thigh, l_knee = sp.symbols('l_hip, l_thigh, l_knee', real=True)  # length

g = sp.symbols('g', real=True)  # Slope of ramp, gravity
phi0, theta1, theta2 = sp.symbols('phi0 theta1 theta2', real=True)  # Angles
omega0, omega1, omega2 = sp.symbols('omega0 omega1 omega2', real=True)  # Angular velocity
alpha0, alpha1, alpha2 = sp.symbols('alpha0 alpha1 alpha2', real=True)  # Angular acceleration

# Generalized Coordinates in ground frame {I}
q = [phi0, theta1, theta2]
qdot = [omega0, omega1, omega2]

# Rotation matrices
R_1_to_0 = sp.Matrix([
    [1, 0, 0],
    [0, sp.cos(phi0), -sp.sin(phi0)],
    [0, sp.sin(phi0), sp.cos(phi0)]
])

R_2_to_1 = sp.Matrix([
    [sp.cos(theta1), 0, -sp.sin(theta1)],
    [0, 1, 0],
    [-sp.sin(theta1), 0, sp.cos(theta1)]
])

R_3_to_2 = sp.Matrix([
    [sp.cos(theta2), 0, -sp.sin(theta2)],
    [0, 1, 0],
    [-sp.sin(theta2), 0, sp.cos(theta2)]
])

t_1_to_0 = sp.Matrix([0, 0, 0])
T_1_to_0 = sp.Matrix([[R_1_to_0, t_1_to_0], [0, 0, 0, 1]])

t_2_to_1 = sp.Matrix([0, 0, 0])
T_2_to_1 = sp.Matrix([[R_2_to_1, t_2_to_1], [0, 0, 0, 1]])

t_3_to_2 = sp.Matrix([0, l_hip, -l_thigh])
T_3_to_2 = sp.Matrix([[R_3_to_2, t_3_to_2], [0, 0, 0, 1]])



# point of foot, calf_com, knee, thigh_com, hip, hip_com
p_foot_in_3 = sp.Matrix([0, 0, -l_knee, 1])
p_foot_in_0 = T_1_to_0 @ T_2_to_1 @ T_3_to_2 @ p_foot_in_3

# print()
# print("=======================")
# print("p_foot")
# print(p_foot_in_0)

p_calf_com_in_3 = sp.Matrix([0, 0, -l_knee/2, 1])
p_calf_com_in_0 = T_1_to_0 @ T_2_to_1 @ T_3_to_2 @ p_calf_com_in_3

# print(p_calf_com_in_0)

p_knee_in_2 = sp.Matrix([0, l_hip, -l_thigh, 1])
p_knee_in_0 = T_1_to_0 @ T_2_to_1 @ p_knee_in_2

# print()
# print("=======================")
# print("p_knee")
# print(p_knee_in_0)

p_thigh_com_in_2 = sp.Matrix([0, l_hip, -l_thigh/2, 1])
p_thigh_com_in_0 = T_1_to_0 @ T_2_to_1 @ p_thigh_com_in_2

# print(p_thigh_com_in_0)

p_hip_in_2 = sp.Matrix([0, l_hip, 0, 1])
p_hip_in_0 = T_1_to_0 @ T_2_to_1 @ p_hip_in_2

# print()
# print("=======================")
# print("p_hip")
# print(p_hip_in_0)

p_hip_com_in_2 = sp.Matrix([0, l_hip/2, 0, 1])
p_hip_com_in_0 = T_1_to_0 @ T_2_to_1 @ p_hip_com_in_2

# print(p_hip_com_in_0)

x_foot, y_foot, z_foot = p_foot_in_0[0], p_foot_in_0[1], p_foot_in_0[2]
x_calf_com, y_calf_com, z_calf_com = p_calf_com_in_0[0], p_calf_com_in_0[1], p_calf_com_in_0[2]
x_knee, y_knee, z_knee = p_knee_in_0[0], p_knee_in_0[1], p_knee_in_0[2]
x_thigh_com, y_thigh_com, z_thigh_com = p_thigh_com_in_0[0], p_thigh_com_in_0[1], p_thigh_com_in_0[2]
x_hip, y_hip, z_hip = p_hip_in_0[0], p_hip_in_0[1], p_hip_in_0[2]
x_hip_com, y_hip_com, z_hip_com = p_hip_com_in_0[0], p_hip_com_in_0[1], p_hip_com_in_0[2]

# linear velocities
v_foot = sp.Matrix([sp.simplify(sp.Matrix([x_foot, y_foot, z_foot]).jacobian(q) @ sp.Matrix(qdot))])
v_calf_com = sp.Matrix([sp.simplify(sp.Matrix([x_calf_com, y_calf_com, z_calf_com]).jacobian(q) @ sp.Matrix(qdot))])
v_knee = sp.Matrix([sp.simplify(sp.Matrix([x_knee, y_knee, z_knee]).jacobian(q) @ sp.Matrix(qdot))])
v_thigh_com = sp.Matrix([sp.simplify(sp.Matrix([x_thigh_com, y_thigh_com, z_thigh_com]).jacobian(q) @ sp.Matrix(qdot))])
v_hip_com = sp.Matrix([sp.simplify(sp.Matrix([x_hip_com, y_hip_com, z_hip_com]).jacobian(q) @ sp.Matrix(qdot))])

# angular velocities

print("POSI IN {0} ACQUIRED")
print("VELO IN {0} ACQUIRED")


# potential
zp_foot = p_foot_in_0[2]
zp_calf_com = p_calf_com_in_0[2]
zp_knee = p_knee_in_0[2]
zp_thigh_com = p_thigh_com_in_0[2]
zp_hip = p_hip_in_0[2]
zp_hip_com = p_hip_com_in_0[2]

print("HEIGHT IN {I} ACQUIRED")

# Kinetic and potential energy
T = 0.5 * (m_foot * v_foot.dot(v_foot) + m_calf * v_calf_com.dot(v_calf_com) + m_knee * v_knee.dot(v_knee) + m_thigh * v_thigh_com.dot(v_thigh_com) + m_hip * v_hip_com.dot(v_hip_com) 
           
           + I1 * omega0**2 + I2 * (omega0 + omega1)**2)
V = sp.simplify(m1 * g * Y_G1 + m2 * g * Y_G2 + M * g * Y_H)
L = T - V
print("LAGRANGIAN ACQUIRED")

exit()


# Potential energy
Y_H = R_H[1]
Y_G1 = R_G1[1]
Y_G2 = R_G2[1]

print("HEIGHT IN {I} ACQUIRED")



# Derive equations of motion
qddot = [ax, ay, alpha0, alpha1]
EOM = []
for i in range(4):
    dLdqdot = sp.diff(L, qdot[i])
    ddt_dLdqdot = sum([sp.diff(dLdqdot, q[j]) * qdot[j] + sp.diff(dLdqdot, qdot[j]) * qddot[j] for j in range(4)])
    # sp.diff(dLdqdot, q[j]) * qdot[j], remark -> d Blah / dt = d Blah / dq * dq / dt = d Blah / dq * qdot
    # sp.diff(dLdqdot, qdot[j]) * qddot[j], remark -> d Blah / dt = d Blah / ddq * ddq / dt = d Blah / dq * qddot
    dLdq = sp.diff(L, q[i])
    EOM.append(sp.simplify(ddt_dLdqdot - dLdq))
print("{d/dt dL/dqdot - dL/dq} ACQUIRED")

EOM_vec = sp.simplify(sp.Matrix([EOM[i] for i in range(4)]))
# print(EOM_vec)
# Compute the system's M_ss matrix and b_ss vector
M_ss = EOM_vec.jacobian(qddot)
# d/dt dL/dqdot - dL/dq = tau
# M(q) q'' + B(q,q') = tau
# use Jacobian to separate the M(q) from EOM
# The linear dependence in EoM allows the Jacobian to extract the mass matrix

b_ss = sp.simplify(EOM_vec.subs([(ax,0), (ay,0), (alpha0,0), (alpha1,0)]))
# M(q) q'' + B(q,q') = tau
# let q'' = 0, we can then get B(q,q')


# M(q) q'' + B(q,q') = tau
#  = M(q) q'' + B(q,q') = J^T F_c (refer to notes)
# -> we only care about the kinematics right now
# -> 3rd & 4th row
M_ss_reduced = M_ss[2:, 2:]
b_ss_reduced = b_ss[2:]

print()
print("SINGLE STANCE")
print("===DERIVATION ENDED===")
print("M_ss = ", M_ss_reduced)
print()
print("M00 = ", M_ss_reduced[0,0])
print("M01 = ", M_ss_reduced[0,1])
print("M10 = ", M_ss_reduced[1,0])
print("M11 = ", M_ss_reduced[1,1])
print()
print("b_ss = ", b_ss_reduced)
print()
print("b_0 = ", b_ss_reduced[0])
print("b_1 = ", b_ss_reduced[1])
print("=====================")
