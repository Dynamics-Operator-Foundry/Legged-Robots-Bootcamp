
import sympy as sp

# Define symbols
M1, M2, I1, I2, n = sp.symbols('M1 M2 I1 I2, n', real=True)  
# Mass hinge, leg, Inertia
l1, l2, l = sp.symbols('l1, l2, l', real=True)  # Distances
g = sp.symbols('g', real=True)  # Slope of ramp, gravity
theta1, theta2 = sp.symbols('theta1 theta2', real=True)  # Angles
dtheta1, dtheta2 = sp.symbols('dtheta1 dtheta2', real=True)  # Angular velocity
ddtheta1, ddtheta2 = sp.symbols('ddtheta1 ddtheta2', real=True)  # Angular acceleration

# T
T_a = 1/2 * M1 * l**2 * dtheta1**2
T_b = 1/2 * M2 * l**2 * (dtheta1**2 + dtheta2**2 + 2 * dtheta1 * dtheta2 * sp.cos(theta1 - theta2))
T_c = 1/6 * n * l**2 * dtheta1**2
T_d = 1/6 * n * l**2 * (dtheta1**2 + 1/3 * dtheta2**2 + dtheta1 * dtheta2 * sp.cos(theta1 - theta2))

q = [theta1, theta2]
qdot = [dtheta1, dtheta2]
qddot = [ddtheta1, ddtheta2]

T = T_a + T_b + T_c + T_d 
T = sp.simplify(T)

print(T)
print()

# V
S1, S2 = sp.symbols('S1 S2', real=True)
V = -M1 * g * l * sp.cos(theta1) - g * sp.cos(theta1) * S1 - M2 * g * l * (sp.cos(theta1) + sp.cos(theta2)) - g * sp.cos(theta1) * n - g * sp.cos(theta2) * S2

V = sp.simplify(V)

print(V)
print()

L = T-V
L = sp.simplify(L)

EOM = []
for i in range(2):
    print(i)
    dLdqdot = sp.diff(L, qdot[i])
    ddt_dLdqdot = sum([sp.diff(dLdqdot, q[j]) * qdot[j] + sp.diff(dLdqdot, qdot[j]) * qddot[j] for j in range(2)])
    # sp.diff(dLdqdot, q[j]) * qdot[j], remark -> d Blah / dt = d Blah / dq * dq / dt = d Blah / dq * qdot
    # sp.diff(dLdqdot, qdot[j]) * qddot[j], remark -> d Blah / dt = d Blah / ddq * ddq / dt = d Blah / dq * qddot
    dLdq = sp.diff(L, q[i])
    EOM.append(sp.simplify(ddt_dLdqdot - dLdq))
print("{d/dt dL/dqdot - dL/dq} ACQUIRED")

EOM_vec = sp.simplify(sp.Matrix([EOM[i] for i in range(2)]))

print(EOM_vec[0])
print()
print(EOM_vec[1])
print()

M_ss = EOM_vec.jacobian(qddot)
b_ss = sp.simplify(EOM_vec.subs([(ddtheta1,0), (ddtheta2,0)]))

G = b_ss.subs([(dtheta1, 0), (dtheta2, 0)])
C = b_ss - G

print()
print("===DERIVATION ENDED===")
print("M_ss = ", M_ss)
print()
print("M00 = ", M_ss[0,0])
print("M01 = ", M_ss[0,1])
print("M10 = ", M_ss[1,0])
print("M11 = ", M_ss[1,1])
print()

print("G = ", sp.simplify(G))
print()
print("G0 = ", sp.simplify(G[0]))
print("G1 = ", sp.simplify(G[1]))
print()

print("C = ", sp.simplify(C))
print()
print("C0 = ", sp.simplify(C[0]))
print("C1 = ", sp.simplify(C[1]))
print("=====================")

