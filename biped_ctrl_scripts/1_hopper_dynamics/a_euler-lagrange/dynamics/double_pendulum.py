import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
l = 1.0          # length of rods (meters)
M1 = 1.0         # mass 1 (kg)
M2 = 1.0         # mass 2 (kg)
n = 0.2          # additional scalar (e.g., inertia or mass)
S1 = 0.0         # offset for G0
S2 = 0.0         # offset for G1
g = 9.81         # gravity (m/s^2)

# Initial conditions (near upright position, θ=π)
theta1 = np.pi - 0.01
theta2 = np.pi - 0.01
dtheta1 = 0.0
dtheta2 = 0.0
y0 = np.array([theta1, theta2, dtheta1, dtheta2])

# Time settings
dt = 0.01
t_max = 10
steps = int(t_max / dt)
time = np.linspace(0, t_max, steps)
result = np.zeros((steps, 4))
result[0] = y0

# Dynamics functions
def mass_matrix(theta1, theta2):
    m00 = l**2 * (M1 + M2 + (2/3)*n)
    m01 = l**2 * (M2 + (1/6)*n) * np.cos(theta1 - theta2)
    m10 = m01
    m11 = l**2 * (M2 + (1/9)*n)
    return np.array([[m00, m01], [m10, m11]])

def coriolis(theta1, theta2, dtheta1, dtheta2):
    c0 = dtheta2**2 * l**2 * (M2 + 1/6*n) * np.sin(theta1 - theta2)
    c1 = -dtheta1**2 * l**2 * (M2 + 1/6*n) * np.sin(theta1 - theta2)
    return np.array([c0, c1])

def gravity(theta1, theta2):
    g0 = g * (M1*l + M2*l + S1 + n) * np.sin(theta1)
    g1 = g * (M2*l + S2) * np.sin(theta2)
    return np.array([g0, g1])

def derivatives(state):
    theta1, theta2, dtheta1, dtheta2 = state
    dtheta = np.array([dtheta1, dtheta2])
    
    M = mass_matrix(theta1, theta2)
    C = coriolis(theta1, theta2, dtheta1, dtheta2)
    G = gravity(theta1, theta2)
    
    ddtheta = np.linalg.solve(M, -C - G)
    return np.array([dtheta1, dtheta2, ddtheta[0], ddtheta[1]])

def rk4_step(state, dt):
    k1 = derivatives(state)
    k2 = derivatives(state + 0.5 * dt * k1)
    k3 = derivatives(state + 0.5 * dt * k2)
    k4 = derivatives(state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

# RK4 simulation
for i in range(1, steps):
    result[i] = rk4_step(result[i - 1], dt)

# Convert to Cartesian coordinates (θ = 0 is down → y = -cos(θ))
x1 = l * np.sin(result[:, 0])
y1 = -l * np.cos(result[:, 0])
x2 = x1 + l * np.sin(result[:, 1])
y2 = y1 - l * np.cos(result[:, 1])

# Set up the animation
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-2 * l, 2 * l)
ax.set_ylim(-2 * l, 2 * l)
ax.set_aspect('equal')
ax.grid(True)

line, = ax.plot([], [], 'o-', lw=2)
trail, = ax.plot([], [], 'r-', lw=1, alpha=0.6)
trail_points = 300  # how many previous points to show

def init():
    line.set_data([], [])
    trail.set_data([], [])
    return line, trail

def update(i):
    x = [0, x1[i], x2[i]]
    y = [0, y1[i], y2[i]]
    line.set_data(x, y)
    
    # Trail for mass 2
    trail_x = x2[max(0, i - trail_points):i]
    trail_y = y2[max(0, i - trail_points):i]
    trail.set_data(trail_x, trail_y)
    
    return line, trail

ani = FuncAnimation(fig, update, frames=steps, init_func=init,
                    blit=True, interval=dt * 1000)

plt.title('Double Pendulum Animation (θ=0 is Down)')
plt.show()
