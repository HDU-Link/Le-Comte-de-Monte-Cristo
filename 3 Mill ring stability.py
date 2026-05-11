import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import lineStyles
from scipy.spatial.distance import pdist, squareform
plt.rcParams['font.family'] = 'Arial'
plt.rcParams['font.sans-serif'] = ['Arial']
plt.rcParams['font.size'] = 14
# ==================== 参数设置 ====================
a = 4
b = 1
alpha = 10
beta = 3
R = 1.077918
N = 200

# ==================== 相互作用力 ====================
def interaction_force(positions):
    forces = np.zeros((N, 2))
    dists = pdist(positions)
    dists_matrix = squareform(dists)
    for i in range(N):
        for j in range(N):
            r = dists_matrix[i, j]
            if i==j:
                continue
            else:
                forces[i,:] += (positions[i] - positions[j]) * r ** (a-2)
                forces[i,:] -= (positions[i] - positions[j]) * r ** (b-2)
    return forces / N

# ==================== 动力学方程 ====================
def dynamics(positions, velocities, dt, num_steps):
    pos_history = [positions.copy()]
    vel_history = [velocities.copy()]
    max_vel_history = [np.max(np.sqrt(np.sum(velocities ** 2, axis=1)))]
    max_force_history = [np.max(np.sqrt(np.sum(interaction_force(positions) ** 2, axis=1)))]

    for step in range(num_steps):
        speed_sq = np.sum(velocities ** 2, axis=1, keepdims=True)
        self_force = (alpha - beta * speed_sq) * velocities
        acc = self_force - interaction_force(positions)
        velocities = velocities + acc * dt
        positions = positions + velocities * dt
        pos_history.append(positions.copy())
        vel_history.append(velocities.copy())
        speeds = np.sqrt(np.sum(velocities ** 2, axis=1))
        forces = np.sqrt(np.sum(interaction_force(positions) ** 2, axis=1))
        max_vel_history.append(np.max(speeds))
        max_force_history.append(np.max(forces))

    return pos_history, vel_history, max_vel_history, max_force_history

def dynamics_copy(positions, velocities, dt, num_steps):
    pos_history = [positions.copy()]

    for step in range(num_steps):
        speed_sq = np.sum(velocities ** 2, axis=1, keepdims=True)
        self_force = (alpha - beta * speed_sq) * velocities
        acc = self_force - interaction_force(positions)
        velocities = velocities + acc * dt
        positions = positions + velocities * dt
        pos_history.append(positions.copy())

    return pos_history
# ==================== 初始位置和速度 ====================
angles = np.linspace(0, 2 * np.pi, N)
positions = np.column_stack([np.cos(angles), np.sin(angles)]) / 4 * R
velocities = np.column_stack([np.cos(angles), np.sin(angles)])
# ==================== 求解 ====================
dt = 0.01
pos_history, vel_history, max_vel_history, max_force_history = dynamics(positions, velocities, dt, int(50 / dt))
velocities2 = np.column_stack([-np.sin(angles), np.cos(angles)])
pos1_history= dynamics_copy(positions * 8, velocities2, dt, int(120 / dt))
pos2_history= dynamics_copy(positions * 16, velocities2, dt, int(120 / dt))
velocities3 = np.column_stack([-np.sin(angles + np.pi / 6), np.cos(angles + np.pi / 6)])
velocities4 = np.column_stack([-np.sin(angles + np.pi / 2 * 0.99), np.cos(angles + np.pi / 2 * 0.99)])
pos3_history = dynamics_copy(positions * 4, velocities3, dt, int(120 / dt))
pos4_history = dynamics_copy(positions * 4, velocities4, dt, int(120 / dt))
# ==================== 绘图 ====================
fig = plt.figure(figsize=(10, 8))
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
t_steps = np.arange(0, int(30/dt)) * dt
ax1.quiver3D(positions[:, 0], positions[:, 1], np.zeros(N),
            velocities[:, 0], velocities[:, 1], np.zeros(N), color='r', length=0.05, linewidth=0.5)
ax1.quiver3D(pos_history[-1][:, 0], pos_history[-1][:, 1], np.ones(N)*30,
           vel_history[-1][:, 0], vel_history[-1][:, 1], np.zeros(N)*30, color='k', length=0.05, linewidth=0.5)
x_traj = [pos_history[step][0, 0] for step in range(0, int(30/dt), 10)]
y_traj = [pos_history[step][0, 1] for step in range(0, int(30/dt), 10)]
z_traj = t_steps[::10]
ax1.plot3D(x_traj, y_traj, z_traj, color='k', linewidth=0.5, linestyle=':')
ax1.set_xlabel('$x$')
ax1.set_ylabel('$y$')
ax1.set_zlabel('Time')
ax1.xaxis.pane.set_alpha(0)
ax1.yaxis.pane.set_alpha(0)
ax1.zaxis.pane.set_alpha(0)
ax1.set_facecolor('none')
ax1.grid(False)
ax1.view_init(elev=45, azim=45)

ax2 = fig.add_subplot(2, 2, 2)
time_array = np.arange(0, 50 + dt, dt)
ax2.set_yscale('log')
ax2.plot(time_array[:int(50/dt)], max_vel_history[:int(50/dt)], label='$||v(x)||$')
ax2.plot(time_array[:int(50/dt)], max_force_history[:int(50/dt)],
         label='$||F(x)||$', color='#D95319', linestyle='--')
ax2.set_xlabel('Time')
ax2.legend()
ax2.grid(True)

ax3 = fig.add_subplot(2, 2, 3)
radius1 = np.zeros(int(120/dt) + 1)
radius2 = np.zeros(int(120/dt) + 1)
for t_idx in np.arange(int(120/dt) + 1):
    xm1 = pos1_history[t_idx].mean(axis=0)
    xm2 = pos2_history[t_idx].mean(axis=0)
    radius1[t_idx] = np.abs(np.mean(np.linalg.norm(pos1_history[t_idx] - xm1, axis=1)) - R)
    radius2[t_idx] = np.abs(np.mean(np.linalg.norm(pos2_history[t_idx] - xm2, axis=1)) - R)
ax3.plot(np.arange(0, 120 + dt, dt), radius1, label="$R_0=2R$")
ax3.plot(np.arange(0, 120 + dt, dt), radius2, color='orange', label="$R_0=4R$", linestyle=':')
ax3.set_xlabel('Time')
ax3.set_yscale('log')
ax3.legend()
ax3.grid(True)

ax4 = fig.add_subplot(2, 2, 4)
gamma1 = np.zeros(int(120/dt) + 1)
gamma2 = np.zeros(int(120/dt) + 1)
for t_idx in np.arange(int(120/dt) + 1):
    xm3 = pos3_history[t_idx].mean(axis=0)
    xm4 = pos4_history[t_idx].mean(axis=0)
    gamma1[t_idx] = np.abs(np.mean(np.linalg.norm(pos3_history[t_idx] - xm3, axis=1)) - R)
    gamma2[t_idx] = np.abs(np.mean(np.linalg.norm(pos4_history[t_idx] - xm4, axis=1)) - R)
ax4.plot(np.arange(0, 120 + dt, dt), gamma1, label=r"$\gamma_0=\frac{\pi}{6}$")
ax4.plot(np.arange(0, 120 + dt, dt), gamma2, color='r', label=r"$\gamma_0=0.99\frac{\pi}{2}$", linestyle='--')
ax4.set_xlabel('Time')
ax4.set_yscale('log')
ax4.legend()
ax4.grid(True)
plt.tight_layout()
plt.show()
