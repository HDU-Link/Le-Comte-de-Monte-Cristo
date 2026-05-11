import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist, squareform
plt.rcParams['font.family'] = 'Arial'
plt.rcParams['font.sans-serif'] = ['Arial']
plt.rcParams['font.size'] = 14
# ==================== 参数设置 ====================
C = 0.6
p = 1.5
l = 0.5
alpha = 2.0
beta = 1.5
M = max(np.sqrt(4/27*alpha**3/beta), 1)
N = 200
gamma = max(1, 1/M * np.sqrt(alpha**3/beta)) + 1
t_span = (0, 2000)
np.random.seed(42)

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
                forces[i,:] += (positions[i] - positions[j]) / r * r ** (p-1) * np.exp(-(r**p) / p)
                forces[i,:] -= C*(positions[i] - positions[j]) / r * (r/l) ** (p-1) * np.exp(-((r/l)**p) / p) / l
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

def JQ_dynamics(positions, velocities, dt, num_steps):
    pos_history = [positions.copy()]
    vel_history = [velocities.copy()]
    max_vel_history = [np.max(np.sqrt(np.sum(velocities ** 2, axis=1)))]
    max_force_history = [np.max(np.sqrt(np.sum(interaction_force(positions) ** 2, axis=1)))]

    for step in range(num_steps):
        speed = np.sqrt(np.sum(velocities ** 2, axis=1))
        speed_sq = np.sum(velocities ** 2, axis=1, keepdims=True)
        self_force = (alpha - beta * speed_sq) * velocities
        acc = self_force - interaction_force(positions)
        for i in range(N):
            if speed[i] < np.sqrt(alpha/beta) / gamma:
                acc[i] -= M * velocities[i] * gamma / np.sqrt(alpha/beta)
            elif speed[i] < np.sqrt(alpha/beta) * gamma:
                acc[i] -= M * velocities[i] / speed[i]
            elif speed[i] < 2 * np.sqrt(alpha/beta) * gamma:
                acc[i] -= M * velocities[i] / speed[i] * (2-speed[i]/np.sqrt(alpha/beta)/gamma)
        velocities = velocities + acc * dt
        positions = positions + velocities * dt
        pos_history.append(positions.copy())
        vel_history.append(velocities.copy())
        force = np.sqrt(np.sum(interaction_force(positions) ** 2, axis=1))
        max_vel_history.append(np.max(speed))
        max_force_history.append(np.max(force))

    return pos_history, vel_history, max_vel_history, max_force_history
# ==================== 初始位置和速度 ====================
positions = np.random.rand(N, 2)
velocities = np.zeros((N, 2))
Rand = np.random.rand(N) / 20
angles = 2 * np.pi * np.random.rand(N)
velocities[:, 0] = np.cos(angles) * Rand
velocities[:, 1] = np.sin(angles) * Rand

# ==================== 求解 ====================
dt = 0.5
num_steps = int(t_span[1] / dt)
pos_history, vel_history, max_vel_history, max_force_history = dynamics(positions, velocities, dt, num_steps)
pos_JQ, vel_JQ, max_vel_JQ, max_force_JQ = JQ_dynamics(positions, velocities, dt, num_steps)
# ==================== 绘图 ====================
fig, axes = plt.subplots(3, 3, figsize=(15, 12))
axes[0,0].scatter(positions[:, 0], positions[:, 1], marker='o', facecolors='none', edgecolors='b', s=24)
# axes[0,0].quiver(positions[:, 0], positions[:, 1], velocities[:, 0], velocities[:, 1], color='red', width=0.005)
# axes[0,0].set_title(f't = 0s')
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
time_array = np.arange(0, t_span[1] + dt, dt)
axes[0,1].set_yscale('log')
axes[0,1].plot(time_array, max_vel_history, label='$||v(x)||$')
axes[0,1].plot(time_array, max_force_history, label='$||F(x)||$', color='#D95319')
axes[0,1].set_xlabel('Time')
axes[0,1].legend()
axes[0,1].grid(True)
axes[0,2].set_yscale('log')
axes[0,2].plot(time_array, max_vel_JQ, label='$||v(x)||$')
axes[0,2].plot(time_array, max_force_JQ, label='$||F(x)||$', color='#D95319', linestyle='--')
axes[0,2].set_xlabel('Time')
axes[0,2].legend()
axes[0,2].grid(True)

timestamps = [4, 20, 200]
indices = [int(t / dt) for t in timestamps]
for idx, t in enumerate(timestamps):
    ax = axes[1, idx]
    pos = pos_history[indices[idx]]
    vel = vel_history[indices[idx]]
    ax.scatter(pos[:, 0], pos[:, 1], marker='o', facecolors='none', edgecolors='b', s=24)
    ax.quiver(pos[:, 0], pos[:, 1], vel[:, 0], vel[:, 1], color='#D95319', width=0.005)
for idx, t in enumerate(timestamps):
    ax = axes[2, idx]
    pos = pos_JQ[indices[idx]]
    vel = vel_JQ[indices[idx]]
    ax.scatter(pos[:, 0], pos[:, 1], marker='o', facecolors='none', edgecolors='b', s=24)
    ax.quiver(pos[:, 0], pos[:, 1], vel[:, 0], vel[:, 1], color='#D95319', width=0.005)
    # ax.set_title(f't = {int(t)}s')

plt.tight_layout()
plt.show()
