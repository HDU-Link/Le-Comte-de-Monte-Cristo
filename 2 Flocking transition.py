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
theta0 = 63.4349/180*np.pi
thetaT = np.pi
v0 = np.sqrt(alpha/beta) * np.array([np.cos(theta0),np.sin(theta0)])
t_span = (0, 200)
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

    for step in range(num_steps):
        speed_sq = np.sum(velocities ** 2, axis=1, keepdims=True)
        self_force = (alpha - beta * speed_sq) * velocities
        theta = theta0 + (thetaT-theta0) * step / num_steps
        R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
        acc = self_force - interaction_force(positions) - M * (velocities - R @ v0)
        velocities = velocities + acc * dt
        positions = positions + velocities * dt
        pos_history.append(positions.copy())
        vel_history.append(velocities.copy())

    return pos_history, vel_history
# ==================== 初始位置和速度 ====================
positions = np.zeros((N, 2))
angles = np.linspace(0, 2 * np.pi, int(N/3))
rand = np.random.rand(int(N/3)) * 0.4
positions[:int(N/3), 0] = 0.45 + np.cos(angles) * rand
positions[:int(N/3), 1] = 0.45 + np.sin(angles) * rand
angles = np.linspace(0, 2 * np.pi, N-int(N/3))
positions[int(N/3):, 0] = 0.45 + np.cos(angles) * 0.4
positions[int(N/3):, 1] = 0.45 + np.sin(angles) * 0.4
velocities = np.ones((N, 2)) * v0
pos0, vel0 = dynamics(positions, velocities, 0.5, int(t_span[1] / 0.5))
positions[:int(N/3)] = [0.45, 0.45] + pos0[-1][:int(N/3)] - np.mean(pos0[-1][int(N/3):],axis=0)
# ==================== 求解 ====================
dt = 0.5
num_steps = int(t_span[1] / dt)
pos_history, vel_history = dynamics(positions, velocities, dt, num_steps)
mean_angles = []
for velocities in vel_history:
    angles = np.arctan2(velocities[:, 1], velocities[:, 0])
    mean_angles.append(np.mean(angles))
# ==================== 绘图 ====================
plt.figure(figsize=(15, 5))
ax1 = plt.subplot(131)
pos = pos_history[0]
vel = vel_history[0]
ax1.scatter(pos[:, 0], pos[:, 1], marker='o', facecolors='none', edgecolors='b', s=24)
ax1.quiver(pos[:, 0], pos[:, 1], vel[:, 0], vel[:, 1], color='#D95319', width=0.005)

ax2 = plt.subplot(132)
pos = pos_history[int(200/dt)]
vel = vel_history[int(200/dt)]
ax2.scatter(pos[:, 0], pos[:, 1], marker='o', facecolors='none', edgecolors='b', s=24)
ax2.quiver(pos[:, 0], pos[:, 1], vel[:, 0], vel[:, 1], color='#D95319', width=0.005)
r = np.ones(num_steps)*np.sqrt(alpha/beta)
theta = np.linspace(theta0, 2*np.pi+mean_angles[-1], num_steps)

ax3 = plt.subplot(133, projection='polar')
ax3.set_rgrids(np.arange(0.0, 1.5, 1.5))
ax3.set_thetagrids(np.arange(0.0, 360.0, 30.0))
ax3.plot(theta, r, linewidth=1)
ax3.grid(True)
plt.tight_layout()
plt.show()
