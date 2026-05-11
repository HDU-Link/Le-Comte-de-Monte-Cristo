import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.spatial.distance import pdist, squareform
plt.rcParams['font.family'] = 'Arial'
plt.rcParams['font.sans-serif'] = ['Arial']
plt.rcParams['font.size'] = 14
# ==================== 参数设置 ====================
a = 4
b = 1
alpha = 10
beta = 3
lambda1 = 1e-3
lambda2 = 1e-3
N = 20
t_span = (0, 60)
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
                forces[i,:] += (positions[i] - positions[j]) * r ** (a-2)
                forces[i,:] -= (positions[i] - positions[j]) * r ** (b-2)
    return forces / N

# ==================== 动力学方程 ====================
def dynamics(positions, velocities, dt, num_steps):
    pos_history = [positions.copy()]
    vel_history = [velocities.copy()]
    u_history = []
    for step in range(num_steps):
        speed_sq = np.sum(velocities ** 2, axis=1, keepdims=True)
        self_force = (alpha - beta * speed_sq) * velocities
        acc = self_force - interaction_force(positions)
        def obj(x):
            u = np.ones((N, 1)) * x
            v = velocities + u * dt
            p = positions + v * dt
            p_mean = p.mean(axis=0)
            obj = lambda1 * np.linalg.norm(x) * N
            obj += lambda2 * np.sum(x ** 2) * N
            obj += np.sum((np.sum((p - p_mean) ** 2,axis=1, keepdims=True) - 1.075 ** 2) ** 2)
            p_norm = np.linalg.norm(p, axis=1, keepdims=True) ** 2
            v_desired = np.sqrt(alpha / beta) * np.column_stack([-p[:, 1] / p_norm[:, 0], p[:, 0] / p_norm[:, 0]])
            obj += np.sum(np.linalg.norm(v - v_desired, axis=1))
            return obj
        res = minimize(obj, np.zeros(2), bounds=([-1, 1], [-1, 1]))
        u = res.x
        theta = np.linspace(0, 2 * np.pi, N, endpoint=False)
        acc[:,0] += u[0] * np.cos(theta) - u[1] * np.sin(theta)
        acc[:,1] += u[0] * np.sin(theta) + u[1] * np.cos(theta)
        velocities = velocities + acc * dt
        positions = positions + velocities * dt
        pos_history.append(positions.copy())
        vel_history.append(velocities.copy())
        u_history.append(u)

    return pos_history, vel_history, u_history

def un_dynamics(positions, velocities, dt, num_steps):
    pos_history = [positions.copy()]
    vel_history = [velocities.copy()]
    for step in range(num_steps):
        speed_sq = np.sum(velocities ** 2, axis=1, keepdims=True)
        self_force = (alpha - beta * speed_sq) * velocities
        acc = self_force - interaction_force(positions)
        velocities = velocities + acc * dt
        positions = positions + velocities * dt
        pos_history.append(positions.copy())
        vel_history.append(velocities.copy())

    return pos_history, vel_history
# ==================== 初始位置和速度 ====================
positions = np.random.rand(N, 2)*3-1
velocities = positions * np.random.rand(N, 2)

# ==================== 求解 ====================
dt = 0.01
num_steps = int(t_span[1] / dt)
pos_history, vel_history , u_history = dynamics(positions, velocities, dt, num_steps)
un_pos_history, un_vel_history = un_dynamics(positions, velocities, dt, num_steps)
# ==================== 绘图 ====================
plt.figure(figsize=(15, 8))
timestamps = [0, 10, 40]
indices = [int(t / dt) for t in timestamps]
for idx, t in enumerate(timestamps):
    ax = plt.subplot(2,3,idx+1)
    pos = pos_history[indices[idx]]
    vel = vel_history[indices[idx]]
    ax.scatter(pos[:, 0], pos[:, 1], marker='o', facecolors='none', edgecolors='b', s=24)
    ax.quiver(pos[:, 0], pos[:, 1], vel[:, 0], vel[:, 1], color='#D95319', width=0.005)
    # ax.set_title(f't = {int(t)}s')

plt.rc('text', usetex=True)
plt.rc('font', family='serif')
ax = plt.subplot(2,2,3)
t_history = np.linspace(0, 60, num_steps+1)
config_radius = np.zeros_like(t_history)
un_config_radius = np.zeros_like(t_history)
for t_idx in np.arange(num_steps+1):
    xm = pos_history[t_idx].mean(axis=0)
    config_radius[t_idx] = np.mean(np.linalg.norm(pos_history[t_idx] - xm, axis=1))
    un_xm = un_pos_history[t_idx].mean(axis=0)
    un_config_radius[t_idx] = np.mean(np.linalg.norm(un_pos_history[t_idx] - un_xm, axis=1))
ax.plot(t_history, un_config_radius, color='k', linestyle='-.', label="Uncontrolled")
ax.plot(t_history, config_radius, color='r', label="Controlled")
ax.plot(t_history, un_config_radius[-1]*np.ones_like(t_history), color='k', label="Mill Radius", linestyle='--')
ax.set_xlabel("Time")
ax.legend()
ax.grid()

ax = plt.subplot(2,2,4)
t_history = np.linspace(0, 60, num_steps)
ax.plot(t_history, np.array(u_history)[:, 0], label='$u^1(t)$', linestyle='--')
ax.plot(t_history, np.array(u_history)[:, 1], label='$u^2(t)$', color='#D95319')
ax.set_xlabel("Time")
ax.legend()
ax.grid()
plt.tight_layout()
plt.show()
