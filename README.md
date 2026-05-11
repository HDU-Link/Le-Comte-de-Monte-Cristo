# Controlling Swarms Toward Flocks and Mills

## Repository Overview

This repository contains Python implementations of the control strategies presented in the paper **"Controlling swarms toward flocks and mills"** by José A. Carrillo, Dante Kalise, Francesco Rossi, and Emmanuel Trélat (SIAM Journal on Applied Mathematics, 2022).

The paper demonstrates how constrained feedback controls can steer self-propelled particle swarms from arbitrary initial configurations to desired collective behaviors, including **flocks** (collective motion with aligned velocities) and **mills** (rotating ring formations). The implementations reproduce the five key numerical experiments from the paper, showcasing different control strategies:

1. **Jurdjevic-Quinn stabilization** - Steering arbitrary initial configurations to a neighborhood of equilibrium
2. **Quasi-static deformation** - Smoothly transitioning between flocks with different directions
3. **Mill ring stability analysis** - Studying the stability of rotating ring formations
4. **Feedback stabilization to mill rings** - Using optimal instantaneous control to achieve milling behavior
5. **Controlled transition from mill to flock** - Switching between different collective regimes

## Mathematical Model

The system consists of \( N \) agents governed by second-order dynamics:

\[
\begin{aligned}
\dot{x}_i(t) &= v_i(t) \\
\dot{v}_i(t) &= (\alpha - \beta |v_i(t)|^2)v_i(t) - \frac{1}{N}\sum_{j=1}^{N}\nabla W(x_i(t) - x_j(t)) + u_i(t)
\end{aligned}
\]

where:
- \( x_i, v_i \in \mathbb{R}^2 \) are position and velocity of agent \( i \)
- \( \alpha, \beta > 0 \) control self-propulsion (agents tend toward speed \( \sqrt{\alpha/\beta} \))
- \( W(x) = U(|x|) \) is a radial interaction potential (attractive-repulsive)
- \( u_i(t) \) is the control input with constraint \( \|u\| \leq M \)

## Repository Structure

```
├── 1_Jurdjevic-Quinn_feedback_control.py    # Figure 1: JQ stabilization
├── 2_Flocking_transition.py                 # Figure 2: Flock direction change
├── 3_Mill_ring_stability.py                 # Figure 3: Mill ring stability analysis
├── 4_Stabilization_towards_mill_ring.py     # Figure 4: Controlled mill formation
├── 5_Mill_to_flock.py                       # Figure 5: Mill to flock transition
└── README.md
```

## Code Descriptions

### 1. Jurdjevic-Quinn Feedback Control (`1_Jurdjevic-Quinn_feedback_control.py`)

**Corresponds to:** Figure 1 in the paper

This script implements the Jurdjevic-Quinn stabilization strategy (Step 1.1 in the paper) using a Lyapunov-based feedback control to steer the system toward \( \Omega_\epsilon = \{(x,v): v=0, \|F(x)\| \leq \epsilon\} \).

**Key features:**
- Quasi-Morse potential: \( W(x) = -e^{-|x|^p/p} + C e^{-|x/l|^p/p} \)
- Piecewise feedback control based on velocity magnitude
- Compares uncontrolled vs. controlled evolution
- Demonstrates convergence of both velocities and interaction forces to near-zero values

**Control law:**
\[
u_i(v_i) = 
\begin{cases}
0 & |v_i| \geq 2\gamma\sqrt{\alpha/\beta} \\
-M\frac{v_i}{|v_i|}\left(2 - \frac{|v_i|}{\gamma\sqrt{\alpha/\beta}}\right) & |v_i| \in (\gamma\sqrt{\alpha/\beta}, 2\gamma\sqrt{\alpha/\beta}) \\
-M\frac{v_i}{|v_i|} & |v_i| \in [\frac{1}{\gamma}\sqrt{\alpha/\beta}, \gamma\sqrt{\alpha/\beta}] \\
-M\frac{\gamma}{\sqrt{\alpha/\beta}} v_i & |v_i| < \frac{1}{\gamma}\sqrt{\alpha/\beta}
\end{cases}
\]

### 2. Flocking Transition (`2_Flocking_transition.py`)

**Corresponds to:** Figure 2 in the paper

This script demonstrates quasi-static deformation (Step 3.2 in the paper) to smoothly transition a flock from one direction to another.

**Key features:**
- Time-varying linear feedback control: \( u_i(v_i,t) = -M(v_i - \mathcal{R}_{\theta(t)}v_0) \)
- Rotation matrix \( \mathcal{R}_{\theta(t)} \) interpolates between initial and target directions
- Maintains constant speed \( \sqrt{\alpha/\beta} \) throughout the transition
- Polar plot shows the smooth angular evolution of the flock

### 3. Mill Ring Stability (`3_Mill_ring_stability.py`)

**Corresponds to:** Figure 3 in the paper

This script analyzes the stability of mill ring solutions for power-law potentials: \( U(s) = |s|^a/a - |s|^b/b \).

**Key features:**
- Power-law potential with \( a=4, b=1 \)
- 3D visualization of a single agent's trajectory converging to the mill
- Stability analysis for different initial radii (\( R_0 = 2R, 4R \))
- Stability analysis for different initial velocity angle offsets (\( \gamma_0 = \pi/6, 0.99\pi/2 \))

**Mill radius condition:**
\[
\sum_{p=1}^{N-1} \sin\left(\frac{p\pi}{N}\right) \tilde{U}'\left(2R\sin\left(\frac{p\pi}{N}\right)\right) = 0, \quad \tilde{U}(r) = U(r) - \omega^2\frac{r^2}{2}
\]

### 4. Stabilization Toward Mill Ring (`4_Stabilization_towards_mill_ring.py`)

**Corresponds to:** Figure 4 in the paper

This script implements an instantaneous optimal feedback control to stabilize an arbitrary configuration toward a mill ring.

**Key features:**
- Model Predictive Control (MPC)-style optimization over a short horizon \( \Delta t \)
- Single control signal applied to all agents via incremental rotation
- Objective function balances:
  - Distance to desired mill radius \( R_m \)
  - Alignment with tangential velocity \( \sqrt{\alpha/\beta} \, x_i^\perp/|x_i| \)
  - \( \ell_1 \) and \( \ell_2 \) control penalties for sparsity
- Compares controlled vs. uncontrolled evolution

**Optimization objective:**
\[
\min_u \sum_{i=1}^N \left|v_i - \sqrt{\frac{\alpha}{\beta}}\frac{x_i^\perp}{|x_i|}\right| + (|x_i - x_m|^2 - R_m^2)^2 + \lambda_1|u| + \lambda_2|u|^2
\]

### 5. Mill to Flock Transition (`5_Mill_to_flock.py`)

**Corresponds to:** Figure 5 in the paper

This script demonstrates controlled transition from a stable mill configuration to a flocking configuration.

**Key features:**
- Starts from a perfect mill ring (radial positions, tangential velocities)
- Uses optimal instantaneous feedback to steer toward flocking regime
- Target: constant velocity \( \bar{v} = (\sqrt{\alpha/\beta}, 0) \) and desired flock radius \( R_f \)
- Shows evolution of swarm radius from mill radius to flock radius

## Dependencies

```bash
pip install numpy scipy matplotlib
```

## Usage

Run any script individually:

```bash
python 1_Jurdjevic-Quinn_feedback_control.py
python 2_Flocking_transition.py
python 3_Mill_ring_stability.py
python 4_Stabilization_towards_mill_ring.py
python 5_Mill_to_flock.py
```

## Key Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| \( \alpha \) | Self-propulsion coefficient | 2.0 - 10.0 |
| \( \beta \) | Friction/damping coefficient | 1.5 - 3.0 |
| \( M \) | Control bound | > \( \sqrt{4\alpha^3/27\beta} \) |
| \( N \) | Number of agents | 20 - 200 |
| \( C, p, l \) | Quasi-Morse potential parameters | 0.6, 1.5, 0.5 |
| \( a, b \) | Power-law potential exponents | 4, 1 |

## Control Strategies Summary

| Strategy | Purpose | Key Technique |
|----------|---------|---------------|
| Jurdjevic-Quinn | Reach \( \Omega_\epsilon \) | Lyapunov-based feedback |
| Local controllability | Fine maneuvering near equilibrium | Control linearization |
| Quasi-static deformation | Transition between flocks | Slowly varying path tracking |
| Instantaneous optimal | Stabilize to mill | Short-horizon optimization |

## Theoretical Guarantees

From the paper, with sufficient control bound \( M \):

- **If \( M > M_{\alpha,\beta} \)**: System can be steered to any flock configuration
- **If \( M > \max(M_{\alpha,\beta}, M_F) \)**: System can be steered to any flock or mill configuration

where \( M_{\alpha,\beta} = \sqrt{4\alpha^3/27\beta} \) and \( M_F = \sup_{r>0}|U'(r)| \).

## References

- Carrillo, J. A., Kalise, D., Rossi, F., & Trélat, E. (2022). Controlling swarms toward flocks and mills. *SIAM Journal on Control and Optimization*, 60(3), 1863-1891. [DOI: 10.1137/21M1404314](https://doi.org/10.1137/21M1404314)
