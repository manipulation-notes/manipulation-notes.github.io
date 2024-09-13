---
layout: default
math: true
---

# Dynamical Movement Primitives (DMPs)

Dynamical Movement Primitives (DMPs) are a powerful framework for representing and 
learning complex motor behaviors in robotics. Originally developed for learning and 
generalizing human motion, DMPs are particularly useful in robotic systems where 
adaptability and robustness to perturbations are required. In this section, we introduce 
DMPs first in the context of point robots and explain the underlying principles. Then,
we'll explore the generalization of DMPs to general articulated serial-link robots.

## Building Intuition with Point Robots

A point robot is a simple representation of a robot where the configuration is specified 
by a position $\mathbf{x}(t) \in \mathbb{R}^n$, where $n$ is the dimension of the space 
(e.g., $n=2$ for a planar point robot). The goal of a DMP is to generate smooth trajectories 
for the robot to follow from its current position $\mathbf{x}$ to a goal position 
$\mathbf{x}_g$ while allowing for flexibility and generalization in the movement.

To fire up your intuition, consider a point robot in 2D. The governing equation of motion is:

$$
    \mathbf{f} = m \ddot{\mathbf{x}}
$$

where $\mathbf{f}$ represents the force we choose to apply to the robot and $\ddot{\mathbf{x}}$
represents the acceleration of the robot. Now imagine that the point robot is attached to its goal through a
ficticous spring and damper. The forces corresponding to these to fictional elements are:

$$
\begin{align}
    \mathbf{f}_s & = k_s(\mathbf{x}_g - \mathbf{x}) \\
    \mathbf{f}_d & = k_d(\dot{\mathbf{x}}_g - \dot{\mathbf{x}})
\end{align}
$$

The effect of this fictious elements are to pull the robot towards the goal. We can see this
by writing down the equations of motion with the force from these elements:

$$
\begin{align}
    & \mathbf{f} = k_s(\mathbf{x}_g - \mathbf{x}) +  k_d(\dot{\mathbf{x}}_g - \dot{\mathbf{x}}) = m \ddot{\mathbf{x}} \\
    & m \ddot{\mathbf{x}} + k_d (\dot{\mathbf{x}} - \dot{\mathbf{x}}_g) + k_s(\mathbf{x} - \mathbf{x}_g ) = 0
\end{align}
$$

Now lets assume that the goal velocity is zero $\dot{\mathbf{x}}_g = 0$, then we can simplify the expression:

$$
\begin{align}
    m \ddot{\mathbf{x}} + k_d \dot{\mathbf{x}} + k_s(\mathbf{x} - \mathbf{x}_g ) = 0
\end{align}
$$

The resultng autonomous dynamical system is guaranteed to converge to $\mathbf{x}_g$ from any point in $\mathbf{x}$.
This convergence is asymptotic as long as $k_d \geq 0$ and $k_s \geq 0$ and the robot has a positive mass.
Further, even if we move $\mathbf{x}_g$, the system will actively track this point. The importance of this result
cannot be understated, by simply choosing a spring and damper control law, we can drive our system to any
desired goal state from any initial state. We note that with our current formulation, the path towards the goal
is always a straight line, and the speed at which we travel along this path is determined by the spring and
damper coefficients. It is entirely possible to design under-damped (over-shooting) or over-damped (no overshoot)
dynamical systems depending on our needs.

**how DMPs address these limitations by given us more flexibility over path traveled and speed along the path**

$$
    \mathbf{x} = \mathbf{x}
$$

### Canonical System

The key idea behind DMPs is to separate the time evolution of the system from the 
spatial component. To achieve this, DMPs introduce a _canonical system_ that acts as 
a phase variable, $s(t) \in [0,1]$, which monotonically decays over time. The canonical 
system is governed by the differential equation:

$$
\begin{equation}
    \dot{s}(t) = -\alpha_s s(t),
\end{equation}
$$

where $\alpha_s$ is a positive constant that controls the rate of decay. The canonical 
system ensures that the DMP formulation is time-invariant and can scale to different time durations 
for the task.

## Transformation System

The spatial part of the movement is handled by the \emph{transformation system}, which 
defines the desired trajectory for the point robot. The transformation system is defined 
by a second-order dynamical system:

$$
\begin{equation}
    \tau \ddot{\mathbf{x}}(t) = \alpha_x \left( \beta_x (\mathbf{x}_g - \mathbf{x}(t)) - \dot{\mathbf{x}}(t) \right) + f(s(t)),
\end{equation}
$$

where $\tau$ is a time-scaling constant, $\alpha_x$ and $\beta_x$ are positive constants 
that determine the stiffness and damping of the system, and $f(s(t))$ is a nonlinear forcing 
term that modulates the trajectory to achieve complex movements.

## Forcing Term

The forcing term $f(s(t))$ is a key component of the DMP framework, as it allows for the 
encoding of arbitrary movements. This term is typically represented as a linear combination 
of Gaussian basis functions:

$$
\begin{equation}
    f(s(t)) = \frac{\sum_{i=1}^N \psi_i(s) w_i}{\sum_{i=1}^N \psi_i(s)} s(t),
\end{equation}
$$

where $\psi_i(s) = \exp(-h_i (s - c_i)^2)$ are Gaussian kernels centered at $c_i$ with 
widths $h_i$, and $w_i$ are the corresponding weights. These weights are learned from 
demonstration data, allowing the DMP to replicate complex motions while maintaining a 
flexible and robust structure.

## Summary of DMP Properties

DMPs offer several desirable properties for motion generation in robotics:

- **Generalization:** DMPs can generalize a learned movement to different start and goal positions by adjusting $\mathbf{x}_0$ and $\mathbf{x}_g$.
- **Robustness:** DMPs naturally handle perturbations by following a stable attractor towards the goal position.
- **Time Invariance:** The separation of the canonical and transformation systems allows the same trajectory to be executed over different time scales.
- **Learning from Demonstration:** DMPs can be efficiently learned from a small number of demonstrations, making them practical for robot learning tasks.


In the next section, we will discuss how to implement DMPs for point robots in practice, and demonstrate their use in trajectory generation for different types of tasks.
