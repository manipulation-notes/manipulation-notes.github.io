---
layout: default
math: true
---

# Planning Through Contact:

### Balancing and Pivoting

#### Balancing in the Plane:

Consider the block visualized in \cref{fig:balance}. Our goal is to balance 
the block on the corner which means that it maintains: a) constant $\theta$ 
angle with respect to the horizontal, and b) sticking contact at the corner 
(which means zero relative velocity between the object and table). This problem 
is static and inertial forces play no role. Our decision variable is the __wrench__ 
applied to the center of mass. We will discuss how this wrench can be supplied 
via an external contact (e.g., exerted by a finger) in following subsections.

\begin{figure}[h!]
    \centering
    \includegraphics[width=0.7\textwidth]{figures/pivoting.png}
    \caption{Figure of a block balancing in static equilibrium subject to the body wrench $\mathbf w$ and frictional contact.}
    \label{fig:balance}
\end{figure}

One approach to formalizing balancing the block in 2D is choosing a "minimal" wrench 
value (in the 2-norm sense) by solving the following optimization program:

$$
\begin{align}
    \begin{aligned}
    \min_{\mathbf w \in \mathbb{R}^3, \mathbf f_c \in \mathbb{R}^2} \quad & \frac{1}{2} \mathbf w^{T} \mathbf Q_w \mathbf w + \frac{1}{2} \mathbf f^T_c \mathbf Q_f \mathbf f_c &  \\
    \textrm{s.t.} \quad &  \mathbf J_c \mathbf f_c + \mathbf w + m\mathbf g= 0 & \text{Static Equilibrium} \\
    &0 \leq f_{c,n} & \text{Contacts can only push} \\
    &0 \leq \mu f_{c,n} - f_{c,t} \quad \quad  & \text{Right Friction Cone} \\
    &0 \leq \mu f_{c,n} + f_{c,t} \quad \quad  & \text{Left Friction Cone}
\end{aligned}
\end{align}
$$

where $\mathbf Q_w$ and $\mathbf Q_f$ are both symmetric positive definite matrices 
of appropriate size, $\mathbf f_c = (f_{c,t}, f_{c,n})$, $\mathbf g \in \mathbb{R}^3$ is the 
gravitational acceleration (a linear acceleration applied to the center of mass with 
no torsional contribution), and $\mathbf J_c$ is the configuration dependent $3\times 2$ 
Jacobian matrix. The Jacobian is a known and constant matrix given the object configuration. 
We can formulate this into a standard QP form by defining:

$$
\begin{align}
    \mathbf x_{5\times 1} & = (\mathbf w^T, \; \mathbf f^T_c)^T\\
    \mathbf Q_{5\times 5} & = \text{diag}\{\mathbf Q_w, \; \mathbf Q_f \} \\
    \mathbf A_{3\times 5} & = \begin{bmatrix} \mathbf J_c & \mathbf I_{3\times 3} \end{bmatrix} \\
    \mathbf b_{3\times 1} & = - m \mathbf g \\
    \mathbf C_{3\times 5} & = -\begin{bmatrix} 0 & 1 & 0 & 0& 0 \\ -1 & \mu & 0 &  0 & 0 \\ 1 & \mu & 0 &  0 & 0 \end{bmatrix} \\ 
    \mathbf d_{3\times 1} & = (0, 0, 0)^T
\end{align}
$$

and writing:

$$
\begin{align}
    \begin{aligned}
        \min_{\mathbf x \in \mathbb{R}^5} \quad & \frac{1}{2} \mathbf x^{T} \mathbf Q \mathbf x &  \\
    \textrm{s.t.} \quad &  \mathbf A \mathbf x = \mathbf b & \\
    & \mathbf C \mathbf x \leq \mathbf d = \mathbf 0  &
    \end{aligned}
\end{align}
$$

If gravity did not exist (i.e., we were trying to pivot in the plane orthogonal 
to the gravitational vector), then by carefully inspecting the constraints to 
this quadratic program, we note that the optimal solution $\mathbf x^*$ must belong 
to the null space of $\mathbf A$ which comes from the force balance constraint. 
Intuitively, this means we can scale the solution uniformly and it would still 
satisfy the constraints. This is equivalent to saying if we increase the external 
wrench applied to the object, the frictional force provided by the table will also 
increase proportionally to cancel it out. However, we do note that this scaling 
effect will increase the loss and so the optimal choice would be to let the object 
rest against the support surface but to not apply any force. Since there are no 
other forces (again we are in the plane orthogonal to gravity) then the object 
would stay still and be "pivoting".

The more interesting case is when gravity does exist -- we can illustrate what 
the solution space looks like by visualization the composite wrench cone due to 
frictional contact and the gravity force in the object frame. \Cref{fig:sol-bal} shows 
the composite wrench cone due to friction in orange. Any wrench on the "Cancel Gravity'' 
plane will balance out the effect of gravity. This plane is characterized by the 
gravity vector and its length. The intersection between this plane and the wrench 
cone is the admissible set (satisfying the balance constraints). The smallest wrench 
lies at the closest point to the origin at the intersection between the plane and the 
cone. The composite wrench cone is open because $\mathbf C$ defines an __open__ cone 
over the admissible contact reaction forces which allows for the arbitrary scaling up 
of the reaction force to accommodate the scaling of the body wrench. 

\begin{figure}[h!]
    \centering
    \includegraphics[width=0.95\textwidth]{figures/balance-solution.png}
    \caption{Visualization of Static Balance Solution Space.}
    \label{fig:sol-bal}
\end{figure}

#### Worked Example

Let's assume $\theta = 30^o$, the block is a square of side length $a=1$, its 
mass is $m=1$, and the gravitational acceleration constant is 10 $m/s^2$, the friction 
coefficient $\mu=0.3$, and use identity matrices for $\mathbf Q_w$ and $\mathbf Q_f$. We will 
first calculate the contact Jacobian $\mathbf J_c$:

$$
\begin{align}
    \mathbf J_c & = \begin{bmatrix} \mathbf I_{2\times 2} \\ \mathbf r^\wedge \end{bmatrix} \mathbf R(\theta) \\
    \mathbf r^\wedge & = \begin{bmatrix} r_y & -r_x \end{bmatrix}
\end{align}
$$

where $\mathbf r = (r_x, r_y) = -\frac{a}{2}(1, 1)$ is defined in the local object 
frame and the rotation matrix $\mathbf R(\theta)$ maps the reaction forces in the 
contact frame into the object frame and is given by:

$$
\begin{align}
    \mathbf R(\theta) = \begin{bmatrix}
        \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta
    \end{bmatrix}
\end{align}
$$

where $\theta=30^o$. The gravity force vector is defined as:

$$
\begin{align}
    m\mathbf g = mg\begin{bmatrix}
        -\cos\theta \\ -\sin\theta \\ 0
    \end{bmatrix} = -10\begin{bmatrix}
        \cos\theta \\ \sin\theta \\ 0
    \end{bmatrix} 
\end{align}
$$

with these, the constraints are defined as:

$$
\begin{align}
    \mathbf Q & = \mathbf I_{5\times 5} \\
    \mathbf A & = \begin{bmatrix}
        \cos\theta & \sin\theta & 1 & 0 & 0 \\
        -\sin\theta & \cos\theta & 0 & 1 & 0 \\
        r_y\cos\theta + r_x\sin\theta & r_y\sin\theta - r_x\cos\theta & 0 & 0 & 1
    \end{bmatrix} \\
    \mathbf b & = 10\begin{bmatrix}
        \cos\theta \\ \sin\theta \\ 0
    \end{bmatrix} \\
    \mathbf C & = -\begin{bmatrix} 0 & 1 & 0 & 0& 0 \\ -1 & \mu & 0 &  0 & 0 \\ 1 & \mu & 0 &  0 & 0 \end{bmatrix} \\     \mathbf d_{3\times 1} & = \mathbf 0_{3\times 1}
\end{align}
$$

which can be passed to solvers such as Gurobi. If done correctly, Gurobi will return:

$$
\begin{align}
    \mathbf x^* & = \begin{bmatrix}
        f_t = 1.39789 \\
        f_n = 4.65964 \\
        f_x = 5.11982 \\
        f_y = 1.66358 \\
        \tau = -0.10201
    \end{bmatrix} \\
    \text{Obj} & = 52.6568
\end{align}
$$

where **Obj** is the value of the objective function at the optimal point.

### Balancing in 3D

When moving from 2D to 3D, we extend both the wrench and gravity vectors. 
Specifically for the wrench, $\mathbf w \in \mathbb{R}^3 \; \rightarrow \; \mathbf w \in \mathbb{R}^6$ where 
we add one linear and 2 torsional components. 
Similarly, $\mathbf g \in \mathbb{R}^3 \; \rightarrow \; \mathbf g \in \mathbb{R}^6$ where the 
last 3 entries (the moments) will be zero. With these changes, the static equilibrium 
constraints are now written in 3D resulting in 6 equality constraints. **The major change that 
significantly complicates the transition from 2D to 3D is that the frictional force constraint.**

The general friction cone in 3D is described by:

$$
\begin{align}
    \mathcal{FC} = \{ (f_n, f_t, f_o) \in \mathbb{R}^3 \; |\; \sqrt{f_t^2 + f_o^2} \leq \mu f_n \; , \; f_n \geq 0   \}
\end{align}
$$

where $(f_o, f_t)$ are the projections of the tangential frictional force along 
two orthogonal axes in the contact plane. Let $(f_o, f_t) = \mathbf x$ and $\mu f_n = t$, 
then we can write the general friction cone as:

$$
\begin{align}
    \mathcal{FC} = \{ (\mathbf x, t), \; \mathbf x \in \mathbb{R}^2, \; t \geq 0 \in \mathbb{R}  \; |\; \lVert \mathbf x \rVert_2 \leq t   \}
\end{align}
$$

which is the standard Second Order Cone (SOC) form. Thus, in 3D, the friction 
cone is a second order cone. A common technique to simplify the SOC constraint is 
to approximate it using an inscribed polyhedral cone. This approximation has been used 
extensively for modeling, planning, and controls. However, we will stay with the SOC form 
due to the recent advances in solvers that work effectively and scale relatively well. 
Using the SOC form, the problem is then a general SOCP (convex quadratic loss and convex 
constraints composed of linear equalities, linear inequalities, and convex SOCs). Before 
writing down the optimization, we write the friction cone constraint in an equivalent SOC 
form that will be useful for optimization:

$$
\begin{align}
    \mathcal{FC} = \{ (\mathbf x, t), \; \mathbf x \in \mathbb{R}^2, \; t \geq 0 \in \mathbb{R}  \; |\; \lVert \mathbf x \rVert_2 \leq t   \} \equiv \lVert \mathbf A \mathbf x + \mathbf b \rVert_2 \leq \mathbf c^T \mathbf x + d
\end{align}
$$

where $\mathbf x = \mathbf f_c = (f_t, f_o, f_n)$, $\mathbf A = \mathbf{I}_n$, $\mathbf b = 0$, $\mathbf c^T = (0, 0, 1)$, and $d=0$. Thus, we can write:

$$
\begin{align}
    \begin{aligned}
    \min_{\mathbf w \in \mathbb{R}^6, \mathbf f_c \in \mathbb{R}^3} \quad & \frac{1}{2} \mathbf w^{T} \mathbf Q_w \mathbf w + \frac{1}{2} \mathbf f^T_c \mathbf Q_f \mathbf f_c &  \\
    \textrm{s.t.} \quad &  \mathbf J_c \mathbf f_c + \mathbf w + m\mathbf g= 0 & \text{Static Equilibrium} \\
    &0 \leq f_{c,n} & \text{Contacts can only push} \\
    &\lVert \mathbf A \mathbf f_c + \mathbf b \rVert_2 \leq \mathbf c^T \mathbf f_c + d  & \text{Friction Cone Constraint} \\
    &0 \leq \mu f_{c,n} + f_{c,t} \quad \quad  & \text{Left Friction Cone}
\end{aligned}
\end{align}
$$

#### A Note on Feasibility

The problem of balancing a rigid object subject to external frictional contacts, 
and an arbitrary wrench applied to COM is always feasible. Let's re-visit the 
optimization we first wrote down for the 2D case (the argument with no adjustment 
holds for the 3D case):

$$
\begin{align}
    \begin{aligned}
    \min_{\mathbf w \in \mathbb{R}^3, \mathbf f_c \in \mathbb{R}^2} \quad & \frac{1}{2} \mathbf w^{T} \mathbf Q_w \mathbf w + \frac{1}{2} \mathbf f^T_c \mathbf Q_f \mathbf f_c &  \\
    \textrm{s.t.} \quad &  \mathbf J_c \mathbf f_c + \mathbf w + m\mathbf g= 0 & \text{Static Equilibrium} \\
    &0 \leq f_{c,n} & \text{Contacts can only push} \\
    &0 \leq \mu f_{c,n} - f_{c,t} \quad \quad  & \text{Right Friction Cone} \\
    &0 \leq \mu f_{c,n} + f_{c,t} \quad \quad  & \text{Left Friction Cone}
\end{aligned}
\end{align}
$$

We can always choose a wrench that cancels gravity, i.e.:

$$
\begin{align}
    \hat{\mathbf w} = \mathbf w + m \mathbf g
\end{align}
$$

and re-write the static equilibrium equality constraint as:

$$
\begin{align}
    \mathbf J_c \mathbf f_c + \hat{\mathbf w} = 0
\end{align}
$$

The only way in which the original QP is \textbf{not} feasible is either 
for the equality constraint to not have a solution or for the inequality 
constraints to form an empty set (which results in an empty feasible set). 
The latter is not a problem since the inequalities form a non-empty open 
convex cone. The former is also not a problem. We first note that $\mathbf J_c$ 
has full row rank and therefore:

$$
\begin{align}
    \mathbf f_c = - (\mathbf J_c^T \mathbf J_c)^{-1} \mathbf J_c^T \hat{\mathbf w}
\end{align}
$$

is always computable since the square matrix $\mathbf J_c^T \mathbf J_c$ is invertible. 
Thus, we can solve for the contact forces and re-write the optimization program 
strictly as a function of the wrench which leads to a convex quadratic loss subject 
to a non-empty set of inequalities which is always feasible. To illustrate, the 
optimization becomes:

$$
\begin{align}
    \begin{aligned}
    \min_{\mathbf w \in \mathbb{R}^3} \quad & \frac{1}{2} \mathbf w^{T} \left( \mathbf Q_w + \mathbf J_c (\mathbf J_c^T \mathbf J_c)^{-T} \mathbf Q_f (\mathbf J_c^T \mathbf J_c)^{-1} \mathbf J_c^T \right) \mathbf w + \frac{1}{2} m\mathbf g^T \mathbf Q_f m\mathbf g &  \\
    \textrm{s.t.} \quad &  \begin{bmatrix} 0 & 1 \\ -1 & \mu \\ 1 & \mu \end{bmatrix} (\mathbf J_c^T \mathbf J_c)^{-1} \mathbf J_c^T \mathbf w  \leq -  m \mathbf g & \text{3 Linear Inequalities} 
\end{aligned}
\end{align}
$$

where all we did was to solve for $\mathbf f_c$ as a function of $\mathbf w$, and 
replace it into the optimization. By solving this slightly lower dimensional 
optimization, we compute $\mathbf w$ and from that $\mathbf f_c$. This result should not 
be surprising. In theory, we could simply apply sufficient wrench to almost lift 
the object off the contact, negating entirely the frictional force but maintaining "balancing". 
This trivial solution is evidence enough that at least the problem is feasible. However, it
turns out that this is not the smallest wrench solution, as demonstrated by the example we solved. 
Simply put, it is inefficient to cancel both gravity and friction. Rather, choosing a 
wrench that exploits both to balance will lead to lower effort.

### Pivoting

The task of pivoting here means to maintain the sticking contact constraint 
between the object and environment while tracking a desired orientation trajectory 
$\theta_t$. This problem may be formulated as a quasi-static, quasi-dynamic, or fully 
dynamic task. Practically speaking, the quasi-dynamic formulation is the most prevalent. 
This is because rarely is an object pivoted quickly enough for significant Coriolis or 
Centrifugal terms to play a effect in the dynamics. The advantage of the quasi-dynamic 
formulation is that we can ignore these complex nonlinear terms in the equations of motion.

#### Rigid-body Quasi-Dynamic Equations of Motion

The general rigid-body equations of motion can be written as:

$$
\begin{align}
    \mathbf M(\mathbf q) \ddot{\mathbf q} + \mathbf C (\dot{\mathbf q}, \mathbf q) \dot{\mathbf q} + \mathbf g(\mathbf q) = \mathbf \tau + \sum \mathbf J(\mathbf q) \mathbf f_c
\end{align}
$$

where $\mathbf q$ is the configuration of the object, $\mathbf M(\mathbf q)$ is the 
configuration dependent intertia matrix, $\mathbf C(\mathbf q, \dot{\mathbf q})\dot{\mathbf q}$ are 
the Centrifugal and Coriolis acceleration terms, $\mathbf g(\mathbf q)$ is the gravitational 
acceleration, $\mathbf \tau$ are the generalized forces/torques, and $\sum \mathbf J(\mathbf q) \mathbf f_c$ 
represent the contribution from externally applied forces. These expressions can 
be discretized in time, for instance in the case of the explicit/forward Euler 
approximation we may write:

$$
\begin{align}
    \frac{1}{h} \mathbf M(\mathbf q_t) (\dot{\mathbf q}_{t+1} - \dot{\mathbf q}_{t}) + \mathbf C (\dot{\mathbf q}_t, \mathbf q_t) \dot{\mathbf q}_t + \mathbf g(\mathbf q_t) & = \mathbf \tau_t + \sum \mathbf J(\mathbf q_t) \mathbf f_{c,t} \\ 
    \mathbf q_{t+1} & = \mathbf q_{t} + h \dot{\mathbf q}_{t}
\end{align}
$$

where given the tuple $(\mathbf q_t, \dot{\mathbf q_t}, \mathbf \tau_t, \mathbf f_{c,t})$, 
we can solve these equations for $(\mathbf q_{t+1}, \dot{\mathbf q}_{t+1})$. The explicit 
Euler approximation results in a linear system of equations that can be solved very 
efficiently. We highlight that while this formulation computationally efficient, the 
explicit Euler approximation suffer from numerical issues when integrating forward in time.

The quasi-dynamic assumption allows for short periods of dynamic motions but makes two 
important assumptions about this motion: First, accelerations do not integrate into 
significant velocities, and two the object velocity from the previous time step is 0. 
The consequence of the first assumption is that the Coriolis and Centrifugal accelerations 
are approximately zero. This is due to the presence of $2^{nd}$ order terms in $\dot{\mathbf q}$ 
terms. Note that this is fundamental to the continuous time derivation and is inherited by 
the discrete dynamics, independent of the discretization scheme used. Thus, these terms are 
set to zero. The consequence of the second assumption is that $\dot{\mathbf q}_t \approx 0$. 
The equations of motion are then:

$$
\begin{align}
    \frac{1}{h} \mathbf M(\mathbf q_t) \dot{\mathbf q}_{t+1} + \mathbf g(\mathbf q_t) = \mathbf \tau_t + \sum \mathbf J(\mathbf q_t) \mathbf f_{c,t}
\end{align}
$$

and we highlight that since $\dot{\mathbf q}_{t+1} = \frac{1}{h}(\mathbf q_{t+1} - \mathbf q_t)$, we only need this one expression to forward predict motion:

$$
\begin{align}
    \mathbf q_{t+1} = \mathbf q_t + h^2 \mathbf M^{-1}(\mathbf q_t)( \mathbf \tau_t + \sum \mathbf J(\mathbf q_t) \mathbf f_{c,t} -\mathbf g(\mathbf q_t))
\end{align}
$$

Effectively, we have reduced the number of variables needed by half. The physical 
insight/intuition behind this formulation is that in general acceleration and velocity 
point in different directions. However, in the particular case of quasi-dynamic motion, 
these two vectors exist only instantaneously and are parallel. 

#### Quasi-Dynamic Pivoting

Returning to our original balancing optimization program, we replace the 
static equilibrium equation with the quasi-dynamic constraint and add a term to 
the cost that captures the penalty for deviation from the desired trajectory:

$$
\begin{align}
    \begin{aligned}
    \min_{\mathbf w_t \in \mathbb{R}^3, \mathbf f_{c,t} \in \mathbb{R}^2} \quad & \sum_{t=1}^{N} \; \frac{1}{2} \mathbf w_t^{T} \mathbf Q_w \mathbf w_t + \frac{1}{2} \mathbf f^T_{c,t} \mathbf Q_f \mathbf f_{c,t} + \frac{1}{2} (\mathbf q_t^d - \mathbf q_t)^T (\mathbf q_t^d - \mathbf q_t) &  \\
    \textrm{s.t.} \quad &  h^2 \mathbf M^{-1}(\mathbf J_c(\mathbf q_t) \mathbf f_{c,t} + \mathbf w_t + m\mathbf g) + \mathbf{q}_{t-1} = \mathbf{q}_{t} & \text{Quasi-dynamic Motion} \\
    &0 \leq f_{c,n} & \text{Contacts can only push} \\
    &0 \leq \mu f_{c,n} - f_{c,t} \quad \quad  & \text{Right Friction Cone} \\
    &0 \leq \mu f_{c,n} + f_{c,t} \quad \quad  & \text{Left Friction Cone} \\
    & \mathbf{q}_{\text{given}} = \mathbf{q}_0 \quad \quad  & \text{Initial Pose}
\end{aligned}
\end{align}
$$

Note that this optimization program is strictly tracking some desired trajectory along 
time $\mathbf q_t^d$. For pivoting, we would choose $\mathbf q_t^d$ to be an arc about the 
contact point.

For the remainder of these notes, we will not continue to discuss pivoting since 
the procedure to go from balancing to pivoting is the addition of a term in the cost 
function and changing the static equilibrium constraint to quasi-dynamic motion.
