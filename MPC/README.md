# Model Predictive Control

## Formulating the MPC problem

The goal of the MPC is to generate valid input controls for $T$ steps ahead that controls the vehicle and follow the reference trajectory as close as possible.

### State and Dynamics Model

We use a kinematic model of the vehicle. The state space is $z=[x, y, \theta, v]$. And the input vector is $u=[a, \delta]$ where $a$ is acceleration and $\delta$ is steering angle. The kinematic model ODEs are:

$$
\dot{x}=v\cos(\theta)
$$

$$
\dot{y}=v\sin(\theta)
$$

$$
\dot{v}=a
$$

$$
\dot{\theta}=\frac{v\tan(\delta)}{L}
$$

Where $L$ is the wheelbase of the vehicle. In summary, we can write the continuous ODEs as:

$$
f(z, u)=A'z+B'u
$$

Here $A'$ and $B'$ are the original continuous system matrices. In the last step, you'll discretize and linearize the system to get new system matrices $A$, $B$ and $C$. Note that you won't have to implement these in code but you can still write out the matrix representation of $A'$ and $B'$. We highly recommend writing out the system of equation in the matrix form.

### Objective Function

First we formulate the objective function of the optimization. We want to minimize three objectives:
1. Deviation of the vehicle from the reference trajectory. Final state deviation weighted by Qf, other state deviations weighted by Q.
2. Influence of the control inputs. Weighted by R.
3. Difference between one control input and the next control input. Weighted by Rd.

<!-- $$\text{minimize}~~~u^TRu + (x-x_{\text{ref}})_{0,\ldots,T-1}^TQ(x-x_{\text{ref}})_{0,\ldots,T-1} + (x-x_{\text{ref}})_{T}^TQ_f(x-x_{\text{ref}})_{T} + (u_{1,\ldots,T}-u_{0,\ldots,T-1})^TR_d(u_{1,\ldots,T}-u_{0,\ldots,T-1})$$ -->

$$
\text{minimize}~~ Q_{f}\left(z_{T, r e f}-z_{T}\right)^{2}+Q \sum_{t=0}^{T-1}\left(z_{t, r e f}-z_{t}\right)^{2}+R \sum_{t=0}^{T} u_{t}^{2}+R_{d} \sum_{t=0}^{T-1}\left(u_{t+1}-u_{t}\right)^{2}
$$

### Constraints

We then formulate the constraints for the optimization problem. The constraints should inclue:
1. Future vehicle states must follow the linearized vehicle dynamic model.
   $$z_{t+1}=Az_t+Bu_t+C$$
2. Initial state in the plan for the current horizon must match current vehicle state.
   $$z_{0}=z_{\text{curr}}$$
3. Inputs generated must be within vehicle limits.
   $$a_{\text{min}} \leq a \leq a_{\text{max}}$$
   $$\delta_{\text{min}} \leq \delta \leq \delta_{\text{max}}$$

## Linearization and Discretization

In order to formulate the problem into a Quadratic Programming (QP), we need to first discretize the dynamical system, and also linearize it around some point.

### Discretization

Here we'll use Forward Euler discretization since it's the easiest. Other methods like RK4/6 should also work. We discretize with sampling time $dt$, which you can pick as a parameter to tune. Thus, we can express the system equation as:

$$z_{t+1} = z_t + f(z_t, u_t)dt$$

### Linearization
We'll use first order Taylor expansion of the two variable function around some $\bar{z}$ and $\bar{u}$:

$$
z_{t+1}=z_t + f(z_t, u_t)dt
$$

$$
z_{t+1}=z_t + (f(\bar{z_t}, \bar{u_t}) + f'_z(\bar{z_t}, \bar{u_t})(z_t - \bar{z_t}) + f'_u(\bar{z_t}, \bar{u_t})(u_t - \bar{u_t}))dt
$$

$$
z_{t+1}=z_t + (f(\bar{z_t}, \bar{u_t}) + A'(z_t - \bar{z_t}) + B'(u_t - \bar{u_t}))dt
$$

$$
z_{t+1}=z_t + (f(\bar{z_t}, \bar{u_t}) + A'z_t - A'\bar{z_t} + B'u_t - B'\bar{u_t})dt
$$

$$
z_{t+1}=(I+dtA')z_t + dtB'u_t + (f(\bar{z_t}, \bar{u_t})- A'\bar{z_t} - B'\bar{u_t})dt
$$

$$
z_{t+1} = Az_t + Bu_t + C
$$

You can then derive what are matrices $A$, $B$, and $C$.

## Reference Trajectory

Create a reference trajectory that has velocity attached to each waypoint that you create. For a smooth velocity profile, we use the curvature information on the waypoint spline we created to interpolate between a maximum and a minimum velocity. The reference has the same states as the vehicle model you've set up in the optimization problem.
## Setting up the Optimization

In Python, we'll be using CVXPY to set up the optimization problem with the OSQP solver. 

## Visualization

We visualize the current selected segment of reference path and the predicted trajectory from MPC to debug.