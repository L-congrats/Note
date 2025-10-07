---
{"dg-publish":true,"permalink":"/luc/rwth/rigid-body-dynamics/regid-body-dynamics/"}
---


#### CONTENT
[1. Rigid body](#####1.%20Rigid%20body)
[2. Momentum conservation](#####2.%20Momentum%20conservation)
[3. Kinematic Constraints](#####3.%20Kinematic%20Constraints)
[4. D‘Alembert-principle](#####4.%20D‘Alembert-principle)
[5. Recap some definition](#####5.%20Recap%20some%20definition)
[Recap: Jacobi Matrix](#####Recap%20Jacobi%20Matrix)
[7. Constraint forces and simulation](#####7.%20Constraint%20forces%20and%20simulation)
[8. Simulation of constrained rigid body with friction](#####8.%20Simulation%20of%20constrained%20rigid%20body%20with%20friction)


##### 1. Rigid body
The objects are assumed to be **non-deformable**, meaning elasticity or flexibility is neglected.  
Therefore, each rigid body can be fully described by **six degrees of freedom** — three for translation and three for rotation — or by its corresponding **joint coordinates** in a multibody system.
1. **Kinematics** focuses only on _how things move_ — describing the geometry of motion (positions, velocities, and accelerations) **without considering the forces** that cause it.
2. **Dynamics**, on the other hand, studies _why things move_ — it relates **forces and torques** to the resulting motion (or inversely, determines the required forces/torques to achieve a desired motion).
3. **Generalized Coordinates** (joint angle \ velocities \ accelerations)
	- Generalized Coordinates are the minimum number of independent variables needed to completely describe the configuration of a mechanical system.
		- Eample
			1. These coordinates are independent and correspond directly to the system's degrees of freedom (DoG)
			2. A single pendulum can be described by one variables : the angle $\theta$
			3. A 2-link robot arm can be described by two joint variables : $(\theta_1 , \theta_2)$
4. **Maximal Coordinates** (position \ orientations \ velocities)
	- Maximal coordinates describe every body's full pose (position and orientation) in the global or world frame, without reducing by constraints.
		- Example 
			1. Pendulum : use coordinates (x, y) of the pendulum mass center (2 variables), and we have a constraint: $x^2 + y^2 = L^2$
			2. N- body system : Each rigid body has 6coordinates (3 translation + 3 rotation in 3D) so total = 6n variables, with many constraints (joints)
5. Why we represent system in maximal coordinates?
	- Because they directly describe each body's position and orientation ,it's easy for us to handle in simulation. And it's simpler to detect collision and apply forces at any point on the body, so physics engines often use maximal coordinates.

$$
\tau_{\text{ext}} 
= \underbrace{\Theta \dot{\omega}}_{\text{惯性矩 $\times$ 角加速度}}
+ \underbrace{\omega \times (\Theta \omega)}_{\text{陀螺/离心耦合项}}
$$
[return](###CONTENT)

##### 2. Momentum conservation
1. 
	![Pasted image 20251002191109.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251002191109.png)
	- ……
		- $P_i$ : Linear momentum
		- $L_i$ : Angular momentum
		- $\Theta_i$ : Inertia tensor 
		- $f_{ext,i}$ : External Force
		- $\tau_{ext,i}$ : External torque
				- 外力、外力矩是驱动力；质量和惯量描述抗拒的能力
2. Simulation of rigid body system : Forward Dynamics and Integration
	- ![Pasted image 20251003203848.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251003203848.png)
	- We can get the acceleration of system by forward dynamics
	- At each small time step $dt$ , compute the acceleration from the external forces, **update the velocity, and then update the position**. Repeating this process simulates how robot or rigid body system moves over time.
##### 3. Kinematic Constraints
- Kinematic constraints are rules that connect rigid bodies together at joints or contact point. They make sure that bodies move consistently, EX. two links in a robot arm sharing the same joint.
	- **Constraint Jacobian** $J = \frac{\partial C}{\partial x}$
		- "J equals the partial derivate of C with respect to x", **it's describe the direction of the constraint**.
		- ![Pasted image 20251003171836.png|150](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251003171836.png)
		- When $C(x,t) = 0$ is satisfied, the velocity in these directions must be zero.
			- When we differentiate the position of a point $p(t)$ on a rigid body, we get$$\dot{P(t)} = \dot{x(t)} + \dot{r_i(t)}$$
				- $\dot{x(t)}$ : is the linear velocity of body's center of mass.
				- $\dot{r_i(t)}$ : is the additional velocity of point $P$ caused by body's rotation around its center of mass.
	[return](###CONTENT)

##### 4. D‘Alembert-principle
**Constraint forces do not perform any work**, it's means they do not add or remove energy from the system - they only restrict the motion so that the bodies move in allowed directions. It's not change the system's energy.
1. Constraint forces : $f_c = J^T \cdot \lambda$
		- $J^T$ describe the direction of the force.
		- $\lambda$ is Lagrange Multipliers, define the magnitude of the constraint forces
		- ![Pasted image 20251003210232.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251003210232.png)
2. Inverse dynamics equation
	- ![Pasted image 20251003211832.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251003211832.png)
	- First part is the effect of the constraint forces on the system, second part is the effect of the external forces and the current velocity.
	- Solving this equation to get $\lambda$ ,and the obtain the actual forces via $f_c = J^T \cdot \lambda$.
3. Simulation loop
	1. state vector: $x(t)$ $\dot{x(t)}$
	2. Constraint Formulation: Define the constraints of the system using $C(x(t),t) = 0$
		This provides the Jacobian $J$ that describes the constraint directions.
	3. Inverse Dynamics: Using equations of motion with the constraint equations to solve for the Lagrange multipliers $\lambda$ , and calculate the constraint forces $f_c = J^T \lambda$
	4. Forward Dynamics : use both external forces and the constraint forces to update the system velocity: $\dot{x}(t+h) = \dot{x}(t) + dt \cdot M^{-1}(t) \cdot (f_{ext} + J^T \lambda)$
	[return](###CONTENT)
	
##### 5. Recap some definition:
1. **rigid body**: is a solid object that does not deform, this means that the distance between any two points inside the body is always same, no matter what  forces are applied.

2. **Center of mass**: is a unique point in space, where the weighted relative position of the distributed mass vanishes. In other words, if we look at the body as many small mass elements, their relative positions balance each other at this point.

3. **Inertia tensor** : describes the relationship between angular velocity and angular momentum. it describes how the mass of a rigid body is distributed, and how this affects its rotation. *It describes how a rigid body resists rotation. Unlike a point mass, a rigid body has mass distributed in space, so it resists rotation differently around different axes.*
	- In linear motion, we have momentum $p = m \cdot v$.
	- In rotational motion, there is a similar formula: angular momentum $L = \Theta \cdot \omega$ 
	- So, the inertia tensor like the "equivalent mass" for rotation
	- Properties of inertia tensor:
		1. symmetric : Inertia tensor is equal to its transpose 
		2. positive definite : For any non-zero vector $x$, $x^T\cdot\Theta\cdot x > 0$

4.  Parallel Axis theorem: allows us to change the reference point of the inertia tensor
	- The inertia tensor $\Theta_{cog}$ is defined at the center of mass $p_{cog}$, we can use parallel Axis theorem gives a formula to shift the inertia tensor from the center of mass to the new point $p'$.![Pasted image 20251004010611.png|400](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251004010611.png)
		displacement vector from center of mass $p_{cog}$ to the new point$P'$:  $(a^2 I - aa^T)$

5.  Orientation of the inertia tensor
	- ![Pasted image 20251004014326.png|350](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251004014326.png)
	- If we know the inertia tensor in coordinate system $W_1$ , we can transform it into coordinate system $W_2$ using the rotation matrix.

6. **Eigenvectors of Inertia Tensor**
	![Pasted image 20251004030007.png|240](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251004030007.png)
	- $\Theta_{cog}$ : the inertia tensor at the center of mass  
	- $\lambda$ : the eigenvalue, which here means the principle moment of inertia
		- Principal moments of inertia: describe how much resistance the body has when rotating around these principal axes. 
	- $x$ : the eigenvector, which here means the principal axis direction
		- Principal axes direction where the rigid body rotates without causing extra coupled motion in other direction.
		![Pasted image 20251004030807.png|300](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251004030807.png)![Pasted image 20251004030824.png|600](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251004030824.png)
		- $\theta_{11}$, $\theta_{22}$, $\theta_{33}$: the principal moment of inertia

7. Quaternions: avoid gimbal lock
	1. Definition and algorithm
		 ![Pasted image 20251004163110.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251004163110.png)
		![Pasted image 20251004163137.png|550](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251004163137.png)
		![Pasted image 20251004163200.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251004163200.png) 
	-  $r_q(x) = qxq^{-1}$ 
		$q$ is quaternions, $x$ is a three dimension vector, we write it as a pure unit quaternions.

8. Body Fixed point
	- $v_p = v_{cog} + \omega \times r$
		$v_p$ : the absolute velocity of the point $p$
		$v_{cog}$ : the absolute velocity of the center of gravity.
		$\omega \times r$ : the rotational velocity of point $p$ relative to the COG.
	- $\omega_{p} = \omega_{cog} =\omega$ 
		All point on the rigid body share the same angular velocity.
	- $\dot{v}_p = \dot{v}_{cog} + \omega \times (\omega \times r) + \dot{\omega} \times r$ 
		$\dot{v}_{cog}$ :acceleration of the center of gravity (due to the rigid body's translation).
		$\omega \times (\omega \times r)$ : Centripetal acceleration, direction toward $-r$.
		$\dot{\omega} \times r$ : Euler acceleration, appearing when the angular velocity changes.
	- $\dot{\omega}_p = \omega_{cog} = \dot{\omega}$ 
		All points have same angular velocity.

9. Non-inertial reference frame: 
	1. A reference frame that is itself accelerating. Newton's laws can not be applied directly inside it; fictitious forces (like centrifugal force or Coriolis force) must be added.
		- *Inertial reference frame: is not accelerating due to external forces, where newton's laws always can be used.*
	2. Position transformation of a point $p$ :
		![Pasted image 20251004192317.png|150](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251004192317.png) ![Pasted image 20251004192607.png|200](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251004192607.png)  
		Transform point's position from the body frame to the world frame.
	3. Velocity of a point $p$ : 
		${}^W v_p = {}^W v_{cog} + \omega \times ( {}^W R_B \cdot {}^B p ) + {}^W R_B \cdot {}^B v_p$
		- ${}^W v_{cog}$ : velocity of the center of gravity
		- $\omega \times ( {}^W R_B \cdot {}^B p )$ : velocity of point $p$ relative to the center of gravity due  to body's rotation.
		- ${}^W R_B \cdot {}^B v_p$ : relative of the point inside the body frame 
		- 参考 ：$v_p = v_{cog} + \omega \times r + (相对速度项)$
	4. Acceleration of point $p$ in the world frame $W$ :
	$${}^W \dot{v}_p ={}^W \dot{v}_{cog} +\dot{\omega} \times ( {}^W R_B \cdot {}^B p ) + \omega \times ( \omega \times ( {}^W R_B \cdot {}^B p ) ) + 2 \cdot \omega \times ( {}^W R_B \cdot {}^B v_p ) + {}^W R_B \cdot {}^B \dot{v}_p$$
		- ${}^W \dot{v}_{cog}$ : …………
		- $\dot{\omega} \times ( {}^W R_B \cdot {}^B p )$ : Euler acceleration additional term caused by the change in angular velocity.
		- $2 \cdot \omega \times ( {}^W R_B \cdot {}^B v_p )$ : Coriolis acceleration is an extra term that appears when the point has relative motion inside non-inertial frame.
		- ${}^W R_B \cdot {}^B \dot{v}_p$ : Relative acceleration of the point in the body frame, transformed into the world frame.

10.  **Constraints: Non-holonomic constraints and Holonomic constraints**
	constraints are used in mechanical engineering to restrict the degrees of freedom of dynamics system.
	1. Non-holonomic constraints: can be described by equation, but can be described by inequality equation between position and orientation: $C(\underline{x}(t),t) \ge 0$. e.g., rolling, contact.
	2. Holonomic constraints: can be describe by equation between the position and orientation variables: $C(\underline{x}(t),t) = 0$ . e.g., hinge joint & prismatic joint.
		- Derivation of holonomic constraints in inertial frame. [ref: c10.d p5-p11](<RWTH/Rigid Body Dynamics/Slides/Rigid body dynamics.pdf#page=43>)
		- Derivation of holonomic constraints in joint frame A1. [ref: c10.d p14](<RWTH/Rigid Body Dynamics/Slides/Rigid body dynamics.pdf#page=52>)
	[return](###CONTENT)
##### Recap Jacobi Matrix
Assume we have a vector function: $f: \mathbb{R}^n \rightarrow \mathbb{R}^m$
The input is an n-dimensional variable vector$x = (x_1, x_2, ..., x_n)^T$
The output is an m-dimensional function vector$f(x) = (f_1(x), f_2(x), ..., f_m(x))^T$
We can get Jacobi matrix:![Pasted image 20251005163836.png|400](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251005163836.png)
Each element is a partial derivative, describing how each output component $f_i$
	changes with respect to each input variable $x_j$.
![Pasted image 20251005170856.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251005170856.png)
	[return](###CONTENT)

##### 6. Conservation of momentum
1. If no external forces or torques are applied, the total momentum of a system of rigid body remains constant: $\sum_i \mathbf{p}_i = \text{const.}$, $\sum_i \mathbf{L}_i = \text{const.}$         ( $p = m\cdot v$,       $L = \Theta \cdot\omega$  )

2. If external forces or torques applied and mass is constant, the momentum of rigid body changes.
		$$f_{ext} = \dot{p} = m \cdot \dot{v}$$    $$\tau'_{ext} = \dot{L} = \Theta \cdot \dot{\omega} + \underbrace{\omega \times \Theta \cdot \omega}_\text{Coriolis term / gyroscopic torque}$$
	 - **Gyroscopic torque** is an extra torque that appears because the inertia tensor of a rotating body changes with time in the world frame.
		![Pasted image 20251005182709.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251005182709.png)
		It makes a spinning body resist sudden change in direction, so when it is disturbed, it shows precession or tumbling
		 受到外力干扰时会进动或翻滚
			precession: smooth change of the rotation axis.
				进动，旋转轴缓慢转斗，整体仍平稳，e.g., 自行车轮子
			umbling: unstable rotation when the spin axis is not aligned with teh principal axis.
				翻滚，旋转轴不稳定，方向不断变化，
				
		If the external force attacks off-centre of the centre of gravity it creates an additional torque.     $\tau'_{ext} = r \times f'_{ext}$

3. Semi-implicit integration: 
	The acceleration calculation uses the state information at the current time $t$. 
	But the position and orientation update uses the newly computed velocity at $t+dt$.
		先用当前时间求加速度，用这个加速度更新速度，在用这个速度求新的位置的方向
		
	- Explicit integration: uses only the state information at the current time $t$ to compute the next step. Fast but less stable
		- Implicit integration: considers both the state at $t$ and at $t+dt$. Stable but computationally expensive.

4. simulation loop:
	![Pasted image 20251005212119.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251005212119.png)
	[return](###CONTENT)

##### 7. Constraint forces and simulation
1. D'Alembert Principle:   [公式](#4.D‘Alembert-principle)
	constraint forces do not perform virtual work.
2. Calculation of constraint force 
	Equation of motion: $M \cdot \ddot{x} = f_{ext} +f_C = f_{ext} + J^T \cdot \lambda$
	Constraint equation: $J \cdot \ddot{x} - b = 0$
	1. Equation system approach:
		![Pasted image 20251005221159.png|270](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251005221159.png)
		Cons: Large system of equations with many dimensions because includes all degrees of freedom. In complex system, solving it directly takes a lot of computation.
	2. **JMJT-approach**
	$$J \cdot M^{-1} \cdot J^T \times \lambda + J \cdot M^{-1} \cdot f_{ext} - b =0$$
		Remove the acceleration $\ddot{x}$ from the equations and only solve for $\lambda$
		Pros: Smaller equation system and forward dynamics is calculated independently
		- Simulation loop for constrained rigid body system (JMJT-approach)
			![Pasted image 20251005223256.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251005223256.png)
3. Numerical drift: 
	In maximal coordinates, constraints are enforced numerically, small integration and linearization errors accumulate, so joint constraints drift over time .
	
4. Baumgarte stabilization:
	$$\ddot{C}(x,t) + \alpha \cdot \dot{C}(x,t) + \beta \cdot C(x,t) = 0$$
	Add a feedback term to the constraint equation, it behave like a spring-damper system that pulls the bodies back to the correct constraint position whenever numerical drift appears.
	We have  $\ddot{C}(x,t) = J \cdot \ddot{x} -b$ , so:
	$$\underbrace{J \ddot{x} - b}_{\text{relative acceleration}} + \alpha \cdot \underbrace{J \cdot \dot{x}}_{\text{relative velocity}} + \beta \cdot \underbrace{x_{rel}}_{\text {violation of position based constraint}} = 0$$
	- Including Baumgarte stabilization to the inverse dynamics
		- ![Pasted image 20251006030401.png|600](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006030401.png)
			- ![Pasted image 20251006033850.png|600](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006033850.png)

5. Impulse-based simulation of constrained rigid bodies.
	Impulse acts instantly and changes velocity suddenly at collision, impulse based simulation directly updates velocity using momentum change. It's more stable and better for collision and contacts.
	In a discrete time step $\Delta t$, the constraint reaction force is computed based on the change in velocity rather than acceleration.
	1. Holonomic constraint
		- The mass matrix $M$ multiplied by the change in velocity equal the sum of external force and constraint reaction (i.e., impulse contributions), $\lambda$ is the impulse magnitude acting during this time interval.
			- 质量矩阵$\times$速度变换量 = 外力 + 约束反力。
			- $\lambda$ 是这段时间内的冲量强度
		- ![Pasted image 20251006035141.png|500](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006035141.png)
		- ![Pasted image 20251006035210.png|500](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006035210.png)
	2. Non-holonomic constraints in inverse dynamics
		![Pasted image 20251006172451.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006172451.png)![Pasted image 20251006172507.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006172507.png)
		![Pasted image 20251006172603.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006172603.png)
		![Pasted image 20251006172616.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006172616.png)
		![Pasted image 20251006172701.png|560](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006172701.png)
		![Pasted image 20251006172732.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006172732.png)
		[return](###CONTENT)
##### 8. Simulation of constrained rigid body with friction
1. Coulomb Friction model
	If the contact force $f_t$ lies inside the cone, the contact no slid.
	If it on the cone's surface, sliding begins.
	- ![Pasted image 20251006173849.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006173849.png)
2. Direction of friction
	![Pasted image 20251006192959.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006192959.png)
	- If body slide, the direction mainly in one opposite direction to motion
	- If body stick, multiple direction are active because the final tangential velocity is unknown.
3. **Two complementarity constraints**
	1.  Relative velocity - friction forces : $J_d \cdot v + \beta \cdot e = 0$ , $\lambda_d > 0$
		- When two bodies are sliding relative to each other, $J_d \ne 0$ , the friction adjusts its direction and magnitude accordingly.
		- When the contact is sticking, $J_d = 0$ , there is no relative motion at the contact point.
	2. Normal forces - friction forces : $\mu \cdot \lambda_n - e^T \cdot \lambda_d = a_{aux} \ge 0$ , $\beta \ge 0$ , $a_{aux} \cdot \beta = 0$ 
		- Ensures that the total friction force does not exceed $\mu \lambda_n$ -the friction cone constraint. When $a_{aux} = 0$, the friction limit is on the surface.
		
		1. $J_d \cdot v$ : $J_d$ is the Jacobian matrix in the friction direction. $v$ is the generalized velocity vector of the whole system, including both linear and angular velocities of the contacting bodies. $J_d \cdot v$ is relative velocity of the contact point along the friction direction.
		2. $\beta$ : Auxiliary multiplier, connecting both complementarity constraints
			$\beta = 0$ : Static friction
			$\beta > 0$ : Dynamic friction
		3. $\lambda_d$ : Friction value of each base vector of the friction cone.
		4. $\lambda_n$ : normal contact force
		5. $a_{aux}$ : auxiliary variable used to determine whether the friction is reached, when $a_{aux} = 0$ , the friction limit is on the surface of the cone.
	
4. Incorporation of the friction constraints to the inverse dynamics [ref: c10.j P10](<RWTH/Rigid Body Dynamics/Slides/Rigid body dynamics.pdf#page=175>)
		![Pasted image 20251006231517.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251006231517.png)
5. Lagrange Formalism [ref: c10-k](<RWTH/Rigid Body Dynamics/Slides/Rigid body dynamics.pdf#page=186>)	
	- ![Pasted image 20251007022831.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251007022831.png)
	- $H(q)$ : the inertia matrix, represents the mass distribution of the system.
	- $C(q,\dot{q})$ : include all nonlinear forces.
	- $\tau$ : external driving torques.
	
6. Forward dynamic and inverse dynamics
	1. ID: Given a desires trajectory or acceleration $\ddot{q}$, compute the joint torques $\tau$ required to achieve it : $\tau = ID(q,\dot{q},\ddot{q})$
		We compute forces required to produce motion.
		- applications: control design, robot controller design, torque prediction and model validation.
	2. FD: Given the joint torques $\tau$ and the current system state $q$, $\dot{q}$ , determine the accelerations $\ddot{q}$ of the system : $\ddot{q} = FD(q,\dot{q},\tau)$
		We compute motion resulting resulting from    forces. $\ddot{q} = H(q)^{-1}(\tau - C(q,\dot{q}))$
		- Applications: robot simulation, physics engines, numerical integration.
	3. FD CRBA (Composite Rigid Body algorithm 复合钢体算法)     [ref：CRBA](<RWTH/Rigid Body Dynamics/附件/CRDA.md>)
			![Pasted image 20251007030044.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251007030044.png)
	4. FD ABA (Articulated Body Algorithm 关节系统算法)   [ref: ABA](<ABA.md>)
		It doesn't need to build or invert the inertia matrix $H(q)$. Instead, it uses a step by step recursive process, which make the computation much faster from $O(n^3)$ to only $O(n)$, where $n$ is the number of joints.
	
7. Newton Euler formalism [ref: 10-k P13](<RWTH/Rigid Body Dynamics/Slides/Rigid body dynamics.pdf#page=196>)
	![Pasted image 20251007035754.png](/img/user/Luc/RWTH/Rigid%20Body%20Dynamics/%E9%99%84%E4%BB%B6/Pasted%20image%2020251007035754.png)
	[return](###CONTENT)