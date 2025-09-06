

<!-- Meanless: Proceedings of the 3rd Conference on Fully Actuated System Theory and Applications May 10-12, 2024, Shenzhen, China のと思います。それできなくなくださっことができていますのですが、それでもしますのですね。それでも、-->

# Time-Optimal and Jerk-Continuous Trajectory Planning and Tracking Control for 6-DOF Manipulator based on High-Order Fully Actuated System Control Theory

Die Zou \( {}^{1} \) ,Likun Huang \( {}^{1} \) ,Wei Wang \( {}^{1} \) ,Mengying Lin \( {}^{1,3} \) ,Zixin Huang \( {}^{1,2,3, * } \)

1. School of Electrical and Information Engineering, Wuhan Institute of Technology, Hubei 430205, P. R. China E-mail: huangzx@wit.edu.cn

2. Hubei Key Laboratory of Digital Textile Equipment, Wuhan Textile University, Hubei 430299, P. R. China

3. Hubei Key Laboratory of Intelligent Robot, Wuhan Institute of Technology, Hubei 430205, P. R. China

Abstract: This paper focuses on a 6-DOF manipulator and studies how to reduce the trajectory operation time while ensuring the continuity and stability during motion and improving the precision of trajectory tracking control. The manipulator system's kinematic model is developed through utilizing the D-H parameter approach. And the manipulator system's dynamic model is derived through the utilization of Lagrangian dynamics equations, and incorporate the theory of fully actuated system to derive high-order fully actuated (HOFA) model of the system. The joint space interpolation trajectory is constructed by using the 7th-order B-spline curve interpolation method to guarantee the continuity of velocity, acceleration, and jerk. Additionally, to minimize the point-to-point operation time, the motion trajectory is optimized in conjunction with the PSO algorithm. Within the HOFA system model's framework, a direct parameterization approach is employed to design a trajectory tracking controller which guarantee rapid and precise tracking of the trajectory designed for the manipulator. Ultimately, the proposed design methods are validated through simulation experiments.

Key Words: High-order fully actuated system, 6-DOF manipulator, Time optimal, Jerk continuous, Trajectory tracking control

## 1 Introduction

Manipulators are widely utilized in diverse manufacturing tasks such as assembly[1], spraying[2], palletizing[3], and sorting[4]. To achieve precise operations, trajectory planning and tracking control strategies are crucial. Traditional first-order state models struggle to accurately describe the dynamic behavior of multi-degree-of-freedom manipulators and overlook the coupling effects between joints. However, high-order fully actuated system (HOFAS) control theory offers more precise description of the manipulator's dynamics, addressing issues like jitter and swing and enables more effective control strategies[5].

The theory of fully actuated system control has demonstrated remarkable superiority in dealing with a series of complex issues such as nonlinearity, time variability, and hysteresis. Duan[6] studied a two-link manipulator system, verifying the feasibility of the HOFAS method on serial link manipulators, but did not verify its feasibility on 6-DOF manipulators. Sun[7] designed a trajectory tracking controller for a highly nonlinear and strongly coupled 6-DOF manipulator utilizing the model of a HOFAS. This controller successfully achieved precise trajectory tracking control for the manipulator system, but the designed nonlinear disturbance observer is only suitable for slow-varying internal and external uncertainties, and its performance needs to be improved.

To sum up, there has been relatively limited research combining HOFAS control theory with trajectory planning and tracking control of manipulator systems, using time as an optimization criterion. This paper will focus on time-optimal and jerk-continuous trajectory planning and tracking control research utilizing HOFAS control theory. By using the research method and control strategy, the operation precision can be effectively improved and enhance the performance.

## 2 System Model

### 2.1 Kinematic Model

The kinematic model is developed through the utilization of the D-H parameter approach. Table 1 displays the model parameters, \( {\alpha }_{i} \) and \( {\theta }_{i} \) represent the angle of rotation around the \( X \) axis and \( Z \) axis, \( {a}_{i} \) and \( {d}_{i} \) signify the distance of translation along the \( X \) axis and \( Z \) axis,respectively.

<!-- Media -->

Table 1: D-H Parameter Table for manipulator

<table><tr><td>Node</td><td>\( {\alpha }_{i}\left( {rad}\right) \)</td><td>\( {a}_{i}\left( m\right) \)</td><td>\( {d}_{i}\left( m\right) \)</td><td>\( {\theta }_{i}\left( {rad}\right) \)</td></tr><tr><td>1</td><td>0</td><td>0</td><td>0.0985</td><td>\( {\theta }_{1} \)</td></tr><tr><td>2</td><td>\( \pi /2 \)</td><td>0</td><td>0.1215</td><td>\( {\theta }_{2} \)</td></tr><tr><td>3</td><td>0</td><td>0.408</td><td>0</td><td>\( {\theta }_{3} \)</td></tr><tr><td>4</td><td>0</td><td>0.376</td><td>0</td><td>\( {\theta }_{4} \)</td></tr><tr><td>5</td><td>\( - \pi /2 \)</td><td>0</td><td>0.1025</td><td>\( {\theta }_{5} \)</td></tr><tr><td>6</td><td>\( \pi /2 \)</td><td>0</td><td>0.094</td><td>\( {\theta }_{6} \)</td></tr></table>

<!-- Media -->

With the coordinate transformations' chain rule as the basis, the matrix representing the transformation between any two adjacent links can be written as:

\[{}^{i - 1}{T}_{i} = R\left( {{x}_{i},{\alpha }_{i - 1}}\right) T\left( {{x}_{i},{a}_{i - 1}}\right) R\left( {{z}_{i - 1},{\theta }_{i}}\right) T\left( {{z}_{i - 1},{d}_{i}}\right) \]

(1)

The total transformation matrix for the end effector concerning the base coordinates is detailed as:

\[{}^{0}{T}_{6} = {}^{0}{T}_{1}{}^{1}{T}_{2}{}^{2}{T}_{3}{}^{3}{T}_{4}{}^{4}{T}_{5}{}^{5}{T}_{6} \tag{2}\]

---

<!-- Footnote -->

This work is supported by Hubei Province Nature Science Foundation (No. 2023AFB380), and Hubei Key Laboratory of Digital Textile Equipment (Wuhan Textile University) (No. KDTL2022003, No. DTL2023002), and Hubei Key Laboratory of Intelligent Robot (Wuhan Institute of Technology) (No. HBIRL202301, No. HBIRL202302), and Research Initiation Fund (Wuhan Institute of Technology) (No. 23QD112), and the Graduate Innovative Fund of Wuhan Institute of Technology (No. CX2023550)

<!-- Footnote -->

---

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:53 UTC from IEEE Xplore. Restrictions apply. 222-->


### 2.2 Fully Actuated Model based on Dynamic Model

Due to the presence of various internal and external uncertainties, It is possible to denote the dynamic model through:

\[M\left( q\right) \ddot{q} + D\left( {q,\dot{q}}\right) \dot{q} + G\left( q\right)  = \tau  + {\tau }_{d}\left( {q,\dot{q}}\right)  \tag{3}\]

where \( q \) stands for the angular position. \( M\left( q\right) \) denotes the positive definite inertia matrix. \( \dot{q} \) denotes the angular velocity. \( D\left( {q,\dot{q}}\right) \) corresponds to the coefficient matrix associated with the Coriolis and centrifugal force vectors. \( \ddot{q} \) states the angular acceleration. The matrix \( G\left( q\right) \) denotes the gravitational term. \( \tau \) refers to the input torque of the control. \( {\tau }_{d} \) indicates the combined internal and external uncertainties.

According to the theory of fully actuated systems[5], the manipulator described by the Lagrange equation is a second-order fully actuated system:

\[{A}_{2}\left( {\theta ,q,\dot{q}}\right) \ddot{q} + {A}_{1}\left( {\theta ,q,\dot{q}}\right) \dot{q} + {A}_{0}\left( {\theta ,q,\dot{q}}\right) q \tag{4}\]

\[ + \xi \left( {\theta ,q,\dot{q}}\right)  = B\left( {\theta ,q,\dot{q}}\right) u\]

In equation (4), \( {A}_{2}\left( {\theta ,q,\dot{q}}\right) ,{A}_{1}\left( {\theta ,q,\dot{q}}\right) ,{A}_{0}\left( {\theta ,q,\dot{q}}\right)  \in \) \( {R}^{n \times  n} \) represent coefficient matrixs, \( B\left( {\theta ,q,\dot{q}}\right)  \in  {R}^{n \times  n} \) stands for input matrix, \( q \in  {R}^{n} \) represents state vector, \( u \in  {R}^{n} \) represents control vector,vector \( \xi \left( {\theta ,q,\dot{q}}\right) \) as a nonlinear term,and \( \theta  = \theta \left( t\right)  \in  {R}^{l} \) represents the parameter vector. The equation (4) corresponds to equation (3).

## 3 Trajectory Planning and Tracking Control

### 3.1 Jerk-Continuous Trajectory Planning

To construct trajectory, utilizing 7th-order B-spline curve (BSC) interpolation approach, the expression is as follows:

\[p\left( u\right)  = \mathop{\sum }\limits_{{i = 0}}^{n}{d}_{i}{B}_{i,7}\left( u\right) \;i = 0,1,\cdots ,n \tag{5}\]

where \( {B}_{i,7}\left( u\right) \) is the basis function,and \( {d}_{i} \) represents the control vertices. The 1st,2nd,and 3rd derivatives of \( p\left( u\right) \) stand for velocity, acceleration, and jerk at the normalized time node vector \( u \in  \left( {{u}_{i},{u}_{i + 1}}\right) \) ,respectively.

Discretizing the trajectory, the discrete Cartesian position points are converted to a position-time sequence through inverse kinematics:

\[N = \left( {{p}_{i},{t}_{i}}\right) \;i = 0,1,\cdots ,n \tag{6}\]

In equation (6), \( {t}_{i} \) is the time node. The control vertices are solved in a matrix form. It is possible to denote it through:

\[{A}_{n} \cdot  {d}_{n} = {p}_{n} \tag{7}\]

where \( {A}_{n} \) represents the coefficient matrix, \( {d}_{n} = \left\lbrack  {d}_{i,0}\right. \) \( {\left. {d}_{i,1}\cdots {d}_{i,n + 1}{d}_{i,n + 2}\cdots {d}_{i,n + 6}\right\rbrack  }^{T} \) and \( {p}_{n} = \left\lbrack  {p}_{i,0}\right. \) \( {p}_{i,1}\cdots {p}_{i,n}{V}_{i,s}{V}_{i,e}{A}_{i,s}{A}_{i,e}{J}_{i,s}{\left. {J}_{i,e}\right\rbrack  }^{T} \) are control points and positions respectively.

Due to the BSCs' convex hull property containing control points, it can be inferred that the constraints on the 7th-order BSCs' control points satisfy:

\[\left\{  \begin{array}{ll} \max \left( \left| {d}_{mi}^{1}\right| \right)  \leq  k{v}_{cm} & i = 1,2,\cdots ,n + 6 \\  \max \left( \left| {d}_{mi}^{2}\right| \right)  \leq  k{a}_{cm} & i = 2,3,\cdots ,n + 6 \\  \max \left( \left| {d}_{mi}^{3}\right| \right)  \leq  k{j}_{cm} & i = 3,4,\cdots ,n + 6 \end{array}\right.  \tag{8}\]

where \( {v}_{cm},{a}_{cm},{j}_{cm} \) respectively represent the constraints on joint velocity, acceleration, and jerk of the manipulator, \( {d}_{mi}^{1},{d}_{mi}^{2},{d}_{mi}^{3} \) are calculated based on the De-Boor recursive formula. Therefore, the kinematic constraints have been transformed into constraints on the control vertices.

### 3.2 Time Optimization Via PSO algorithm

The goal of PSO algorithm is to minimize the time required for the manipulator to complete a trajectory within the constrained velocity range of each joint[8].

\[f\left( x\right)  = \min \mathop{\sum }\limits_{{i = 0}}^{{n - 1}}{t}_{i} \tag{9}\]

\[\mathop{\max }\limits_{{i = 1,2,\cdots ,n + 6}}\left\{  \left| {{d}_{mi}^{1}\left( x\right) }\right| \right\}   - {v}_{cm} \leq  0\]

\[\mathop{\max }\limits_{{i = 2,3,\cdots ,n + 6}}\left\{  \left| {{d}_{mi}^{2}\left( x\right) }\right| \right\}   - {a}_{cm} \leq  0 \tag{10}\]

\[\mathop{\max }\limits_{{i = 3,4,\cdots ,n + 6}}\left\{  \left| {{d}_{mi}^{3}\left( x\right) }\right| \right\}   - {j}_{cm} \leq  0\]

In equation (10), \( x = {\left\lbrack  {x}_{0},{x}_{1},\cdots ,{x}_{n - 1}\right\rbrack  }^{T},{x}_{i} = \Delta {t}_{i} = \) \( {t}_{i + 1} - {t}_{i},i = 0,1,\cdots ,n - 1,\Delta {t}_{i} \) has a lower bound.

The PSO algorithm is utilized for settling the optimization issue with nonlinear constrained outlined in (10). And PSO algorithm's schematic in accordance with figure.

<!-- Media -->

<!-- figureText: Y Start Initialize Particle Swarm Parameters Randomly initialize the position and velocity of each particle Satisfy Termination Criteria LN Update the velocity and position of each particle Calculate the fitness of each particle Update the personal best fitness value and position of each particle Update the global best fitness value and position of the swarm Update other parameters Plot the optimal solution End -->

<img src="https://cdn.noedgeai.com/bo_d2slbk3ef24c73b2kgeg_1.jpg?x=844&y=788&w=683&h=866&r=0"/>

Fig. 1: PSO Algorithm

<!-- Media -->

### 3.3 Tracking Controller Design

The trajectory tracking controller for the manipulator is developed via the direct parameter approach within the framework of a HOFAS model:

\[\left\{  \begin{array}{l} u = {u}_{c} + {u}_{f} \\  {u}_{c} = {B}^{-1}\left( {\theta ,q,\dot{q}}\right) \xi \left( {\theta ,q,\dot{q}}\right) \\  {u}_{f} = {K}_{0}\left( {\theta ,q,\dot{q}}\right) q + {K}_{1}\left( {\theta ,q,\dot{q}}\right) \dot{q} + v \end{array}\right.  \tag{11}\]

In equation (11), \( {u}_{c} \) represents the compensatory controller,and \( {u}_{f} \) represents the state feedback controller, \( {K}_{0}\left( {\theta ,q,\dot{q}}\right) ,\;{K}_{1}\left( {\theta ,q,\dot{q}}\right)  \in  {R}^{6 \times  6} \) denote feedback gain matrixs,and \( v \) represents the external signal.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:53 UTC from IEEE Xplore. Restrictions apply. 223-->




<!-- Media -->

<!-- figureText: State Feedback \( u \) Fully Actuated Manipulator System Controller Compensator Controller \( \xi \left( {\theta ,q,\dot{q}}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2slbk3ef24c73b2kgeg_2.jpg?x=122&y=145&w=680&h=307&r=0"/>

Fig. 2: Structure diagram of the fully actuated control system

<!-- Media -->

Convert it into the following first-order system:

\[\dot{X} = {A}_{c}X + {B}_{c}v \tag{12}\]

where \( X = {\left\lbrack  \begin{array}{ll} q & \dot{q} \end{array}\right\rbrack  }^{T},{B}_{c} = {\left\lbrack  \begin{array}{ll} 0 & {A}_{2}^{-1}B \end{array}\right\rbrack  }^{T} \) ,

\[{A}_{c} = \left\lbrack  \begin{matrix} 0 & {I}_{n} \\   - {A}_{2}^{-1}\left( {{A}_{0} - B{K}_{0}}\right) &  - {A}_{2}^{-1}\left( {{A}_{1} - B{K}_{1}}\right)  \end{matrix}\right\rbrack  \]

According to the defined expected system performance, if the canonical form \( F \in  {R}^{{2n} \times  {2n}} \) and \( \exists Z \in  {R}^{n \times  {2n}} \) satisfy:

\[\det {V}_{e}\left( {Z,F}\right)  \neq  0 \tag{13}\]

In equation (13),it has the following relation: \( {V}_{e}\left( {Z,F}\right)  = \) \( \left\lbrack  \begin{array}{ll} V\left( {Z,F}\right) & {V}_{\infty }^{c} \end{array}\right\rbrack  ,V\left( {Z,F}\right)  = \left\lbrack  \begin{array}{ll} Z & {ZF} \end{array}\right\rbrack  ,V \) represents the finite eigenvectors associated with \( F,{V}_{\infty }^{c} \) denotes generalized infinite eigenvector matrix, \( \left\lbrack  \begin{array}{ll} V & {V}_{\infty }^{c} \end{array}\right\rbrack \) serves as the overall eigenvector matrix. To increase the speed of tracking the target trajectory in the system, the performance index is expressed:

\[J\left( {F,Z}\right)  = \parallel V\parallel \begin{Vmatrix}{V}^{-1}\end{Vmatrix} \tag{14}\]

The matrix \( Z \) can be obtained by minimizing \( J \) ,and then substituting \( Z \) and \( F \) into equation (15):

\[\left\{  \begin{array}{l} \left\lbrack  \begin{array}{ll} {K}_{0} & {K}_{1} \end{array}\right\rbrack   = W{V}^{-1}\left( {Z,F}\right) \\  W = {B}^{-1}\left( {{A}_{2}Z{F}^{2} + {A}_{1}{ZF} + {A}_{0}Z}\right)  \end{array}\right.  \tag{15}\]

Equation (15) determines the value of \( {K}_{0} \) and \( {K}_{1} \) ,thereby obtaining the desired first-order steady-state system.

## 4 Simulation

### 4.1 Obtaining Trajectories Towards the Target

In an effort to prove the practicability of PSO algorithm to plan trajectories that minimize time, Matlab is used for simulation experiments. The position sequence required for each joint of the manipulator is shown in Table 2.

Set \( {V}_{\max } = \left\lbrack  \begin{array}{llllll} {100} & {95} & {100} & {150} & {130} & {110} \end{array}\right\rbrack  ,{A}_{\max } = \) \( \left\lbrack  \begin{array}{llllll} {45} & {40} & {75} & {70} & {90} & {80} \end{array}\right\rbrack  ,{J}_{\max } = \left\lbrack  \begin{array}{llllll} {60} & {60} & {55} & {70} & {75} & {70} \end{array}\right\rbrack \) . the PSO algorithm for optimizing time-efficient trajectories. Utilizing 50 iterations and a swarm comprised of 50 particles, the number of particles in the swarm to 50 . Set the inertia weight to 0.5 , the individual learning factor to 1 , and the social learning factor to 2 .

From the simulation results (Fig.3), it can be observed that the start-stop velocities and accelerations are zero, which fulfills the set requirements. Additionally, the curves in Fig.3(b-d) are continuous without any abrupt changes, and they comply with the kinematic constraints, ensuring smooth motion. Using the PSO algorithm to find the best motion time while adhering to kinematic constraints, the maximum time required to execute the trajectory is \( {39.60}\mathrm{\;s} \) ,which is relatively short and meets the requirements.

<!-- Media -->

Table 2: Sequence of joint positions

<table><tr><td/><td>Joint1</td><td>Joint2</td><td>Joint3</td><td>Joint4</td><td>Joint5</td><td>Joint6</td></tr><tr><td>M1</td><td>16.99</td><td>-33.12</td><td>43.89</td><td>25.70</td><td>110.36</td><td>-25.95</td></tr><tr><td>M2</td><td>17.23</td><td>-13.88</td><td>47.70</td><td>60.69</td><td>105.23</td><td>-28.51</td></tr><tr><td>M3</td><td>17.88</td><td>-19.60</td><td>55.23</td><td>79.05</td><td>103.25</td><td>-16.05</td></tr><tr><td>M4</td><td>14.39</td><td>-28.46</td><td>63.88</td><td>75.21</td><td>97.78</td><td>-5.21</td></tr><tr><td>M5</td><td>11.25</td><td>-41.72</td><td>68.35</td><td>72.70</td><td>86.04</td><td>6.50</td></tr><tr><td>M6</td><td>3.100</td><td>-45.34</td><td>72.41</td><td>67.45</td><td>79.94</td><td>12.20</td></tr><tr><td>M7</td><td>-20.13</td><td>-42.69</td><td>79.65</td><td>53.54</td><td>72.76</td><td>16.24</td></tr><tr><td>M8</td><td>-25.78</td><td>-40.34</td><td>82.17</td><td>47.80</td><td>78.16</td><td>13.55</td></tr></table>

<!-- figureText: joint angular velocity \( \mathrm{v}/\left( {}^{ \circ  }\right) {\mathrm{s}}^{-1} \) 15 -Joint I Joint2 Joints 39.60 20 30 40 -Joint1 b 39.60 20 30 40 -Joint I ............ Joint4 Joint6 39.60 20 30 40 ............ Joint4 Joint2 Joint5 Joint6 39.60 30 40 Time t/s 10 -5 10 15 10 Joint angular velocity \( \mathrm{v}/\left( {}^{ \circ  }\right) {\mathrm{s}}^{-1} \) 15 10 -5 -10 -15 0 10 Joint angular acceleration \( \mathrm{v}/\left( {}^{ \circ  }\right) \mathrm{s} \) 6 2 0 -4 -6 -8 -10 0 10 6 Joint jerk \( \mathrm{v}/\left( {}^{ \circ  }\right) {\mathrm{s}}^{-3} \) 2 0 0 10 -->

<img src="https://cdn.noedgeai.com/bo_d2slbk3ef24c73b2kgeg_2.jpg?x=858&y=634&w=669&h=1364&r=0"/>

Fig. 3: Trajectory Plot of Joint

<!-- Media -->

### 4.2 Trajectory Tracking Control

In an effort to test and verify the significance of the control algorithm. Matrix \( F \) is selected according to the anticipated system feature configuration and is a diagonal matrix containing eighteen elements. If we set the standard type \( F : \mathrm{F} = \) diag( - 50, - 50, - 50, - 100, - 100, - 100, - 90, - 90, - 90, - 45, \( - {45}, - {45}) \) . According to the optimization function equation (14),use the fmincon solver in Matlab software to obtain \( Z \) .

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:53 UTC from IEEE Xplore. Restrictions apply. 224-->




<!-- Media -->

<!-- figureText: 20 a 39.60 20 30 40 Actual tracking trajectory Reference trajectory b 39.60 20 30 40 C Actual tracking trajectory Reference trajectory 39.60 30 40 Joint angle q/(°) 10 10 20 Actual tracking trajectory Reference trajectory -30 0 10 Joint angle q/( \( {}^{ \circ  } \) ) -20 -40 -60 10 100 80 Joint angle q/(°) 60 40 20 10 -->

<img src="https://cdn.noedgeai.com/bo_d2slbk3ef24c73b2kgeg_3.jpg?x=126&y=342&w=678&h=1024&r=0"/>

Fig. 4: Joint 1-3 Angle Tracking

<!-- Media -->

The direct parameter method control model is constructed to control the manipulator system, and the results of tracking the reference trajectory of each joint of the manipulator are plotted, as shown in Fig.4-Fig.5. From the experimental results, it is evident that the system may quickly follow the target trajectory within 3 seconds. And manipulator can quickly respond to the designed time-optimal and jerk-continuous reference trajectory and achieve precise tracking.

## 5 Conclusion

This paper focuses on the 6-DOF manipulator and analy its kinematic model and fully actuated model. A trajectory planning scheme via the BSC interpolation approach and employ the PSO algorithm for obtaining the best desired trajectory. With the foundation in the fully actuated system control theory, a trajectory tracking controller has been introduced employing the direct parameter method to track the desired trajectory. The problem of the optimal time and jerk continuous trajectory planning and tracking control of the manipulator is solved, and the continuity and tracking precision of the trajectory are improved. And better results are achieved in the aspects of trajectory time and tracking error. References

<!-- Media -->

<!-- figureText: 10 20 30 40 d Actual tracking trajectory Reference trajectory 39.60 20 30 40 e Actual tracking trajectory Reference trajectory 39.60 20 30 40 Actual tracking trajectory Reference trajectory 39.60 20 30 40 Time (s) 80 Joint angle q/(°) 60 40 20 0 10 120 Joint angle q/(°) 80 40 10 30 Joint angle q/(°) 20 10 -10 -20 -30 10 -->

<img src="https://cdn.noedgeai.com/bo_d2slbk3ef24c73b2kgeg_3.jpg?x=849&y=145&w=680&h=1050&r=0"/>

Fig. 5: Joint 4-6 Angle Tracking

<!-- Media -->

[1] Sun Y, et al. Learn how to assist humans through human teaching and robot learning in human-robot collaborative assembly. IEEE Transactions on Systems, Man, and Cybernetics: Systems, 2022, 52(2): 728-738.

[2] Zhang H, Meng Z, et al. Trajectory optimization of glue spraying on athletic footwear sole based on multivariable spray torch model. IEEE Access, 2023, 11: 79422-79433.

[3] Sun H H, Zhang Y J, et al. Dynamic modeling and error analysis of a cable-linkage serial-parallel palletizing robot. IEEE Access, 2021, 9: 2188-2200.

[4] Leveziel M, et al. A 4-DOF parallel robot with a built-in gripper for waste sorting. IEEE Robotics and Automation Letters, 2022, 7(4): 9834-9841.

[5] Duan G R. High-order system approaches: I. Fully-actuated systems and parametric designs. Acta Automatica Sinica, 2020, 46(7): 1333-1345.

[6] Duan G R. A direct parametric approach for missile guidance-case of sea targets. Proceedings of the 33rd Chinese Control Conference, Nanjing, China, 2014: 1044-1050.

[7] Sun H, Huang L. Trajectory tracking control of a 6-DOF manipulator. 2023 2nd Conference on Fully Actuated System Theory and Applications (CFASTA), 2023: 775-782.

[8] Kim J J, et al. Trajectory optimization with particle swarm optimization for manipulator motion planning. IEEE Transactions on Industrial Informatics, 2015, 11(3): 620-631.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:53 UTC from IEEE Xplore. Restrictions apply. 225-->

