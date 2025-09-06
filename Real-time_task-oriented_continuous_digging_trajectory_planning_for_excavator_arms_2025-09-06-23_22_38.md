

<!-- Meanless: Automation in Construction 152 (2023) 104916 Contents lists available at ScienceDirect Automation in Construction ELSEVIER journal homepage: www.elsevier.com/locate/autcon updates-->

# Real-time task-oriented continuous digging trajectory planning for excavator arms

Zongwei Yao, Shichao Zhao, Xiaodan Tan \( {}^{ * } \) , Wen Wei, Yong Wang

Key Laboratory of CNC Equipment Reliability, Ministry of Education, School of Mechanical and Aerospace Engineering, Jilin University, Changchun, 130025, China

## ARTICLEINFO

Keywords:

Excavators

Trajectory planning

Task-oriented

Real-time planning

PINN

## A B S T R A C T

Current digging trajectory planning methods for excavator arms are limited to a single digging cycle, which does not meet the continuous excavation demands of the task. To address this issue, a real-time task-oriented continuous digging trajectory planning method for autonomous excavators is presented. The method involves optimizing digging trajectory for a single excavation cycle using multi-objective PSO method, building a PINN model using optimization results as training samples for real-time planning, and embedding the PINN model in planning framework for typical tasks. The method was validated using four different cross-section shapes of the trench. Results show that the average time taken to plan a single digging trajectory is less than 4.5ms, which is negligible compared to the time taken by PSO. The overall performance of the trajectory is only about 5% different from those planned by PSO. This method offers a more efficient and effective solution for continuous excavation tasks.

## 1. Introduction

Construction machinery play an irreplaceable role in economic development. As one of the typical construction machinery, excavators are widely used in urban infrastructure, farmland water conservancy, disaster relief, mining, construction and other fields because of its high efficiency and economic advantages.

At present, excavators are mainly operated by the operator on-site, which follows with many problems \( \left\lbrack  {1,2}\right\rbrack \) . Additionally,the construction industry is facing the challenge caused by labor shortage and skill gap. On the one hand, the aging problem causes a growing shortage of labor. On the other hand, the accumulation of working experience needs long period of training and large social resources. Therefore, autonomous machine operation is a great solution for these challenges [3].

Since the 1970s, research communities have been committed to realizing the intelligence and automation of the excavating process, resulted in a number of typical intelligent excavator systems, such as Autonomous Loading System (ALS) introduced by Cannon [4], Intelligent Exclusion System (IES) introduced by Seo [5] and Lancaster university computerized intelligent excavator (LUCIE) introduced by Gu [6].

Digging trajectory planning is vital for autonomous excavation because the trajectories directly determine the digging efficiency and power consumption [7]. Trajectory planning is a kind of optimization task taking the geometrical parameters of the trajectory or motion matrix of the working device as variables [8]. Some researchers treat the planning as a multi-objective optimization [9,10] while others focus on one specific performance \( \left\lbrack  {{11},{12}}\right\rbrack \) . For the selection of methods,both traditional \( \left\lbrack  {{13},{14}}\right\rbrack \) and biology-inspired \( \left\lbrack  {{15},{16}}\right\rbrack \) optimization methods are commonly applied for planning. Indicators such as digging time, energy consumption and bucket fullness are generally used for the evaluation. Specially, for autonomous excavation, the computation time for trajectory planning is important as it significantly influence the total operation time of one digging cycle.

In recent years, the combination of classic optimization algorithm and machine learning has been increasingly applied on digging trajectory planning [17,18]. The strong ability of nonlinear relationship approximation and the high efficiency of neural network are the two main reasons for the popularity. However, it is insufficient to build the planning model completely based on data without physical constraints. But unfortunately, loss functions corresponding to physical performance do not exist in most of the networks. Under such circumstances, PINN seem to be an reasonable option because it applies additional physical constraints described by nonlinear partial differential equations in the training process to get a more generalized model. PINN has attracted wide attention in various fields soon after its publication in 2018 [19]. Raymond et al. Raymond et al. [20] proposed a PINN model for test of head attack which resulted in better performance in both accuracy and efficiency than traditional methods. Zheng et al. Zheng et al. [21] proposed a physical based semantic repair framework based on PINN to improve the image inpainting performance by overcoming the limitation of normal framework that only direct measurement data were included. Additionally, PINN has also been applied in researches on dynamic performance prediction. Fu et al. Fu et al. [22] applied a novel hybrid PINN for dynamic digging force prediction of mining shovels. Xu et al. Xu et al. [23] took PINN for dynamic performance prediction of unmanned surface vehicles and achieved better generalization performance with fewer training samples. Based on the above description, it can be concluded that the unique ability to take physical performance as loss function makes PINN suitable for engineering applications. Besides, comparing with other networks, PINN requires a relatively smaller number of samples and fewer computation resource, which makes it a reasonable choice for field operations.

---

<!-- Footnote -->

* Corresponding author.

E-mail addresses: yzw@jlu.edu.cn (Z. Yao), zhaosc20@mails.jlu.edu.cn (S. Zhao), tanxd21@mails.jlu.edu.cn (X. Tan), weiwen21@mails.jlu.edu.cn (W. Wei), wyong22@mails.jlu.edu.cn (Y. Wang).

URL: https://teachers.jlu.edu.cn/ZongweiYao/zh_CN/index.htm (Z. Yao).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.autcon.2023.104916 Received 19 January 2023; Received in revised form 25 April 2023; Accepted 30 April 2023 Available online 12 May 2023 0926-5805/(C) 2023 Elsevier B.V. All rights reserved.-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

In order to complete a preset task, excavator often needs to carry out a group of repeated and continuous operation cycles. Therefore, continuous trajectory planning is a necessity and the key to the feasibility of intelligent excavator. An efficient real-time digging planning method based on PINN is proposed in this paper. Being different from most of the existed methods that only plan one trajectory for the current excavation, this proposed method can generate a group of trajectories based on the overall task for the autonomous continuous digging operation.

To be more specific, PSO is used for one-shot trajectory planning, and the optimization results are taken as the PINN training samples. After the training process, the PINN model is embedded into the task-oriented continuous trajectory planning framework. And finally, a series of trajectories can be quickly planned according to specific task demands. It should be noted that PSO is a widely used biology-inspired method for multi-objective robotic arm trajectory planning [24-26]. Comparing with other intelligent optimization methods, PSO has fewer parameters with simpler theoretical principle, making it a reliable option for complex applications.

The rest of this article is organized as follows. Section 2 summarizes the related works of trajectory planning for excavator arms. Section 3 gives a brief introduction to the basic work of this paper. Section 4 introduces the proposed method in detail. Section 5 verifies the feasibility of the method through experiments and results analysis. This is followed by the conclusion in Section 6, summarizing the advantages and disadvantages of the proposed method.

## 2. Related works

As mentioned above, the method proposed in this paper involves trajectory planning method and machine learning technology, this section will review the relevant research of these two aspects respectively.

### 2.1. Research on trajectory planning method

The digging trajectory planning of excavator arms refers to building a motion sequence from the starting point to the end point under the premise of satisfying the relevant constraints. Some scholars tried to improve one specific performance by an optimal digging strategy while some others taking the planning as an multi-objective optimization problem.

Wang et al. Wang et al. [11] planned the trajectory aiming at the lowest energy consumption based on different polynomial equations, but the velocity and acceleration were not constrained in the trajectory planning. Elnagar et al. Elnagar and Hussein [12] used the numerical iteration method to obtain the optimal trajectory in terms of energy consumption in a 3D environment with certain boundary conditions. Martln et al. Martin and Bobrow [27] optimized the joint torque function for the minimum driving power based on the B spline curve. Yoo et al. 2010 optimized the displacement curve of the excavator arms for the minimum energy consumption based on the B spline curve. Nagy et al. Nagy and Vajk [13] used the direct conversion method to generate the velocity curves and realized the global time optimization of excavator arms. Sun et al. Sun et al. [15] used PSO to optimize the interpolation time of piecewise polynomials. Liu et al. Liu et al. [14] used B-spline curves to interpolate in joint space and used sequential quadratic programming (SQP) to optimize the trajectory with the shortest time, but they did not take into account the adverse effects of large impact on excavators. Boryga et al. Boryga and Graboó [29] used a higher-order polynomial with only one unknown parameter to achieve time optimal trajectory planning, for which the maximum acceleration value of the terminal moving along the path must be given. Wang et al. Wang et al. [30] obtained the point-to-point time optimal trajectory based on the multi root form interpolation expression, but each joint obtained by the solution did not reach the limits of the angular velocity, acceleration and acceleration synchronously. Barnett et al. Barnett and Gosselin [31] used the dichotomy method for time optimal trajectory planning, and the algorithm has strong robustness and fast calculation speed. Jud et al. Jud et al. [32,33] defined the digging trajectory of a single cycle through end-execution force-torque, which could be applied to different types of soil, and proposed a large-scale iterative planner to continuously execute single digging until the desired ground geometry was reached.

Generally, the single-object optimization cannot meet all the operation requirements. Hence, many scholars have applied multi-objective optimization on digging trajectory planning.

Wang et al. Wang et al. [16] used the Beetle Swarm algorithm to optimize the trajectory of time and joint rotation angle. Zhang et al. Zhang et al. [9] used SQP method, interpolated cubic splines in joint space, and found the best Time-jerk trajectory with joint angular velocity, angular acceleration and jitter SQP as constraints. The optimal trajectory was planned by Zhao et al. Zhao et al. [10] considering the mixed objectives of optimization time, energy and acceleration. Huang et al. Huang et al. [34] interpolated in joint space through fifth order B-spline, and then used NSGA-II algorithm to optimize time and jitter. Chen et al. Chen et al. [35] proposed a multi-objective trajectory planning method based on improved immune clone selection algorithm (IICSA) for time, energy consumption and stability, using a quintic B-spline curve to plan the motion trajectory of the mobile platform in Cartesian space. Kim et al. Kim et al. [36] proposed an algorithm for generating dynamic optimal excavation trajectory, and used recursive geometry algorithm to accomplish the trajectory planning for the shortest time and the minimum torque. Zou et al. Zou et al. [37] used B-spline to carry out multi-objective trajectory planning for digging time, energy utilization and mechanical maintenance. Yang et al. Yang et al. [38] developed an planning frame work for digging trajectory aiming at the optimal bucket fullness and excavation time, which was tested by simulations. Lee et al. Lee et al. [39] proposed a real-time planning algorithm for one-shot digging task of hydraulic excavators. The algorithm consists of global planner and local planner. In the global planning stage, Bernstein parameterization was used to generate the trajectory that maximized the fill factor and minimized the energy consumption under the operational constraints. The high-fidelity local planner can track the generated global trajectory and meet the physical constraints in real time. The algorithm was implemented in simulation, but has not been verified in actual excavator. Zhang et al. [40] proposed a point to point excavation trajectory planning method to rapidly obtain a reasonable mining trajectory to improve both the mining efficiency and fill factor and to reduce the energy consumption. Zhao et al. Zhao et al. [41] proposed a trajectory generation method for autonomous excavator teach-and-plan applications which transforms the arbitrary slow and jerky trajectory of human excavation into a topologically equivalent path that is guaranteed to be fast, smooth and dynamically feasible.

Based on the above description, it can be concluded that profound academic achievements have been made on digging trajectory planning. However, for the single-objective optimization method, although it has the advantage in computation efficiency, it cannot perfectly meet the various of requirements. On the other hand, multi-objective optimization method apparently need large computation resources which is not suitable for real-time applications. Because of the ability to take the advantage of big data, machine learning appears to be the key for such bottleneck problem. In recent years, more and more scholars have tried to apply machine learning on digging trajectory planning.

<!-- Meanless: 2-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

### 2.2. Application of machine learning on excavation

Kurinov et al. Kurinov et al. [17] proposed an algorithm based on reinforcement learning, which combined multi-body method, near-end strategy optimization and covariance matrix adaptive learning algorithm to simulate digging process. Osa et al. Osa and Aizawa [42] proposed a learning policy based on Qt-Opt, which is a variant of Q-learning algorithms for continuous action space, for planning the trajectory of the excavator bucket using depth images. Vu et al. Vu et al. [43] proposed a reference trajectory generation algorithm for excavator arms in dynamic environment by using recursive neural network and model reference adaptive controller (MRAC). The algorithm changes the trajectory according to variation in material, so as to maximize the fill factor. Son et al. Son et al. [18] proposed a trajectory planning algorithm of kinematics and dynamics strategies that can effectively learn and retrieve experts. The robustness of the algorithm was guaranteed by DMP learning the human expert digging trajectory that interacts with complex soil models. Meanwhile, the online trajectory modulation was added to prevent excessive digging and adapt to the dynamic changes of the ground. In view of the complex and difficult quantification of soil properties in the digging process. Lee et al. Lee et al. [44] identified soil parameters through neural networks, thus generating the excavation trajectory of the bucket. Zhang et al. Zhang et al. [45] reported an approach that combines the strength of inverse reinforcement learning and data-driven imitation learning with the efficiency of optimization-based methods to uncover the motion pattern of human-operated excavation movements, which was integrated with a stochastic optimization-based algorithm for trajectory generation for scooping and dumping motions. Zhao et al. Zhao and Zhang [46] proposed a neural digging task planner-TaskNet for autonomous excavators containing two convolutional neural networks, which can effectively learn the demonstrated task decomposition strategy and generate reasonable task plans and digging trajectories for different tasks. Hodel et al. Hodel [47] applied reinforcement learning to accomplish the grading operation using excavator. Wang et al. Wang et al. [48] applied two ANFIS(Adaptive Neural Fuzzy Inference System) to build the inverse kinematics model of excavator arms, which was used for trajectory planning and tracking.

It can be summarized from the literature listed above that machine learning has shown great potential in complex engineering applications. However, it should noted that most of the current research focuses on the one-shot digging trajectory. From the perspective of the whole task execution, this paper proposes a continuous trajectory planning method based on neural network for autonomous digging operations with optimal performance in digging time, fill factor and energy consumption.

## 3. Basic works

### 3.1. Mechanism of excavator arms

In this paper, the D-H method [49] and the Lagrangian method [50, 51] are used to establish the kinematics and dynamics models of excavator arms respectively, and the resistance during the digging process is analyzed to lay the foundation for the subsequent research.

<!-- Media -->

<!-- figureText: \( {O}_{1}\left( {Z}_{1}\right) \) \( \left( {Z}_{2}\right) \) \( {X}_{3} \) \( \left( {Z}_{3}\right)  ○ \) \( {Y}_{3} \) \( \left( {Z}_{4}\right) O \) \( {X}_{1} \) \( {Y}_{41} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_2.jpg?x=914&y=159&w=720&h=436&r=0"/>

Fig. 1. D-H coordinate systems of excavator arms.

<!-- Media -->

#### 3.1.1. Kinematics model

The principle of \( \mathrm{D} - \mathrm{H} \) method is to establish a coordinate system on each rod in a multi-linkage system to describe the mutual position and motion relationship between adjacent bars. In this section, the \( \mathrm{D} - \mathrm{H} \) coordinate system is established to describe the motion state of excavator arms.

Fig. 1 shows the schematic diagram of the excavator arms established in the D-H coordinate systems (without considering swing). Where, \( {O}_{1}{X}_{1}{Y}_{1}{Z}_{1} \) is the boom coordinate system (set as the base coordinate system), \( {O}_{2}{X}_{2}{Y}_{2}{Z}_{2} \) is the stick coordinate system, \( {O}_{3}{X}_{3}{Y}_{3}{Z}_{3} \) is the bucket coordinate system, \( {O}_{4}{X}_{4}{Y}_{4}{Z}_{4} \) is the bucket tip coordinate system. \( {l}_{1} \) is the boom length, \( {l}_{2} \) is the stick length, \( {l}_{3} \) is the bucket length. \( {\theta }_{1} \) is the boom joint angle, \( {\theta }_{2} \) is the stick joint angle, \( {\theta }_{3} \) is the bucket joint angle, \( \zeta \) is the value of bucket tip attitude angle.

The forward kinematics solution is to convert the joint space to the position and attitude space, that is, given a set of joint angles \( \left\lbrack  {{\theta }_{1},{\theta }_{2},{\theta }_{3}}\right\rbrack \) ,to solve the bucket tip’s position \( \left\lbrack  {x,y,z}\right\rbrack \) in the Cartesian coordinate system and the attitude angle \( \zeta \) . As shown in Fig. 1,by multiplying the homogeneous transformation matrix \( {T}_{i + 1}^{i} \) between adjacent coordinate systems,the transformation matrix \( {\mathbf{T}}_{4}^{1} \) from bucket coordinate system \( {O}_{4}{X}_{4}{Y}_{4}{Z}_{4} \) to base coordinate system \( {O}_{1}{X}_{1}{Y}_{1}{Z}_{1} \) can be obtained, expressed as:

\[{\mathbf{T}}_{4}^{1} = \left\lbrack  \begin{matrix} {c}_{123} &  - {s}_{123} & 0 & {l}_{1}{c}_{1} + {l}_{2}{c}_{2} + {l}_{3}{c}_{123} \\  {s}_{123} & {c}_{123} & 0 & {l}_{1}{s}_{1} + {l}_{2}{s}_{12} + {l}_{3}{s}_{123} \\  0 & 0 & 1 & 0 \\  0 & 0 & 0 & 1 \end{matrix}\right\rbrack   \tag{1}\]

The attitude angle of bucket tip can be expressed as:

\[\zeta  = {\theta }_{1} + {\theta }_{2} + {\theta }_{3} \tag{2}\]

The solution of the forward kinematics of the excavator arms can be obtained as follows:

\[\left\{  \begin{array}{l} x = {l}_{1}{c}_{1} + {l}_{2}{c}_{12} + {l}_{3}{c}_{123} \\  y = {l}_{1}{s}_{1} + {l}_{2}{s}_{12} + {l}_{3}{s}_{123} \\  z = 0 \\  \zeta  = {\theta }_{1} + {\theta }_{2} + {\theta }_{3} \end{array}\right.  \tag{3}\]

where, \( {c}_{i} = \cos {\theta }_{i},{s}_{i} = \sin {\theta }_{i},{c}_{ij} = \cos \left( {{\theta }_{i} + {\theta }_{j}}\right) ,{s}_{ij} = \sin \left( {{\theta }_{i} + {\theta }_{j}}\right) \) , \( {c}_{ijk} = \cos \left( {{\theta }_{i} + {\theta }_{j} + {\theta }_{k}}\right) ,{s}_{ijk} = \sin \left( {{\theta }_{i} + {\theta }_{j} + {\theta }_{k}}\right) . \)

The inverse kinematics solution is to convert the position and attitude space to the joint space, that is, given the bucket tip's position \( \left\lbrack  {x,y,z}\right\rbrack \) in the Cartesian coordinate system and attitude angle \( \zeta \) ,to solve the joint Angles \( \left\lbrack  {{\theta }_{1},{\theta }_{2},{\theta }_{3}}\right\rbrack \) . As shown in Fig. 1 (regardless of swing, i.e., \( z = 0 \) ),the solution of the inverse kinematics problem of the excavator arms can be obtained by geometric method, express as:

<!-- Meanless: 3-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

\[\left\{  \begin{array}{l} {\theta }_{1} = {\alpha }_{1} + {\alpha }_{2} + {\alpha }_{3} \\  {\theta }_{2} = \angle {O}_{1}{O}_{2}{O}_{3} - \pi \\  {\theta }_{3} = \zeta  - {\theta }_{1} - {\theta }_{2} \end{array}\right.  \tag{4}\]

where, \( {\alpha }_{1} = \angle {O}_{2}{O}_{1}{O}_{3},{\alpha }_{2} = \angle {O}_{3}{O}_{1}{O}_{4},{\alpha }_{3} = \angle {O}_{4}{O}_{1}{X}_{1} \) ,can be calculated by geometric relationship.

#### 3.1.2. Dynamics model

The dynamics model of the excavator arms can be obtained through the Lagrange functions of the kinetic energy and potential energy of all the rods, and then substituting them into the Lagrange equation. The Lagrange function defined as:

\[{f}_{L} = {E}_{k} - {E}_{p} \tag{5}\]

where, \( {E}_{k} \) is the kinetic energy of the whole excavator arms, \( {E}_{p} \) is the potential energy.

The Lagrange equation can be further described as:

\[{\tau }_{i} = \frac{\mathrm{d}}{\mathrm{d}t}\left( \frac{\partial {E}_{k}}{\partial {\dot{q}}_{i}}\right)  - \frac{\partial {E}_{k}}{\partial {\dot{q}}_{i}} + \frac{\partial {E}_{p}}{\partial {\dot{q}}_{i}} \tag{6}\]

where, \( {q}_{i} \) is the generalized coordinate,here means joint angle \( {\theta }_{1},{\theta }_{2} \) , and \( {\theta }_{3} \) respectively; \( {\tau }_{i} \) is the generalized force,here means joint torque \( {M}_{1},{M}_{2} \) and \( {M}_{3} \) respectively; \( {\dot{q}}_{i} \) is the generalized velocity,here means joint angular velocity \( {\dot{\theta }}_{1},{\dot{\theta }}_{2} \) ,and \( {\dot{\theta }}_{3} \) respectively.

It can be expressed in the form of matrix and vector as:

\[\mathbf{M}\left( q\right) \ddot{q} + \mathbf{C}\left( {q,\dot{q}}\right) \dot{q} + \mathbf{G}\left( q\right)  = \mathbf{\tau } - {\mathbf{F}}_{L} \tag{7}\]

where, \( \mathbf{C} \) is the square term of velocity,representing the influence of centrifugal force and Coriolis force; \( \mathbf{G} \) is the influence term of gravity; \( {\mathbf{F}}_{L} \) is the interaction moment between the bucket and the environment in the digging process.

#### 3.1.3. Digging resistance force simplification

There are three main operation modes for excavator arms: stick operation, bucket operation, mixed operation, etc., and the digging resistance changes along with the operation method. The digging condition selected in this paper requires all joints to move at the same time, that is, mixed digging. According to the trajectory, the digging resistance can be decomposed into tangential resistance \( {W}_{O} \) and normal resistance \( {W}_{f} \) :

\[\left\{  \begin{array}{l} {W}_{Q} = {K}_{0}{bh} \\  {W}_{f} = \mu {W}_{Q} \end{array}\right.  \tag{8}\]

where, \( {K}_{0} \) is the specific resistance coefficient of digging, \( b \) is the width of bucket digging, \( h \) is the depth of digging, \( \mu \) is the digging resistance coefficient.

### 3.2. Digging trajectory description

In the joint space of excavator arms, the joint is driven by the hydraulic/electric cylinder of the boom, stick and bucket to form a compound movement, so that the excavator bucket can complete the planned trajectory. Regardless of the swing in unloading process of the excavator, this paper adopts the 6 -order polynomial to fit the joint angle curve of the excavator arms from the initial point to the termination point of excavation. And the joint angle curve is converted into the trajectory curve of the bucket tip through the forward kinematics relationship. The 6 -order polynomial of the \( i \) th joint can be expressed as:

\[{\theta }_{i}\left( t\right)  = \mathop{\sum }\limits_{{j = 0}}^{6}{a}_{ij}{t}^{j} \tag{9}\]

where, \( {\theta }_{i}\left( t\right) \) is the interpolation function of the \( i \) th joint (where \( i = \) 1,2,3,representing the boom,stick and bucket respectively), \( a \) is the coefficient of the polynomial, \( t \) is the time taken by the three joints to complete the trajectory together.

<!-- Media -->

<!-- figureText: \( {X}_{1} \) DO \( {P}_{3} \) digging direction -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_3.jpg?x=897&y=153&w=746&h=379&r=0"/>

Fig. 2. Trajectory types.

<!-- Media -->

### 3.3. Material volume calculation for continuous digging

As shown in Fig. 2, the excavator locates on the ground surface and the gray area is the cross section of the material to be excavated. \( {P}_{1} \) is the starting point of the first digging, from which the first trajectory is planned, \( {P}_{2} \) is the end point of the trajectory,and the trajectory shape is shown as the blue solid line in Fig. 2. Similarly, the purple solid line connecting \( {P}_{3} \) and \( {P}_{4} \) in Fig. 2 represents the second digging trajectory. Both the blue and purple solid line shown in Fig. 2 can be described by combining Eq. (9) and the kinematic model.

The area enclosed by the planned trajectory and the material surface (such as the area of dark yellow in Fig. 2) is calculated through the microelement method, and then multiplied by the bucket width to get the volume of excavated material.

## 4. Method

### 4.1. Overall method framework

The overall framework of the method mainly includes three parts, as shown in Fig. 3. The first part is the one-shot trajectory planning. Firstly, the optimization model is established, including variables, objectives and constraints. Then, PSO is applied for the optimization, and the trajectory parameters obtained are taken as the output of subsequent training process. The second part is the PINN model, which takes the coordinates of the excavation points as the input and the trajectory parameters as the output. And the loss function based on physics is used for training. The third part is the task-oriented continuous trajectory planning model, in which the trained PINN is embedded to carry out the continuous trajectory planning of excavator arms for typical tasks.

### 4.2. One-shot trajectory planning

The digging resistance force generated from the interaction between the bucket tip and the material can be extremely large if the bucket pose were set improperly. Such phenomenon could be even more obvious in the initial phase of digging because of the surface soil hardening. Hence, it is necessary to limit the initial digging angle into a certain range to ensure a proper entering pose so that the resistance force can be overcome by the driving system. Two different types of digging trajectory are optimized by PSO. The main distinction is the initial digging angle of the trajectory \( {\theta }_{p} \) ,which is defined as the angle between the digging direction and the horizontal plane at the initial digging position shown in Fig. 2. The first type is downward digging trajectory with the value of \( {\theta }_{p} \) ranged from \( {40}^{ \circ  } \) to \( {90}^{ \circ  } \) . And the second type is backward trajectory corresponding to the \( {\theta }_{p} \) less than \( {40}^{ \circ  } \) .

<!-- Meanless: 4-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

<!-- Media -->

<!-- figureText: Part I: One-shot trajectory planning Part III: Task-oriented continuous trajectories planning Enter task End the task requirements Initial excavator Constraints arm posture satisfied? Input X of PINN Execute digging PINN model Output Y of PINN Optimization PSO variables Objectives Constraints Optimal variables Part II: PINN Input \( \mathrm{X} \) of PINN training samples Output Y of training samples Network parameters -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_4.jpg?x=338&y=157&w=1068&h=565&r=0"/>

Fig. 3. Framework of proposed method.

<!-- Media -->

#### 4.2.1. Optimization variables

Given the joint angles of excavator arms corresponding to the trajectory starting point \( \left\lbrack  {{\theta }_{11},{\theta }_{12},{\theta }_{13}}\right\rbrack \) ,the end point \( \left\lbrack  {{\theta }_{21},{\theta }_{22},{\theta }_{23}}\right\rbrack \) ,the polynomial coefficients can be expressed as:

\[\left\{  \begin{array}{l} {a}_{i5} = \frac{6\left( {{\theta }_{2i} - {\theta }_{1i}}\right) }{{t}^{3}} - 3{a}_{i6}t \\  {a}_{i4} = \frac{-{15}\left( {{\theta }_{2i} - {\theta }_{1i}}\right) }{{t}^{4}} - 3{a}_{i6}{t}^{2} \\  {a}_{i3} = \frac{{10}\left( {{\theta }_{2i} - {\theta }_{1i}}\right) }{{t}^{3}} - {a}_{i6}{t}^{3} \\  {a}_{i2} = 0 \\  {a}_{i1} = 0 \\  {a}_{i0} = {\theta }_{1i} \end{array}\right.  \tag{10}\]

where, \( {a}_{i6} \) and \( t \) are variables to be optimized. Therefore,the optimized particle dimension is 4,which are \( {a}_{16},{a}_{26},{a}_{36} \) and \( t \) respectively.

#### 4.2.2. Objectives

Three objectives are set for the optimization of the digging trajectory planned by the 6 -order polynomial: digging time, energy consumption and bucket fill factor. Those three targets are normalized and weighted to obtain the objective function shown in Eq. (11).

\[\left\{  \begin{array}{l} {T}_{t} = \left( {t - {t}_{\min }}\right) /\left( {{t}_{\max } - {t}_{\min }}\right) \\  {E}_{e} = \left( {{W}_{E} - {E}_{\min }}\right) /\left( {{E}_{\max } - {E}_{\min }}\right) \\  {V}_{v} = \left( {{V}_{p\max } - {V}_{p}}\right) /\left( {{V}_{p\max } - {V}_{p\min }}\right)  \end{array}\right.  \tag{11}\]

where, \( t \) is the digging time, \( {t}_{\max } \) and \( {t}_{\min } \) are the time thresholds. \( {W}_{E} \) is the energy consumption of digging, \( {E}_{\max } \) and \( {E}_{\min } \) are the energy consumption thresholds. \( {V}_{p} \) is the volume of excavated material, \( {V}_{\max } \) and \( {V}_{\min } \) are the material volume thresholds. \( {T}_{t},{E}_{e} \) and \( {V}_{v} \) are the normalized optimization targets.

#### 4.2.3. Constraints

The above 6-order polynomial groups should satisfy the following constraints. It should be noted that, all the boundary values listed in Table 2 are set based on the performance limits of the scaled experiment device introduced in Section 5.

1. To prevent large shocks on the excavator at the starting point and the end point of the trajectory, the angular velocity and acceleration at these two points are set to zero. At the same time, the joint angel \( {\theta }_{i} \) ,angular velocity \( {\dot{\theta }}_{i} \) and acceleration \( {\ddot{\theta }}_{i} \) should be continuous and within the range of the limitations listed in Tables 1 and 2.

2. To ensure the feasibility of digging operation, The required driving torque \( {T}_{q} \) and power \( P \) should be less than the maximum value listed in Table 2.

<!-- Media -->

Table 1

\( \mathrm{D} - \mathrm{H} \) parameters of excavator arms.

<table><tr><td>Joint</td><td>Joint offset \( \left( {{d}_{i}/\mathrm{m}}\right) \)</td><td>Joint rod length \( \left( {{l}_{i}/\mathrm{m}}\right) \)</td><td>Joint twist angle \( \left( {{\Phi }_{i}/\mathrm{{deg}}}\right) \)</td><td>Joint angle \( \left( {{\theta }_{i}/\mathrm{{deg}}}\right) \)</td></tr><tr><td>#1</td><td>0</td><td>0.740</td><td>0</td><td>\( - {20} \sim  {65} \)</td></tr><tr><td>#2</td><td>0</td><td>0.350</td><td>0</td><td>\( - {160} \sim  {10} \)</td></tr><tr><td>#3</td><td>0</td><td>0.220</td><td>0</td><td>160~5</td></tr></table>

<!-- Media -->

3. To ensure the rationality of digging, the digging angle at the initial stage of the trajectory should be limited within a certain range. For the first type (downward) of digging trajectory, the value of \( {\theta }_{p} \) ranged from \( {40}^{ \circ  } \) to \( {90}^{ \circ  } \) . And for the second type (backward) of trajectory,the value of \( {\theta }_{p} \) ranged from \( {0}^{ \circ  } \) to \( {40}^{ \circ  } \) .

4. It is obvious that the moving of working device even with an empty bucket would still cause energy consumption. Therefore, a larger volume of the excavated material indicates a higher effective utilization rate of energy. However, on the other hand, an excessively filled bucket might cause overload for the driving system and the dropped material during the moving process would cause energy waste. Hence, the fill factor is generally limited between 0.8 to 1.2 times of the struck volume.

5. The time required for one single digging cycle should be limited within a certain range listed in Table 2.

To sum up, the optimization model is expressed as:

\[\mathop{\min }\limits_{{{a}_{16},{a}_{26},{a}_{36},t}}\;{\lambda }_{1}{T}_{t} + {\lambda }_{2}{E}_{e} + {\lambda }_{3}{V}_{v}\]

\[\text{s.t.}\;{\theta }_{i\min } \leq  {\theta }_{i} \leq  {\theta }_{i\max }\]

\[\max \left\{  \left| {\dot{\theta }}_{i}\right| \right\}   \leq  {\dot{\theta }}_{i\max }\]

\[\max \left\{  \left| {\ddot{\theta }}_{i}\right| \right\}   \leq  {\ddot{\theta }}_{i\max } \tag{12}\]

\[\max \{ P\}  \leq  {P}_{\max }\]

\[\max \left\{  {T}_{q}\right\}   \leq  {T}_{q\max }\]

\[{\theta }_{p\min } \leq  {\theta }_{p} \leq  {\theta }_{p\max }\]

\[{V}_{p\min } \leq  {V}_{p} \leq  {V}_{p\max }\]

\[{t}_{\min } \leq  t \leq  {t}_{\max }\]

where, \( {\theta }_{i\max } \) and \( {\theta }_{i\min } \) are the thresholds for joint angle; \( {\dot{\theta }}_{i} \) and \( {\ddot{\theta }}_{i} \) are the angular velocities and accelerations of each joint angle varying with time, \( {\dot{\theta }}_{i\max } \) and \( {\ddot{\theta }}_{i\max } \) are the maximum values of the angular velocities and accelerations; \( P \) is the driving power for digging, \( {P}_{\max } \) represents its maximum value; \( {T}_{q} \) is the joint driving torque for digging, \( {T}_{q\max } \) represents its maximum value; \( {\theta }_{p} \) is the digging angle at the start point, \( {\theta }_{p\min } \) and \( {\theta }_{p\max } \) are the corresponding thresholds.

<!-- Meanless: 5-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

<!-- Media -->

<!-- figureText: Input Hidden Hidden Hidden Output Layer #q-1 Layer #q layer Y \( {a}_{36} \) \( {\mathbf{{Loss}}}_{\mathrm{z}}\left( \mathbf{\Theta }\right) \) Backpropagation Learning Algorithm layer Layer #1 \( {\theta }_{11} \) \( {\theta }_{12} \) \( {\theta }_{13} \) \( {\theta }_{22} \) \( {\theta }_{23} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_5.jpg?x=462&y=153&w=820&h=515&r=0"/>

Fig. 4. Structure diagram of PINN.

Table 2

Key parameters of excavator employed in the experiments.

<table><tr><td>Parameter/Unit</td><td>Value</td><td>Parameter/Unit</td><td>Value</td></tr><tr><td>\( {\dot{\theta }}_{1\max }/\left( {\mathrm{{deg}}/\mathrm{s}}\right) \)</td><td>15</td><td>\( {T}_{q\max }/\left( \mathrm{{Nm}}\right) \)</td><td>\( {1.0} \times  {10}^{3} \)</td></tr><tr><td>\( {\dot{\theta }}_{2\max }/\left( {\mathrm{{deg}}/\mathrm{s}}\right) \)</td><td>28</td><td>\( {V}_{p\max }/\left( {\mathrm{{cm}}}^{3}\right) \)</td><td>\( {5.72} \times  {10}^{3} \)</td></tr><tr><td>\( {\dot{\theta }}_{3\max }/\left( {\mathrm{{deg}}/\mathrm{s}}\right) \)</td><td>30</td><td>\( {V}_{p\min }/\left( {\mathrm{{cm}}}^{3}\right) \)</td><td>\( {3.74} \times  {10}^{3} \)</td></tr><tr><td>\( {\ddot{\theta }}_{1\max }/\left( {\mathrm{{deg}}/{\mathrm{s}}^{2}}\right) \)</td><td>10</td><td>\( {t}_{\max }/\left( \mathrm{s}\right) \)</td><td>10</td></tr><tr><td>\( {\ddot{\theta }}_{2\max }/\left( {\mathrm{{deg}}/{\mathrm{s}}^{2}}\right) \)</td><td>10</td><td>\( {t}_{\min }/\left( \mathrm{s}\right) \)</td><td>5</td></tr><tr><td>\( {\ddot{\theta }}_{3\max }/\left( {\mathrm{{deg}}/{\mathrm{s}}^{2}}\right) \)</td><td>10</td><td>\( {E}_{\max }/\left( \mathrm{{Wh}}\right) \)</td><td>0.110</td></tr><tr><td>\( {P}_{\max }/\left( \mathrm{W}\right) \)</td><td>400</td><td>\( {E}_{\min }/\left( \mathrm{{Wh}}\right) \)</td><td>0.025</td></tr><tr><td>\( {K}_{0}/\left( {\mathrm{N}/{\mathrm{{cm}}}^{2}}\right) \)</td><td>5.0</td><td>\( {\lambda }_{1} \)</td><td>0.5</td></tr><tr><td>\( b/\left( \mathrm{m}\right) \)</td><td>0.22</td><td>\( {\lambda }_{2} \)</td><td>0.2</td></tr><tr><td>\( \mu \)</td><td>0.40</td><td>\( {\lambda }_{3} \)</td><td>0.3</td></tr></table>

<!-- Media -->

### 4.3. PINN planning

The PINN algorithm framework used in this paper mainly includes three parts: fully connected neural network, physics-based loss function and back propagation algorithm. The main difference between the proposed method and the general back propagation training of BP neural networks is the loss function which is not a single mean square error loss (MSE) or cross entropy loss, but includes the loss based on physical rules.

#### 4.3.1. Training samples

The one-shot digging trajectory parameters optimized by PSO are used as the output of the data set for training PINN. And the joint angles corresponding to the starting and end point of trajectory are taken as the input for training together with the parameter \( H \) . It should be noted that the parameter \( H \) is a height value used for the calculation of digging area. Therefore,the input \( \mathbf{X} \) of the PINN model is \( \left\lbrack  {{\theta }_{11},{\theta }_{12},{\theta }_{13},{\theta }_{21},{\theta }_{22},{\theta }_{23},H}\right\rbrack \) ,and the output \( \mathbf{Y} \) is \( \left\lbrack  {{a}_{16},{a}_{26},{a}_{36},t}\right\rbrack \) .

#### 4.3.2. Physics-based loss function

The planned trajectory should ensure the kinematic and dynamic performance of the excavator arms. Therefore, the kinematics and dynamics models are incorporated into the loss function of the neural network as constraints, so as to generate a stable and feasible trajectory. Similarly, in order to ensure the planned trajectory practical, the constraints of initial digging angle, fill factor, and other performance indicators are also included into the loss function. Hence, the loss function consists of the following five parts: MSE, mathematical model constraint loss of the excavator arms mechanism, digging angle constraint, fill factor constraint loss and performance loss.

In neural network, MSE is the most commonly used loss function in regression problems, which is used to measure the average error

between the predicted value \( \widehat{\mathbf{Y}} \) and the true value \( \mathbf{Y} \) of the neural network. Using \( {\operatorname{Loss}}_{1}\left( {\mathbf{Y},\widehat{\mathbf{Y}}}\right) \) to express as:

\[{\operatorname{Loss}}_{1}\left( {\mathbf{Y},\widehat{\mathbf{Y}}}\right)  = \mathop{\sum }\limits_{{n = 1}}^{N}{\begin{Vmatrix}{\widehat{\mathbf{Y}}}_{n} - {\mathbf{Y}}_{n}\end{Vmatrix}}_{2}^{2} \tag{13}\]

where, \( N \) represents the number of training samples, \( {\widehat{\mathbf{Y}}}_{n} \) and \( {\mathbf{Y}}_{n} \) are the true value and the predicted value of the \( n \) th training sample respectively.

For the mathematical model constraints loss, the kinematics constraints include joint angle, angular velocity and acceleration, while the dynamics constraints mainly include power and torque. Using \( {\operatorname{Loss}}_{2}\left( \widehat{\mathbf{Y}}\right) \) to express as:

\[{\operatorname{Loss}}_{2}\left( \widehat{\mathbf{Y}}\right)  = \mathop{\sum }\limits_{{i = 1}}^{3}\max \left\{  {0,{e}_{i\min } - {\widehat{e}}_{i}}\right\}   + \max \left\{  {0,{\widehat{e}}_{i} - {e}_{i\max }}\right\}   \tag{14}\]

where,max \( \{  \cdot  \} \) is used to establish inequality constraints,and \( e \) represents the norm form of each constraint,including joint angle \( \theta \) ,angular velocity \( \dot{\theta } \) ,angular acceleration \( \ddot{\theta } \) ,power \( P \) and joint torque \( {T}_{q} \) .

To ensure that the bucket can successfully cut into the soil, the digging angle at the initial stage of the trajectory is constrained within a certain range. Using \( {\operatorname{Loss}}_{3}\left( \widehat{\mathbf{Y}}\right) \) to express as:

\[{\operatorname{Loss}}_{3}\left( \widehat{\mathbf{Y}}\right)  = \max \left\{  {0,{\theta }_{p\min } - {\widehat{\theta }}_{p}}\right\}   + \max \left\{  {0,{\widehat{\theta }}_{p} - {\theta }_{p\max }}\right\}   \tag{15}\]

For the fill factor,the digging volume in is limited to 0.8 .1.2 times of the standard volume for each cycle. Using \( {\operatorname{Loss}}_{4}\left( \widehat{\mathbf{Y}}\right) \) to express as:

\[{\operatorname{Loss}}_{4}\left( \widehat{\mathbf{Y}}\right)  = \max \left\{  {0,{V}_{p\min } - {\widehat{V}}_{p}}\right\}   + \max \left\{  {0,{\widehat{V}}_{p} - {V}_{p\max }}\right\}   \tag{16}\]

Therefore,the total constraint loss function \( {\operatorname{Loss}}_{\mathrm{c}}\left( \widehat{\mathbf{Y}}\right) \) can be expressed as:

\[{\operatorname{Loss}}_{\mathrm{c}}\left( \widehat{\mathbf{Y}}\right)  = {\operatorname{Loss}}_{2}\left( \widehat{\mathbf{Y}}\right)  + {\operatorname{Loss}}_{3}\left( \widehat{\mathbf{Y}}\right)  + {\operatorname{Loss}}_{4}\left( \widehat{\mathbf{Y}}\right)  \tag{17}\]

Additionally, the objective function (Eq. (12)) with the optimal digging time, energy consumption and fill factor is incorporated into the loss function to form the performance loss function \( {\operatorname{Loss}}_{5}\left( \widehat{\mathbf{Y}}\right) \) :

\[{\operatorname{Loss}}_{5}\left( \widehat{\mathbf{Y}}\right)  = {\lambda }_{1}{T}_{t} + {\lambda }_{2}{E}_{e} + {\lambda }_{3}{V}_{v} \tag{18}\]

To sum up, the total loss function can be expressed as:

\[{\operatorname{Loss}}_{\mathrm{z}}\left( \mathbf{\Theta }\right)  = {\lambda }_{M}{\operatorname{Loss}}_{1}\left( {\mathbf{Y},\widehat{\mathbf{Y}}}\right)  + {\lambda }_{C}{\operatorname{Loss}}_{\mathrm{c}}\left( \widehat{\mathbf{Y}}\right)  \tag{19}\]

\[ + {\lambda }_{F}{\operatorname{Loss}}_{5}\left( \widehat{\mathbf{Y}}\right)  + {\lambda }_{R}R\left( \mathbf{W}\right) \]

Where, \( \mathbf{\Theta } = \left( {\mathbf{W},\mathbf{B}}\right) \) are the parameters of the neural network, \( \mathbf{W} \) is the weight matrix and \( \mathbf{B} \) is the bias matrix. \( {\lambda }_{M},{\lambda }_{C} \) and \( {\lambda }_{F} \) are the weight coefficients of each sub loss function (for simplicity, \( {\lambda }_{M} = {\lambda }_{C} = \) \( {\lambda }_{F} = 1),R\left( \mathbf{W}\right) \) represents the regularization function of the model complexity, \( {\lambda }_{R} \) is the hyperparameter.

<!-- Meanless: 6-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

<!-- Media -->

<!-- figureText: Enter task Plan the second type trajectory by PINN Driving the joints \( \mathrm{N} \) Move the excavator? Drive the excavator \( \mathrm{N} \) demand length2 End the task requirements Initial excavator arm posture Plan the first type trajectory by PINN Driving the joints Y demand depth2 -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_6.jpg?x=148&y=148&w=662&h=788&r=0"/>

Fig. 5. Flowchart of task-oriented continuous trajectory planning.

<!-- Media -->

After constructing the loss function, using the back propagation algorithm to train the network parameter \( \mathbf{\Theta } \) :

\[\{ \widehat{\mathbf{\Theta }}\}  = \arg \min {\operatorname{Loss}}_{\mathrm{z}}\left( \mathbf{\Theta }\right)  \tag{20}\]

After training, PINN has learned the physical rules contained in the loss function and can predict new data. And the complete PINN model structure is shown in Fig. 4.

### 4.4. Task-oriented continuous trajectory planning

After successfully training the PINN model, embed it into the task-oriented continuous trajectory planning model. The detailed planning process of Part III in Fig. 3 is shown in Fig. 5.

Firstly, input the demands of preset digging task, including the ditch shape (length, width, depth, slope angle) and the initial digging position, based on which the position and pose of the excavator are initialized. Then, use the trained PINN model to plan the first type digging trajectory. Next, compare the excavated depth corresponding to the planned trajectory with the preset target. For negative result, continue to plan the first type digging trajectory downward until the depth demand were met. For positive result, plan the second type digging trajectory backward, and judge whether it is necessary to move the excavator's position until the length demand of the task is reached. Finally, several trajectories are planned through the PINN model.

## 5. Experiments and results

Trenching is one of the typical working condition for excavators where the continuous digging is required. Therefore, the trenching operation is chosen for both virtual and physical experiments to validate the proposed task-oriented trajectory planning model. Fig. 6 shows the scale-modeled excavator applied for the experiments.

### 5.1. Experimental setup

For accurate positioning and control, the servo electric cylinders are used to drive each joint. The crawler and swing mechanism are driven by brushless DC motors,which are powered by a \( {24}\mathrm{\;V} \) regulated power supply. The other devices are shown in Fig. 6. The upper controller is a computer, which uses SIMULINK to compile the main control program. The lower controllers are the servo drivers and Vehicle Control Unit (VCU). The signal communication is accomplished through CAN bus using the device of Kvaser. The servo drivers receive the CAN message sent by the upper computer, and then analyze it into the corresponding instructions (position, speed or torque), so as to convert the rotation motion of the servo motors into the linear motion of the electric cylinders, and finally drive each joint of the excavator arms to rotate around the hinged point. The VCU receives the CAN message sent by the upper computer, and sends the corresponding PWM signal according to the program burned in the VCU to control the walking and swing motors.

<!-- Media -->

<!-- figureText: Computer VCU Stick motor Stick electric cylinder Bucket motor Bucket electric cylinder Rotary motor Boom motor Boom electric cylinder Kvaser Servo drivers 24V regulated Walking motor power supply -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_6.jpg?x=338&y=1414&w=1071&h=718&r=0"/>

Fig. 6. Experimental setup.

<!-- Meanless: 7-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

<!-- figureText: 0.52 30 40 50 Epochs 0.50 0.48 Fitness 0.46 0.44 0.42 0.40 0 10 20 -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_7.jpg?x=112&y=149&w=722&h=687&r=0"/>

Fig. 7. Fitness varies with iterations.

<!-- Media -->

The D-H parameters of the excavator arms corresponding to the experiment device shown in Fig. 6 are listed in Table 1, where joint #1 is the boom, joint #2 is the stick rod and joint #3 is the bucket. The parameters used in trajectory optimization with PSO and PINN training are shown in Table 2.

### 5.2. Analysis of PINN training results

PSO was used to optimize the two types of trajectories (downward and backward, shown in Fig. 2) respectively to generate the samples for PINN. A group of trajectory input parameters were selected from the workspace of the excavator arms, and the PSO optimization results were obtained, as shown in Figs. 7 and 8.

Fig. 7 shows the change curve of the fitness with the iterations of PSO. It converges at the 38th iteration, and the value of the objective function is 0.408 .

Fig. 8 shows the optimization results, where Fig. 8(a) shows the joint angles of the trajectory and the bucket attitude angle \( \zeta \) changing with time, Figs. 8(b) and 8(c) respectively show the curves of angular velocities and angular accelerations of each joint changing with time, and Fig. 8(d) shows the calculated theoretical driving torque of each joint. From Fig. 8(a) to 8(d), it can be seen that the whole trajectory fully satisfies the kinematic and dynamic constraints during operation, and the angular velocities and angular accelerations of each joint are continuous and smooth, without excessive shock.

As shown in Fig. 9, for the experimental device, the green points represent the working range of the bucket tip. Generally, the PSO samples should cover most of the reachable digging area. To be more specific, for the trenching operations, the majority of the points below the crawler(-0.25m)should be taken for samples. Therefore,the area inside the red rectangle with the length of \( {0.8}\mathrm{\;m} \) and the depth of \( {0.4}\mathrm{\;m} \) is taken for the sampling. Numerically,for the first type of trajectory, the number of samples is 1800 groups (1600 groups were used for training and 200 groups were used for testing), and the average performance value of samples was 0.403 . For the second type of trajectory, the number of samples is 1800 groups (1600 groups for training and 200 groups for testing), and the average performance value of samples is 0.303 .

The first type of trajectory adopted 4-layer neural network to train the network parameters \( \theta \) . There were three hidden layers,each layer contained 70 nodes. The hyperbolic tangent function (tanh) was selected as the activation function. The small-batch gradient descent method (MBGD) was selected as backpropagation algorithm, the batch size was set to 50 , and the learning rate of the network was set to 0.0001 .

Fig. 10 shows the change of each part of the loss function when training the first type of trajectory for both the training set and testing set. Fig. 10(a) shows the MSE \( {\operatorname{Loss}}_{1} \) ,the target accuracy is set to \( {1.5} \times \) \( {10}^{-1} \) . Fig. 10(b) shows the constraint loss \( {\mathrm{{Loss}}}_{2} \) with the target accuracy set to \( {1.5} \times  {10}^{-5} \) . Fig. \( {10}\left( \mathrm{c}\right) \) shows the digging angle constraint loss \( {\mathrm{{Loss}}}_{3} \) with the target accuracy set to \( {5.0} \times  {10}^{-3} \) . Fig. \( {10}\left( \mathrm{\;d}\right) \) shows the loss of fill factor \( {\operatorname{Loss}}_{4} \) with the target accuracy set to \( {1.5} \times  {10}^{-5} \) . Fig. \( {10}\left( \mathrm{e}\right) \) shows the loss of performance \( {\mathrm{{Loss}}}_{5} \) with the target accuracy set to 0.443 . Fig. 10(f) shows the total loss function \( {\mathrm{{Loss}}}_{\mathrm{z}} \) . According to Fig. 10(a) to \( {10}\left( \mathrm{e}\right) \) ,it can be seen that the mathematical model constraints loss of the working device fluctuated soon after the iteration, which might be caused by the influence from other loss functions. All constraint losses \( {\operatorname{Loss}}_{2} \sim  {\operatorname{Loss}}_{4} \) simultaneously reach the target accuracy at the 1090th training, and the average performance value of training samples is 0.439 , which is about 91% of the PSO sample.

The second type of trajectory adopted 4-layer neural network to train the network parameters \( \theta \) . There were three hidden layers,each layer contained 60 nodes. The activation function, back propagation algorithm and learning rate were the same as those in the first type of trajectory training, and the batch size was set to 100 .

Fig. 11 shows the change of each part of the loss function when training the second type of trajectory for both the training set and testing set. The target accuracy of the performance \( {\operatorname{Loss}}_{5} \) is set to 0.333, with the other targets the same as those of the first type of trajectory training. As shown in Fig. 11(b), the loss function for the working device converged soon after the iteration although other loss functions still being processing, which indicate the training meet the kinematic and dynamic constraints but not others in the initiation stage.

According to Fig. 11(a) to 11(e), it can be seen that all constraint losses \( {\mathrm{{Loss}}}_{2} \sim  {\mathrm{{Loss}}}_{4} \) simultaneously reach the target accuracy at the530th training \( \left( {\mathrm{{Loss}}}_{2}\right. \) reaches the target accuracy in the second iteration), and the average performance value of training samples is 0.331 , which is about 91% of the PSO sample.

<!-- Media -->

<!-- figureText: 16 Suck 45.0 0.6 brque(Nm) 0.0 -0.6 -1.8 100 7.5 Time (s) Time (s) (c) (d) Angle (deg) -0.5 -1.5 -135 - 2.5 -20 Time (s) Time (s) (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_7.jpg?x=209&y=1790&w=1328&h=320&r=0"/>

Fig. 8. Variation of the key parameters of the excavator arm over time when executing an optimized trajectory: (a) joint angle, (b) angular velocity, (c) angular acceleration, (d) driving torque.

<!-- Meanless: 8-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

<!-- figureText: 1 X/m 0.6 Y/m 0.2 -0.2 Ground -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_8.jpg?x=162&y=160&w=636&h=505&r=0"/>

Fig. 9. Samples displacement.

<!-- Media -->

### 5.3. Task-oriented continuous digging trajectory planning experiment

Fig. 12 shows the experimental platform for continuous digging trajectory planning. The material used in the experiment is rubber particles. The length of the excavated area is \( {0.90}\mathrm{\;m} \) ,the width is \( {0.60}\mathrm{\;m} \) and the depth is \( {0.30}\mathrm{\;m} \) . The excavator is placed on a box with the same height of material platform. In order to prevent the dust generated during the experiment from damaging the servo drivers and other equipment, the transparent plastic sheet was used for protection.

As shown in Fig. 13,the area enclosed by \( {Q}_{1},{Q}_{2},{Q}_{3} \) and \( {Q}_{4} \) is the contour section of the task to be excavated. Four trenching tasks are set for the validation of the trajectory planning method, as listed in Table 3.

#### 5.3.1. Analysis of theoretical planning results

According to the flowchart described in Fig. 3 and the continuous planning process shown in Fig. 5, the PINN model was used for trajectory planning, and the results for all the four tasks are shown in Fig. 14. Under the same initial conditions, PSO was used to plan trajectories respectively. And the computation time and the digging performance corresponding to the trajectories were compared with the PINN, as shown in Table 4.

As can be seen from Fig. 14, PINN can generate different numbers of trajectory based on the task requirements. And the planned trajectories can adjust to different shape of the cross section. To be more specific, the difference between the area covered by trajectories and the designed shape for trenching task \( \mathrm{A} \sim  \mathrm{D} \) are \( {8.7}\% ,{3.7}\% ,{1.2}\% \) and 3.3% respectively.

As can be seen from Table 4, the average computation time for PINN ranges from \( {4.2}\mathrm{\;{ms}} \) to \( {4.5}\mathrm{\;{ms}} \) while \( {69}\mathrm{\;s} \) to \( {71}\mathrm{\;s} \) for PSO. For the average digging time, digging volume and energy consumption, the trajectories planned by PINN perform almost the same as those by PSO with the difference less than 5%.

Additionally, the comparison between the proposed PINN method and other trajectory planning method on computation time and digging performance corresponding to planned results are listed in Table 5.

<!-- Media -->

<!-- figureText: 0.30 2.5 2.5 Training Training Test Loss, (Cutting angle constraint) 2.0 1.5 0.0 600 800 1000 1200 200 400 600 800 1000 1200 Epochs Epochs (c) Training 3.0 Training Test 2.5 Loss: (All) 2.0 1.0 0.5 600 800 1000 1200 200 400 600 800 1000 1200 Epochs Epoch: (f) Training 0.25 \( {\text{Loss}}_{2} \) (Working device constraint) 2.0 1.5 0.5 Loss1 (MSE) 0.20 0.15 0.0 200 400 600 800 1000 1200 200 400 Epochs (a) (b) \( {10}^{-1} \) 0.49 Training Test \( {10}^{-2} \) Loss \( {}_{5} \) (Performance constraint) 0.47 0.46 0.45 0.44 Loss \( {}_{4} \) (Fill factor constraint) \( {10}^{-3} \) \( {10}^{-4} \) \( {10}^{-6} \) 0.43 200 400 800 1000 1200 200 400 Epochs (d) (e) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_8.jpg?x=240&y=1181&w=1266&h=888&r=0"/>

Fig. 10. Losses of PINN for training the first type of trajectory: (a) MSE, (b) working device constraint, (c) cutting angle, (d) fill factor constraint, (e) performance constraint, (f) total loss.

<!-- Meanless: 9-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

<!-- figureText: 0.6 0.16 Training 0.14 Loss \( {}_{3} \) (Cutting angle constraint) 0.12 0.10 0.08 0.06 0.04 0.02 300 400 500 100 200 300 400 500 Epochs Epochs (c) Training Test 1.0 0.9 Loss (All) 0.8 0.7 0.6 0.5 300 200 500 Epochs Epochs (f) Training Test 0.14 Loss1 (MSE) \( {\text{Loss}}_{2} \) (Working device constraint) 0.12 0.08 0.06 0.04 0.2 0.02 0.00 0 100 200 300 400 500 0 200 Epochs (a) (b) \( {10}^{-2} \) 0.45 Test Loss \( {}_{4} \) (Fill factor constraint) \( {10}^{-4} \) Loss \( {}_{5} \) (Performance constraint) 0.35 0.30 0.25 \( {10}^{-5} \) 0.20 100 400 Epochs (d) (e) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_9.jpg?x=240&y=214&w=1266&h=910&r=0"/>

Fig. 11. Losses of PINN for training the second type of trajectory: (a) MSE, (b) working device constraint, (c) cutting angle, (d) fill factor constraint, (e) performance constraint, (f) total loss.

<!-- figureText: Experimental materials Computer Excavator -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_9.jpg?x=120&y=1261&w=716&h=411&r=0"/>

Fig. 12. Experiment for continuous trajectory planning.

<!-- figureText: \( L \) Q Q -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_9.jpg?x=153&y=1743&w=654&h=262&r=0"/>

Fig. 13. Contour section of task.

<!-- Media -->

As can be seen from Table 5, the proposed PINN has the obvious advantage in computation time. For the digging performance, Zhang et al. Zhang et al. [9] only took digging efficiency for objective using the SQP method with the energy consumption and bucket fullness ignored in the optimization. Wang et al. Wang et al. [52] applied STP method for optimization based on surrogate model. However, the planning time was relatively too long with a higher energy consumption per unit digging mass. Sandzimier et al. Sandzimier and Asada [53] proposed a data-driven method for digging trajectory planning with a relatively lower bucket fullness corresponding to the results. Besides, it should be noticed that the proposed PINN method can generated a group of digging trajectories according to the total operation task in one time of planning while most of the exist planning method can only come up with one single trajectory.

<!-- Media -->

Table 3

Four trenching tasks.

<table><tr><td>No.</td><td>\( L\left( \mathrm{\;m}\right) \)</td><td>\( D\left( \mathrm{\;m}\right) \)</td><td>\( {\beta }_{1} \) (deg)</td><td>\( {\beta }_{2}\left( \mathrm{\;{deg}}\right) \)</td></tr><tr><td>A</td><td>0.6</td><td>0.15</td><td>60</td><td>60</td></tr><tr><td>B</td><td>0.7</td><td>0.1</td><td>65</td><td>40</td></tr><tr><td>C</td><td>0.55</td><td>0.15</td><td>60</td><td>50</td></tr><tr><td>D</td><td>0.65</td><td>0.2</td><td>50</td><td>60</td></tr></table>

<!-- Media -->

Based on the above analysis based on comparisons, it can be concluded that the PINN method is adaptive for different trench shapes and it is an effective real-time solution for continuous excavation planning.

#### 5.3.2. Analysis of experimental results

After obtaining the digging trajectory parameters using the PINN model, the following experiments are executed, with the results analyzed and discussed. Fig. 15 shows the status of excavator arms before and after executing the first digging trajectory.

<!-- Meanless: 10-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

<!-- Media -->

<!-- figureText: -0.20 Trenching task (A) -0.20 Trenching task (B) -0.25 Y (m) The task contour -0.30 Trajectory #1 Trajectory #2 Trajectory #3 -0.35 -0.40 0.5 0.8 X (m) -0.20 Trenching task (D) -0.25 -0.30 Trajectory #1 Y (m) -0.35 Trajectory #2 Trajectory #3 -0.40 Trajectory #4 Trajectory #5 -0.45 -0.50 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0 1.1 X (m) -0.25 The task contour Y (m) -0.30 Trajectory #1 Trajectory #2 -0.35 Trajectory #3 Trajectory #4 -0.40 -0.45 0.6 0.7 1.1 X (m) -0.20 Trenching task (C) -0.25 \( \mathrm{Y}\left( \mathrm{m}\right) \) -0.30 The task contour Trajectory # -0.35 Trajectory #2 Trajectory #3 -0.40 -0.45 0.4 0.5 0.6 0.7 0.8 0.9 1.0 1.1 \( \mathrm{X}\left( \mathrm{m}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_10.jpg?x=245&y=213&w=1258&h=659&r=0"/>

Fig. 14. Trajectories planned by PINN for trenching task A~D.

Table 4

Performance comparison between PSO and PINN.

<table><tr><td rowspan="2">NO.</td><td colspan="4">PSO</td><td colspan="4">PINN</td></tr><tr><td>t/(s)</td><td>\( {V}_{p}/\left( {{10}^{3}{\mathrm{\;{cm}}}^{3}}\right) \)</td><td>\( {W}_{E}/\left( \mathrm{{Wh}}\right) \)</td><td>\( {t}_{c}/\left( \mathrm{s}\right) \)</td><td>t/(s)</td><td>\( {V}_{p}/\left( {{10}^{3}{\mathrm{\;{cm}}}^{3}}\right) \)</td><td>\( {W}_{E}/\left( \mathrm{{Wh}}\right) \)</td><td>\( {t}_{c}/\left( \mathrm{{ms}}\right) \)</td></tr><tr><td>#1-A</td><td>7.60</td><td>5.12</td><td>0.061</td><td>68.05</td><td>7.85</td><td>5.07</td><td>0.063</td><td>4.48</td></tr><tr><td>#2-A</td><td>7.71</td><td>5.10</td><td>0.060</td><td>69.95</td><td>7.78</td><td>5.05</td><td>0.065</td><td>4.14</td></tr><tr><td>#3-A</td><td>5.98</td><td>4.57</td><td>0.053</td><td>70.91</td><td>6.01</td><td>4.80</td><td>0.064</td><td>4.12</td></tr><tr><td>#4-A</td><td>6.15</td><td>4.53</td><td>0.052</td><td>68.12</td><td>6.20</td><td>4.35</td><td>0.052</td><td>4.05</td></tr><tr><td>Ave-A.</td><td>6.86</td><td>4.83</td><td>0.057</td><td>69.26</td><td>6.96</td><td>4.82</td><td>0.061</td><td>4.20</td></tr><tr><td>#1-B</td><td>7.66</td><td>5.03</td><td>0.062</td><td>70.24</td><td>7.59</td><td>5.01</td><td>0.063</td><td>4.39</td></tr><tr><td>#2-B</td><td>5.98</td><td>4.82</td><td>0.055</td><td>70.87</td><td>6.13</td><td>4.85</td><td>0.060</td><td>4.46</td></tr><tr><td>#3-B</td><td>6.03</td><td>4.95</td><td>0.059</td><td>69.59</td><td>6.22</td><td>4.96</td><td>0.062</td><td>4.53</td></tr><tr><td>Ave-B.</td><td>6.56</td><td>4.93</td><td>0.059</td><td>70.23</td><td>6.65</td><td>4.94</td><td>0.062</td><td>4.46</td></tr><tr><td>#1-C</td><td>7.76</td><td>5.05</td><td>0.059</td><td>69.11</td><td>7.93</td><td>5.06</td><td>0.061</td><td>4.76</td></tr><tr><td>#2-C</td><td>7.65</td><td>5.09</td><td>0.062</td><td>69.58</td><td>7.69</td><td>5.08</td><td>0.063</td><td>4.03</td></tr><tr><td>#3-C</td><td>6.03</td><td>4.98</td><td>0.061</td><td>70.13</td><td>6.27</td><td>5.01</td><td>0.066</td><td>4.35</td></tr><tr><td>Ave-C.</td><td>7.15</td><td>5.04</td><td>0.061</td><td>69.61</td><td>7.30</td><td>5.05</td><td>0.063</td><td>4.38</td></tr><tr><td>#1-D</td><td>7.60</td><td>5.12</td><td>0.061</td><td>68.05</td><td>7.85</td><td>5.07</td><td>0.063</td><td>4.48</td></tr><tr><td>#2-D</td><td>7.57</td><td>5.09</td><td>0.063</td><td>69.76</td><td>7.63</td><td>5.04</td><td>0.062</td><td>4.25</td></tr><tr><td>#3-D</td><td>7.72</td><td>5.06</td><td>0.063</td><td>71.13</td><td>7.58</td><td>5.05</td><td>0.064</td><td>4.27</td></tr><tr><td>#4-D</td><td>6.02</td><td>4.76</td><td>0.060</td><td>69.53</td><td>6.64</td><td>4.92</td><td>0.067</td><td>4.09</td></tr><tr><td>#5-D</td><td>5.73</td><td>4.23</td><td>0.054</td><td>69.34</td><td>5.85</td><td>4.11</td><td>0.053</td><td>4.11</td></tr><tr><td>Ave-D.</td><td>6.93</td><td>4.85</td><td>0.060</td><td>59.56</td><td>7.07</td><td>4.84</td><td>0.062</td><td>4.24</td></tr></table>

Table 5

Planning method comparison.

<table><tr><td>Method</td><td>Computation time (s)</td><td>Unit energy consumption \( \left( {\mathrm{{kWh}}/{\mathrm{m}}^{3}}\right) \)</td><td>Bucket fullness (%)</td></tr><tr><td>PINN model (ours)</td><td>0.0042</td><td>\( {1.27} \times  {10}^{-2} \)</td><td>101.9</td></tr><tr><td>Method in Zhang et al. [9]</td><td>1.0272</td><td>-</td><td>-</td></tr><tr><td>Method in Wang et al. [52]</td><td>41.1980</td><td>\( {7.26} \times  {10}^{-2} \)</td><td>-</td></tr><tr><td>Method in Sandzimier and Asada [53]</td><td>-</td><td>-</td><td>95.0</td></tr></table>

<!-- Media -->

Excavation experiment for trenching task A was implemented with the performance corresponding to #1-A trajectory shown in Figs. 16- 21. Due to the fast response of the servo motor, it can be directly seen that the velocity and displacement of the cylinders match with the planned curve to a good extend, which makes the actual digging trajectory basically coincide with the planned one The energy consumption calculated based on torque (Fig. 16) and velocity (Fig. 17) is about 13% larger than the theoretical result, which could be explained by the ignorance of the joint friction and the difference on soil parameters. The excavated material volume in the bucket is measured to be about \( {4.63} \times  {10}^{3}{\mathrm{\;{cm}}}^{3} \) ,which differs from the theoretical calculation about \( {0.44} \times  {10}^{3}{\mathrm{\;{cm}}}^{3} \) . The main reason for the difference can be explained as the flow behavior of bulk material. All the material inside the area enclosed by digging trajectories are considered totally filled into the bucket in theoretical calculation. However, the material would follow to both sides during the digging process but not totally into the bucket, especially when the bucket moving backward. Additionally, the material would fall out of the bucket when being moved out of the pile. Hence, the ignorance of such flow behavior in calculation caused the final result differ from the measurement.

<!-- Meanless: 11-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

<!-- Media -->

<!-- figureText: (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_11.jpg?x=172&y=153&w=613&h=1126&r=0"/>

Fig. 15. The first digging: (a) starting, (b) ending.

<!-- Media -->

After four times of continuous digging, the final shape of the trench is shown in Fig. 21, and the comparison with the designed shape is shown in Table 6. The relative error of length \( L \) is \( {6.7}\% \) ,width \( B \) is \( {8.7}\% \) ,depth \( D \) is \( {6.7}\% \) ,slope angle \( {\beta }_{1} \) is \( {5.0}\% \) and slope angle \( {\beta }_{2} \) is 11.7% (due to the influence of material flow,the relative error of \( {\beta }_{2} \) is slightly large). Although the actual excavated task contour is different from the preset value to some extent due to the influence of material flow, the feasibility of this method is proved.

Based on the analysis of the theoretical planning results and experimental results discussed above, it can be concluded that it is feasible to use the PINN model proposed in this paper to plan the continuous trajectories of the excavator arms. Firstly, the trajectory can be planned in milliseconds making it meet the requirement for real-time planning. Secondly, the planned trajectory satisfies various physical constraints for execution. The last but not the least, the overall performance of the digging trajectory planned by PINN model can also reach a high level compared with PSO.

<!-- Media -->

<!-- figureText: 2 19.5 17.0 14.5 12.0 9.5 Torque (Nm) 7.0 4.5 2.0 -0.5 - 3.0 Boom 1 Bucket Stick 0 -1 Torque (Nm) -2 -3 -4 -5 -6 Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_11.jpg?x=909&y=149&w=726&h=621&r=0"/>

Fig. 16. Motor torque of each joint.

<!-- figureText: 8 28.0 24.5 21.0 17.5 Velocity (mm/s) 14.0 10.5 7.0 3.5 0.0 6 8 6 Planned of Boom Actural of &tick Planned of Stick 4 Actural At Bucket Planned of Bucket Velocity (mm/s) 2 0 -2 -4 -6 -8 0 2 4 Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_11.jpg?x=911&y=854&w=718&h=619&r=0"/>

Fig. 17. Velocity of each electrical cylinder.

Table 6

Parameter comparison of planned and actual task profile.

<table><tr><td colspan="4">F</td></tr><tr><td>Parameters</td><td>Planned</td><td>Actual</td><td>Relative error (%)</td></tr><tr><td>\( L/\left( \mathrm{m}\right) \)</td><td>0.60</td><td>0.56</td><td>6.7</td></tr><tr><td>\( B/\left( \mathrm{m}\right) \)</td><td>0.23</td><td>0.25</td><td>8.7</td></tr><tr><td>\( D/\left( \mathrm{m}\right) \)</td><td>0.15</td><td>0.14</td><td>6.7</td></tr><tr><td>\( {\beta }_{1}/\left( \mathrm{{deg}}\right) \)</td><td>60</td><td>57</td><td>5.0</td></tr><tr><td>\( {\beta }_{2}/\left( \mathrm{{deg}}\right) \)</td><td>60</td><td>53</td><td>11.7</td></tr></table>

<!-- Media -->

## 6. Conclusions

Focusing on the challenge of task-oriented autonomous excavation, this paper presented a method for continuous digging trajectory planning using machine learning algorithm. The application of PINN directly took all the excavator arm constrains relevant to digging trajectory into the planning process, which not only improved the interpretability of the algorithm but also prevented the over simplification of traditional planning models. Because the method can generate a group of trajectories in one single planning process matching different trench profiles, it makes the autonomous excavation a more compact process, which is applicable in long-distance excavation operations such as pipeline and foundation constructions. Furthermore, utilizing optimization results as training samples and PINN as the framework enables both computation efficiency of planning and digging performance of the planned trajectories. The simulations and experiments described in this paper, which evaluated the computation time, profile matching and motion dynamic as the proof of the proposed real-time planning method, demonstrated high efficiency and adaptiveness in different tasks.

<!-- Meanless: 12-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

<!-- Media -->

<!-- figureText: 50 100 Actural of Boom Planned of Boom Actural of Stick 80 Actural of Bucket Planned of Bucket Displacement (mm) 40 20 0 45 Displacement (mm) 40 35 30 25 Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_12.jpg?x=117&y=226&w=720&h=638&r=0"/>

Fig. 18. Displacement of each electrical cylinder.

<!-- figureText: 5.5 0 Actural of Bogan Planned of Bøom Actural of Btick -4 Planned of Stick Actural of Bucket igular velocity (deg/s) -12 -16 -24 -28 4 6 8 Time (s) 4.0 Angular velocity (deg/s) 2.5 1.0 -0.5 -2.0 -3.5 -5.0 0 2 -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_12.jpg?x=117&y=994&w=720&h=621&r=0"/>

Fig. 19. Actual angular velocity of each joint.

<!-- figureText: -0.20 Planned trajectory Actual trajectory 1.0 1.1 X (m) -0.25 Y (m) -0.30 -0.35 -0.40 0.7 0.8 -->

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_12.jpg?x=149&y=1770&w=656&h=348&r=0"/>

Fig. 20. First digging trajectory.

<img src="https://cdn.noedgeai.com/bo_d2sl8j77aajc738sf7og_12.jpg?x=981&y=216&w=578&h=380&r=0"/>

Fig. 21. Final profile of task.

<!-- Media -->

Future work should investigate the influence of material flow behavior on the cross-section profile of excavated space, which could result in a more accurate shape in practical operations. The difference in dynamic characters between hydraulic and electric cylinders should also be investigated for the motion curve modification to expend the application field of the work. This PINN-based digging trajectory planning method has practical significance as it could be used to improve efficiency of robotic excavators, as well as provide a new technological base for continuous operation of autonomous digging.

## Declaration of competing interest

The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper.

## Data availability

Data will be made available on request.

## Acknowledgments

This work was supported by the National Natural Science Foundation of China [grant numbers 51875232, 52105100 and 52005208], Jilin Province Science and Technology Development Program, China [grant number 20210101382JC], and Science and Technology Research Projects of the Education Department of Jilin Province, China [grant number JJKH20220987KJ].

## Appendix. Nomenclature

See Table A.7 References

<!-- Meanless: 13-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

<!-- Media -->

Table A. 7

Nomenclature.

<table><tr><td>Symbol/(Unit)</td><td>Explanation</td><td>Symbol/(Unit)</td><td>Explanation</td></tr><tr><td>\( a \)</td><td>Coefficient of the polynomial.</td><td>Loss</td><td>Loss of cutting angle constraint.</td></tr><tr><td>\( {\alpha }_{1},{\alpha }_{2},{\alpha }_{3} \)</td><td>\( \angle {O}_{2}{O}_{1}{O}_{3},{O}_{3}{O}_{1}{O}_{4} \) and \( \angle {O}_{4}{O}_{1}{X}_{1} \) .</td><td>Loss \( {}_{4} \)</td><td>Loss of fill factor constraint.</td></tr><tr><td>B</td><td>Bias matrix of neural network.</td><td>Loss</td><td>Loss of performance constraint.</td></tr><tr><td>\( B/\left( \mathrm{m}\right) \)</td><td>Width of trenching task.</td><td>Loss \( {}_{c} \)</td><td>Total loss of constraint</td></tr><tr><td>\( b/\left( \mathrm{m}\right) \)</td><td>Width of bucket digging</td><td>\( {\text{Loss}}_{z} \)</td><td>Total loss function</td></tr><tr><td>\( {\beta }_{1},{\beta }_{2}/\left( \mathrm{{deg}}\right) \)</td><td>Start and end slope angle of trenching task.</td><td>\( \mu \)</td><td>Digging resistance coefficient.</td></tr><tr><td>\( D/\left( \mathrm{m}\right) \)</td><td>Depth of trenching task.</td><td>\( N \)</td><td>Number of samples.</td></tr><tr><td>\( {E}_{e}/\left( \mathrm{{Wh}}\right) \)</td><td>Normalized single digging energy consumption.</td><td>\( {O}_{1}{X}_{1}{Y}_{1}{Z}_{1} \)</td><td>Boom coordinate system(base coordinate system).</td></tr><tr><td>\( {E}_{k},{E}_{p}/\left( \mathrm{J}\right) \)</td><td>Kinetic energy and potential energy of the whole excavator arms.</td><td>\( {O}_{2}{X}_{2}{Y}_{2}{Z}_{2} \)</td><td>Stick coordinate system.</td></tr><tr><td>\( {E}_{\max },{E}_{\min }/\left( \mathrm{{Wh}}\right) \)</td><td>Maximum and minimum of single digging energy consumption.</td><td>\( {O}_{3}{X}_{3}{Y}_{3}{Z}_{3} \)</td><td>Bucket coordinate system.</td></tr><tr><td>\( e \)</td><td>Norm form of \( {\theta }_{i},{\dot{\theta }}_{i},{\ddot{\theta }}_{i},P \) and \( {T}_{a} \) .</td><td>\( {O}_{4}{X}_{4}{Y}_{4}{Z}_{4} \)</td><td>Bucket tip coordinate system.</td></tr><tr><td>\( {f}_{L} \)</td><td>Lagrange function of the whole excavator arms.</td><td>\( P/\left( \mathrm{W}\right) \)</td><td>Driving power of digging.</td></tr><tr><td>\( \zeta /\left( \mathrm{{deg}}\right) \)</td><td>Bucket tip attitude angle.</td><td>\( {P}_{\max }/\left( \mathrm{W}\right) \)</td><td>Maximum value of driving power.</td></tr><tr><td>\( H/\left( \mathrm{m}\right) \)</td><td>Depth used for digging area calculation.</td><td>\( R\left( \mathbf{W}\right) \)</td><td>Regularization function.</td></tr><tr><td>\( h/\left( \mathrm{m}\right) \)</td><td>Depth of digging.</td><td>\( {T}_{q}/\left( \mathrm{{Nm}}\right) \)</td><td>Driving torque of digging.</td></tr><tr><td>\( \theta \)</td><td>Parameters of neural network.</td><td>\( {T}_{q\max }/\left( \mathrm{{Nm}}\right) \)</td><td>Maximum value of driving torque.</td></tr><tr><td>\( {\theta }_{i}/\left( \mathrm{{deg}}\right) ,{\dot{\theta }}_{i}/\left( {\mathrm{{deg}}/\mathrm{s}}\right) ,{\ddot{\theta }}_{i}/\left( {\mathrm{{deg}}/{\mathrm{s}}^{2}}\right) \)</td><td>Angle, angular velocity and angular acceleration of the \( i \) th joint.</td><td>\( {T}_{t}/\left( \mathrm{\;s}\right) \)</td><td>Normalized single digging time.</td></tr><tr><td>\( {\dot{\theta }}_{i\max }/\left( {\mathrm{{deg}}/\mathrm{s}}\right) ,{\ddot{\theta }}_{i\max }/\left( {\mathrm{{deg}}/{\mathrm{s}}^{2}}\right) \)</td><td>Maximum angular velocity and maximum angular acceleration of the \( i \) th joint.</td><td>\( t/\left( \mathrm{\;s}\right) \)</td><td>Time for single digging trajectory.</td></tr><tr><td>\( {\theta }_{i}\left( t\right) /\left( \mathrm{{deg}}\right) \)</td><td>Interpolation function of the \( i \) th joint.</td><td>\( {t}_{C}/\left( \mathrm{\;s}\right) \)</td><td>Computation time.</td></tr><tr><td>\( {\theta }_{p}/\left( \mathrm{{deg}}\right) \)</td><td>Digging angle at the start point.</td><td>\( {t}_{\max },{t}_{\min }/\left( \mathrm{s}\right) \)</td><td>Maximum and minimum of single digging time</td></tr><tr><td>\( {\widehat{\theta }}_{n}/\left( \mathrm{{deg}}\right) \)</td><td>Digging angle at the start point calculated by \( \widehat{\mathbf{Y}} \) .</td><td>\( {V}_{n}/\left( {\mathrm{{cm}}}^{3}\right) \)</td><td>Material volume of single digging</td></tr><tr><td>\( {\theta }_{p\max },{\theta }_{p\min }/\left( \mathrm{{deg}}\right) \)</td><td>Maximum and minimum of digging angle at the start point.</td><td>\( {\widehat{V}}_{p}/\left( {\mathrm{{cm}}}^{3}\right) \)</td><td>Material volume calculated by \( \widehat{\mathbf{Y}} \) .</td></tr><tr><td>\( i \)</td><td>\( i = 1,2,3 \) ,represents the boom,stick,and bucket, respectively.</td><td>\( {V}_{p\max },{V}_{p\min }/\left( {\mathrm{{cm}}}^{3}\right) \)</td><td>Maximum and minimum of single digging material volume.</td></tr><tr><td>\( {K}_{0}/\left( {\mathrm{N}/{\mathrm{{cm}}}^{2}}\right) \)</td><td>Specific resistance coefficient of digging.</td><td>\( {V}_{v}/\left( {\mathrm{{cm}}}^{3}\right) \)</td><td>Normalized single digging material volume.</td></tr><tr><td>\( {\lambda }_{1},{\lambda }_{2},{\lambda }_{3} \)</td><td>Weight coefficient of \( {T}_{t},{E}_{e} \) and \( {V}_{n} \) .</td><td>W</td><td>Weight matrix of neural network.</td></tr><tr><td>\( {\lambda }_{M},{\lambda }_{C},{\lambda }_{F},{\lambda }_{R} \)</td><td>Weight coefficient of \( {\operatorname{Loss}}_{1},{\operatorname{Loss}}_{c},{\operatorname{Loss}}_{5} \) and regularization function.</td><td>\( {W}_{E}/\left( \mathrm{{Wh}}\right) \)</td><td>Energy consumption of single digging.</td></tr><tr><td>\( L/\left( \mathrm{m}\right) \)</td><td>Length of trenching task.</td><td>\( {W}_{Q},{W}_{f}/\left( \mathrm{N}\right) \)</td><td>Digging resistance in tangential and normal direction.</td></tr><tr><td>\( {l}_{i}/\left( \mathrm{m}\right) \)</td><td>Rod length of the \( i \) th joint.</td><td>\( \mathbf{X},\mathbf{Y} \)</td><td>Input and output of training samples.</td></tr><tr><td>Loss</td><td>MSE.</td><td>\( x,y,z/\left( \mathrm{m}\right) \)</td><td>Coordinates of the bucket tip in the base coordinate system.</td></tr><tr><td>\( {\text{Loss}}_{2} \)</td><td>Loss of working device constraint.</td><td>\( \widehat{\mathbf{Y}} \)</td><td>Neural network prediction output</td></tr></table>

<!-- Media -->

[1] A.A. Yusof, M.N.A. Saadun, H. Sulaiman, S.A. Sabaruddin, The development of tele-operated electro-hydraulic actuator (t-EHA) for mini excavator tele-operation, in: 2016 2nd IEEE International Symposium on Robotics and Manufacturing Automation, ROMA, 2016, pp. 1-6, http://dx.doi.org/10.1109/ ROMA.2016.7847800.

[2] S.-U. Lee, P.H. Chang, Control of a heavy-duty robotic excavator using time delay control with integral sliding surface, Control Eng. Pract. 10 (7) (2002) 697-711, http://dx.doi.org/10.1016/S0967-0661(02)00027-8.

[3] O.M.U. Eraliev, K.-H. Lee, D.-Y. Shin, C.-H. Lee, Sensing, perception, decision, planning and action of autonomous excavators, Autom. Constr. 141 (2022) (2022) 1-13, http://dx.doi.org/10.1016/j.autcon.2022.104428.

[4] H.N. Cannon, Extended Earthmoving with an Autonomous Excavator (Master's thesis), Carnegie Mellon University Pittsburgh, Pa, USA, 1999, URL: https:// www.cs.cmu.edu/afs/cs/Web/People/hcannon/thesis.pdf.

[5] J. Seo, S. Lee, J. Kim, S.-K. Kim, Task planner design for an automated excavation system, Autom. Constr. 20 (7) (2011) 954-966, http://dx.doi.org/10.1016/j.autcon.2011.03.013.

[6] J. Gu, J. Taylor, D. Seward, Proportional-integral-plus (PIP) gain scheduling control of an intelligent excavator, in: 21st International Symposium on Automation and Robotics in Construction, Jeju, Korea, 2004, pp. 1-6, http://dx.doi.org/10.22260/ISARC2004/0049.

[7] F. Ng, J.A. Harding, J. Glass, An eco-approach to optimise efficiency and productivity of a hydraulic excavator, J. Clean. Prod. 112 (2016) 3966-3976, http://dx.doi.org/10.1016/j.jclepro.2015.06.110.

[8] Y. Li, R. Fan, L. Yang, B. Zhao, L. Quan, Research status and development trend of intelligent excavators, J. Mach. Eng. 56 (2020) 165-178, http://dx.doi.org/ 10.3901/JME.2020.13.165.

[9] Y. Zhang, Z. Sun, Q. Sun, Y. Wang, X. Li, J. Yang, Time-Jerk optimal trajectory planning of hydraulic robotic excavator, Adv. Mech. Eng. 13 (7) (2021) 1-13, http://dx.doi.org/10.1177/16878140211034611.

[10] Y. Zhao, H.-C. Lin, M. Tomizuka, Efficient trajectory optimization for robot motion planning, in: 2018 15th International Conference on Control, Automation, Robotics and Vision, ICARCV, 2018, pp. 260-265, http://dx.doi.org/10.1109/ ICARCV.2018.8581059.

[11] X. Wang, W. Sun, E. Li, X. Song, Energy-minimum optimization of the intelligent excavating process for large cable shovel through trajectory planning, Struct. Multidiscip. Optim. 58 (5) (2018) 2219-2237, http://dx.doi.org/10.1007/ s00158-018-2011-6.

[12] A. Elnagar, A. Hussein, On optimal constrained trajectory planning in 3D environments, Robot. Auton. Syst. 33 (4) (2000) 195-206, http://dx.doi.org/ 10.1016/S0921-8890(00)00095-6.

[13] Á. Nagy, I. Vajk, Nonconvex time-optimal trajectory planning for robot manipulators, J. Dyn. Syst. Meas. Control 141 (11) (2019) 1-10, http://dx.doi.org/10.1115/1.4044216.

[14] H. Liu, X. Lai, W. Wu, Time-optimal and jerk-continuous trajectory planning for robot manipulators with kinematic constraints, Robot. Comput.-Integr. Manuf. 29 (2) (2013) 309-317, http://dx.doi.org/10.1016/j.rcim.2012.08.002.

[15] Z. Sun, Y. Zhang, H. Li, Q. Sun, Y. Wang, Time optimal trajectory planning of excavator, J. Mech. Eng. 55 (5) (2019) 166-174, http://dx.doi.org/10.3901/ JME.2019.05.166.

[16] L. Wang, Q. Wu, F. Lin, S. Li, D. Chen, A new trajectory-planning beetle swarm optimization algorithm for trajectory planning of robot manipulators, IEEE Access 7 (2019) 154331-154345, http://dx.doi.org/10.1109/ACCESS.2019.2949271.

[17] I. Kurinov, G. Orzechowski, P. Hämäläinen, A. Mikkola, Automated excavator based on reinforcement learning and multibody system dynamics, IEEE Access 8 (2020) 213998-214006, http://dx.doi.org/10.1109/ACCESS.2020.3040246.

[18] B. Son, C. Kim, C. Kim, D. Lee, Expert-emulating excavation trajectory planning for autonomous robotic industrial excavator, in: 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems, IROS, 2020, pp. 2656-2662, http://dx.doi.org/10.1109/IROS45743.2020.9341036.

[19] M. Raissi, P. Perdikaris, G.E. Karniadakis, Physics-informed neural networks: A deep learning framework for solving forward and inverse problems involving nonlinear partial differential equations, J. Comput. Phys. 378 (2019) 686-707, http://dx.doi.org/10.1016/j.jcp.2018.10.045.

[20] S.J. Raymond, N.J. Cecchi, H.V. Alizadeh, A.A. Callan, E. Rice, Y. Liu, Z. Zhou, M. Zeineh, D.B. Camarillo, Physics-informed machine learning improves detection of head impacts, Ann. Biomed. Eng. 50 (11) (2022) 1534-1545, http://dx.doi.org/10.1007/s10439-022-02911-6.

<!-- Meanless: 14-->




<!-- Meanless: Z. Yao et al. Automation in Construction 152 (2023) 104916-->

[21] Q. Zheng, L. Zeng, G.E. Karniadakis, Physics-informed semantic inpainting: application to geostatistical modeling, J. Comput. Phys. 419 (2020) 1-12, http: //dx.doi.org/10.1016/j.jcp.2020.109676.

[22] T. Fu, T. Zhang, Y. Cui, X. Song, Novel hybrid physics-informed deep neural network for dynamic load prediction of electric cable shovel, Chin. J. Mech. Eng. 35 (1) (2022) 1-14, http://dx.doi.org/10.1186/s10033-022-00817-x.

[23] P.-F. Xu, C.-B. Han, H.-X. Cheng, C. Cheng, T. Ge, A physics-informed neural network for the prediction of unmanned surface vehicle dynamics, J. Mar. Sci. Eng. 10 (2) (2022) 1-11, http://dx.doi.org/10.3390/jmse10020148.

[24] C.-T. Chen, T.-T. Liao, A hybrid strategy for the time- and energy-efficient trajectory planning of parallel platform manipulators, Robot. Comput.-Integr. Manuf. 27 (1) (2011) 72-81, http://dx.doi.org/10.1016/j.rcim.2010.06.012.

[25] Y. Du, Y. Chen, Time optimal trajectory planning algorithm for robotic manipulator based on locally chaotic particle swarm optimization, Chin. J. Electron. 31 (5) (2022) 906-914, http://dx.doi.org/10.1049/cje.2021.00.373.

[26] H. Wu, F. Sun, Z. Sun, L. Wu, Optimal trajectory planning of a flexible dual-arm space robot with vibration reduction, J. Intell. Robot. Syst. 40 (2) (2004) 147-163, http://dx.doi.org/10.1023/B:JINT.0000038946.21921.c7.

[27] B.J. Martin, J.E. Bobrow, Minimum-effort motions for open-chain manipulators with task-dependent end-effector constraints, Int. J. Robot. Res. 18 (2) (1999) 213-224, http://dx.doi.org/10.1177/027836499901800206.

[28] S. Yoo, C.-G. Park, S.-H. You, B. Lim, A dynamics-based optimal trajectory generation for controlling an automated excavator, Proc. Inst. Mech. Eng. C 224 (10) (2010) 2109-2119, http://dx.doi.org/10.1243/09544062JMES2032.

[29] M. Boryga, A. Graboš, Planning of manipulator motion trajectory with higher-degree polynomials use, Mech. Mach. Theory 44 (7) (2009) 1400-1419, http: //dx.doi.org/10.1016/j.mechmachtheory.2008.11.003.

[30] H. Wang, H. Wang, J. Huang, B. Zhao, L. Quan, Smooth point-to-point trajectory planning for industrial robots with kinematical constraints based on high-order polynomial curve, Mech. Mach. Theory 139 (2019) 284-293, http://dx.doi.org/ 10.1016/j.mechmachtheory.2019.05.002.

[31] E. Barnett, C. Gosselin, A bisection algorithm for time-optimal trajectory planning along fully specified paths, IEEE Trans. Robot. 37 (1) (2021) 131-145, http: //dx.doi.org/10.1109/TRO.2020.3010632.

[32] D. Jud, G. Hottiger, P. Leemann, M. Hutter, Planning and control for autonomous excavation, IEEE Robot. Autom. Lett. 2 (4) (2017) 2151-2158, http://dx.doi.org/ 10.1109/LRA.2017.2721551.

[33] D. Jud, P. Leemann, S. Kerscher, M. Hutter, Autonomous free-form trenching using a walking excavator, IEEE Robot. Autom. Lett. 4 (4) (2019) 3208-3215, http://dx.doi.org/10.1109/LRA.2019.2925758.

[34] J. Huang, P. Hu, K. Wu, M. Zeng, Optimal time-jerk trajectory planning for industrial robots, Mech. Mach. Theory 121 (2018) 530-544, http://dx.doi.org/ 10.1016/j.mechmachtheory.2017.11.006.

[35] D. Chen, S. Li, J. Wang, Y. Feng, Y. Liu, A multi-objective trajectory planning method based on the improved immune clonal selection algorithm, Robot. Comput.-Integr. Manuf. 59 (2019) 431-442, http://dx.doi.org/10.1016/j.rcim.2019.04.016.

[36] Y.B. Kim, J. Ha, H. Kang, P.Y. Kim, J. Park, F. Park, Dynamically optimal trajectories for earthmoving excavators, Autom. Constr. 35 (2013) 568-578, http://dx.doi.org/10.1016/j.autcon.2013.01.007.

[37] Z. Zou, J. Chen, X. Pang, Task space-based dynamic trajectory planning for digging process of a hydraulic excavator with the integration of Soil-Bucket interaction, Proc. Inst. Mech. Eng. K 233 (3) (2019) 598-616, http://dx.doi.org/ 10.1177/1464419318812589.

[38] Y. Yang, P. Long, X. Song, J. Pan, L. Zhang, Optimization-based framework for excavation trajectory generation, IEEE Robot. Autom. Lett. 6 (2) (2021) 1479-1486, http://dx.doi.org/10.1109/LRA.2021.3058071.

[39] D. Lee, I. Jang, J. Byun, H. Seo, H.J. Kim, Real-time motion planning of a hydraulic excavator using trajectory optimization and model predictive control, in: 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems, IROS, 2021, pp. 2135-2142, http://dx.doi.org/10.1109/IROS51168.2021.9635965.

[40] T. Zhang, T. Fu, Y. Cui, X. Song, Toward autonomous mining: Design and development of an unmanned electric shovel via point cloud-based optimal trajectory planning, Front. Mech. Eng. 17 (3) (2022) 1-17, http://dx.doi.org/ 10.1007/s11465-022-0686-2.

[41] J. Zhao, Y. Hu, C. Liu, M. Tian, X. Xia, Spline-based optimal trajectory generation for autonomous excavator, Machines 10 (7) (2022) 1-17, http://dx.doi.org/10.3390/machines10070538.

[42] T. Osa, M. Aizawa, Deep reinforcement learning with adversarial training for automated excavation using depth images, IEEE Access 10 (2022) 4523-4535, http://dx.doi.org/10.1109/ACCESS.2022.3140781.

[43] N.T.T. Vu, N.P. Tran, N.H. Nguyen, Recurrent neural network-based path planning for an excavator arm under varying environment, Eng. Technol. Appl. Sci. Res. 11 (3) (2021) 7088-7093, http://dx.doi.org/10.48084/etasr.4125.

[44] S. Lee, D. Hong, H. Park, J. Bae, Optimal path generation for excavator with neural networks based soil models, in: 2008 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems, 2008, pp. 632-637, http://dx.doi.org/10.1109/MFI.2008.4648015.

[45] L. Zhang, J. Zhao, P. Long, L. Wang, L. Qian, F. Lu, X. Song, D. Manocha, An autonomous excavator system for material loading tasks, Science Robotics 6 (55) (2021) eabc3164, http://dx.doi.org/10.1126/scirobotics.abc3164.

[46] J. Zhao, L. Zhang, TaskNet: A neural task planner for autonomous excavator, in: 2021 IEEE International Conference on Robotics and Automation, ICRA), 2021, pp. 2220-2226, http://dx.doi.org/10.1109/ICRA48506.2021.9561629.

[47] B.J. Hodel, Learning to operate an excavator via policy optimization, Procedia Comput. Sci. 140 (2018) 376-382, http://dx.doi.org/10.1016/j.procs.2018.10.301.

[48] F.B. Wang, J. Liu, Z.K. Chen, D.M. Guo, The research of neural-fuzzy inference system model for arm of excavator robot, Adv. Mater. Res. 143-144 (2011) 1352-1357, http://dx.doi.org/10.4028/www.scientific.net/AMR.143-144.1352.

[49] A.J. Koivo, M. Thoma, E. Kocaoglan, J. Andrade-Cetto, Modeling and control of excavator dynamics during digging operation, J. Aerosp. Eng. 9 (1) (1996) 10-18, http://dx.doi.org/10.1061/(ASCE)0893-1321(1996)9:1(10).

[50] Z. Towarek, Dynamics of a single-bucket excavator on a deformable soil foundation during the digging of ground, Int. J. Mech. Sci. 45 (6) (2003) 1053-1076, http://dx.doi.org/10.1016/j.ijmecsci.2003.09.004.

[51] B. Fox, L.S. Jennings, A.Y. Zomaya, On the modelling of actuator dynamics and the computation of prescribed trajectories, Comput. Struct. 80 (7) (2002) 605-614, http://dx.doi.org/10.1016/S0045-7949(02)00029-9.

[52] X. Wang, X. Song, W. Sun, Surrogate based trajectory planning method for an unmanned electric shovel, Mech. Mach. Theory 158 (2021) (2021) 1-20, http://dx.doi.org/10.1016/j.mechmachtheory.2020.104230.

[53] R.J. Sandzimier, H.H. Asada, A data-driven approach to prediction and optimal bucket-filling control for autonomous excavators, IEEE Robot. Autom. Lett. 5 (2) (2020) 2682-2689, http://dx.doi.org/10.1109/LRA.2020.2969944.

<!-- Meanless: 15-->

