

<!-- Meanless: Robotics and Computer-Integrated Manufacturing 81 (2023) 102500 Contents lists available at ScienceDirect Robotics and Computer-Integrated Manufacturing ELSEVIER journal homepage: www.elsevier.com/locate/rcim updates-->

# Automatic joint motion planning of 9-DOF robot based on redundancy optimization for wheel hub polishing

Zhiheng Liu \( {}^{a,b} \) ,Ruifeng Li \( {}^{a} \) ,Lijun Zhao \( {}^{a,b, * } \) ,Yi Xia \( {}^{a} \) ,Zhonghao Qin \( {}^{a} \) ,Kui Zhu \( {}^{a} \)

\( {}^{a} \) State Key Laboratory of Robotics and Systems,Harbin Institute of Technology,Harbin 150001,China

\( {}^{b} \) Wuhu Robot Industry Technology Research Institute,Harbin Institute of Technology,Wuhu 241000,China

## ARTICLEINFO

Keywords:

Robotic polishing system

Wheel hub

Offline programming

Kinematic redundancy

Joint motion planning

## A B S T R A C T

Automatic joint motion planning is very important in robotic wheel hub polishing systems. Higher flexibility is achieved based on the joint configuration with multiple solutions, which means that the robot has kinematic redundancy for machining tasks. Redundant joints can be used to optimize the motion of the robot, but less research has been done on multi-dimensional redundant optimization. In this paper, a 6-axis robot with a 3-axis actuator is designed for wheel hub polishing. We propose an automatic joint motion planning method for a nine-axis industrial robot to achieve the shortest processing time. Firstly, offline programming is designed to generate paths for the complex surface of the hub. In order to reduce the machining path points on the surface of the hub, a improved Douglas-Peucker (DP) algorithm is proposed, which can take into account the change of the path point posture. Secondly, the Greedy Best First Search (GBFS) and Sine cosine algorithm (SCA) are combined to find the optimal joint motion efficiently. Moreover, we use nested SCA for comparison to test whether the combined algorithm can avoid local optima. Finally, the performance and computational efficiency of the method are validated in both simulation and real environments based on the hub surface.

## 1. Introduction

With the development of intelligent manufacturing, the application of industrial robots is rapidly increasing in the machining industry due to their flexibility, adaptability, and low cost. At present, industrial robots can be used for complex machining tasks, and are gradually replacing manual labor, such as complex curved surface welding [1], wheel hub polishing [2], blade grinding [3], etc. Among them, because the wheel hub polishing environment is harmful to workers, there is an urgent need for robots to replace manual polishing. To solve this problem, a robotic polishing system for the wheel hub needs to be designed.

Compared with manual polishing, the robotic polishing system has the advantages of high polishing efficiency, high polishing quality, and good product consistency. Several robotic polishing systems are described below. H Ochoa et al. [4] proposed a collaborative robot with 7 Degrees of Freedom (DOF) (Panda, from Franka Emika) for mold polishing based on human demonstration. An effective robot positioning integration algorithm is proposed to ensure the precise task pose of the robot. Mohammad A et al. [5] presented a polishing system containing macro-micro robots, where the macro robots were used to position the micro robots and the micro robots were used for force control. Yu, Z et al. [6] designed a polishing system compatible with any robotic manipulator. The designed system used a ball screw mechanism actuated by a stepper motor to tolerate the variance in surface geometry.

In the robotic polishing system, the robotic automatic path planning is studied due to the complexity of the wheel hub polishing task [7]. Path planning is based on CAD model open source libraries (such as Open-CASCADE [8]) and offline programming platforms (such as ROS [9]) to extract model information to generate machining paths. This process of automatically generating the wheel surface polishing path based on the CAD model is called offline programming [10]. Wang, C et al. [11] developed an offline programming module and simulation module for automatic hole machining. Erds G et al. [12] presented a design method for converting physical work units into parametric digital twins and applied it to the machining of cast aluminum workpieces. Zheng C et al. [13] introduced a hybrid offline programming method based on the interactive activities of vision and CAD, which can adapt to the shipbuilding industry with highly complex and diverse parts. In the above offline programming research, the machining of complex workpieces requires a corresponding offline programming process.

The above-mentioned robotic polishing system adopts polishing equipment with 6 or more degrees of freedom [4-6]. Since polishing only requires 3 degrees of freedom of motion and 2 degrees of freedom of rotation, the robot has a redundancy problem. There are multiple solutions for the robot to reach a given position in the task space. To solve the problem, redundant optimization is studied to obtain the motion paths of robot joints. Redundancy optimization improves robot kinematic performance by proposing optimization goals.

---

<!-- Footnote -->

* Corresponding author.

E-mail address: zhaolj@hit.edu.cn (L. Zhao).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.rcim.2022.102500 Received 29 January 2022; Received in revised form 28 October 2022; Accepted 17 November 2022 Available online 24 November 2022 0736-5845/(© 2022 Elsevier Ltd. All rights reserved.-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

<!-- Media -->

<!-- figureText: ROBOT BASE GLOBAL FLANGE FRAME TASK FRAME TOOL FRAME WORKPIECE ERAME ✘ POSITIONER ✘ FRAME -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_1.jpg?x=413&y=157&w=923&h=761&r=0"/>

Fig. 1. Wheel hub polishing system.

<!-- figureText: J2 J3 J5 J4 J6 -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_1.jpg?x=262&y=1013&w=438&h=494&r=0"/>

Fig. 2. Polishing robot.

<!-- Media -->

Until recently, many optimization goals have been proposed for robot machining [14], such as optimal processing time, optimal joint smoothness,optimal stiffness,etc. Gao J et al. [15] used two-stage optimization strategies to find a time-optimal smooth joint trajectory. This strategy combined the "global coarse search" in the initial stage and the "iterative local fine search" in the final search through heuristic ideas. However, their method had low computational efficiency for multi-dimensional redundancy. Lu Y A et al. [16] planned collision-free and smooth joint motion trajectory for five-axis flank milling. The proposed algorithm combines Dijkstra's shortest path technique and Differential Evolution algorithm to speed up the calculation and avoid getting into a locally optimal solution. However, the DE algorithm is more sensitive to the selection of parameters, and the parameter adjustment process is cumbersome. Dai C et al. [17] built a redundant node graph based on a sampling strategy [18] and optimized the path through two steps. After determining an initial path by graph search, a greedy algorithm is adopted to optimize a path by locally applying adaptive filers in the regions with large jerks. However, the proposed method cannot guarantee the finding of the global optimal solution. Gao W et al. [19] proposed a improved Rapidly-Exploring Random Tree algorithm to solve the continuous trajectory planning problems. Their method avoided the collision by adjusting three Euler angles, but it is too complicated. Due to the relatively low rigidity of industrial robots, many research efforts have been made to improve the rigidity of the robot by optimizing the robot's posture. Therefore, many stiffness indexes have been proposed as optimization targets to increase the stiffness of the robot [20-21]. In summary, the existing redundancy optimization methods have their advantages and disadvantages, among which the optimization of multi-dimensional redundancy is less.

In this paper, a 9-DOF robot is designed for polishing the surface of the wheel hub. This 9-DOF robot consists of a 6-axis serial robot and a 3- axis parallel actuator. This kind of mechanism can improve the flexibility of movement, but also brings about the problem of kinematic redundancy. To solve the motion redundancy problem, an automatic joint motion planning method is proposed, which consists of two parts. The first part is offline programming based on the surface of the hub and proposes a improved Douglas-Peucker algorithm [22] to optimize the path. The purpose is to reduce machining path points. The second part is the optimization of kinematics redundancy, and a improved Sine cosine algorithm [23-24] is proposed. Finally, the joint motion angle of the robot is obtained based on the time-optimal goal.

The remainder of this paper is organized as follows. In Section 2, the Wheel hub polishing system is established. In Section 3, the mathematical model and algorithm for offline programming and redundancy optimization. In Section 4, the effectiveness of the method is verified in both simulated and real environments. Section 5 concludes the paper.

<!-- Meanless: 2-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

<!-- Media -->

<!-- figureText: Passive Spherical Joint Passive Flexure Prismatic Joint Moving Platform Base Platform Active Prismatic Joint -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_2.jpg?x=150&y=154&w=650&h=814&r=0"/>

Fig. 3. Polishing actuator [25].

<!-- Media -->

## 2. Wheel hub polishing system

### 2.1. Modeling the hub polishing system

The main equipment of the wheel hub polishing system includes the polishing robot, the polishing actuator, and the positioner, as shown in Fig. 1. The polishing robotic arm has 6 motion axes, which can provide 6-DOF, including three directions of movement and three directions of rotation. The polishing actuator has 3 motion axes, which can provide 3- DOF, including one direction movement and two direction rotation. The positioner can provide one direction of rotation. The polishing robot carries the end-effector for polishing, which can provide a larger range of motion. Here, the robot adopts a special offset wrist mechanism, it can improve the flexibility of the robot and adjust the posture of the robot more flexibly, as shown in Fig. 2. The polishing actuator is used for polishing, and its mechanism has a small range of posture adjustment functions, which can better adapt to the machining of the complex curved surface of the wheel hub, as shown in Fig. 3. The positioner is used to install the hub to be processed, which can control the rotation of the wheel hub along the axis to improve machining flexibility.

In Fig. 1, the relationship between the three motion mechanisms can be seen. The polishing robot is fixed on the ground, and its base is marked as ROBOT BASE FRAME, the end frame is FLANGE FRAME; The polishing actuator is fixedly installed at the end of the arm, its base frame is FLANGE FRAME, and the end frame is TOOL FRAME; The positioner is fixed on the ground, its base is marked as POSITIONER FRAME, and the end frame is WORKPIECE FRAME; The hub is fixed at the end of the positioner, its base frame is WORKPIECE FRAME, and the frame of the point to be processed on it is TASK FRAME. The positioner is used in part of the polishing process, and its action process is known. The conversion relationship between workpiece frame and positioner frame is known, so the conversion relationship between task frame and global frame can be obtained:

\[{}_{\text{task }}^{\text{global }}\mathbf{T} = {}_{\text{positioner }}^{\text{global }}\mathbf{T} \cdot  {}_{\text{workpiece }}^{\text{positioner }}\mathbf{T} \cdot  {}_{\text{task }}^{\text{workpiece }}\mathbf{T} \tag{1}\]

Where positioner \( \mathbf{T} \) is fixed, \( {}_{\text{workpiece }}^{\text{positioner }}\mathbf{T} \) is known, \( {}_{\text{task }}^{\text{workpiece }}\mathbf{T} \) can be obtained by the hub model and polishing process in Section 3.1. In the polishing process, the end of the polishing actuator is in contact with the machining point of the hub surface. The origins of the tool frame and task frame have coincided, and the transformation matrix is:

\[{}_{\text{task }}^{\text{tool }}\mathbf{T} = \left\lbrack  \begin{matrix} 1 & 0 & 0 & 0 \\  0 &  - 1 & 0 & 0 \\  0 & 0 &  - 1 & 0 \\  0 & 0 & 0 & 1 \end{matrix}\right\rbrack   \tag{2}\]

Therefore, from the conversion of the polishing robot and polishing actuator, the conversion relationship between the global frame and task frame can be obtained:

\[{}_{\text{task }}^{\text{global }}\mathbf{T} = {}_{\text{base }}^{\text{global }}\mathbf{T} \cdot  {}_{\text{flange }}^{\text{base }}\mathbf{T} \cdot  {}_{\text{tool }}^{\text{flange }}\mathbf{T} \cdot  {}_{\text{task }}^{\text{tool }}\mathbf{T} \tag{3}\]

Where \( {}_{\text{base }}^{\text{global }}\mathbf{T} \) is fixed, \( {}_{\text{flange }}^{\text{base }}\mathbf{T} \) and \( {}_{\text{tool }}^{\text{flange }}\mathbf{T} \) can be obtained from the kinematic of the polishing robot and the polishing actuator, respectively.

The polishing robot and polishing actuator are the main actuators for hub polishing, and their kinematics calculation requires the parameters of each joint. Here,the value vector of each joint of the robot is \( {q}_{ri}(i = \)

\( 1,\ldots ,6 \) ),and the value vector of each joint of the actuator is \( {q}_{ai}(i = \) 1,2,3),the formula is obtained as follows:

\[{}_{\text{flange }}^{\text{base }}\mathbf{T} = \mathop{\prod }\limits_{{i = 1}}^{6}{}_{i}^{i - 1}\mathbf{T}\left( {q}_{ri}\right) \]

\[{}_{\text{tool }}^{\text{flange }}\mathbf{T} = \mathbf{T}\left( {{q}_{a1},{q}_{a2},{q}_{a3}}\right)  \tag{4}\]

Where \( {}_{i}^{i - 1}\mathbf{T}\left( {q}_{ri}\right) \) is the transformation matrix of two adjacent joints of the robot, which is obtained according to the Denavit-Hartenberg (DH) method. The transformation matrix is formed by the combination of basic rotation and translation is \( {}_{i}^{i - 1}\mathbf{T}\left( {q}_{ri}\right)  = \mathrm{R}\left( {{Z}_{i - 1},{\theta }_{ri}}\right)  \cdot  \mathrm{T}\left( {{\mathrm{Z}}_{i - 1},{d}_{ri}}\right) \) . \( \mathrm{T}\left( {{\mathrm{X}}_{i},{l}_{ri}}\right)  \cdot  \mathrm{R}\left( {{\mathrm{X}}_{i},{\alpha }_{ri}}\right) \) ,where \( \mathrm{T}\left( {\cdot , \cdot  }\right) \) refers to the translation transformation matrix, \( \mathrm{R}\left( {\cdot , \cdot  }\right) \) refers to the rotation transformation matrix. \( {l}_{ri} \) , \( {\alpha }_{ri},{d}_{ri} \) and \( {\theta }_{ri} \) are generally named as link length,link twist,link offset, and joint angle,respectively. \( \mathbf{T}\left( {{q}_{a1},{q}_{a2},{q}_{a3}}\right) \) is the transformation matrix of the actuator, calculated by the geometric method [25].

According to Eqs. (1) and (3), the conversion relationship between the tool frame and the base frame of the robot can be represented as follows:

\[{}_{\text{flange }}^{\text{base }}\mathbf{T} \cdot  {}_{\text{tool }}^{\text{flange }}\mathbf{T} = {}_{\text{base }}^{\text{global }}\mathbf{T} \cdot  {}_{\text{positioner }}^{\text{global }}\mathbf{T} \cdot  {}_{\text{workpiece }}^{\text{positioner }}\mathbf{T} \cdot  {}_{\text{task }}^{\text{worl }}\mathbf{T} \tag{5}\]

The tool frame \( {}_{\text{tool }}^{\text{base }}\mathbf{T} \) can be obtained from the machining task point pose matrix \( {}_{\text{task }}^{\text{workpiece }}\mathbf{T} \) . The polishing robot and the polishing actuator are taken as a whole to obtain a polishing robot with 9 joints. In addition, setting the value vector of each joint of the robot as \( {q}_{i}\left( {i = 1,\ldots ,9}\right) \) , and the tool frame to the base frame can be represented:

\[{}_{\text{tool }}^{\text{base }}\mathbf{T} = \mathop{\prod }\limits_{{i = 1}}^{6}{}_{i}^{i - 1}\mathbf{T}\left( {q}_{ri}\right)  \cdot  \mathbf{T}\left( {{q}_{a1},{q}_{a2},{q}_{a3}}\right)  = \mathop{\prod }\limits_{{i = 1}}^{6}{}_{i}^{i - 1}\mathbf{T}\left( {q}_{i}\right)  \cdot  \mathbf{T}\left( {{q}_{7},{q}_{8},{q}_{9}}\right) \]

(6)

### 2.2. Polishing redundancy problem

The main mechanism of the designed hub polishing system is the polishing robot and polishing actuator. The polishing robot provides a wide range of polishing and non-machining movements, and the movement speed is fast. For the machining of complex workpieces, it cannot effectively fit the processing surface, and the attitude adjustment is more frequent, which affects the machining ambition. The polishing actuator provides a small range of motion, which can control the polishing tool to fit the processing surface, reduce the adjustment process of the polishing robot, and improve the machining efficiency. The 3 degrees of freedom of the polishing actuator partially coincide with the 6 degrees of freedom of the polishing robot, which are the movement of the \( Z \) -axis and the rotation around the \( X \) -axis and \( Y \) -axis,as shown in Fig. 4. The repetitive degrees of freedom optimize the motion process, increase the flexibility of the motion, and reduce the singularity of the motion. However, the polishing robot and polishing actuator form a 9- DOF robot, which is redundant to the 6-DOF spatial motion. During robot processing, infinite robot joint values are obtained for each end pose, as shown in Fig. 5. This makes the motion of the polishing robot and polishing actuator difficult to achieve. Therefore, there is a need to solve the redundancy problem of robotic polishing hubs.

<!-- Meanless: 3-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

<!-- Media -->

<!-- figureText: ✘ RY RX -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_3.jpg?x=156&y=154&w=643&h=526&r=0"/>

Fig. 4. Redundant joint based on polishing robot and polishing actuator.

<!-- figureText: Polishing X 乙 Polishing path robot Polishing actuator TITTITI -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_3.jpg?x=113&y=784&w=723&h=563&r=0"/>

Fig. 5. The description of robot joint redundancy.The red curve is the polishing path. The fixed points \( \mathrm{O} \) and \( \mathrm{P} \) represent the robot base point and processing point, respectively. The joints of different polishing robots and polishing actuators are given through lines of different colors, such as yellow, blue, green, etc.

<!-- figureText: \( {\mathrm{q}}_{\mathrm{j}}\left( {t}_{\mathrm{i}}\right) \) \( {\mathrm{q}}_{\mathrm{j}}\left( {t}_{\mathrm{i} + 2}\right) \) \( {\phi }_{\mathrm{j}}\left( {t}_{\mathrm{i} + 1}\right) \) \( {\mathrm{q}}_{\mathrm{j}}\left( {t}_{\mathrm{i} + 1}\right) \) \( {\phi }_{\mathrm{j}}\left( {t}_{\mathrm{i}}\right) \) \( {\mathrm{q}}_{\mathrm{j}}\left( {t}_{\mathrm{i} - 1}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_3.jpg?x=228&y=1553&w=498&h=208&r=0"/>

Fig. 6. The corner of the joint path.

<!-- Media -->

### 2.3. Objective function

For the inverse kinematics solution of redundant robots, the joint values are obtained by optimizing the objective function and adding constraint functions. When polishing the hub with the robot, our goal is to find an executable, fast and smooth motion trajectory for the robot. Moreover, the general constraints of the robot are joint motion range, maximum speed, maximum acceleration, etc.

We define \( P = \left\{  {{P}_{0},{P}_{1},\ldots ,{P}_{n}}\right\} \) as a sequence of discrete machining points on the surface of the wheel hub. The information contained in each point includes position and posture. The end of the robot moves along the sequence point in time T, the corresponding time sequence is \( \left\{  {{t}_{0},{t}_{1},\ldots ,{t}_{n}}\right\} \) ,where \( {t}_{0} = 0,{t}_{n} = \mathrm{T} \) . The robot joint motion path is defined as \( \mathbf{q}\left( {t}_{i}\right) \) ,so the robot kinematic constraints are:

\[{P}_{i} = f\left( {q\left( {t}_{i}\right) }\right) \left( {\forall i = 0,1,\ldots ,n}\right)  \tag{7}\]

Where \( f\left( \cdot \right) \) refers to polishing robot kinematics. Additionally,we get that the general motion constraints of the robot are:

\[{q}_{\min } \leq  q\left( {t}_{i}\right)  \leq  {q}_{\max },\left| {\dot{q}\left( {t}_{i}\right) }\right|  \leq  {v}_{\max },\ddot{q}\left( {t}_{i}\right)  \mid   \leq  {a}_{\max } \tag{8}\]

Where \( {\mathbf{q}}_{\min } \) and \( {\mathbf{q}}_{\max } \) are the minimum and maximum of each joint’s range of motion,respectively. \( {\mathbf{v}}_{\max } \) is the maximum velocity of each joint, \( {\mathbf{a}}_{\max } \) is the maximum acceleration of each joint.

Optimize the motion performance of the robot according to the speed and smoothness requirements of the hub polishing. The speed requirement can be achieved by optimizing the processing time of the robot, so the objective function is proposed:

\[\arg \mathop{\min }\limits_{\left\{  q\left( {t}_{i}\right) \right\}  }\mathrm{T} \tag{9}\]

For smoothness requirements, it can be achieved by reducing the fluctuation of the motion path of each joint of the robot. As shown in Fig. 6, the corner of the joint path at the discrete machining point is defined as \( \phi \) ,and the supplementary angle of the corner is \( \left( {\pi  - \phi }\right) \) . The smaller the supplementary angle, the smoother the joint path. We get the maximum corner constraint for each joint path as:

\[\max \left( {\pi  - \left| {\phi \left( {t}_{i}\right) }\right| }\right)  < {\phi }_{\max } \tag{10}\]

## 3. Proposed methodology

The main goal of this paper is to plan fast and smooth joint paths for the redundant robot to polish the surface of the wheel hub. For 5-DOF polishing tasks that do not require rotation around the Z-axis, the 9- DOF robot has 4 redundant joints. Since the polishing tool is installed along the Z-axis of the end, the rotation angle of the robot's 6-axis only affects the rotation attitude of the Z-axis, and this angle can be ignored, as shown in Fig. 1. Therefore, the remaining 3 joints are used for redundancy optimization. The movement of each joint is reasonably distributed so that the end tool can fit the surface of the hub, and the movement is fast and smooth during the machining process. Two reasonable assumptions are made here. (1) The models of polishing robots, polishing actuators, positioners and wheels are known and have high accuracy. (2) The relative positional relationship between the robot and the wheel is known and has high precision.

We propose a robot automatic joint motion planning method based on wheel hub polishing, the process of which is shown in Fig. 7. The method is divided into two steps, the first step is offline programming is based on the surface of the wheel hub, and the second step is kinematics redundancy optimization based on path points.

We can extract feature information through the wheel hub CAD model to generate the machining path points in the workpiece coordinate system. However, the path points are relatively dense, which leads to a large amount of calculation in the subsequent redundant optimization process. First, we analyze the hub processing requirements and divide the hub processing area into redundant optimization areas(ROA) and non-redundant optimization areas(NROA), which can reduce a part of the calculation amount. Second, for the problem of dense path points in redundant optimization regions, a improved DP algorithm is proposed to compress the number of machining path points. The algorithm considers the influence of position and posture on the machining path to obtain the critical path points. Finally, the discrete wheel hub machining path points are obtained, and the following redundancy optimization is performed to obtain the robot joint values.

<!-- Meanless: 4-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

<!-- Media -->

<!-- figureText: Processing area Optimizing waypoints Discreting objective function Creating redundant spaces Wheel model Generating waypoints Automatic joint Robot Offline Programming motion planning method Kinematics redundancy optimization Getting the joint Kinematics redundancy motion path optimization -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_4.jpg?x=354&y=152&w=1036&h=718&r=0"/>

Fig. 7. Algorithm flowchart.

<!-- figureText: Rim Hub End face Hub Hole face Spoke -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_4.jpg?x=116&y=977&w=722&h=458&r=0"/>

Fig. 8. The wheel hub model.

<!-- figureText: W \( {P}_{i} \) \( X \) -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_4.jpg?x=255&y=1537&w=432&h=393&r=0"/>

Fig. 9. Hub surface machining points.

<!-- Media -->

The kinematics redundancy optimization needs to determine the redundant joints and establish the redundant space. Since the polishing actuator is closer to the end tool than the polishing robot, the joints of the actuator are used as redundant joints. Therefore, the redundant space of the robot is established by multiple actuator joint spaces, and the actuator joint spaces are obtained by combining the actuator joints. In addition, since the machining path points are discrete, the objective function and constraint function need to be discrete. Next, a improved SCA is used to optimize the objective function, and the optimal joint values are obtained in the robot redundancy space. So far, the robot joint motion path can be directly used for wheel hub polishing after offline programming and redundancy optimization.

### 3.1. Offline programming for hub

## 1. Divide the hub surface machining area

There are many kinds of car hubs with different shapes, most of which have complex special-shaped surfaces to be polished. In order to facilitate automated processing, these surfaces are divided into the hub end-face (including the rim end-face and the spoke end-face) and the hub hole-face (including the rim side and the spoke side), as shown in Fig. 8. According to the characteristics of the divided areas, the surface of the hub is processed by two processing methods: end-face polishing and side polishing. The end-face polishing uses the edge of the large radius rotating disk to process the hub end-face, and the side polishing uses the side surface of the small radius rotary body to process the hub hole-face. The end-face of the hub can be regarded as a convex curve formed by rotating around the Z-axis of the hub center coordinate system. During the machining process, the robot reciprocates along the curve, and the positioner drives the hub to rotate along the Z-axis. This area belongs to NROA. During the processing, the robot motion is simple and does not require redundant joint motion. The hub hole-face is composed of revolving surfaces with different curvatures, and the shape is relatively complex. This area belongs to ROA. During the polishing process of the robot, the posture of the end changes drastically, and redundant joint motion is required. Therefore, this paper mainly studies the polishing process of the hub hole-face.

<!-- Meanless: 5-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

<!-- Media -->

<!-- figureText: Path position optimization C \( d \) \( {P}_{\mathrm{n}} \) B \( {P}_{\mathrm{n}} \) \( {P}_{1} \) \( {P}_{2} \) \( {P}_{0} \) Path posture optimization \( P \) \( {P}_{2} \) \( {P}_{0} \) -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_5.jpg?x=126&y=148&w=704&h=461&r=0"/>

Fig. 10. Path position and posture are optimized.

Algorithm 1

Improved DP algorithm.

---

Input: The position threshold is \( {\Delta d} \) . The posture threshold is \( {\Delta \theta } \) . Set the machining
																point pose is \( {P}_{i}\left( {i = 0,1,\ldots ,n}\right) \) .
																	Output: Machining point after optimization PO.
																Initialization: \( \mathrm{{PO}} = \left\lbrack  {\mathrm{A},\mathrm{B}}\right\rbrack \) .
															\( A = {P}_{0};B = {P}_{\mathrm{n}}; \)
																	\( {d}_{\max },{P}_{i} \leftarrow \) Helen formula \( \left( {\mathrm{A},\mathrm{B},{P}_{i}}\right) \) ;
																if \( {d}_{\max } > {\Delta d} \) then
															\( \mathbf{{PO}} = \left\lbrack  {\mathbf{{PO}},{P}_{\mathrm{i}}}\right\rbrack \)
																\( \mathbf{{PO}} \leftarrow  \operatorname{Algorithm1}\left( \left\{  {\mathrm{A},\ldots ,{P}_{i}}\right\}  \right) \) ;
																	\( \mathbf{{PO}} \leftarrow  \operatorname{Algorithm1}\left( {\left\{  {{P}_{i},\ldots ,\mathrm{B}}\right\}  \text{,}}\right) \)
																	else
																for \( j = \left( {A + 1}\right) \) to B do
																	\( {\theta }_{\max },{P}_{j} \leftarrow  \operatorname{Eq.}{12}\left( {{P}_{j - 1},{P}_{j}}\right) \) ;
																	if \( {\theta }_{\max } > {\Delta \theta } \) then
																	\( \mathbf{{PO}} = \left\lbrack  {\mathbf{{PO}},{P}_{j}}\right\rbrack \)
																end
																	end
																	end

---

<!-- Media -->

## 2. Generate discrete path points

When the robot polishes the surface of the hub, the machining path points can be realized through online teaching or offline programming. Due to the complex shape of the hub hole-face, offline programming is used to generate machining path points. The OpenCASCADE library is used to import the hub model, obtain geometric feature information, and generate discrete machining points in the workpiece frame. The position and posture of each machining point constitute each TASK FRAME. The position of the machining point \( {\mathbf{P}}_{i} \) is defined as \( \left( {{x}_{i},{y}_{i},{z}_{i}}\right) \) , and the posture is defined as \( {\mathbf{r}}_{i} = \left( {{\mathbf{n}}_{i},{\mathbf{o}}_{i},{\mathbf{a}}_{i}}\right) \) . Fig. 9 shows two machining points \( {\mathbf{P}}_{i},{\mathbf{P}}_{i + 1} \) in the machining path of the hub hole-face. The positions of the machining path points are evenly distributed on the surface of the hub hole,and the interval distance is defined as \( {\Delta s} \) to discretize the machining path into limited machining points. The sparseness of the machining path points is controlled by the parameter \( {\Delta s} \) ,which can be modified according to the complexity of the surface.

The posture of the machining path point is generated as follows: The first step is to obtain the normal vector of the machining point \( {\mathbf{P}}_{i} \) from the geometric information of the model. The X-axis direction \( {\mathbf{n}}_{i} \) of the \( {\mathbf{P}}_{i} \) point is defined as this vector, corresponding to the blue line in Fig. 9. The second step is to calculate the forward direction \( {P}_{i} \rightarrow  {P}_{i + 1} \) of the machining path,which is defined as \( \Delta {\mathbf{p}}_{i} = \left| \right| \left( {{x}_{i + 1} - {x}_{i}}\right) ,\left( {{y}_{i + 1} - {y}_{i}}\right) ,\left( {{z}_{i + 1} - {z}_{i}}\right) \) ||. We assume that the Z-axis direction is perpendicular to the plane formed by the normal direction and the forward direction. The Z-axis direction is obtained according to the right-hand rule, which is defined as \( {\mathbf{a}}_{i} = {\mathbf{n}}_{i} \times  \Delta {\mathbf{p}}_{i} \) ,corresponding to the green line in Fig. 9. Similarly,the Y-axis direction is calculated as \( {\mathbf{o}}_{i} = {\mathbf{a}}_{i} \times  {\mathbf{n}}_{i} \) ,corresponding to the red line in Fig. 9. Finally, the posture of the machining points is generated in turn.

So far, the initial machining path points of the hub hole face can be obtained, see Section 4. In addition, the method can be applied to the waypoint generation in other regions.

## 3. Path optimization strategy

For the initial waypoint generation method,when the parameter \( {\Delta s} \) takes a smaller value, more waypoints are obtained; when the parameter \( {\Delta s} \) takes a larger value,fewer waypoints are obtained. More path points mean that the machining trajectory can better fit the model surface. Therefore,to obtain more path points,the value of parameter \( {\Delta s} \) is small in processing, but it leads to a large number of robot programs and is inconvenient for later maintenance. To solve this problem, a improved DP algorithm is proposed to compress the initial path points and remove the non-key points.

(1) Path preprocessing. The initial path points on the hub hole-face are connected end to end, and the DP algorithm cannot be directly applied for compression optimization. We use the following method to divide the initial path into two paths. The initial path points are projected onto the XY plane of the hub workpiece frame, in Fig. 8. On the plane, calculate the Euclidean distance from the path point to the hub center point \( \mathrm{W} \) respectively. Defining \( {\mathbf{P}}_{\min } \) as the path point with the smallest distance, calculated as follows:

\[{P}_{\min } = \arg \mathop{\min }\limits_{\left\{  {P}_{k}\right\}  }{d}_{i},i = 1,2,\ldots ,k \tag{11}\]

Where \( {d}_{i} \) is the Euclidean distance from the \( \mathrm{i} \) th path point to the W point in the XY plane, \( k \) is the number of path points on the hub hole-face. The path point \( {\mathbf{P}}_{\min } \) is used as the starting point of the path,and the first \( k/2 \) path points are obtained to form the path \( \left\{  {\mathbf{P}}_{n}\right\} \) ,and the rest of the path is defined as \( \left\{  {\mathbf{P}}_{m}\right\} \) . In this way,the path point of the hub hole-face can be divided into two paths that are not connected at the end.

(2) Improved DP algorithm. The DP algorithm reduces the machining path points by setting the threshold and does not affect the machining trajectory. However, this method only compresses and optimizes the position information of the path points without considering the change of posture. If the path point with little change in position and a large change in posture is deleted, it may cause the tool to collide with the workpiece or damage the equipment. Therefore, a improved DP algorithm is proposed, which can adapt to the change of path posture. The process is as follows:

a. For a machining path point is \( \left\{  {\mathbf{P}}_{n}\right\} \) ,the first and last points are connected by the line AB.

b. Using Helen’s formula to calculate the distance \( d \) between the path point on \( \left\{  {\mathbf{P}}_{n}\right\} \) and the straight line AB in turn. The path point with the maximum distance is defined as C, as shown in Fig. 10.

c. Comparing the distance \( d \) with the predetermined threshold \( {\Delta d} \) . If it is less than \( {\Delta d} \) ,then extract the posture information of the path point \( \left\{  {\mathbf{P}}_{n}\right\} \) on the straight line \( \mathrm{{AB}} \) ,where the endpoint \( \mathrm{A} \) corresponds to the path point \( {\mathbf{P}}_{0} \) ,and the endpoint B corresponds to the path point \( {\mathbf{P}}_{n} \) , and go to step (d). If it is greater than \( {\Delta d} \) ,use path point \( \mathrm{C} \) as the dividing point to divide the offline path point into two segments \( \left\{  {\mathbf{P}}_{a}\right\} \) and \( \left\{  {\mathbf{P}}_{b}\right\} \) ,and perform step (a) on the two paths respectively.

d. Starting from the A endpoint, calculate the rotation angle of the posture between the two points in \( \left\{  {\mathbf{P}}_{n}\right\} \) in turn,and end at the \( \mathrm{B} \) endpoint. Path point \( {\mathbf{P}}_{0} \) is used as the comparison path point to verify the change of path point \( {\mathbf{P}}_{1} \) ,as shown in Fig. 9. The calculation formula is as follows:

\[\theta \left( {{r}_{0},{r}_{1}}\right)  = \operatorname{arcos}\left\lbrack  \frac{\operatorname{Tr}\left( {{r}_{0}^{-1} \cdot  {r}_{1}}\right)  - 1}{2}\right\rbrack   \tag{12}\]

<!-- Meanless: 6-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

where \( {\mathbf{r}}_{0},{\mathbf{r}}_{1} \) are the posture matrix of \( {\mathbf{P}}_{0} \) and \( {\mathbf{P}}_{1} \) respectively. \( \theta \left( {{\mathbf{r}}_{0},{\mathbf{r}}_{1}}\right) \) is the rotation angle of \( {\mathbf{P}}_{0} \) and \( {\mathbf{P}}_{1} \) .

<!-- Media -->

<!-- figureText: \( {q}_{a2}^{1} \) \( {q}_{a3}^{k3} \) \( {q}_{a3}^{2} \) \( {q}_{a3}^{1} \) \( {q}_{a1}^{k1} \) \( {q}_{a2}^{2} \) ... ... \( {q}_{a2}^{{k2} - 1} \) \( {q}_{a2}^{k2} \) ... \( {q}_{a1}^{1} \) \( {q}_{a1}^{2} \) ... -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_6.jpg?x=149&y=149&w=656&h=476&r=0"/>

Fig. 11. Actuator joint space diagram.

<!-- figureText: \( {\mathbf{P}}_{0} \) \( {\mathbf{P}}_{1} \) \( {\mathbf{P}}_{2} \) \( {\mathbf{P}}_{i} \) \( {q}^{1} \) \( {q}^{2} \) \( {q}^{j} \) \( {q}^{m} \) -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_6.jpg?x=114&y=712&w=721&h=416&r=0"/>

Fig. 12. Robot redundant space diagram.

<!-- Media -->

e. Comparing the angle \( \theta \) with the predetermined threshold \( {\Delta \theta } \) . If it is less than \( {\Delta \theta } \) ,the path point \( {\mathbf{P}}_{1} \) is not retained,otherwise,keep the path point \( {\mathbf{P}}_{1} \) . The next path point \( {\mathbf{P}}_{2} \) is to be verified,and step (d) is performed.

f. When the path points on \( \left\{  {\mathbf{P}}_{n}\right\} \) are all processed,the polyline formed by connecting each dividing point and the reserved point, in turn, can be used as an approximation of the processing path point.

The path points can be greatly reduced through improved DP algorithm processing. The degree of fit between the path and the model is controlled by a threshold, see Algorithm 1.

### 3.2. Robot kinematics redundancy optimization

## 1. Robot redundant space

During the polishing process of the surface of the wheel hub, each joint of the robot moves along the time series according to the given value. Therefore, when the pose of the discrete path points is known, it is necessary to solve the values of each joint of the robot. However, the inverse kinematics solution of the polishing robot is redundant, and the joint values cannot be directly obtained. We need redundant optimization of robot kinematics, using the redundant three degrees of freedom to establish a redundant space for robots.

The 9-DOF robot consists of a 6-axis polishing robot and a 3-axis polishing actuator, where the polishing actuator is close to the end tool and has a small range of motion. Hence, the three joints of the polishing actuator are used as redundant variables to optimize the motion of the polishing robot. For each pose matrix \( {}_{\text{tool }}^{\text{base }}\mathbf{T} \) on the machining path point,it is assumed that each joint variable \( {q}_{a} \in  \left\lbrack  {{q}_{a}^{\min },{q}_{a}^{\max }}\right\rbrack \) of the polishing actuator is known:

\[{q}_{a1}^{k1} = {q}_{a1}^{\min } + r{a}_{1} \cdot  \left( {{q}_{a1}^{\max } - {q}_{a1}^{\min }}\right) \]

\[{q}_{a2}^{k2} = {q}_{a2}^{\min } + r{a}_{2} \cdot  \left( {{q}_{a2}^{\max } - {q}_{a2}^{\min }}\right) \]

\[{q}_{a3}^{k3} = {q}_{a3}^{\min } + r{a}_{3} \cdot  \left( {{q}_{a3}^{\max } - {q}_{a3}^{\min }}\right)  \tag{13}\]

Where \( r{a}_{1},r{a}_{2},r{a}_{3} \) is the coefficient of each joint respectively,and their numerical range is between \( \left\lbrack  {0,1}\right\rbrack  .{q}_{a1}^{k1},{q}_{a2}^{k2},{q}_{a3}^{k3} \) is a set of joint values of the polished actuator. Besides, the transformation matrix of the polishing actuator is obtained:

\[{}_{\text{tool }}^{\text{flange }}\mathbf{T} = {f}_{\mathrm{a}}\left( {{q}_{a1}^{k1},{q}_{a2}^{k2},{q}_{a3}^{k3}}\right)  \tag{14}\]

where \( {f}_{\mathrm{a}}\left( \cdot \right) \) is the forward kinematics of the polishing actuator. As \( {}_{\text{tool }}^{\text{base }}\mathbf{T} \) and \( {}_{\text{tool }}^{\text{flange }}\mathbf{T} \) are known, \( {}_{\text{flange }}^{\text{base }}\mathbf{T} \) can be got according to the conversion relationship. For a 6-axis polishing robot, each joint value is obtained by the matrix \( {}_{\text{flange }}^{\text{base }}\mathbf{T} \) :

\[{q}_{r}^{m} = {f}_{\mathrm{r}}^{-1}\left( {{}_{\text{flange }}^{\text{base }}\mathbf{T}}\right)  \tag{15}\]

Where \( {f}_{\mathrm{r}}^{-1}\left( \cdot \right) \) represents the inverse kinematics of polishing robot, \( {q}_{r}{}^{m} \) is a set of joint values of the polished robot. Finally, for the machining point \( {\mathbf{P}}_{i} \) ,according to a set of joint values \( {q}_{a1}^{k1},{q}_{a2}^{k2},{q}_{a3}^{k3} \) of the polishing actuator, the joint values of the polishing robot can be obtained:

\[{q}_{r}^{m} = {f}_{\mathrm{r}}^{-1}\left( {{P}_{i},\left( {{q}_{a1}^{k1},{q}_{a2}^{k2},{q}_{a3}^{k3}}\right) }\right) ,\forall i = 1,2,\ldots ,n \tag{16}\]

According to Eq. (13),when \( r{a}_{1},r{a}_{2},r{a}_{3} \) take different values,a series of continuous polished actuator joint values are obtained, which are \( \left\{  {{q}_{a}^{1}{}_{j},{q}_{a}^{2}{}_{j},\ldots ,{q}_{a}^{k}{}_{j}^{j},\ldots }\right\}  j = 1,2,3 \) . For a machining point \( {\mathbf{P}}_{i} \) ,the joints of the polishing actuator are infinite and independent, and an infinite number of joint groups can be obtained by combining them. Creating the actuator joint space, as shown in Fig. 11. According to Eq. (16), the corresponding generated countless polished robot joint values are \( \left\{  {{q}_{r}{}^{1}}\right. \) , \( \left. {{q}_{r}^{2},\ldots ,{q}_{r}^{m},\ldots }\right\} \) .

The robot performs continuous and uninterrupted machining on a single spoke hole-face, and the corresponding machining point sequence is \( \left\{  {{P}_{0},{P}_{1},\ldots ,{P}_{n}}\right\} \) . Each processing point corresponds to \( \mathrm{m} \) groups of joint values is \( \left\{  {{q}^{1},{q}^{2},\ldots ,{q}^{m}}\right\} \) ,and \( n \times  m \) groups of redundant machining points are obtained. Creating the robot redundant space, as shown in Fig. 12.

## 2. Redundancy optimization strategy

## (1) Discretion objective function

The purpose of redundancy optimization is to find a path that satisfies the constraints and has an optimal objective function in the redundant space of the robot. Solve the inverse kinematics for each machining point \( {\mathbf{P}}_{i} \) ,and obtain the unique joint value \( {\mathbf{q}}_{r} \) according to the objective function. Since the machining path points are discrete, the objective function and constraint function need to be discrete.

We take the processing time of a single hub hole as the objective function in Eq. (9). The processing time in the objective function consists of the processing time of each discrete point. The processing time between discrete points is estimated as follows:

\[\Delta {t}_{i} = \mathop{\max }\limits_{{j = 1,\ldots ,6}}\left( {\left| {{q}_{rj}\left( {t}_{i}\right)  - {q}_{rj}\left( {t}_{i - 1}\right) }\right| /{v}_{\max ,j}}\right) ,i = 1,2,\ldots ,n \tag{17}\]

where, calculating the movement time of each joint at the maximum speed, and using the maximum time as the processing time between discrete points. Moreover, the processing time of a single hub hole face is obtained:

<!-- Meanless: 7-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

<!-- Media -->

<!-- figureText: \( {\mathrm{q}}_{\mathrm{i}}\left( {t}_{\mathrm{i} - 1}\right) \) \( {\mathrm{q}}_{\mathrm{j}}\left( {t}_{\mathrm{i} + 1}\right. \) \( {\mathrm{q}}_{\mathrm{j}}\left( {t}_{\mathrm{i}}\right) \) \( \Delta {\mathrm{t}}_{\mathrm{i}} \) -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_7.jpg?x=330&y=157&w=291&h=225&r=0"/>

Fig. 13. Schematic diagram of joint rotation angle.

<!-- Media -->

## Algorithm 2

<!-- Media -->

SCA-GBFS algorithm.

Input: The value range of redundant joints is \( \left\lbrack  {lbub}\right\rbrack \) ,and dimension is \( D \) . The

---

population size is \( \mathrm{M} \) . The maximum number of iterations is \( \mathrm{G} \) . Setting the machining
points is \( {P}_{i}\left( {i = 0,1,\ldots ,\mathrm{N}}\right) \) .
	Output: Processing time T. Robot optimal joint value \( \mathbf{q} \) .
Initialization: Setting the joint value of the actuator to the median value \( {q}_{a}^{0} = \lbrack 1 \) ,
				\( 1,1\rbrack  \cdot  \left( {\mathrm{{ub}} - \mathrm{{lb}}}\right) /2 \) . Calculating the robot joint value of the machining point \( {P}_{0} \)
according to Eq. (15),the joint value is \( {q}_{r}^{0} \) . So the joint value of machining point \( {\mathbf{P}}_{0} \)
is \( {q}^{0} = \left\lbrack  {{q}_{r}^{0},{q}_{a}^{0}}\right\rbrack \) .
for \( i = 1 \) to \( \mathrm{N} \) do
\( {X}_{N \times  D} = \operatorname{rand}\left( {\mathrm{N},\mathrm{D}}\right)  \cdot  \left( {\mathrm{{ub}} - \mathrm{{lb}}}\right)  + \mathrm{{lb}}; \)
\( \Delta {t}_{i, * } = \) inf;
forg \( = 1 \) to \( \mathrm{G} \) do
for \( j = 1 \) to \( \mathrm{M} \) do
\( {q}^{i,j} = \left\lbrack  {{f}_{\mathrm{r}}^{-1}\left( {{P}_{i},X\left( {j, : }\right) }\right) ,X\left( {j, : }\right) }\right\rbrack  ; \)
If \( \left( {i = 1}\right) \parallel \left( {\text{Satisfy constraints}\left( {{q}^{i,j},{q}^{i - 1},{q}^{i - 2}}\right) }\right) \) then
	\( \Delta {t}_{i} = \max \left( {\left| {{q}^{i,j} - {q}^{i - 1}}\right| /{v}_{\max }}\right) ; \)
	else
	\( \Delta {t}_{i} = \max \left( {\operatorname{inv}\left( {\operatorname{Eq}{.19}\left( {a}_{\max }\right) }\right) ,\operatorname{inv}\left( {\operatorname{Eq}{.20}\left( {\phi }_{\max }\right) }\right) }\right) ; \)
	end
	if \( \Delta {t}_{i} < \Delta {t}_{i, * } \) then
	\( \Delta {t}_{i, * } = \Delta {t}_{i};{q}^{i} = {q}^{i,j};{x}_{i, * } = X\left( {j, : }\right) ; \)
	end
	end
\( {X}_{N \times  \mathrm{D}} \leftarrow  \operatorname{Eq.21}\left( {{r}_{1},{r}_{2},{r}_{3},{r}_{4},{x}_{i, * },{X}_{N \times  D}}\right) ; \)
	end
	end
\( T = \mathop{\sum }\limits_{{i = 1}}^{n}\Delta {t}_{i};q = \left\lbrack  {{q}^{0},{q}^{1},\ldots ,{q}^{n}}\right\rbrack \)

---

<!-- Media -->

Algorithm 3

\[T = \mathop{\sum }\limits_{{i = 1}}^{n}\Delta {t}_{i} \tag{18}\]

<!-- Media -->

SC-SCA.

Input: The value range of redundant joints is \( \left\lbrack  {lbub}\right\rbrack \) ,and dimension is \( D \) . The

---

	population size is \( \mathrm{M} \) . The maximum number of iterations is G. Setting the machining
points is \( {P}_{i}\left( {i = 0,1,\ldots ,\mathrm{N}}\right) \) .
	Output: Processing time \( {\mathrm{T}}_{ * } \) . Robot optimal joint value \( \mathbf{q} \) .
\( {X}_{N \times  D} = \operatorname{rand}\left( {\mathrm{N},\mathrm{D}}\right)  \cdot  \left( {\mathrm{{ub}} - \mathrm{{lb}}}\right)  + \mathrm{{lb}}; \)
\( {\mathrm{T}}_{ * } = \) inf;
	for \( g = 1 \) to \( \mathrm{G} \) do
	for \( j = 1 \) to \( \mathrm{M} \) do
	\( {q}^{0} = \left\lbrack  {{f}_{\mathrm{r}}^{-1}\left( {{P}_{0},X\left( {j, : }\right) }\right) ,X\left( {j, : }\right) }\right\rbrack  ; \)
	T, \( q \leftarrow  \operatorname{Algorithm2}\left( {q}^{0}\right) \) ;
if \( T < {\mathrm{T}}_{ * } \) then
\( {\mathrm{T}}_{ * } = \mathrm{T};{q}^{ * } = q;{x}_{ * } = X\left( {j, : }\right) ; \)
	end
	end
\( {X}_{N \times  D} \leftarrow  \operatorname{Eq.20}\left( {{r}_{1},{r}_{2},{r}_{3},{r}_{4},{x}_{ * },{X}_{N \times  D}}\right) ; \)
end

---

<!-- Media -->

For the constraint function Eq. (8), the joint motion range can be directly constrained in the discrete state, the joint velocity is constrained in Eq. (17), and the motion acceleration constraint in the discrete state is estimated:

\[\frac{2 \cdot  \left| {\left( {{q}_{r}\left( {t}_{i + 1}\right)  - {q}_{r}\left( {t}_{i}\right) }\right) /\Delta {t}_{i + 1} - \left( {{q}_{r}\left( {t}_{i}\right)  - {q}_{r}\left( {t}_{i - 1}\right) }\right) /\Delta {t}_{i}}\right| }{\left( \Delta {t}_{i + 1} + \Delta {t}_{i}\right) } \leq  {a}_{\max } \tag{19}\]

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_7.jpg?x=908&y=156&w=719&h=333&r=0"/>

Fig. 14. The discrete path on the hub hole-face. The blue, red, and green lines correspond to the \( X,Y \) ,and \( Z \) axis directions of the processing point attitude, respectively.

<!-- Media -->

The acceleration constraint judgment of each joint is performed at each discrete processing point.

The maximum corner of the joint for each discrete machining point is constrained in Eq. (10), as shown in Fig. 13. According to the law of cosines,the angle of each joint \( {\phi }_{j}\left( {t}_{i}\right) \) of the discrete points is estimated as follows:

\[{\phi }_{\mathrm{j}}\left( {\mathrm{t}}_{\mathrm{i}}\right)  = \arccos \left( \frac{{\mathrm{q}}_{\mathrm{j}}{\left( {\mathrm{t}}_{\mathrm{i}},{\mathrm{t}}_{\mathrm{i} - 1}\right) }^{2} + {\mathrm{q}}_{\mathrm{j}}{\left( {\mathrm{t}}_{i + 1},{\mathrm{t}}_{\mathrm{i}}\right) }^{2} - {\mathrm{q}}_{\mathrm{j}}{\left( {\mathrm{t}}_{i + 1},{\mathrm{t}}_{\mathrm{i} - 1}\right) }^{2}}{2 \cdot  {\mathrm{q}}_{\mathrm{j}}\left( {{\mathrm{t}}_{\mathrm{i}},{\mathrm{t}}_{\mathrm{i} - 1}}\right)  \cdot  {\mathrm{q}}_{\mathrm{j}}\left( {{\mathrm{t}}_{i + 1},{\mathrm{t}}_{\mathrm{i}}}\right) }\right) \]

\[{\mathrm{q}}_{\mathrm{j}}\left( {{\mathrm{t}}_{\mathrm{i}},{\mathrm{t}}_{\mathrm{i} - 1}}\right)  = \sqrt{{\left( {\mathrm{q}}_{\mathrm{j}}\left( {\mathrm{t}}_{\mathrm{i}}\right)  - {\mathrm{q}}_{\mathrm{j}}\left( {\mathrm{t}}_{\mathrm{i} - 1}\right) \right) }^{2} + {\left( \Delta {\mathrm{t}}_{\mathrm{i}}\right) }^{2}}\]

\[{\mathrm{q}}_{\mathrm{j}}\left( {{\mathrm{t}}_{i + 1},{\mathrm{t}}_{\mathrm{i}}}\right)  = \sqrt{{\left( {\mathrm{q}}_{\mathrm{j}}\left( {\mathrm{t}}_{i + 1}\right)  - {\mathrm{q}}_{\mathrm{j}}\left( {\mathrm{t}}_{\mathrm{i}}\right) \right) }^{2} + {\left( \Delta {\mathrm{t}}_{i + 1}\right) }^{2}}\]

\[{q}_{j}\left( {{t}_{i + 1},{t}_{i - 1}}\right)  = \sqrt{{\left( {q}_{j}\left( {t}_{i + 1}\right)  - {q}_{j}\left( {t}_{i - 1}\right) \right) }^{2} + {\left( \Delta {t}_{i} + \Delta {t}_{i + 1}\right) }^{2}}\]

machining path smoothness constraint is achieved by combining Eq. 20 and Eq. (10).

Besides, when the robot polishes the wheel hub, the rotation of the tool Z-axis is not considered, and the robot 6-axis does not participate in the movement. However, the joint values of the robot's 6-axis affect the motion of the polishing actuator. Thus, the 6-axis joint value is fixed during the inverse kinematics solution, and a new transformation matrix tool \( \mathrm{{tool}} \) tool the actual joint values are solved based on the inverse kinematics of the polished actuator.

## (2) Improved Sine cosine algorithm

Through the redundant optimization algorithm, we find the robot motion path that satisfies the constraints and the shortest processing time in the robot redundant space. Usually, redundant joints are dis-cretized to obtain a series of discrete points, and then a graph search algorithm (such as BFS, Dijkstra, etc.) is used for optimization. These methods are complete for optimization when the discrete points are denser and can find the minimum processing time. However, since the optimization process needs to solve the inverse kinematics of the biased robot, the dense discrete points will cause a long time. Hence, we need an optimization algorithm that can take into account both accuracy and time.

SCA is a natural-like optimization algorithm. It uses sine and cosine mathematical models to solve optimization problems by creating multiple random candidate solutions. In the search process, SCA balances the global exploration and local development capabilities of the algorithm by adaptively changing the amplitude of the sine and cosine function, and finally finds the global optimal solution.

Assuming \( {X}_{j} = \left( {{x}_{j1},{x}_{j2},\ldots ,{x}_{j\mathrm{D}}}\right) \) is an individual in the population \( \left( {j = 1,2,\ldots ,\mathrm{M}}\right) \) ,Where \( \mathrm{D} \) represents the dimensionality of the search space, \( \mathrm{M} \) represents the size of the population,and \( {X}_{ * } = \left( {{x}_{*1},{x}_{*2},\ldots }\right. \) , \( {x}_{*\mathrm{D}} \) ) is the global optimal individual. In the optimization process,The position update formula of the k -th \( \left( {k = 1,2,\ldots ,\mathrm{D}}\right) \) dimension of the i th individual is as follows:

<!-- Meanless: 8-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

\[{x}_{ik}^{g + 1} = \left\{  \begin{array}{l} {x}_{ik}^{g} + {r}_{1} \cdot  \sin \left( {r}_{2}\right)  \cdot  \left| {{r}_{3}{x}_{*k}^{g} - {x}_{ik}^{g}}\right| ,{r}_{4} < {0.5} \\  {x}_{ik}^{g} + {r}_{1} \cdot  \cos \left( {r}_{2}\right)  \cdot  \left| {{r}_{3}{x}_{*k}^{g} - {x}_{ik}^{g}}\right| ,{r}_{4} \geq  {0.5} \end{array}\right.  \tag{21}\]

<!-- Media -->

Table 1

The results of the improved DP algorithm and DP algorithm.

<!-- figureText: \( \Delta \mathrm{d} \) Improved DP algorithm DP algorithm 40 20 y(mm) 0 -20 -40 1410 1310 1330 1350 1370 1390 1410 x(mm) The number of path points is 93 40 20 y(mm) 0 -20 -40 1410 1310 1330 1350 1370 1390 1410 x(mm) The number of path points is 61 40 20 y(mm) -20 -40 1410 1310 1330 1350 1370 1390 1410 The number of path points is 42 40 20 y(mm) -20 -40 1410 1310 1330 1350 1370 1390 1410 x(mm) The number of path points is 30 40 20 y(mm) 0 -20 -40 1410 1310 1330 1350 1370 1390 1410 x(mm) The number of path points is 28 /mm 0.05 40 20 y(mm) -20 -40 1310 1330 1350 1370 1390 x(mm) The number of path points is 94 0.1 40 20 y(mm) -20 -40 1310 1330 1350 1370 1390 x(mm) The number of path points is 73 0.2 20 y(mm) -20 -40 1310 1330 1350 1370 1390 The number of path points is 65 0.3 40 20 y(mm) -20 -40 1310 1330 1350 1370 1390 x(mm) The number of path points is 54 0.5 40 20 y(mm) -20 -40 1310 1330 1350 1370 1390 x(mm) The number of path points is 52 -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_8.jpg?x=96&y=216&w=1561&h=1447&r=0"/>

<!-- Media -->

Where \( {x}_{i}^{t}{}_{k},{x}_{i}^{t + 1} \) represents the individual of the \( g \) -th generation and the \( \left( {g + 1}\right) \) -th generation, \( {r}_{2},{r}_{3},{r}_{4} \) are random numbers that obey uniform distribution respectively, \( {r}_{2} \in  \left\lbrack  {0,{2\pi }}\right\rbrack  ,{r}_{3} \in  \left\lbrack  {0,2}\right\rbrack  ,{r}_{4} \in  \left\lbrack  {0,1}\right\rbrack  .{r}_{1} \) is the control parameter, which controls the amplitude of the sine and cosine function, and its value can be obtained as follows:

\[{r}_{1} = a \cdot  \left( {1 - \frac{g}{\mathrm{G}}}\right)  \tag{22}\]

Where \( \mathrm{G} \) is the maximum number of generations,and \( a \) is a constant.

For a single machining point \( {\mathbf{P}}_{i} \) ,the redundant 3 actuator joints are used as the three-dimensional search space. The optimal processing time of a single processing point is solved based on the SCA, and the optimal joint value is obtained. Greedy Best First Search (GBFS) is an efficient image search algorithm. The algorithm starts from the root node, expands, and traverses the nodes of each layer, and according to the priority popping strategy, the highest priority node of each layer is popped up until it reaches the final node. The optimal path is obtained by connecting all the highest priority nodes. For \( n \) -th machining points,the GBFS algorithm is used to search and expand the nodes of each layer, and SCA is used to solve the optimal node of each layer, which can further improve the efficiency of the solution. This algorithm combining SCA and GBFS is called the SCA-GBFS algorithm. The optimization process starts with the initial joint value of the robot, and the optimal processing time of a single point is solved according to the order of machining points. The shorter the processing time, the higher the priority, and the joint value with the highest priority will be popped up as the robot joint value at the current point. Optimize the processing time of n-th points,in turn,to obtain the optimal joint values of \( n \) groups of robots, see Algorithm 2.

<!-- Meanless: 9-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

<!-- Media -->

<!-- figureText: 0.4 Convergence curve Best T(S) obtained SCA 0.075 0.0745 0.074 20 40 60 80 100 Iteration (b) T(S) 0.2 0 40 40 20 20 \( {\mathrm{q}}_{2}\left( \mathrm{{mm}}\right) \) 0 q, (mm) (a) -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_9.jpg?x=400&y=149&w=940&h=481&r=0"/>

Fig. 15. Processing time convergence performance based on the SCA. (a) is the distribution of the processing time of two-dimensional variables; (b) is the convergence curve.

Table 2

Processing time is obtained based on different \( M \) and \( G \) parameters.

<table><tr><td>T/s</td><td>\( M = {10} \)</td><td>\( M = {20} \)</td><td>\( M = {30} \)</td><td>\( M = {40} \)</td></tr><tr><td>\( G = {50} \)</td><td>0.110118</td><td>0.098782</td><td>0.075285</td><td>0.073855</td></tr><tr><td>\( G = {100} \)</td><td>0.108725</td><td>0.087815</td><td>0.073743</td><td>0.072885</td></tr><tr><td>\( G = {200} \)</td><td>0.109627</td><td>0.084864</td><td>0.073360</td><td>0.071551</td></tr><tr><td>\( G = {500} \)</td><td>0.103127</td><td>0.089555</td><td>0.072286</td><td>0.072050</td></tr></table>

<!-- figureText: SC-SCA 30 40 50 Index of path points 0.3 SCA-GBFS rocessing time of a path point(s) 0.25 0.2 0.15 0.1 0.05 0 0 10 20 -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_9.jpg?x=113&y=1031&w=730&h=579&r=0"/>

Fig. 16. The processing time line graph of each path point is obtained based on SCA-GBFS algorithm and SC-SCA algorithm.

Table 3

The results of processing time and calculation time.

<table><tr><td/><td>Processing time(s)</td><td>Calculating time(s)</td></tr><tr><td>Original</td><td>9.420708</td><td>0.055</td></tr><tr><td>SCA-GBFS</td><td>5.845533</td><td>165</td></tr><tr><td>SC-SCA</td><td>5.685742</td><td>495,000</td></tr></table>

<!-- Media -->

The SCA-GBFS algorithm can quickly obtain the optimal processing time for a given initial joint value, but it does not consider the redundancy of the initial joint value. Since the optimization of the machining time is not probabilistically optimal, the influence of the initial joint value is considered. Taking the total processing time obtained under different initial joint values as the objective function, SCA optimization is performed on the three joints with redundant initial points. A two-stage SCA(SC-SCA) was used for comparison, and the specific steps are shown in Algorithm 3.

<!-- Media -->

<!-- figureText: 2.5 Joint1 Joint2 Joint4 Joint5 30 40 50 Index of path points 2 Joint angle value(rad) 1.5 1 0.5 0 -0.5 - 1 -1.5 -2 10 20 -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_9.jpg?x=906&y=752&w=726&h=516&r=0"/>

Fig. 17. Original joint value.

<!-- figureText: 2.5 Joint1 Joint3 Joint4 Joint5 30 40 50 Index of path points 2 Joint angle value(rad) 1.5 1 0.5 0 -0.5 -1 -1.5 -2 10 20 -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_9.jpg?x=928&y=1367&w=683&h=484&r=0"/>

Fig. 18. Joint values optimized by the SCA-GBFS algorithm.

<!-- Meanless: 10-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

<!-- figureText: Polishing Robot Polishing Actuator Pneumatic Spindle Wheel Hub Positioner -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_10.jpg?x=116&y=152&w=720&h=497&r=0"/>

Fig. 19. Experimental platform.

Table 4

Experimental results of joint parameters.

<table><tr><td/><td>Maximum speed (rad/s)</td><td>Minimum speed (rad/ s)</td><td>Maximum acceleration (rad/ \( {\mathrm{s}}^{2}) \)</td><td>Minimum acceleration (rad/ \( {\mathrm{s}}^{2}) \)</td></tr><tr><td>Joint 1</td><td>0.270846</td><td>-0.251755</td><td>3.107234</td><td>-3.510271</td></tr><tr><td>Joint 2</td><td>0.404065</td><td>-0.459743</td><td>5.481948</td><td>-4.968220</td></tr><tr><td>Joint 3</td><td>0.698288</td><td>-0.525897</td><td>5.444007</td><td>-7.940618</td></tr><tr><td>Joint 4</td><td>0.702237</td><td>-0.716520</td><td>6.622720</td><td>-7.132450</td></tr><tr><td>Joint 5</td><td>0.745063</td><td>-0.719511</td><td>9.742878</td><td>-7.942063</td></tr><tr><td>Joint 6</td><td>0</td><td>0</td><td>0</td><td>0</td></tr></table>

<!-- Media -->

The SC-SCA is complete in terms of probability and can ensure that the time-optimal joint value of the robot is found. However, compared with the SCA-GBFS algorithm, the calculation time is longer, and the specific performance is simulated in Section 4.

## 4. Experiment

### 4.1. Simulation environment

In order to verify the effectiveness of the automatic joint motion planning algorithm, the simulation experiments of the offline programming of the hub surface and the optimization of the robot redundancy were carried out based on the hub polishing system. Since the components of the hub are the same, the complex hole-face areas are all composed of irregular toroidal surfaces, and the polishing process is similar. Let's choose a wheel hub as an example. The hole-face of the hub is symmetrically distributed around the circumference. Offline programming generates a path on a hole-face of the hub,sets \( {\Delta s} = 1 \) , and obtains the pointing pose of the continuous machining path, as in Fig. 14. The number of paths to generate a hole-face is 304 , and there are many path points, which cannot be directly used for robot redundancy optimization.

The improved DP algorithm and DP algorithm are optimized for path point compression, and the number of path points and path point distribution are obtained. The path point density \( {\Delta s} = 1 \) ,and the distance between path position points is fixed. Considering the maximum position speed and posture speed of the robot, the posture change threshold \( {\Delta \theta } = {0.1} \) is set. When the parameter \( {\Delta d} \) takes different values,the path point results obtained are shown in Table 1. The diagram in the table shows the projection of the path points on the XY plane of the hub workpiece frame, where the black lines are the paths before compression optimization, and the red points are the path points after compression optimization. It can be seen that as the value of \( d \) is larger,the number of path points obtained after optimization is less. The number of path points is reduced than the number of path points before optimization, indicating that the DP algorithm can effectively compress the machining path points. Comparing the improved DP algorithm and the optimization results of the DP algorithm,it can be seen that when \( {\Delta d} = {0.05} \) , the number of path points obtained is not much different. Due to the small position gap, the path points are relatively dense. The remaining path points differ by about 20 points. Observing the path point distribution of the two algorithms, it can be seen that the increase of path points is mainly at the corners of the hole-face. In this position, the posture change is greater than the position change. If only the position is considered for optimization, the obtained waypoint posture is far from the actual posture. For the hub hole-face, the improved DP algorithm can greatly reduce the path points in the area with smaller curvature, and reserve enough points for the transition of posture in the area with larger curvature. Therefore, the improved DP algorithm can compress the path points and reduce the effect of pose changes.

<!-- Media -->

<!-- figureText: Hole-face after polishing Hole-face before polishing -->

<img src="https://cdn.noedgeai.com/bo_d2slab3ef24c73b2kfbg_10.jpg?x=163&y=1582&w=1427&h=562&r=0"/>

Fig. 20. Wheel hub surface processing effect.

<!-- Media -->

<!-- Meanless: 11-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

SCA-GBFS algorithm is used for robot redundancy optimization, it is necessary to verify that the SCA optimization method is convergent to the objective function. A group of path points is selected, and the processing time is optimized through the SCA. Testing whether the processing time can converge to a fixed value in a limited iteration step. The first group of path points is selected for optimization,setting \( M = {30},G \) \( = {100} \) ,and getting the convergence curve of the objective function in Fig. 15. The processing time is in an inverted cone distribution, with a minimum value, and can quickly converge to a fixed value during the optimization process. Finally, the minimum processing time of this path point is 0.0735 s.

Next, testing the influence of the parameters M and G in the SCA on the convergence of the algorithm. These two parameters represent the population size and the maximum number of iterations respectively. The sixth group of path points is selected for testing, and it is also possible to select other path points. The orthogonal experiment method is used to test two parameters,setting \( M = {10},{20},{30},{40} \) and \( G = {50},{100},{200} \) , 500 respectively, and getting the result in Table 2. The value of the processing time basically decreases with the increase of the parameters \( \mathrm{M} \) and \( \mathrm{G} \) ,but when the parameters are \( M = {30} \) and \( G = {100} \) ,the processing time no longer decreases, and fluctuations will occur. This shows that the processing time has converged to near the minimum value. Increasing the values of the parameters \( \mathrm{M} \) and \( \mathrm{G} \) have little effect on the result. Therefore, when using the SCA-GBFS algorithm for path point simulation,setting \( M = {30} \) and \( G = {100} \) .

Considering the requirements of wheel hub polishing, set the maximum speed, acceleration, and corner of each joint as 0.70 rad/s \( \left( {{40}^{ \circ  }/\mathrm{s}}\right) ,{8.73}\mathrm{{rad}}/{\mathrm{s}}^{2}\left( {{500}^{ \circ  }/{\mathrm{s}}^{2}}\right) \) ,and 1.05 rad \( \left( {60}^{ \circ  }\right) \) in the simulation experiment. The processing time of 55 groups of path points is obtained based on the SCA-GBFS algorithm and the SC-SCA in Fig. 16. The processing time trends obtained by the two algorithms are the same, and the total processing time is also basically the same in Table 3. The processing time has been improved by comparing the two optimization algorithms and the original path. However, the calculation time obtained based on the SCA-GBFS algorithm is much less than that based on the SC-SCA. Although the SCA-GBFS algorithm is not complete in terms of probability, the processing time obtained is similar to that of the SC-SCA. Therefore, the SCA-GBFS algorithm should be used for the surface polishing of the wheel hub. Finally, the first 5 joint values of the robot are given based on the SCA-GBFS algorithm in Fig. 18. Comparing the original joint angle in Fig. 17, it can be clearly seen that the joint angle is much smoother after optimization.

In this part, we show how the improved DP algorithm and the SCA-GBFS algorithm can be used for hub polishing. Offline programming automatically generates machining paths, and redundancy optimization can find the path with the shortest processing time and smooth joint motion. The comparison and verification show that our proposed algorithm performs well in automatic joint motion planning.

### 4.2. Real environment

Our experimental platform is shown in Fig. 19. The pneumatic spindle is installed at the end of the polishing actuator, which can output rated power of \( {240}\mathrm{\;w} \) and rated speed of \( {25},{000}\mathrm{r}/\mathrm{{min}} \) . The polishing robot has a load of \( {70}\mathrm{\;{kg}} \) ,an arm span of \( {1950}\mathrm{\;{mm}} \) ,and a repeatability of \( {0.05}\mathrm{\;{mm}} \) . The range of motion of each joint of the polishing actuator is \( 0 - {40}\mathrm{\;{mm}} \) . The hub is fixed on the positioner.

The main purpose of this polishing process is to remove the attachments from the hub surface. We use the end polishing method to polish the end-face of the hub and the side polishing method to polish the hole-face of the hub. For the ROA of the wheel hub, we perform joint trajectory planning on the redundantly optimized joint paths, and obtain the angle value, motion velocity and acceleration of each joint during the polishing process. In the actual experiment, the processing time of a hole surface of the hub is \( {5.946}\mathrm{\;s} \) ,which increases the acceleration and deceleration time of the start and end of the path compared with the simulation time of 5.846 s. The speed and acceleration trajectory of each joint of the robot basically meet the set maximum speed \( \left( {{0.70}\mathrm{{rad}}/\mathrm{s}}\right) \) and acceleration \( \left( {{8.73}\mathrm{{rad}}/{\mathrm{s}}^{2}}\right) \) . The maximum speed and acceleration of some of the joints exceed the set values, but the difference is not large and does not exceed the performance limit of the robot joints, as shown in Table 4. This is caused by the actual instantaneous trajectory planning, which will be further studied in the subsequent trajectory optimization. Experiments show that the redundant optimization algorithm can reduce the processing time, and the joint parameters are controllable.

After several polishing tests, we completed the polishing of the hub, and the surface processing effect is shown in Fig. 20. The polishing process of the wheel hub requires that the surface be fully machined. We achieved polishing of the entire surface of the wheel hub, especially the hole-face corners. Experiments show that our proposed automatic joint motion planning algorithm is effective in real environments.

## 5. Conclusion

For the need of automatic polishing of wheel hub, a polishing system with a 6-axis robot and a 3-axis actuator is designed. In this paper, we propose a combined SCA-GBFS algorithm for joint motion planning. The hub surface is divided into ROA and NROA. Offline programming is used to automatically generate machining paths for ROA. A improved DP algorithm is proposed to consider the position and attitude of the way-points to compress the number of processing points. Next, take the minimum processing time as the objective function and the corner points of the joint angle curve as the constraint function to optimize the joint motion of the robot. Besides, a redundant space of the robot is established to illustrate the redundant dimensions, and the objective function and constraint function are discretized. Finally, the combined SCA-GBFS algorithm is simulated and compared with SC-SCA. It is verified that the SCA-GBFS algorithm can effectively optimize the joint motion and avoid falling into the local optimal solution. Meanwhile, the robot automatic joint motion planning method is used for wheel hub polishing in the real environment.

## Declaration of competing interest

The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper.

## Data availability

Data will be made available on request.

## Acknowledgement

This work was supported by National Key Research and Development Project of China (Grant Number 2018YFB1308900) and "Ten Thousand Million" Engineering Technology Major Special Support Action Plano of Heilongjiang Province, China (Grant Number SC2021ZX02A0040). References

[1] J. Fan, S. Deng, F. Jing, C. Zhou, M. Tan, An initial point alignment and seam-tracking system for narrow weld, IEEE Trans. Ind. Inform. (99) (2019). PP1-1.

[2] H. Chen, C.Y. Chen, G. Li, Z. Fang, J. Li, Research on polishing parameters analysis for wheel hub, in: 2017 IEEE International Conference on Cybernetics and Intelligent Systems (CIS) and IEEE Conference on Robotics, Automation and Mechatronics (RAM), IEEE, 2017.

[3] H. Wenbo, L. Xiaojun, An optimization study of polishing efficiency of blisk and its technological parameters, Sci. Prog. 103 (3) (2020) 1-16.

<!-- Meanless: 12-->




<!-- Meanless: Z. Liu et al. Robotics and Computer-Integrated Manufacturing 81 (2023) 102500-->

[4] H. Ochoa, C. Rui, Impedance control architecture for robotic-assisted mold polishing based on human demonstration, IEEE Trans. Ind. Electron. 99 (2021). PP. 1-1.

[5] A. Mohammad, J. Hong, D. Wang, Design of a force-controlled end-effector with low-inertial effect for robotic polishing using macro-mini robot approach, Robot. Comput. Integr. Manuf. 49 (2018) 54-65.

[6] Z. Yu, H.I. Lin, Development of robotic polishing/fettling system on ceramic pots, Int. J. Adv. Robot. Syst. 18 (3) (2021), 172988142110128.

[7] Z. Fang, J. Li, C. Zhang, G. Yang, Vision-based initial point alignment control for the wheel hubs in the robotic polishing system, Int. J. Adv. Manuf. Technol. 111 (5) (2020) 1-11.

[8] A.K. Bedaka, C.Y. Lin, CAD-based offline programming platform for welding applications using 6-DOF and 2-DOF robots, in: 2020 International Conference on Advanced Robotics and Intelligent Systems (ARIS), 2020.

[9] Z. Liu, J. Chen, Z. Mei, C. Li, ROS-based robot offline planning simulation system, in: IOP Conference Series: Materials Science and Engineering 711, 2020, 012002 (8pp).

[10] Pedro Neto, Nuno Mendes, Direct off-line robot programming via a common CAD package, Robot. Autonomous Syst. (2013).

[11] C. Wang, W. Wang, Development of off-line programming and simulation system for automatic hole making equipment, in: 2019 IEEE 3rd Information Technology, Networking, Electronic and Automation Control Conference, 2019.

[12] G. Erds, I. Paniti, B. Tipary, Transformation of robotic workcells to digital twins, CIRP Annal. - Manuf. Technol. 69 (1) (2020).

[13] C. Zheng, Y. An, Z. Wang, H. Wu, Y. Zhang, Hybrid offline programming method for robotic welding systems, Robot. Comput. Integr. Manuf. 73 (2022), 102238.

[14] M. Shahabi, H. Ghariblu. Optimal joint motion for complicated welding geometry by a redundant robotic system, Eng. Optim. 52:5, 875-895.

[15] J. Gao, A. Pashkevich, S. Caro, Optimization of the robot and positioner motion in a redundant fiber placement workcell, Mech. Mach. Theory 114 (2018) 170-189.

[16] Y.A. Lu, K. Tang, C.Y. Wang, Collision-free and smooth joint motion planning for six-axis industrial robots by redundancy optimization, Robot. Comput. Integr. Manuf. 68 (2021), 102091.

[17] Dai C., Lefebvre S., Yu K.M., Geraedts J.M.P, Wang C.C.L. Planning jerk-optimized trajectory with discrete time constraints for redundant robots, IEEE Trans. Automat. Sci. Eng.. PP(99), 1-14.

[18] G. Erdös, A. Kovács, J. Váncza, Optimized joint motion planning for redundant industrial robots, CIRP Annal. - Manuf. Technol. 65 (1) (2016).

[19] Gao W., Tang Q., Yao J., Yang Y. Automatic motion planning for complex welding problems by considering angular redundancy, Robot. Comput. Integr. Manuf.. 62: 101862-101862.

[20] Z.Y. Liao, J.R. Li, H.L. Xie, Q.H. Wang, X.F Zhou, Region-based toolpath generation for robotic milling of freeform surfaces with stiffness optimization, Robot. Comput. Integr. Manuf. 64 (2020), 101953.

[21] G. Xiong, Y. Ding, L.M Zhu, Stiffness-based pose optimization of an industrial robot for five-axis milling, Robot. Comput. Integr. Manuf. 55 (2019) 19-28. FEB.

[22] C. Tang, H. Wang, J. Zhao, Y. Tang, Y. Xiao, A method for compressing AIS trajectory data based on the adaptive-threshold Douglas-Peucker algorithm, Ocean Eng. 232 (4) (2021), 109041.

[23] S. Mirjalili, SCA: a sine cosine algorithm for solving optimization problems, Knowl. Based Syst. (2016) 96.

[24] L. Abualigah, A. Diabat, Advances in sine cosine algorithm: a comprehensive survey, Artif. Intell. Rev. (3) (2021).

[25] G. Yang, R. Zhu, Z. Fang, C.Y. Chen, C. Zhang, Kinematic design of a 2R1T robotic End-effector with flexure joints, IEEE Access PP 99 (2020), 1-1.

<!-- Meanless: 13-->

