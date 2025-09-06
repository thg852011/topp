

<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING 1-->

# Segmented Dynamic Adaptive Look-Ahead Smoothing Feedrate Scheduling With Joint Jerk Constraints of 6R Robot Manipulators

Yan Xu \( {}^{ - } \) , Yaqiu Liu \( {}^{ - } \) , Xun Liu, Jiabin Cao \( {}^{ - } \) , and Lin Zhang, Member, IEEE

Abstract-With the continuous development of robotic technology, there is an increasing demand for efficient, smooth, and precise trajectory planning and interpolation. For the robot manipulators with 6 rotational (6R) joints, smooth task paths are crucial for high-quality task and precise dynamic trajectory tracking. However, nonlinear mapping between joint and cartesian space complicates feedrate scheduling under joint constraints. Existing methods, like S-shape feedrate profiles or time-optimal approaches, are inefficient or compromise trajectory stability. This paper proposes a segment-based dynamically adaptive smooth look-ahead feedrate scheduling method based on local dynamic window and a Maximum Velocity Curve (MVC) for \( 6\mathrm{R} \) robot manipulators. It balances the efficiency and stability of motion execution while considering spline trajectory and joint constraints. Five types of segmented velocities under the local dynamic window are identified, with adaptive smoothing strategies developed. Feedrate remains constant within segments and transitions smoothly between them, enhancing trajectory quality. The results of the smoothing preprocessing can be directly used for feedrate profile generation, ensuring smooth, non-oscillating motion while meeting performance and constraint requirements, which is better suited for real-time interpolation. Simulation and experimentation confirm the proposed method's effectiveness.

Note to Practitioners-The motivation of this article stems from the need to develop a feedrate scheduling method with joint jerk constraints of \( 6\mathrm{R} \) robot manipulators for practical tasks like polishing, engraving, welding, and spraying. 6R robots, due to their nonlinear coupling kinematics, often fail to meet joint constraints. Existing methods, inefficient or requiring frequent changes in joint acceleration/jerk for time optimality, adversely affect task quality. To balance smoothness and operational efficiency for \( 6\mathrm{R} \) robot tasks,we propose the segmented dynamic adaptive look-ahead smoothing feedrate scheduling with joint jerk constraints of \( 6\mathrm{R} \) robot manipulators,ensuring trajectory execution within joint limits and balancing smoothness and operational efficiency, which is better suited for real-time interpolation. This method can be of great interest to readers working on precision manufacturing and robotics feedrate profile generation method.

Index Terms-Robot manipulator, joint jerk constraints, trajectory geometric constraints, feedrate scheduling.

## I. INTRODUCTION

ROBOT manipulators with 6 rotational (6R) joints are vital in tasks like surface treatment, assembly, and welding [1], [2], [3]. Non-Uniform Rational B-Splines (NURBS) interpolators are preferred over linear interpolators for \( 6\mathrm{R} \) robot trajectory planning due to smoother feedrates and higher accuracy [4], [5], [6]. Planning 6R robot trajectories involves feedrate scheduling and interpolation. Though position spline [7], [8], [9] or double spline interpolation [10] can be used in \( 6\mathrm{R} \) robot,complex nonlinear mapping between joint and Cartesian space complicates trajectory planning, resulting from the strong coupling of the six robot joints. High Cartesian feedrates may exceed joint constraints, causing vibrations, while low feedrates reduce efficiency. Balancing efficiency, joint constraints, and curve geometry constraints is crucial in \( 6\mathrm{R} \) robot trajectory planning [11].

In Cartesian space trajectory planning, there is a clear lineage between computer numerical control (CNC) systems and robot control systems, e.g. the SIEMENS NC system, SINUMERIK 840Dsl/828D, directly controls 6R robots for tasks like polishing and machining [12], [13], indicating potential cross-applicability of trajectory generation techniques between them. Both domains consider task and joint constraints to ensure process integrity and precision. Zhong et al. [14] addressed axial constraints in three-axis machine tools, devising a real-time interpolator for parameter curves (RTIPC) integrating feedrate and acceleration lookahead operations. Tang et al. [15], [16] proposed a master-based feedrate scheduling (MBFS) approach for five-axis CNC machine tools, adjusting feed parameters using the five-axis feed regulation formula (FFRF) to maintain smooth trajectories along linear tool paths. Beudaert et al. [17] introduced a time-optimal feedrate scheduling method for five-axis NURBS and G1 paths, iteratively computing arc lengths to meet velocity, acceleration, and jerk constraints, applicable to \( 6\mathrm{R} \) robots. Zhou et al. [18] and Zhao et al. [10] respectively proposed feedrate scheduling approaches based on linear programming (LP) optimization to optimize machining time considering various constraints. However, these offline optimization methods are impractical for real-time processing in CNC systems managing numerous tool paths.

---

<!-- Footnote -->

Received 2 August 2024; accepted 9 September 2024. This article was recommended for publication by Associate Editor C. Dai and Editor H. Moon upon evaluation of the reviewers' comments. This work was supported in part by the Fundamental Research Funds for the Central Universities under Grant 2572023CT15-03, in part by the Natural Science Foundation of Chongqing under Grant 2023NSCQ-MSX0758, and in part by the Science and Technology Research Program of Chongqing Municipal Education Commission under Grant KJZD-K202301403. (Corresponding authors: Yaqiu Liu; Lin Zhang.)

Yan Xu and Yaqiu Liu are with the College of Computer and Control Engineering, Northeast Forestry University, Harbin 150040, China (e-mail: xuyan@nefu.edu.cn; yaqiuLiu@nefu.edu.cn).

Xun Liu, Jiabin Cao, and Lin Zhang are with the School of Mechanical Engineering, Shanghai Jiao Tong University, Shanghai 200240, China (e-mail: liux_robot@126.com; caojiabin@sjtu.edu.cn; lin.zhang_2014@hotmail.com).

This article has supplementary material provided by the authors and color versions of one or more figures available at https://doi.org/10.1109/TASE.2024.3458984.

Digital Object Identifier 10.1109/TASE.2024.3458984

<!-- Footnote -->

---

<!-- Meanless: 1545-5955 © 2024 IEEE. Personal use is permitted, but republication/redistribution requires IEEE permission. See https://www.ieee.org/publications/rights/index.html for more information. Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. 2 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING-->

Trajectory planning for \( 6\mathrm{R} \) robots faces challenges in adapting constraint-based techniques from CNC machine tools due to the following reasons: (1) The CNC machine tools typically have simpler configurations with three or five axes, while 6R robots exhibit strong coupling among their six joints, posing complex nonlinear problems; (2) The lightweight nature and flexible joint structures of \( 6\mathrm{R} \) robots lead to less rigidity compared to CNC machine tools [19], [20], exacerbating issues with vibrations and deformations during trajectory execution [21]. Common methods for constrained trajectory generation in \( 6\mathrm{R} \) robots include convex optimization, numerical integration, and online planning [22], [23]. Verscheure et al. [24] applied convex optimization, specifically second-order cone programming (SOCP), to derive time-optimal trajectories considering joint kinematics and torque constraints. M. S. Paing et al. [25] constructs a bi-objective optimization problem to balance time and jerk. Liu et al. [26] decouple time-optimal trajectory generation with nonlinear jerk constraints into two sub-linear programming (LP) problems. The solution from the first sub-LP scales the nonlinear constraints, improving the optimization efficiency and integrating trajectory geometric constraints. Similarly, Nagy and Vajk [27] simplified this into a linear programming (LP) problem, utilizing sequential optimization. Numerical integration methods [28], [29], [30] iteratively derive time-optimal velocity profiles primarily for joint space trajectories. Online planning methods like Ruckig [31] and Reflexxes [32] focus on the constraints in Cartesian space but overlook joint constraints. Additionally, Ruckig's applicability is limited to point-to-point or sparse waypoint trajectories and lacks suitability for high-precision continuous trajectories. Lange and Albu-Schäffer [33] proposed a method suited for online trajectory generation in scenarios with unforeseen events. This method lacks a Cartesian trajectory error constraint mechanism (the chord error constraint [34]) and exhibits significant tracking errors. These methods often sacrifice end-point accuracy and involve frequent changes in joint acceleration/jerk to achieve time optimality, resulting in jerky motion during practical applications and affecting the quality of task.

This paper draws on the segmented real-time look-ahead acceleration and deceleration algorithm of CNC machine tools, considering spline trajectory geometry and robot joint constraints. It establishes a Maximum Velocity Curve (MVC) under multiple constraints. Smooth velocity transitions in segmented curves reduce acceleration and jerk changes. A local dynamic window approach categorizes segmented MVC, and adaptive feedrate smoothing is conducted based on different strategies. Look-ahead feasibility is analyzed, and the results of the smoothing preprocessing are used for real-time velocity planning and interpolation. Compared to traditional spline trajectory feedrate scheduling and interpolation methods, the proposed approach incorporates comprehensive constraints, ensuring trajectory execution within joint limits. Compared to time-optimal feedrate scheduling methods, it enables constant feedrate scheduling within specified intervals, averting drastic fluctuations in acceleration and jerk. This results in smoother feedrate profiles and task trajectories. The method produces smooth end-effector task trajectories, balancing smoothness and operational efficiency for \( 6\mathrm{R} \) robot tasks,aligning better with actual conditions. The main research content and the whole architecture can be shown in Fig.1.

The rest of this paper is organized as follows. Section II defines the MVC for \( 6\mathrm{R} \) robot manipulators under spline trajectory geometric constraints and joint kinematic constraints. Building upon this, Section III proposes a comprehensive adaptive look-ahead feedrate scheduling scheme. The simulation and experiments are conducted in Section IV. Finally, the conclusions are drawn in Section V.

### II.The Maximum Velocity Curve With COMPREHENSIVE MULTI-CONSTRAINTS

The motion relationship between the robot joints and the end-effector can be represented as:

\[\left\{  \begin{array}{l} \dot{\mathbf{q}} = {\mathbf{q}}_{s}\dot{s} \\  \ddot{\mathbf{q}} = {\mathbf{q}}_{ss}{\dot{s}}^{2} + {\mathbf{q}}_{s}\ddot{s} \\  \ddot{\mathbf{q}} = {\mathbf{q}}_{sss}{\dot{s}}^{3} + 3{\mathbf{q}}_{ss}\ddot{s}\dot{s} + {\mathbf{q}}_{s}\ddot{s} \end{array}\right.  \tag{1}\]

where \( \mathbf{q} \in  {\mathbb{R}}^{6} \) represents for the joint position vector; \( \dot{\mathbf{q}},\ddot{\mathbf{q}} \) and \( \ddot{\mathbf{q}} \) are the first-order,second-order,and third-order derivatives of \( q \) with respect to time,i.e.,the joint velocity,acceleration, and jerk vector,respectively; \( s \) represents for end-effector motion distances; thus, \( \dot{s},\ddot{s} \) and \( \dddot{s} \) are end-effector’s feedrate, tangential acceleration,and tangential jerk,respectively; \( {\mathbf{q}}_{s} \) , \( {\mathbf{q}}_{ss} \) and \( {\mathbf{q}}_{sss} \) are the first-order,second-order,and third-order derivatives of \( \mathbf{q} \) with respect to \( s \) ,respectively.

The kinematic constraints of each joint of the robot are known as follows:

\[\left\{  {\begin{array}{l}  - {\dot{\mathbf{q}}}_{\max }\left( k\right)  \leq  \dot{\mathbf{q}}\left( k\right)  \leq  {\dot{\mathbf{q}}}_{\max }\left( k\right) \\   - {\ddot{\mathbf{q}}}_{\max }\left( k\right)  \leq  \ddot{\mathbf{q}}\left( k\right)  \leq  {\ddot{\mathbf{q}}}_{\max }\left( k\right) \\   - {\dot{\mathbf{q}}}_{\max }\left( k\right)  \leq  \dot{\mathbf{q}}\left( k\right)  \leq  {\dot{\mathbf{q}}}_{\max }\left( k\right)  \end{array},k = 1,2,\ldots ,6}\right.  \tag{2}\]

By simultaneously solving (1) and (2), we can obtain:

\[\left| {\dot{\mathbf{q}}\left( k\right) }\right|  = \left| {{\mathbf{q}}_{s}\left( k\right) }\right| \dot{s} \leq  \left| {{\mathbf{q}}_{s}\left( k\right) }\right| {v}_{lim} \leq  {\dot{\mathbf{q}}}_{\max }\left( k\right) ,\;k = 1,2,\ldots ,6\]

(3)

\[\left| {\ddot{\mathbf{q}}\left( k\right) }\right|  = \left| {{\mathbf{q}}_{ss}\left( k\right) {\dot{s}}^{2} + {\mathbf{q}}_{s}\left( k\right) \ddot{s}}\right|  \leq  \left| {{\mathbf{q}}_{ss}\left( k\right) }\right| {\dot{s}}^{2} + \left| {{\mathbf{q}}_{s}\left( k\right) }\right| \left| \ddot{s}\right| \]

\[ \leq  \left| {{\mathbf{q}}_{ss}\left( k\right) }\right| {v}_{lim}^{2} + \left| {{\mathbf{q}}_{s}\left( k\right) }\right| A{t}_{lim} \leq  {\ddot{\mathbf{q}}}_{\max }\left( k\right) ,\]

\[k = 1,2,\ldots ,6 \tag{4}\]

\[\left| {\ddot{\mathbf{q}}\left( k\right) }\right|  = \left| {{\mathbf{q}}_{sss}\left( k\right) {\dot{s}}^{3} + 3{\mathbf{q}}_{ss}\left( k\right) \ddot{s}\dot{s} + {\mathbf{q}}_{s}\left( k\right) \ddot{s}}\right| \]

\[ \leq  \left| {{\mathbf{q}}_{sss}\left( k\right) }\right| {\dot{s}}^{3} + 3\left| {{\mathbf{q}}_{ss}\left( k\right) }\right| \left| \ddot{s}\right| \dot{s} + \left| {{\mathbf{q}}_{s}\left( k\right) }\right| \left| \dddot{s}\right| \]

\[ \leq  \left| {{\mathbf{q}}_{sss}\left( k\right) }\right| {v}_{lim}^{3} + 3\left| {{\mathbf{q}}_{ss}\left( k\right) }\right| A{t}_{lim}{v}_{lim} + \left| {{\mathbf{q}}_{s}\left( k\right) }\right| J{t}_{lim}\]

\[ \leq  {\ddot{\mathbf{q}}}_{\max }\left( k\right) ,\;k = 1,2,\ldots ,6 \tag{5}\]

where \( {v}_{lim},A{t}_{lim} \) and \( J{t}_{lim} \) are the maximum permissible feedrate, acceleration, and jerk of 6R robot end-effector, respectively.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. XU et al.: SEGMENTED DYNAMIC ADAPTIVE LOOK-AHEAD SMOOTHING FEEDRATE SCHEDULING 3-->

<!-- Media -->

<!-- figureText: Off-line preprocessing On-line interpolator A The S-shape feedrate profile generation Arc length increment \( \Delta {s}_{t} \) Obtaining the interpolation points by the second-order Taylor expansion method Moved the window PlanBlock[1] forward Head Tail PlanBlock[1] Buffer After planning the current look- ahead window NURBS curve Order Weights Knot vector Control points Determined the look-ahead window size to be \( N \) Maximum velocity curve Maximum Look-ahead window programmed feedrate Joint velocity Constrained Joint acceleration by Joint jerk Chord error Normal acceleration \( N \) Normal jerk Local dynamic window scanning Look-ahead window- In the current look-ahead window: Segmented dynamic adaptive smoothing preprocessing of MVC based on local dynamic window PlanBlock=\{Seg(i)\} \( i = 1,\cdots ,N \) Local dynamic window -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_2.jpg?x=398&y=147&w=1006&h=745&r=0"/>

Fig. 1. Schematic diagram of the proposed method.

<!-- Media -->

By discretizing the spline curve based on a given arc length step size,we can obtain the trajectory divided into \( n \) segments and \( n + 1 \) points. The derivatives of the robot joint position with respect to path point \( i,{\mathbf{q}}_{s,i},{\mathbf{q}}_{{ss},i} \) ,and \( {\mathbf{q}}_{{sss},i} \) ,can be expressed in the following discrete form:

\[\left\{  \begin{array}{l} {\mathbf{q}}_{s,i} = \frac{{\mathbf{q}}_{i + 1} - {\mathbf{q}}_{i}}{\Delta {s}_{i}} \\  {\mathbf{q}}_{{ss},i} = \frac{{\mathbf{q}}_{i + 1} - 2{\mathbf{q}}_{i} + {\mathbf{q}}_{i - 1}}{\Delta {s}_{i}^{2}} \\  {\mathbf{q}}_{{sss},i} = \frac{{\mathbf{q}}_{i + 1} - 3{\mathbf{q}}_{i} + 3{\mathbf{q}}_{i - 1} - {\mathbf{q}}_{i - 2}}{\Delta {s}_{i}^{3}} \end{array}\right.  \tag{6}\]

where \( {\mathbf{q}}_{i} \) represents the robot joint position obtained through inverse kinematics for path point \( i \) ,while \( \Delta {s}_{i} \) represents the arc length of the discretized curve for the \( i \) -th segment.

Upon discretizing the trajectory, robot joint kinematic constraints on end-effector's feedrate are considered. Assuming that the maximum feedrate is achieved at point \( i \) ,based on the fundamental principles of motion, when robot's end-effector reaches its maximum feedrate, any further acceleration or jerk in the direction of motion would exceed the system's feedrate limit, which is not permissible. Consequently, at the instant the maximum feedrate is achieved, the tangential acceleration/jerk are zero. Combining (3)-(6), we obtain the maximum permissible feedrate at point \( i \) ,satisfying the robot joint kinematic constraints,denoted as \( {v}_{\max ,i}^{q\text{ limit }} \) :

\[{v}_{\max ,i}^{q\text{limit}} = \min \left( {\min \left( \frac{{\dot{\mathbf{q}}}_{\max }}{\left| {\mathbf{q}}_{s,i}\right| }\right) ,\min \left( \sqrt{\frac{{\ddot{\mathbf{q}}}_{\max }}{\left| {\mathbf{q}}_{{ss}.i}\right| }}\right) ,\min \left( \sqrt[3]{\frac{{\ddot{\mathbf{q}}}_{\max }}{\left| {\mathbf{q}}_{{sss}.i}\right| }}\right) }\right) \]

(7)

The influence of trajectory geometry constraints on the end-effector's feedrate also needs to be considered for spline trajectories. High curvature areas affect high-speed motion dynamics. Thus, it is necessary to compute the maximum permissible feedrate \( {v}_{\max ,i}^{cr} \) at discrete point \( i \) ,while considering the chord error \( \delta \) ,maximum normal acceleration \( A{n}_{\max } \) and normal jerk \( J{n}_{\max } \) constraints [35]:

\[{v}_{\max ,i}^{cr} = \min \left( {\frac{2}{{T}_{s}}\sqrt{\frac{2\delta }{{\kappa }_{i}} - {\delta }^{2}},\sqrt{\frac{A{n}_{\max }}{{\kappa }_{i}}},\sqrt[3]{\frac{J{n}_{\max }}{{\kappa }_{i}^{2}}}}\right)  \tag{8}\]

where \( {T}_{s} \) is the sampling period, \( {\kappa }_{i} \) is the curvature of the spline. The maximum permissible feedrate of end-effector at discrete point \( i \) ,considering the kinematic constraints of the robot joints and the geometric constraints of the trajectory, can be obtained by combining (7) and (8):

\[{v}_{{lim},i} = \min \left( {{v}_{\max ,i}^{q\text{ limit }},{v}_{\max ,i}^{cr},{V}_{\max }}\right)  \tag{9}\]

where \( {V}_{\max } \) represents the maximum programmed feedrate. Based on the above analysis, it can be observed that when both tangential acceleration and jerk are zero, although the maximum permissible feedrate can be achieved, it is unrealistic to perform acceleration/deceleration within segmented trajectories. Directly using constant programmed tangential acceleration/jerk \( A{t}_{\max } \) and \( J{t}_{\max } \) may violate joint constraints due to nonlinear mapping. Therefore, adjusting parameters based on constraints is necessary. In this paper, an analytical method for adaptively computing the maximum permissible tangential acceleration/jerk \( A{t}_{lim} \) and \( J{t}_{lim} \) based on the discrete configuration of each point and joint constraints is proposed without programmed values. Considering that each robot joint’s maximum performance at discrete point \( i \) is only triggered by acceleration/jerk, we can obtain:

\[A{t}_{\max ,i} = \min \left( \frac{{\ddot{\mathbf{q}}}_{\max }}{\left| {\mathbf{q}}_{s,i}\right| }\right)  \tag{10}\]

\[J{t}_{\max ,i} = \min \left( \frac{{\ddot{\mathbf{q}}}_{\max }}{\left| {\mathbf{q}}_{s,i}\right| }\right)  \tag{11}\]

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. 4 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING-->

<!-- Media -->

<!-- figureText: 160 Chord-error limited Normal acceleration limited Normal jerk limited Maximum programmed feedrate 0.6 0.8 Normalization s Feedrate (mm/s) 120 80 40 Joint velocity limited Joint acceleration limited- Joint jerk limited 0 0.2 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_3.jpg?x=287&y=146&w=448&h=355&r=0"/>

Fig. 2. Schematic diagram of the maximum velocity curve.

<!-- Media -->

The proportional adjustment method proposed by Sun et al. [36] is employed, whereby the proportional adjustment parameter \( {k}_{v} \) can be calculated to determine the maximum permissible parameters that satisfy the joint constraints within the acceleration/deceleration phase, as follows:

\[\left\{  \begin{array}{l} {v}_{\text{lim }} = {k}_{v} \cdot  {V}_{\max } \\  A{t}_{\text{lim }} = {k}_{v}^{2} \cdot  A{t}_{\max } \\  J{t}_{\text{lim }} = {k}_{v}^{3} \cdot  J{t}_{\max } \end{array}\right.  \tag{12}\]

By combining (12) with (3)-(5), inequalities determining the requirement for \( {k}_{v,i} \) at discrete point \( i \) can be derived as follows:

\[\left\{  \begin{array}{l} {\dot{\mathbf{q}}}_{\max } \geq  {k}_{v,i}\left| {\mathbf{q}}_{s,i}\right| {V}_{\max } \\  {\ddot{\mathbf{q}}}_{\max } \geq  {k}_{v,i}^{2}\left( {\left| {\mathbf{q}}_{{ss},i}\right| {V}_{\max }^{2} + \left| {\mathbf{q}}_{s,i}\right| A{t}_{\max ,i}}\right) \\  {\dddot{\mathbf{q}}}_{\max } \geq  {k}_{v,i}^{3}\left( {\left| {{\mathbf{q}}_{{sss},i}\left| {{V}_{\max }^{3} + 3}\right| {\mathbf{q}}_{{ss},i}}\right| A{t}_{\max ,i}{V}_{\max } + \left| {\mathbf{q}}_{s,i}\right| J{t}_{\max ,i}}\right)  \end{array}\right. \]

(13)

Hence, the following can be obtained:

\[{k}_{v,i} = \min \left( {\frac{{\dot{\mathbf{q}}}_{\max }}{\left| {\mathbf{q}}_{s,i}\right| },\sqrt{\frac{{\ddot{\mathbf{q}}}_{\max }}{\left| {\mathbf{q}}_{s,i}\right| {V}_{\max }^{2} + \left| {\mathbf{q}}_{s,i}\right| A{t}_{\max ,i}}},}\right. \]

\[\sqrt[3]{\frac{{\ddot{\mathbf{q}}}_{\max }}{\left| {\mathbf{q}}_{{sss},i}\right| {V}_{\max }^{3} + 3\left| {\mathbf{q}}_{{ss},i}\right| A{t}_{\max ,i}{V}_{\max } + \left| {\mathbf{q}}_{s,i}\right| J{t}_{\max ,i}}},1)\]

(14)

The maximum permissible \( {k}_{v,i} \) cannot be higher than 1, as it should not surpass the maximum programmed feedrate. By combining (10)-(12) and (14), the maximum permissible tangential acceleration/jerk \( A{t}_{lim} \) and \( J{t}_{lim} \) that satisfy the joint constraints within the acceleration/deceleration phase can be determined.

Fig. 2 depicts the MVC derived from (9) and combined with (12) to calculate \( A{t}_{lim} \) and \( J{t}_{lim} \) within discrete segments. Keeping programmed feedrate below the MVC ensures adherence to joint and trajectory constraints during motion. Traditionally, bidirectional scanning validates velocity requirements before applying the S-shaped method [7]. However, for curves with short arcs and frequent feedrate fluctuations as shown in the red box, using the traditional approach would inevitably result in frequent changes in feedrate, leading to a stepped feedrate waveform, frequent acceleration/deceleration, and unstable motion causing system vibrations. This paper introduces a dynamic and adaptive look-ahead feedrate scheduling algorithm based on the local dynamic window, smoothing MVC segment-wise without bidirectional scanning for subsequent interpolation, as detailed in the following section.

<!-- Media -->

<!-- figureText: \( v\left( t\right)  \land \) \( {a}_{t}\left( t\right) \) \( {t}_{6} \) \( {a}_{t} \leq  A{t}_{\max } \) \( {a}_{t}\left( t\right) \) \( A{t}_{\max } \) \( A{t}_{\mathrm{{ma}}} \) \( {a}_{t} = A{t}_{\max } \) \( {v}_{3} \) \( {v}_{4} \) \( V \) \( {t}_{3} \) \( {t}_{4} \) \( {t}_{5} \) 0 \( {a}_{t}\left( t\right)  \uparrow  A{t}_{\max } \) 0 \( {j}_{t}\left( t\right)  \uparrow  J{t}_{\max } \) - \( J{t}_{\max } \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_3.jpg?x=926&y=158&w=719&h=424&r=0"/>

Fig. 3. The S-shape feedrate profile with seven sections.

<!-- Media -->

## III. SEGMENTED DYNAMIC ADAPTIVE LOOK-AHEAD SMOOTHING FEEDRATE SCHEDULING BASED ON THE LOCAL DYNAMIC WINDOW FOR THE MVC

## A. S-Shape Acceleration/Deceleration Principle

Here, the S-shaped acceleration/deceleration principle is briefly introduced, crucial for subsequent pre-processing and feedrate scheduling, depicted in Fig.3. Acceleration curves are categorized into two-segment,at \( \leq  A{t}_{\max } \) ,and three-segment types, \( {at} = A{t}_{\max } \) ,with deceleration mirroring acceleration,allowing for the same formulas’ application. \( {t}_{i} \) \( \left( {i = 1,2,\ldots ,7}\right) \) represents the relative time parameter; \( {v}_{s} \) and \( {v}_{e} \) represent the initial and final feedrates,respectively; \( {v}_{i}\left( {i = 1,2,\ldots ,6}\right) \) denotes the final feedrate at each phase; \( A{t}_{\max } \) and \( J{t}_{\max } \) indicate the maximum permissible tangential acceleration and jerk for the current phase. \( {t}_{1},{t}_{2} \) ,and \( {t}_{3} \) are calculated as follows:

\[\left\{  \begin{array}{l} {t}_{1} = \left\{  \begin{array}{ll} \frac{A{t}_{\max }}{J{t}_{\max }}, & \left| {{v}_{e} - {v}_{s}}\right|  > \frac{A{t}_{\max }^{2}}{J{t}_{\max }} \\  \sqrt{\left| {v}_{e} - {v}_{s}\right| }, & \left| {{v}_{e} - {v}_{s}}\right|  \leq  \frac{A{t}_{\max }^{2}}{J{t}_{\max }} \end{array}\right. \\  {t}_{2} = \left\{  \begin{array}{ll} \frac{\left| {{v}_{e} - {v}_{s}}\right|  - \frac{A{t}_{\max }^{2}}{J{t}_{\max }}}{J{t}_{\max }}, & \left| {{v}_{e} - {v}_{s}}\right|  > \frac{A{t}_{\max }^{2}}{J{t}_{\max }} \\  0, & \left| {{v}_{e} - {v}_{s}}\right|  \leq  \frac{A{t}_{\max }^{2}}{J{t}_{\max }} \end{array}\right.  \end{array}\right.  \tag{15}\]

If \( {v}_{s} < {v}_{e} \) ,the acceleration displacement \( {S}_{a}\left( {{v}_{s},{v}_{e}}\right) \) can be derived as:

\[{S}_{a}\left( {{v}_{s},{v}_{e}}\right) \]

\[ = \left\{  \begin{array}{ll} \left( {{v}_{s} + {v}_{e}}\right) \sqrt{\frac{{v}_{e} - {v}_{s}}{J{t}_{\max }}}, & {v}_{e} - {v}_{s} \leq  \frac{A{t}_{\max }^{2}}{J{t}_{\max }} \\  \frac{1}{2}\left( {{v}_{s} + {v}_{e}}\right) \left( {\frac{A{t}_{\max }}{J{t}_{\max }} + \frac{\left( {v}_{e} - {v}_{s}\right) }{A{t}_{\max }}}\right) , & {v}_{e} - {v}_{s} > \frac{A{t}_{\max }^{2}}{J{t}_{\max }} \end{array}\right. \]

(16)

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. XU et al.: SEGMENTED DYNAMIC ADAPTIVE LOOK-AHEAD SMOOTHING FEEDRATE SCHEDULING 5-->

If \( {v}_{s} > {v}_{e} \) ,the deceleration displacement \( {S}_{d}\left( {{v}_{s},{v}_{e}}\right) \) can be derived as:

\[{S}_{d}\left( {{v}_{s},{v}_{e}}\right) \]

\[ = \left\{  \begin{array}{ll} \left( {{v}_{s}v + {v}_{e}}\right) \sqrt{\frac{{v}_{s} - {v}_{e}}{J{t}_{\max }}}, & {v}_{s} - {v}_{e} \leq  \frac{A{t}_{\max }^{2}}{J{t}_{\max }} \\  \frac{1}{2}\left( {{v}_{s} + {v}_{e}}\right) \left( {\frac{A{t}_{\max }}{J{t}_{\max }} + \frac{\left( {v}_{s} - {v}_{e}\right) }{A{t}_{\max }}}\right) , & {v}_{s} - {v}_{e} > \frac{A{t}_{\max }^{2}}{J{t}_{\max }} \end{array}\right. \]

(17)

B. Segmented Dynamic Adaptive Look-Ahead Smoothing Feedrate Pre-Processing Based on the Local Dynamic Window for the MVC

Before preprocessing, determining the length of the lookahead window is crucial. The braking distance, \( {S}_{\text{brake }} \) ,defined as the displacement required to decelerate from the \( {V}_{\max } \) to zero, can be calculated by (17):

\[{S}_{\text{brake }} = \left\{  \begin{array}{ll} {V}_{\max }\sqrt{\frac{{V}_{\max }}{J{t}_{\max }}}, & {V}_{\max } \leq  \frac{A{t}_{\max }^{2}}{J{t}_{\max }} \\  \frac{1}{2}{V}_{\max }\left( {\frac{A{t}_{\max }}{J{t}_{\max }} + \frac{{V}_{\max }}{A{t}_{\max }}}\right) , & {V}_{\max } > \frac{A{t}_{\max }^{2}}{J{t}_{\max }} \end{array}\right. \]

(18)

With a predetermined look-ahead window size \( M \) ,the curve is discretized and arc length \( {S}_{M} \) of \( \mathrm{M} \) discrete segments is computed. Utilizing the method from Section II, the MVC considering \( 6\mathrm{R} \) robot’s joint kinematic and trajectory geometric constraints is derived. \( A{t}_{{lim},M} \) and \( J{t}_{{lim},M} \) at \( M \) - th segment are used to calculate braking distance \( {S}_{\text{brake }} \) . If \( {S}_{M} > {S}_{\text{brake }} \) ,the look-ahead window size \( N = M \) , ensuring adequate braking distance. Otherwise, arc length is cumulatively increased until \( {S}_{M} + {S}_{L} > {S}_{\text{brake }} \) ,updating \( N \) to \( N = M + L \) . If remaining curve segments \( {M}_{\text{left }} < M \) , indicating the final stop segment, the look-ahead window size is updated to \( N = N + {M}_{\text{left }} \) ,representing the last window.

After determining the look-ahead window size \( N \) ,the curve undergoes dynamic scanning and segmentation with a local dynamic window size of 3 , aiming for smoother feedrate scheduling. Segmented curve data is recorded in the data buffer region:

\[\operatorname{Seg}\left( i\right)  = \left\{  {{u}_{s,i},{u}_{e,i},{v}_{s,i},{v}_{e,i},{v}_{{lim},i},A{t}_{i},J{t}_{i},{l}_{i}}\right\}  ,\]

\[i = 1,2,\ldots ,N \tag{19}\]

where \( {u}_{s,i} \) and \( {u}_{e,i} \) represent spline parameter values at the start and end points of the \( i \) -th curve segment,respectively; \( {v}_{s,i} \) and \( {v}_{e,i} \) denote the corresponding feedrates,with the initial value matching \( {v}_{{lim},i} \) for the current segment. \( {v}_{{lim},i} \) is the maximum permissible feedrate for the \( i \) -th segment. \( A{t}_{i} \) and \( J{t}_{i} \) indicate maximum permissible tangential acceleration \( / \) jerk for the \( i \) -th segment,while \( {l}_{i} \) is the segment’s arc length. Based on this data, further feedrate pre-planning occurs. Beginning from the second segment, a local dynamic window of size 3 advances until the second-to-last segment, saving preplanned data for subsequent interpolation. Analysis shows that with a window including segments \( i - 1,i \) ,and \( i + 1 \) ,based on the relationship between \( {v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j} \) ,and \( {v}_{{lim},i + 1}^{j} \) for the three segments,(1) \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} \leq  \operatorname{Seg}\left( {i + 1}\right) \) . \( {v}_{{lim},i + 1}^{j} \leq  \operatorname{Seg}\left( i\right) .{v}_{{lim},i}^{j} \) ,named planUpCase1,as shown in Fig.4; (2) \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} \leq  \operatorname{Seg}\left( i\right) .{v}_{{lim},i}^{j} \leq  \operatorname{Seg}\left( {i + 1}\right) \) . \( {v}_{{lim},i + 1}^{j} \) ,named planUpCase 2,as shown in Fig.5; (3) Seg(i). \( {v}_{{lim},i}^{j} > \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} \geq  \operatorname{Seg}\left( {i + 1}\right) .{v}_{{lim},i + 1}^{j} \) ,named planDownCase1,as shown in Fig.6; (4) \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} \geq \) \( \operatorname{Seg}\left( i\right) .{v}_{{lim},i}^{j} \geq  \operatorname{Seg}\left( {i + 1}\right) .{v}_{{lim},i + 1}^{j} \) ,named planDownCase 2, as shown in Fig.7; (5) Seg(i). \( {v}_{{lim},i}^{j} < \) Seg \( \left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} < \) \( \operatorname{Seg}\left( {i + 1}\right) .{v}_{{lim},i + 1}^{j} \) ,named planDownUpCase,as shown in Fig.8. Five feedrate pre-processing algorithms are proposed for smoothing feedrate across various types of segmented curves under a local dynamic window.

<!-- Media -->

<!-- figureText: V4 Window \( j \) \( {l}_{i - 1}^{J} \) \( {\mathcal{U}}_{S,i} \) \( {u}_{e,i} \) (b) \( {v}_{\text{lim. }i - 1} \) \( {v}_{{lim},i} \) \( {v}_{\text{lim,}i + 1} \) \( i - 1 \) \( i \) \( i + 1 \) \( {u}_{s,i} \) \( {u}_{e,i} \) \( u \) (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_4.jpg?x=923&y=147&w=720&h=335&r=0"/>

Fig. 4. Schematic diagram of planUpCase1: (a) two cases; (b) feedrate pre-planning.

<!-- Media -->

In Algorithm 1, Step 2.2 describes the strategy for handling merged segments. A specific example can be found in the description related to planUpCase 2 and no further details here. Both borrowing displacement and merging necessitate redefining spline parameters within curve segments. When borrowing displacement from segment \( i + 1,\operatorname{Seg}\left( i\right) .{u}_{s,i}^{j} \) remains unaltered, while endpoint parameter values are determined through arc length integration. An unconstrained convex optimization model is established, with the objective function meeting the second-order conditions of a convex function definition. Newton's method can be employed to rapidly find the solution:

\[\operatorname{Seg}\left( i\right) .{u}_{e,i}^{j} = \underset{u \in  \left\lbrack  {\operatorname{Seg}\left( i\right) .{u}_{s,i}^{j},\operatorname{Seg}\left( {i + 1}\right) .{u}_{e,i + 1}^{j}}\right\rbrack  }{\arg \min }{\left( L\left( u\right)  - \operatorname{Seg}\left( i\right) .{l}_{i}^{j}\right) }^{2}\]

(20)

where \( L\left( u\right)  = {\int }_{{Seg}\left( i\right) .{u}_{s,i}^{j}}^{{Seg}\left( i\right) .{u}_{e,i}^{j}}\begin{Vmatrix}{{\mathbf{C}}^{\prime }\left( u\right) }\end{Vmatrix}{du},{\mathbf{C}}^{\prime }\left( u\right) \) represents the first-order tangent vector of the given spline curve. By utilizing Newton’s method,the updated spline parameter \( \operatorname{Seg}\left( i\right) .{u}_{e,i}^{j} \) corresponding to the updated \( \operatorname{Seg}\left( i\right) .{l}_{i}^{j} \) can be obtained,and \( \operatorname{Seg}\left( {i + 1}\right) .{u}_{s,i + 1}^{j} = \operatorname{Seg}\left( i\right) .{u}_{e,i}^{j} \) . The merging process is straightforward,where the intervals \( \left\lbrack  {{u}_{s,i}^{j},{u}_{e,i}^{j}}\right\rbrack \) and \( \left\lbrack  {u}_{s,i + 1}^{j}\right. \) , \( \left. {u}_{e,i + 1}^{j}\right\rbrack \) are combined into a single interval \( \left\lbrack  {{u}_{s,i}^{j},{u}_{e,i + 1}^{j}}\right\rbrack \) ,which is assigned to the merged \( \operatorname{Seg}\left( i\right) \) .

As shown in Fig.5(a), the two aforementioned types can be categorized as planUpCase2. The details are presented as in Algorithm 2.

In Step 2.2 of the Algorithm 2, referring to Fig.5(c), after merging in window \( j \) ,the planUpCase 2 is pursued if \( {v}_{{lim},i + 1}^{j} > {v}_{{lim},i}^{j} \) ; otherwise,planUpCase1 or planDownCase1 in Fig.5(c), \( v{m}_{{lim},i}^{j} \) represents the maximum permissible feedrate for the original merged curve segment \( i \) ,while \( l{m}_{i}^{j} \) represents the arc length of the newly merged segment. If \( \mathop{\lim }\limits_{i}^{j} \geq  {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right)  + {S}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,Algorithm 1 Step 2.1 is executed, as shown by the dashed line ① in Fig.5(c). The planned ① may exceed \( v{m}_{{lim},i}^{j} \) ,and Appendix A presents solutions to this issue, which will not be reiterated here. If not, although it is possible to implement the velocity curve shown as line ③ in Fig.5(c) directly through Algorithm 1 Step 2.3, this would reduce work efficiency. Considering the already merged segments and fulfilling the conditions of Algorithm 1 Step 2.2, if \( \mathop{\operatorname{lm}}\limits_{i}^{j} \geq  {S}_{a}\left( {{v}_{{lim},i - 1}^{j},v{m}_{{lim},i}^{j}}\right)  + {S}_{d}\left( {v{m}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,it means that although the arc length after merging cannot accelerate to the new \( {v}_{{lim},i}^{j} \) ,it can accelerate to \( v{m}_{{lim},i}^{j} \) and then decelerate to \( {v}_{{lim},i + 1}^{j} \) . This scenario is executed according to Step 2.2a- 2.2b of Algorithm 1, as shown by the curve ② in Fig.5(c). If none of these conditions are met, as a precaution, solely the acceleration process from \( {v}_{{lim},i - 1}^{j} \) to \( {v}_{{lim},i + 1}^{j} \) is planned, as shown by the curve ③ in Fig.5(c), as detailed in Algorithm 1 Step 2.3.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. 6 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING-->

<!-- Media -->

<!-- figureText: VA Window \( j \) Window \( j \) V A \( v{m}_{li}^{j} \) \( i + 1 \) \( i - 1 \) \( i + 1 \) \( {u}_{e,i} \) \( {u}_{s,i} \) \( {u}_{e,i} \) (c) \( {v}_{{lim},i + 1} \) V A \( {v}_{{lim},i - 1} \) \( {V}_{\text{lim },i - 1}^{J} \) \( i - 1 \) \( i + 1 \) \( i - 1 \) \( {u}_{s,i} \) \( {u}_{e,i} \) \( u \) \( {u}_{s,i} \) (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_5.jpg?x=327&y=146&w=1144&h=346&r=0"/>

Fig. 5. Schematic diagram of planUpCase2: (a) two cases; (b) non-merge situation; (c) merge situation.

---

Algorithm 1 Feedrate Pre-Planning for planUpCase 1
	Input: Segmented curve data under local dynamic window \( j\operatorname{Seg}\left( {i - 1}\right) \) ,
	\( \operatorname{Seg}\left( i\right) ,\operatorname{Seg}\left( {i + 1}\right) \) ,merged sequence of segments,mergedSeg.
	Output: Updated \( \operatorname{Seg}\left( i\right) \) ,window movement flag,moveFlag,merged
	sequence of segments, mergedSeg
		(1) Calculate \( {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right) \) and \( {S}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \) using (   )																																					(16)
				and (17).
		(2) (2.1) If \( {l}_{i}^{j} \geq  {\mathrm{S}}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right)  + {\mathrm{S}}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,indicating that
				segment \( i \) satisfies the long arc length feedrate scheduling,as shown
				by the dashed line ① in Fig.4(b). Update \( \operatorname{Seg}\left( i\right) .{v}_{s,i}^{j} = {v}_{{lim},i - 1}^{j} \) ,
				\( \operatorname{Seg}\left( i\right) .{v}_{e,i}^{j} = {v}_{{lim},i + 1}^{j} \) ,the local window \( j \) is shifted,moveFlag \( = \)
				moveForward, and go to Step 3. Otherwise, go to Step 2.2.
	(2.2) If merged segments exist under the current window \( j \) and
				mergedSeg(end). \( {v}_{lim} \geq  {v}_{{lim},i + 1}^{j} \) ,go to Step 2.2a. Othe-rwise,go to
				Step 2.3.
				(2.2a) Iterate over each segment \( k \) in mergedSeg,set
				\( {v}_{i,\text{ temp }} = \operatorname{mergedSeg}\left( k\right) .{v}_{lim} \) . If \( {v}_{i,\text{ temp }} \geq  {v}_{{lim},i + 1}^{j} \) ,go to
				Step 2.2b. Otherwise, repeat Step 2.2a.
				(2.2b) Calculate \( {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{i,\text{ temp }}}\right) \) and \( {S}_{d}\left( {{v}_{i,\text{ temp }},{v}_{{lim},i + 1}^{j}}\right) \)
				using (16) and (17). If \( {l}_{i}^{j} \geq  {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{i,\text{ temp }}}\right)  + {S}_{d}\left( {v}_{i,\text{ temp }}\right. \) ,
				\( \left. {v}_{{lim},i + 1}^{j}\right) \) ,update \( \operatorname{Seg}\left( i\right) .{v}_{s,i}^{j} = {v}_{{lim},i - 1}^{j},\operatorname{Seg}\left( i\right) .{v}_{e,i}^{j} = \)
				\( {v}_{{lim},i + 1}^{j},\operatorname{Seg}\left( i\right) .{v}_{{lim},i}^{j} = {v}_{i,\text{ temp }} \) ,the local window \( j \) is shifted,
				moveFlag=moveForward, go to Step 3. Otherwise, go to Step 2.2a.
	(2.3) If \( {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right)  \leq  {l}_{i}^{j} < {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right)  + \)
				\( {S}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,indicating that \( i \) -th segment follows a short arc
				length feedrate scheduling where the actual achievable maximum
				feedrate \( {v}_{a} < {v}_{\text{lim },i}^{j} \) . If \( A{t}_{i} \) and \( J{t}_{i} \) cannot be dynamically adjusted,
				overspeed during deceleration is possible. To be cautious, only the
				acceleration process is executed, as shown by the dashed line ②
				in Fig.4(b). Update Seg(i). \( {v}_{s,i}^{j} = {v}_{{lim},i - 1}^{j} \) ,Seg(i). \( {v}_{e,i}^{j} = {v}_{{lim},i + 1}^{j} \) ,
				Seg(i). \( {v}_{{lim},i}^{j} = {v}_{{lim},i + 1}^{j} \) ,moveFlag=moveForward,and go to Step 3.
				Otherwise, go to Step 2.4.
	(2.4) If \( {l}_{i}^{j} < {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right) \) and \( {l}_{i}^{j} + {l}_{i + 1}^{j} > {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right) \) ,
				indicating that the segment \( i \) is too short for full acceleration,
				borrow displacement from segment \( i + 1 \) to update segment \( i \)
				length \( \operatorname{Seg}\left( i\right) .{l}_{i}^{j} = {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right) \) ,update \( \operatorname{Seg}\left( i\right) .{v}_{s,i}^{j} = \)
				\( {v}_{{lim},i - 1}^{j},\operatorname{Seg}\left( i\right) .{v}_{e,i}^{j} = {v}_{{lim},i + 1}^{j},\operatorname{Seg}\left( i\right) .{v}_{{lim},i}^{j} = {v}_{{lim},i + 1}^{j},\operatorname{Seg}(i + \)
				1). \( {l}_{i}^{j} = \operatorname{Seg}\left( {\mathrm{i} + 1}\right) .{l}_{i}^{j} - {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right)  + {l}_{i}^{j} \) ,shift window
				\( j \) ,moveFlag \( = \) moveForward,and go to Step 3. Otherwise,go to
				Step 2.5.
	(2.5) If \( {l}_{i}^{j} + {l}_{i + 1}^{j} < {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right) \) ,indicating that neither segment
				can complete a full acceleration. Store \( \operatorname{Seg}\left( i\right) \) into the mergedSeg,
				set \( \operatorname{Seg}\left( i\right)  = \operatorname{Seg}\left( {i + 1}\right) \) ,merge the displacements of segments \( i \)
				and \( i + 1 \) and update \( \operatorname{Seg}\left( i\right) .{l}_{i}^{j} = \operatorname{Seg}\left( i\right) .{l}_{i}^{j} + \operatorname{Seg}\left( {i + 1}\right) .{l}_{i + 1}^{j} \) . The
				average values of the tangential acceleration \( A{t}_{i}^{j} \) and jerk \( J{t}_{i}^{j} \) for the
				combined segments are derived from the correspondding parameters.
				Clear Seg \( \left( {i + 1}\right) \) ,and segment \( i + 2 \) becomes the new segment \( i + 1 \) ,
				replan window \( j \) ,and output moveFlag \( = \) remainStill.
		(3) Store the updated \( \operatorname{Seg}\left( i\right) \) into the look-ahead data buffer PlanBlock
				and output moveFlag.
is executed. Taking planUpCase 1 as an example. As shown

---

Algorithm 2 Feedrate Pre-Planning for planUpCase 2

---

Input: Segmented curve data under local dynamic window \( j\operatorname{Seg}\left( {i - 1}\right) \) ,
\( \operatorname{Seg}\left( i\right) \) ,
\( \operatorname{Seg}\left( {i + 1}\right) \) ,merged sequence of segments,mergedSeg.
Output: Updated \( \operatorname{Seg}\left( i\right) \) ,window movement flag,moveFlag,merged
sequence of segments, mergedSeg.
	(1) Calculate \( {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right) \) using (16);
	(2) (2.1) If \( {l}_{i}^{j} \geq  {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right) \) ,indicating adequate arc length for
			complete acceleration, represented by dashed line ① in Fig.5(b), short
			arc length feedrate scheduling is employed. Update \( \operatorname{Seg}\left( i\right) .{v}_{s,i}^{j} = \)
			\( \operatorname{Seg}\left( {i - 1}\right) .{v}_{e,i - 1}^{j} \) ,set moveFlag=moveForward,and go to Step 3.
			Otherwise, go to Step 2.2;
(2.2) If \( {l}_{i}^{j} < {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right) \) ,indicating that the actual maximum
			feedrate \( {v}_{a} < {v}_{\text{lim. }i}^{j} \) ,illustrated by dashed line ② in Fig.5(b),the same
			merging strategy as Algorithm 1 Step 2.5 is applied. Store \( \operatorname{Seg}\left( i\right) \) into
			the mergedSeg, set moveFlag=remainStill and output moveFlag;
	(3) Store the updated \( \operatorname{Seg}\left( i\right) \) into the look-ahead data buffer PlanBlock
			and output moveFlag.

---

<!-- figureText: \( {v}_{\text{lim,}i - 1} \) \( {v}_{{lim},i} \) \( {v}_{\text{ lim },i + 1} \) \( i + 1 \) \( {u}_{e,i} \) \( u \) \( i - 1 \) \( {u}_{s,i} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_5.jpg?x=1109&y=1147&w=348&h=297&r=0"/>

Fig. 6. Schematic diagram of planDownCase1.

<!-- Media -->

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. XU et al.: SEGMENTED DYNAMIC ADAPTIVE LOOK-AHEAD SMOOTHING FEEDRATE SCHEDULING 7-->

These two scenarios can be considered similar and referred to as planDownCase1 in Fig.6. The details of the algorithm are presented as follows, where Step 2.1 and 2.2 are similar to those in planUpCase1.

<!-- Media -->

Algorithm 3 Feedrate Pre-Planning for planDownCase 1

---

Input: Segmented curve data under local dynamic window \( j\operatorname{Seg}\left( {i - 1}\right) \) ,
\( \operatorname{Seg}\left( i\right) ,\operatorname{Seg}\left( {i + 1}\right) \) ,merged sequence of segments,mergedSeg.
Output: Updated \( \operatorname{Seg}\left( i\right) \) ,window movement flag,moveFlag,merged
sequence of segments, mergedSeg.
	(1) Calculate \( {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right) \; \) and \( \;{S}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \)
	(2) (2.1) If \( {l}_{i}^{j} \geq  {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right)  + {S}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,Seg(i)
			data is updated, and the local window is shifted forwards. Set
			moveFlag=moveForward and go to Steo 3. Otherwise, go to Step 2.2.
(2.2) If merged segments exist within the current window \( j \) ,consider
			merging the segment data as in Algorithm 1 Step 2.2. Otherwise,
			proceed to 2.3 .
(2.3) If \( {S}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right)  \leq  {l}_{i}^{j} < {S}_{a}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right)  + \)
			\( {S}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,Seg(i)data is updated,and the local window
			is shifted forwards. Set moveFlag=moveForward and go to Step 3.
			Otherwise, go to Step 2.4;
(2.4) If \( {l}_{i}^{j} < {S}_{d}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i + 1}^{j}}\right) \) and \( {l}_{i}^{j} + {l}_{i - 1}^{j} > {S}_{d}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,
			it suggests inadequate arc length for complete deceleration.
			Borrowing displacement from segment \( i + 1 \) could surpass segment \( i + \)
			1’s feedrate limit,hence displacement is borrowed from segment \( i - 1 \) ,
			necessitating \( \operatorname{Seg}\left( {i - 1}\right) \) re-planning due to reduced displacement,and
			go to Step 2.4a. Otherwise, go to Step 2.5;
			(2.4a) If \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{s,i - 1}^{j} \geq  \operatorname{Seg}\left( {i - 1}\right) .{v}_{e,i - 1}^{j} \) ,or \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{s,i - 1}^{j} \)
			\( < \operatorname{Seg}\left( {i - 1}\right) .{v}_{e,i - 1}^{j} \) and the updated \( \operatorname{Seg}\left( {i - 1}\right) .{l}_{i - 1}^{j} \) accommodates
			the variable speed process,update \( \operatorname{Seg}\left( i\right) .{l}_{i}^{j} = {S}_{d}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,
			\( \operatorname{Seg}\left( {i - 1}\right) .{l}_{i - 1}^{j} = \operatorname{Seg}\left( {i - 1}\right) .{l}_{i - 1}^{j} - \left( {{S}_{d}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i + 1}^{j}}\right)  - \operatorname{Seg}\left( i\right) .{l}_{i}^{j}}\right) , \)
			update the stored \( \operatorname{Seg}\left( {i - 1}\right) \) in the buffer. Determine \( \operatorname{Seg}\left( {i - 1}\right) .{u}_{e,i - 1}^{j} \)
			and shift the local window forwards. Set moveFlag \( = \) moveForward
			and go to Step 3. Otherwise, go to Step 2.4b.
			(2.4b) Considering \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{s,i - 1}^{j},\operatorname{Seg}\left( {i - 1}\right) .{l}_{i - 1}^{j} \) ,and the
			maximum tangential acceleration/jerk of this segment, calculate the
			new final feedrate \( {v}_{e\text{ new }} \) satisfying arc length requirement. Reset
			and update \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{s,i - 1}^{j} = \operatorname{Seg}\left( {i - 1}\right) .{v}_{e,i - 1}^{j} = \operatorname{Seg}\left( {i - 1}\right) \) .
			\( {v}_{\text{lim. }i - 1}^{j} = {v}_{e\_ \text{new }} \) . Shift the local window back to \( j - 1 \) . Set
			moveFlag=moveBackward and output moveFlag.
(2.5) If \( {l}_{i}^{j} + {l}_{i - 1}^{j} < {S}_{d}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,the displacement of original
			segment \( i - 1 \) and segment \( i \) within the window are combined
			and assigned to the new segment \( i - 1 \) . Store \( \operatorname{Seg}\left( i\right) \) into the
			mergedSeg and update \( \operatorname{Seg}\left( {i - 1}\right) .{l}_{i - 1}^{j} = \operatorname{Seg}\left( {i - 1}\right) .{l}_{i - 1}^{j} + \operatorname{Seg}\left( i\right) \) .
			\( {l}_{i}^{j} \) . Average values of the corresponding parameters for the combined
			segment,namely the tangential acceleration \( A{t}_{i - 1}^{j} \) and jerk \( J{t}_{i - 1}^{j} \) ,
			are calculated,and clear \( \operatorname{Seg}\left( i\right) \) . Set moveFlag \( = \) moveBackward and
			output moveFlag.
			) Store the updated \( \operatorname{Seg}\left( i\right) \) into the look-ahead data buffer PlanBlock
			and output moveFlag.

---

<!-- Media -->

As shown in Fig.7(a), these scenarios are analogous and labelled as planDownCase2, and the details of the algorithm are presented as in Algorithm 4. In the Step 2.2 of the Algorithm 4, unlike planDownCase 1, here re-planning the new segment \( i - 1 \) precedes the new window \( j \) . This is because in planDownCase1, premise for merging is intermediate segments \( i \) unable to attain \( {v}_{{lim},i} \) . To stay within the MVC, the maximum feedrate of the newly merged segment \( i - 1 \) will equal to \( {v}_{{lim},i - 1}^{j} \) of the original segment \( i - 1 \) ,consistent with the original segment \( i - 1 \) ,thus no need to re-plan new segment \( i - 1 \) . As shown in Fig.7(c),in planDownCase 2, segment \( i - 1 \) in the current window \( j \) already planned in the previous window \( j - 1 \) ,and its starting feedrate \( {v}_{s,i - 1} \) has only two possibilities: (1) \( {v}_{s,i - 1} = {v}_{{lim},i - 1}^{j} \) ; (2) \( {v}_{s,i - 1} < {v}_{{lim},i - 1}^{j} \) . In either case, proceed to Step 2.2a-2.2c.

<!-- Media -->

## Algorithm 4 Feedrate Pre-Planning for planDownCase 2

---

Input: Segmented curve data under local dynamic window \( j\operatorname{Seg}\left( {i - 1}\right) \) ,
\( \operatorname{Seg}\left( i\right) ,\operatorname{Seg}\left( {i + 1}\right) \) ,merged sequence of segments,mergedSeg.
Output: Updated \( \operatorname{Seg}\left( i\right) \) ,window movement flag,moveFlag,merged
sequence of segments, mergedSeg.
	(1) Calculate \( {S}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \) using (17).
	(2) (2.1) If \( {l}_{i}^{j} \geq  {S}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,the deceleration process is feasible,
			as shown in Fig.7(b)①. Update \( \operatorname{Seg}\left( i\right) .{v}_{e,i}^{j} = \operatorname{Seg}\left( {i + 1}\right) .{v}_{{lim},i + 1}^{j} \) ,set
			moveFlag=moveForward and go to Step 3. Otherwise, go to Step 2.2;
(2.2) If \( {l}_{i}^{j} < {S}_{d}\left( {{v}_{{lim},i}^{j},{v}_{{lim},i + 1}^{j}}\right) \) ,store \( \operatorname{Seg}\left( i\right) \) into the mergedSeg,merging
			displacement of segment \( i - 1 \) and \( i \) as new displacement of segment
			\( i - 1 \) ,with original segment \( i + 1 \) as new segment \( i \) ,and original
			segment \( i + 2 \) as new segment \( i + 1 \) . Merged parameters are shown
			in the Fig.7(c). Go to Step 2.2a.
			(2.2a) If \( l{m}_{i}^{j} \geq  {S}_{a}\left( {{v}_{s,i - 1},{v}_{{lim},i - 1}^{j}}\right)  + {S}_{d}\left( {{v}_{{lim},i - 1}^{j},{v}_{{lim},i}^{j}}\right) \) ,the new
			segment \( i - 1 \) can satisfy acceleration from \( {v}_{s,i - 1} \) to \( {v}_{{lim},i - 1}^{j} \) and then
			decelerating to \( {v}_{{lim},i}^{j} \) ,as in Fig.7(c)①. Update \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{e,i - 1}^{j} = \)
			\( {v}_{{lim},i}^{j},A{t}_{i - 1}^{j} \) and \( J{t}_{i - 1}^{j} \) after merging are the average values of the
			corresponding parameters in the merged segment. Set moveFlag=
			remainStill and output moveFlag. Otherwise, go to Step 2.2b.
			(2.2b) Consider already merged segment’s data,when \( l{m}_{i}^{j} \geq \)
			\( {S}_{a}\left( {{v}_{s,i - 1},v{m}_{{lim},i}^{j}}\right)  + {S}_{d}\left( {v{m}_{{lim},i}^{j},{v}_{{lim},i}^{j}}\right) \) ,the feedrate curve shown in
			Fig.7(c)② achieved,and \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} = v{m}_{{lim},i}^{j} \) and \( \operatorname{Seg}\left( {i - 1}\right) \) .
			\( {v}_{e,i - 1}^{j} = {v}_{{lim},i}^{j} \) . Set moveFlag \( = \) remainStill and output moveFlag.
			Otherwise, go to Step 2.2c.
			(2.2c) The new segment \( i - 1 \) can only plan the deceleration
			process from \( {v}_{s,i - 1} \) to \( {v}_{{lim},i}^{j} \) ,as shown in Fig.7(c)③. If \( l{m}_{i}^{j} \geq \)
			\( {S}_{d}\left( {{v}_{s,i - 1},{v}_{{lim}.i}^{j}}\right) \) ,merged arc length satisfies deceleration,update
			\( \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} = \operatorname{Seg}\left( {i - 1}\right) .{v}_{s,i - 1}^{j} \) and \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{e,i - 1}^{j} = {v}_{{lim},i}^{j} \) .
			Set moveFlag \( = \) remainStill and output moveFlag; If \( l{m}_{i}^{j} < {S}_{d}\left( {v}_{s,i - 1}\right. \) ,
			\( \left. {v}_{\text{lim },i}^{j}\right) ,\operatorname{Seg}\left( {i - 1}\right) \) can be reset,updating \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{s,i - 1}^{j} = \operatorname{Seg}\left( {i - 1}\right) \) .
			\( {v}_{e,i - 1}^{j} = \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} = {v}_{s,i - 1} \) . Set moveFlag=moveBackward
			and output moveFlag.
	(3) Store the updated \( \operatorname{Seg}\left( i\right) \) into the look-ahead data buffer PlanBlock
			and output moveFlag.

---

<!-- Media -->

Fig. 8 illustrates a straightforward situation termed planDownUpCase. However, it's crucial to pre-plan segment \( i + 2 \) without saving the data,as the next window \( j + 1 \) may be planDownCase 1 due to inadequate arc length for complete deceleration in intermediate segments. In such instances, the strategy mirrors planDownCase1. Otherwise, the window can be shifted forward without additional processing of the current window.

Both initial acceleration and final deceleration segments require a variable feedrate process to achieve their feedrate extremes, starting and ending at zero feedrate as shown in Fig. 8. This may involve multiple segmented curves, each adhering to specific feedrate constraints. Employing the segmented strategy could potentially restrict the initial acceleration process to a very low feedrate, as shown by curve \( a \) in Fig.9,thereby diminishing operational efficiency. Alternatively, employing a step-by-step planning approach could yield a stepped velocity waveform, inducing motion instability. To address these issues, this paper proposes a Cross-Segment Merging Algorithm that consolidates ineffective feedrate limiting points during acceleration, updating subsequent segmentation strategies' starting points. Detailed explanation of this method follows in the subsequent section.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. 8 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING-->

<!-- Media -->

<!-- figureText: V A Window \( j \) Window \( j \) \( {l}_{i + 1}^{j} \) \( {lm} \) \( v{m}_{b}^{j} \) \( {V}_{\text{lim },i + 1} \) \( i + 1 \) \( {u}_{e,i} \) \( u \) \( {u}_{e,i} \) \( u \) (c) \( {v}_{\text{lim,}i - 1} \) V A \( {l}_{i}^{j} \) \( {v}_{{lim},i}^{j} \) \( {v}_{{lim},i + 1} \) \( {v}_{{lim},i} \) \( i - 1 \) \( i + 1 \) \( {u}_{s,i} \) \( {u}_{e,i} \) \( {u}_{s,i} \) (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_7.jpg?x=302&y=147&w=1189&h=354&r=0"/>

Fig. 7. Schematic diagram of planDownCase2: (a) two cases; (b) non-merge situation; (c) merge situation.

<!-- figureText: \( {v}_{{lim},i - 1} \) \( {v}_{\text{lim,}i + 1} \) \( i + 1 \) \( {u}_{e,i} \) \( u \) \( {\gamma }_{{lim},i} \) \( i - 1 \) \( i \) \( {u}_{s,i} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_7.jpg?x=327&y=564&w=370&h=309&r=0"/>

Fig. 8. Schematic diagram of planDownUpCase.

<!-- figureText: \( V \) Vlim,5 Vlim -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_7.jpg?x=318&y=920&w=387&h=291&r=0"/>

Fig. 9. The cross-segment merging algorithm.

<!-- Media -->

Feedrate limiting points are identified through the acceleration process referring to Fig. 9. For instance, when the initial acceleration segment starts from 0 and accelerates with known arc length \( {l}_{1} \) ,acceleration \( a{t}_{1} \) ,and jerk \( j{t}_{1} \) ,and using \( {V}_{\max } \) as the final feedrate for acceleration, if the velocities at the second to fifth points stay within their respective limits \( \left( {v}_{{lim},1}\right. \) to \( {v}_{\text{lim },4} \) ),as shown by curve \( b \) in Fig.9,these segments become candidates for cross-segment merging. However, if velocity exceeds \( {v}_{{lim},5} \) at the sixth point during continued acceleration, merging halts, and the total arc length becomes the sum of individual segments. The fifth point becomes the new starting point for the adaptive smoothing algorithm for further segmentation,as shown by curve \( c \) in Fig.9. This method can similarly merge the final deceleration segment by reversing numbering and treating it as the initial acceleration. The algorithm also adjusts for a look-ahead window, setting the initial feedrate \( {v}_{s} \) as the previous window’s calculated end feedrate. The algorithm flow is outlined in Algorithm 5.

In conclusion, utilizing the MVC with multi-constraints from the previous section and recording data for each discrete segment in the cache area, \( \operatorname{Seg}\left( i\right)  = \left\{  {{u}_{s,i},{u}_{e,i},{v}_{s,i},{v}_{e,i}}\right. \) , \( \left. {{v}_{{lim},i},A{t}_{i},J{t}_{i},{l}_{i}}\right\} \) ,where \( i = 1,2,\ldots ,N \) . Initially,the cross-segment merging algorithm consolidates start and end segments. Subsequently, a local window of size 3 , beginning from the new start point post-merging, is established. Curve segments within this window are categorized, and corresponding smoothing strategies are applied. The window moves incrementally until all segmented curves are processed, completing the dynamic adaptive feedrate smoothing process for all paths. For instance, as depicted in Fig.10, cross-segment merging consolidates start and end segments. Local window I starts from the merged start point and applies planUpCase1, saving updated Seg into PlanBlock. Subsequently, window II employs planDownCase2, merging the preceding segment to form the new window II. PlanDownCase1 is then applied, and data is saved. This process continues for windows III and IV until all curve segments are covered. All segments within the current look-ahead window are pre-planning and prepared for interpolation, while the look-ahead window moves forward and reads new path units. The overall algorithmic process is outlined in Algorithm 6.

<!-- Media -->

<!-- figureText: Merging III IV ... \( u \) \( C \) Cross-Segment Merging -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_7.jpg?x=911&y=564&w=746&h=344&r=0"/>

Fig. 10. Schematic diagram of segmented dynamic and adaptive look-ahead smoothing feedrate pre-processing based on the local dynamic window for the MVC.

<!-- Media -->

## C. Look-Ahead Feedrate Scheduling and Interpolation

Section III-B presents PlanBlock, containing pre-planning data for each segmented curve within the look-ahead window. Traditionally, bidirectional scanning [7], [8] is employed for ensuring global feedrate continuity. However, this method is not suitable for look-ahead feedrate scheduling. Appendix B explains this unsuitability and offers a detailed analysis of why the method proposed is particularly well-suited and inherently applicable to look-ahead feedrate scheduling.

Upon the output of the head result PlanBlock[1] \( = \operatorname{Seg}\left( i\right) \) from the look-ahead window for interpolation, the stored motion parameters can be directly utilized for the 7-segment S-shaped feedrate profile, as shown in Fig.3. A more detailed formula expression of the S-shaped feedrate profile is given by [7], which are not reiterated here. Furthermore, when

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. XU et al.: SEGMENTED DYNAMIC ADAPTIVE LOOK-AHEAD SMOOTHING FEEDRATE SCHEDULING 9-->

Algorithm 5 Cross-Segment Merging Algorithm

---

Input: The number of discrete segments \( N \) ; Maximum programmed feedrate \( {V}_{\max } \) ; The
maximum permissible feedrate \( {\left\{  {v}_{{lim},i}\right\}  }_{i = 1}^{N + 1} \) of \( 6\mathrm{R} \) robot’s end-effector; The maximum
permissible tangential acceleration \( {\left\{  A{t}_{{lim},i}\right\}  }_{i = 1}^{N + 1} \) and jerk \( {\left\{  J{t}_{{lim},i}\right\}  }_{i = 1}^{N + 1} \) ; The arc length
of each discrete segment \( {\left\{  {l}_{i}\right\}  }_{i = 1}^{N} \) ;
Output: The new \( {\left\{  {v}_{{lim},i}\right\}  }_{i = 1}^{N + 1},{\left\{  A{t}_{{lim},i}\right\}  }_{i = 1}^{N + 1},{\left\{  J{t}_{{lim},i}\right\}  }_{i = 1}^{N + 1} \) ,and \( {\left\{  {l}_{i}\right\}  }_{i = 1}^{N} \) after merging;
	(1) Let \( i = 1 \) ,the initial feedrate \( {v}_{s} = 0,{v}_{\text{merge }} = {v}_{s} \) ,and \( {L}_{i} = {l}_{1} \) ;
	(2) If \( {v}_{\text{merge }} > {v}_{\text{lim },i} \) ,go to Step 3; otherwise,go to Step 2;
(2.1) \( {at} = \operatorname{mean}\left( {\left\{  A{t}_{{lim},i}\right\}  }_{i = 1}^{N + 1}\right) \) and \( {jt} = \operatorname{mean}\left( {\left\{  J{t}_{{lim},i}\right\}  }_{i = 1}^{N + 1}\right) \) are used for the tangential
		acceleration and jerk. The displacement, \( s \) ,is calculated for the motion from
		\( {v}_{s} \) to \( {V}_{\max } \) . If \( {L}_{i} > s \) ,go to Step 3 . Otherwise,go to Step 2.2;
(2.2) If \( {V}_{\max } > {\left( at\right) }^{2}/{jt} + {v}_{s} \) ,it indicates a three-segment acceleration mode for
		transitioning from \( {v}_{s} \) to \( {V}_{\max } \) . Otherwise,go to Step 2.3. To solve for the
		feedrate, \( {v}_{\text{merge }} \) ,corresponding to \( {L}_{i} \) within the acceleration segment,the
		required time and displacement for each segment are calculated separately:

\[\left\{  \begin{array}{l} {t}_{1} = {t}_{3} = \frac{at}{jt} \\  {t}_{2} = \frac{{V}_{\max } - {v}_{s}}{at} - {t}_{1} \end{array}\right.  \tag{21}\]

\[\left\{  \begin{array}{l} {s}_{1} = {v}_{s}{t}_{1} + \frac{1}{6}{jt} \cdot  {t}_{1}^{3} \\  {s}_{2} = {s}_{1} + \left( {{v}_{s} + \frac{1}{2}{jt} \cdot  {t}_{1}^{2}}\right) {t}_{2} + \frac{1}{2}{at} \cdot  {t}_{2}^{2} \end{array}\right.  \tag{22}\]

		(2.2a) If \( {L}_{i} \leq  {s}_{1} \) ,a cubic equation in terms of \( {dt} \) can be established as follows:

\[\frac{1}{6}{jt} \cdot  d{t}^{3} + {v}_{s}{dt} - {L}_{i} = 0 \tag{23}\]

		solve \( {dt} \) ,and \( {v}_{\text{merge }} = {v}_{s} + \frac{1}{2}{jt} \cdot  d{t}^{2} \) can be obtained;
		(2.2b) If \( {L}_{i} \leq  {s}_{2} \) ,a quadratic equation can be established as follows:

\[\frac{1}{2}{at} \cdot  d{t}^{2} + \left( {{v}_{s} + \frac{1}{2}{jt} \cdot  {t}_{1}^{2}}\right) {dt} + {s}_{1} - {L}_{i} = 0 \tag{24}\]

		solve \( {dt} \) and \( {v}_{\text{merge }} = {v}_{s} + \frac{1}{2}{jt} \cdot  {t}_{1}^{2} + {at} \cdot  {dt} \) can be obtained;
		(2.2c) If none of the conditions are satisfied, a cubic equation can be established:

\[ - \frac{1}{6}{jt} \cdot  d{t}^{3} + \frac{1}{2}{at} \cdot  d{t}^{2}\]

\[ + \left( {{v}_{s} + \frac{1}{2}{jt} \cdot  {t}_{1}^{2} + {at} \cdot  {t}_{2}}\right) {dt} + {s}_{2} - {L}_{i} = 0 \tag{25}\]

		\( {dt} \) and \( {v}_{\text{merge }} = {v}_{s} + \frac{1}{2}{jt} \cdot  {t}_{1}^{2} + {at} \cdot  {t}_{2} + {at} \cdot  {dt} - \frac{1}{2}{jt} \cdot  d{t}^{2} \) can be obtained;
(2.3) If \( {V}_{\max } \leq  {\left( at\right) }^{2}/{jt} + {v}_{s} \) ,it indicates a two-segment acceleration mode for
		transitioning from \( {v}_{s} \) to \( {V}_{\max } \) . To solve for the \( {v}_{\text{merge }} \) corresponding to \( {L}_{i} \) within
		the acceleration segment, the required time and displacement are separately
		calculated:

\[{t}_{1} = {t}_{2} = \sqrt{\frac{{V}_{\max } - {v}_{s}}{jt}} \tag{26}\]

\[{s}_{h} = {v}_{s}{t}_{1} + \frac{1}{6}{jt} \cdot  {t}_{1}^{3} \tag{27}\]

---

(2.3a) If \( {L}_{i} \leq  {s}_{h} \) ,a cubic equation can be established as follows:

\[\frac{1}{6}{jt} \cdot  d{t}^{3} + {v}_{s}{dt} - {L}_{i} = 0 \tag{28}\]

solve \( {dt} \) and \( {v}_{\text{merge }} = {v}_{s} + \frac{1}{2}{jt} \cdot  d{t}^{2} \) can be obtained;

(2.3b) If \( {L}_{i} > {s}_{h} \) ,a cubic equation can be established as follows:

\[ - \frac{1}{6}{jt} \cdot  d{t}^{3} + \frac{1}{2}{jt} \cdot  {t}_{1}d{t}^{2}\]

\[ + \left( {{v}_{s} + \frac{1}{2}{jt} \cdot  {t}_{1}^{2}}\right) {dt} + {v}_{s}{t}_{1} + \frac{1}{6}{jt} \cdot  {t}_{1}^{3} - {L}_{i} = 0 \tag{29}\]

\( {dt} \) and \( {v}_{\text{merge }} = {v}_{s} + \frac{1}{2}\left( {{V}_{\max } - {v}_{s}}\right)  + {jt} \cdot  {t}_{1}{dt} - \frac{1}{2}{jt} \cdot  d{t}^{2} \) can be obtained. (2.4) Let \( {L}_{i} = \operatorname{sum}\left( {\left\{  {l}_{i}\right\}  }_{1}^{i + 1}\right) ,i = i + 1 \) and go to Step 2;

(3) Update \( {v}_{{lim},1} = {v}_{{lim},i},A{t}_{{lim},1} = \operatorname{mean}\left( {\left\{  A{t}_{{lim},i}\right\}  }_{1}^{i}\right) ,J{t}_{{lim},1} = \operatorname{mean}\left( {\left\{  J{t}_{{lim},i}\right\}  }_{1}^{i}\right) \) , and \( {l}_{i} = \operatorname{sum}\left( {\left\{  {l}_{i}\right\}  }_{1}^{i}\right) \) ; The local \( {\left\{  {v}_{{lim},i}\right\}  }_{2}^{i},{\left\{  A{t}_{{lim},i}\right\}  }_{2}^{i},{\left\{  J{t}_{{lim},i}\right\}  }_{2}^{i} \) ,and \( {\left\{  {l}_{i}\right\}  }_{2}^{i} \) are set to empty, and the updated data is stored in the buffer PlanBlock. calculating motion data based on interpolation frequency, rounding planning time to an integer multiple of the sampling period \( {T}_{s} \) is necessary. However,this may introduce time errors, causing velocity, acceleration, and jerk discontinuities at segment junctions, leading to unstable motion.

<!-- Media -->

Algorithm 6 Segmented Dynamic Adaptive Look-Ahead Smoothing Feedrate Pre-Processing Based on the Local Dynamic Window for the MVC

Input: The number of discrete segments \( N \) ; The data of each discrete

---

segment \( \operatorname{Seg}\left( i\right)  = \left\{  {{u}_{s,i},{u}_{e,i},{v}_{s,i},{v}_{e,i},{v}_{{lim},i},A{t}_{i},J{t}_{i},{l}_{i}}\right\}  ,i = 1,2,\ldots ,N \) .
Output: The buffer for look-ahead, PlanBlock.
	(1) The start and end segments are merged using the cross-segment
			merging algorithm,resulting in the new \( \operatorname{Seg}\left( 1\right) \) and \( \operatorname{Seg}\left( {N * }\right) \) ,where
			\( N * \) represents the number of new discrete segments after completing
			the merging of the start and end.
	(2) Let \( i = 2,j = 1 \) .
	(3) If \( i = N *  + 1 \) ,go to step 6 . Otherwise,go to Step 4.
	(4) The current window localWindow \( \left( j\right)  = \{ \operatorname{Seg}\left( {i - 1}\right) ,\operatorname{Seg}\left( i\right) ,\operatorname{Seg}(i + \)
			1)\},
			(a) If \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} \leq  \operatorname{Seg}\left( {i + 1}\right) .{v}_{{lim},i + 1}^{j} \leq  \operatorname{Seg}\left( i\right) .{v}_{{lim},i}^{j} \) ,go to
			planUpCase1.
			(b) If \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} \leq  \operatorname{Seg}\left( i\right) .{v}_{{lim},i}^{j} \leq  \operatorname{Seg}\left( {i + 1}\right) .{v}_{{lim},i + 1}^{j} \) ,go to
			planUpCase2.
			(c) If \( \operatorname{Seg}\left( i\right) .{v}_{{lim},i}^{j} > \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} \geq  \operatorname{Seg}\left( {i + 1}\right) .{v}_{{lim},i + 1}^{j} \) ,go to
			planDownCase1.
			(d) If \( \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} \geq  \operatorname{Seg}\left( i\right) .{v}_{{lim},i}^{j} \geq  \operatorname{Seg}\left( {i + 1}\right) .{v}_{{lim},i + 1}^{j} \) ,go to
			planDownCase2.
			(e) If \( \operatorname{Seg}\left( i\right) .{v}_{{lim},i}^{j} < \operatorname{Seg}\left( {i - 1}\right) .{v}_{{lim},i - 1}^{j} < \operatorname{Seg}\left( {i + 1}\right) .{v}_{{lim},i + 1}^{j} \) ,go to
			planDownUpCase.
	(5) Based on the status of the moveFlag, the movement of the local
			window is determined:
			(a) If moveFlag=moveForward,let \( i = i + 1 \) ,clear mergedSeg,and
			go to Step 3.
			(b) If moveFlag \( = \) remainStill,go to Step 3.
			(c) If moveFlag \( = \) moveBackward,let \( i = i - 1 \) ,and go to Step 3;
	(6) Output the look-ahead data buffer, PlanBlock.

---

<!-- figureText: \( {T}_{i} \) \( {S}_{i}\left( {t + {T}_{s}}\right) \) Seg \( \left( {i + 1}\right) \) \( {S}_{i + 1}\left( {t + {T}_{s} - {T}_{i}}\right) \) \( {S}_{i}\left( t\right) \) Seg(i) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_8.jpg?x=1074&y=1123&w=421&h=176&r=0"/>

Fig. 11. Cross-segment interpolation.

<!-- figureText: 50 440 480 x (mm) 25 y (mm) 0 -25 NURBS Curve -50 - - - Control Polygon 360 400 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_8.jpg?x=1048&y=1347&w=476&h=387&r=0"/>

Fig. 12. S-shaped NURBS curve.

<!-- Media -->

This paper employs a time interpolation method where accumulated time \( t \) increases incrementally within the total time \( T \) based on sampling period \( {T}_{s} \) . Interpolation data is computed using different formulas in various time segments, and issues with time rounding are addressed. Challenges arise with time accumulation across segments, illustrated in Fig.11. If \( t + {T}_{s} > {T}_{i} \) ,the interpolation point \( {S}_{i}\left( {t + {T}_{s}}\right) \) crosses into segment \( i + 1 \) . The next segment’s start time is then set as \( {t}_{\text{start }} = t + {T}_{s} - {T}_{i} \) ,and its corresponding interpolation point \( {S}_{i + 1}\left( {t + {T}_{s} - {T}_{i}}\right) \) is computed. This method avoids compensating for rounding errors within each feedrate profile segment separately. While rounding errors may occur at the trajectory's end if the total time not being an integer multiple of \( {T}_{s} \) ,they are typically negligible as the final segment usually ends with zero velocity, minimizing unstable motion.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. 10 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING-->

<!-- Media -->

<!-- figureText: 200 Feedrate (mm/s) 50 \( {v}_{\text{lim },1} = {87.83} \sim \) \( {v}_{{lim},7} = {58.54} \) \( {v}_{\text{lim. }2} = {87.83} \) 25 y (mm) \( {v}_{lim.6} = {78.59} \) \( {v}_{{lim},3} = {94.92} \) 0 \( {v}_{{lim},8} = {70.67} \) \( {v}_{{lim},5} = {78.59} \) -25 =70.67 \( {v}_{\text{lim }4} = {62.91} \) -50 360 400 440 480 x (mm) (b) Feedrate (mm/s) 160 120 80 Feed rate Chord-error limited 40 Joint velocity limited Normal acceleration limited Joint acceleration limited Normal jerk limited Joint jerk limited - Maximum programmed Feed rate 0.2 0.4 0.6 0.8 Normalization s (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_9.jpg?x=346&y=153&w=1090&h=419&r=0"/>

Fig. 13. The scheduled feedrate for S-shaped NURBS curve: (a) muti-limited velocities and the scheduled feedrate; (b) the scheduled feedrate on the curve.

<!-- Media -->

Following the motion parameter scheduling in the lookahead window head results,PlanBlock(1) \( = \operatorname{Seg}\left( i\right) \) ,arc length increment \( {\Delta s} \) calculated by accumulating time \( t \) based on \( {T}_{s} \) . The next cycle’s curve parameter \( {u}_{i + 1} \) is computed using the second-order Taylor expansion formula based on current curve parameters \( {u}_{i} \) and \( \Delta {s}_{i} \) . Finally,the position for the next cycle is established as \( {\mathbf{P}}_{i + 1} = \mathbf{C}\left( {u}_{i + 1}\right) \) ,and \( \mathbf{C}\left( u\right) \) represents the spline curve function. Extensive testing indicates that the second-order Taylor expansion formula fails when the first-order tangent vector is zero. In such cases, if the second-order tangent vector is nonzero, it is used directly. If both first and second-order tangent vectors are zero, the third-order tangent vector is used, which is never zero for higher-order curves. The formula is as follows:

\[{u}_{i + 1} = \left\{  \begin{array}{l} {u}_{i} + \frac{1}{\begin{Vmatrix}{\mathbf{C}}^{\prime }\left( {u}_{i}\right) \end{Vmatrix}}\Delta {s}_{i} - \frac{{\mathbf{C}}^{\prime }\left( {u}_{i}\right)  \cdot  {\mathbf{C}}^{\prime \prime }\left( {u}_{i}\right) }{2{\begin{Vmatrix}{\mathbf{C}}^{\prime }\left( {u}_{i}\right) \end{Vmatrix}}^{4}}\Delta {s}_{i}^{2}, \\  \;\text{ if }\begin{Vmatrix}{{\mathbf{C}}^{\prime }\left( {u}_{i}\right) }\end{Vmatrix} \neq  0 \\  {u}_{i} + \sqrt{\frac{{2\Delta }{s}_{i}}{\begin{Vmatrix}{\mathbf{C}}^{\prime \prime }\left( {u}_{i}\right) \end{Vmatrix}}}, \\  \;\text{ else if }\begin{Vmatrix}{{\mathbf{C}}^{\prime \prime }\left( {u}_{i}\right) }\end{Vmatrix} \neq  0 \\  {u}_{i} + \sqrt[3]{\frac{{6\Delta }{s}_{i}}{\begin{Vmatrix}{\mathbf{C}}^{\prime \prime }\left( {u}_{i}\right) \end{Vmatrix}}}, \\  \;\text{ if }\begin{Vmatrix}{{\mathbf{C}}^{\prime \prime }\left( {u}_{i}\right) }\end{Vmatrix} = 0 \end{array}\right.  \tag{30}\]

## IV. Simulation and Experimental Results

## A. Simulation Results

To validate the feasibility of the proposed robot NURBS trajectory feedrate scheduling and interpolation method under comprehensive multi-constraints, all algorithms were implemented on a personal computer with an Intel Core i7-11800H CPU of \( {2.30}\mathrm{{GHz}} \) and \( {64}\mathrm{{GB}} \) RAM,utilizing MATLAB R2021a. A 6R robot simulation model was established in MATLAB, employing a S-shaped NURBS curve for simulation, as depicted in Fig.12. The joint constraints for the robot were defined as follows: \( {\dot{q}}_{\max } = \lbrack {0.2},{0.2} \) , \( {0.3},{0.3},{0.4},{0.4}{\rbrack }^{\mathrm{T}}\mathrm{{rad}}/\mathrm{s},{\ddot{q}}_{\max } = \lbrack {0.5},{0.5},{0.75},{0.75},{1.0} \) , 1.0] \( {}^{\mathrm{T}}\mathrm{{rad}}/{\mathrm{s}}^{2},\ddot{\overline{\mathbf{q}}} = {\left\lbrack  {6.0},{6.0},{8.5},{8.5},{10.0},{10.0}\right\rbrack  }^{\mathrm{T}}\mathrm{{rad}}/{\mathrm{s}}^{3} \) . The maximum programmed feedrate \( {V}_{\max } = {120}\mathrm{\;{mm}}/\mathrm{s} \) , with specified maximum tangential acceleration and jerk as \( {1000}\mathrm{\;{mm}}/{\mathrm{s}}^{2} \) and \( {2000}\mathrm{\;{mm}}/{\mathrm{s}}^{3} \) ,respectively.

Fig. 13 illustrates the feedrate profile obtained using the proposed method, with the normalized arc length parameter \( {s}_{\text{Nor }} \) . The feedrate consistently remains below the MVC and maintains stability, particularly within the acceleration segments. Although not optimized for time, this approach ensures smooth task execution at the \( 6\mathrm{R} \) robot’s end-effector, enhancing overall work quality. This paper contrasts its approach with Du's method [7], originally designed for three-axis machine tools but adapted for experimental comparison with a \( 6\mathrm{R} \) robot. Employing identical test parameters, comparative tests were conducted for this paper's method, Du's method, and Du's method with only the constraints proposed in this paper. Fig. 14 shows feedrate and tangential acceleration/jerk profiles obtained using the three methods, while Fig. 15 compares kinematic parameters of the \( 6\mathrm{R} \) robot’s joints,including velocity,acceleration, and jerk. Du's method offers temporal advantages but neglects joint constraints, leading to significant violations in acceleration and jerk limits, reaching up to 162.08%, \( {334.96}\% \) ,and 223.72% of the respective constraint values. Applying only the constraints proposed in this paper maintains strict joint limitations but introduces noticeable acceleration and jerk oscillations, compromising end-effector stability. The proposed method strikes a balance between the two approaches, ensuring adherence to set constraints for all joint parameters while maintaining smooth trajectory feedrate variations, demonstrating its effectiveness.

The S-shaped NURBS curve undergoes only positional changes without any alterations in orientation. To assess the general applicability of the proposed method, further testing is conducted using a more complicated open pocket path that involves both positional and orientational variations, as illustrated in Fig.16. Simultaneously, modifications are made to the \( 6\mathrm{R} \) robot’s joint constraints: \( {\dot{q}}_{\max } = \lbrack {0.32},{0.52} \) , \( {0.39},{0.41},{0.62},{0.56}{\rbrack }^{\mathrm{T}}\mathrm{{rad}}/\mathrm{s},{\ddot{q}}_{\max } = \lbrack {0.715},{0.715},{0.715}, \) \( {0.97},{0.90},{1.38}{\rbrack }^{\mathrm{T}}\mathrm{{rad}}/{\mathrm{s}}^{2},{\dddot{q}}_{\max } = \lbrack {6.28},{6.28},{6.28},{6.28} \) , \( {6.28},{6.28}{\rbrack }^{\mathrm{T}}\mathrm{{rad}}/{\mathrm{s}}^{3} \) ,and the maximum programmed feedrate \( {V}_{\max } = {80}\mathrm{\;{mm}}/\mathrm{s} \) .

The feedrate profile from the proposed method, as shown in Fig.17, consistently maintains feedrates below the MVC, ensuring smoothness. Employing the same comparative approach, Fig. 18 illustrates the various Cartesian trajectory kinema-tic parameters, while Fig. 19 presents the actual joint kinematic parameters of the \( 6\mathrm{R} \) robot. Du’s method allows actual joint velocities/accelerations/jerks to exceed constraints, reaching up to \( {193.28}\% ,{235.46}\% \) ,and 268.78%,respectively. In contrast, the proposed method rigorously maintains joint parameters within limits while ensuring smooth feedrate variations, making it suitable for scenarios with simultaneous position and orientation changes.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. XU et al.: SEGMENTED DYNAMIC ADAPTIVE LOOK-AHEAD SMOOTHING FEEDRATE SCHEDULING 11-->

<!-- Media -->

<!-- figureText: Proposed Du's method without joint constraints Du's method with joint constraints 0.5 0.6 0.7 0.8 0.9 0.5 0.6 0.7 0.8 0.9 0.5 0.6 0.7 0.8 0.9 Normalization s 100 \( \left. {\mathrm{{nm}}/{\mathrm{s}}^{3}}\right) \) Acceleration \( \left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) \) Feedrate ( 50 0 0.1 0.2 0.3 0.4 500 0 0.1 0.2 0.3 0.4 2000 0 -2000 0.1 0.2 0.3 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_10.jpg?x=441&y=161&w=919&h=642&r=0"/>

Fig. 14. Comparison of feedrate, tangential acceleration, and tangential jerk profiles scheduled by different methods for the S-shaped NURBS curve.

<!-- figureText: j1 (rad/s) 0.2 1 (rad/s -2 \( \mathrm{j}1\left( {\mathrm{{rad}}/{\mathrm{s}}^{3}}\right) \) 10 -10 0.6 0.8 0.2 0.4 0.6 0.8 j2 (rad/s3 -10 0.6 0.8 0.2 0.4 0.6 0.8 0.6 0.8 \( \mathrm{j}3\left( {\mathrm{{rad}}/{\mathrm{s}}^{3}}\right) \) 0.2 0.4 0.6 0.8 j4 (rad/s3) 10 0 0.6 0.2 0.4 0.6 j5 (rad/s3) 0 0.6 0.8 0.2 0.4 0.6 0.8 \( 6\left( {\mathrm{{rad}}/{\mathrm{s}}^{3}}\right) \) -10 0.5 0.2 0.4 0.6 0.8 Normalization s Normalization s (b) (c) -0.2 0.2 0.4 0.6 0.8 0.2 0.4 j2 (rad/s) 0.2 -0.2 2 (rad/s2 -1 0.2 0.4 0.6 0.8 0.2 0.4 j3 (rad/s) 0.2 j3 (rad/s2) -0.2 0.2 0.8 0.2 0.4 j4 (rad/s) 0.2 4 (rad/s -0.2 0 0.2 0.4 0.6 0.8 0 j5 (rad/s) 0 \( 5{\text{(rad/s}}^{2} \) 0 0.2 0.4 0.6 0.8 0.2 0.4 j6 (rad/s) 0.5 0 (rad/s) \( {}^{2} \) -0.5 -2 0.2 0.4 0.6 0.8 Normalization s (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_10.jpg?x=242&y=875&w=1315&h=712&r=0"/>

Fig. 15. Comparison of 6R robot's joint trajectories scheduled by different methods for the S-shaped NURBS curve. (a) velocity; (b) acceleration; (c) jerk.

<!-- figureText: z (mm) 160 450 400 350 x (mm) 140 80 40 y (mm) 20 300 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_10.jpg?x=211&y=1715&w=593&h=269&r=0"/>

Fig. 16. The open pocket path.

<!-- Media -->

## B. Experimental Results

This section validates the proposed method using a collaborative robotic arm, as shown in the Fig.20. The experiments are conducted on a Dobot CR7 collaborative robot controlled by a Beckhoff C6930-0060 controller, featuring an Intel Core i7-6700TE 2.4GHz CPU and 8 GB of memory. The controller communicates with the robot motor drives via the EtherCAT protocol at a \( {500}\mathrm{\;{Hz}} \) sampling frequency,with a sampling period of \( 2\mathrm{\;{ms}} \) . Unlike simulation, experiments utilize different trajectory to showcase the algorithm's versatility.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. 12 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING-->

<!-- Media -->

<!-- figureText: 140 Feedrate (mm/s) \( {v}_{lim.6} = {80} \) \( {v}_{\text{lim },6} = {41.39} \) =45.52 z (mm) 160 \( {v}_{{lim},5} = {80} \) 140 \( {v}_{\text{lim },3} = {65.92} \) 450 \( - {v}_{{lim},1} = {63.72} \) 400 40 350 y (mm) 20 x (mm) 300 (b) 120 Feedrate (mm/s) 100 60 40 Chord-error limited 20 Joint velocity limited Normal acceleration limited Joint acceleration limited Normal jerk limited 0 0.2 0.4 0.6 0.8 Normalization s (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_11.jpg?x=298&y=154&w=1198&h=411&r=0"/>

Fig. 17. The scheduled feedrate for the open pocket path: (a) muti-limited velocities and the scheduled feedrate; (b) the scheduled feedrate on the path.

<!-- figureText: Proposed Du's method without joint constraints Du's method with joint constraints 0.5 0.6 0.7 0.8 0.9 0.6 0.7 0.9 0.5 0.6 0.7 0.8 0.9 Normalization s \( \left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \; \) Acceleration \( \left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) \) Feedrate \( (\mathrm{{mn}} \) 40 0.1 0.2 0.3 0.4 400 -400 0 0.2 0.3 0.4 2000 0 -2000 0 0.1 0.2 0.3 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_11.jpg?x=464&y=625&w=870&h=611&r=0"/>

Fig. 18. Comparison of feedrate, tangential acceleration, and tangential jerk profiles scheduled by different methods for the open pocket path.

<!-- figureText: j1 (rad/s) 0.32 j1 (rad/s 0.6 0.4 0.8 0 0.2 0.4 0.6 0.8 \( \mathrm{j}3\left( {\mathrm{{rad}}/{\mathrm{s}}^{3}}\right) \) -10 0.6 0.8 0 0.2 0.4 0.6 0.8 \( \mathrm{j}4\left( {\mathrm{{rad}}/{\mathrm{s}}^{3}}\right) \) 0.2 0.8 0.8 j5 (rad/s \( {}^{3} \) ) 0 0.2 0.4 0.6 0.8 j6 (rad/s \( {}^{3} \) ) 0.8 1 0 0.2 0.4 0.6 0.8 Normalization s Normalization s (b) (c) 0.2 0.4 0.6 0.8 j2 (rad/s) 0 \( \mathrm{j}2\left( {\mathrm{{rad}}/{\mathrm{s}}^{2}}\right) \) -0.52 0.2 0.4 0.6 0.8 0.2 0.4 j3 (rad/s) 0.39 j3 (rad/s2) 0 -0.39 0.2 0.4 0.6 0.8 0.2 0.4 j4 (rad/s) 0.41 -0.41 0 0.2 0.2 j5 (rad/s) -0.62 j5 (rad/s \( {}^{2} \) ) -2 0.2 0.4 0.2 0.4 0.6 0.8 j6 (rad/s) 0 j6 (rad/s2) -0.56 0.2 0.4 0.6 0.8 1 0.2 0.4 Normalization s (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_11.jpg?x=312&y=1301&w=1174&h=644&r=0"/>

Fig. 19. Comparison of 6R robot's joint trajectories scheduled by different methods for the open pocket path: (a) velocity; (b) acceleration; (c) jerk.

<!-- Media -->

The experiment utilizes a butterfly-shaped NURBS curve as the test curve, as depicted in Fig.21(a). The robot joint constraints are modified as follows: \( {\dot{q}}_{\max } = \lbrack {0.15},{0.15} \) , \( {0.15},{0.15},{0.15},{0.15}{\rbrack }^{\mathrm{T}} \) rad/s, \( {\ddot{q}}_{\max } = \lbrack {0.20},{0.20},{0.20} \) , \( {0.20},{0.20},{0.20}{\rbrack }^{\mathrm{T}}\mathrm{{rad}}/{\mathrm{s}}^{2},{\ddot{\mathbf{q}}}_{\max } = \lbrack {6.28},{6.28},{6.28},{6.28} \) , \( {6.28},{6.28}{\rbrack }^{\mathrm{T}}\mathrm{{rad}}/{\mathrm{s}}^{3} \) ,and the maximum programmed feedrate \( {V}_{\max } = {40}\mathrm{\;{mm}}/\mathrm{s} \) ,with chord error constraint \( \delta  = 1\mathrm{e} - 3\mathrm{\;{mm}} \) . As shown in Fig.21(b), the actual value in terms of chord error is very small, indicating that the precision of the actual trajectory is high. Meanwhile, to highlight the advantages of the method proposed, a comparison has been made with Liu's method [26].

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. XU et al.: SEGMENTED DYNAMIC ADAPTIVE LOOK-AHEAD SMOOTHING FEEDRATE SCHEDULING 13-->

<!-- Media -->

<!-- figureText: Dobot CR7 Programmed Interface Real-Time Controller collaborative robot -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_12.jpg?x=255&y=146&w=506&h=415&r=0"/>

Fig. 20. The setup of the collaborative robot.

<!-- Media -->

Fig. 22 shows the feedrate curves: the solid blue line uses the proposed method and the dashed red line uses Liu's method. It is observed that the proposed method maintains high velocity in low-curvature areas and slows in high-curvature zones, while maintaining constant velocity across multiple regions, thus preserving motion stability. Liu's method, with its frequent feedrate changes, achieves stability only at maximum feedrate, causing oscillations elsewhere. The analytic constraint method proposed is nearly identical to the MVC established by the modeling optimization constraint method, providing superior real-time benefits. Fig. 23 confirms that joint motion parameters adhere to their constraints, underscoring the proposed method's effectiveness. Table I compares Liu's method and ours in terms of the planning and machining time (the time for the robot to track along the writing task path), and chord error at different number of discrete points. The proposed method, based on a sliding look-ahead window,achieves a time complexity of \( O\left( N\right) \) , whereas Liu's method, which uses the solution of the first sub-LP problem as the boundary constraint for the second sub-LP problem,exhibits a time complexity of \( O\left( {N}^{2}\right) \) ,where \( N \) is the number of discrete points. The total runtime indicates that Liu's method is suitable only for offline use, contrasts with our method which, based on the look-ahead window-for example, when there are 3063 discrete points, the total program runtime is \( {0.4638}\mathrm{\;s} \) with 54 segments-averages 0.008589s per segment, which is only 4-5 times the interpolation period, proves better for real-time application. As depicted in Fig.1, these calculations are performed in a non-real-time thread. The overall complexity of \( \mathrm{O}\left( N\right) \) does not impact the performance of the real-time thread. The planning result PlanBlock[1] obtained after each movement of the lookahead window, are stored in the circular queue buffer. The real-time thread retrieves these results from the buffer for interpolation as long as it's not empty. This method effectively separates the computation from interpolation, allowing both threads to operate independently, suitable for long-distance complex trajectories. Our method ensures smoother, more stable motion, reducing both maximum and average chord errors. Although the machining time is longer than that of Liu's method, the calculation time maintains consistent per segment, independent of path length or point count in the same path. Moreover, the motion is smoother and more precise, emphasizing stability and reliability in practical applications, balancing efficiency and stability.

This section further validates the proposed method by conducting slot milling experiments using S-shaped NURBS curve and end milling experiments using butterfly-shaped NURBS curve on wood material. It also includes comparative experiments using Liu's method, with a focus on analyzing the actual machining quality of different methods. The experiments utilized a self-developed harmonic drive reducer \( 6\mathrm{R} \) robot with a maximum end-effector load of \( {35}\mathrm{\;{kg}} \) , as depicted in the Fig.24. The controller hardware platform employed a SpeedGoat RCP controller with a sampling period of \( 1\mathrm{\;{ms}} \) . The upper computer control system was autonomously developed using MATLAB Simulink Real-Time functionality, with MATLAB version 2022b, running on a computer with a i5 CPU and \( {32}\mathrm{{GB}} \) of \( {3200}\mathrm{{MHz}} \) memory. Table II presents the feedrate scheduling parameters, machining process parameters, and joint constraint parameters. Conservative planning parameters were selected to avoid medium to high-speed impacts due to the substantial weight of industrial robots.

Fig. 25 shows the feedrate curves: the solid blue line uses the proposed method and the dashed red line uses Liu's method. Fig.25(a) derives from the slot milling using S-shaped NURBS curves, and Fig.25(b) from the end milling with Butterfly NURBS curves. Fig.26 shows the actual interpolation feedback of joint velocity, acceleration, and jerk curves obtained in slot milling experiments using the proposed method. Similarly, Fig.27 illustrates these parameters from end milling experiments using the proposed method. Analysis reveals the proposed method maintains consistent or smoothly changing trajectory feedrates across most regions while adhering to joint kinematic constraints, whereas Liu's method leads to excessive acceleration and deceleration, causing few stable velocities and widespread oscillations. The machined workpieces are illustrated in Fig.28 and 29. Fig.28(a) reveals that the proposed method yields a slot machining surface with no noticeable tool marks or burrs, ensuring a smooth surface quality throughout, regardless of sharp turns or straight sections with small curvature. In contrast, Fig.28(b) shows Liu's method results in a machined surface with apparent striations, vertical lines, and numerous burrs on the bottom surface, caused by uneven feedrates and oscillations. Similarly, Fig.29(a) demonstrates that the proposed method produces clear and smooth Butterfly NURBS end milling curves, with complete and smooth curve details in areas of high local curvature and noticeable deceleration, indicating good overall curve quality. Conversely, Fig.29(b) shows that Liu's method, although clear, has rough details with many burrs due to frequent feedrate changes. The machining experiments further validate the effectiveness of the proposed method.

Additionally, both methods used identical discrete points and hardware conditions. For instance, in slot milling with S-shaped NURBS, Liu's method required 14.8821s for Sub-LP1 modeling, 0.56s for optimization, 331.8605s for Sub-LP2 modeling,and 2.22s for optimization,resulting in \( {10.278}\mathrm{\;s} \) machining time. The total runtime for the proposed method was \( {0.2969}\mathrm{\;s} \) ,with an actual machining time of 10.313s. It is evident that Liu's method achieves shorter machining times due to a feedrate closely aligned with the MVC and frequent acceleration and deceleration, which, while yielding a time- balances efficiency and quality, and its computation time does optimal feedrate curve, results in significant speed oscillations and lower machining quality. Conversely, the proposed method not increase with the number of discrete points in the same path, making it suitable for a look-ahead window and highly efficient in algorithmic computation, thus better suited for real-time interpolation.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. 14 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING-->

<!-- Media -->

<!-- figureText: 50 1.2 Chord error (mm) 0.8 0.6 0.4 0.2 0 0.2 0.4 0.6 0.8 Normalization s (b) Control Polygon y (mm) -50 -100 450 500 550 600 x (mm) (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_13.jpg?x=430&y=146&w=943&h=388&r=0"/>

Fig. 21. The butterfly-shaped NURBS curve: (a) the geometric model;(b) the actual chord error.

<!-- figureText: Feedrate (mm/s) 40 Feedrate (mm/s) 40 20 30 \( {v}_{{lim},9} = {40} \) y(mm) -20 20 -40 \( {v}_{\text{lim },{13},{15}} = {40} \) -60 10 -80 -100 450 500 550 600 x (mm) (b) 30 Proposed Joint acceleration Normal acceleration limited Liu's method of mal jerk limited Maximum programmed 0.4 0.6 Normalization s (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_13.jpg?x=340&y=595&w=1114&h=395&r=0"/>

Fig. 22. The scheduled feedrate for the butterfly-shaped NURBS curve: (a) muti-limited velocities, the comparison of feedrate profiles scheduled by the proposed method and Liu's method; (b) the feedrate scheduled by the proposed method on the curve.

<!-- figureText: 13 j1 j2 15 1.5 1 Joint jerk (rad/s \( {}^{3} \) ) 0.5 0 -0.5 -1 -1.5 0.4 0.6 0.8 1 0.2 0.4 0.6 0.8 Normalization s Normalization s (c) 0.15 Joint acceleration \( \left( {\mathrm{{rad}}/{\mathrm{s}}^{2}}\right) \) 0.2 0.1 0 0.1 -0.2 Joint velocity (rad/s) 0.1 0.05 0 -0.05 -0.1 -0.15 0.2 0.4 0.6 0.8 0 0.2 Normalization s (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_13.jpg?x=184&y=1066&w=1427&h=412&r=0"/>

Fig. 23. The 6R robot's joint trajectories scheduled by the proposed method: (a) velocity; (b) acceleration; (c) jerk.

TABLE I

PERFORMANCE COMPARISON OF NURBS CURVE UNDER LIU'S METHOD AND THE PROPOSED

<table><tr><td colspan="2" rowspan="2">Evaluation Metrics</td><td colspan="3">Discretized at a step length of \( {0.25}\mathrm{\;{mm}} \) ,resulting in 3063 discrete points</td><td colspan="3">Discretized at a step length of \( {0.12}\mathrm{\;{mm}} \) ,resulting in 6381 discrete points</td></tr><tr><td>Liu's method</td><td>Sub-LP1:</td><td>Ours</td><td/><td>Liu's method Sub-LP1:</td><td>Ours</td></tr><tr><td rowspan="5">Time (s)</td><td rowspan="4">Total Runtime</td><td rowspan="2">Modeling time</td><td>Sub-LP2:</td><td rowspan="3">\\.</td><td rowspan="2">Modeling time</td><td>Sub-LP2:</td><td rowspan="3">\\</td></tr><tr><td>592.3782</td><td>2750.2098</td></tr><tr><td>Optimization solution time</td><td>Sub-LP1: 0.89 Sub-LP2: 7.50</td><td>Optimization solution time</td><td>Sub-LP1: 1.45 Sub-LP2: 8.55</td></tr><tr><td>Total time</td><td>622.7305</td><td>0.4638</td><td>Total time</td><td>2816.1082</td><td>0.4854</td></tr><tr><td>21.9623 55.8984 Machining</td><td colspan="2">26.0170</td><td>32.0380</td><td colspan="2">26.0430</td><td>32.1020</td></tr><tr><td rowspan="2">Chord error (mm)</td><td>Mean</td><td colspan="2">\( {1.8618}\mathrm{e} - 5 \)</td><td>\( {1.1467}\mathrm{e} - 5 \)</td><td colspan="2">1.8534e-5</td><td>\( {1.3101}\mathrm{e} - 5 \)</td></tr><tr><td>Maximum</td><td colspan="2">1.7968e-4</td><td>\( {1.2111}\mathrm{e} - 4 \)</td><td colspan="2">1.2377e-4</td><td>8.1483e-5</td></tr></table>

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. XU et al.: SEGMENTED DYNAMIC ADAPTIVE LOOK-AHEAD SMOOTHING FEEDRATE SCHEDULING 15-->

<!-- figureText: Machining processing of the Robot Workpiece S-shaped NURBS curve Machining processing of the butterfly NURBS curve Spindle -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_14.jpg?x=362&y=148&w=1071&h=530&r=0"/>

Fig. 24. The setup of the robot milling experiment and the machining process.

TABLE II

Machining Parameters

<!-- figureText: Parameters S-shaped curve Butterfly curve 1 mm 15000 rpm \( {V}_{\max } : {30}\mathrm{\;{mm}}/\mathrm{s} \) \( A{t}_{\max } : {50}\mathrm{\;{mm}}/{\mathrm{s}}^{2}J{t}_{\max } : {500}\mathrm{\;{mm}}/{\mathrm{s}}^{3}\delta  : {0.01}\mathrm{\;{mm}}{T}_{s} : 1\mathrm{\;{ms}} \) \( \left\lbrack  {{0.1},{0.1},{0.1},{0.1},{0.1},{0.1}}\right\rbrack  \mathrm{{rad}}/\mathrm{s} \) \( \left\lbrack  {{0.1},{0.1},{0.1},{0.1},{0.1},{0.1}}\right\rbrack  \mathrm{{rad}}/{\mathrm{s}}^{2} \) \( \left\lbrack  {{2.0},{2.0},{2.0},{2.0},{2.0},{2.0}}\right\rbrack  \mathrm{{rad}}/{\mathrm{s}}^{3} \) 40 Feedrate (mm/s) 30 20 Joint acceleration rmal acceleration 10 Proposed limited hited Liu's method - Joint jerk 1 hited Normal jerk limited Joint velocity limited aximum programmec feedrate 0.2 0.4 0.8 Normalization s (b) Depth of cut in a single pass 2 mm Spindle speed Feedrate scheduling parame- \( {V}_{\max } : {50}\mathrm{\;{mm}}/\mathrm{s} \) ters Joint velocity constraint Joint acceleration constraint Joint jerk constraint 90 80 70 Feedrate (mm/s) 50 30 Joint acceleration Normal acceleration Proposed limited limited Liu's method - Joint jerk limited Normal jerk limited 10 Joint velocity limited Chord-error limited - Maximum programme feedrate 0.2 0.4 0.6 0.8 Normalization s (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_14.jpg?x=257&y=811&w=1284&h=722&r=0"/>

Fig. 25. The comparison of feedrate profiles scheduled by the proposed method and Liu's method: (a) the S-shaped NURBS curve;(b) the butterfly-shaped NURBS curve.

<!-- figureText: 0.1 Joint acceleration (rad \( /{\mathrm{s}}^{2} \) ) 0.1 0.5 Joint jerk (rad/s \( {}^{3} \) ) 0.25 0 0.25 -0.5 0.4 0.6 0.8 1 0.2 0.4 0.6 0.8 Normalization s Normalization s (c) 0.05 0 -0.05 -0.1 Joint velocity (rad/s) 0.05 -0.05 -0.1 0.2 0.4 0.6 0.8 0 0.2 Normalization s (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_14.jpg?x=155&y=1594&w=1480&h=421&r=0"/>

Fig. 26. The 6R robot's joint trajectories scheduled by the proposed method for the S-shaped NURBS curve: (a) velocity; (b) acceleration; (c) jerk.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING-->

<!-- figureText: 0.1 Joint acceleration \( \left( {\mathrm{{rad}}/{\mathrm{s}}^{2}}\right) \) 0.1 j2 0.6 0.4 Joint jerk (rad/s \( {}^{3} \) ) 0.2 0 -0.2 -0.4 -0.6 0.4 0.6 0.8 0.2 0.4 0.6 0.8 1 Normalization s Normalization s (b) (c) 0.05 0 -0.05 -0.1 Joint velocity (rad/s) 0.05 -0.05 -0.1 0 0.2 0.4 0.6 0.8 1 0 0.2 Normalization s (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_15.jpg?x=147&y=150&w=1503&h=433&r=0"/>

Fig. 27. The 6R robot's joint trajectories scheduled by the proposed method for the butterfly-shaped NURBS curve: (a) velocity; (b) acceleration; (c) jerk.

<!-- figureText: (a) ⑤ (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_15.jpg?x=140&y=630&w=1516&h=492&r=0"/>

Fig. 28. The comparison of S-shaped curve slot milling workpieces machined using the proposed method and Liu's method: (a) the proposed method; (b) Liu's method.

<!-- figureText: (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_15.jpg?x=149&y=1189&w=1501&h=359&r=0"/>

Fig. 29. The comparison of butterfly curve end milling workpieces machined using the proposed method and Liu's method: (a) the proposed method; (b) Liu's method.

<!-- Media -->

## V. CONCLUSION

This paper presents a new method for adaptive smooth lookahead feedrate scheduling for \( 6\mathrm{R} \) robot splines under comprehensive multi-constraints, integrating NURBS curve trajectory geometry and joint velocity/acceleration/jerk constraints. The goal is to balance operational efficiency, trajectory smoothness, and quality. The robot trajectory's MVC is segmented and scanned using a local dynamic window approach, followed by adaptive smoothing preprocessing to mitigate step-like feedrate waveforms and frequent acceleration starts and stops. Parameters such as acceleration and deceleration initiation points, along with corresponding feedrate and arc length parameters, are obtained during preprocessing, enabling direct feedrate scheduling and real-time interpolation. Simulation and experimentation on a \( 6\mathrm{R} \) robot validate the method, indicating that while it may not be time-optimal, it enables the reverse computation of motion parameters based on joint constraints, eliminating the need for iterative trial-and-error processes. Moreover, the trajectory feedrate remains constant or smoothly varies in most regions, ensuring stability in end-effector motion, which is advantageous for practical load-bearing tasks. This method offers theoretical support and experimental validation for achieving smooth end-effector trajectory planning in \( 6\mathrm{R} \) robots,aiming to balance trajectory smoothness and operational efficiency.

## Appendix A

When a feedrate limit is exceeded within segment \( i \) , as depicted by curve \( a \) in Fig.30,it suggests that the tangential acceleration \( A{t}_{i}^{j} \) within the segment is excessively high. This paper utilizes a dynamic search method to adjust the tangential acceleration and update the feedrate within the segment. The upper boundary of the search interval can be set at the current segment’s tangential acceleration \( A{t}_{i}^{j} \) . To enhance the efficiency of the search, it is assumed that a two-stage acceleration curve is employed within the segment, as shown by curve \( b \) . The tangential acceleration corresponding to curve \( b \) can be calculated as follows:

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. XU et al.: SEGMENTED DYNAMIC ADAPTIVE LOOK-AHEAD SMOOTHING FEEDRATE SCHEDULING 17-->

\[A{t}_{i,b}^{j} = \sqrt{J{t}_{i}^{j} \cdot  \left( {v{m}_{{lim},i}^{j} - {v}_{{lim},i - 1}^{j}}\right) } \tag{A.1}\]

<!-- Media -->

<!-- figureText: V A \( l{m}_{i}^{j} \) \( v{m}_{lin}^{j} \) \( {v}_{{lim},i + 1}^{j} \) \( i + 1 \) \( {u}_{e,i} \) \( u \) \( i - 1 \) \( {u}_{s,i} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_16.jpg?x=312&y=147&w=401&h=291&r=0"/>

Fig. 30. The merge situation of planUpCase2.

<!-- Media -->

When a feedrate limit is exceeded, the search interval is set between \( \left\lbrack  {A{t}_{i,b}^{j},A{t}_{i}^{j}}\right\rbrack \) . The tangential acceleration \( A{t}_{i}^{j} = {0.95A}{t}_{i}^{j} \) . This adjustment continues iteratively until the feedrate \( V \) ,under the revised tangential acceleration,complies with the MVC within the segment. The search concludes successfully,as illustrated by curve \( c \) ; otherwise,the iteration continues until \( A{t}_{i}^{j} = A{t}_{i,b}^{j} \) .

## Appendix B

Traditional bidirectional scanning [7], [8] is not suitable for look-ahead feedrate scheduling as feedrate unreachability occurs mainly during acceleration or deceleration, with deceleration being the time reversal of acceleration. Thus, bidirectional scanning feasibility hinges on the nature of the acceleration segment. For the 7-segment S-shaped feedrate scheduling,considering tangential acceleration and jerk \( {At}/{Jt} \) , and arc length \( s \) ,the feedrate function \( f\left( {{v}_{s},{v}_{e}}\right)  = 0 \) can be expressed in two forms:

\[f\left( {{v}_{s},{v}_{e}}\right) \]

\[ = \left\{  \begin{array}{l} {v}_{e}^{3} + {v}_{s}{v}_{e}^{2} - {v}_{s}^{2}{v}_{e} - {v}_{s}^{3} - {Jt} \cdot  {s}^{2},{v}_{e} \leq  A{t}^{2}/{Jt} + {v}_{s} \\  {Jt} \cdot  {v}_{e}^{2} + {At} \cdot  {v}_{e} - {Jt} \cdot  {v}_{s}^{2} + A{t}^{2} \cdot  {v}_{s} \\   - {2At} \cdot  {Jt} \cdot  s,{v}_{e} > A{t}^{2}/{Jt} + {v}_{s} \end{array}\right. \]

(B.1)

Through implicit differentiation and given constant \( {At},{Jt} \) , and \( s \) ,there is no globally strict monotonic relationship between initial and final feedrates \( \left( {v}_{s}\right. \) and \( \left. {v}_{e}\right) \) within \( \left\{  {{v}_{s},{v}_{e} \mid  {v}_{s}}\right. \) \( \left. { \geq  0,{v}_{e} > 0\text{ and }{v}_{s} \neq  {v}_{e}}\right\} \) . This means as \( {v}_{s} \) increases, \( {v}_{e} \) might incre-ase or decrease under the same conditions. In look-ahead window planning illustrated in Fig.31, \( {v}_{1}^{p},{v}_{2}^{p} \) ,and \( {v}_{3}^{p} \) denote reachable feedrates at points 1, 2, and 3, respectively, obtained via bidirectional scanning in the red window. Upon moving the window to the green region, \( {v}_{1}^{p} \) becomes the initial feedrate, but influenced by a new segment at the tail end, the feedrate at point 3 rises to \( {v}_{3} \) . Without adjustment,two scenarios arise:

<!-- Media -->

<!-- figureText: \( {v}_{1}^{p}\left( {v}_{0}\right) \) \( {V}_{3} \) ② \( {v}_{2} \) 2 ... 0 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_16.jpg?x=1085&y=156&w=408&h=256&r=0"/>

Fig. 31. Schematic diagram of the look-ahead feedrate scheduling failure.

<!-- Media -->

(1) If \( {v}_{2} \) during forward scanning exceeds the original \( {v}_{2}^{p} \) , the minimum successful planned feedrate is adopted, enabling feasibility.

(2) If the backward scanning yields \( {v}_{2} \) smaller than \( {v}_{2}^{p} \) , it prevents reducing the initial \( {v}_{1}^{p} \) to \( {v}_{2} \) in the current window, risking algorithm to fail. Applying feedrate scheduling directly to all segments with initial and final velocities both at zero eliminates these issues but reduces efficiency.

The proposed method introduces a segmented dynamic adaptive smoothing pre-processing approach based on the local dynamic window for the MVC. This method facilitates feedrate scheduling without bidirectional scanning, ensuring the feedrate curve continuity within the current window and is particularly well-suited to look-ahead feedrate scheduling. The feasibility of this approach is examined below:

As the lookahead window is moved forward, outputted head results are utilized for interpolation, and the incoming tail of new segment is employed to rectify the last segment in the preceding window's planning. As depicted in Fig.32, after planning in the blue window,PlanBlock stores \( N \) pre-planned datasets \( \operatorname{Seg}\left( i\right) \) . These data are determined through the local dynamic window scanning in Section III-B, utilizing the 7-segment S-shaped acceleration/deceleration algorithm. This paper proposes restricting the pre-planning task to only include a bidirectional verification of one acceleration/ deceleration within each window to address acceleration segment limitations. Completion of local dynamic window scanning ensures the viability of cached data for interpolation. Upon window shift and new segment enters the tail, a preliminary check assesses if additional curve segments exist behind the look-ahead window, with two possible scenarios:

(1) If no subsequent curve segments exist (green window in Fig.32), indicating a stop segment in the current window,the incoming segment \( \operatorname{Seg}\left( N\right) \) undergoes cross-segment merging. Subsequently, each segment is scanned and smoothed via local dynamic window preprocessing. Upon completion, the entire output is ready for interpolation, marking the end of the entire trajectory planning process.

(2) If curve segments exist behind the look-ahead (the pink window in the Fig.32), indicating no stop segment in the current window,the new segment \( \operatorname{Seg}\left( N\right) \) does not adopt the cross-segment merging algorithm. Instead, it uses initial feedrate \( {v}_{s,N} = 0 \) and the data of \( \operatorname{Seg}\left( N\right) \) as motion parameters. The final reachable feedrate \( {v}_{e} \) is calculated using (B.1),correcting \( \operatorname{Seg}\left( {N - 1}\right) .{v}_{e,N - 1} = \) \( \min \left( {{v}_{{lim},N},\operatorname{Seg}\left( {N - 1}\right) .{v}_{{lim},N - 1},{v}_{e}}\right) \) . If \( \operatorname{Seg}\left( {N - 1}\right) \) . \( {l}_{N - 1} \) satisfies the variable-feedrate process for the \( N - 1 \) segment,the original parameters of \( \operatorname{Seg}\left( {N - 1}\right) \) are updated,setting \( \operatorname{Seg}\left( N\right) .{v}_{s,N} = \operatorname{Seg}\left( {N - 1}\right) .{v}_{e,N - 1} \) and \( \operatorname{Seg}\left( N\right) .{v}_{e,N} = 0 \) ,facilitating direct look-ahead window advancement to enhance planning speed. The head results are then prepared for interpolation; If not satisfied, as per Section III-B, merging arc lengths is necessary to maintain the variable-feedrate process. After dynamic scanning and smoothing of each segment in the look-ahead window, the lookahead window is shifted forward, and the head results are prepared for interpolation. REFERENCES

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. 18 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING-->

<!-- Media -->

<!-- figureText: \( V \) Look-ahead window 0 Moving direction -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_17.jpg?x=148&y=160&w=722&h=331&r=0"/>

Fig. 32. Schematic diagram of the feasibility analysis of the proposed.

<!-- Media -->

[1] W. Wang, Q. Guo, Z. Yang, Y. Jiang, and J. Xu, "A state-of-the-art review on robotic milling of complex parts with high efficiency and precision," Robot. Comput.-Integr. Manuf., vol. 79, Feb. 2023, Art. no. 102436, doi: 10.1016/j.rcim.2022.102436.

[2] X. Zeng, G. Zhu, Z. Gao, R. Ji, J. Ansari, and C. Lu, "Surface polishing by industrial robots: A review," Int. J. Adv. Manuf. Technol., vol. 125, nos. 9-10, pp. 3981-4012, Apr. 2023, doi: 10.1007/s00170-023-10887- 3.

[3] A. Verl, A. Valente, S. Melkote, C. Brecher, E. Ozturk, and L. T. Tunc, "Robots in machining," CIRP Ann.-Manuf. Technol., vol. 68, no. 2, pp. 799-822, Aug. 2019, doi: 10.1016/j.cirp.2019.05.009.

[4] G. Wang, Q. Shu, J. Wang, and L. Li, "Research on adaptive nonuniform rational B-spline real-time interpolation technology based on acceleration constraints," Int. J. Adv. Manuf. Technol., vol. 91, nos. 5-8, pp. 2089-2100, Jul. 2017, doi: 10.1007/s00170-016-9914-4.

[5] G. Zhang, J. Gao, L. Zhang, X. Wang, Y. Luo, and X. Chen, "Generalised NURBS interpolator with nonlinear feedrate scheduling and interpolation error compensation," Int. J. Mach. Tools Manuf., vol. 183, Dec. 2022, Art. no. 103956, doi: 10.1016/j.ijmachtools.2022.103956.

[6] L. Fang, G. Liu, Q. Li, and H. Zhang, "A high-precision non-uniform rational B-spline interpolator based on S-shaped feedrate scheduling," Int. J. Adv. Manuf. Technol., vol. 121, nos. 3-4, pp. 2585-2595, Jul. 2022, doi: 10.1007/s00170-022-09411-w.

[7] X. Du, J. Huang, and L.-M. Zhu, "A complete S-shape feed rate scheduling approach for NURBS interpolator," J. Comput. Des. Eng., vol. 2, no. 4, pp. 206-217, Oct. 2015, doi: 10.1016/j.jcde.2015.06.004.

[8] H. Zhao, L. Zhu, and H. Ding, "A real-time look-ahead interpolation methodology with curvature-continuous B-spline transition scheme for CNC machining of short line segments," Int. J. Mach. Tools Manuf., vol. 65, pp. 88-98, Feb. 2013, doi: 10.1016/j.ijmachtools.2012.10.005.

[9] S. Y. Jeong, Y. J. Choi, P. Park, and S. G. Choi, "Jerk limited velocity profile generation for high speed industrial robot trajectories," IFAC Proc. Volumes, vol. 38, no. 1, pp. 595-600, 2005.

[10] X. Zhao, H. Zhao, J. Yang, and H. Ding, "An adaptive feedrate scheduling method with multi-constraints for five-axis machine tools," in Intelligent Robotics and Applications, H. Liu, N. Kubota, X. Zhu, R. Dillmann, and D. Zhou, Eds., Cham, Switzerland: Springer, 2015, pp. 553-564, doi: 10.1007/978-3-319-22876-1_48.

[11] Y. Sun, J. Jia, J. Xu, M. Chen, and J. Niu, "Path, feedrate and trajectory planning for free-form surface machining: A state-of-the-art review," Chin. J. Aeronaut., vol. 35, no. 8, pp. 12-29, Aug. 2022, doi: 10.1016/j.cja.2021.06.011.

[12] Siemens AG. (Dec. 2018). SINUMERIK 840D SL Special Functions. [Online]. Available: https://support.industry.siemens.com/cs/document/109763229/sinumerik-840d-sl-special-functions?dti=0&lc=en-WW

[13] Siemens AG. (Jul. 15, 2023). CNC Robotics-New PerspecTives When Using Robots. [Online]. Available: https://www.siemens.com/global/en/industries/machinebuilding/machine-tools/cnc4you/technologies/cnc-robotics.html

[14] W. Zhong, X. Luo, W. Chang, F. Ding, and Y. Cai, "A real-time interpolator for parametric curves," Int. J. Mach. Tools Manuf., vol. 125, pp. 133-145, Feb. 2018, doi: 10.1016/j.ijmachtools.2017.11.010.

[15] P.-Y. Tang, M.-T. Lin, and M.-S. Tsai, "Real-time master-based feedrate scheduling with kinematic constraints for five-axis machining," Int. J. Adv. Manuf. Technol., vol. 123, nos. 1-2, pp. 493-510, Nov. 2022, doi: 10.1007/s00170-022-10172-9.

[16] S.-K. Wu, M.-S. Tsai, M.-T. Lin, and H.-W. Huang, "Development of novel tool center point velocity planning algorithm for five axis machine tool," Int. J. Precis. Eng. Manuf., vol. 19, no. 8, pp. 1187-1199, Aug. 2018, doi: 10.1007/s12541-018-0140-x.

[17] X. Beudaert, S. Lavernhe, and C. Tournier, "Feedrate interpolation with axis jerk constraints on 5-axis NURBS and G1 tool path," Int. J. Mach. Tools. Manuf., vol. 57, pp. 73-82, 2012, doi: 10.1016/j.ijmachtools.2012.02.005.

[18] J. Zhou, Y. Sun, and D. Guo, "Adaptive feedrate interpolation with multiconstraints for five-axis parametric toolpath," Int. J. Adv. Manuf. Technol., vol. 71, nos. 9-12, pp. 1873-1882, Apr. 2014, doi: 10.1007/s00170-014-5635-8.

[19] P. Xu et al., "Stiffness modeling of an industrial robot with a gravity compensator considering link weights," Mechanism Mach. Theory, vol. 161, Jul. 2021, Art. no. 104331, doi: 10.1016/j.mechmachtheory.2021.104331.

[20] Z.-Y. Liao, Z.-Z. Qin, H.-L. Xie, Q.-H. Wang, and X.-F. Zhou, "Constant load toolpath planning and stiffness matching optimization in robotic surface milling," Int. J. Adv. Manuf. Technol., vol. 130, nos. 1-2, pp. 353-368, Jan. 2024, doi: 10.1007/s00170-023-12639-9.

[21] J.-W. Ma, S. Gao, H.-T. Yan, Q. Lv, and G.-Q. Hu, "A new approach to time-optimal trajectory planning with torque and jerk limits for robot," Robot. Auto. Syst., vol. 140, Jun. 2021, Art. no. 103744, doi: 10.1016/j.robot.2021.103744.

[22] R. Wang, Y. Xie, X. Chen, and Y. Li, "Path-constrained time-optimal motion planning for robot manipulators with third-order constraints," IEEE/ASME Trans. Mechatronics, vol. 28, no. 6, pp. 3005-3016, Dec. 2023, doi: 10.1109/TMECH.2023.3234584.

[23] C. Ji, Z. Zhang, G. Cheng, M. Kong, and R. Li, "A convex optimization method to time-optimal trajectory planning with jerk constraint for industrial robotic manipulators," IEEE Trans. Autom. Sci. Eng., early access, Dec. 29, 2004, doi: 10.1109/TASE.2023.3346693.

[24] D. Verscheure, B. Demeulenaere, J. Swevers, J. De Schutter, and M. Diehl, "Time-optimal path tracking for robots: A convex optimization approach," IEEE Trans. Autom. Control, vol. 54, no. 10, pp. 2318-2327, Oct. 2009, doi: 10.1109/TAC.2009.2028959.

[25] M. S. Paing, E. W. Nshama, and N. Uchiyama, "A kinematically constrained reparameterization approach to optimal time and jerk motion of industrial machines," IEEE Access, vol. 9, pp. 97843-97854, 2021, doi: 10.1109/ACCESS.2021.3095847.

[26] G. Liu, Q. Li, B. Yang, H. Zhang, and L. Fang, "An efficient linear programming-based time-optimal feedrate planning considering kinematic and dynamics constraints of robots," IEEE Robot. Autom. Lett., vol. 9, no. 3, pp. 2742-2749, Mar. 2024, doi: 10.1109/LRA.2024.3359547.

[27] Á. Nagy and I. Vajk, "Sequential time-optimal path-tracking algorithm for robots," IEEE Trans. Robot., vol. 35, no. 5, pp. 1253-1259, Oct. 2019, doi: 10.1109/TRO.2019.2920090.

[28] T. Kunz and M. Stilman, "Time-optimal trajectory generation for path following with bounded acceleration and velocity," in Proc. Robot. Sci. Syst., 2012, pp. 1-8, doi: 10.7551/mitpress/9816.003.0032.

[29] Q.-C. Pham, "A general, fast, and robust implementation of the time-optimal path parameterization algorithm," IEEE Trans. Robot., vol. 30, no. 6, pp. 1533-1540, Dec. 2014, doi: 10.1109/TRO.2014.2351113.

[30] H. Pham and Q.-C. Pham, "A new approach to time-optimal path parameterization based on reachability analysis," IEEE Trans. Robot., vol. 34, no. 3, pp. 645-659, Jun. 2018, doi: 10.1109/TRO.2018.2819195.

[31] L. Berscheid and T. Kröger, "Jerk-limited real-time trajectory generation with arbitrary target states," 2021, arXiv:2105.04830.

[32] T. Kröger and F. M. Wahl, "Online trajectory generation: Basic concepts for instantaneous reactions to unforeseen events," IEEE Trans. Robot., vol. 26, no. 1, pp. 94-111, Feb. 2010, doi: 10.1109/TRO.2009.2035744.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: This article has been accepted for inclusion in a future issue of this journal. Content is final as presented, with the exception of pagination. XU et al.: SEGMENTED DYNAMIC ADAPTIVE LOOK-AHEAD SMOOTHING FEEDRATE SCHEDULING 19-->

[33] F. Lange and A. Albu-Schäffer, "Path-accurate online trajectory generation for jerk-limited industrial robots," IEEE Robot. Autom. Lett., vol. 1, no. 1, pp. 82-89, Jan. 2016, doi: 10.1109/LRA.2015.2506899.

[34] X. Du, J. Huang, L.-M. Zhu, and H. Ding, "Third-order chord error estimation for freeform contour in computer-aided manufacturing and computer numerical control systems," Proc. Inst. Mech. Eng., \( B,J.{Eng}.{Manuf}., \) vol. 233,no. 3,pp. 863-874,Feb. 2018,doi: 10.1177/0954405418757266.

[35] A.-C. Lee, M.-T. Lin, Y.-R. Pan, and W.-Y. Lin, "The feedrate scheduling of NURBS interpolator for CNC machine tools," Comput.-Aided Des., vol. 43, no. 6, pp. 612-628, Jun. 2011, doi: 10.1016/j.cad.2011.02.014.

[36] Y. Sun, Y. Zhao, Y. Bao, and D. Guo, "A novel adaptive-feedrate interpolation method for NURBS tool path with drive constraints," Int. J. Mach. Tools Manuf., vol. 77, pp. 74-81, Feb. 2014, doi: 10.1016/j.ijmachtools.2013.11.002.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_18.jpg?x=140&y=678&w=220&h=278&r=0"/>

<!-- Media -->

Yan Xu received the master's degree in mechanical engineering from Northeastern University, Liaoning, China, in 2020. She is currently pursuing the Ph.D. degree with the College of Computer and Control Engineering, Northeast Forestry University. Her current research interests include robot motion planning, high-precision trajectory planning, and composite robot control.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_18.jpg?x=138&y=1054&w=221&h=278&r=0"/>

<!-- Media -->

Yaqiu Liu received the Ph.D. degree from Harbin Institute of Technology. Currently, he is a Professor with the College of Computer and Control Engineering, Northeast Forestry University. His current research interests include process control, robot control, trajectory, and motion planning.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_18.jpg?x=912&y=172&w=218&h=275&r=0"/>

<!-- Media -->

Xun Liu received the Ph.D. degree from the College of Computer and Control Engineering, Northeast Forestry University. He is currently a Post-Doctoral Researcher in mechanical engineering with Shanghai Jiao Tong University, Shanghai, China. His research interests include robot control, motion planning, and control algorithms.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_18.jpg?x=909&y=583&w=226&h=280&r=0"/>

<!-- Media -->

Jiabin Cao received the B.Sc. degree in automation from Harbin Engineering University. He is currently pursuing the master's degree with the School of Mechanical Engineering, Shanghai Jiao Tong University. His main research interests include robotic machining, vibration control, and intelligent control.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2sl9p77aajc738sf820_18.jpg?x=906&y=989&w=233&h=277&r=0"/>

<!-- Media -->

Lin Zhang (Member, IEEE) received the B.Sc. degree in mechanical engineering and automation from China University of Mining and Technology and the joint Ph.D. degree in mechanical manufacturing and automation from China University of Mining and Technology and the University of California at San Diego. He is currently an Associate Professor with Yantze Normal University. He is also with the School of Mechanical Engineering, Shanghai Jiao Tong University, as a Post-Doctoral Researcher. His main research interests include climbing robotics, intelligent control, metamorphotic theory, sensorless sensing, and reinforcement robot learning.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:15:52 UTC from IEEE Xplore. Restrictions apply.-->

