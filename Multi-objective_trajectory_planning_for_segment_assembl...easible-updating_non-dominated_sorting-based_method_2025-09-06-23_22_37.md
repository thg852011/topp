

<!-- Meanless: Applied Soft Computing Journal 152 (2024) 111216 Contents lists available at ScienceDirect Applied Soft Computing ELSEVIER journal homepage: www.elsevier.com/locate/asoc updates-->

# Multi-objective trajectory planning for segment assembly robots using a B-spline interpolation- and infeasible-updating non-dominated sorting-based method

Hao Sun \( {}^{a} \) ,Jianfeng Tao \( {}^{a, * } \) ,Chengjin Qin \( {}^{a} \) ,Chang Dong \( {}^{a} \) ,Shuang Xu \( {}^{a} \) ,Qianwei Zhuang \( {}^{b} \) , Chengliang Liu \( {}^{a} \)

\( {}^{a} \) State Key Laboratory of Mechanical System and Vibration,School of Mechanical Engineering,Shanghai Jiao Tong University,Shanghai 200240,China \( {}^{\mathrm{b}} \) Shanghai Tunnel Engineering Co.,Ltd.,Shanghai 200232,China

## \( \mathrm{H}\mathrm{I}\mathrm{G}\mathrm{H}\mathrm{L}\mathrm{I}\mathrm{G}\mathrm{H}\mathrm{T}\mathrm{S} \)

- A multi-objective optimization model for trajectory planning for the segment assembly robot of non-circular shield machines has been established.

- An infeasible-updating non-dominated sorting-based evolutionary algorithm (INSEA) is presented to solve this problem and achieve better performance.

- Based on the proposed INSEA, the interpolation performance of B-spline curves performs better than the common cubic spline curves.

- The proposed method could give researchers a wide range of options to achieve the optimum trade-off for such trajectory planning problems.

## ARTICLEINFO

Keywords:

Segment assembly robot

Multi-objective trajectory planning

B-spline

Feasibility rule

NSGA-II

## A B S T R A C T

The rapid and smooth functioning of segment assembly robots, which is always conflicting, is critical to improving efficiency and ensuring safety during tunneling construction, particularly for the series-actuated robots employed in non-circular shield machines. However, the trade-off between the aforementioned goals has not been explored for trajectory planning in joint space. Less is known about how to acquire superior trade-off Pareto solutions for this constrained multi-objective optimization problem. To fill this gap, this paper proposes a B-spline interpolation- and infeasible-updating non-dominated sorting-based method for multi-objective trajectory planning of shield machine segment assembly robots. In particular, a multi-objective optimization model is detailed in terms of time, acceleration, and jerk of joint trajectories while accounting for operational efficiency and motion smoothness. The given objective functions can be determined using B-spline interpolation and time information based on the known via-points of hydraulic joints. Meanwhile, the constraints are transformed into a limited number of control point-derived forms. Following that, the infeasible-updating non-dominated sorting-based evolutionary algorithm (INSEA) is introduced to solve this problem and find Pareto-optimal solutions. The main improvement is that the multi-objective function information of infeasible solutions is utilized so as to update the non-dominated sorting process, which adaptively applies population division and individual replacement. The findings indicate that the proposed method is capable of executing multi-objective trajectory planning during different stages of the assembly procedure, and the computed metrics are all greater than serval advanced algorithms. Furthermore, on the basis of the proposed INSEA, multi-degree B-spline interpolation provides lower acceleration peaks and smoother global trajectories than common cubic spline curves throughout the process. Therefore, this method can provide researchers with a wide range of alternatives to achieve the optimum trade-off for multi-objective trajectory planning.

---

<!-- Footnote -->

* Corresponding author.

E-mail address: jftao@sjtu.edu.cn (J. Tao).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.asoc.2023.111216 Received 19 June 2023; Received in revised form 24 November 2023; Accepted 24 December 2023 Available online 29 December 2023 1568-4946/(C) 2023 Elsevier B.V. All rights reserved.-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

## 1. Introduction

The tunnels are excavated by a shield machine, using cutter head for excavation and assembling segments behind the machine. Segment assembly robots are hydraulically powered manipulators that could be employed to insert concrete segments to line tunnels. During the process, the quick movement and minimal vibration of robots are critical to guaranteeing efficient and safe operation. However, no theoretical work has been done to optimize the assembly trajectory while keeping the objectives in mind. How to explore excellent non-dominated solutions to plan trajectories based on the spline interpolation and the multi-objective evolutionary algorithm has not been investigated for constrained multi-objective trajectory planning (MOTP).

One of the primary processes in shield tunnel building is segment assembly \( \left\lbrack  {1,2}\right\rbrack \) . After digging a distance,an erector is used to construct a ring of segments. This procedure is essential for ensuring tunnel dimension accuracy [3], improving water tightness [4,5], and reducing ground surface settling [6]. Significantly, the segment assembly working cycle is time-consuming, resulting in lower operational efficiency for the whole tunnel lining construction [7]. To address this issue, Kosuge et al. [8] omitted the exact sensing method in order to speed up segment assembly. Rather than employing exact position and orientation information for a segment, the pressure sensor is connected to each chamber of hydraulic actuators to compute the output force for control. Gong et al. [9-11] raised the operation speed of the segment erector to ensure position accuracy and reduce shock force. The suggested system, which uses speed and position compound control, can save around \( {60}\% \) more time for a 10-kilometer-long tunnel, but it would necessarily result in an unsmooth trajectory. Yang et al. [12] employed a magnetorheological damper to achieve segment erector shock absorption, which considerably decreases the negative impact produced by excessive acceleration. It is worth mentioning that enhancing efficiency and maintaining smooth operation are vital for shield machine segment assembly robots.

In addition to the circular shield machines discussed above, noncircular shield machines are also used to build underground space in order to maximize space usage. Correspondingly, the usual configuration of segment assembly machines has been replaced [13-15]. Instead of traditional series-parallel mechanisms, serial multiple-joint manipulators are employed to finish segment erection for flexibility. In the meantime, the hydraulic circuit employs a proportional directional valve-controlled actuator with counterbalance valves [16,17]. Nonetheless, the vibrations caused by such hydraulic joints, which are induced by the reference input, cannot be ignored. If load pressure is defined to include the additional force characterized by restricted acceleration, it is preferred for such hydraulic systems to have lower values so as to have sufficient control capability.

Trajectory planning in cartesian or joint space is an efficient technique to solve the challenges mentioned above. Each joint actuator will receive the desired control command. As noted in [11], multi-axis simultaneous movement could improve the efficiency of hydraulic manipulators, even though it is influenced by coupling issues. In the configuration space, Morales et al. [18,19] maximized the velocity along the predefined path. The produced trajectory is shaped near the velocity and link restrictions of the hydraulic manipulator. To create smooth transitions, a certain discrepancy between the beginning and final states is permitted. Zhang et al. [20] optimized the time and jerk of a cubic spline-interpolated trajectory in joint space employing sequential quadratic programming. The best result could be used to ensure the efficient and smooth operation of the hydraulic excavator. By giving weight factors, the two objectives are turned into a single-objective optimization. Furthermore, Zou et al. [21] regarded time efficiency, energy savings, and machine damage as optimization goals to construct the corresponding trajectory. The conflicting nature of the optimization goals has also been revealed. As can be seen, the simultaneous consideration of multiple objectives, particularly those that appear to be contradictory, is critical for hydraulic manipulator trajectory planning. Pareto-optimal solutions provide numerous flexible and reliable options for corresponding requirements. Meanwhile, considering the characteristics of hydraulic joints, planning in joint space could provide trajectories more directly beneficial to the smooth operation of joints than planning in the operation space. Consequently, it is advantageous for the segment assembly robot in the noncircular shield machine to simultaneously optimize time, acceleration, and jerk during the operation. For trajectory planning in joint space, a multi-objective optimization method is effective. Researchers could collect valuable information that would aid them in making compromise decisions. That is why traditional methods, which incorporate multiple objectives into a single goal, are not frequently utilized [22,23].

The above case is essentially a constrained multi-objective optimization problem (CMOP). An optimal collection of non-dominated solutions allows for a suitable trade-off between competing objectives. On this premise, evolutionary algorithms are widely used since they just demand that the value of the optimized function be computable and not continuous and differentiable, particularly if one or more constraints are present [24-26]. When combined with constraint-handling techniques, the algorithms may solve difficult CMOPs [27]. The non-dominated sorting genetic algorithm II (NSGA-II) has been frequently applied in this field to address a wide range of multi-objective optimization problems [28-30]. It is distinguished by a fast non-dominated sorting strategy, a fast-crowded distance estimation procedure, and a simple crowded comparison operator. This algorithm and its updated version have also been effectively employed for industrial robot planning [31-34]. There are many inequality restrictions governing the velocity, acceleration, and jerk of the joints, which results in a highly irregular shape of the feasible region, particularly as the limitations increase. In this case, the typical feasibility rule presented in [28] makes it impossible to effectively explore optimal solutions. Current advancements are mostly focused on constraint management approaches [27,35], non-dominated sorting [36,37], and evolutionary algorithms [38]. In particular, the extraction of useful information from the population could improve the environment of evolution and permits more potential solutions to reach the following generation. An effective way is that a substitute technique described in [39] is to reduce the greediness of the feasibility rule by taking objective function information for the infeasible individual into account. This gives alternative evolutionary pathways from infeasible to feasible areas and is well described and utilized in differential evolution for solving single-objective problems. However, no commensurate enhancement or explanation has been provided for the NSGA-II standard process. Meanwhile, obtaining the genuine Pareto front is always a difficult task for multi-objective trajectory planning.

To address the issues, a B-spline interpolation- and infeasible-updating non-dominated sorting-based method is introduced for multi-objective trajectory planning of shield machine segment assembly robots. To start with, the multi-objective optimization model is built with operational efficiency and motion smoothness in mind. The execution time of the intended trajectory is considered an essential aim for the operational efficiency of the assembly procedure. Meanwhile, the root-mean-square (RMS) acceleration and jerk are utilized to characterize the smoothness of trajectories for segment assembly robot hydraulic joints. The robot motion trajectory is then fitted using multi-degree nonuniform rational B-spline (NURBS) curves across multiple reference points. Furthermore, by utilizing a limited number of control points, the established constraints for kinematic parameters can be simplified. Consequently, the objective functions can be determined using time-related design variables. On this basis, the proposed infeasible-updating non-dominated sorting-based evolutionary algorithm (INSEA) is applied to optimize the preceding model to find the best Pareto solutions. Specifically, the improved feasibility rule incorporates multi-objective function information for infeasible solutions, thereby increasing the evolution direction. To validate the proposed INSEA, multiple advanced algorithms are compared during the two operation stages. Meanwhile, ablation experiments are conducted further to verify the effectiveness of the infeasible-updating non-dominated sorting strategy. In addition to NURBS, the most popular cubic spline curves are also used to investigate the interpolation performance based on the proposed algorithm. The following are the significant contributions of this work:

<!-- Meanless: 2-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

1) A multi-objective optimization model for trajectory planning for the segment assembly robot of non-circular shield machines has been developed, considering factors such as time, acceleration, and jerk of the hydraulic servo joints.

2) An improved non-dominated sorting process incorporating multi-objective function information is presented, which is especially beneficial for discovering true Pareto solutions in the case of complex feasible areas on trajectory planning problems.

3) Based on the proposed INSEA, the interpolation performance of NURBS and cubic spline curves is investigated. It not only reduces acceleration peaks, but it also assures smooth trajectories, particularly at the start and end of the assembly stages.

4) When confronted with different assembly requirements, the proposed method provides researchers with more great trade-off options for implementing multi-objective trajectory planning of shield machine segment assembly robots.

The remaining section of this paper can be summarized as follows. The multi-objective optimization model of trajectory planning is established in Section 2. The proposed method for solving this problem is thoroughly described in Section 3. Validation and analysis are covered in Section 4. The conclusions are found in Section 5.

## 2. Mathematical model of MOTP problems

During segment erection, the end effector of the segment assembly robot should be moved from where it started to the final position. In this study, path planning was done in the operational space of robots before trajectory planning. It is possible to get successive positions and orientations of the end effector through a series of via-points on the path. Then, using kinematic inversion, they are translated into the right positions in joint space to carry out control actions. These via-points in joint space could be written as

\[{}^{m}\mathbf{p} = \left\{  {m}_{{p}_{i}}\right\}   = \left\{  \left( {{t}_{i},{m}_{{p}_{i}}}\right) \right\}  \;\left( {i = 0,1,\ldots ,{n}_{p},m = 1,\ldots ,M}\right)  \tag{1}\]

where \( {}^{m}{p}_{i} \) and \( {t}_{i} \) denote the position and time of the \( m \) -th joint at the \( i \) -th via-point,respectively; \( {n}_{p} \) and \( M \) represent the entire number of via-points as well as joints, respectively.

The objective function is developed during trajectory planning in joint space to increase operating efficiency, which can be stated as

\[{f}_{1} = \mathop{\sum }\limits_{{i = 0}}^{{{n}_{p} - 1}}{h}_{i + 1} = \mathop{\sum }\limits_{{i = 0}}^{{{n}_{p} - 1}}\left( {{t}_{i + 1} - {t}_{i}}\right)  \tag{2}\]

where \( {h}_{i + 1} \) is the calculated time interval across both positions on the path.

The hydraulic servo joints of the robot should have enough control capacity. The remaining power margin can help control methods to achieve better tracking performance by decreasing the RMS acceleration of the executed trajectory \( \left\lbrack  {{16},{40}}\right\rbrack \) . It can be expressed mathematically as

\[{f}_{2} = \frac{1}{M}\mathop{\sum }\limits_{{m = 1}}^{M}\sqrt{\mathop{\sum }\limits_{{i = 0}}^{{n}_{p}}\frac{{a}_{m}^{2}\left( {t}_{i}\right) }{{n}_{p} + 1}} \tag{3}\]

where \( {a}_{m}\left( {t}_{i}\right) \) denotes the acceleration of the \( m \) -th joint along the \( i \) -th via-point.

A high time rate of acceleration along the trajectory of the joint actuator promotes the excitation of resonant frequencies and generates further vibration in the hydraulic joints. As a result, the RMS jerk could be considered an additional optimization target to ensure motion smoothness \( \left\lbrack  {{41},{42}}\right\rbrack \) ,namely

\[{f}_{3} = \frac{1}{M}\mathop{\sum }\limits_{{m = 1}}^{M}\sqrt{\mathop{\sum }\limits_{{i = 0}}^{{n}_{p}}\frac{{j}_{m}^{2}\left( {t}_{i}\right) }{{n}_{p} + 1}} \tag{4}\]

where \( {j}_{m}\left( {t}_{i}\right) \) is the jerk of the \( m \) -th joint at the \( i \) -th via-point.

The kinematic parameters, such as actuator velocity, acceleration, and jerk, must be defined on a constrained interval, which is an indirect method that ignores the dynamical characteristics of the system model. The following limitations apply:

\[\left\{  \begin{array}{l} \left| {{v}_{m}\left( t\right) }\right|  \leq  {}^{m}{V}_{\max } \\  \left| {{a}_{m}\left( t\right) }\right|  \leq  {}^{m}{A}_{\max } \\  \left| {{j}_{m}\left( t\right) }\right|  \leq  {}^{m}{J}_{\max } \end{array}\right.  \tag{5}\]

where \( {v}_{m}\left( t\right) \) denotes the velocity of the \( m \) -th joint at any moment; \( {}^{m}{V}_{\max } \) , \( {}^{m}{A}_{\max } \) ,and \( {}^{m}{J}_{\max } \) represent the upper bound of kinematic constraints, respectively.

This model seeks to minimize the objective functions \( {f}_{1},{f}_{2} \) ,and \( {f}_{3} \) . By minimizing \( {f}_{1} \) ,segment assembly efficiency can be improved. Reducing \( {f}_{2} \) and \( {f}_{3} \) will avoid inadequate control and excessive mechanical vibration. There is a certain contradiction between minimizing assembly time and reducing trajectory acceleration or jerk in segment assembly robots. Although decreasing trajectory acceleration or jerk can enhance stability and reduce vibrations, this often comes at the cost of sacrificing operational efficiency. Implementing the three objectives permits a flexible option that establishes a balance between efficient execution and smooth performance.

### 3.The proposed trajectory planning method

In this section, NURBS curves are employed to construct the joint trajectories of the hydraulic manipulator. The control points of the NURBS curves, meanwhile, transform and represent the kinematic constraints of the produced trajectories. Based on the feasibility rule in [28], an update mechanism for the non-dominated sorting process of NSGA-II is presented that takes objective function information into account for infeasible solutions. The complete procedure of the proposed method is provided to solve MOTP problems.

### 3.1. B-spline trajectory construction

The trajectory generated by the planning algorithm should be sufficiently smooth to avoid mechanical resonance excitation. Particularly, a smooth jerk trajectory can reduce this impact from the joint actuator. Therefore, multi-degree non-uniform rational B-spline (NURBS) curves are often utilized to build the trajectory since the function of finite order is continuously differentiable. They also have a local support feature, which means that modifying one node does not change the trajectory curve as a whole. In this research, B-spline curves are defined as

\[\left\{  \begin{matrix} m{}_{p}\left( u\right)  = \mathop{\sum }\limits_{{j = 0}}^{n}{}^{m}{d}_{j}{N}_{j,k}\left( u\right)  = \mathop{\sum }\limits_{{j = {r}_{ind} - k}}^{{r}_{ind}}{}^{m}{d}_{j}{N}_{j,k}\left( u\right) \\  u \in  \left\lbrack  {{u}_{{r}_{ind}},{u}_{{r}_{ind} + 1}) \subset  \left\lbrack  {{u}_{k},{u}_{n + 1}}\right\rbrack  }\right\rbrack   \subset  \left\lbrack  {{u}_{0},{u}_{n + k + 1}}\right\rbrack  \left( {{r}_{ind} = k,k + 1,\ldots ,n}\right)  \end{matrix}\right.  \tag{6}\]

where \( u \) denotes the normalized knot variable; \( k \) is the degree of B-spline curves; \( {d}_{j} \) is the \( j \) -th control point of the total number \( \left( {n + 1}\right) ;{N}_{j,k}\left( u\right) \) is the basis function of B-spline curves.

The knot points of the interpolated B-spline curve could correspond aforementioned \( \left( {{n}_{p} + 1}\right) \) via-points in joint space. To construct clamped B-spline curves, the multiplicity of the knot points at both ends is set to \( \left( {k + 1}\right) \) . Thus,the knot vector of \( k \) -th-degree B-splines could be defined as

\[\mathbf{U} = \left\lbrack  {{u}_{0},\ldots ,{u}_{k},{u}_{k + 1},\ldots ,{u}_{{n}_{p} + k - 1},{u}_{{n}_{p} + k},\ldots ,{u}_{{n}_{p} + {2k}}}\right\rbrack   \tag{7}\]

By adopting the accumulative chord length method, the knot variable \( {u}_{r} \) is normalized by the time variable \( {t}_{i} \) as

<!-- Meanless: 3-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

\[{u}_{r} = {u}_{r - 1} + \frac{{t}_{r - k} - {t}_{r - k - 1}}{{t}_{{n}_{p}} - {t}_{0}},r = k + 1,k + 2,\ldots ,{n}_{p} + k - 1 \tag{8}\]

Additional knots are extended at each end, namely

\[{u}_{r} = \left\{  \begin{matrix} 0,r = 0,1,\ldots ,k \\  1,r = {n}_{p} + k,{n}_{p} + k + 1,\ldots ,{n}_{p} + {2k} \end{matrix}\right.  \tag{9}\]

A \( k \) -th degree B-spline for interpolating \( \left( {{n}_{p} + 1}\right) \) path points in joint space is denoted as

\[{}^{m}p\left( {u}_{i + k}\right)  = \mathop{\sum }\limits_{{j = i}}^{{i + k}}{}^{m}{d}_{j}{N}_{j,k}\left( {u}_{i + k}\right)  = {}^{m}{p}_{i},{u}_{i + k} \in  \left\lbrack  {{u}_{k},{u}_{{n}_{p} + k}}\right\rbrack   \tag{10}\]

where \( j = 0,1,\ldots ,{n}_{p} + k - 1 \) represents \( \left( {{n}_{p} + k}\right) \) control points are prepared to be calculated; \( {N}_{j,k}\left( {u}_{i + k}\right) \) is derived according to the Cox-de Boor recursion formula [43].

From Eq. (10),there are \( \left( {{n}_{p} + 1}\right) \) known equations that can be used to solve \( \left( {{n}_{p} + k}\right) \) control points. Additional(k - 1)equations could be given by the boundary conditions of kinematic parameters that are expressed by the \( w \) -th order derivative \( {}^{m}{p}^{w}\left( u\right) \) . In particular,at two endpoints \( \left( {t}_{0}\right. \) and \( \left. {t}_{{n}_{p}}\right) \) ,the values of these parameters,including velocity, acceleration, and jerk, should be assigned to zero to further promote smooth movement of the hydraulic joint. Then the degree \( k \) of the B-spline can be determined. The \( w \) -th order derivative \( {}^{m}{p}^{w}\left( u\right) \) is formulated as

\[{}^{m}{p}^{w}\left( u\right)  = \frac{{\mathrm{d}}^{w}}{\mathrm{\;d}{u}^{w}}\mathop{\sum }\limits_{{j = 0}}^{{{n}_{p} + k - 1}}{}^{m}{d}_{j}{N}_{j,k}\left( u\right)  = \mathop{\sum }\limits_{{j = {r}_{\text{ind }} - k + w}}^{{r}_{\text{ind }}}{}^{m}{d}_{j}^{w}{N}_{j,k - w}\left( u\right) ,u\]

\[ \in  \left\lbrack  {{u}_{{r}_{\text{ind }}},{u}_{{r}_{\text{ind }} + 1}}\right)  \subset  \left\lbrack  {{u}_{k},{u}_{{n}_{p} + k}}\right\rbrack   \tag{11}\]

where

\[{}^{m}{d}_{j}^{s} = \left\{  {\begin{matrix} {}^{m}{d}_{j},s = 0 \\  \frac{{\left( k - s + 1\right) }^{m}{d}_{j}^{s - 1} - {}^{m}{d}_{j - 1}^{s - 1}}{{u}_{j + k - s + 1} - {u}_{j}},s = 1,2,\ldots ,w \\  j = {r}_{ind} - k + s,{r}_{ind} - k + s + 1,\ldots ,{r}_{ind} \end{matrix},}\right.  \tag{12}\]

According to Eq. (11),the \( w \) -th order derivative of generated trajectories is always(k - w)-th degree B-splines. Correspondingly,the control points at both ends still coincide with the starting point and ending point of the trajectory due to the clamped B-spline curve generated above. The vectors of velocity, acceleration, and jerk could be represented with regard to \( w = 1,2 \) ,and 3,respectively. Therefore,the boundary conditions regarding the kinematic parameters can be derived as follows:

\[V\left( {t}_{0}\right)  = {\left. {}^{m}{P}^{1}\left( u\right) \right| }_{u = {u}_{k}} = \mathop{\sum }\limits_{{j = k - k + 1}}^{k}{}^{m}{d}_{j}^{1}{N}_{j,k - 1}\left( {u}_{k}\right)  = {}^{m}{d}_{1}^{1} \tag{13}\]

\[V\left( {t}_{{n}_{p}}\right)  = {\left. {}^{m}{p}^{1}\left( u\right) \right| }_{u = {u}_{{n}_{p} + k}} = \mathop{\sum }\limits_{{j = {n}_{p} + k - 1 - k + 1}}^{{{n}_{p} + k - 1}}{}^{m}{d}_{j}^{1}{N}_{j,k - 1}\left( {u}_{{n}_{p} + k}\right)  = {}^{m}{d}_{{n}_{p} + k - 1}^{1} \tag{14}\]

\[A\left( {t}_{0}\right)  = {\left. {}^{m}{p}^{2}\left( u\right) \right| }_{u = {u}_{k}} = \mathop{\sum }\limits_{{j = k - k + 2}}^{k}{}^{m}{d}_{j}^{2}{N}_{j,k - 1}\left( {u}_{k}\right)  = {}^{m}{d}_{2}^{2} \tag{15}\]

\[A\left( {t}_{{n}_{p}}\right)  = {\left. {}^{m}{p}^{2}\left( u\right) \right| }_{u = {u}_{{n}_{p} + k}} = \mathop{\sum }\limits_{{j = {n}_{p} + k - 1 - k + 2}}^{{{n}_{p} + k - 1}}{}^{m}{d}_{j}^{2}{N}_{j,k - 2}\left( {u}_{{n}_{p} + k}\right)  = {}^{m}{d}_{{n}_{p} + k - 1}^{2} \tag{16}\]

\[J\left( {t}_{0}\right)  = {\left. {}^{m}{p}^{3}\left( u\right) \right| }_{u = {u}_{k}} = \mathop{\sum }\limits_{{j = k - k + 3}}^{k}{}^{m}{d}_{j}^{3}{N}_{j,k - 3}\left( {u}_{k}\right)  = {}^{m}{d}_{3}^{3} \tag{17}\]

\[J\left( {t}_{{n}_{p}}\right)  = {\left. {}^{m}{p}^{3}\left( u\right) \right| }_{u = {u}_{{n}_{p} + k}} = \mathop{\sum }\limits_{{j = {n}_{p} + k - 1 - k + 3}}^{{{n}_{p} + k - 1}}{}^{m}{d}_{j}^{3}{N}_{j,k - 3}\left( {u}_{{n}_{p} + k}\right)  = {}^{m}{d}_{{n}_{p} + k - 1}^{3} \tag{18}\]

where \( V\left( {t}_{0}\right) ,V\left( {t}_{{n}_{p}}\right) ,A\left( {t}_{0}\right) ,A\left( {t}_{{n}_{p}}\right) ,J\left( {t}_{0}\right) \) and \( J\left( {t}_{{n}_{p}}\right) \) represent the velocity \( {v}_{m}\left( t\right) \) ,acceleration \( {a}_{m}\left( t\right) \) and jerk \( {j}_{m}\left( t\right) \) of the joint actuator at its start and end times \( \left( {t}_{0}\right. \) and \( \left. {t}_{{n}_{p}}\right) \) in joint space,respectively; \( {}^{m}{d}_{1}^{1},{}^{m}{d}_{{n}_{p} + k - 1}^{1},{}^{m}{d}_{1}^{2} \) , \( {}^{m}{d}_{{n}_{p} + k - 1}^{2},{}^{m}{d}_{1}^{3} \) and \( {}^{m}{d}_{{n}_{p} + k - 1}^{3} \) are the two endpoints of the control points of the corresponding trajectory, respectively, as determined by the recursion formula Eq. (12).

After preparing the aforementioned \( \left( {{n}_{p} + k}\right) \) equations,the control points \( {d}_{j}\left( {j = 0,1,\ldots ,{n}_{p} + k - 1}\right) \) could be solved. Then,the NURBS curve crossing through via-points in joint space is able to be generated according to the control points, the normalized knot vector, and the degree of B-spline curves. Note that the trajectories generated by B-splines with \( k = 7 \) could regard Eqs. (13)-(18) as additional kinematic constraints. The above kinematic parameters at the initial and end moments could be defined flexibly according to the requirements. If only the parameters of velocity and acceleration are the concerns, the 5-th degree B-splines could be utilized to interpolate via-points by employing Eqs. (13)-(16) as the boundary conditions. Similarly, for the case of \( k = 3 \) ,the boundary conditions Eqs. (13)-(14) can be used to create B-spline curves from a set of given points. Subsequently, the aforementioned objective function values used for evolutionary processes will be calculated on the basis of the design variables.

### 3.2. Kinematic constraint transformation

A significant benefit of the B-spline curve is that it constantly resides within the convex hull of the control points that modify it locally. As the degree decreases, the control polyline follows the modified B-spline curve more closely. Therefore, for ease of computation, the constraints in Eq. (5) could be adjusted as follows:

\[\max \left( \left| {{}^{m}{d}_{j}^{1}}\right| \right)  \leq  {k}_{v}{}^{m}{V}_{\max },j = 1,2,\ldots ,{n}_{p} + k - 1 \tag{19}\]

\[\max \left( \left| {{}^{m}{d}_{j}^{2}}\right| \right)  \leq  {k}_{a}{}^{m}{A}_{\max },j = 2,3,\ldots ,{n}_{p} + k - 1 \tag{20}\]

\[\max \left( \left| {{}^{m}{d}_{j}^{3}}\right| \right)  \leq  {k}_{j}{}^{m}{J}_{\max },j = 3,4,\ldots ,{n}_{p} + k - 1 \tag{21}\]

where \( {k}_{v},{k}_{a} \) ,and \( {k}_{j} \) are the coefficients to facilitate the adjustment of the constraints. The above constraints only apply to a limited number of control points, which considerably reduces the computing complexity of the task.

Eqs. (19)-(21) could be redefined as

\[{}^{m}{g}_{v}\left( \mathbf{h}\right)  = \max \left( \left| {{}^{m}{d}_{j}^{1}}\right| \right)  - {k}_{v}{}^{m}{V}_{\max } \leq  0,j = 1,2,\ldots ,{n}_{p} + k - 1 \tag{22}\]

\[{}^{m}{g}_{a}\left( \mathbf{h}\right)  = \max \left( \left| {{}^{m}{d}_{j}^{2}}\right| \right)  - {k}_{a}{}^{m}{A}_{\max } \leq  0,j = 2,3,\ldots ,{n}_{p} + k - 1 \tag{23}\]

\[{}^{m}{g}_{j}\left( \mathbf{h}\right)  = \max \left( \left| {{}^{m}{d}_{j}^{3}}\right| \right)  - {k}_{j}{}^{m}{J}_{\max } \leq  0,j = 3,4,\ldots ,{n}_{p} + k - 1 \tag{24}\]

where \( \mathbf{h} = \left\{  {{h}_{1},{h}_{2},\ldots ,{h}_{{n}_{p}}}\right\} \) . Thus,the constraint violations can be utilized to determine the direction of evolution from infeasible to feasible regions of a search space.

### 3.3. Infeasible-updating non-dominated sorting

The standard NSGA-II non-dominated sorting process could assign two attributes to all individuals involved in the evolution: nondomination levels and crowding distance. They could be used to select the best solutions for the next parent population \( {P}_{t + 1}^{\text{orig }} \) of size \( N \) from the population of size \( {2N} \) by merging both parent \( {P}_{t} \) and offspring \( {Q}_{t} \) . This process can be separated into three stages: no feasible solution, at least one feasible solution, and all feasible solutions. Especially for cases with infeasible solutions, the greedy nature of classical feasibility rules [28] is not conducive to fully exploring optimal solutions within a limited number of evolutions due to complex feasible regions [39]. Therefore, the improved feasibility rule considers information about the multi-objective function values for infeasible solutions.

<!-- Meanless: 4-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

<!-- Media -->

<!-- figureText: Start \( {n}_{FE} = 0 \) \( {2N} \) individuals Feasible solutions Infeasible solutions Case 1: All individuals are infeasible Q Execute Algorithm 1 Case 2: Feasible individuals less than \( N \) \( N \) individuals (parent population for the next iteratio Q3 Execute Algorithm 1 Case 3: Feasible individuals exceed \( N \) Rejected Nondominated sorting Crowding distance sorting Initialize population of size \( N \) and calculate fitness values Execute neighbor pairing strategy, differential evolution, and polynomial mutation Generate \( N \) individuals as offspring \( {Q}_{t} \) Perform B-spline interpolation; Generate \( N \) individuals as calculate objective functions and constraint violations Combine parent \( {P}_{t} \) and offspring \( {Q}_{t} \) of size \( {2N} \) Execute infeasible-updating nondominated sorting \( {n}_{FE} = {n}_{FE} + N \) \( {n}_{FE} \leq  {N}_{MaxFE} \) No 1 End -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_4.jpg?x=291&y=148&w=1170&h=791&r=0"/>

Fig. 1. Complete procedure of the proposed trajectory planning method.

<!-- figureText: Revolute Joint 2 Revolute Joint 3 Revolute Joint 5, 6 and 7 (yaw, pitch, and roll) Prismatic Joint 1 Prismatic Joint 4 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_4.jpg?x=229&y=1010&w=1291&h=534&r=0"/>

Fig. 2. Segment assembly robot of the shield machine.

<!-- Media -->

According to Eqs. (22)-(24), the overall constraint violation degree adopted in this research is defined as

\[C\left( \mathbf{h}\right)  = \mathop{\sum }\limits_{{m = 1}}^{M}\max \left\{  {0,{}^{m}{g}_{v}\left( \mathbf{h}\right) }\right\}   + \mathop{\sum }\limits_{{m = 1}}^{M}\max \left\{  {0,{}^{m}{g}_{a}\left( \mathbf{h}\right) }\right\}   + \mathop{\sum }\limits_{{m = 1}}^{M}\max \left\{  {0,{}^{m}{g}_{j}\left( \mathbf{h}\right) }\right\}  \]

(25)

If \( C\left( \mathbf{h}\right)  = 0 \) ,then a collection of design variables is feasible,while \( C\left( \mathbf{h}\right)  < 0 \) denotes infeasible solutions. Therefore,feasible and infeasible domains are established in the decision space.

The replacement mechanism in [39] consists mainly of two steps: the selection of individuals to be replaced and the establishment of a predefined archive. In this study,the improvement is based on \( {N}_{\text{infea }} \) infeasible solutions in the original parent population \( {P}_{t + 1}^{\text{orig }} \) of size \( N \) . To prevent trapping in a local domain, the infeasible individuals in the original parent population are resorted according to Pareto dominance and divided into non-dominated levels \( {R}_{m} \) in descending order of magnitude. It means the individuals in each rank are of the same level according to the objective function information. The individual \( {\mathbf{h}}_{iR} \) with the maximum constraint violation will thereafter be available for replacement in the first rank. Note that the previous procedure just picks the individuals that need to be replaced. The feasibility rule is still followed for the selection of parent populations, and only the prepared individuals will be substituted in their original positions. This implements adaptive updates for the parent population, preserving the original non-domination level of prepared individuals while providing new individuals for subsequent offspring generation procedures.

The predefined archive \( {P}_{t + 1}^{\text{arch }} \) of size \( N \) is determined by the individuals that cannot survive in the next parent population, which is continually updated as each generation evolves. And this archive is then sorted according to the constraint violation in ascending order. Each individual \( {\mathbf{h}}_{u} \) in this order will be compared with the unique candidate of the first rank according to Pareto dominance one by one. If the former could dominate the latter, the individual in the archive could be selected for replacement and subsequently removed; otherwise, the replacement operation is not carried out. This procedure will terminate if a candidate is successfully replaced or \( N \) individuals in the archive are traversed.

<!-- Meanless: 5-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

The above steps are repeated for the individual in the second rank with the greatest degree of constraint violation until the divided ranks have been updated. In other words, the total divided ranks of the infeasible individuals in the original parent population determine the maximum number of replacements. Following that, the new parent population \( {P}_{t + 1} \) could be collected for further use. The implementation of the above process is simplified in Algorithm 1. The computational complexity of lines 1-4 is obvious to be \( O\left( {{N}_{M}{N}^{2}}\right) \) [28],where \( {N}_{M} \) denotes the number of objectives. The computational complexity of the sort process of lines 6 and 7 is \( O\left( {{N}_{M}{N}_{\text{infea }}{}^{2}}\right) \) and \( O\left( {N\log N}\right) \) ,respectively. Given that an individual in the predefined archive may be traversed \( N \) times, the worst complexity of the update operation of lines 8-20 is \( O\left( {{N}_{M}{R}_{m}N}\right) \) . As a result,the computational complexity of algorithm 1 in one generation is \( O\left( {N}^{2}\right) \) .

Algorithm 1. Procedure of infeasible-updating non-dominated sorting.

<!-- Media -->

---

Input: A combined population of size \( {2N} \) from parent \( {P}_{t} \) and offspring \( {Q}_{t} \)
Output: A new parent population \( {P}_{t + 1} \) of size \( N \) for the next iteration
Sort the whole population of size \( {2N} \) by nondomination;
Calculate the crowding distance for each individual;
Select the populations as the original parent population \( {P}_{t + 1}^{\text{orig }} \) of size \( N \) and
the predefined archive \( {P}_{t + 1}^{\text{arch }} \) of size \( N \) ,respectively;
Determine the number of infeasible solutions \( {N}_{\text{infea }} \) in the original parent
population;
If \( {N}_{\text{infea }} \neq  0 \)
		Sort \( {N}_{\text{infea }} \) infeasible solutions in descending order based on the non-
domination levels to get the total number of ranks \( {R}_{m} \) ;
		Sort \( N \) individuals in the predefined archive \( {P}_{t + 1}^{\text{arch }} \) with the constraint
violation in ascending order;
		For \( {i}_{R} = 1 : {R}_{m} \)
			Select the individual \( {\mathbf{h}}_{iR} \) with the maximum degree of constraint
violation from the \( {i}_{R} \) -th rank of \( {N}_{\text{infea }} \) infeasible solutions;
			Initialize the number of individuals in the archive \( N \) ;
			\( u = 1;\;{l}_{\text{logic }} = 1 \)
			While \( u \neq  N + 1 \) and \( {l}_{\text{logic }} = 0 \)
				Select the individual \( {\mathbf{h}}_{u} \) with the minimum degree of constraint
violation in the archive in sequence;
					If \( {f}_{1}\left( {\mathbf{h}}_{u}\right)  < {f}_{1}\left( {\mathbf{h}}_{{i}_{R}}\right) ,{f}_{2}\left( {\mathbf{h}}_{u}\right)  < {f}_{2}\left( {\mathbf{h}}_{{i}_{R}}\right) \) ,and \( {f}_{3}\left( {\mathbf{h}}_{u}\right)  < {f}_{3}\left( {\mathbf{h}}_{{i}_{R}}\right) \)
					\( {l}_{\text{logic }} = 1 \)
					Else
					\( {l}_{\text{logic }} = 0 \)
					End If
					\( u = u + 1 \)
			End While
			If \( {l}_{\text{logic }} = 1 \)
				Replace \( {\mathbf{h}}_{iR} \) with \( {\mathbf{h}}_{u} \) and remove \( {\mathbf{h}}_{u} \) from the archive;
			End If
		End For
End If
Return \( {P}_{t + 1} \) .

---

<!-- Meanless: 6-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

<!-- figureText: Revolute Joint 2 Segment 5 Segment 6 Segment 4 Segment Assembly Robot Segment 1 Segment 3 Segment 2 (b) Revolute Joint 5 Prismatic Revolute \( {\overset{ \sim  }{Z}}_{1}\left( {\overset{ \sim  }{Z}}_{2}\right) \) Joint 1 Joint 7, \( {\gamma }_{7} \sim \) Joint 3 Prismatic \( {Z}_{7}^{\prime } \) Revolute Joint 4 Joint 6 (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_6.jpg?x=221&y=149&w=1304&h=570&r=0"/>

Fig. 3. D-H link coordinate of segment assembly robot (a) and simplified transverse section of a segment ring (b).

Table 1

\( \mathrm{D} - \mathrm{H} \) parameters of the segment assembly robot.

<table><tr><td>Joint No. \( m \)</td><td>\( {\alpha }_{m - 1}/\deg \)</td><td>\( {a}_{m - 1}/\mathrm{{mm}} \)</td><td>\( {d}_{m}/\mathrm{{mm}} \)</td><td>\( {\theta }_{i} \)</td></tr><tr><td>1</td><td>0</td><td>0</td><td>\( {d}_{1} \)</td><td>0</td></tr><tr><td>2</td><td>0</td><td>0</td><td>0</td><td>\( {\theta }_{2} \)</td></tr><tr><td>3</td><td>0</td><td>\( {L}_{OA} \)</td><td>0</td><td>\( {\theta }_{3} \)</td></tr><tr><td>4</td><td>\( {90}^{ \circ  } \)</td><td>0</td><td>\( {d}_{4} \)</td><td>0</td></tr><tr><td>5</td><td>\( {270}^{ \circ  } \)</td><td>0</td><td>0</td><td>\( {\theta }_{5} \)</td></tr><tr><td>6</td><td>\( {90}^{ \circ  } \)</td><td>\( {L}_{BC} \)</td><td>\( {L}_{CD} \)</td><td>\( {\theta }_{6} \)</td></tr><tr><td>7</td><td>\( {90}^{ \circ  } \)</td><td>0</td><td>\( {L}_{DE} \)</td><td>\( {\theta }_{7} \)</td></tr></table>

<!-- Media -->

### 3.4. Complete procedure of the proposed method

The evolutionary algorithm with the infeasible-updating non-dominated sorting strategy is further employed to do multi-objective trajectory planning for shield machine segment assembly robots. For each evolutionary stage, the population can be divided into independent subareas. As a consequence, all of the objective function values can be computed in parallel. Each individual in the population represents the time information of the hydraulic joint via-points. The boundary conditions are defined before initialization as

\[\left\{  {\begin{array}{l} {h}_{i + 1}^{\min } = \mathop{\max }\limits_{{m = 1,2,\ldots ,M}}\left\lbrack  \frac{{}^{m}{p}_{i + 1} - {}^{m}{p}_{i}}{{}^{m}{V}_{\max }}\right\rbrack  \\  {h}_{i + 1}^{\max } = {h}_{i + 1}^{\min } + {t}_{\text{span }} \end{array}\;i = 0,1,\ldots ,{n}_{p} - 1}\right.  \tag{26}\]

where the maximum time interval \( {t}_{\text{span }} \) is set by the consideration of the movement time of each hydraulic joint. Accordingly, the population of size \( N \) could be initialized,and the normalized knot vector in Eq. (7) can be constructed. At this point, the objective function values and constraint violations should be calculated to determine the fitness values of each individual, which will be utilized to generate offspring.

<!-- Media -->

Table 3

Kinematic constraints of each joint for two stages of the assembly procedure.

<table><tr><td>Cycle stage</td><td>Joint No. \( m \)</td><td>\( {}^{m}{V}_{\max } \) (mm/s, deg/s)</td><td>\( {}^{m}{A}_{\max }\left( {\mathrm{{mm}}/{\mathrm{s}}^{2},}\right. \) \( \deg /{\mathrm{s}}^{2} \) )</td><td>\( {}^{m}{J}_{\max }\left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right. \) , \( \deg /{\mathrm{s}}^{3} \) )</td></tr><tr><td rowspan="6">I</td><td>1</td><td>110</td><td>60</td><td>50</td></tr><tr><td>2</td><td>20</td><td>10</td><td>6</td></tr><tr><td>3</td><td>70</td><td>45</td><td>30</td></tr><tr><td>5</td><td>100</td><td>70</td><td>60</td></tr><tr><td>6</td><td>3</td><td>2</td><td>2</td></tr><tr><td>7</td><td>5</td><td>5</td><td>2</td></tr><tr><td rowspan="3">II</td><td>3</td><td>55</td><td>30</td><td>25</td></tr><tr><td>4</td><td>65</td><td>40</td><td>35</td></tr><tr><td>5</td><td>45</td><td>35</td><td>20</td></tr></table>

<!-- Media -->

To make the algorithm work even better, offspring populations are made using the neighbor pairing strategy \( \left\lbrack  {{44},{45}}\right\rbrack \) ,differential evolution [46], and polynomial mutation [47]. On this MOTP problem, this combination outperformed traditional genetic procedures such as tournament selection [48], simulated binary crossover [49], and polynomial mutation in many tests. Sect. 4.5 goes through the results in detail. If the aforementioned update happens, the genetic information utilized for the subsequent operators will be modified. The parent population is then used to create a mating pool. Following that, a differential evolution operator and polynomial mutation can be used to construct an offspring population of size \( N \) . Combining the parent and offspring populations of the current generation creates a population of size \( {2N} \) in order to choose the best solutions for the following generation. By repeating the above steps, the parent population of the next generation will be utilized to produce new offspring.

<!-- Media -->

Table 2

Joint via-points in the first and second stages of assembly for segment 1 .

<table><tr><td>Cycle stage</td><td>Via-points</td><td>Joint 1 (mm)</td><td>Joint 2 (deg)</td><td>Joint 3 (mm)</td><td>Joint 4 (mm)</td><td>Joint 5 (mm)</td><td>Joint 6 (mm)</td><td>Joint 7 (mm)</td></tr><tr><td rowspan="8">I</td><td>1</td><td>1447.790</td><td>-7.227</td><td>241.343</td><td>2.400</td><td>256.621</td><td>23.098</td><td>2.123</td></tr><tr><td>2</td><td>1261.043</td><td>-12.146</td><td>155.850</td><td>2.400</td><td>191.053</td><td>23.124</td><td>1.892</td></tr><tr><td>3</td><td>1052.746</td><td>-21.974</td><td>88.202</td><td>2.400</td><td>215.252</td><td>23.265</td><td>1.675</td></tr><tr><td>4</td><td>849.857</td><td>-28.468</td><td>26.873</td><td>2.400</td><td>239.397</td><td>23.341</td><td>1.352</td></tr><tr><td>5</td><td>668.448</td><td>-36.818</td><td>8.619</td><td>2.400</td><td>249.802</td><td>23.654</td><td>0.897</td></tr><tr><td>6</td><td>553.458</td><td>-54.736</td><td>53.824</td><td>2.400</td><td>285.070</td><td>23.785</td><td>0.543</td></tr><tr><td>7</td><td>314.404</td><td>-89.199</td><td>147.702</td><td>2.400</td><td>325.439</td><td>23.921</td><td>0.251</td></tr><tr><td>8</td><td>100.000</td><td>-122.124</td><td>257.479</td><td>2.400</td><td>368.862</td><td>24.052</td><td>0.000</td></tr><tr><td>II</td><td>9</td><td>100.000</td><td>-122.124</td><td>257.479</td><td>2.400</td><td>368.862</td><td>24.052</td><td>0.000</td></tr><tr><td/><td>10</td><td>100.000</td><td>-122.124</td><td>441.931</td><td>33.637</td><td>406.766</td><td>24.052</td><td>0.000</td></tr><tr><td/><td>11</td><td>100.000</td><td>-122.124</td><td>583.019</td><td>73.422</td><td>413.937</td><td>24.052</td><td>0.000</td></tr><tr><td/><td>12</td><td>100.000</td><td>-122.124</td><td>723.186</td><td>66.558</td><td>420.451</td><td>24.052</td><td>0.000</td></tr><tr><td/><td>13</td><td>100.000</td><td>-122.124</td><td>889.881</td><td>59.034</td><td>428.615</td><td>24.052</td><td>0.000</td></tr><tr><td/><td>14</td><td>100.000</td><td>-122.124</td><td>984.410</td><td>101.078</td><td>435.280</td><td>24.052</td><td>0.000</td></tr><tr><td/><td>15</td><td>100.000</td><td>-122.124</td><td>1083.467</td><td>136.991</td><td>391.343</td><td>24.052</td><td>0.000</td></tr><tr><td/><td>16</td><td>100.000</td><td>-122.124</td><td>1178.183</td><td>178.392</td><td>373.336</td><td>24.052</td><td>0.000</td></tr></table>

<!-- Meanless: 7-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

Table 4

Initialization parameters of the MOTP problem.

<table><tr><td>Stage</td><td>\( {t}_{\text{span }}\left( \mathrm{s}\right) \)</td><td>\( {k}_{v} \)</td><td>\( {k}_{a} \)</td><td>\( {k}_{j} \)</td><td>\( {N}_{M} \)</td><td>\( {N}_{D} \)</td><td>\( {n}_{g} \)</td></tr><tr><td>I</td><td>6</td><td>1.2</td><td>1.2</td><td>1.2</td><td>3</td><td>7</td><td>18</td></tr><tr><td>II</td><td>6</td><td>1.2</td><td>1.2</td><td>1.2</td><td>3</td><td>7</td><td>9</td></tr></table>

Table 5

Calculated nadir and ideal points in two stages of the MOTP problem.

<table><tr><td>Stage</td><td>\( {f}_{1}^{\text{nadir }} \)</td><td>\( {f}_{2}^{\text{nadir }} \)</td><td>\( {f}_{3}^{\text{nadir }} \)</td><td>\( {f}_{1}^{\text{ideal }} \)</td><td>\( {f}_{2}^{ideal} \)</td><td>\( {f}_{3}^{\text{ideal }} \)</td></tr><tr><td>I</td><td>36.485</td><td>29.792</td><td>29.249</td><td>22.271</td><td>3.790</td><td>3.344</td></tr><tr><td>II</td><td>42.945</td><td>10.722</td><td>9.589</td><td>27.744</td><td>2.961</td><td>2.129</td></tr></table>

<!-- Media -->

The parameter of non-dominated levels is available for infeasible solutions based on the feasibility rule [28]. It assures that the evolutionary tendency is always from infeasible to feasible, and then from feasible to optimal. Consequently,the solutions of size \( N \) are selected as the original parent population based on the elite-preserving operator. The suggested updating strategy for non-dominated sorting in Sect. 3.3 is used to make the best solutions more up-to-date and create a new parent population. Before deciding whether to utilize this improved procedure, it is necessary to first count the infeasible solutions of the original parent population. Note that the update operation will terminate when all of the individuals in the parent population are feasible. The entire procedure will not be finished until the maximum number of function evaluations \( {N}_{\text{MaxFE }} \) is achieved. As previously stated,Fig. 1 depicts the processing of the proposed trajectory planning method. In the figure, solutions highlighted in green are feasible, where lighter shades indicate higher non-domination levels among these solutions. For infeasible solutions, the lighter shades of color red suggest they have smaller constraint violations.

## 4. Results and analysis

To demonstrate the effectiveness of the proposed MOTP method, the proposed INSEA and a number of advanced algorithms are used for comparison in two stages of the assembly operation. Furthermore, ablation experiments are given to illustrate the efficacy of infeasible-updating non-dominated sorting. The interpolation performances of NURBS curves and cubic spline curves are then analyzed by adopting the proposed INSEA.

### 4.1. Problem description and initialization

The path data acquired from a platform of segment assembly robots instrumented at Shanghai Tunnel Engineering Ltd. was used to validate the effectiveness of the proposed method. As depicted in Fig. 2, the robot has seven degrees of freedom, allowing it to perform complicated motions that are more dexterous than the typical configuration. Due to a large workspace, it is appropriate that seven degrees of freedom could be adequately implemented and controlled, especially for several segments. However, it is difficult to directly calculate inverse kinematics for this robot. An effective way for locking joints 2 or 4 could be adopted. Then the joint parameters could be derived by using kinematics equations that provide a desired position and rotation for the robot end-effector. The D-H link coordinate of its 7-DOF serial structure is highlighted in Fig. 3(a), and the D-H parameters are listed in Table 1. Fig. 3 (b) represents a transverse section of all the segments. As shown, every ring has six segments during construction.

In a global static coordinate system, the positions and rotation matrices of segments via predefined path points were determined. For segment 1, taken as a reference, inverse kinematics provided the variable joint parameters to ensure that the position and orientation of the segment could satisfy the requirements. After grasping the segment, the assembly procedure comprises two main stages, and each requires the independent execution of multi-objective trajectory planning. The initial procedure is to move the segment to the desired position and prepare it for further installation by locking joint 4. Subsequently, after locking joint 2, a more precise adjustment is performed to erect the segment to the final location. Table 2 presents the actuator angles and strokes, calculated using inverse kinematics for these two stages, to facilitate smooth trajectory construction via spline fitting. The results are then delivered to the robot controller and set as the reference command, subject to the kinematic limitations in Table 3. Note that the second joint is driven by a rotary actuator, while the others are powered by linear actuators.

Eq. (26) calculates the upper and lower limits of decision variables by specifying an appropriate time interval \( {t}_{\text{span }} \) . Because of how B-spline curves are made, many tests are done to find the coefficients of the constraints in Eqs. (19)-(21), making sure that the kinematic parameters of the fitted trajectories are in the range shown in Table 3. As a result, this problem can be initialized, and the essential settings are provided in Table 4. The number of decision variables and inequality constraints is represented by \( {N}_{D} \) and \( {n}_{g} \) ,respectively.

### 4.2. Advanced algorithms and parameter settings

For the two stages of the task, the multi-objective trajectory planning of segment assembly robots is implemented, respectively. Multi-objective evolutionary algorithms can be used to find optimal tradeoff solutions. In addition to the proposed INSEA, six advanced CMOP-solving algorithms are used to address this problem for comparison. They are NSGA-II [28], CCMO [50], TSTI [51], TiGE-2 [52], C3M [53], and ICMA [45], and they perform well on synthetic benchmark problems.

<!-- Media -->

Table 6

Grid-IGD values acquired by INSEA, NSGA-II, CCMO, TSTI, TiGE-2, C3M, and ICMA on MOTP problems.

<table><tr><td>Stage</td><td>INSEA</td><td>NSGA-II</td><td>CCMO</td><td>TSTI</td><td>TiGE-2</td><td>C3M</td><td>ICMA</td></tr><tr><td>I</td><td>0.2559 (0.0184)</td><td>0.4725 (0.0536) -</td><td>0.5277 (0.0726) -</td><td>0.5044 (0.0570) -</td><td>3.8291 (1.5505) -</td><td>0.4597 (0.0450) -</td><td>0.2848 (0.0177) -</td></tr><tr><td>II</td><td>0.1301 (0.0060)</td><td>0.2036 (0.0210) -</td><td>0.3035 (0.0480) -</td><td>0.2213 (0.0226) -</td><td>0.9528 (0.2689) -</td><td>0.2788 (0.0375) -</td><td>0.1391 (0.0078) -</td></tr></table>

Table 7

HV values acquired by INSEA, NSGA-II, CCMO, TSTI, TiGE-2, C3M, and ICMA on MOTP problems.

<table><tr><td>Stage</td><td>INSEA</td><td>NSGA-II</td><td>CCMO</td><td>TSTI</td><td>TiGE-2</td><td>\( \mathrm{C}3\mathrm{M} \)</td><td>ICMA</td></tr><tr><td>I</td><td>0.8257 (0.0029)</td><td>0.8036 (0.0054) -</td><td>0.7980 (0.0068) -</td><td>0.7998 (0.0067) -</td><td>0.5285 (0.0993) -</td><td>\( {0.8020}\left( {0.0048}\right)  - \)</td><td>0.8213 (0.0025) -</td></tr><tr><td>II</td><td>0.7437 (0.0008)</td><td>0.7322 (0.0037) -</td><td>0.7218 (0.0043) -</td><td>0.7314 (0.0027) -</td><td>0.6317 (0.0229) -</td><td>0.7272 (0.0033) -</td><td>0.7426 (0.0013) -</td></tr></table>

<!-- Meanless: 8-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

<!-- figureText: INSEA in Stage I NSGA-II in Stage I CCMO in Stage I 30 Feasible solutions 25 Approximate Pareto fron 20 RMS Jerk 15 10 5,20 10 20 25 5 15 20 RMS Acceleration RMS Acceleration (c) TiGE-2 in Stage I C3M in Stage I Feasible solutions 25 Approximate Pareto front RMS Jerk 15 0 20 25 30 15 20 30 RMS Acceleration RMS Acceleration (f) ICMA in Stage I 25 30 RMS Acceleration 30 30 Feasible solutions Feasible solutions Approximate Pareto front Approximate Pareto front 20 20 RMS Jerk 15 RMS Jerk 15 10 10 20 5 15 20 25 30 RMS Acceleration (a) (b) TSTI in Stage I 30 30 Feasible solutions Feasible solutions 25 Approximate Pareto front 25 Approximate Pareto front RMS Jerk 15 RMS Jerk 15 10 10 C. 20 15 20 30 5 RMS Acceleration (d) (e) 30 Feasible solutions 25 Approximate Pareto front 20 10 5 (g) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_8.jpg?x=112&y=146&w=1526&h=1368&r=0"/>

Fig. 4. Pareto optimal solutions with median Grid-IGD value among 30 runs in stage I: (a) INSEA; (b) NSGA-II; (c) CCMO; (d) TSTI; (e) TiGE-2; (f) C3M; (g) ICMA.

<!-- Media -->

Specifically, NSGA-II emphasizes a fast non-dominated sorting approach that can give a good distribution and convergence of solutions when combined with a conventional constraint-handling strategy. However, the challenge of handling objective functions and constraints still exists when facing complex or small feasible regions. In the case of limited feasible domains, CCMO is one of the sophisticated coevolutionary algorithms that can effectively balance objective functions and constraints for CMOPs. Using the same optimizer, such as NSGA-II, the algorithm creates a parallel problem with weak fitness to generate new offspring to supplement the original. In contrast to dominance-based algorithms, indicator-based algorithms allow for a more flexible determination of the fitness value, ensuring that better solutions emerge in the next generation. To evaluate the contribution of each solution, ICMA employed a cost-value-based distance and constraints to define an indicator. The population can be guided to explore prospective areas evenly, and individual diversity could be further improved. During evolution, the process is frequently separated into several stages to handle complicated problems caused by infeasible solutions. As a result, indicators for convergence, diversity, and feasibility can be developed and assigned to work on associated stages, such as in TSTI and C3M. \( \mathrm{{C3M}} \) is especially good at handling problems with small feasible areas because of multiple constraints. To address constrained many-objective optimization problems, TiGE defines three indicators to balance the three aspects above through multiple balancing schemes and ranking techniques. Therefore, this study uses these algorithms to compare the proposed INSEA on the MOTP problem.

The detailed codes of the existing algorithms are taken from PLA-TEMO [54], a MATLAB platform for multi- or many-objective optimization. All algorithms are implemented on MATLAB r2022a on a PC running Linux x86,two Intel Xeon ICX Platinum 8358 (2.6 GHz, 32 cores) CPUs, and 512 GB of RAM. NSGA-II, CCMO, TSTI, and TiGE-2 employ simulated binary crossover [49] and polynomial mutation [47], whereas INSEA, ICMA, and C3M use differential evolution [46] and polynomial mutation. INSEA and ICMA, in particular, use the neighbor pairing strategy \( \left\lbrack  {{44},{45}}\right\rbrack \) ,while the others use the binary tournament selection [48]. The operators with the best performance have been selected as the basis for comparison in this study. The crossover probability is set to 1 , and the distribution index in the simulated binary crossover is set to 20 . The parameters of CR and F are set to 1 and 0.5 in differential evolution, respectively. The probability for the polynomial mutation is set to \( 1/{n}_{p} \) ,and the distribution index is set to 20. The remaining parameters of the aforementioned algorithms are specified as suggested in their original papers. For CCMO, the size of the parent and offspring population for the parallel problem is half the total. The penalty factor used to adjust the preference of constraints in TSTI and TiGE-2 is set to 0.05 , and the scaling factor used to update the gradually increased penalty at each generation is set to 1.01 . The number of nearest neighbors for each individual in ISNEA and ICMA is set to 10 . To guarantee fairness, the population size is set to 200, and the maximum number of function evaluations is set to 100,000 .

<!-- Meanless: 9-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

<!-- Media -->

<!-- figureText: INSEA in Stage II NSGA-II in Stage II CCMO in Stage II Feasible solution: Approximate Pareto front 10 RMS Jerk 25 40 RMS 12 Acceleration (c) TiGE-2 in Stage II C3M in Stage II Feasible solutions Approximate Pareto front 了 8 RMS Jerk 2 40 45 Acceleration 10 12 45 (f) ICMA in Stage II 25 10 12 (g) Feasible solutions Feasible solution: Approximate Pareto front Approximate Pareto front 10 8 RMS Jerk 6 RMS Jerk 4 2 0 RMS 10 celeration (a) (b) TSTI in Stage II Feasible solutions Feasible solutions Approximate Pareto front Approximate Pareto front 8 RMS Jerk 4 RMS Jerk 2 0 RMS 40 RMS 12 (d) (e) Feasible solutions Approximate Pareto front 10 8 RMS Jerk 2 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_9.jpg?x=109&y=140&w=1540&h=1419&r=0"/>

Fig. 5. Pareto optimal solutions with median Grid-IGD value among 30 runs in stage II: (a) INSEA; (b) NSGA-II; (c) CCMO; (d) TSTI; (e) TiGE-2; (f) C3M; (g) ICMA.

<!-- Media -->

### 4.3. Performance metrics of algorithms

The inverted generational distance (IGD) [55] and hypervolume (HV) [56] are extensively used to assess the performance of multi-objective optimization algorithms. Both indicators are capable of evaluating the diversity and convergence of the optimal solutions.

A lower IGD value suggests the results are closer to the true Pareto front. However, for real-world problems, the true Pareto front is always unknown, making IGD computation difficult. A large number of independent trials using existing algorithms can be used to generate an approximate Pareto front of problems. Nonetheless, the specification of the reference set on Pareto front approximations strongly influences the IGD-based comparison findings. A more equal distribution is advantageous for ensuring the dependability of the results. Therefore, this study uses the Gird-IGD indicator developed in [57] to assess the overall quality of Pareto optimal solutions. The approximation front of problems in a grid environment can generate a collection of reference points based on the representative non-dominated solutions. Before that, the \( l \) -th objective is divided into \( K \) equal intervals within the range of approximations for the ideal point \( {f}_{l}^{\text{ideal }} \) and the extended nadir point \( {f}_{l}^{\text{enadir }} \) . The grid interval is calculated by

<!-- Meanless: 10-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

\[{d}_{l} = \frac{{f}_{l}^{\text{enadir }} - {f}_{l}^{\text{ideal }}}{K} \tag{27}\]

<!-- Media -->

<!-- figureText: 1.2 1.2 INSEA NSGA-II CCMO TSTI TiGE-2 C3M ICMA Constraint violation 0.8 0.6 0.4 0.1 0.2 0 0 2 4 10 Number of function evaluations \( \times  {10}^{4} \) (b) INSEA NSGA-II CCMO TSTI TiGE-2 C3M ICMA Constraint violation 0.8 0.6 0.4 1000 2000 0.2 0.1 0.2 0 2 8 10 Number of function evaluations \( \times  {10}^{4} \) (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_10.jpg?x=229&y=151&w=1288&h=541&r=0"/>

Fig. 6. Convergence profiles of constraint violations averaged over 30 runs: (a) Stage I; (b) Stage II.

<!-- figureText: 18 18 NSGA-II CCMO TSTI 16 TiGE-2 C3M ICMA 14 12 Grid-IGD 10 \( \times  {10}^{4} \) 0.4 0.3 0.2 0.1 4 8 Number of function evaluations \( \times  {10}^{4} \) (b) 16 TiGE-2 C3M ICMA 14 12 \( \times  {10}^{4} \) Grid-IGD 10 8 6 4 2 2 Number of function evaluations \( \times  {10}^{4} \) (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_10.jpg?x=229&y=770&w=1288&h=531&r=0"/>

Fig. 7. Convergence profiles of Grid-IGD values averaged over 30 runs: (a) Stage I; (b) Stage II.

<!-- Media -->

The extended nadir point is redefined as

\[{f}_{l}^{\text{enadir }} = {f}_{l}^{\text{nadir }} + \frac{{f}_{l}^{\text{nadir }} - {f}_{l}^{\text{ideal }}}{K}\left( {l = 1,2,\ldots ,{N}_{M}}\right)  \tag{28}\]

where \( {f}_{l}^{\text{nadir }} \) denotes the original nadir point; \( K \) is set to 100 .

Higher values for the HV indicator indicate that the algorithm-generated Pareto front is closer to the true Pareto front. In the normalized objective space,a reference vector of length \( {N}_{M} \) is assigned to \( {\left\lbrack  {1.1},{1.1},\ldots ,{1.1}\right\rbrack  }^{T} \) . The approximate ideal and nadir points are used to normalize the value of the \( l \) -th objective function,which can be obtained by

\[{f}_{l} = \frac{{f}_{l} - {f}_{l}^{\text{ideal }}}{{f}_{l}^{\text{nadir }} - {f}_{l}^{\text{ideal }}}\left( {l = 1,2,\ldots ,{N}_{M}}\right)  \tag{29}\]

The aforementioned algorithms are employed to generate the approximate Pareto front with enough points via 60 independent experiments for each stage of this problem. All reference points could be calculated according to the representative non-dominated solutions, which are then used to calculate the Grid-IGD in further experiments. The approximate ideal and nadir points can also be determined, as reported in Table 5.

### 4.4. Experimental results on MOTP problems

The MOPT in this work consists mainly of two stages that have to be completed independently. Tables 6 and 7 show the mean and standard deviation of the Grid-IGD and HV values produced by INSEA, NSGA-II, CCMO, TSTI, TiGE-2, C3M, and ICMA on the MOTP problem, respectively. Each algorithm is run 30 times for two stages of this problem. The detailed calculation results are provided as supplementary data in Tables A. 1-4. In this study, the Wilcoxon rank-sum test [58] with a significance level of 0.05 is utilized to evaluate the performance metrics of the experimental results, and the best result is highlighted. Meanwhile,the symbols " + , " " - ," and " \( \approx \) " indicate that the result is significantly better, significantly worse, and statistically similar to that obtained by INSEA, respectively.

The proposed INSEA outperforms the other advanced algorithms on this problem, particularly in stage I with more constraints, according to the results of significance tests in Tables 6 and 7. As the number of constraints reduces in stage II, the performance of ICMA gradually approaches that of INSEA but does not yet surpass it. In addition, the difficulty of solving such multi-objective optimization problems for TiGE-2 is relatively high. The algorithm performances can be easily distinguished via the Grid-IGD values. However, the difference between NSGA-II, CCMO, TSTI, and C3M is not significant based on the results of HV values.

<!-- Meanless: 11-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

<!-- Media -->

<!-- figureText: INSEA NSGA-II CCMO TSTI INSEA NSGA-II CCMO TSTI TiGE-2 C3M ICMA 0.8 0.6 0.74 0.4 0.72 0.2 \( \times  {10}^{4} \) 0 2 4 10 Number of function evaluations \( \times  {10}^{4} \) (b) TiGE-2 C3M ICMA 0.8 0.6 HV 0.4 0.82 0.2 9.8 \( \times  {10}^{4} \) 0 2 8 10 Number of function evaluations \( \times  {10}^{4} \) (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_11.jpg?x=227&y=154&w=1290&h=534&r=0"/>

Fig. 8. Convergence profiles of HV values averaged over 30 runs: (a) Stage I; (b) Stage II.

<!-- Media -->

Figs. 4 and 5 show the final Pareto solutions with median Grid-IGD obtained by the seven algorithms throughout stages I and II of this problem. In terms of diversity and convergence, the proposed INSEA outperforms the other algorithms on the MOTP problem. The approximate Pareto front in stage I is more complicated than in stage II due to the multiple constraints, resulting in an irregular characteristic in the objective space. As shown in Fig. 4, NSGA-II, CCMO, TSTI, C3M, and ICMA guide the populations into local optimization, which is improved in the second stage of the problem in Fig. 5. In this case, ICMA presents a relatively good performance for the regular regions of the Pareto front, according to the results in Table 6. This is because the defined indicator based on cost-value distances can lead the population to explore evenly as it develops closer to its origin. However, CCMO performs poorly, which can be attributable mostly to the partial offspring created by a helper problem. There are no specific guidelines for selecting the best solutions by balancing objectives and constraints. Instead, in the helper problem, all of the individuals only utilize objective function information as a basis for comparison, greatly restricting selection flexibility. For NSGA-II, the non-dominated sorting strategy combined with constraint handling technology always has a greedy characteristic when facing complex CMOPs. In the first stage of the problem, it is easy to fall into local areas and disregard some potential individuals. Although NSGA-II performs better than TSTI and C3M in the second stage from Table 6, the indicator-based algorithms still have the potential to improve population diversity, as shown in Fig. 5(d) and (f). That is due to their capacity to explore unknown areas in the early stages of evolution.

<!-- Media -->

Table 8

Time consumption of each algorithm in two stages of the MOTP problem.

<table><tr><td rowspan="2">Algorithm</td><td colspan="2">Stage I</td><td colspan="2">Stage II</td></tr><tr><td>Single runtime (s)</td><td>Total runtime (s)</td><td>Single runtime (s)</td><td>Total runtime (s)</td></tr><tr><td>INSEA</td><td>14.33</td><td>7165.89</td><td>14.35</td><td>7176.30</td></tr><tr><td>NSGA-II</td><td>14.36</td><td>7181.42</td><td>14.34</td><td>7172.35</td></tr><tr><td>CCMO</td><td>15.93</td><td>7965.13</td><td>16.32</td><td>8160.71</td></tr><tr><td>TSTI</td><td>14.63</td><td>7317.28</td><td>14.58</td><td>7291.37</td></tr><tr><td>TiGE-2</td><td>14.23</td><td>7116.27</td><td>14.34</td><td>7170.98</td></tr><tr><td>C3M</td><td>14.61</td><td>7306.89</td><td>14.62</td><td>7312.34</td></tr><tr><td>ICMA</td><td>14.29</td><td>7145.62</td><td>14.21</td><td>7106.88</td></tr></table>

Note: Single runtime and total runtime denote the average time consumed by parallel computing when the number of function evaluations is 200 and 100,000,respectively.

<!-- Media -->

To investigate the evolutionary process, the defined constraint violations in Eq. (25) are calculated by 30 runs for the seven algorithms. The average results at each generation during evolution are normalized for both stages of the problem, as illustrated in Fig. 6(a) and (b), respectively. The convergence rate of TiGE-2 is slow until the last generation does not fully reach the feasible domain. On the contrary, NSGA-II has the significant advantage of immediately guiding populations into feasible areas and remaining feasible for subsequent results. Nonetheless, it cannot guarantee excellent performance for the final results of this work. Therefore, effective balancing strategies, such as INSEA, ICMA, and C3M in stage I, as well as INSEA and ICMA in stage II, are essential for exploring unknown areas during the early period of evolution.

Figs. 7 and 8 show the convergence profiles of Grid-IGD and HV values averaged over 30 runs for INSEA, NSGA-II, CCMO, TSTI, TiGE-2, C3M, and ICMA. TSTI and C3M exhibit different convergence behaviors compared to INSEA, NSGA-II, CCMO, ICMA, and TiGE-2, related to the multi-stage mechanism proposed in the two algorithms. Although numerous infeasible solutions exist in the early stages of evolution, they could quickly converge to the feasible domain in the later stages. More feasible solutions with a wide range of diversity can be developed based on this mechanism. Nonetheless, the convergence of both algorithms has room for improvement when compared to INSEA, ICMA, and NSGA-II. In the early stages of evolution, the convergence rate of INSEA is slower than that of NSGA-II because some infeasible solutions have to be replaced. This opens up the possibility of evolutionary diversity toward the irregular part of the Pareto front, which is especially important for the first stage of the problem. According to Figs. 7 and 8, the Grid-IGD and HV values of INSEA maintain a steady and consistent advantage against the other state-of-the-art algorithms during the latter period. Hence, the proposed INSEA can achieve a good balance between convergence, diversity, and feasibility for the MOTP problem.

<!-- Media -->

Table 9

Mean and standard deviation of Grid-IGD and HV values obtained by INSEA, NSEA, INSEA2, NSEA2, INSEA3, NSEA3, INSEA4 and NSEA4 on MOTP problems.

<table><tr><td rowspan="2">NO.</td><td rowspan="2">Algorithm</td><td colspan="2">Grid-IGD</td><td colspan="2">HV</td></tr><tr><td>Stage I</td><td>Stage II</td><td>Stage I</td><td>Stage II</td></tr><tr><td rowspan="2">1</td><td>INSEA</td><td>0.2467 (0.0103)</td><td>0.1289 (0.0051)</td><td>0.8261 (0.0024)</td><td>0.7439 (0.0006)</td></tr><tr><td>NSEA</td><td>0.2575 (0.0128) -</td><td>0.1338 (0.0050) -</td><td>\( {0.8260}\left( {0.0018}\right)  \approx \)</td><td>0.7434 (0.0006) -</td></tr><tr><td rowspan="2">2</td><td>INSEA1</td><td>0.3105 (0.0119)</td><td>0.1506 (0.0064)</td><td>0.8189 (0.0019)</td><td>0.7401 (0.0009)</td></tr><tr><td>NSEA1</td><td>0.3245 (0.0127) -</td><td>0.1552 (0.0059) -</td><td>0.8172 (0.0018) -</td><td>0.7397 (0.0007) ≈</td></tr><tr><td rowspan="2">3</td><td>INSEA2</td><td>0.4628 (0.0429)</td><td>0.2051 (0.0200)</td><td>0.8029 (0.0038)</td><td>0.7337 (0.0020)</td></tr><tr><td>NSEA2</td><td>0.4818 (0.0614) ≈</td><td>0.2126 (0.0331) ≈</td><td>\( {0.8006}\left( {0.0059}\right)  \approx \)</td><td>\( {0.7330}\left( {0.0033}\right)  \approx \)</td></tr><tr><td rowspan="2">4</td><td>INSEA3</td><td>0.4532 (0.0467)</td><td>0.2020 (0.0195)</td><td>0.8054 (0.0044)</td><td>0.7326 (0.0015)</td></tr><tr><td>NSEA3</td><td>\( {0.4801}\left( {0.0600}\right)  \approx \)</td><td>0.2070 (0.0191) ≈</td><td>\( {0.8030}\left( {0.0065}\right)  \approx \)</td><td>0.7323 (0.0038) \( \approx \)</td></tr></table>

<!-- Meanless: 12-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

<!-- figureText: INSEA in Stage I 700 INSEA in Stage I 20 Approximate Pareto front Infeasible solutions in next generation 18 Feasible solutions in next generation Best solutions in feasible solutions 16 80 14 60 12 RMS Jerk 10 40 100 8 20 6 20 25 35 40 2 Execution Time (b) NSEA in Stage I 22 Approximate Pareto front 20 Infeasible solutions in next generation Feasible solutions in next generation 18 Best solutions in feasible solutions 16 14 12 RMS Jerk 40 10 20 100 8 30 2 Execution Time (d) Approximate Pareto front Infeasible solutions in next generation 600 Feasible solutions in next generation Best solutions in feasible solutions 500 80 60 RMS Jerk 40 300 100 20 200 100 20 25 30 35 40 45 Execution Time (a) NSEA in Stage I Approximate Pareto front 500 Infeasible solutions in next generation Feasible solutions in next generation Best solutions in feasible solutions 400 60 300 RMS Jerk 40 20 100 200 0 50 100 20 25 30 35 Execution Time 45 (c) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_12.jpg?x=219&y=147&w=1315&h=1137&r=0"/>

Fig. 9. Fourth-generation results with median Grid-IGD in stage I: (a) and (b) INSEA; (c) and (d) NSEA.

<!-- Media -->

To compare time costs, the single runtime with function evaluations of 200 times and the total runtime with function evaluations of 100,000 times are averaged across 30 runs for the two stages of the problem, as indicated in Table 8. The time consumed by the proposed INSEA is not significantly different in order of magnitude from other algorithms. During evolution, the objective function values of the population are calculated in parallel on the MATLAB platform. In this procedure, the inverse operation of the matrix is required to solve the control points of the fitted curve, which is usually time-consuming for such problems. Therefore, in parallel computing, the time consumed has a direct impact on the runtime. This is also why decomposition-based algorithms have yet to be used to handle such problems in this work.

### 4.5. Effectiveness of infeasible-updating non-dominated sorting

In this section, the ablation experiments are used to illustrate the effectiveness of the proposed INSEA. The main variant is the infeasible-updating non-dominated sorting. Therefore, for comparison, the standard non-dominated sorting process of NSGA-II is adopted. To guarantee fairness, offspring populations are generated using the neighbor pairing strategy \( \left\lbrack  {{44},{45}}\right\rbrack \) ,differential evolution \( \left\lbrack  {46}\right\rbrack \) ,and polynomial mutation [47]. In this study, this is referred to as the non-dominated sorting evolutionary algorithm (NSEA), which only modifies the genetic operations compared to NSGA-II. Following that, we set up another three sets of ablation trials with different combinations of genetic procedures. Specifically, tournament selection [48], differential evolution, and polynomial mutation are utilized for the first set. The second set adopts the neighbor pairing strategy, simulated binary crossover [49], and polynomial mutation for generating offspring. In the third set, the same genetic methods are used as in the normal NSGA-II. These are tournament selection, simulated binary crossover, and polynomial mutation. All algorithms are independently conducted 15 times with the same initial population for each set of ablation trials. Table 9 shows the mean and standard deviation of the Grid-IGD and HV values for the MOTP problem. The best result in each group is highlighted, and the symbols "+," "-," and " \( \approx \) " indicate that the result achieved is significantly better, significantly worse, or statistically similar to INSEA, INSEA1, INSEA2, and INSEA3, respectively.

As seen in Table 9, INSEA, INSEA1, INSEA2, and INSEA3 have the advantage of average values in both stages of the problem. However, it is difficult to distinguish between differences via significance analysis based on HV values alone. According to the results of significance tests, the proposed INSEA outperforms the NSEA by utilizing Grid-IGD values. As a consequence, the infeasible-updating non-dominated sorting strategy presented for INSEA is effective. The differential evolution operator, on the other hand, is more helpful than the crossover operator in reflecting the effectiveness of infeasible-updating non-dominated sorting. Furthermore, the combination of neighbor pairing strategy, differential evolution, and polynomial mutation can help INSEA attain better performance than other combinations, proving that the proposed INSEA is effective.

<!-- Meanless: 13-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

<!-- Media -->

<!-- figureText: 1500 1200 Via-points of B-spline Via-points of B-spline Via-points of Cubic spline 1000 Via-points of Cubic spline Extra points of Cubic spline Extra points of Cubic spline B-spline 800 - B-spline Position(mm) Cubic spline 600 200 0 -200 30 40 50 60 10 20 30 40 50 60 Time Time (b) Trajectory #2 (c) Trajectory #3 24.2 24 Via-points of B-spline Position(mm) 23.8 23.6 Via-points of B-spline 23.4 Via-points of Cubic spline Via-points of Cubic spline Extra points of Cubic spline Extra points of Cubic spline 23.2 Cubic spline Cubic spline 23 40 0 30 60 Time Time (e) Trajectory #5 (f) Trajectory #6 Via-points of Cubic spline Extra points of Cubic spline B-spline Cubic spline 30 40 50 60 Time Via-points of B-spline Via-points of Cubic spline -20 Extra points of Cubic spline B-spline -40 Position(mm) 500 Position(mm) -60 -100 -120 0 -140 10 20 30 40 50 60 10 20 Time (a) Trajectory #1 200 450 Via-points of B-spline 400 150 Via-points of Cubic spline Position(mm) Extra points of Cubic spline cition(mm) 350 300 B-spline Cubic spline 100 50 200 150 20 40 10 (d) Trajectory #4 2.5 2 Position(mm) 1.5 0.5 10 20 (g) Trajectory #7 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_13.jpg?x=112&y=146&w=1526&h=1447&r=0"/>

Fig. 10. An example of trajectory planning by using INSEA for autonomous erecting segment 1: actuator reference inputs used to drive robot joints.

<!-- Media -->

Fig. 9 displays the evolutionary results of the fourth generation acquired by INSEA and NSEA in stage I of the problem, which are taken from the median Grid-IGD values among 15 runs as a reference. In Fig. 9 (a) and (c), the color assigned to an individual represents constraint violations, whereas in Fig. 9(b) and (d), it denotes non-domination levels. It is obvious that the proposed INSEA, utilizing objective function information, broadens the direction of evolution, particularly at the start with numerous infeasible solutions. A good balance between objectives and constraints is achieved. As shown in Fig. 9(a) and (b), many individuals with higher constraint violations and near the Pareto front are chosen to join the next generation rather than being eliminated.

### 4.6. Evaluation of interpolation performance

Based on the results above, the best Pareto fronts from the proposed INSEA could be obtained to generate trajectories for each joint of the segment assembly robot. To guarantee efficiency, the solutions with the shortest time are chosen to plot the trajectory of stage I. Stage II employs individuals who have the longest time to carry out a smooth process for moving to the final position. It should be noted that they can be chosen freely based on different requirements. Both above are derived from the results with median Grid-IGD in Sect. 4.4, which is taken as a reference. Furthermore, cubic spline curves are utilized to compare the effectiveness of the B-spline interpolation by using INSEA. The cubic method is the most common and creates a smooth motion. By introducing additional points, users can define the velocities and accelerations of the beginning and ending positions. Figs. 10 and 11 show a comparison of two trajectory curves for position, velocity, acceleration, and jerk during the assembly procedure.

<!-- Meanless: 14-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

<!-- Media -->

<!-- figureText: \( {i}_{1}\left( {\mathrm{{mm}}/\mathrm{s}}\right) {a}_{1}\left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) {j}_{1}\left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \) 20 \( {v}_{2}\left( {\mathrm{{deg}}/\mathrm{s}}\right) {a}_{2}\left( {\mathrm{{deg}}/{\mathrm{s}}^{2}}\right) {j}_{2}\left( {\mathrm{{deg}}/{\mathrm{s}}^{3}}\right) \) -5 (B-spline) \( {v}_{3}\left( {\mathrm{{mm}}/\mathrm{s}}\right) {a}_{3}\left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) {j}_{3}\left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \) 40 0 \( {}_{7} \) (B-spline) -20 \( {}_{7} \) (B-spline) -40 2 (Cubic spline) (Cubic spline) \( {j}_{7} \) (Cubic spline) \( {j}_{7} \) (B-spline) \( {v}_{7} \) (Cubic spline) \( {a}_{7} \) (Cubic spline) \( {j}_{7} \) (Cubic spline) -60 40 50 60 10 20 30 40 60 Time Time (c) Trajectory #3 \( {}_{7} \) (B-spline) \( {v}_{6}\left( {\mathrm{{mm}}/\mathrm{s}}\right) {a}_{6}\left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) {j}_{6}\left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \) \( {v}_{7} \) (B-spline) \( {a}_{7}\;( \) B-spline \( ) \) 0.1 \( {}_{7} \) (B-spline) (Cubic spline) (Cubic spline) 0.05 \( {j}_{7} \) (Cubic spline) -0.05 7 (B-spline) \( {}_{7} \) (B-spline) (Cubic spline) \( {\iota }_{7} \) (Cubic spline) \( {j}_{7} \) (Cubic spline) -0.1 40 50 0 10 20 30 40 50 60 Time Time (e) Trajectory #5 (f) Trajectory #6 (B-spline) (B-spline) (B-spline) 7 (Cubic spline) \( {a}_{7} \) (Cubic spline) \( {j}_{7} \) (Cubic spline) 30 60 Time (g) Trajectory #7 -10 -20 -40 \( {v}_{7} \) (B-spline) \( {a}_{7} \) (B-spline) -80 \( {j}_{7} \) (B-spline) \( {v}_{7} \) (Cubic spline) -100 \( {a}_{7} \) (Cubic spline) \( {j}_{7} \) (Cubic spline) -120 -15 10 20 40 50 60 10 20 30 Time (a) Trajectory #1 (b) Trajectory #2 \( {v}_{4}\left( {\mathrm{{mm}}/\mathrm{s}}\right) {a}_{4}\left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) {j}_{4}\left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \) 15 \( {v}_{5}\left( {\mathrm{{mm}}/\mathrm{s}}\right) {a}_{5}\left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) {j}_{5}\left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \) 30 20 10 0 -10 -20 (B-spline) (B-spline) (Cubic spline) 7 (Cubic spline) \( {j}_{7} \) (Cubic spline) 0 10 30 40 60 10 20 Time (d) Trajectory #4 0.1 \( {v}_{7}\left( {\mathrm{{mm}}/\mathrm{s}}\right) {a}_{7}\left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) {j}_{7}\left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \) 0.05 0 -0.05 -0.1 -0.15 -0.2 -0.25 20 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9in7aajc738sf80g_14.jpg?x=122&y=148&w=1519&h=1448&r=0"/>

Fig. 11. Velocity, acceleration and jerk trajectories versus time.

<!-- Media -->

It can be seen from Figs. 10 and 11 that the velocity and acceleration are all continuous and bounded. Furthermore, the initial and final values of joint velocity and acceleration could be configured almost arbitrarily as required. However, compared with B-spline, the acceleration trajectory for cubic spline curves is continuous but not differentiable, and the jerk trajectory is not continuous. Multi-degree NURBS curves increase trajectory smoothness, and the initial and final jerk values can be imposed depending on requirements. To further investigate the motion trajectories, the maximum acceleration and jerk values are employed as performance indicators. In addition to the findings in Figs. 10 and 11, two additional tests on cubic spline interpolation are carried out, and all the results are displayed in Tables 10 and 11. It can be seen that the acceleration trajectory fitted using B-spline exhibits lower peak values and smoother curves. Although the maximum jerk of the cubic spline curve trajectory is sometimes smaller than that of the B spline curve for serval joints, the jerk curve discontinuity and inability to define the start and end points are inevitable problems.

## 5. Conclusions

In this research, a B-spline interpolation- and infeasible-updating non-dominated sorting-based method for multi-objective trajectory planning of shield machine segment assembly robots is put forward. The study starts by developing a mathematical model of the constrained multi-objective optimization problem for segment assembly trajectory planning in underground tunneling. To design multi-objective optimization functions, it considers time, acceleration, and jerk of the joint trajectories. Higher-degree NURBS curves are utilized to fit the trajectories to determine the values, and constraints can be specified by a limited number of control points. Based on this, the multi-objective function information is used for infeasible solutions to reduce the greediness of the common feasibility rule, especially when dealing with a complex search space. Some infeasible individuals could be adaptively replaced with solutions that violate the constraints more severely but are closer to the true Pareto front.

<!-- Meanless: 15-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

<!-- Media -->

Table 10

Maximum values of acceleration trajectories optimized by INSEA.

<table><tr><td rowspan="2">Trajectory No.</td><td rowspan="2">B-spline</td><td colspan="3">Cubic spline</td></tr><tr><td>1</td><td>2</td><td>3</td></tr><tr><td>\( 1\left( {\mathrm{\;{mm}}/{\mathrm{s}}^{2}}\right) \)</td><td>27.160</td><td>28.942</td><td>28.011</td><td>31.181</td></tr><tr><td>2 (deg \( /{\mathrm{s}}^{2} \) )</td><td>3.798</td><td>4.827</td><td>4.470</td><td>6.254</td></tr><tr><td>\( 3\left( {\mathrm{\;{mm}}/{\mathrm{s}}^{2}}\right) \)</td><td>14.680</td><td>20.647</td><td>19.969</td><td>26.811</td></tr><tr><td>4 (mm \( /{\mathrm{s}}^{2} \) )</td><td>4.008</td><td>4.147</td><td>4.158</td><td>4.450</td></tr><tr><td>\( 5\left( {\mathrm{\;{mm}}/{\mathrm{s}}^{2}}\right) \)</td><td>18.619</td><td>21.100</td><td>21.745</td><td>18.762</td></tr><tr><td>6 (mm \( /{\mathrm{s}}^{2} \) )</td><td>0.045</td><td>0.081</td><td>0.082</td><td>0.091</td></tr><tr><td>7 (mm \( /{\mathrm{s}}^{2} \) )</td><td>0.060</td><td>0.070</td><td>0.063</td><td>0.094</td></tr></table>

Table 11

Maximum values of jerk trajectories optimized by INSEA.

<table><tr><td rowspan="2">Trajectory No.</td><td rowspan="2">B-spline</td><td colspan="3">Cubic spline</td></tr><tr><td>1</td><td>2</td><td>3</td></tr><tr><td>\( 1\left( {\mathrm{\;{mm}}/{\mathrm{s}}^{3}}\right) \)</td><td>13.301</td><td>20.181</td><td>17.983</td><td>12.062</td></tr><tr><td>\( 2\left( {\mathrm{{deg}}/{\mathrm{s}}^{3}}\right) \)</td><td>2.285</td><td>2.121</td><td>1.897</td><td>4.201</td></tr><tr><td>\( 3\left( {\mathrm{\;{mm}}/{\mathrm{s}}^{3}}\right) \)</td><td>7.839</td><td>9.358</td><td>9.746</td><td>18.726</td></tr><tr><td>\( 4\left( {\mathrm{\;{mm}}/{\mathrm{s}}^{3}}\right) \)</td><td>1.659</td><td>1.430</td><td>1.514</td><td>1.623</td></tr><tr><td>\( 5\left( {\mathrm{\;{mm}}/{\mathrm{s}}^{3}}\right) \)</td><td>15.141</td><td>12.401</td><td>12.873</td><td>17.396</td></tr><tr><td>6 (mm \( /{\mathrm{s}}^{3} \) )</td><td>0.045</td><td>0.058</td><td>0.055</td><td>0.065</td></tr><tr><td>7 (mm \( /{\mathrm{s}}^{3} \) )</td><td>0.056</td><td>0.060</td><td>0.046</td><td>0.083</td></tr></table>

<!-- Media -->

The data on path points acquired from the segment assembly robot instrumented at Shanghai Tunnel Engineering Ltd. was utilized to validate the effectiveness of the proposed method. The assembly procedure is separated into two stages to satisfy different work requirements. Due to increasing constraints, the complexity of feasible areas in the first stage is greater than in the second stage. To validate the proposed INSEA, NSGA-II, CCMO, TSTI, TiGE-2, C3M, and ICMA are employed to address the multi-objective trajectory planning problem in two stages of the assembly procedure. The Grid-IGD and HV results show that the proposed INSEA outperforms the six state-of-the-art MOTP algorithms regarding overall performance. It can give optimal trade-off alternatives for multi-objective trajectory planning to researchers. Meanwhile, ablation experiments show that infeasible-updating non-dominated sorting is effective. Some individuals with large constraint violations and close to the Pareto front are selected for the next generation, which could enhance the diversity and convergence of Pareto optimal solutions for the MOTP problem. In addition, conventional cubic spline curves are compared to NURBS curves based on the proposed INSEA. NURBS curves make sure that joint trajectories are sufficiently smooth and that acceleration peaks are small, even though two more points can be used to set the speed and acceleration of both the start and end points of cubic spline trajectories. As a result, this study shows that the proposed MOTP method is valuable for researchers engaged in trajectory planning for segment assembly robots of non-circular shield machines.

In the future, complex objectives and constraints can be constructed for the MOTP problem. The appropriate interpolation methods have to be considered to ensure trajectory performance while reducing the time required to calculate the objective function values. Based on this, it is still worthwhile to investigate how to further balance multiple objectives and constraints. The operator that generates offspring cannot be ignored in improving the overall performance of the algorithm.

## CRediT authorship contribution statement

Xu Shuang: Data curation, Formal analysis, Validation. Dong Chang: Methodology, Validation, Visualization. Qin Chengjin: Validation, Writing - review & editing. Tao Jianfeng: Supervision, Writing

- review & editing. Sun Hao: Conceptualization, Methodology, Software, Writing - original draft, Writing - review & editing. Liu Chen-gliang: Project administration, Supervision, Validation. Zhuang Qianwei: Investigation, Project administration.

## Declaration of Competing Interest

The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper.

## Data availability

Data will be made available on request.

## Acknowledgments

This work was supported by National Natural Science Foundation of China (Grant No. 52075320), Shanghai Rising-Star Program (Grant No. 23QC1400400), Shanghai Pujiang Program (Grant No. 21PJ1407100), and State Key Laboratory of Mechanical System and Vibration (Grant No. MSVZD202006). The computations in this paper were run on the Siyuan-1 cluster supported by the Center for High Performance Computing at Shanghai Jiao Tong University.

## Appendix A. Supporting information

Supplementary data associated with this article can be found in the online version at doi:10.1016/j.asoc.2023.111216. References

[1] Y. Zhou, Y. Wang, L. Ding, P.E. Love, Utilizing, IFC for shield segment assembly in underground tunneling, Autom. Constr. 93 (2018) 178-191.

[2] M. Zheng, M. Lan, C.-L. Zhu, D.-S. Lin, W.-H. Zhang, Overall design for a tunnel segment assembly system of shield machine based on virtual reality technology, in: Proceedings of the IEEE International Conference on Architecture, Construction, Environment and Hydraulics (ICACEH), 2019, IEEE,, 2019, pp. 78-81.

[3] C. Wu, X.-J. Liu, L.-P. Wang, J. Wang, Dimension optimization of an orientation fine-tuning manipulator for segment assembly robots in shield tunneling machines, Autom. Constr. 20 (2011) 353-359.

[4] Z. Junwei, G. Sicen, C. Yunyao, Mechanical behavior of sealed waterproof for shield tunnel segment joint under different assembling ellipticity, Sci. Prog. 104 (2021), 0036850420987044.

[5] D.-M. Zhang, J. Liu, Z.-K. Huang, G.-H. Yang, Y. Jiang, K. Jia, Waterproof performance of tunnel segmental joints under different deformation conditions, Tunn. Undergr. Space Technol. 123 (2022) 104437.

[6] P. Lou, Y. Li, H. Xiao, Z. Zhang, S. Lu, Influence of small radius curve shield tunneling on settlement of ground surface and mechanical properties of surrounding rock and segment, Appl. Sci. 12 (2022) 9119.

[7] Y. Tanaka, Automatic segment assembly robot for shield tunneling machine, Comput. -Aided Civ. Infrastruct. Eng. 10 (1995) 325-337.

[8] K. Kosuge, K. Takeo, D. Taguchi, T. Fukuda, H. Murakami, Task-oriented force control of parallel link robot for the assembly of segments of a shield tunnel excavation system, IEEE/ASME Trans. Mechatron. 1 (1996) 250-258.

[9] H. Shi, G. Gong, H. Yang, L. Wang, Positioning speed and precision control of a segment erector for a shield tunneling machine. Proceedings of the 2010 IEEE/ ASME International Conference on Advanced Intelligent Mechatronics, IEEE,, 2010, pp. 1076-1080.

[10] L. Wang, G. Gong, H. Yang, X. Yang, D. Hou, The development of a high-speed segment erecting system for shield tunneling machine, IEEE/ASME Trans. Mechatron. 18 (2013) 1713-1723.

[11] L. Wang, W. Sun, G. Gong, H. Yang, Electro-hydraulic control of high-speed segment erection processes, Autom. Constr. 73 (2017) 67-77.

<!-- Meanless: 16-->




<!-- Meanless: H. Sun et al. Applied Soft Computing 152 (2024) 111216-->

[12] B. Yang, A. Zhang, Y. Bai, K. Zhang, H. Li, Development and simulation of magnetorheological damper for segment erector vibration control, Trans. Can. Soc. Mech. Eng. 43 (2018) 237-247.

[13] H. Nakamura, T. Kubota, M. Furukawa, T. Nakao, Unified construction of running track tunnel and crossover tunnel for subway by rectangular shape double track cross-section shield machine, Tunn. Undergr. Space Technol. 18 (2003) 253-262.

[14] Y. Moriya, Special shield tunnelling methods in Japan. Tunnels and Underground Structures, Routledge,, 2017, pp. 249-254.

[15] J. Li, Key technologies and applications of the design and manufacturing of noncircular TBMs, Engineering 3 (2017) 905-914.

[16] H. Sun, J. Tao, C. Qin, H. Yu, S. Xu, Q. Zhuang, C. Liu, Optimal energy consumption and response capability assessment for hydraulic servo systems containing counterbalance valves, J. Mech. Des. 145 (2023) 053501.

[17] G.F. Ritelli, A. Vacca, Energetic and dynamic impact of counterbalance valves in fluid power machines, Energy Convers. Manag. 76 (2013) 701-711.

[18] D. Ortiz Morales, S. Westerberg, P.X. La Hera, U. Mettin, L. Freidovich, A. S. Shiriaev, Increasing the level of automation in the forestry logging process with crane trajectory planning and control, J. Field Robot. 31 (2014) 343-363.

[19] D.O. Morales, P. La Hera, U. Mettin, L.B. Freidovich, A.S. Shiriaev, S. Westerberg, Steps in trajectory planning and controller design for a hydraulically driven crane with limited sensing, in: Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems, 2010, IEEE,, 2010, pp. 3836-3841.

[20] Y. Zhang, Z. Sun, Q. Sun, Y. Wang, X. Li, J. Yang, Time-jerk optimal trajectory planning of hydraulic robotic excavator, Adv. Mech. Eng. 13 (2021), 16878140211034611.

[21] Z. Zou, J. Chen, X. Pang, Task space-based dynamic trajectory planning for digging process of a hydraulic excavator with the integration of soil-bucket interaction, in: Proceedings of the Institution of Mechanical Engineers, 233, Part K: Journal of Multi-body Dynamics,, 2019, pp. 598-616.

[22] G. Sun, H. Zhang, J. Fang, G. Li, Q. Li, Multi-objective and multi-case reliability-based design optimization for tailor rolled blank (TRB) structures, Struct. Multidiscip. Optim. 55 (2017) 1899-1916.

[23] Y. Lu, S. Wang, Y. Zhao, C. Yan, Renewable energy system optimization of low/ zero energy buildings using single-objective and multi-objective optimization methods, Energy Build. 89 (2015) 61-75.

[24] A. Slowik, H. Kwasnicka, Evolutionary algorithms and their applications to engineering problems, Neural Comput. Appl. 32 (2020) 12363-12379.

[25] A. Kumar, G. Wu, M.Z. Ali, Q. Luo, R. Mallipeddi, P.N. Suganthan, S. Das, A benchmark-suite of real-world constrained multi-objective optimization problems and some baseline results, Swarm Evolut. Comput. 67 (2021) 100961.

[26] J. Liang, X. Ban, K. Yu, B. Qu, K. Qiao, C. Yue, K. Chen, K.C. Tan, A survey on evolutionary constrained multiobjective optimization, IEEE Trans. Evolut. Comput. 27 (2022) 201-221.

[27] I. Rahimi, A.H. Gandomi, F. Chen, E. Mezura-Montes, A review on constraint handling techniques for population-based algorithms: from single-objective to multi-objective optimization, Arch. Comput. Methods Eng. 30 (2023) 2181-2209.

[28] K. Deb, A. Pratap, S. Agarwal, T. Meyarivan, A fast and elitist multiobjective genetic algorithm: NSGA-II, IEEE Trans. Evolut. Comput. 6 (2002) 182-197.

[29] Y. Yusoff, M.S. Ngadiman, A.M. Zain, Overview of NSGA-II for optimizing machining process parameters, Procedia Eng. 15 (2011) 3978-3983.

[30] S. Verma, M. Pant, V. Snasel, A comprehensive review on NSGA-II for multi-objective combinatorial optimization problems, IEEE Access 9 (2021) 57757-57791.

[31] J. Huang, P. Hu, K. Wu, M. Zeng, Optimal time-jerk trajectory planning for industrial robots, Mech. Mach. Theory 121 (2018) 530-544.

[32] Y. Huang, M. Fei, Motion planning of robot manipulator based on improved NSGA-II, Int. J. Control, Autom. Syst. 16 (2018) 1878-1886.

[33] R. Saravanan, S. Ramabalan, C. Balamurugan, A. Subash, Evolutionary trajectory planning for an industrial robot, Int. J. Autom. Comput. 7 (2010) 190-198.

[34] M. da Graça Marcos, J.T. Machado, T.-P. Azevedo-Perdicoúlis, A multi-objective approach for the motion planning of redundant manipulators, Appl. Soft Comput. 12 (2012) 589-599.

[35] C.-g Cui, Y.-j Li, T.-j Wu, A relative feasibility degree based approach for constrained optimization problems, J. Zhejiang Univ. Sci. C 11 (2010) 249-260.

[36] H. Ma, H. Wei, Y. Tian, R. Cheng, X. Zhang, A multi-stage evolutionary algorithm for multi-objective optimization with complex constraints, Inf. Sci. 560 (2021) 68-91.

[37] K. Deb, H. Jain, An evolutionary many-objective optimization algorithm using reference-point-based non-dominated sorting approach, part I: solving problems with box constraints, IEEE Trans. Evolut. Comput. 18 (2013) 577-601.

[38] L. Pan, W. Xu, L. Li, C. He, R. Cheng, Adaptive simulated binary crossover for rotated multi-objective optimization, Swarm Evolut. Comput. 60 (2021) 100759.

[39] Y. Wang, B.-C. Wang, H.-X. Li, G.G. Yen, Incorporating objective function information into the feasibility rule for constrained evolutionary optimization, IEEE Trans. Cybern. 46 (2015) 2938-2952.

[40] A. Gasparetto, A. Lanzutti, R. Vidoni, V. Zanotto, Experimental validation and comparative analysis of optimal time-jerk algorithms for trajectory planning, Robot. Comput. -Integr. Manuf. 28 (2012) 164-181.

[41] Y. Li, T. Huang, D.G. Chetwynd, An approach for smooth trajectory planning of high-speed pick-and-place parallel robots using quintic B-splines, Mech. Mach. Theory 126 (2018) 479-490.

[42] Z. Wang, Y. Li, P. Sun, Y. Luo, B. Chen, W. Zhu, A multi-objective approach for the trajectory planning of a 7-DOF serial-parallel hybrid humanoid arm, Mech. Mach. Theory 165 (2021) 104423.

[43] C. De Boor, A Practical Guide to Splines, Springer,, New York, 2001.

[44] J. Yuan, H.-L. Liu, C. Peng, Population decomposition-based greedy approach algorithm for the multi-objective knapsack problems, Int. J. Pattern Recognit. Artif. Intell. 31 (2017) 1759006.

[45] J. Yuan, H.-L. Liu, Y.-S. Ong, Z. He, Indicator-based evolutionary algorithm for solving constrained multiobjective optimization problems, IEEE Trans. Evolut. Comput. 26 (2021) 379-391.

[46] H. Li, Q. Zhang, Multiobjective optimization problems with complicated Pareto sets, MOEA/D and NSGA-II, IEEE Trans. Evolut. Comput. 13 (2008) 284-302.

[47] K. Deb, M. Goyal, A combined genetic adaptive search (GeneAS) for engineering design, Comput. Sci. Inform. 26 (1996) 30-45.

[48] Y. Fang, J. Li, A review of tournament selection in genetic programming. Proceedings of the International Symposium on Intelligence Computation and Applications, Springer,, 2010, pp. 181-192.

[49] K. Deb, R.B. Agrawal, Simulated binary crossover for continuous search space, Complex Syst. 9 (1995) 115-148.

[50] Y. Tian, T. Zhang, J. Xiao, X. Zhang, Y. Jin, A coevolutionary framework for constrained multiobjective optimization problems, IEEE Trans. Evolut. Comput. 25 (2020) 102-116.

[51] J. Dong, W. Gong, F. Ming, L. Wang, A two-stage evolutionary algorithm based on three indicators for constrained multi-objective optimization, Expert Syst. Appl. 195 (2022) 116499.

[52] Y. Zhou, M. Zhu, J. Wang, Z. Zhang, Y. Xiang, J. Zhang, Tri-goal evolution framework for constrained many-objective optimization, IEEE Trans. Syst. Man, Cybern.: Syst. 50 (2018) 3086-3099.

[53] R. Sun, J. Zou, Y. Liu, S. Yang, J. Zheng, A. Multi-stage, Algorithm for solving multi-objective optimization problems with multi-constraints, IEEE Trans. Evolut. Comput. (2022).

[54] Y. Tian, R. Cheng, X. Zhang, Y. Jin, PlatEMO: a MATLAB platform for evolutionary multi-objective optimization [educational forum], IEEE Comput. Intell. Mag. 12 (2017) 73-87.

[55] P.A. Bosman, D. Thierens, The balance between proximity and diversity in multiobjective evolutionary algorithms, IEEE Trans. Evolut. Comput. 7 (2003) 174-188.

[56] E. Zitzler, D. Brockhoff, L. Thiele, The hypervolume indicator revisited: On the design of Pareto-compliant indicators via weighted integration. In: Proceedings of the 4th International Conference on Evolutionary Multi-Criterion Optimization, EMO 2007, Matsushima, Japan, March 5-8, 2007. Proceedings 4, Springer, 2007, pp. 862-876.

[57] X. Cai, Y. Xiao, M. Li, H. Hu, H. Ishibuchi, X. Li, A grid-based inverted generational distance for multi/many-objective optimization, IEEE Trans. Evolut. Comput. 25 (2020) 21-34.

[58] R.G. d Steel, J.H. Torrie, Principles and Procedures of Statistics: A Biometrical Approach, McGraw-Hill,, New York, NY, USA, 1986.

<!-- Meanless: 17-->

