

<!-- Meanless: IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING, VOL. 21, NO. 4, OCTOBER 2024 7629-->

# A Convex Optimization Method to Time-Optimal Trajectory Planning With Jerk Constraint for Industrial Robotic Manipulators

Chen Ji \( {}^{ - } \) ,Zhongqiang Zhang,Guanggui Cheng,Minxiu Kong,and Ruifeng Li \( {}^{ - } \)

Abstract-This paper proposes a convex time-optimal trajectory planning method for industrial robotic manipulators with jerk constraints. To achieve smooth and efficient trajectories, the square of the pseudo velocity profile is constructed using a cubic uniform B-spline, and a linear relationship is defined with the control points of the B-spline to preserve convexity in the pseudo states. Bi-linear and non-convex jerk constraints are introduced in the optimization problem, and a convex restriction method is applied to achieve convexity. The proposed method is evaluated through three case studies: two contour following tasks and a pick-and-place task. Comparative optimization results demonstrate that the proposed method achieves time optimality and trajectory smoothness simultaneously in the reformulated and jerk-restricted optimization problem. The proposed method provides a practical approach to address the non-convexity of jerk constraints in trajectory optimization for industrial robotic manipulators.

Note to Practitioners-This work was motivated by the challenge of generating smooth and time-optimal trajectories for industrial manipulators while considering jerk and dynamic constraints. Existing approaches typically use multi-objective methods that treat jerk constraints as soft constraints. In contrast, this work proposes a B-spline-based convex optimization method that treats jerk constraints as hard constraints. Three case studies are conducted, involving butterfly-type, door-type and 'OPTEC'-type paths, with the UR5 cooperative robot. The numerical and experimental results demonstrate that the proposed method sacrifices some time optimality to achieve trajectory smoothness and computational efficiency due to the convex restriction reformulation of the jerk constraints. Moreover, the proposed method is applicable to general industrial manipulators. Supplementary video is available at https://youtu.be/gVz5IuPKD-o.

Index Terms- Trajectory optimization, jerk constraint, convex optimization, cubic B-spline.

## I. INTRODUCTION

ROBOT manipulators have a wide range of industrial applications such as loading and unloading, welding, and painting due to their high precision and fast execution cycles. Time-optimal trajectory planning is a significant challenge in engineering domains such as robotics and aerospace, as it requires generating a trajectory that meets specified constraints while minimizing execution time. Due to the complexity and high dynamics of robot manipulators, solving for time-optimal trajectories subject to kinematic and dynamic constraints is notably challenging [1]. A common set of constraints includes limits on velocity, acceleration, jerk, torque, and torque rate, which ensure smooth and efficient motion of the system [2], [3], [4], [5], [6], [7]. While extensive research has focused on trajectory optimization with velocity and torque constraints, there has been comparatively less attention paid to optimizing trajectories subject to jerk constraints. Jerk, defined as the time derivative of acceleration, plays a crucial role in determining the trajectory's smoothness. Trajectories without jerk constraints potentially cause mechanical wear and tear and excite resonance frequencies [8]. However, imposing jerk constraints on the system's dynamics increases the optimization problem's complexity and computational expense due to the non-convex nature of these constraints. Therefore, developing efficient algorithms for time-optimal trajectories subject to jerk constraints is an important area for practical application.

There are two primary methods for addressing jerk constraints in time-optimal trajectory planning problems: jerk-limited trajectory planning method and optimization-based trajectory planning method. Jerk-limited trajectory planning generates trajectories that satisfy the jerk constraint using specialized algorithms, such as S-curve based method [9], [10], [11] or polynomial-based methods [12], [13]. The former involves defining an S-shaped velocity profile that restricts the rate of change of acceleration, while the latter determines the coefficients of a polynomial function that satisfies the jerk constraint. Due to their clear and configurable constraints as well as their computational efficiency, these methods are widely utilized for online trajectory planning in industrial manipulators. However, these methods do not achieve time-optimal solutions as they do not consider the system dynamics of manipulators.

On the other hand, optimization-based trajectory planning involves formulating the trajectory planning problem as an optimization problem [2], [14], [15]. To achieve jerk constrained and time-optimal trajectories, various indirect or direct objective functions, such as time-square of torque rate [6], time-square of joint torques [16], and time-absolute of joint jerks [2], have been proposed to be incorporated with minimizing the execution time. Multi-objective optimization methods are used to simultaneously optimize for minimum execution time and minimum inclusion of jerk by adjusting weighting coefficients based on penalty function methods. In this way, jerk constraint is treated as soft constraint in optimization problem which means the inclusion of jerk is minimized as much as possible, but without strict guarantees that it will satisfy the limitation. This limits the effectiveness of its optimization trajectories. An alternative approach is to incorporate the jerk constraint as hard constraint into the optimization problem which requiring the optimal solution to satisfy the constraints at any time [17], [18], [19]. In comparison, the optimality of the results of multi-objective optimization depends on the choice of weighting coefficients, which it is not trivial to determine. Furthermore, using hard constraint can intuitively configure the specific limit value based on physical jerk limitations and finally obtain a more desirable optimal solution, such as dynamic programming method [20], numerical integration method [21], [22]. However, incorporating the jerk constraint as hard constraints introduces non-convexity. It results an NP-hard problem to obtain the global optimal solution within its feasible domain and requiring significant computation time. Therefore, this leads to uncertainty and instability of the optimal solution and increases the difficulty of online planning, thereby limiting its widespread application in industrial scenarios.

---

<!-- Footnote -->

Manuscript received 6 October 2023; revised 1 November 2023 and 1 December 2023; accepted 21 December 2023. Date of publication 29 December 2023; date of current version 16 October 2024. This article was recommended for publication by Associate Editor X. Li and Editor L. Zhang upon evaluation of the reviewers' comments. This work was supported by the Jiangsu Robotics and Intelligent Manufacturing Equipment Engineering Laboratory, Soochow University. (Corresponding author: Chen Ji.)

Chen Ji, Zhongqiang Zhang, and Guanggui Cheng are with the School of Mechanical Engineering, Jiangsu University, Zhenjiang 212013, China (e-mail: jichen@ujs.edu.cn; zhangzq@ujs.edu.cn; ggcheng@ujs.edu.cn).

Minxiu Kong and Ruifeng Li are with the State Key Laboratory of Robotics and Systems, Harbin Institute of Technology, Harbin 150001, China (e-mail: exk@hit.edu.cn; lrf100@hit.edu.cn).

This article has supplementary material provided by the authors and color versions of one or more figures available at https://doi.org/10.1109/ TASE.2023.3346693.

Digital Object Identifier 10.1109/TASE.2023.3346693

<!-- Footnote -->

---

<!-- Meanless: 1545-5955 © 2023 IEEE. Personal use is permitted, but republication/redistribution requires IEEE permission. See https://www.ieee.org/publications/rights/index.html for more information. Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 7630 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING, VOL. 21, NO. 4, OCTOBER 2024-->

Therefore, some scholars are seeking breakthroughs in non-convex optimization algorithms and convex relaxation methods. For instance, PSO-type algorithms can solve non-convex problems, but they still have limitations in terms of solving efficiency and local optimal solutions [23]. In addition, some scholars have proposed using the McCormick envelope [24] and SDP relaxation methods to relax the non-convex and bi-linear jerk constraints, transforming them into convex constraints. Although these methods improve the computational efficiency, the system dynamics are not incorporated into the time-optimal problem. Therefore, the complexity and application of convex relaxation transformation without system dynamics is limited.

The composite trajectory optimization algorithm utilizing the jerk-limited and optimization based methods has gained significant attention due to its ability to quickly generate time-optimal and smooth trajectories while leveraging the advantages of each method [25], [26]. This approach utilizes spline-based trajectory planning method to generate smooth paths or trajectories in advance, which are then transformed into a time-optimal trajectory optimization problem. By leveraging the convex hull properties of splines, both acceleration and jerk constraints are inherently enforced, effectively incorporating jerk constraints into the optimization problem. In our previous work [27], the authors proposed a convex optimization method to generate smooth and time-optimal trajectories subject to dynamic and kinematic constraints of industrial manipulators. The cubic B-spline is utilized to construct the pseudo velocity profile, the bang-bang like trajectories become smooth taking advantage of the C2 smoothness of clamped spline which reduce the sudden change at the beginning and end of the motion. However, the shortcoming of the previous work is that jerk constraint is not considered and experiment validation is not conducted to verify the effectiveness on real industrial manipulators.

The contribution of this work is proposing a convex optimization method incorporating jerk constraint to efficiently generate smooth and nearly time-optimal trajectories of robot manipulator. And the experiment validations of optimization results are conducted on the universal robot UR5. In this work, the non-convex jerk constraint is transformed to convex constraints by convex restriction method and also constrained by curvature of B-spline, which improves the computational efficiency of original time-optimal problem subject to dynamic and jerk constraint. Convex restriction is a convex inner approximation of the feasible set which identifies a convex subset of the original nonconvex feasible set [28]. In contrast, convex relaxation is interpreted as convex outer approximations of the nonconvex feasible set. In practice, using stricter jerk constraints is more reasonable than relaxing the constraints to make the trajectories smooth enough to reduce impact. However, this results in narrowing the original feasible set, sacrificing some time optimality to achieve smoothness and computational efficiency. Three case studies in typical industrial scenarios are conducted. Contour-following tasks, including butterfly-type and 'OPTEC'-type paths, as well as a standard door-type pick-and-place task are conducted to indicate the effectiveness of the proposed method on the time optimality and smoothness of trajectories in the reformulated and jerk-restricted optimization problem.

The remainder of the paper is organized as follows. The pseudo states are introduced and constructed as the decision states of the trajectory optimization problem. The original time-optimal and jerk-constrained trajectory planning problem is presented as well in section II. Based on cubic B-spline, the pseudo velocity and the pseudo acceleration are reformulated with control points of B-spline in section III. By introducing the convex jerk constraint, the time-optimal problem is discretized with the direct transcription method and solved with a commercial numerical optimization solver GUROBI [29]. Then the comparison results with different optimization methods are presented in section IV.

## II. Problem Formulation

The time-optimal trajectory planning problem for industrial robots is a challenging task due to the strongly coupled and nonlinear system dynamics, especially when considering non-convex jerk constraints. In this section, we provide a detailed formulation of the original problem, incorporating the time-optimal and jerk-constrained objectives.

## A. System Dynamics

Consider a manipulator as a serial and open kinematic chain having \( \mathrm{N} + 1 \) rigid links,which are interconnected by \( \mathrm{N} \) joints. The link-side coordinates \( \mathbf{q} \in  {\mathbb{R}}^{N} \) represent joint angles. \( \dot{\mathbf{q}} \) and \( \ddot{\mathbf{q}} \) denote velocity and acceleration of robot joints,respectively. Following the Lagrangian approach, the equation of robot dynamics is derived and represented as

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: JI et al.: CONVEX OPTIMIZATION METHOD TO TIME-OPTIMAL TRAJECTORY PLANNING WITH JERK CONSTRAINT 7631-->

\[\mathbf{M}\left( \mathbf{q}\right) \ddot{\mathbf{q}} + \mathbf{C}\left( {\mathbf{q},\dot{\mathbf{q}}}\right) \dot{\mathbf{q}} + {\mathbf{\tau }}_{f}\left( \mathbf{q}\right) \operatorname{sgn}\left( \dot{\mathbf{q}}\right)  + \mathbf{G}\left( \mathbf{q}\right)  = \mathbf{\tau } \tag{1}\]

where \( \mathbf{\tau } \in  {\mathbb{R}}^{N} \) is applied torques by joint actuators and \( \mathbf{M}\left( \mathbf{q}\right)  \in  {\mathbb{R}}^{N \times  N} \) is the symmetric and positive definite inertia matrix, \( \mathbf{C}\left( {\mathbf{q},\dot{\mathbf{q}}}\right)  \in  {\mathbb{R}}^{N \times  N} \) is a matrix accounting for Coriolis and centrifugal effects, \( {\mathbf{\tau }}_{\mathbf{f}}\left( \mathbf{q}\right) \operatorname{sgn}\left( \dot{\mathbf{q}}\right)  \in  {\mathbb{R}}^{N} \) is a vector of Coulomb friction torques, \( \mathbf{G}\left( \mathbf{q}\right)  \in  {\mathbb{R}}^{N} \) denotes a vector of gravity torques which influenced only by the robot configuration.

Since robot motion along a prescribed geometry path is expressed with path coordinate \( s \in  \{ 0 \leq  s \leq  1\} ,\dot{s}\left( t\right)  = \) \( {ds}/{dt} \) and its time derivative \( \ddot{s}\left( t\right)  = {d}^{2}s/d{t}^{2} \) are defined as pseudo-velocity and pseudo-acceleration, respectively. Furthermore,the duration of motion is represented as \( T \) . The time dependent function \( s\left( t\right) \) starts from \( s\left( 0\right)  = 0 \) and ends at \( s\left( T\right)  = 1 \) . Therefore,the duration of motion is also expressed with path coordinate \( s \) as \( t = {\int }_{0}^{s}\left( {1/\dot{s}}\right) {ds} \) . Thus,the angles \( \mathbf{q} \) , velocity \( \dot{\mathbf{q}} \) and acceleration \( \ddot{\mathbf{q}} \) of robot joints are reexpresed with path coordinate \( s \) by.

\[\dot{\mathbf{q}}\left( s\right)  = {\mathbf{q}}^{\prime }\left( s\right) \dot{s} \tag{2}\]

\[\ddot{\mathbf{q}}\left( s\right)  = {\mathbf{q}}^{\prime }\left( s\right) \ddot{s} + {\mathbf{q}}^{\prime \prime }\left( s\right) {\dot{s}}^{2} \tag{3}\]

\[\dddot{\mathbf{q}}\left( s\right)  = {\mathbf{q}}^{\prime }\dddot{s} + 3{\mathbf{q}}^{\prime \prime }\dddot{s} + {\mathbf{q}}^{\prime \prime \prime }{\dot{s}}^{3} \tag{4}\]

where \( {\mathbf{q}}^{\prime } = \partial \mathbf{q}\left( s\right) /\partial s,{\mathbf{q}}^{\prime \prime } = {\partial }^{2}\mathbf{q}\left( s\right) /\partial {s}^{2} \) and \( {\mathbf{q}}^{\prime \prime \prime } = \) \( {\partial }^{3}\mathbf{q}\left( s\right) /\partial {s}^{3} \) are defined as partial derivative. And \( b\left( s\right) \) and \( a\left( s\right) \) are introduced to replace these two pseudo states \( \ddot{s} \) and \( {\dot{s}}^{2} \) as

\[a\left( s\right)  \triangleq  \ddot{s},b\left( s\right)  \triangleq  {\dot{s}}^{2} \tag{5}\]

Substitute (2), (3) and (5) into (1), the system dynamic of robotic manipulator is represented as a new form by

\[\mathbf{\tau }\left( s\right)  = \mathbf{m}\left( s\right) \ddot{s} + \mathbf{c}\left( s\right) {\dot{s}}^{2} + \mathbf{g}\left( s\right)  \tag{6}\]

\[ = \mathbf{m}\left( s\right) a\left( s\right)  + \mathbf{c}\left( s\right) b\left( s\right)  + \mathbf{g}\left( s\right)  \tag{7}\]

where

\[\mathbf{m}\left( s\right)  = \mathbf{M}\left( {\mathbf{q}\left( s\right) }\right) {\mathbf{q}}^{\prime }\left( s\right)  \tag{8}\]

\[\mathbf{c}\left( s\right)  = \mathbf{M}\left( {\mathbf{q}\left( s\right) }\right) {\mathbf{q}}^{\prime \prime }\left( s\right)  + \mathbf{C}\left( {\mathbf{q}\left( s\right) ,{\mathbf{q}}^{\prime }\left( s\right) }\right) {\mathbf{q}}^{\prime }\left( s\right)  \tag{9}\]

\[\mathbf{g}\left( s\right)  = {\mathbf{\tau }}_{\mathbf{f}}\left( {\mathbf{q}\left( s\right) }\right) \operatorname{sgn}\left( {{\mathbf{q}}^{\prime }\left( s\right) }\right)  + \mathbf{G}\left( {\mathbf{q}\left( s\right) }\right)  \tag{10}\]

## B. Dynamic Constraint

The joint torque are limited by physical saturation of electric motors, thus the joint torques of robot should be constrained as

\[{\mathbf{\tau }}_{\min } \leq  \mathbf{m}\left( s\right) a\left( s\right)  + \mathbf{c}\left( s\right) b\left( s\right)  + \mathbf{g}\left( s\right)  \leq  {\mathbf{\tau }}_{\max } \tag{11}\]

where \( {\mathbf{\tau }}_{\min } \in  {\mathbb{R}}^{N} \) and \( {\mathbf{\tau }}_{\max } \in  {\mathbb{R}}^{N} \) are the upper and lower boundaries of input torques due to the physical limitation.

## C. Kinematic Constraint

Besides the dynamic constraints stated above, the kinematic constraints including the velocity and acceleration of joints are transformed with new pseudo states as well and the derivation are expressed as follows.

\[{\dot{\mathbf{q}}}_{\min } \leq  {\mathbf{q}}^{\prime }\left( s\right) \dot{s} \leq  {\dot{\mathbf{q}}}_{\max } \Rightarrow  {\mathbf{q}}^{\prime }{\left( s\right) }^{2}b\left( s\right)  \leq  {\dot{\mathbf{q}}}_{\max } \tag{12}\]

\[{\ddot{\mathbf{q}}}_{\min } \leq  {\mathbf{q}}^{\prime }\left( s\right) \ddot{s} + {\mathbf{q}}^{\prime \prime }\left( s\right) {\dot{s}}^{2} \leq  {\ddot{\mathbf{q}}}_{\max } \tag{13}\]

\[ \Rightarrow   - {\ddot{\mathbf{q}}}_{\max } \leq  {\mathbf{q}}^{\prime }\left( s\right) a\left( s\right)  + {\mathbf{q}}^{\prime \prime }\left( s\right) b\left( s\right)  \leq  {\ddot{\mathbf{q}}}_{\max } \tag{14}\]

where \( {\dot{\mathbf{q}}}_{\min },{\dot{\mathbf{q}}}_{\max },{\ddot{\mathbf{q}}}_{\min },{\ddot{\mathbf{q}}}_{\max } \) are boundaries of velocity and acceleration,respectively. \( {\dot{\mathbf{q}}}_{\min } \) is set as \( - {\dot{\mathbf{q}}}_{\max } \) and \( {\ddot{\mathbf{q}}}_{\min } \) is set as \( - {\ddot{\mathbf{q}}}_{\max } \) . Furthermore,to reduce the vibration during time-optimum motion caused by instant acceleration, jerk which is the first order derivative of joint acceleration is introduced as constraints of optimization problem to maintain the smoothness of motion. Thus, \( d\left( s\right) \) is defined and introduced to reformulate joint jerk (4) in path coordinate space as

\[d\left( s\right)  \triangleq  \ddot{s}/\dot{s} \tag{15}\]

Substitute (15) into (4), then the joint jerks are reexpressed by

\[{\dddot{\mathbf{q}}}_{\min } \leq  {\mathbf{q}}^{\prime \prime \prime }\left( s\right) {\dot{s}}^{3} + 3{\mathbf{q}}^{\prime \prime }\left( s\right) \dot{s}\ddot{s} + {\mathbf{q}}^{\prime }\left( s\right) \dddot{s} \leq  {\dddot{\mathbf{q}}}_{\max }\]

\[ \Rightarrow   - {\dddot{\mathbf{q}}}_{\max } \leq  \sqrt{b\left( s\right) }\left( {{\mathbf{q}}^{\prime \prime \prime }\left( s\right) b\left( s\right)  + 3{\mathbf{q}}^{\prime \prime }\left( s\right) a\left( s\right) }\right. \]

\[\left. {+{\mathbf{q}}^{\prime }\left( s\right) d\left( s\right) }\right)  \leq  {\dddot{\mathbf{q}}}_{\max } \tag{16}\]

where \( {\ddot{\mathbf{q}}}_{\min },{\ddot{\mathbf{q}}}_{\max } \) are the boundaries of jerk respectively. And \( {\ddot{\mathbf{q}}}_{\min } \) is set as \( - {\ddot{\mathbf{q}}}_{\max } \) . The presence of the square of \( b\left( s\right) \) in (16) gives rise to non-convex and bi-linear jerk constraints, rendering the trajectory optimization problem with such constraints challenging to solve.

## D. Performance Criterion

Considering the time-optimal trajectory planning problem, the objective function is defined with the path coordinate and expressed as follows.

\[\min T = \min {\int }_{0}^{1}\frac{1}{\dot{s}}{ds} = \min {\int }_{0}^{1}\frac{1}{\sqrt{b\left( s\right) }}{ds} \tag{17}\]

Therefore, combined the reformulated kinematic and dynamic constraints above, the time-optimal and jerk constrained trajectory planning problem is represented as

\[\mathop{\min }\limits_{{a\left( s\right) ,b\left( s\right) ,d\left( s\right) }}{\int }_{0}^{1}\frac{1}{\sqrt{b\left( s\right) }}{ds}\]

\[\text{ s.t. }\left\{  \begin{array}{l} {\left( {\mathbf{q}}^{\prime }\left( s\right) \right) }^{2}b\left( s\right)  \leq  {\dot{\mathbf{q}}}_{\max }^{2} \\   - {\ddot{\mathbf{q}}}_{\max } \leq  {\mathbf{q}}^{\prime }\left( s\right) a\left( s\right)  + {\mathbf{q}}^{\prime \prime }\left( s\right) b\left( s\right)  \leq  {\ddot{\mathbf{q}}}_{\max } \\   - {\ddot{\mathbf{q}}}_{\max } \leq  \sqrt{b\left( s\right) }\left( {{\mathbf{q}}^{\prime \prime \prime }\left( s\right) b\left( s\right) }\right) \\  \; + 3{\mathbf{q}}^{\prime \prime }\left( s\right) a\left( s\right)  + {\mathbf{q}}^{\prime }\left( s\right) d\left( s\right) ) \leq  {\ddot{\mathbf{q}}}_{\max } \\  {\mathbf{\tau }}_{\min } \leq  \mathbf{m}\left( s\right) a\left( s\right)  + c\left( s\right) b\left( s\right)  + \mathbf{g}\left( s\right)  \leq  {\mathbf{\tau }}_{\max } \\  a\left( s\right)  = \ddot{s},b\left( s\right)  = {\dot{s}}^{2},d\left( s\right)  = \ddot{s},/\dot{s} \\  s\left( 0\right)  = 0,s\left( T\right)  = 1 \\  s\left( 0\right)  = 1,s\left( T\right)  = 1 \end{array}\right.  \tag{18}\]

where \( T \) denotes the motion duration. The pseudo states \( a\left( s\right) ,b\left( s\right) ,d\left( s\right) \) are considered as decision states of the optimization problem. The switch points of \( \dot{s} = \sqrt{b\left( s\right) } \) during robot motion are found through the search method. To find time-optimal trajectories is to achieve maximum switch curve subject to dynamic and kinematic constraints. Then the procedure based on numerical integration can be conducted to find the solutions of the switch curve [1].

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 7632 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING, VOL. 21, NO. 4, OCTOBER 2024-->

The bi-linear and non-convex nature of the jerk constraint in (16) deteriorates the convexity of the original time-optimal problem. As shown in (18), the time-optimal trajectory planning problem without jerk constraints is convex, allowing for global optimality and efficient solution. However, handling of jerk constraints is crucial to improve the smoothness and stability for executing time-optimum trajectories in real robots.

## III. Convex Optimization Method Based ON CUBIC B-SPLINE

In the optimization problem formulated in (18), the pseudo states \( a\left( s\right) ,b\left( s\right) \) ,and \( d\left( s\right) \) are treated as decision states of the optimization problem. To achieve time-optimal trajectory along a predefined path,the maximum pseudo velocity \( \dot{s} \) profile must be obtained, while respecting the physical constraints of the robot manipulators.

However, time-optimum trajectories lead to bang-bang-like trajectories of at least one of the robot joints, resulting in unwanted vibration during motion. This vibration negatively impact the performance of industrial robot applications, such as tracking accuracy or manipulation stability. In the authors' previous work,the pseudo state \( b\left( s\right) \) was discretized at the path coordinate \( s \) and then reconstructed using a uniform cubic B-spline. This approach maintained convexity of the optimization problem while achieving C1-continuity of the pseudo acceleration.

In this work, jerk constraints of robot joints are incorporated and reformulated into a convex form to efficiently achieve a optimized solution that is both jerk-bounded and time-optimal.

## A. B-Spline Based Reformulation of Pseudo States

The simplified form of uniform cubic B-spline is expressed as

\[f\left( x\right)  = \mathop{\sum }\limits_{{i = 1}}^{n}{f}_{i}\left( {{P}_{i - 1},{P}_{i},{P}_{i + 1},{P}_{i + 2},x}\right)  \tag{19}\]

where \( P = \left\{  {{P}_{i} \in  \mathbb{R},i = 0,\cdots ,n + 2}\right\} \) are defined as the control points of B-spline and \( {f}_{i}\left( x\right) \) is defined as \( i \) -th basic function of order 3 which is derived from Cox-deBoor recursive formulation and expressed as

\[{f}_{i}\left( x\right)  = \frac{1}{6}\left\lbrack  {1,x,{x}^{2},{x}^{3}}\right\rbrack  \left\lbrack  \begin{matrix} 1 & 4 & 1 & 0 \\   - 3 & 0 & 3 & 0 \\  3 &  - 6 & 3 & 0 \\   - 1 & 3 &  - 3 & 1 \end{matrix}\right\rbrack  \left\lbrack  \begin{matrix} {P}_{i - 1} \\  {P}_{i} \\  {P}_{i + 1} \\  {P}_{i + 2} \end{matrix}\right\rbrack  ,x \in  \left\lbrack  {0,1}\right\rbrack  \]

(20)

Based on the definition of \( b\left( s\right) \) ,the derivative of \( b\left( s\right) \) with respect to time is derived by

\[\dot{b}\left( s\right)  = {b}^{\prime }\left( s\right) \dot{s} = 2\dot{s}\ddot{s} \tag{21}\]

<!-- Media -->

<!-- figureText: \( {b}_{i + }^{w} \) \( {b}_{i + 2}^{w} \) \( {s}_{i}\;{s}_{i + 1}{s}_{i + 2} \) \( {s}_{n} = 1 \) \( {b}_{i}^{w} \) 0 0 0 \( {s}_{0} = 0 \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_3.jpg?x=912&y=153&w=747&h=498&r=0"/>

Fig. 1. The pseudo state \( b\left( s\right) \) profile reformulated with cubic uniform B-spline. The way points are noted with red square mark. At each interval \( \left\lbrack  {{s}_{i},{s}_{i + 1}}\right\rbrack \) ,the polynomial function is meshed and validated with more fined girds in the optimization problem.

<!-- Media -->

Furthermore, the first and second order partial derivative of \( b\left( s\right) \) with respect to the path coordinate \( s \) are derived by

\[{b}^{\prime }\left( s\right)  = {2a}\left( s\right)  = 2\ddot{s} \tag{22}\]

\[{b}^{\prime \prime }\left( s\right)  = {2d}\left( s\right)  = 2\dddot{s}/\dot{s} \tag{23}\]

Thus, according to the inner relationship in (22) and (23), the pseudo states \( a\left( s\right) \) and \( d\left( s\right) \) are linear with the partial derivative of \( b\left( s\right) \) . Due to the continuity and convex hull property of B-spline,the pseudo state \( b\left( s\right) \) is constructed with uniform cubic B-spline. Furthermore, the continuous pseudo state \( b\left( s\right) \) is discretized with scalar variables \( \left\{  {{b}_{i}^{w},i = }\right. \) \( 0,\cdots ,n\} \) which are defined as way points that the pseudo state profile must get through at path coordinate grid point \( {s}_{i} \) . As shown in Fig.1,the pseudo velocity \( b\left( s\right) \) is piece-wise function at each interval \( \left\lbrack  {{s}_{i},{s}_{i + 1}}\right\rbrack \) . Furthermore,the pseudo states \( a\left( s\right) \) and \( d\left( s\right) \) are expressed with polynomial function as well and reexpressed with the control points as

\[\left\lbrack  \begin{matrix} {b}_{i}\left( s\right) \\  2{a}_{i}\left( s\right) \\  2{d}_{i}\left( s\right)  \end{matrix}\right\rbrack   = \left\lbrack  \begin{matrix} {f}_{i}\left( s\right) \\  {f}_{i}^{\prime }\left( s\right) \\  {f}_{i}^{\prime \prime }\left( s\right)  \end{matrix}\right\rbrack   = \frac{1}{6}\left\lbrack  \begin{matrix} 1{s}_{m} & {s}_{m}^{2} & {s}_{m}^{3} & \\  0 & 1 & 2{s}_{m} & 3{s}_{m}^{2} \\  0 & 0 & 2 & 6{s}_{m} \end{matrix}\right\rbrack  \]

\[ \times  \left\lbrack  \begin{matrix} 1 & 4 & 1 & 0 \\   - 3 & 0 & 3 & 0 \\  3 &  - 6 & 3 & 0 \\   - 1 & 3 &  - 3 & 1 \end{matrix}\right\rbrack  \left\lbrack  \begin{matrix} {P}_{i - 1} \\  {P}_{i} \\  {P}_{i + 1} \\  {P}_{i + 2} \end{matrix}\right\rbrack  ,s \in  \left\lbrack  {{s}_{i},{s}_{i + 1}}\right\rbrack   \tag{24}\]

where \( {s}_{m} = \left( {s - {s}_{i}}\right) /\left( {{s}_{i + 1} - {s}_{i}}\right) \) . Therefore,the basic function of B-spline at each sub-interval is expressed as an algebraic form with four adjacent control points in (24). Then the pseudo states at the way points of the pseudo state profile are presented with control points linearly as

\[\left\lbrack  \begin{matrix} {b}_{i}\left( {s}_{i}\right) \\  2{a}_{i}\left( {s}_{i}\right)  \end{matrix}\right\rbrack   = \frac{1}{6}\left\lbrack  \begin{array}{rrrr} 1 & 4 & 1 & 0 \\   - 3 & 0 & 3 & 0 \end{array}\right\rbrack  \left\lbrack  \begin{matrix} {P}_{i - 1} \\  {P}_{i} \\  {P}_{i + 1} \\  {P}_{i + 2} \end{matrix}\right\rbrack   \tag{25}\]

At each sub-interval, the way points are represented with the control points due to the boundary conditions as (26). However, two more additional conditions (27) and (28) are added because there are only \( n + 1 \) equations for \( n + 3 \) unknown control points. Therefore, the clamped conditions are employed to constrain the initial and end points of the pseudo state \( {b}^{\prime }\left( s\right) \) as \( {b}^{\prime }\left( 0\right)  = {b}^{\prime }\left( 1\right)  = 0 \) .

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: JI et al.: CONVEX OPTIMIZATION METHOD TO TIME-OPTIMAL TRAJECTORY PLANNING WITH JERK CONSTRAINT 7633-->

\[\left\{  \begin{array}{l} {P}_{i - 1} + 4{P}_{i} + {P}_{i + 1} = 6{b}_{i}^{w} \\   - 3{P}_{0} + 3{P}_{2} = 0 \\   - 3{P}_{n} + 3{P}_{n + 2} = 0 \end{array}\right.  \tag{26}\]

(27)

(28)

Therefore,to obtain the control points \( \left\{  {{P}_{i} \in  \mathbb{R},i = }\right. \) \( 0,\cdots ,n + 2\} \) ,the homogeneous linear equations (26),(27),(28) are transferred as linear constraints in the optimization problem (18). What's more, the clamped conditions make the zero initial velocities and accelerations of the robot joints at the beginning and end of the time-optimal trajectories which is also shown in Fig.1. It leads a warm start and stop which make the trajectories smooth enough to be executed in a real robot.

## B. Convex Restriction for Jerk Constraint

At the last section, the pseudo states are represented with the three order polynomial functions and transformed as the linear equations of the adjacent control points of B-spline. Thus, the jerk constraints in the original optimization problem is represented as

\[ - {\dddot{\mathbf{q}}}_{\max } \leq  \sqrt{b\left( s\right) }\left( {{\mathbf{q}}^{\prime \prime \prime }\left( s\right) b\left( s\right)  + 3{\mathbf{q}}^{\prime \prime }\left( s\right) a\left( s\right) }\right. \]

\[\left. {+{\mathbf{q}}^{\prime }\left( s\right) d\left( s\right) }\right)  \leq  {\dddot{\mathbf{q}}}_{\max } \tag{29}\]

where \( \sqrt{b\left( s\right) } = \dot{s} \) denotes the pseudo velocity in the path coordinate space. Due to the objective function \( {\int }_{0}^{1}\frac{1}{\sqrt{b\left( s\right) }}{ds} \) , to generate the time-optimal trajectory along a predefined path is to find the maximum pseudo velocity without violating the physical constraints as

\[b\left( s\right)  \leq  {b}^{ * }\left( s\right)  \leq  {b}_{\max } \tag{30}\]

where \( {b}^{ * }\left( s\right) \) is the time-optimum profile subject to merely torque constraints which is the shortest motion duration under actuation limitations. And \( {b}_{\max } \) is the maximum of the pseudo state \( b\left( s\right) \) which is obtained by solving the linear programming problem to maximum \( b\left( s\right) \) subject to dynamic and kinematic constraints. Since \( a\left( s\right) ,b\left( s\right) ,d\left( s\right) \) have the linear relationship with control points,the term \( \mathbf{\eta }\left( s\right)  \triangleq  {\mathbf{q}}^{\prime \prime \prime }\left( s\right) b\left( s\right)  + 3{\mathbf{q}}^{\prime \prime }\left( s\right) a\left( s\right)  + \) \( {\mathbf{q}}^{\prime }\left( s\right) d\left( s\right) \) in (29) is linear due to the superposition property. And the \( {b}^{ * }\left( s\right) \) is utilized to substitute \( b\left( s\right) \) which is constant profile computed in advance. Therefore, the following relationship is satisfied

\[0 \leq  \sqrt{b\left( s\right) } \leq  \sqrt{{b}^{ * }\left( s\right) } \leq  \sqrt{{b}_{\max }} \tag{31}\]

Thus,if \( \mathbf{\eta }\left( s\right)  > \mathbf{0} \) ,there exists

\[\sqrt{b\left( s\right) }\mathbf{\eta }\left( s\right)  \leq  \sqrt{{b}^{ * }}\mathbf{\eta }\left( s\right) . \tag{32}\]

If \( \mathbf{\eta }\left( s\right)  \leq  \mathbf{0} \) ,there exists

\[\sqrt{b\left( s\right) }\mathbf{\eta }\left( s\right)  \geq  \sqrt{{b}^{ * }}\mathbf{\eta }\left( s\right) . \tag{33}\]

Therefore, a convex restriction is made for the jerk constraints by

\[ - {\dddot{\mathbf{q}}}_{\max } \leq  \sqrt{{b}^{ * }}\left( {{\mathbf{q}}^{\prime \prime \prime }\left( s\right) b\left( s\right)  + 3{\mathbf{q}}^{\prime \prime }\left( s\right) a\left( s\right) }\right. \]

\[\left. {+{\mathbf{q}}^{\prime }\left( s\right) d\left( s\right) }\right)  \leq  {\ddot{\mathbf{q}}}_{\max } \tag{34}\]

Since the jerk constraint is converted as linear and convex form in (34), the reformulated and jerk-constrained optimization problem can be solved efficiently and preciously. Compared with the McCormick envelope relaxation method, the proposed convex restriction method is much more easier to applied because the boundaries of \( b\left( s\right) ,a\left( s\right) ,d\left( s\right) \) are not clear with dynamic constraints which are necessary for McCormick envelope reformulations. Furthermore, it is easier to incorporate jerk constraints as hard constraint in the optimization problem. It is practical and intuitive to setup threshold of jerk based on the desired smoothness of application cases and physical limitations of robot actuators. However, the choice of substitute of \( b\left( s\right) \) is crucial to get close to the theoretical time-optimum solutions with jerk constraints. In this case, the time-optimum pseudo profile \( {b}^{ * }\left( s\right) \) is always larger than the theoretical ones which leads near-time optimal solution subject to jerk constraints. It is the trade-off between computation efficiency and time optimality.

## C. Numerical Solution

To solve the optimization problem stated above, the direct transcription method is used to reconstruct the original problem into a discretized form as a large scale problem. It is discretized on the path coordinate \( s \) with \( n + 1 \) grid points \( {s}_{0} = 0 \leq  {s}_{k} \leq  1 = {s}_{n} \) . Then the continuous pseudo states \( a\left( s\right) ,b\left( s\right) ,d\left( s\right) \) are modeling with a finite number of variables \( {a}^{k},{b}^{k},{d}^{k} \) while \( {b}^{k} \) are chosen as the feature points \( {b}_{k}^{F} \) of cubic B-spline to reformulate the pseudo states at each interval \( \left\lbrack  {{s}_{k},{s}_{k + 1}}\right\rbrack \) . It is worthy to note that the grid size should be chosen reasonably based on the trade-off between the computation efficiency and the tolerance of the solution. Therefore, the reformulated time-optimal problem with convex restriction method is converted to a discretized form as (35) and then the optimization procedure is shown in detail as follows.

\[\mathop{\min }\limits_{{{P}_{i - 1},{P}_{i},{P}_{i + 1},{P}_{i + 2}}}\mathop{\sum }\limits_{{k = 0}}^{{n + 1}}{\int }_{{s}_{k}}^{{s}_{k + 1}}\frac{1}{\sqrt{b\left( s\right) }}{ds}\]

\[s.t.\left\{  \begin{array}{l} \tau \left( {s}_{k}\right)  = m\left( {s}_{k}\right) {a}^{k} + \tau \left( {s}_{k}\right) {b}^{k} + g\left( {s}_{k}\right) \\   - {\tau }_{\max } \leq  \tau \left( {s}_{k}\right)  \leq  {\tau }_{\max } \\   - {\ddot{q}}_{\max } \leq  \sqrt{{q}_{k}^{2} + {q}^{\prime }{}^{4}/{v}^{4} + {\ddot{q}}_{\max }} \\   - {\ddot{q}}_{\max } \leq  \sqrt{{v}_{k}{}^{4}/{v}^{4}/{v}^{4} + {g}^{\prime }{w}_{1}{a}^{k}} + q{\dot{l}}^{d} \leq  {\ddot{q}}_{\max } \\  {q}^{\prime }{}^{k}b \leq  {\ddot{q}}_{\max }^{2} \\  {b}^{\prime }{}^{k} - {\ddot{q}}_{1 - 1} + \frac{2}{3}{q}_{1 + 5} \\  {a}^{\prime }{}_{k} =  - \frac{2}{3}{q}_{1 - 1} + \frac{2}{3}{q}_{1 + 1} \\   - {a}^{\prime }{}_{k} =  - \frac{2}{3}{q}_{1 - 1} + \frac{2}{3}{q}_{1 + 1} \\   - {a}^{\prime }{}_{k} =  - {a}^{\prime }{}_{k} = 0 \\  {s}_{0} = 0,s = 1,s = 0 \\  {s}_{0} = 0,s = 1,s = 0 \\  {s}_{0} = 0,s = 1,s = 0 \end{array}\right. \]

(35)

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 7634 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING, VOL. 21, NO. 4, OCTOBER 2024-->

<!-- Media -->

---

Algorithm 1 B-Spline Based Time-Optimal and Jerk Con-
strained Trajectory Planning Method
		procedure Numerical Optimization
					Initialize the robot kinematic and dynamic parameters
			and motion path in Cartesian space.
						Discretize the path coordinate \( s \) with \( n + 1 \) grid points.
					\( \mathbf{q},{\mathbf{q}}^{\prime },{\mathbf{q}}^{\prime \prime },{\mathbf{q}}^{\prime \prime \prime } \leftarrow \) KINEMATIC
					\( \mathbf{m},\mathbf{c},\mathbf{g} \leftarrow  \operatorname{DYNAMIC}\left( {\mathbf{q},{\mathbf{q}}^{\prime },{\mathbf{q}}^{\prime \prime }}\right) \)
							Obtain \( {b}^{ * }\left( s\right) \) by solving the time-optimal
			problem (35) subject to merely torque constraints.
						Reconstruct the pseudo states with uniform cubic
			B-spline with (24),(26),(27) and (28).
						Formulate the time-optimal problem as (35)
					repeat
									Solve the NLP optimization problem with
			GUROBI.
					until \( \left| \delta \right|  \leq  1{e}^{-6} \)
						Compute time-optimal trajectory \( \mathbf{q}\left( t\right) ,\dot{\mathbf{q}}\left( t\right) ,\ddot{\mathbf{q}}\left( t\right) \) with
			the optimization result of (35).
			end procedure
			function KINEMATIC
					for \( k = 0 \) to \( n \) do
								Generate the position \( \mathbf{p}\left( {s}_{k}\right) \) and orientation \( \mathbf{R}\left( {s}_{k}\right) \)
			of manipulator in Cartesian space.
								Compute the position \( \mathbf{q}\left( {s}_{k}\right) \) in joint space through
			inverse kinematic.
								Compute \( {\mathbf{q}}_{k}^{\prime },{\mathbf{q}}_{k}^{\prime \prime },{\mathbf{q}}_{k}^{\prime \prime \prime } \) with difference method from
			\( {q}_{k} \) .
					end for
			end function
			function DynAMIC \( \left( {\mathbf{q},{\mathbf{q}}^{\prime },{\mathbf{q}}^{\prime \prime }}\right) \)
					for \( k = 0 \) to \( n \) do
								Compute the dynamic coefficient \( \mathbf{m}\left( {s}_{k}\right) ,\mathbf{c}\left( {s}_{k}\right) \) ,and
			\( \mathbf{g}\left( {s}_{k}\right) \) with (8),(9) and (10).
					end for
			end function

---

<!-- Media -->

The numerical method to solve the time-optimal problem follows the next steps of Algorithm 1.

1) In the Cartesian space, a predefined path with orientation is expressed with homogeneous matrix as \( \left\lbrack  \begin{matrix} \mathbf{R}\left( s\right) & \mathbf{p}\left( s\right) \\  0 & 1 \end{matrix}\right\rbrack \) . The position coordinates \( \mathbf{p}\left( s\right)  = \left\lbrack  {X\left( s\right) ,Y\left( s\right) ,Z\left( s\right) }\right\rbrack   \in \) \( {\mathbb{R}}^{3} \) is evaluated on the path coordinate grid and expressed as \( {X}^{k},{Y}^{k},{Z}^{k} \) . And the orientation matrix \( \mathbf{R}\left( s\right)  \in  {\mathbb{R}}^{3 \times  3} \) on the grids are expressed as \( {\mathbf{R}}^{k}\left( s\right) \) .

) Through the inverse kinematic of robot, the joint angles \( {\mathbf{q}}_{k} \) can be computed on each path coordinate grids with \( {X}^{k},{Y}^{k},{Z}^{k},{\mathbf{R}}^{k} \) . And then to generate the velocity and acceleration of joints in the state space, \( {\mathbf{q}}^{\prime },{\mathbf{q}}^{\prime \prime },{\mathbf{q}}^{\prime \prime \prime } \) are obtained by the interpolation with the parabola polynomials.

3) Substitute \( \left\{  {{\mathbf{q}}_{k},{\mathbf{q}}_{k}^{\prime },{\mathbf{q}}_{k}^{\prime \prime },k = 0,\cdots ,n + 1}\right\} \) into (8),(9) and (10),the dynamic matrices \( {\mathbf{m}}^{k} = \mathbf{m}\left( {s}_{k}\right) ,{\mathbf{c}}^{k} = \) \( \mathbf{c}\left( {s}_{k}\right) ,{\mathbf{g}}^{k} = \mathbf{g}\left( {s}_{k}\right) \) are obtained in a discretized form.

4) Reconstruct the original time-optimal problem with the convex formulation based on cubic B-spline. It is also discretized on the grid points in the coordinate space as shown in (26),(27) and (28). Thus,the control points \( {P}_{i} \) of cubic B-spline are treated as new decision states of optimization problem.

5) Generate the time-optimum trajectories subject to merely torque constraints and then obtain \( {b}^{ * }\left( s\right) \) . Replace the original jerk constraint in the time-optimal problem with the restricted ones in (34). Thus, the time-optimal trajectory planning problem is converted as a large scale problem as shown in (35). In this work, YALMIP optimization toolbox is employed to program the optimization problem and then it is solved with commercial numerical optimization solver GUROBI.

6) Based on the optimization result, the time-optimum pseudo velocity profile of the reformulated optimization problem is obtained. Then the motion profiles are calculated according to the optimization results including motion duration, joint velocity, joint acceleration, joint torques, etc.

## IV. CASE STUDIES

For industrial applications of robotic manipulators, the smoothness and computational efficiency of motion are both essential for the quality and quantity of manufacturing. In this section, three case studies of a generic 6-DoF serial manipulator are used to demonstrate the performance of the proposed method. Two case studies are contour following tasks which make the robot follows a butterfly-type path and a 'OPTEC'-type path. The other case study is a pick-and-place task which makes the robot execute a repetitive pick-and-place motion following a standard door-type path.

A general robotic manipulator UR5 from universal robot company which has a small size and payload up to \( 5\mathrm{\;{kg}} \) is used for case studies. In Table I, the kinematic and dynamic parameters are listed in detail. Furthermore, the physical limitations of the robot actuators are presented in detail at the end of Table I.

In this work, the trajectory optimization is ran at off-line mode on a workstation with an Intel Core i9-9900KF of \( {3.60}\mathrm{{GHz}} \) and \( {64}\mathrm{{GB}} \) RAM. YALMIP optimization toolbox is also employed to make the optimization problem programmed easily and quickly. In this work, GUROBI is employed to solve the NLP problem stated in (35) which shows better performance than the other commercial optimization solver MOSEK [30]. And the parameters of optimization solver is set up with default configurations in both case studies. The number of grid points is set as 1000 including 200 segments with 5 points at each interval. Through the RTDE (Real Time Data Exchange) interface of UR5, the optimized trajectories are executed at \( 8\mathrm{\;{ms}} \) servo cycle and the actual Cartesian positions and velocities are obtained. Therefore the actual accelerations and jerks of robot joints are obtained by taking the time derivative of velocities.

In this work, we compare the proposed method with the convex optimization method (TOPP-CO) [6]. It has utilized a transcription method to formulate the optimization problem as a SOCP problem. Thus it speeds up the optimization procedure to get a solution faster. However, since the jerk constraints are not considered in the TOPP-CO, the multiple-objective function incorporated with minimizing motion duration instead of introducing the jerk constraints directly. The referenced different objective functions include time-square of torque rate [6] and time-absolute of joint jerks [2]. The objective function is expressed with an extra jerk-related term as follows.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: JI et al.: CONVEX OPTIMIZATION METHOD TO TIME-OPTIMAL TRAJECTORY PLANNING WITH JERK CONSTRAINT 7635-->

\[J = T + {\gamma }_{1}{\int }_{0}^{1}\mathop{\sum }\limits_{{n = 0}}^{N}{\left( {\dot{\tau }}_{n}\right) }^{2}{ds} + {\gamma }_{2}{\int }_{0}^{1}\mathop{\sum }\limits_{{n = 0}}^{N}{\left( {\dddot{q}}_{n}\right) }^{2}{ds} \tag{36}\]

<!-- Media -->

TABLE I

The Kinematic and Dynamic Parameters and Actuation Limitations of UR5

<table><tr><td>Joint Number</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td></tr><tr><td>mass(kg)</td><td>3.7</td><td>8.393</td><td>2.275</td><td>1.219</td><td>1.219</td><td>0.1879</td></tr><tr><td>\( {\theta }_{i} \) (rad)</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td></tr><tr><td>\( {d}_{i}\left( \mathrm{{mm}}\right) \)</td><td>89.16</td><td>0</td><td>0</td><td>109.15</td><td>94.65</td><td>82.3</td></tr><tr><td>\( {\alpha }_{i} \) (rad)</td><td>0</td><td>\( \pi /2 \)</td><td>0</td><td>0</td><td>\( \pi /2 \)</td><td>\( - \pi /2 \)</td></tr><tr><td>\( {a}_{i}\left( \mathrm{{mm}}\right) \)</td><td>0</td><td>-425</td><td>-392.25</td><td>0</td><td>0</td><td>0</td></tr><tr><td>\( {p}_{x}\left( \mathrm{{mm}}\right) \)</td><td>0</td><td>-212.5</td><td>-272.32</td><td>0</td><td>0</td><td>0</td></tr><tr><td>\( {p}_{y}\left( \mathrm{{mm}}\right) \)</td><td>-1.93</td><td>0</td><td>0</td><td>16.34</td><td>16.34</td><td>0</td></tr><tr><td>\( {p}_{z}\left( \mathrm{{mm}}\right) \)</td><td>-26.51</td><td>113.36</td><td>26.5</td><td>-107.35</td><td>-1.8</td><td>-1.159</td></tr><tr><td>\( {\mathrm{I}}_{\mathrm{{xx}}}\left( {{10}^{-4}\mathrm{\;{kg}} \cdot  {\mathrm{m}}^{2}}\right) \)</td><td>84</td><td>78</td><td>16</td><td>16</td><td>16</td><td>1</td></tr><tr><td>\( {\mathrm{I}}_{\mathrm{{vv}}}\left( {{10}^{-4}\mathrm{\;{kg}} \cdot  {\mathrm{m}}^{2}}\right) \)</td><td>64</td><td>21</td><td>462</td><td>16</td><td>16</td><td>1</td></tr><tr><td>\( {\mathrm{I}}_{\mathrm{{zz}}}\left( {{10}^{-4}\mathrm{\;{kg}} \cdot  {\mathrm{m}}^{2}}\right) \)</td><td>84</td><td>21</td><td>462</td><td>9</td><td>9</td><td>1</td></tr><tr><td>\( {\mathrm{I}}_{\mathrm{{XV}}}\left( {{10}^{-4}\mathrm{\;{kg}} \cdot  {\mathrm{m}}^{2}}\right) \)</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td></tr><tr><td>\( {\mathrm{I}}_{\mathrm{{yz}}}\left( {{10}^{-4}\mathrm{\;{kg}} \cdot  {\mathrm{m}}^{2}}\right) \)</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td></tr><tr><td>\( {\mathrm{I}}_{\mathrm{{xz}}}\left( {{10}^{-4}\mathrm{\;{kg}} \cdot  {\mathrm{m}}^{2}}\right) \)</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td></tr><tr><td>\( {\tau }^{i}{}_{\max }\left( \mathrm{{Nm}}\right) \)</td><td>120</td><td>120</td><td>120</td><td>22.4</td><td>22.4</td><td>22.4</td></tr><tr><td>\( {\dot{q}}_{\max }^{i}\left( {{}^{ \circ  }/s}\right) \)</td><td>144</td><td>144</td><td>144</td><td>144</td><td>144</td><td>144</td></tr><tr><td>\( {\ddot{q}}_{\max }^{i}\left( {{}^{ \circ  }/{s}^{2}}\right) \)</td><td>2290</td><td>2290</td><td>2290</td><td>2290</td><td>2290</td><td>2290</td></tr><tr><td>\( {\dddot{q}}_{\max }^{i}\left( {{}^{ \circ  }/{s}^{3}}\right) \)</td><td>229000</td><td>229000</td><td>229000</td><td>229000</td><td>229000</td><td>229000</td></tr></table>

<!-- Media -->

where \( {\gamma }_{1} \) is the weight factor of the inclusion of the time-square torque rate and \( {\gamma }_{2} \) is the weight factor of the inclusion of time-square of absolute jerk. For its simplicity, while \( {\gamma }_{1} = 0,{\gamma }_{2} \neq  0 \) ,this method is referenced as time-jerk optimal trajectory planning method (JTOPP-CO). While \( {\gamma }_{1} \neq  0,{\gamma }_{2} = \) 0 , it is referenced as the time-torque-rate optimal trajectory planning method (TRTOPP-CO). Furthermore, to verify the effectiveness of the proposed method, B-spline based time-optimal planning methods [27] is conducted and referenced as TOPP-BS. Jerk-constrained TOPP-BS proposed in this work is references as JTOPP-BS. The comparison results with sequential convex programming [19] (TOPP-SCP) and the linear acceleration-constrained method [17] (TOPP-LAC) are included as well. To get the time-optimum results, TOPP-LAC is formulated as convex optimization problem in which the third order derivative \( \dddot{s} \) is constrained according the key idea in [17] and solved with sequential convex programming method. Furthermore, energy-based objective function \( {\int }_{0}^{1}\mathop{\sum }\limits_{{n = 0}}^{N}{\left( {\tau }_{n}\right) }^{2}{ds} \) is also verified in case studies. Compared with the other jerk-constrained objective function, it weaken the time optimality dramatically while reduce the jerk of motion since energy consumption of robot joints is constrained strictly. We also reproduce the optimal results with Dynamic programming method in [20] which no feasible trajectories are obtained in the case studies. Therefore, their results are not listed in comparison. TOPP-CO serves as a reference method for comparison purposes in the context of our study which is globally time-optimal with torque, velocity and acceleration constraints. Thus it is treated as the baseline compared with the proposed jerk-constrained method and the other jerk constrained optimization methods.

<!-- Media -->

TABLE II

COMPARISON RESULTS OF OPTIMUM TIME, COMPUTATION TIME, MAXIMUM JERK OF ROBOT MANIPULATOR ALONG THE BUTTERFLY-TYPE PATH WITH DIFFERENT OPTIMIZATION Methods. The Jerk of Each Joint Is Uniformed WITH SETUP JERK CONSTRAINT IN TABLE I. \( \dddot{q} \) REPRESENTS THE MAXIMUM JERK \( {\dddot{q}}_{\max } \)

<table><tr><td rowspan="2">Method</td><td rowspan="2">Optimum Time(s)</td><td rowspan="2">Computation Time(s)</td><td colspan="6">\( \max {\ddot{q}}_{i}/\left| {\ddot{q}}_{i}\right| \)</td></tr><tr><td>J1</td><td>J2</td><td>J3</td><td>J4</td><td>J5</td><td>J6</td></tr><tr><td>TOPP-CO</td><td>2.6554</td><td>0.6174</td><td>7.47</td><td>7.06</td><td>6.47</td><td>2.03</td><td>0.00</td><td>7.47</td></tr><tr><td>TOPP-SCP</td><td>2.66</td><td>4.7895</td><td>1.0</td><td>1.0</td><td>1.0</td><td>0.35</td><td>0.00</td><td>1.0</td></tr><tr><td>TOPP-LAC</td><td>3.1252</td><td>5.842</td><td>0.52</td><td>0.82</td><td>1.00</td><td>0.27</td><td>0.00</td><td>0.52</td></tr><tr><td>TOPP-BS</td><td>2.8198</td><td>0.5021</td><td>0.95</td><td>1.27</td><td>1.24</td><td>0.33</td><td>0.00</td><td>0.95</td></tr><tr><td>JTOPP-CO</td><td>2.9574</td><td>2.0732</td><td>3.00</td><td>3.23</td><td>3.75</td><td>1.04</td><td>0.00</td><td>3.00</td></tr><tr><td>TRTOPP-CO</td><td>2.8075</td><td>1.4533</td><td>1.60</td><td>1.60</td><td>1.85</td><td>0.50</td><td>0.00</td><td>1.60</td></tr><tr><td>JTOPP-BS</td><td>3.1544</td><td>1.4638</td><td>0.40</td><td>0.44</td><td>0.39</td><td>0.19</td><td>0.00</td><td>0.40</td></tr></table>

<!-- Media -->

## A. Butterfly-Type Contour Following Task

In this section, a contour following task of robot manipulators is conducted. The butterfly-type path, which is widely used as test path in CNC trajectory planning research [31], is utilized to verify the effectiveness of the proposed method. The optimization results are presented in Table II. The trajectories optimized by TOPP-CO are time-optimum with the limitations of torque, velocity and acceleration of robot joints. The obtained maximum jerk is over 7 times of jerk restrictions in Table I. Thanks to the hull property of B-spline, jerks of optimized trajectories by TOPP-BS are also bounded. JTOPP-BS which is TOPP-BS with jerk constraints shows a clear restriction of maximum jerk to get a smoother trajectories in Fig 14. To compare with the other jerk constrained optimization methods, the weight factors of JTOPP-CO and TRTOPP-CO are set as \( {\gamma }_{1} = {10}^{-2},{\gamma }_{2} = {10}^{-5} \) . They are chosen by simulation to obtain a close optimum time compared with JTOPP-BS. The numerical results show the effectiveness of jerk constrained trajectories with JTOPP-CO and TRTOPP-CO compared with TOPP-CO. The limitation of \( \ddot{s} \) is set as \( \pm  {59.5}{\mathrm{\;s}}^{-3} \) which is chosen to obtain time-optimal results just right without violating the jerk constraints. Due to the recursively computation through SCP, the computation time with both TOPP-SCP and TOPP-LAC increases multiple times greater than TOPP-CO. However, obtaining efficient and robust optimization results are crucial in real industrial applications. Therefore, due to the convex restriction of jerk constraints, computational efficiency is maintained with the proposed method. The results obtained with JTOPP-BS demonstrate that it sacrifices some time optimality to achieve smoothness and computational efficiency in the time-optimal and jerk-constrained optimization problem. The results in Table II are illustrated in a more clear way with bar chart in Fig.2. The trajectories generated using different methods exhibit distinct results in terms of maximum jerk. The results with TOPP-SCP is the time-optimum without violating the jerk constraints. Through indirect jerk constraints with TOPP-LAC, its results are sub-optimal without violating the jerk constraints. However, compared with JTOPP-BS and similar to TOPP-CO, its control inputs are bang-bang-like, lacking consideration for the overall smoothness of trajectories, especially at the start and end stages of motion. This omission heightens the potential for vibrations in the optimized trajectory when executed on actual robotic systems. In comparison to alternative approaches, JTOPP-BS demonstrates the most favorable outcomes, further constraining jerk to achieve the expected results in a computationally efficient manner.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 7636 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING, VOL. 21, NO. 4, OCTOBER 2024-->

<!-- Media -->

<!-- figureText: 6 J1 J2 J3 J4 J5 J6 TOPP-BS JTOPP-CC TRTOPP \( {\dddot{q}}_{i}/\left| {\dddot{q}}_{i}\right| \) 2 TOPP-CO TOPP-SCP TOPP-LAC -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_7.jpg?x=158&y=149&w=703&h=426&r=0"/>

Fig. 2. Bar chart of the results maximum uniformed jerks of robot joints along butterfly-type path with different methods.

<!-- Media -->

The pseudo-velocity profiles obtained using various trajectory optimization methods with different jerk constraints are illustrated in Fig.3. The solid red line corresponds to the maximum velocity profile (MVC), which represents the shortest motion time subject to velocity, acceleration, and torque constraints. The area enclosed by the pseudo-velocity profile reflects the duration of the motion, with larger areas indicating shorter motion times. Comparatively, the pseudo-velocity profile generated by the TOPP-CO method, represented by the blue dashed line, exhibits the largest enclosed area and thus yields the shortest optimization time in Table II. In contrast, the pseudo-velocity profiles obtained from other methods lie below the TOPP-CO profile, providing further confirmation of the correctness of the optimization results.

An in-depth analysis reveals that trajectory optimization methods based on B-splines, including TOPP-BS and JTOPP-BS, generate pseudo-velocity profiles characterized by second-order continuous smoothness. These profiles exhibit smoother starting and stopping segments compared to the other methods. However, the effectiveness of the JTOPP-CO method is comparatively inferior as it fails to achieve both time optimality and trajectory smoothness. Additionally, in this case study, TOPP-BS and TOPP-CO exhibit similar pseudo-velocity profiles. Nevertheless, TOPP-BS imposes more constraints at specific positions, resulting in smaller variations in pseudo-velocity for the same \( {\Delta s} \) ,particularly at \( \mathrm{s} = {0.32},{0.35} \) ,and 0.54 . This phenomenon arises due to the constraint imposed by the convex hull property of B-spline on the curvature of the pseudo-velocity profile, leading to insufficient pseudo-acceleration at these positions. Furthermore, the black solid line represents the results obtained using JTOPP-BS, which enforces stricter constraints on the pseudo-velocity profile compared to TOPP-BS, providing additional evidence for the effectiveness of the proposed jerk constrained method. Interestingly, an apparent contradiction arises when examining the pseudo-velocity profile of TRTOPP-CO, depicted by the green solid line. Despite visually encompassing a larger area, the optimized minimum time is only \( {2.8075}\mathrm{\;s} \) ,whereas JTOPP-BS requires \( {3.1544}\mathrm{\;s} \) . Further analysis reveals that the smoother starting and stopping segments of the JTOPP-BS profile significantly contribute to this difference.

Fig. 4 depicts the numerical results of robot joint velocity, acceleration, and jerk obtained using different optimization methods, represented in a normalized manner. The black dotted lines in the figure represent the normalized constraints. It can be observed from the figure that the optimized velocity and acceleration satisfy the imposed constraints, while TOPP-LAC, TOPP-SCP and JTOPP-BS meet the constraints on jerk. Additionally, at the starting and ending points of the velocity and acceleration curves, only JTOPP-BS satisfy the zero initial boundary conditions. In comparison, the trajectory obtained through the B-spline-based optimization methods appears to be smoother. Although the jerk curve based on JTOPP-BS satisfies the constraints, it does not reach the boundary of the jerk constraint. This difference arises from the convex restriction approach employed for jerk,where \( {b}^{ * }\left( s\right) \) is greater than the actual \( b\left( s\right) \) ,resulting in a smaller maximum jerk value obtained through the solution. Hence, the proposed method in this study provides a near time-optimal result under the jerk constraints. This finding also highlights an area for future improvement and refinement.

In the experimental setup, the optimized robot joint positions were interpolated using a cubic spline interpolation algorithm and inputted to the UR5 robot motion controller as the command trajectory at a frequency of \( {125}\mathrm{{Hz}} \) . The UR5 utilized the servoj command to receive and execute the command trajectory points, employing a lookahead and proportional position tracking control method internally. In case studies of this paper, to ensure optimal trajectory tracking performance, the parameters were set to their permissible limits, with a lookahead parameter of 0.03 and a proportional gain of 2000. Fig. 5 illustrates the actual TCP (Tool Center Point) position in Cartesian space obtained using different jerk-constrained optimization methods, with the solid red line representing the reference path.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: JI et al.: CONVEX OPTIMIZATION METHOD TO TIME-OPTIMAL TRAJECTORY PLANNING WITH JERK CONSTRAINT 7637-->

<!-- Media -->

<!-- figureText: MVC TOPP-BS JTOPP-CO TOPP-LAC JTOPP-BS TRTOPP-CO TOPP-SCP 0.5 0.55 0.6 0.65 0.7 0.75 0.8 0.85 0.9 0.95 1 path coordinate (-) 0.8 TOPP-CO (1/spseudo-velocity (1/s) 0.6 0.4 0.2 0 0.05 0.1 0.15 0.2 0.25 0.3 0.35 0.4 0.45 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_8.jpg?x=204&y=147&w=1371&h=504&r=0"/>

Fig. 3. The comparison result of the pseudo velocity along the butterfly-type path with different methods.

<!-- figureText: \( {\dot{q}}_{1}/\left| {\dot{q}}_{1}\right| \) \( {\ddot{q}}_{1}/\left| {\overline{\ddot{q}}}_{1}\right| \) \( {\ddot{q}}_{1}/\left| {\ddot{q}}_{1}\right| \) \( {\ddot{q}}_{2}/\left| {\ddot{q}}_{2}\right| \) \( {\ddot{q}}_{3}/\left| {\ddot{q}}_{3}\right| \) \( {\ddot{q}}_{4}/\left| {\overline{\ddot{q}}}_{4}\right| \) \( {\ddot{q}}_{5}/\left| {\ddot{q}}_{5}\right| \) \( {\ddot{q}}_{6}/\left| {\ddot{q}}_{6}\right| \) 0.6 0.8 0 0.2 0.4 0.6 0.8 (b) (c) -1 -1 \( {\dot{q}}_{2}/\left| {\dot{q}}_{2}\right| \) \( {\ddot{q}}_{2}/\left| {\ddot{q}}_{2}\right| \) -1 -1 \( {\dot{q}}_{3}/\left| {\dot{q}}_{3}\right| \) \( {\ddot{q}}_{3}/\left| {\ddot{q}}_{3}\right| \) -1 -1 1 \( {\dot{q}}_{4}/\left| {\overline{\dot{q}}}_{4}\right| \) \( {\ddot{q}}_{4}/\left| {\overline{\ddot{q}}}_{4}\right| \) -1 -1 1 \( {\dot{q}}_{5}/\left| {\dot{q}}_{5}\right| \) \( {\ddot{q}}_{5}/\left| {\ddot{q}}_{5}\right| \) -1 - 1 \( {\dot{q}}_{6}/\left| {\dot{q}}_{6}\right| \) \( {\ddot{q}}_{6}/\left| {\overline{\ddot{q}}}_{6}\right| \) -1 0.2 0.4 0.6 0.8 0 0.2 0.4 (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_8.jpg?x=239&y=732&w=1310&h=1164&r=0"/>

Fig. 4. The numerical results of velocity, acceleration and jerk of robot joints compared with different methods. TOPP-CO is marked with blue dashed line. TOPP-BS is marked with magenta dotted line. JTOPP-CO is marked with green solid line. TRTOPP-CO is marked with cyan dash-dotted line. JTOPP-BS is marked with black solid line. TOPP-SCP is marked with yellow dotted line. TOPP-LAC is marked with purple dotted line. In the rest of paper, these methods use the same marks. (a) joint velocity. (b) joint acceleration. (c) joint jerk.

<!-- Media -->

Fig.5a illustrates that the paths in Cartesian space obtained by various methods are relatively close to each other, but there remains a maximum tracking error of approximately \( 5\mathrm{\;{mm}} \) with respect to the reference path in a more clear view in Fig.5c. The lookahead functionality in the UR5 motion controller significantly influences the trajectory tracking performance, manifesting in two aspects. Firstly, the lookahead algorithm acts as a smoothing filter for the input command trajectory, resulting in smoothed trajectories with zero velocity and acceleration at the starting point and ensuring sufficiently smooth motion that can be executed normally. Secondly, the lookahead algorithm introduces a delay between the actual and command trajectories, with the delay time approximately equal to the set lookahead parameter time, which in this case is \( {30}\mathrm{\;{ms}} \) . This delay leads to significant tracking errors in the actual joint trajectories. Fig. 6 depicts the actual joint velocity, acceleration, and jerk trajectories in the experiment. The trajectories reveals that both velocity and acceleration of robot joints exhibit overshoot, exceeding their constraints. The main reasons for these results are caused by the relatively long control period and the absence of a differential component in the position tracking controller. However, these differences did not trigger any alarms in the UR robot, except for TOPP-CO and TOPP-SCP. Both of their results caused a protective warning, and the UR5 stopped in the middle of the path. Therefore, the acceleration constraints were reduced to 90% to avoid triggering a UR5 warning. Consequently, the optimum time with TOPP-CO and TOPP-SCP was prolonged to 2.7676s and 2.7741s. Due to the effect of the lookahead smoothing filter, the actual jerk values are noticeably reduced compared to the simulation results. Nevertheless, the jerk values obtained using the proposed JTOPP-BS method remain within the minimum range in comparison.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 7638 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING, VOL. 21, NO. 4, OCTOBER 2024-->

<!-- Media -->

<!-- figureText: Spatial trajectory Trajectory in the plane parallel to the XY-plane Trajectory in the plane parallel to the XZ-plane 0.305 0.304 z-coordinate (m) 0.301 0.3 0.299 0 0.2 -0.15 -0.1 -0.05 0.05 x-coordinate (m) x-coordinate (m) (c) 0.6 0.55 z-coordinate (m) 0.31 y-coordinate (m) 0.45 0.35 0.3 0.3 0.29 -0.3 -0.2 -0.1 0.4 0.2 0.1 0.3 x-coordinate (m) 0.2 y-coordinate (m) -0.3 -0.2 0.2 (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_9.jpg?x=176&y=149&w=1449&h=406&r=0"/>

Fig. 5. The experiment results of contour tracking with different methods. The curve marked with red line is referenced path. (a) 3D-view. (b) XY-view. (c) XZ-view.

<!-- figureText: \( {\dot{q}}_{1}/\left| {\overline{\dot{q}}}_{1}\right| \) \( {\ddot{q}}_{1}/\left| {\ddot{q}}_{1}\right| \) \( {\ddot{q}}_{1}/\left| {\ddot{q}}_{1}\right| \) 1 \( {\overset{...}{q}}_{2}/|{\overset{...}{q}}_{2}| \) -1 1 \( {\dddot{q}}_{3}/\left| {\dddot{q}}_{3}\right| \) -1 1 \( {\ddot{q}}_{4}/\left| {\ddot{q}}_{4}\right| \) -1 \( {\ddot{q}}_{5}/\left| {\ddot{q}}_{5}\right| \) -1 \( {\ddot{q}}_{6}/\left| {\ddot{q}}_{6}\right| \) 0.6 0.8 1 0.2 0.4 0.6 0.8 1 (b) (c) 1 \( {\dot{q}}_{2}/\left| {\dot{q}}_{2}\right| \) \( {\ddot{q}}_{2}/\left| {\overline{\ddot{q}}}_{2}\right| \) -1 1 \( {\dot{q}}_{3}/\left| {\overline{\dot{q}}}_{3}\right| \) \( {\overset{..}{q}}_{3}/\left| {\overset{..}{q}}_{3}\right| \) 1 \( {\dot{q}}_{4}/\left| {\overline{\dot{q}}}_{4}\right| \) \( {\ddot{q}}_{4}/\left| {\ddot{q}}_{4}\right| \) -1 \( {\overset{ \cdot  }{q}}_{5}/\left| {\overset{ \cdot  }{\overset{ \cdot  }{q}}}_{5}\right| \) \( {\ddot{q}}_{5}/\left| {\overline{\ddot{q}}}_{5}\right| \) 0 - 1 1 \( {\dot{q}}_{6}/\left| {\overline{\dot{q}}}_{6}\right| \) \( {\ddot{q}}_{6}/\left| {\ddot{q}}_{6}\right| \) 0 0.2 0.4 0.6 0.8 0.2 0.4 (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_9.jpg?x=211&y=651&w=1369&h=1226&r=0"/>

Fig. 6. The experiment results of actual velocity, acceleration and jerk of robot joints compared with different methods. (a) actual joint velocity. (b) actual joint acceleration. (c) actual joint jerk.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: JI et al.: CONVEX OPTIMIZATION METHOD TO TIME-OPTIMAL TRAJECTORY PLANNING WITH JERK CONSTRAINT 7639-->

<!-- figureText: MVC IIII TOPP-BS JTOPP-CO TOPP-LAC JTOPP-BS TRTOPP-CO TOPP-SCP 0.5 0.55 0.6 0.65 0.7 0.75 0.8 0.85 0.9 0.95 1 path coordinate (-) pseudo-velocity (1/s) TOPP-CO 0.8 0.6 0.4 0.2 0 0 0.05 0.1 0.15 0.2 0.25 0.3 0.35 0.4 0.45 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_10.jpg?x=224&y=147&w=1334&h=488&r=0"/>

Fig. 7. The comparison result of the pseudo velocity along the 'OPTEC'-type path with different methods.

TABLE III

COMPARISON RESULTS OF OPTIMUM TIME, COMPUTATION TIME, MAXIMUM JERK OF ROBOT MANIPULATOR ALONG THE ‘OPTEC’-TYPE PATH WITH DIFFERENT OPTIMIZATION METHODS. The Jerk of Each Joint Is Uniformed With Setup Jerk CONSTRAINT IN TABLE I. \( \dddot{q} \) REPRESENTS THE MAXIMUM JERK \( {\dddot{q}}_{\max } \)

<table><tr><td rowspan="2">Method</td><td rowspan="2">Optimum Time(s)</td><td rowspan="2">Computation Time(s)</td><td colspan="6">\( \max {\ddot{q}}_{i}/\left| {\ddot{q}}_{i}\right| \)</td></tr><tr><td>J1</td><td>J2</td><td>J3</td><td>J4</td><td>J5</td><td>J6</td></tr><tr><td>TOPP-CO</td><td>3.1524</td><td>0.2993</td><td>11.3</td><td>11.9</td><td>13.85</td><td>7.5</td><td>0.00</td><td>11.3</td></tr><tr><td>TOPP-SCP</td><td>3.2597</td><td>3.328</td><td>1.00</td><td>0.97</td><td>1.0</td><td>0.71</td><td>0.00</td><td>1.00</td></tr><tr><td>TOPP-LAC</td><td>3.6462</td><td>1.9811</td><td>0.42</td><td>0.76</td><td>1.0</td><td>0.28</td><td>0.00</td><td>0.42</td></tr><tr><td>TOPP-BS</td><td>3.4236</td><td>0.8128</td><td>3.01</td><td>5.30</td><td>7.2</td><td>2.84</td><td>0.00</td><td>3.01</td></tr><tr><td>JTOPP-CO</td><td>3.4302</td><td>0.5577</td><td>8.89</td><td>9.63</td><td>11.3</td><td>6.04</td><td>0.00</td><td>8.89</td></tr><tr><td>TRTOPP-CO</td><td>3.5719</td><td>0.6835</td><td>1.00</td><td>1.16</td><td>1.84</td><td>0.58</td><td>0.00</td><td>1.00</td></tr><tr><td>JTOPP-BS</td><td>3.731</td><td>1.0933</td><td>0.69</td><td>0.56</td><td>0.67</td><td>0.29</td><td>0.00</td><td>0.69</td></tr></table>

<!-- Media -->

## B. 'OPTEC'-Type Contour Following Task

To further verify the effectiveness of the proposed method, another contour following task along a more complicated 'OPTEC'-type path [6] is conducted. The optimization results, compared with other methods, are presented in Table III. The maximum uniformed jerk of robot joints is also illustrated in Fig.8. The weight factors of JTOPP-CO and TRTOPP-CO are set as \( {\gamma }_{1} = {10}^{-2} \) and \( {\gamma }_{2} = {10}^{-5} \) ,the same as in the butterfly-type contour following task. Furthermore, the limitation of \( \dddot{s} \) for TOPP-LAC is set as \( \pm  {113}{\mathrm{\;s}}^{-3} \) ,chosen to obtain time-optimal results precisely without violating the jerk constraints.

<!-- Media -->

<!-- figureText: \( {\ddot{q}}_{i}/\left| {\ddot{q}}_{i}\right| \) JJ J2 J3 3.14 .15 ]J6 TOPP-BS JTOPP-CO ROPP TOPP-CO TOPP-SCP TOPP-LAC -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_10.jpg?x=934&y=734&w=698&h=424&r=0"/>

Fig. 8. Bar chart of the results of maximum uniformed jerk of robot joints along 'OPTEC'-type path with different methods.

<!-- Media -->

The results in Table III demonstrate optimization outcomes consistent with the butterfly-type path. The global optimum time is achieved by TOPP-SCP while satisfying jerk constraints. In comparison to the optimization results obtained in the butterfly-type contour following task under the same optimization parameters, TRTOPP-CO and JTOPP-CO still fail to meet the specified jerk constraints. TOPP-SCP and TOPP-LAC, due to their iterative approximation approach to finding the optimal solution, still require multiple times the computation time of TOPP-CO. Therefore, the proposed JTOPP-BS method in this paper achieves optimization results that satisfy jerk constraints while also approaching nearly the global optimum time. Compared to the optimum time obtained with TOPP-SCP in the butterfly-type contour following task, the optimum time obtained with the JTOPP-BS method is extended by 18.59%. However, in this case study, this time is only extended by 14.46%. The improvement in time optimality is mainly due to the reduced time consumed in the initial and final segments of the motion. This can be observed in the comparison of pseudo-velocity along the corresponding paths in Fig. 3 and Fig. 7. The comparison results with TOPP-CO and TOPP-SCP illustrate that the pseudo velocity at \( s = 0 \) and \( s = 1 \) is approximately 0.4 in Fig. 7,slightly smaller than the pseudo velocity along the butterfly-type path in Fig. 3.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 7640 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING, VOL. 21, NO. 4, OCTOBER 2024-->

<!-- Media -->

<!-- figureText: \( {\dot{q}}_{1}/\left| {\dot{q}}_{1}\right| \) \( {\ddot{q}}_{1}/\left| {\ddot{q}}_{1}\right| \) \( {\ddot{q}}_{1}/\left| {\ddot{q}}_{1}\right| \) \( {\ddot{q}}_{2}/\left| {\ddot{q}}_{2}\right| \) \( {\ddot{q}}_{3}/\left| {\ddot{q}}_{3}\right| \) \( {\ddot{q}}_{4}/\left| {\ddot{q}}_{4}\right| \) 1 \( {\ddot{q}}_{5}/\left| {\ddot{q}}_{5}\right| \) \( {\ddot{q}}_{6}/\left| {\ddot{q}}_{6}\right| \) 0.6 0.8 0.2 0.4 0.6 0.8 1 (b) (c) -1 -1 1 \( {\dot{q}}_{2}/\left| {\dot{q}}_{2}\right| \) \( {\ddot{q}}_{2}/\left| {\ddot{q}}_{2}\right| \) -1 -1 1 \( {\dot{q}}_{3}/\left| {\dot{q}}_{3}\right| \) 0 \( {\ddot{q}}_{3}/\left| {\overline{\ddot{q}}}_{3}\right| \) -1 \( {\dot{q}}_{4}/\left| {\dot{q}}_{4}\right| \) 0 \( {\ddot{q}}_{4}/\left| {\ddot{q}}_{4}\right| \) 1 \( {\dot{q}}_{5}/\left| {\dot{q}}_{5}\right| \) 0 \( {\ddot{q}}_{5}/\left| {\overline{\ddot{q}}}_{5}\right| \) -1 -1 1 \( {\dot{q}}_{6}/\left| {\dot{q}}_{6}\right| \) \( {\ddot{q}}_{6}/\left| {\ddot{q}}_{6}\right| \) -1 0 0.2 0.4 0.6 0.8 1 0 0.2 0.4 (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_11.jpg?x=172&y=154&w=1448&h=1286&r=0"/>

Fig. 9. The numerical results of velocity, acceleration and jerk of robot joints compared with different methods. (a) joint velocity. (b) joint acceleration. (c) joint jerk.

<!-- Media -->

Furthermore, it can be observed that the time optimality of TOPP-BS is less affected by the convex hull characteristics of the B-spline, allowing it to better approximate the global optimum solution of TOPP-CO. It can also be noticed that the pseudo velocity of JTOPP-BS is smaller than the pseudo velocity profile obtained by TOPP-SCP for the global optimum,especially in the region from \( s = {0.3} \) to \( s = {0.42} \) . This indicates the negative impact of the convex restriction method on the time optimality.

The numerical and experimental results with different optimization methods are illustrated in Fig.9, Fig.10 and Fig.11. The numerical results in Fig.9 indicate that all optimization methods can satisfy velocity and acceleration constraints, as concluded in the numerical results of the butterfly-type contour following task. Meanwhile, Fig.9c illustrates the trajectory of joint jerk acceleration, showing that only the optimization results obtained with TOPP-LAC, TOPP-SCP, and JTOPP-BS do not violate jerk acceleration constraints. Since JTOPP-BS is based on a B-spline, it exhibits smoother trajectories compared to the other two methods. The numerical results in the butterfly-type contour following task also support the same conclusion. However, due to the convex restriction method making the original jerk constraints more stringent, this results in jerk acceleration not reaching the constraint boundaries, implying that JTOPP-BS sacrifices some time optimality. Therefore, in the future, efforts should be directed towards further improving the time optimality of the optimization method while maintaining trajectory smoothness and computational efficiency.

As shown in Fig.10, experimental results of 'OPTEC'-type contour following task indicate a maximum tracking error of approximately \( 5\mathrm{\;{mm}} \) . In Fig.10c,the maximum path tracking error for TOPP-CO which is marked with the blue dashed line,slightly exceeding \( 5\mathrm{\;{mm}} \) . Assessing the deviation from the commanded path, the black solid line representing JTOPP-BS exhibits a slightly smaller path tracking error compared to TOPP-SCP and TOPP-LAC. The path tracking results in the butterfly-type contour following task also confirm the same conclusion. Nevertheless, due to the smoothing effect of the lookahead function of the UR robot, optimization methods that do not consider trajectory smoothness, such as TOPP-SCP and TOPP-CO, can still be executed on a real robot without triggering alarms. In this case study, despite the 'optec'-type path being more complex and less smooth compared to the butterfly-type path, and in contrast to the previous results from the butterfly-type contour tracking task, neither TOPP-CO nor TOPP-SCP triggered any protective warning and stopped UR5. The specific reasons for triggering alarms will be further investigated in future research. Additionally, the actual joint trajectories in Fig. 11 show instances where velocity and acceleration slightly exceed the specified constraints; however, this does not trigger alarms on the robot. Similar to the butterfly-type contour following task, the maximum joint jerks are significantly reduced in Fig.11c, which might be another reason the trajectory can be executed normally on a real robot. However, this also diminishes the advantages of trajectory smoothness proposed by the methods JTOPP-BS and TOPP-BS. Therefore, in future research, the proposed methods in this paper will undergo further validation on the other robots.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: JI et al.: CONVEX OPTIMIZATION METHOD TO TIME-OPTIMAL TRAJECTORY PLANNING WITH JERK CONSTRAINT 7641-->

<!-- Media -->

<!-- figureText: Spatial trajectory Trajectory in the plane parallel to the XY-plane Trajectory in the plane parallel to the XZ-plane 0.405 0.4045 0.404 z-coordinate (m) 0.403 0.402 0.4015 0.401 0.400£ 0.4 0 -0.15 0.1 0.15 0.2 x-coordinate (m) x-coordinate (m) (b) (c) 0.75 0.7 z-coordinate (m) 0.405 y-coordinate (m) 0.65 0.6 0.395 -0.2 0.5 0.6 0.45 0.5 0.4 x-coordinate (m) 0.2 y-coordinate (m) -0.2 (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_12.jpg?x=177&y=150&w=1443&h=399&r=0"/>

Fig. 10. The experiment results of contour tracking with different methods. The curve marked with red line is referenced path. (a)3D-view. (b)XY-view. (c) XZ-view.

<!-- figureText: \( {\overset{ \cdot  }{q}}_{1}/|{\overset{ \cdot  }{q}}_{1} \) 0 \( {\overset{..}{q}}_{1}/\left| {\overline{\overset{\ldots }{q}}}_{1}\right| \) \( {\dddot{q}}_{1}/\left| {\dddot{q}}_{1}\right| \) -1 1 \( {\ddot{q}}_{2}/\left| {\ddot{q}}_{2}\right| \) -1 1 \( {\ddot{q}}_{3}/\left| {\ddot{q}}_{3}\right| \) -1 \( {\ddot{q}}_{4}/\left| {\ddot{q}}_{4}\right| \) - 1 1 \( {\ddot{q}}_{5}/\left| {\ddot{q}}_{5}\right| \) 0 1 \( {\ddot{q}}_{6}/\left| {\ddot{q}}_{6}\right| \) - 1 0.6 0.8 1 0 0.2 0.4 0.6 0.8 (b) (c) 1 \( {\overset{ \cdot  }{q}}_{2}/\left| {\overset{ \cdot  }{q}}_{2}\right| \) \( {\overset{..}{q}}_{2}/\left| {\overset{..}{\overset{..}{q}}}_{2}\right| \) -1 -1 1 \( {\dot{q}}_{3}/\left| {\dot{q}}_{3}\right| \) \( {\ddot{q}}_{3}/\left| {\overline{\ddot{q}}}_{3}\right| \) - 1 -1 \( {\dot{q}}_{4}/\left| {\dot{q}}_{4}\right| \) 0 \( {\ddot{q}}_{4}/\left| {\overline{\ddot{q}}}_{4}\right| \) -1 -1 1 \( {\overset{ \cdot  }{q}}_{5}/\left| {\overset{ \cdot  }{q}}_{5}\right| \) 0 \( {\overset{..}{q}}_{5}/\left| {\overset{.}{\overset{\ldots }{q}}}_{5}\right| \) 1 \( {\dot{q}}_{6}/\left| {\dot{q}}_{6}\right| \) \( {\ddot{q}}_{6}/\left| {\overline{\ddot{q}}}_{6}\right| \) -1 -1 0.2 0.4 0.6 0.8 0.2 (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_12.jpg?x=173&y=643&w=1447&h=993&r=0"/>

Fig. 11. The experiment results of actual velocity, acceleration and jerk of robot joints compared with different methods. (a) actual joint velocity. (b) actual joint acceleration. (c) actual joint jerk.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 7642 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING, VOL. 21, NO. 4, OCTOBER 2024-->

<!-- figureText: 1.2 MVC Pseudo-velocity TOPP-BS JTOPP-CO TOPP-LAC JTOPP-BS TRTOPP-CO TOPP-SCP 0.5 0.55 0.6 0.65 0.7 0.75 0.8 0.85 0.9 0.95 1 path coordinate (-) pseudo-velocity (1/s) 1 TOPP-CO 0.8 0.6 0.4 0.2 0 0 0.05 0.1 0.15 0.2 0.25 0.3 0.35 0.4 0.45 -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_13.jpg?x=231&y=148&w=1333&h=477&r=0"/>

Fig. 12. The comparison result of the pseudo velocity along the standard 100-600-100mm door-type path with different methods.

<!-- Media -->

## C. Pick-and-Place Task

The pick and place task is widely considered as one of the most commonly performed tasks in industrial robot applications. The efficiency and stability demonstrated by industrial robots are key factors contributing to their extensive use in material handling scenarios. Achieving time optimality within the performance limits of robots is an important aspect pursued by robot manufacturers. Therefore, apart from complex path tracking tasks above, this paper focuses on investigating the pick and place task as another case study.

To ensure consistency in the comparison of different methods, a door-type path was pre-programmed in the UR controller. The program set the mixed radius to \( {50}\mathrm{\;{mm}} \) ,the distance in the \( \mathrm{Z} \) direction to \( {100}\mathrm{\;{mm}} \) ,and in the \( \mathrm{X} \) direction to \( {600}\mathrm{\;{mm}} \) . In addition to the jerk-constrained optimization methods discussed earlier, the results of the S-curve planning method, which is an inherent planning method within the UR5 controller itself, were also considered for comparison. The motion parameters for the S-curve method were set to their maximum values,with a speed of \( {1000}\mathrm{\;{mm}}/\mathrm{s} \) and an acceleration of \( {10000}\mathrm{\;{mm}}/{\mathrm{s}}^{2} \) .

Analyzing the data presented in Table IV, it is evident that the motion time obtained using the S-curve method significantly differs from the results obtained using the optimization-based methods. Furthermore, the jerk constraints are not considered in the S-curve method. Comparing the results in Table IV, it can be observed that the optimization results generally correspond to the results along butterfly-type path.

Although JTOPP-CO and TRTOPP-CO still employ weight coefficients of \( {\gamma }_{1} = {10}^{-2},{\gamma }_{2} = {10}^{-5} \) ,respectively,the optimization results slightly differ from those obtained for the butterfly-type path. Notably, JTOPP-CO yields a longer optimum time compared to JTOPP-BS. The limitation of \( \ddot{s} \) in this case for TOPP-LAC is set as \( \pm  {670}{\mathrm{\;s}}^{-3} \) . However, when compared to the butterfly-type path, the optimum time obtained with TOPP-LAC is close to TOPP-CO and TOPP-SCP. We think it is because the limitation of \( \dddot{s} \) is loose enough.

<!-- Media -->

## TABLE IV

COMPARISON RESULTS OF OPTIMUM TIME, COMPUTATION TIME, MAXIMUM JERK OF ROBOT MANIPULATOR ALONG THE DOOR-TYPE PATH WITH DIFFERENT OPTIMIZATION METHODS. THE JERK OF EACH JOINT IS UNIFORMED WITH SETUP JERK CONSTRAINT IN TABLE I. \( \dddot{q} \) REPRESENTS THE MAXIMUM JERK \( {\dddot{q}}_{\max } \) .THE RESULT OF THE S-CURVE METHOD IS OBTAINED FROM UR5 CONTROLLER

<table><tr><td rowspan="2">Method</td><td rowspan="2">Optimum Time(s)</td><td rowspan="2">Computation Time(s)</td><td colspan="6">\( \max {\ddot{q}}_{i}/\left| {\ddot{q}}_{i}\right| \)</td></tr><tr><td>J1</td><td>J2</td><td>J3</td><td>J4</td><td>J5</td><td>J6</td></tr><tr><td>TOPP-CO</td><td>1.4733</td><td>0.4023</td><td>9.41</td><td>6.11</td><td>6.70</td><td>7.34</td><td>0.07</td><td>9.41</td></tr><tr><td>TOPP-SCP</td><td>1.4748</td><td>5.3667</td><td>1.0</td><td>0.97</td><td>0.94</td><td>1.0</td><td>0.00</td><td>1.0</td></tr><tr><td>TOPP-LAC</td><td>1.4803</td><td>3.7434</td><td>0.6</td><td>0.48</td><td>0.67</td><td>1.00</td><td>0.00</td><td>0.6</td></tr><tr><td>TOPP-BS</td><td>1.578</td><td>0.8462</td><td>2.25</td><td>1.54</td><td>1.71</td><td>1.47</td><td>0.02</td><td>2.25</td></tr><tr><td>JTOPP-CO</td><td>1.6588</td><td>1.2757</td><td>3.59</td><td>2.00</td><td>1.94</td><td>3.79</td><td>0.02</td><td>3.59</td></tr><tr><td>TRTOPP-CO</td><td>1.529</td><td>1.8475</td><td>1.92</td><td>1.03</td><td>1.25</td><td>2.19</td><td>0.02</td><td>1.92</td></tr><tr><td>S-curve</td><td>2.12</td><td>-</td><td>0.85</td><td>0.99</td><td>1.89</td><td>2.86</td><td>0.23</td><td>0.93</td></tr><tr><td>JTOPP-BS</td><td>1.602</td><td>1.8368</td><td>0.66</td><td>0.54</td><td>0.62</td><td>0.67</td><td>0.01</td><td>0.66</td></tr></table>

<!-- figureText: 8 J2 J4 J6 JTOPP-CO TRTOPP-CC S-curve JTOPP-BS 7 6 \( {\dddot{q}}_{i}/\overline{{\dddot{q}}_{i}} \) 3 2 1 0 TOPP-CO TOPP-SCP TOPP-LAC TOPP-BS -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_13.jpg?x=926&y=1350&w=716&h=404&r=0"/>

Fig. 13. Bar chart of the results of maximum uniformed jerk of robot joints along door-type path with different methods.

<!-- Media -->

Fig. 13 illustrates the results from Table IV in a bar chart, highlighting the superior constraint capability of JTOPP-BS for joint jerk when compared to other jerk constrained methods. However, Furthermore, Fig. 12 depicts the pseudo velocities obtained using different optimization methods. It is evident that JTOPP-BS and TOPP-BS yield larger enclosed areas compared to TRTOPP-CO. However, the optimal time obtained for JTOPP-BS is the opposite of the enclosed areas. This further supports the notion that sacrificing more time for smoother starting and ending points is preferable. Fig. 14 depicts the numerical results of robot joint velocity, acceleration, and jerk obtained using different optimization methods, represented in a normalized manner. It demonstrates the effectiveness of the proposed JTOPP-BS method in achieving time optimality and ensuring trajectory smoothness for industrial manipulators, particularly in pick and place tasks. This holds great potential for enhancing productivity and efficiency in manufacturing processes involving industrial manipulators.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: JI et al.: CONVEX OPTIMIZATION METHOD TO TIME-OPTIMAL TRAJECTORY PLANNING WITH JERK CONSTRAINT 7643-->

<!-- Media -->

<!-- figureText: 1 1 \( {\ddot{q}}_{1}/\left| {\ddot{q}}_{1}\right| \) 1 \( {\ddot{q}}_{2}/\left| {\ddot{q}}_{2}\right| \) -1 \( {\ddot{q}}_{3}/\left| {\ddot{q}}_{3}\right| \) \( {\ddot{q}}_{4}/\left| {\ddot{q}}_{4}\right| \) 1 \( {\ddot{q}}_{5}/\left| {\ddot{q}}_{5}\right| \) -1 \( {\ddot{q}}_{6}/\left| {\ddot{q}}_{6}\right| \) 0.6 0.8 0.2 0.4 0.6 0.8 (b) (c) \( {\dot{q}}_{1}/\left| {\overline{\dot{q}}}_{1}\right| \) \( {\ddot{q}}_{1}/\left| {\ddot{q}}_{1}\right| \) -1 -1 1 \( {\dot{q}}_{2}/\left| {\dot{q}}_{2}\right| \) 0 \( {\ddot{q}}_{2}/\left| {\overline{\ddot{q}}}_{2}\right| \) -1 -1 1 \( {\dot{q}}_{3}/\left| {\overline{\dot{q}}}_{3}\right| \) 0 \( {\ddot{q}}_{3}/\left| {\overline{\ddot{q}}}_{3}\right| \) -1 \( {\dot{q}}_{4}/\left| {\dot{q}}_{4}\right| \) 0 \( {\ddot{q}}_{4}/\left| {\ddot{q}}_{4}\right| \) 1 \( {\overset{ \cdot  }{q}}_{5}/\left| {\overset{ \cdot  }{q}}_{5}\right| \) 0 \( {\ddot{q}}_{5}/\left| {\ddot{q}}_{5}\right| \) -1 -1 1 \( {\dot{q}}_{6}/\left| {\dot{q}}_{6}\right| \) 0 \( {\ddot{q}}_{6}/\left| {\ddot{q}}_{6}\right| \) -1 0.2 0.4 0.6 0.8 1 0 0.2 0.4 (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_14.jpg?x=170&y=155&w=1451&h=1284&r=0"/>

Fig. 14. The numerical results of velocity, acceleration and jerk of robot joints compared with different methods. (a) joint velocity. (b) joint acceleration. (c) joint jerk.

<!-- figureText: Spatial trajectory Trajectory in the plane parallel to the XZ-plane Trajectory in the plane parallel to the YZ-plane 0.3 z-coordinate (m) 0.28 0.26 0.24 0.22 0 0.1 0.2 0.3 0.458 0.459 0.4595 0.46 0.4605 0.46 x-coordinate (m) y-coordinate (m) (b) (c) 0.35 z-coordinate (m) 0.3 z-coordinate (m) 0.3 0.2 0.25 0.2 0.47 -0.2 -0.3 -0.2 -0.1 y-coordinate (m) 0.455 x-coordinate (m) (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_14.jpg?x=189&y=1535&w=1412&h=404&r=0"/>

Fig. 15. The experiment results of contour tracking with different methods. The curve marked with red line is referenced path. The S-curve from UR5 controller is marked with orange solid line. (a)3D-view. (b)XZ-view. (c) YZ-view.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 7644 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING, VOL. 21, NO. 4, OCTOBER 2024-->

<!-- figureText: 1 1 \( {\ddot{q}}_{1}/\left| {\ddot{q}}_{1}\right| \) -1 1 \( {\ddot{q}}_{2}/\left| {\ddot{q}}_{2}\right| \) -1 1 \( {\ddot{q}}_{3}/\left| {\ddot{q}}_{3}\right| \) -1 \( {\ddot{q}}_{4}/\left| {\ddot{q}}_{4}\right| \) 1 \( {\ddot{q}}_{5}/\left| {\ddot{q}}_{5}\right| \) -1 \( {\ddot{q}}_{6}/\left| {\ddot{q}}_{6}\right| \) 0.6 0.8 0.2 0.4 0.6 0.8 (b) (c) \( {\dot{q}}_{1}/\left| {\overline{\dot{q}}}_{1}\right| \) \( {\ddot{q}}_{1}/\left| {\ddot{q}}_{1}\right| \) -1 -1 1 \( {\dot{q}}_{2}/\left| {\dot{q}}_{2}\right| \) 0 \( {\ddot{q}}_{2}/\left| {\ddot{q}}_{2}\right| \) -1 -1 1 \( {\dot{q}}_{3}/\left| {\overline{\dot{q}}}_{3}\right| \) 0 \( {\ddot{q}}_{3}/\left| {\overline{\ddot{q}}}_{3}\right| \) -1 \( {\dot{q}}_{4}/\left| {\dot{q}}_{4}\right| \) 0 \( {\ddot{q}}_{4}/\left| {\overline{\ddot{q}}}_{4}\right| \) 1 \( {\dot{q}}_{5}/\left| {\dot{q}}_{5}\right| \) 0 \( {\ddot{q}}_{5}/\left| {\ddot{q}}_{5}\right| \) -1 -1 1 \( {\dot{q}}_{6}/\left| {\dot{q}}_{6}\right| \) 0 \( {\ddot{q}}_{6}/\left| {\ddot{q}}_{6}\right| \) -1 0 0.2 0.4 0.6 0.8 1 0 0.2 0.4 (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_15.jpg?x=170&y=155&w=1448&h=1285&r=0"/>

Fig. 16. The experiment results of actual velocity, acceleration and jerk of robot joints compared with different methods. (a) actual joint velocity. (b) actual joint acceleration. (c) actual joint jerk.

<!-- Media -->

Fig. 15 illustrates the trajectory of the actual TCP positions collected during the experiments. The red solid line represents the reference path, while the orange solid line represents the tracking trajectory of the UR5's built-in control algorithm. From Fig.15a and 15c, it can be observed that the UR5's internal controller demonstrates good trajectory tracking performance, with a maximum tracking error of approximately \( {0.5}\mathrm{\;{mm}} \) . In contrast,the trajectory maximum tracking errors of the other optimization algorithms are larger, around \( {2.5}\mathrm{\;{mm}} \) ,and particularly more pronounced in the latter half of the door-type path. However, the maximum tracking error in 3D space is approximately \( {3.5}\mathrm{\;{mm}} \) . To investigate this further, additional experiments were conducted, where the same trajectory collected from the UR5 controller was used as the servoj command position input to the controller. The resulting tracking error was found to be similar to that of the optimization-based methods. This suggests that the discrepancy may be attributed to differences in the external control command cycle and the internal interpolation algorithm cycle, with the internal interpolation algorithm having a faster servo cycle. Additionally, the performance of the forward control algorithm and proportional position controller is highly influenced by the version, which limits better tracking performance.

Fig. 16 depicts the collected actual joint velocity, acceleration, and jerk trajectories during the experiments. In addition to the overshoot phenomenon mentioned earlier, it can be observed that without sufficient constraints, the residual vibration of the robot's motion is more pronounced in the jerk trajectory. However, the proposed JTOPP-BS method effectively minimizes the impact of residual vibrations on trajectory accuracy. The actual joint jerk values represented by the black solid line in Fig.16c also remain within a relatively small range, further validating the effectiveness of the proposed method.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: JI et al.: CONVEX OPTIMIZATION METHOD TO TIME-OPTIMAL TRAJECTORY PLANNING WITH JERK CONSTRAINT 7645-->

Considering the results from the three case studies, although the numerical calculations have confirmed the superiority of the proposed method in terms of jerk constraints and computational efficiency, the full potential of the algorithm is not fully realized due to limitations imposed by the robot's internal algorithms and physical constraints. Therefore, experimental results indicate that the tracking accuracy of the optimized trajectories using the RTDE interface is inferior compared to that achieved with the internal tracking controller of the UR robot. This is mainly reflected in two aspects: Firstly, the UR5 robot exhibits strong flexibility, and due to the limitations of joint tracking performance, the maximum torque of the joint drive cannot be fully utilized. As a result, the acceleration constraints in the optimization method play a dominant role instead of torque constraints. Secondly, the UR5 adopts the servoj command to execute discrete command trajectories, which is subject to control cycle and forward control algorithm limitations, resulting in poorer trajectory tracking performance compared to the internal control algorithm. However, in the UR5e version,the control cycle will be increased to \( 2\mathrm{\;{ms}} \) ,and most industrial robots can achieve control cycles as short as \( 1\mathrm{\;{ms}} \) . Furthermore,the smoothness of the trajectory at the start and end points significantly affects the time optimality of the trajectory. Therefore, robots capable of withstanding larger jerk values will also exhibit better time optimality. Hence, future research will involve validating the proposed algorithm on a wider range of robots.

## V. CONCLUSION

This study introduces a convex optimization approach to address the problem of time-optimal and jerk-constrained trajectory planning. The method employs cubic B-splines to reconstruct pseudo states, namely pseudo velocity and pseudo acceleration, using control points. These pseudo states are discretized and formulated as linear constraints in the optimization problem. To handle the non-convex jerk constraints, a convex restriction technique is applied by substituting the time-optimum pseudo velocity into the jerk constraints. The proposed method is evaluated experimentally through three case studies involving two contour following tasks and a pick-and-place task. The optimization results demonstrate the effectiveness of the jerk-constrained method. However, due to the convex restriction of jerk constraints, the proposed method, JTOPP-BS, sacrifices some time optimality to achieve the smoothness of trajectories and computational efficiency for industrial manipulators. Given the robustness and smoothness of the proposed method, there is significant potential for enhancing productivity and increasing manufacturing output through the use of industrial manipulators. In the future, more efforts will be directed towards improving time optimality while maintaining smoothness and computational efficiency. REFERENCES

[1] J. E. Bobrow, S. Dubowsky, and J. S. Gibson, "Time-optimal control of robotic manipulators along specified paths," Int. J. Robot. Res., vol. 4, no. 3, pp. 3-17, Sep. 1985.

[2] D. Constantinescu and E. A. Croft, "Smooth and time-optimal trajectory planning for industrial manipulators along specified paths," J. Robot. Syst., vol. 17, no. 5, pp. 233-249, May 2000.

[3] J.-J.-E. Slotine and H. S. Yang, "Improving the efficiency of time-optimal path-following algorithms," IEEE Trans. Robot. Autom., vol. 5, no. 1, pp. 118-124, Mar. 1989.

[4] Y. Wen and P. Pagilla, "Path-constrained and collision-free optimal trajectory planning for robot manipulators," IEEE Trans. Autom. Sci. Eng., vol. 20, no. 2, pp. 763-774, Apr. 2023.

[5] H. Gattringer, A. Mueller, M. Oberherber, and D. Kaserer, "Time-optimal robotic manipulation on a predefined path of loosely placed objects: Modeling and experiment," Mechatronics, vol. 84, Jun. 2022, Art. no. 102753.

[6] D. Verscheure, B. Demeulenaere, J. Swevers, J. De Schutter, and M. Diehl, "Time-optimal path tracking for robots: A convex optimization approach," IEEE Trans. Autom. Control, vol. 54, no. 10, pp. 2318-2327, Oct. 2009.

[7] H. Pham and Q.-C. Pham, "A new approach to time-optimal path parameterization based on reachability analysis," IEEE Trans. Robot., vol. 34, no. 3, pp. 645-659, Jun. 2018.

[8] A. Gasparetto and V. Zanotto, "A new method for smooth trajectory planning of robot manipulators," Mechanism Mach. Theory, vol. 42, no. 4, pp. 455-471, Apr. 2007.

[9] H. Wang, H. Wang, J. Huang, B. Zhao, and L. Quan, "Smooth point-to-point trajectory planning for industrial robots with kinematical constraints based on high-order polynomial curve," Mechanism Mach. Theory, vol. 139, pp. 284-293, Sep. 2019.

[10] J. Kim and E. A. Croft, "Online near time-optimal trajectory planning for industrial robots," Robot. Comput.-Integr. Manuf., vol. 58, pp. 158-171, Aug. 2019.

[11] Y. Fang, J. Qi, J. Hu, W. Wang, and Y. Peng, "An approach for jerk-continuous trajectory generation of robotic manipulators with kinematical constraints," Mechanism Mach. Theory, vol. 153, Nov. 2020, Art. no. 103957.

[12] Y. Fang, J. Hu, W. Liu, Q. Shao, J. Qi, and Y. Peng, "Smooth and time-optimal S-curve trajectory planning for automated robots and machines," Mechanism Mach. Theory, vol. 137, pp. 127-153, Jul. 2019.

[13] G. Wu, W. Zhao, and X. Zhang, "Optimum time-energy-jerk trajectory planning for serial robotic manipulators by reparameterized quintic NURBS curves," Proc. Inst. Mech. Eng., C, J. Mech. Eng. Sci., vol. 235, no. 19, pp. 4382-4393, Oct. 2021.

[14] A. Alzaydi, "Time-optimal, minimum-jerk, and acceleration continuous looping and stitching trajectory generation for 5-axis on-the-fly laser drilling," Mech. Syst. Signal Process., vol. 121, pp. 532-550, Apr. 2019.

[15] A. Artuñedo, J. Villagra, and J. Godoy, "Jerk-limited time-optimal speed planning for arbitrary paths," IEEE Trans. Intell. Transp. Syst., vol. 23, no. 7, pp. 8194-8208, Jul. 2022.

[16] Z. Shiller, "Time-energy optimal control of articulated systems with geometric path constraints," J. Dyn. Syst., Meas., Control, vol. 118, no. 1, pp. 139-143, Mar. 1996.

[17] J.-W. Ma, S. Gao, H.-T. Yan, Q. Lv, and G.-Q. Hu, "A new approach to time-optimal trajectory planning with torque and jerk limits for robot," Robot. Auto. Syst., vol. 140, Jun. 2021, Art. no. 103744.

[18] F. Wang, Z. Wu, and T. Bao, "Time-jerk optimal trajectory planning of industrial robots based on a hybrid WOA-GA algorithm," Processes, vol. 10, no. 5, p. 1014, May 2022.

[19] F. Debrouwere et al., "Time-optimal path following for robots with convex-concave constraints using sequential convex programming," IEEE Trans. Robot., vol. 29, no. 6, pp. 1485-1495, Dec. 2013.

[20] D. Kaserer, H. Gattringer, and A. Müller, "Nearly optimal path following with jerk and torque rate limits using dynamic programming," IEEE Trans. Robot., vol. 35, no. 2, pp. 521-528, Apr. 2019.

[21] H. Pham and Q.-C. Pham, "On the structure of the time-optimal path parameterization problem with third-order constraints," in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), May 2017, pp. 679-686.

[22] P. Shen, X. Zhang, and Y. Fang, "Complete and time-optimal path-constrained trajectory planning with torque and velocity constraints: Theory and applications," IEEE/ASME Trans. Mechatronics, vol. 23, no. 2, pp. 735-746, Apr. 2018.

[23] J. Zhao, X. Zhu, and T. Song, "Serial manipulator time-jerk optimal trajectory planning based on hybrid IWOA-PSO algorithm," IEEE Access, vol. 10, pp. 6592-6604, 2022.

[24] A. Palleschi, M. Garabini, D. Caporale, and L. Pallottino, "Time-optimal path tracking for jerk controlled robots," IEEE Robot. Autom. Lett., vol. 4, no. 4, pp. 3932-3939, Oct. 2019.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 7646 IEEE TRANSACTIONS ON AUTOMATION SCIENCE AND ENGINEERING, VOL. 21, NO. 4, OCTOBER 2024-->

[25] Y. Ding, Y. Wang, and B. Chen, "Smooth and proximate time-optimal trajectory planning of robotic manipulators," Trans. Can. Soc. Mech. Eng., vol. 46, no. 2, pp. 466-476, Jun. 2022.

[26] J. Huang, P. Hu, K. Wu, and M. Zeng, "Optimal time-jerk trajectory planning for industrial robots," Mechanism Mach. Theory, vol. 121, pp. 530-544, Mar. 2018.

[27] M.-X. Kong, C. Ji, Z.-S. Chen, and R.-F. Li, "Smooth and near time-optimal trajectory planning of robotic manipulator with smooth constraint based on cubic B-spline," in Proc. IEEE Int. Conf. Robot. Biomimetics (ROBIO), Dec. 2013, pp. 2328-2333.

[28] D. Lee, K. Turitsyn, D. K. Molzahn, and L. A. Roald, "Robust AC optimal power flow with robust convex restriction," IEEE Trans. Power Syst., vol. 36, no. 6, pp. 4953-4966, Nov. 2021.

[29] Gurobi Optimizer Reference Manual, Gurobi Optimization, LLC, Beaverton, OR, USA, 2022.

[30] M. Aps, "Mosek optimization toolbox for MATLAB," User's Guide Reference Manual, vol. 4, no. 1, Sep. 2019.

[31] L. Lu et al., "Joint-smooth toolpath planning by optimized differential vector for robot surface machining considering the tool orientation constraints," IEEE/ASME Trans. Mechatronics, vol. 27, no. 4, pp. 2301-2311, Aug. 2022.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_17.jpg?x=140&y=846&w=217&h=279&r=0"/>

<!-- Media -->

Chen Ji received the M.S. degree from the Harbin Institute of Technology, Harbin, China, in 2013, and the Ph.D. degree from the State Key Laboratory of Robotics and System, Harbin Institute of Technology, in 2019. He is currently a Lecturer with the School of Mechanical Engineering, Jiangsu University, China. His research interests include trajectory optimization, trajectory planning method, the motion controller of industrial robot, and variable stiffness actuation.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_17.jpg?x=139&y=1244&w=219&h=276&r=0"/>

<!-- Media -->

Zhongqiang Zhang is currently a Professor with the School of Mechanical Engineering, Jiangsu University. His research interests include soft robot and microfluidics and fluidic devices.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_17.jpg?x=911&y=172&w=219&h=278&r=0"/>

<!-- Media -->

Guanggui Cheng is currently a Professor with the School of Mechanical Engineering, Jiangsu University, China. His research interests include soft robot and friction energy harvesting and applications.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_17.jpg?x=912&y=709&w=218&h=276&r=0"/>

<!-- Media -->

Minxiu Kong is currently an Assistant Professor with the State Key Laboratory of Robotics and System, School of Mechatronics Engineering, Harbin Institute of Technology, China. His research interests include industrial robot and non-linear control theory.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2sl9cref24c73b2kel0_17.jpg?x=911&y=1243&w=219&h=277&r=0"/>

<!-- Media -->

Ruifeng Li is currently a Professor and a Ph.D. Candidate Supervisor with the State Key Laboratory of Robotics and System, School of Mechatronics Engineering, Harbin Institute of Technology, China. His research interests include industrial robot and service robot.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:14:36 UTC from IEEE Xplore. Restrictions apply.-->

