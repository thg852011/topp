

<!-- Meanless: Robotics and Computer-Integrated Manufacturing 83 (2023) 102576 Contents lists available at ScienceDirect Robotics and Computer-Integrated Manufacturing ELSEVIER journal homepage: www.elsevier.com/locate/rcim updates-->

Full length Article

# A novel cartesian trajectory planning method by using triple NURBS curves for industrial robots

Xiangfei Li, Huan Zhao \( {}^{ * } \) , Xianming He, Han Ding

State Key Laboratory of Digital Manufacturing Equipment and Technology, Huazhong University of Science and Technology, Wuhan 430074, P. R. China

## ARTICLEINFO

Keywords:

Industrial robots

Trajectory planning

Triple NURBS curves

Limited linear jerk

Bidirectional interpolation

## A B S T R A C T

This article presents a novel method of robot pose trajectory synchronization planning. First of all, based on triple NURBS curves, a method of describing the position and orientation synchronization of the robot is proposed. Then, through considering geometric and kinematic constraints, especially angular velocity constraint, and employing bidirectional interpolation algorithm, a robot pose trajectory planning approach is developed, which has limited linear jerk, continuous bounded angular velocity and approximate optimal time, and does not need an optimization program. Ultimately, two robot pose paths, blade-shaped curve and fan-shaped curve, are utilized for simulations, and the results indicate that the proposed trajectory planning method can satisfy the given constraint conditions, i.e. the linear jerk is limited and the angular velocity is continuous bounded. The trajectory tracking experiments are further carried out on a 6-DOF industrial robot, and the results show that the proposed planning method can generate smooth trajectories to ensure the stability of the robot motion without impact in practical situations.

## 1. Introduction

Robots have the advantages of good flexibility, low cost and high degree of intelligence, so they are gradually applied in machining, assembly, additive manufacturing and other fields [1-3]. In these fields, driving the robots involves a basic problem, namely trajectory planning. Trajectory planning is usually defined as generating the desired inputs for the robot control system, and its goal is to take the geometric path, the robot kinematics and dynamics and other constraints as the inputs, and output the continuous and smooth trajectories of the end-effector or the joints, which are represented by a series of position, velocity and acceleration values \( \left\lbrack  {4,5}\right\rbrack \) . Obviously,trajectory planning plays a very important role in ensuring the stability of the robot motion, which determines the performance of the robot motion fundamentally.

Trajectory planning is generally performed in the joint space or the Cartesian space [6] or a mixed space of both \( \left\lbrack  {7,8}\right\rbrack \) . For the trajectory planning in the joint space, the trajectory is described as a function of joint angles, so the planning is simple and has no singularity. In addition, because the robot motion control system acts directly on the joints, the smoothness of the joint trajectory is more important than that of the Cartesian trajectory. So far, various approaches have been proposed to obtain the desired joint trajectory. As a class of piecewise functions defined by multi-order polynomials, splines have the advantages of simple structure, strong ability to approximate complex shapes and high calculation accuracy, so they have received a lot of attention. Because cubic spline provides the continuity of the velocity and acceleration of knots, it is widely employed in the joint space trajectory planning [9-14]. However, cubic spline has an obvious drawback that the velocity and acceleration fluctuations may occur at the first and last knots. To address this problem, higher order is usually adopted. For example, fifth-order or quintic B-spline [15-18], even seven-order polynomial or septuple B-spline [19-21]. Since the trajectory planning in the joint space often requires to satisfy the joint kinematics and dynamics constraints, it naturally becomes a nonlinearly constrained optimization problem. The existing optimization models include minimum-jerk models, time-optimal models, minimum-energy models, multi-criteria models, etc., and the optimization algorithms include genetic algorithm, greedy algorithm, dynamic programming, particle swarm optimization and sequential quadratic programming and so on [4,6,22]. On the whole, however, since the joint space coordinates are neither orthogonal nor can separate the position and orientation, the convenience of use is slightly insufficient. Besides, the nonlinearity is introduced by the kinematics, so the movement of the end-effector is not intuitive and easy to predict, which makes the joint space trajectory planning difficult to be directly applied in occasions with strict path tracking error requirements such as grinding and milling.

---

<!-- Footnote -->

* Corresponding author.

E-mail address: huanzhao@hust.edu.cn (H. Zhao).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.rcim.2023.102576 Received 18 October 2022; Received in revised form 6 April 2023; Accepted 12 April 2023 Available online 25 April 2023 0736-5845/(© 2023 Elsevier Ltd. All rights reserved.-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

Since the tasks and the obstacles are described naturally in the Cartesian space or the operational space, the geometric paths are usually specified in this space, which makes the trajectory planning in the Cartesian space more intuitive than that in the joint space [23]. In addition, required motion law can be imposed to ensure the path accuracy of practical applications [24]. By considering the dynamical behavior of the robot, Valero et al. [25] utilized the sequential quadratic programming technique to optimize the trajectory in the workspace. Da Graça Marcos et al. [26] gave a trajectory planning method that combines closed-loop pseudoinverse with genetic algorithms in the operational space. Via analyzing the unconstrained variational calculus of optimal control problem and considering the holonomic constraints, Gregory et al. [27] planned the trajectory of the manipulator. By taking the coupling effect index as the objective function and solving it with the genetic algorithm, Zheng et al. [28] introduced a new trajectory planning approach to reduce the vibration errors. However, because the above trajectory planning approaches are based on the optimization algorithms, their calculations are usually time-consuming and online use may not be easy. In view of this, the trajectory planning without optimization solution process has attracted attention. Macfarlane and Croft [29] developed an online approach to acquire jerk-bounded trajectories through a concatenation of fifth-order polynomials. Rossi and Savino [30] presented a method called the envelope of tangents planning for the robot trajectory planning. Trigatti et al. [31] proposed a path-constrained trajectory planning method in the operational space for the spray painting robots, which does not resort to optimization routines. Through considering both the workspace and joint space constraints in the machining process, Lu et al. [32] put forward a time-optimal motion planning algorithm, and solved it by numerical integration. Recently, by combining the dynamics model and input shaping technique, Zhang et al. [33] designed a time-optimal smooth trajectory planning approach for the manipulators. Based on the kinematic and dynamic constraints in a third-order form, Wang et al. [34] proposed an approximate time-optimal feedrate planning method for the robot end-effector. However, whether there is optimization or not, the above work only involves the position trajectory planning, while the orientation trajectory planning, which is equally important for the robot pose adjustment, is not considered. Li et al. [35] presented a virtual repulsive potential field method for the trajectory planning in the robotic milling. However, it focuses mainly on the orientation trajectory planning.

To synchronously plan the position and orientation (i.e. pose) trajectories of the robot, using the pose ruled surface optimization enhanced by a genetic algorithm, Zha [36] proposed a unified method to plan the optimal pose trajectory for the manipulators in the Cartesian space. However, since the orientation is represented by a vector composed of two points, the synchronous description of the robot pose is essentially similar to the dual NURBS curves in the pose description of the five-axis machine tools, which leads to only the five degrees of freedom (DOFs) of the robot being constrained. Besides, owing to the introduction of the optimization algorithm, the amount of calculation is relatively large. Similar problems can also be found in [37]. Bianco and Ghilardelli [38] developed a real-time trajectory planner based on the path-velocity decomposition in the operational space, where the orientation is represented by the Euler angles, and a given set of kinematic constraints is considered. Based on the improved immune clonal selection strategy, Chen et al. [39] gave a multi-objective trajectory planning approach, in which the Euler angles are adopted to specify the orientation, and all the trajectories are parameterized by the quintic B-spline, and the inverse kinematics and dynamics constraints are considered. Recently, Jin et al. [40] formulated the Cartesian trajectory planning of space robots as a multi-objective optimization, and solved it utilizing the chaotic particle swarm algorithm, where the unit quaternion and Euler angles are used to express the orientation of the base and the end-effector, respectively, and the position and orientation are parameterized by the Bézier curves. Although the synchronous planning of the position and orientation is considered, the purpose of this work is mainly to avoid dynamic singularities by least squares and feedback compensation. Amersdorfer and Meurer [41] presented a Cartesian trajectory planning technique, where the rotation matrix is utilized for the orientation trajectory planning, and the synchronization of the position and orientation is determined by the surface geometry. Based on the least squares curve fitting and a thin plate energy model, Wang et al. [42] designed a smooth tool trajectory generation method, in which the position and orientation synchronization of the robot is achieved by the homogeneous transformation matrix, and the constraints of the robot stiffness, dexterity and joint angles are considered. In general, however, the above approaches seldom consider the angular velocity limit, which may lead to the angular velocity exceeding the constraint in the process of robot pose trajectory planning.

Based on the analysis above, by using triple NURBS curves and considering geometric and kinematic constraints, especially angular velocity constraint, a novel method of robot pose trajectory synchronization planning is proposed in this article. First of all, through adopting three space point coordinates to describe the position and orientation, a synchronous description method of position and orientation paths based on the triple NURBS curves is constructed. Then, considering the geometric and kinematic constraints such as chord error, velocity and acceleration limits, etc., and utilizing bidirectional interpolation technique, a robot pose trajectory planning method with limited linear jerk, continuous bounded angular velocity and approximate optimal time without optimization process is formulated in detail. Although there have been a few reports on the application of triple NURBS curves to the error compensation of multi-axis [43] or five-axis machining [44], to the authors' knowledge, this may be the first article to use the triple NURBS curves to achieve the synchronous planning of robot position and orientation trajectories. To sum up, the main contributions of this article are as follows:

1) A robot position and orientation synchronization description method based on triple NURBS curves is constructed for the first time.

2) Based on the triple NURBS curves, through considering the geometric and kinematic constraints, especially the angular velocity constraint, a robot pose trajectory planning method with limited linear jerk, continuous bounded angular velocity and approximate optimal time is given without resorting to the optimization program.

3) Two robot pose paths are employed to verify the effectiveness of the proposed method.

The rest of this article is structured as follows. In Section 2, the synchronous description of robot position and orientation paths based on the triple NURBS curves is introduced. In Section 3, the proposed robot pose trajectory planning method is elaborated. In Section 4, the simulation and experimental results are presented. In Section 5, the conclusions of this article are summarized and the future work is given.

## 2. Synchronous description of position and orientation paths

In the process of the robot pose trajectory planning, because the position and orientation are not in the same space (i.e. the position is in Euclidean space and the orientation is in Lie group space), it is still difficult to achieve robot pose trajectory synchronization planning. Inspired by the use of dual NURBS curves in five-axis machine tools to synchronously plan the pose trajectory of the machine tools, for 6-DOF industrial robots, novel triple NURBS curves are proposed to synchronously describe the pose path and plan the pose trajectory. Its core is to determine the local coordinate system that represents the orientation through the position NURBS curve and the other two NURBS curves, so as to realize the synchronous description of the robot pose path and the synchronous planning of pose trajectory. To this end, this section first presents a method to synchronously describe the position and orientation paths using triple NURBS curves. The triple NURBS curves have the same degree and knot vector, but the control points are different. The first curve \( {C}_{1} \) represents the position of the robot end-effector,and this curve is combined with the other two curves \( {\mathbf{C}}_{2} \) and \( {\mathbf{C}}_{3} \) to determine the orientation of the robot end-effector, as shown in Fig. 1.

<!-- Meanless: 2-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

<!-- Media -->

<!-- figureText: \( {z}_{2} \) , \( {O}_{n} \) \( {\mathbf{C}}_{2}\left( u\right) \) \( {\mathbf{C}}_{3}\left( u\right) \) \( {O}_{2} \) \( {y}_{2} \) \( {O}_{1} \) \( {y}_{1} \) \( {\mathbf{C}}_{1}\left( u\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_2.jpg?x=116&y=148&w=722&h=352&r=0"/>

Fig. 1. Position and orientation synchronization based on triple NURBS curves.

<!-- figureText: Robot end-effector \( {p}_{1} \) \( {\mathbf{p}}_{3} \) on the plane -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_2.jpg?x=259&y=596&w=433&h=399&r=0"/>

Fig. 2. Selection of points in local coordinate system fixed with the robot end-effector.

<!-- Media -->

### 2.1. Position path description

Given a series of robot positions as \( \left\{  {\mathbf{Q}}_{i}\right\}  ,i = 0,1,\cdots ,n \) ,the purpose of path description is to utilize a parametric curve to interpolate these positions to generate a continuous path. As one of the most popular industry standards, due to its exact analytical representation and local adjustment ability, NURBS curve has been widely used in path description, which can usually be expressed as [45]

\[\mathbf{C}\left( u\right)  = \frac{\mathop{\sum }\limits_{{i = 0}}^{n}{N}_{i,p}\left( u\right) {\omega }_{i,1}{\mathbf{P}}_{i,1}}{\mathop{\sum }\limits_{{i = 0}}^{n}{N}_{i,p}\left( u\right) {\omega }_{i,1}},u \in  \left\lbrack  {0,1}\right\rbrack   \tag{1}\]

where \( u \) and \( p \) are the parameter and degree of the curve,respectively, and \( \left\{  {\mathbf{P}}_{i,1}\right\} \) are the control points,and \( n \) is the number of the control points,and \( \left\{  {\omega }_{i,1}\right\} \) are the weight factors,and \( \left\{  {{N}_{i,p}\left( u\right) }\right\} \) are the basis functions. Assuming that \( \mathbf{U} = \left\lbrack  {{u}_{0},{u}_{1},\cdots ,{u}_{k},\cdots ,{u}_{n + p + 1}}\right\rbrack \) is the aperiodic and non-uniform knot vector, the basis functions can be defined as

\[\left\{  \begin{array}{l} {N}_{i,0}\left( u\right)  = \left\{  \begin{array}{l} 1\text{ if }{u}_{i} \leq  u \leq  {u}_{i + 1} \\  0\text{ otherwise } \end{array}\right. \\  {N}_{i,p}\left( u\right)  = \frac{u - {u}_{i}}{{u}_{i + p} - {u}_{i}}{N}_{i,p - 1}\left( u\right)  + \frac{{u}_{i + p + 1} - u}{{u}_{i + p + 1} - {u}_{i + 1}}{N}_{i + 1,p - 1}\left( u\right)  \end{array}\right.  \tag{2}\]

Obtaining the knot vector and the control points is the crucial task of NURBS curve describing the path, and how to determine the parameter values at the positions \( \left\{  {Q}_{i}\right\} \) will influence the shape and parameterization of the curve. In this subsection, the parameter values are obtained by the centripetal method as

\[{\bar{u}}_{0} = 0,{\bar{u}}_{n} = 1,{\bar{u}}_{i} = {\bar{u}}_{i - 1} + \frac{\sqrt{\left| {\mathbf{Q}}_{i} - {\mathbf{Q}}_{i - 1}\right| }}{\mathop{\sum }\limits_{{i = 1}}^{n}\sqrt{\left| {\mathbf{Q}}_{i} - {\mathbf{Q}}_{i - 1}\right| }},i = 1,2,\cdots ,n - 1 \tag{3}\]

The knot vector \( \mathbf{U} \) can be calculated by the averaging technique as

\[\left\{  \begin{array}{l} {u}_{0} = {u}_{1} = \cdots  = {u}_{p} = 0 \\  {u}_{j + p} = \frac{1}{p}\mathop{\sum }\limits_{{i = j}}^{{j + p - 1}}{\bar{u}}_{i},j = 1,2,\cdots ,n - p \\  {u}_{n + 1} = {u}_{n + 2} = \cdots  = {u}_{n + p + 1} = 1 \end{array}\right.  \tag{4}\]

The first and last control points are defined as \( {\mathbf{P}}_{0,1} = {\mathbf{Q}}_{0},{\mathbf{P}}_{n,1} = {\mathbf{Q}}_{n} \) , respectively,and other control points \( \left\{  {\mathbf{P}}_{i,1}\right\} \) can be calculated as

\[\left\lbrack  \begin{matrix} {N}_{1,p}\left( {\bar{u}}_{1}\right) & \cdots & {N}_{n - 1,p}\left( {\bar{u}}_{1}\right) \\  \vdots &  \ddots  & \vdots \\  {N}_{1,p}\left( {\bar{u}}_{n - 1}\right) & \cdots & {N}_{n - 1,p}\left( {\bar{u}}_{n - 1}\right)  \end{matrix}\right\rbrack  \left\lbrack  \begin{matrix} {\mathbf{P}}_{1,1} \\  \vdots \\  {\mathbf{P}}_{n - 1,1} \end{matrix}\right\rbrack   = \left\lbrack  \begin{matrix} {\mathbf{Q}}_{1} \\  \vdots \\  {\mathbf{Q}}_{n - 1} \end{matrix}\right\rbrack   \tag{5}\]

It can be seen from Eq. (5) that the elements in the coefficient matrix are the values of the basis functions, and they are related only to the knot values. By solving Eq. (5),the control points \( \left\{  {{\mathbf{P}}_{1,1},{\mathbf{P}}_{2,1},\cdots ,{\mathbf{P}}_{n - 1,1}}\right\} \) can be obtained. Since the limited linear jerk is considered in the subsequent position trajectory planning, the cubic NURBS curve is adopted here to describe the position path as

\[{\mathbf{C}}_{1}\left( u\right)  = \frac{\mathop{\sum }\limits_{{i = 0}}^{n}{N}_{i,3}\left( u\right) {\omega }_{i,1}{\mathbf{P}}_{i,1}}{\mathop{\sum }\limits_{{i = 0}}^{n}{N}_{i,3}\left( u\right) {\omega }_{i,1}},u \in  \left\lbrack  {0,1}\right\rbrack   \tag{6}\]

### 2.2. Synchronization of orientation and position paths

A well-defined rigid body configuration in three-dimensional (3D) space requires six DOFs: three for the position and three for the orientation. According to the Chasles' theorem [46], the position and orientation of rigid body can be handled independently. For the position, Cartesian coordinates in Euclidean space are usually adopted. However, since the orientation is a manifold based on Lie groups, there is no general method to describe it [47]. Rotation matrix, quaternion and Euler angles are often used to describe the orientation of the robot end-effector. Due to the convenience of matrix algebra, the rotation matrix is widely applied in the orientation description. However, although the position and orientation can be decoupled in the Cartesian space, the continuous path operation rarely allows arbitrary orientation of the robot, and most applications require the coupling at Cartesian-orientation level. Therefore, the following will introduce an orientation mapping method between the rotation matrix and three points on triple NURBS curves, so as to realize the synchronous description of the orientation and position of the robot end-effector, as shown in Fig. 1.

Supposing that the position of the robot end-effector at a certain moment is the first point \( {\mathbf{p}}_{1} \) ,and taking a point on the end-effector axis as the second point \( {\mathbf{p}}_{2} \) ,the third point \( {\mathbf{p}}_{3} \) is located on the plane perpendicular to the vector composed of the first and second points and passing through the first point. When a point not coincident with \( {\mathbf{p}}_{1} \) is selected on the plane as \( {\mathbf{p}}_{3} \) ,these three points can form a local coordinate system fixed with the robot end-effector, as shown by the three dotted lines in Fig. 2. When the robot end-effector moves in space, the above three points will leave three paths in space. Therefore, if the coordinates of the three points at each moment are determined, the position and orientation of the robot end-effector can be synchronously determined, and the specific details are described as follows.

First, the position path has been described, and the other two paths are interpolated using cubic NURBS curves in the same way. To keep these three paths synchronized, the three paths are given the same parameter \( u \) ,that is,the knot vector \( \mathbf{U} \) of the other two NURBS curves is consistent with the position curve. Thus, the other two curves are given as

<!-- Meanless: 3-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

\[{\mathbf{C}}_{2}\left( u\right)  = \frac{\mathop{\sum }\limits_{{i = 0}}^{n}{N}_{i,3}\left( u\right) {\omega }_{i,2}{\mathbf{P}}_{i,2}}{\mathop{\sum }\limits_{{i = 0}}^{n}{N}_{i,3}\left( u\right) {\omega }_{i,2}},u \in  \left\lbrack  {0,1}\right\rbrack  ,{\mathbf{C}}_{3}\left( u\right)  = \frac{\mathop{\sum }\limits_{{i = 0}}^{n}{N}_{i,3}\left( u\right) {\omega }_{i,3}{\mathbf{P}}_{i,3}}{\mathop{\sum }\limits_{{i = 0}}^{n}{N}_{i,3}\left( u\right) {\omega }_{i,3}},u \in  \left\lbrack  {0,1}\right\rbrack  \]

(7)

<!-- Media -->

<!-- figureText: Input information Velocity extremum curve generation Current curve parameter \( {u}_{k} \rightarrow  {u}_{k + 1} \) Calculate the curvature \( \kappa \left( {u}_{k}\right) \) Calculate the maximum allowable linear velocity \( {v}_{m}\left( {u}_{k}\right) \) Check the maximum velocity \( \mathrm{N} \) \( {u}_{k} = 1 \) ? Y Scan velocity extremum curve and segment path Store parameters and velocity values Trajectory planning completed Robot pose trajectory Control points, knot vector and weight factors of triple NURBS curves Commanded velocity Interpolation period Maximum chord error Maximum tangential acceleration Maximum tangential jerk Maximum angular velocity Joint velocity, acceleration and jerk limits Bidirectional interpolation of path segments Perform S-type acceleration and deceleration planning Interpolate the pose using second-order Taylor expansion -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_3.jpg?x=242&y=150&w=1264&h=1021&r=0"/>

Fig. 3. Proposed robot pose trajectory planning method.

<!-- Media -->

where \( \left\{  {\mathbf{P}}_{i,2}\right\}  ,\left\{  {\mathbf{P}}_{i,3}\right\}  ,\left\{  {\omega }_{i,2}\right\} \) and \( \left\{  {\omega }_{i,3}\right\} \) are the control points and weight factors of the other two curves, respectively. After obtaining these three curves,the two unit vectors at \( {u}_{k} \) can be calculated as

\[{\mathbf{H}}_{1}\left( {u}_{k}\right)  = \frac{{\mathbf{C}}_{2}\left( {u}_{k}\right)  - {\mathbf{C}}_{1}\left( {u}_{k}\right) }{\begin{Vmatrix}{\mathbf{C}}_{2}\left( {u}_{k}\right)  - {\mathbf{C}}_{1}\left( {u}_{k}\right) \end{Vmatrix}},{\mathbf{H}}_{2}\left( {u}_{k}\right)  = \frac{{\mathbf{C}}_{3}\left( {u}_{k}\right)  - {\mathbf{C}}_{1}\left( {u}_{k}\right) }{\begin{Vmatrix}{\mathbf{C}}_{3}\left( {u}_{k}\right)  - {\mathbf{C}}_{1}\left( {u}_{k}\right) \end{Vmatrix}} \tag{8}\]

where \( \left| \right|  \cdot  \left| \right| \) denotes the Euclidean norm.

Then, \( {\mathbf{H}}_{1}\left( {u}_{k}\right) \) is selected as the unit vector of \( z \) -axis \( \mathbf{z}\left( {u}_{k}\right)  = \left\lbrack  {{z}_{1}\left( {u}_{k}\right) }\right. \) , \( {\left. {z}_{2}\left( {u}_{k}\right) ,{z}_{3}\left( {u}_{k}\right) \right\rbrack  }^{T} \) at \( {u}_{k} \) . For the three interpolated NURBS curves,it is difficult to ensure that the vectors \( {\mathbf{H}}_{1}\left( {u}_{k}\right) \) and \( {\mathbf{H}}_{2}\left( {u}_{k}\right) \) remain absolutely perpendicular and equidistant during the movement of the robot, but the three points must be on the same plane,and the \( x \) -axis is perpendicular to the plane and passes through the position point. Hence, the unit vector of the \( x \) -axis at \( {u}_{k} \) is determined as

\[\mathbf{x}\left( {u}_{k}\right)  = {\left\lbrack  {x}_{1}\left( {u}_{k}\right) ,{x}_{2}\left( {u}_{k}\right) ,{x}_{3}\left( {u}_{k}\right) \right\rbrack  }^{T} = \frac{\operatorname{cross}\left( {{\mathbf{H}}_{1}\left( {u}_{k}\right) ,{\mathbf{H}}_{2}\left( {u}_{k}\right) }\right) }{\begin{Vmatrix}\operatorname{cross}\left( {\mathbf{H}}_{1}\left( {u}_{k}\right) ,{\mathbf{H}}_{2}\left( {u}_{k}\right) \right) \end{Vmatrix}} \tag{9}\]

where \( \operatorname{cross}\left( \cdot \right) \) is the cross product. After obtaining the unit vectors of the \( z \) -axis and \( x \) -axis,according to the orthogonal relationship of coordinate system,the unit vector of \( y \) -axis at \( {u}_{k} \) can be acquired as

\[\mathbf{y}\left( {u}_{k}\right)  = {\left\lbrack  {y}_{1}\left( {u}_{k}\right) ,{y}_{2}\left( {u}_{k}\right) ,{y}_{3}\left( {u}_{k}\right) \right\rbrack  }^{T} = \operatorname{cross}\left( {\mathbf{z}\left( {u}_{k}\right) ,\mathbf{x}\left( {u}_{k}\right) }\right)  \tag{10}\]

Consequently,the rotation matrix at \( {u}_{k} \) can be obtained as

\[\mathbf{R}\left( {u}_{k}\right)  = \left\lbrack  {\mathbf{x}\left( {u}_{k}\right) ,\mathbf{y}\left( {u}_{k}\right) ,\mathbf{z}\left( {u}_{k}\right) }\right\rbrack   = \left\lbrack  \begin{array}{lll} {x}_{1}\left( {u}_{k}\right) & {y}_{1}\left( {u}_{k}\right) & {z}_{1}\left( {u}_{k}\right) \\  {x}_{2}\left( {u}_{k}\right) & {y}_{2}\left( {u}_{k}\right) & {z}_{2}\left( {u}_{k}\right) \\  {x}_{3}\left( {u}_{k}\right) & {y}_{3}\left( {u}_{k}\right) & {z}_{3}\left( {u}_{k}\right)  \end{array}\right\rbrack   \tag{11}\]

It can be found from the above analysis that the rotation matrix at \( {u}_{k} \) can be determined uniquely by the three points \( \left\{  {{C}_{1}\left( {u}_{k}\right) ,{C}_{2}\left( {u}_{k}\right) ,{C}_{3}\left( {u}_{k}\right) }\right\} \) on the triple NURBS curves. Combined with the position, the homogeneous transformation matrix of the robot end-effector at \( {u}_{k} \) can be calculated as

\[\mathbf{T}\left( {u}_{k}\right)  = \left\lbrack  \begin{matrix} \mathbf{R}\left( {u}_{k}\right) & {\mathbf{C}}_{1}\left( {u}_{k}\right) \\  \mathbf{0} & 1 \end{matrix}\right\rbrack   \tag{12}\]

It can be seen from Eq. (12) that the position and the rotation matrix representing the orientation can be obtained simultaneously by the same parameter value. Besides, compared with Euler angles and quaternion, which employ three and four quantities to describe the orientation, respectively, using triple NURBS curves requires six quantities to describe the orientation (except three position quantities). However, using triple NURBS curves to obtain the orientation can significantly reduce the amount of storage, since only the control point coordinates and weight factors of the other two curves need to be stored.

## 3. Proposed robot pose trajectory planning method

When moving along a specific path, the position and orientation of the robot end-effector will change constantly, and the angular velocity is important to ensure the smooth change of the end-effector. On the basis of the triple NURBS curve paths generated in the previous section, this section gives an approximate time-optimal pose trajectory planning method with limited linear jerk and continuous bounded angular velocity, as shown in Fig. 3. The method has two main modules: velocity extremum curve generation and bidirectional interpolation of path segments. The first module aims to obtain the velocity extremum curve under a given path and segment the path. For the input path in the form of triple NURBS curves,first,the curvature \( \kappa \left( {u}_{k}\right) \) of the position path at \( {u}_{k} \) is calculated,and the maximum allowable linear velocity \( {v}_{m}\left( {u}_{k}\right) \) under different constraints is acquired. Then, whether the maximum velocity meets the kinematic constraints of all joint axes of the robot is checked. If not, the bisection search algorithm is used to obtain the feasible maximum velocity that meets the kinematic constraints of the robot. Finally, the obtained velocity extremum curve is scanned, and the path is segmented by using the local minimum velocity as the segmentation point, so as to determine the starting and ending point parameters and velocities and the maximum allowable velocity of each path segment, and these data are stored in the characteristic data buffer. The second module is to utilize the S-type acceleration and deceleration method to plan under the velocity extremum curve, and output the information of each path segment, such as the starting and ending point velocities, the maximum velocity within the segment, the length and time of each segment, etc. Then, a series of desired positions and orientations can be acquired using proper interpolation algorithm such as second-order Taylor expansion. After the above operations, the pose trajectory of the robot can be obtained.

<!-- Meanless: 4-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

### 3.1. Velocity extremum curve generation

Velocity extremum curve generation is to prescan the position path to get the maximum allowable velocity at each path point, which satisfies the constraints such as the allowable tangential velocity, tangential acceleration, tangential jerk and path deviation of the robot end-effector and curve curvature, etc.

For the NURBS curve interpolation, the interpolation points are all located on the curve, which means that there is no radial error and only the chord error is involved. For a given maximum chord error \( {\delta }_{m} \) ,the maximum allowable linear velocity \( {v}_{mh} \) at \( {u}_{k} \) is

\[{v}_{mh}\left( {u}_{k}\right)  = \left\{  \begin{matrix} V,\;\text{ if }\frac{2}{{T}_{s}}\sqrt{{\delta }_{m}\left( {{2\rho }\left( {u}_{k}\right)  - {\delta }_{m}}\right) } > V \\  \frac{2}{{T}_{s}}\sqrt{{\delta }_{m}\left( {{2\rho }\left( {u}_{k}\right)  - {\delta }_{m}}\right) },\;\text{ if }\frac{2}{{T}_{s}}\sqrt{{\delta }_{m}\left( {{2\rho }\left( {u}_{k}\right)  - {\delta }_{m}}\right) } \leq  V \end{matrix}\right.  \tag{13}\]

where \( V \) is the commanded velocity,and \( {T}_{s} \) is the interpolation period, and \( \rho \left( {u}_{k}\right)  = 1/\kappa \left( {u}_{k}\right) \) is the radius of curvature. For the position path,the curvature at \( {u}_{k} \) can be calculated as

\[\kappa \left( {u}_{k}\right)  = \frac{\begin{Vmatrix}\operatorname{cross}\left( {\mathbf{C}}^{\prime }{}_{1}\left( {u}_{k}\right) ,{\mathbf{C}}^{\prime \prime }{}_{1}\left( {u}_{k}\right) \right) \end{Vmatrix}}{{\begin{Vmatrix}{\mathbf{C}}^{\prime }{}_{1}\left( {u}_{k}\right) \end{Vmatrix}}^{3}} \tag{14}\]

where \( {\mathbf{C}}^{\prime }{}_{1}\left( {u}_{k}\right) \) and \( {\mathbf{C}}^{\prime \prime }{}_{1}\left( {u}_{k}\right) \) are the first-order and second-order derivatives of the position path at \( {u}_{k} \) ,respectively,and the specific calculation process can be referred to Ref. [45]. It can be found from Eq. (13) that under the constraint of the same chord error, the allowable velocity in the flat area of the curve is greater than that in the large curvature area.

The linear velocity should also satisfy the constraints of normal acceleration and normal jerk as

\[{v}_{mc}\left( {u}_{k}\right)  = \sqrt{\frac{{a}_{m}}{\kappa \left( {u}_{k}\right) }},{v}_{mj}\left( {u}_{k}\right)  = \sqrt[3]{\frac{{j}_{m}}{{\kappa }^{2}\left( {u}_{k}\right) }} \tag{15}\]

where \( {a}_{m} \) and \( {j}_{m} \) are the maximum allowable normal acceleration and jerk,respectively,and \( {v}_{mc} \) and \( {v}_{mj} \) are the maximum allowable velocity at \( {u}_{k} \) under the constraints of the normal acceleration and jerk, respectively. It is worth noting that the maximum normal acceleration and normal jerk can usually be selected to be equal to the maximum tangential acceleration and tangential jerk, respectively.

In the area with large path curvature, the chord deviation, velocity, acceleration and the driving force of the motors will change abruptly. To improve this situation, the maximum linear velocity constraint based on the geometric characteristics of the path is introduced as

\[{v}_{mg}\left( {u}_{k}\right)  = \frac{{\kappa }_{c}}{\kappa \left( {u}_{k}\right)  + {\kappa }_{c}}V \tag{16}\]

where \( {\kappa }_{c} \) is a constant used to maintain the derivative continuity of \( {v}_{mg}\left( {u}_{k}\right) \) .

The sharp change of the robot position and orientation will affect the task quality, such as the workpiece quality in the robotic machining process. To ensure the smooth motion of the robot, except for the linear velocity constraints on the position of the end-effector, the angular velocity of the orientation of the end-effector should also be constrained. When the robot end-effector moves along the given path, the linear velocity can be calculated as

\[v\left( {u}_{k}\right)  = {\begin{Vmatrix}\frac{d{\mathbf{C}}_{1}\left( u\right) }{dt}\end{Vmatrix}}_{u = {u}_{k}} = {\begin{Vmatrix}{\mathbf{C}}^{\prime }{}_{1}\left( u\right) \end{Vmatrix}}_{u = {u}_{k}} \cdot  {\left. \frac{du}{dt}\right| }_{t = {t}_{k}} \tag{17}\]

According to Eq. (17),the time derivative of the parameter \( u \) at \( {u}_{k} \) can be obtained as

\[{\left. \frac{du}{dt}\right| }_{t = {t}_{k}} = \frac{v\left( {u}_{k}\right) }{\begin{Vmatrix}{\mathbf{C}}^{\prime }\left( {u}_{k}\right) \end{Vmatrix}} \tag{18}\]

Because the rotation matrix at \( {u}_{k} \) can be calculated using Eq. (11), and let \( \mathbf{R}\left( {u}_{k}\right) \) and \( \mathbf{R}\left( {{u}_{k} + {\Delta u}}\right) \) be the orientations corresponding to the parameters \( {u}_{k} \) and \( {u}_{k} + {\Delta u} \) ,respectively,where \( {\Delta u} \) is a small increment, the transformation relationship between these two orientations can be expressed as

\[{\mathbf{R}}_{\Delta u} = {\mathbf{R}}^{T}\left( {u}_{k}\right) \mathbf{R}\left( {{u}_{k} + {\Delta u}}\right)  = \left\lbrack  \begin{array}{lll} {r}_{11} & {r}_{12} & {r}_{13} \\  {r}_{21} & {r}_{22} & {r}_{23} \\  {r}_{31} & {r}_{32} & {r}_{33} \end{array}\right\rbrack   \tag{19}\]

where \( {r}_{ij} \) is the element of row \( i \) and column \( j \) of \( {\mathbf{R}}_{\Delta u} \) . To calculate the angular velocity, the axis/angle form is used here, and the corresponding angle \( {\vartheta }_{a} \) and unit axis \( {\mathbf{r}}_{a} \) are obtained as

\[{\vartheta }_{a} = \arccos \left( \frac{{r}_{11} + {r}_{22} + {r}_{33} - 1}{2}\right) ,{\mathbf{r}}_{a} = \frac{1}{2\sin {\vartheta }_{a}}\left\lbrack  \begin{matrix} {r}_{32} - {r}_{23} \\  {r}_{13} - {r}_{31} \\  {r}_{21} - {r}_{12} \end{matrix}\right\rbrack   \tag{20}\]

Combining Eqs. (18) and (20), it can be shown that the angular velocity of the robot end-effector at \( {u}_{k} \) is

\[\mathbf{\omega }\left( {u}_{k}\right)  = {\left. \frac{{\mathbf{r}}_{a}{\vartheta }_{a}}{dt}\right| }_{t = {t}_{k}} = {\left. \frac{{\mathbf{r}}_{a}{\vartheta }_{a}}{du}\frac{du}{dt}\right| }_{t = {t}_{k}} = \frac{{\mathbf{r}}_{a}{\vartheta }_{a}}{du}\frac{v\left( {u}_{k}\right) }{\begin{Vmatrix}{\mathbf{C}}^{\prime }\left( {u}_{k}\right) \end{Vmatrix}} \tag{21}\]

According to Eq. (21), the maximum linear velocity satisfying the maximum allowable angular velocity constraint \( {\omega }_{m} \) can be computed as

\[{v}_{ma}\left( {u}_{k}\right)  = \frac{{\omega }_{m}{\Delta u}\begin{Vmatrix}{{\mathbf{C}}^{\prime }{}_{1}\left( {u}_{k}\right) }\end{Vmatrix}}{{\vartheta }_{a}} \tag{22}\]

Therefore,for the \( {u}_{k} \) on the triple NURBS curves,by combining Eqs. (13), (15), (16) and (22), the maximum allowable linear velocity under different constraints can be calculated as \( {v}_{m}\left( {u}_{k}\right)  = \min \left\{  {{v}_{mh}\left( {u}_{k}\right) ,{v}_{mc}\left( {u}_{k}\right) }\right. \) , \( \left. {{v}_{mj}\left( {u}_{k}\right) ,{v}_{mg}\left( {u}_{k}\right) ,{v}_{ma}\left( {u}_{k}\right) }\right\} \) ,where \( \min \{  \cdot  \} \) denotes the minimum value. When the above algorithm is executed periodically until the parameter \( {u}_{k} = 1 \) ,the velocity extremum curve on the whole path can be acquired. It is obvious that the velocity extremum curve is the upper limit of all reasonable velocity planning curves on the path. If the planned velocity curve crosses over the extremum curve, the motion velocity of the robot cannot meet the requirements, or the robot does not move according to the specified path. Hence, the velocity planning is performed in the area below the extremum curve.

<!-- Meanless: 5-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

Although the velocity extremum curve that satisfies different constraints can be acquired through the above process, since the motion of the robot is achieved in the joint space, even under the Cartesian space planning, it is necessary to check whether each point on the velocity extremum curve satisfies the kinematic constraints of all joint axes of the robot, that is

\[\begin{Vmatrix}{\dot{q}}_{i}\end{Vmatrix} \leq  {V}_{i},\begin{Vmatrix}{\ddot{q}}_{i}\end{Vmatrix} \leq  {A}_{i},\begin{Vmatrix}{\dddot{q}}_{i}\end{Vmatrix} \leq  {J}_{i},\;i = 1,2,\ldots ,6 \tag{23}\]

where \( {\dot{q}}_{i},{\ddot{q}}_{i},{q}_{i} \) are the velocity,acceleration and jerk of the \( i \) th joint axis, respectively,and \( {V}_{i},{A}_{i},{J}_{i} \) are their upper limits,respectively. When the velocity extremum at a point on the curve does not meet the joint kinematics constraints, by taking zero and the current velocity extremum as the boundary, the bisection search algorithm can be effectively adopted to find iteratively a kinematically feasible velocity extremum within a specified tolerance.

Furthermore, the target of velocity planning is to plan a velocity-time curve under the velocity extremum curve. Therefore, if the whole path is planned only once, it may be very difficult to satisfy the requirements of accuracy, especially in the places with large position curvature or sharp orientation changes. For avoiding the above problems, a more strict path must be selected to allow the maximum velocity, which seriously influences the motion efficiency of the robot. Thus, it is necessary to segment the path according to the extremum curve, and perform segmented acceleration and deceleration planning. The specific strategy is to scan the velocity extremum curve of the whole path prospectively, and segment the path into a series of parameter sub-intervals \( \left\lbrack  {{u}_{s},{u}_{s + 1}}\right\rbrack \) using the minimum value of local velocity,where the subscript \( s \) denotes the \( s \) -th path segment,and store the starting and ending point parameter values \( {u}_{s},{u}_{s + 1} \) of each path segment,their corresponding velocities \( {v}_{us} \) , \( {v}_{{us} + 1} \) and maximum allowable velocity \( {v}_{ms} \) in the characteristic data buffer.

It is worth noting that the acceleration and jerk constraints are considered as shown in Eq. (15). However, because this only constrains the linear acceleration and the linear jerk, and it is still difficult to synchronously constrain the angular acceleration and the angular jerk in the velocity extremum curve generation, the above algorithm can only ensure that the linear jerk is limited, but not the angular jerk. Nevertheless, according to the survey of the existing 6-DOF robot pose trajectory planning work in the introduction part, it is a good progress to be able to explicitly constrain the angular velocity when generating the velocity extremum curve, since most of the work does not explicitly constrain the angular velocity, which may lead to the angular velocity exceeding the constraint. In addition, in some robot applications such as machining and assembly, the orientation change is not very large, so although the proposed method only limits the angular velocity, it can still ensure practicality.

In addition, the dynamic constraints are not taken into account. This is mainly because the dynamics of the six-DOF robot is very complex, and with the dynamic constraints, the trajectory planning problem of the robot will become more complex, which requires optimization or iterative techniques to solve. Although the dynamic constraints are not considered in the above velocity extremum curve generation, it can still be a practical method, which is mainly based on two considerations. First of all, many robot applications do not need the high velocity of the robot, so only considering the geometric and kinematic constraints can also obtain good results. Second, in high-velocity or fast response applications, the slightly conservative geometric and kinematic constraints can be given to enable the robot to respond normally even without considering the dynamic constraints.

### 3.2. Bidirectional interpolation of path segments

Owing to the smooth scheduling and controllable characteristics of the acceleration and deceleration (Acc/Dec), S-type Acc/Dec control mode is often used for the velocity planning or interpolation. However, the traditional S-type Acc/Dec mode based interpolation is usually performed in one direction, and it is difficult to obtain the velocity constraint information in real time, resulting in some curve regions exceeding the constraints. To address this problem, the bidirectional S-type Acc/Dec mode based interpolation algorithm, namely bidirectional interpolation,can be used to interpolate the path segments \( \left\lbrack  {{48},{49}}\right\rbrack \) . Bidirectional interpolation has two advantages. First, it can guarantee the adaptive velocity planning under the velocity extremum curve, and always meet the velocity continuity constraint. Second, the acceleration phase is carried out through forward interpolation, and the deceleration phase is carried out through backward interpolation, and a smooth velocity curve can be generated by ensuring that the interpolation in these two directions meets under zero acceleration. Here, based on the work in [49], an improved bidirectional interpolation algorithm is designed as follows.

First, in the interpolation process, each interpolation point should satisfy the following inequalities as

\[{v}_{k} \leq  {v}_{m}\left( {u}_{k}\right) ,{a}_{k} \leq  {a}_{m},{j}_{k} \leq  {j}_{m} \tag{24}\]

where \( {v}_{k},{a}_{k} \) and \( {j}_{k} \) are the velocity,acceleration and jerk in the \( k \) -th interpolation period, respectively. According to the results in [49], when the information of the \( k \) -th interpolation period is known,the information of the next (i.e. \( k + 1 \) ) interpolation period can be calculated as

\[\left\{  \begin{array}{l} {a}_{k + 1} = \min \left( {{a}_{m},{a}_{k} + {j}_{m}{T}_{s},\frac{-{j}_{m}{T}_{s} + \sqrt{{j}_{m}^{2}{T}_{s}^{2} - 4{j}_{m}\left\lbrack  {{a}_{k}{T}_{s} - 2\left( {{v}_{ms} - {v}_{k}}\right) }\right\rbrack  }}{2}}\right) \\  {v}_{k + 1} = \min \left( {{v}_{m}\left( {u}_{k}\right) ,{v}_{k} + \frac{\left( {{a}_{k + 1} + {a}_{k}}\right) {T}_{s}}{2}}\right)  \end{array}\right. \]

(25)

where \( {v}_{ms} \) is the maximum allowable velocity of the path segment being interpolated.

Then, to realize efficient and smooth bidirectional interpolation, the following three main problems need to be solved: 1) an adaptive forward and backward interpolation scheduling strategy is required to determine the forward and backward interpolation sequence; 2) how to ensure that the acceleration of the forward and backward interpolation is reduced to zero when they meet; 3) how to adjust the endpoint velocity to avoid sudden change of the velocity when the velocity of the forward and backward interpolation does not satisfy the meeting requirements.

## 1) Adaptive scheduling of the forward and backward interpolation

When the forward interpolation and the backward interpolation meet, their acceleration must be zero. Supposing that the velocity and acceleration of the forward interpolation in the \( k \) -th interpolation period are \( {v}_{fk} \) and \( {a}_{fk} \) ,respectively,and the velocity and acceleration of the backward interpolation are \( {v}_{bk} \) and \( {a}_{bk} \) ,respectively,it can be obtained that

\[\left\{  \begin{array}{l} {v}_{mfk} = {v}_{fk} + {a}_{fk}{t}_{fk} - \frac{{j}_{m}{t}_{fk}^{2}}{2} = {v}_{fk} + \frac{{a}_{fk}^{2}}{2{j}_{m}} \\  {v}_{mbk} = {v}_{bk} + {a}_{bk}{t}_{bk} - \frac{{j}_{m}{t}_{bk}^{2}}{2} = {v}_{bk} + \frac{{a}_{bk}^{2}}{2{j}_{m}} \end{array}\right.  \tag{26}\]

where \( {v}_{mfk} \) and \( {v}_{mbk} \) are the maximum velocities that can be reached when the acceleration of the forward and backward interpolation is reduced to zero in the \( k \) -th interpolation period,respectively. \( {t}_{fk} \) and \( {t}_{bk} \) are the corresponding time,i.e. \( {t}_{fk} = {a}_{fk}/{j}_{m},{t}_{bk} = {a}_{bk}/{j}_{m} \) . If \( {v}_{mfk} > {v}_{mbk} \) ,a backward interpolation is performed; otherwise, a forward interpolation is performed.

1) Meeting processing of the forward and backward interpolation

<!-- Meanless: 6-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

<!-- Media -->

<!-- figureText: Load path segment parameters Y Backward interpolation N \( {d}_{r} \leq  {d}_{fk} + {d}_{bk} \) Y Reset \( {v}_{msc},{v}_{mfk},{v}_{mbk},{j}_{fk},{j}_{bk} \) using Eqs. (30) and (32) \( {v}_{fk} > {v}_{bk} \) N Update \( {j}_{fk} \) using Eq. (30) Forward interpolation \( \mathrm{N} \) \( {v}_{tk} = {v}_{msc} \) and \( {v}_{bk} = {v}_{m} \) Initialize \( {u}_{fk} = {u}_{s},{u}_{bk} = {u}_{s + 1},{v}_{fk} = {v}_{us},{v}_{bk} = \) \( {v}_{{us} + 1} \) and calculate \( {v}_{mfk},{v}_{mbk} \) using Eq. (26) \( \mathrm{N} \) \( {v}_{mfk} > {v}_{mbk} \) Forward interpolation \( \mathrm{N} \) \( {v}_{fk} = {v}_{ms} \) and \( {v}_{bk} = {v}_{m} \) Y Forward interpolation Y Update \( {j}_{bk} \) using Eq. (30) Y \( {u}_{bk} > {u}_{fk} \) Backward interpolation \( \mathrm{N} \) Y End -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_6.jpg?x=234&y=153&w=1277&h=1370&r=0"/>

Fig. 4. flowchart of the improved bidirectional interpolation of path segments.

<!-- Media -->

If the path segment of the curve is long, the forward and backward interpolation can reach the maximum velocity,i.e. \( {v}_{fk} = {v}_{bk} = {v}_{ms} \) . In this case, the backward interpolation will be stopped and the meeting point will be recorded, and the forward interpolation will continue to the meeting point at the maximum velocity. If the path segment is short, it may be difficult to reach the maximum velocity \( {v}_{ms} \) . To ensure that the acceleration of the forward and backward interpolation at the meeting point is reduced to zero, it is necessary to reduce the acceleration in advance. When the deceleration starts from the current position and both sides meet at the same velocity \( {v}_{msc} \) (obviously \( {v}_{msc} < {v}_{ms} \) ,and \( {v}_{msc} \) will be adopted to replace \( {v}_{ms} \) to calculate Eq. (25) in this case),the distance they pass is

\[{d}_{fk} + {d}_{bk} = {v}_{fk}{t}_{fk} + {v}_{bk}{t}_{bk} + \frac{{a}_{fk}{t}_{fk}^{2} + {a}_{bk}{t}_{bk}^{2}}{2} - \frac{{j}_{m}\left( {{t}_{fk}^{3} + {t}_{bk}^{3}}\right) }{6} + c\left( {{v}_{mfk} + {v}_{mbk}}\right) \]

(27)

where \( {d}_{fk} \) and \( {d}_{bk} \) are the distances that the velocity of the forward and backward interpolation decelerates to \( {v}_{msc} \) ,respectively,and \( c \) is a coefficient related to the path, which can ensure that the acceleration is zero when the forward interpolation and the backward interpolation meet. Thus,when the remaining path segment length \( {d}_{r} \) satisfies \( {d}_{r} \leq  {d}_{fk} \) \( + {d}_{bk} \) ,it immediately enters the deceleration state,where \( {d}_{r} \) can be obtained using the adaptive Simpson's method with a given error bound \( {\varepsilon }_{m} \) . Since \( {v}_{mfk} \) and \( {v}_{mbk} \) may not be equal,the larger of them is usually selected as the velocity \( {v}_{msc} \) when the interpolation in two directions meets, i.e.

\[{v}_{msc} = \max \left( {{v}_{mfk},{v}_{mbk}}\right)  \tag{28}\]

where \( \max \left( \cdot \right) \) denotes the maximum value.

It can be seen from Eq. (26) that in order to ensure that the side with smaller velocity can reach the velocity \( {v}_{msc} \) when meeting,the jerk needs to be adjusted as

<!-- Meanless: 7-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

\[\left\{  \begin{array}{l} {j}_{fk} = \frac{{a}_{fk}^{2}}{2\left( {{v}_{msc} - {v}_{fk}}\right) },{j}_{bk} = {j}_{m},\text{ if }{v}_{mfk} < {v}_{mbk} \\  {j}_{bk} = \frac{{a}_{bk}^{2}}{2\left( {{v}_{msc} - {v}_{bk}}\right) },{j}_{jk} = {j}_{m},\text{ if }{v}_{mfk} > {v}_{mbk} \end{array}\right.  \tag{29}\]

<!-- Media -->

<!-- figureText: (a) (b) -1.8 -2 \( \gamma \) [rad] -2.2 -2.4 -2.6 -0.5 \( \alpha \) [rad] - 1 -1.8 -1.2 -1.4 -1.6 \( \beta \) [rad] 160 Z [mm] 140 120 30 -20 -30 60 -40 X [mm] 70 -50 Y [mm] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_7.jpg?x=238&y=151&w=1262&h=477&r=0"/>

Fig. 5. Blade-shaped curve. (a) Position spline; (b) Orientation spline.

Table 1

Velocity planning parameters for the blade-shaped curve.

<table><tr><td>Parameters</td><td>Values</td><td>Parameters</td><td>Values</td></tr><tr><td>Interpolation period \( {T}_{s} \)</td><td>1 ms</td><td>Commanded velocity \( V \)</td><td>80 mm/s</td></tr><tr><td>Arc length calculation error bound \( {\varepsilon }_{m} \)</td><td>\( {10}^{-8}\mathrm{\;{mm}} \)</td><td>Maximum acceleration \( {a}_{m} \)</td><td>400 mm/ \( {s}^{2} \)</td></tr><tr><td>Maximum chord error \( {\delta }_{m} \)</td><td>0.5μm</td><td>Maximum jerk \( {j}_{m} \)</td><td>2500 mm/ \( {\mathrm{s}}^{3} \)</td></tr><tr><td>Maximum angular velocity \( {\omega }_{m} \)</td><td>1.35 rad/s</td><td>Curvature constant \( {\kappa }_{c} \)</td><td>1</td></tr></table>

<!-- Media -->

where \( {j}_{fk} \) and \( {j}_{bk} \) are the jerks of the forward and backward deceleration phases, respectively.

However,if only one jerk adjustment is made after \( {d}_{r} \leq  {d}_{fk} + {d}_{bk} \) is satisfied, when the deceleration movement with constant jerk is constrained by the extremum curve during the forward and backward interpolation, at least one side cannot reach the maximum velocity, and the acceleration of one side may not be guaranteed to be zero when the forward interpolation and the backward interpolation meet, which will lead to a large jerk impact. To address this problem,when \( {d}_{r} \leq  {d}_{fk} + {d}_{bk} \) is satisfied, the jerk is adjusted as

<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

\[\left\{  \begin{array}{l} {j}_{fk} = \frac{{a}_{fk}^{2}}{2\left( {{v}_{msc} - {v}_{fk}}\right) },{j}_{bk} = {j}_{m},\text{ if }{v}_{fk} < {v}_{bk} \\  {j}_{bk} = \frac{{a}_{bk}^{2}}{2\left( {{v}_{msc} - {v}_{bk}}\right) },{j}_{fk} = {j}_{m},\text{ if }{v}_{fk} > {v}_{bk} \end{array}\right.  \tag{30}\]

<!-- Media -->

<!-- figureText: (a) (b) 80 Linear velocity [mm/s] 60 40 20 0 0.5 1 1.5 2 2.5 Time [sec] (d) 3000 2000 Linear jerk [mm \( /{\mathrm{s}}^{3} \) ] 1000 0 -1000 -2000 -3000 0.5 1.5 2 2.5 Time [sec] 100 Extremum curve Planning curve 80 Velocity [mm/s] 60 40 20 0.2 0.4 0.6 0.8 1 U (c) Linear acceleration \( \left\lbrack  {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right\rbrack \) 400 200 -200 -400 0 0.5 1.5 2 2.5 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_7.jpg?x=238&y=1109&w=1273&h=1009&r=0"/>

Fig. 6. Results of the proposed planning approach for the blade-shaped curve. (a) Velocity extremum and planning curves under parameter \( u \) ; (b) Linear velocity; (c) Linear acceleration; (d) Linear jerk.

<!-- Meanless: 8-->


<!-- figureText: (a) 1.5 (b) 10 Angular acceleration \( \left\lbrack  {\mathrm{{rad}}/{\mathrm{s}}^{2}}\right\rbrack \) -5 -10 0.5 1 1.5 2 2.5 Time [sec] Angular velocity [rad/s] 1 0.5 0 0.5 1 1.5 2 2.5 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_8.jpg?x=245&y=150&w=1259&h=500&r=0"/>

Fig. 7. Results of the proposed planning approach for the blade-shaped curve. (a) Angular velocity; (b) Angular acceleration.

<!-- figureText: X [mm]☑ -330 (b) \( \alpha \left\lbrack  \mathrm{{rad}}\right\rbrack \) -0.5 - 1 0 0.5 1 1.5 2 2.5 Time [sec] 3 [rad] -1.4 0.5 1.5 2 2.5 Time [sec] \( \gamma \left\lbrack  \mathrm{{rad}}\right\rbrack \) -2.5 0.5 1 1.5 2 2.5 Time [sec] -370 0 0.5 1.5 2 2.5 Time [sec] Y [mm] 380 360 0 0.5 1.5 2 2.5 Time [sec] Z [mm] 460 420 0.5 1.5 2 2.5 0 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_8.jpg?x=240&y=743&w=1267&h=487&r=0"/>

Fig. 8. Pose trajectory planned by the proposed approach for the blade-shaped curve. (a) Position trajectory; (b) Orientation trajectory.

<!-- figureText: (a) (b) -1.4 -1.6 \( \gamma \) [rad] -1.8 -2 -2.2 -1.6 -2.2 -2.4 -2.6 -1.5 -2.8 -3 -1.4 \( \beta \) [rad] \( \alpha \) [rad] 10 Z [mm] 5 0 -5 -100 100 -50 50 Y [mm] 0 0 X [mm] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_8.jpg?x=240&y=1346&w=1257&h=432&r=0"/>

Fig. 9. Fan-shaped curve. (a) Position spline; (b) Orientation spline.

<!-- Media -->

After entering the deceleration state, the acceleration of the forward interpolation and the backward interpolation can be calculated as

\[\left\{  \begin{array}{l} {a}_{f,k + 1} = \max \left( {{a}_{fk} - {j}_{fk}{T}_{s},0}\right) \\  {a}_{b,k + 1} = \max \left( {{a}_{bk} - {j}_{bk}{T}_{s},0}\right)  \end{array}\right.  \tag{31}\]

## 1) Velocity adjustment at the endpoint

When the length of the path segment is very short, and the velocity difference between the given two endpoints is large, only one end continues to accelerate with the maximum jerk \( {j}_{m} \) until \( {d}_{r} \leq  {d}_{fk} + {d}_{bk} \) is satisfied, and then decelerates. In this situation, it is possible that the velocity at the other end is still less than the velocity \( {v}_{msc} \) when the forward interpolation and the backward interpolation meet, and the velocity of the endpoint with larger velocity should be set to \( {v}_{msc} \) . Therefore, Eq. (28) is modified as

<!-- Meanless: 9-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

\[{v}_{msc} = \left\{  \begin{array}{l} {v}_{mfk},\text{ if }{a}_{fk} \neq  0,{a}_{bk} = 0 \\  {v}_{mbk},\text{ if }{a}_{fk} = 0,{a}_{bk} \neq  0 \\  \max \left( {{v}_{mfk},{v}_{mbk}}\right) ,\text{ if }{a}_{fk} \neq  0,{a}_{bk} \neq  0 \end{array}\right.  \tag{32}\]

<!-- Media -->

<!-- figureText: (a) 00 (b) 80 Linear velocity [mm/s] 60 40 20 1 2 3 Time [sec] (d) 3000 2000 Linear jerk \( \left\lbrack  {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right\rbrack \) 1000 0 -1000 -2000 -3000 1 2 3 4 Time [sec] Extrumum curve Planning curve 80 Velocity [mm/s] 60 40 20 0 0.2 0.4 0.6 0.8 1 (c) Linear acceleration \( \left\lbrack  {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right\rbrack \) 400 200 -200 -400 0 1 2 3 4 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_9.jpg?x=236&y=146&w=1275&h=1011&r=0"/>

Fig. 10. Results of the proposed planning approach for the fan-shaped curve. (a) Velocity extremum and planning curves under parameter \( u \) ; (b) Linear velocity; (c) Linear acceleration; (d) Linear jerk.

<!-- figureText: (a) (b) 2 Angular acceleration \( \left\lbrack  {\mathrm{{rad}}/{\mathrm{s}}^{2}}\right\rbrack \) 0 0 1 3 4 Time [sec] Angular velocity [rad/s] 0.5 0.4 0.3 0.2 0.1 1 2 3 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_9.jpg?x=243&y=1279&w=1263&h=499&r=0"/>

Fig. 11. Results of the proposed planning approach for the fan-shaped curve. (a) Angular velocity; (b) Angular acceleration.

<!-- Media -->

After the velocity planning is completed, the interpolation algorithm determines the arc length increment of the \( k \) -th period by sampling at a fixed period in the time law,and then determines the curve parameter \( u \) to obtain the desired pose trajectory. Since the arc length of spline curve is a non-analytical integral form of the curve parameter, the mapping relationship from the curve arc length to the curve parameter may not be calculated accurately. At present, numerical methods are usually used to approximately calculate the curve parameter, including first-order and second-order Taylor expansions, velocity correction polynomials and Runge-Kutta, etc. [50]. Here, the second-order Taylor expansion is classically adopted as

<!-- Meanless: 10-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

\[{u}_{k + 1} = {u}_{k} + \frac{1}{\begin{Vmatrix}{\mathbf{C}}_{1}^{\prime }\left( {u}_{k}\right) \end{Vmatrix}}\Delta {s}_{k} - \frac{\operatorname{inner}\left( {{\mathbf{C}}_{1}^{\prime }\left( {u}_{k}\right) ,{\mathbf{C}}_{1}^{\prime \prime }\left( {u}_{k}\right) }\right) }{2{\begin{Vmatrix}{\mathbf{C}}_{1}^{\prime }\left( {u}_{k}\right) \end{Vmatrix}}^{4}}\Delta {s}_{k}^{2} \tag{33}\]

<!-- Media -->

<!-- figureText: X [mm]% 400 \( \alpha \) [rad] \( \subset \) -2.4 -2.6 0 0.5 1.5 2 Time [sec] \( \beta \) [rad] -1.4 -1.6 0 1 3 4 Time [sec] \( \gamma \left\lbrack  \mathrm{{rad}}\right\rbrack \) -1.5 1 2 3 Time [sec] 300 0 1 2 3 Time [sec] Y [mm] 0 -100 0 1 2 3 Time [sec] Z [mm] 655 645 0 1 2 3 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_10.jpg?x=238&y=149&w=1269&h=489&r=0"/>

Fig. 12. Pose trajectory planned by the proposed approach for the fan-shaped curve. (a) Position trajectory; (b) Orientation trajectory.

<!-- figureText: Windows 7 Ethernet Ethernet ROKAE industrial robot VC++ 2019 Trajectory planning Control board Results : \( {\theta }_{1}\left( {k{T}_{s}}\right) ,\square ,{\theta }_{6}\left( {k{T}_{s}}\right) \) MATLAB R2017a Simulink8.1/RTW Driver box -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_10.jpg?x=244&y=729&w=1259&h=553&r=0"/>

Fig. 13. Experimental platform.

Table 2

DH parameters of the ROKAE robot (unit: mm or rad).

<table><tr><td>Link</td><td>\( {\alpha }_{i} \)</td><td>\( {\mathrm{a}}_{\mathrm{i}} \)</td><td>\( {\mathrm{d}}_{\mathrm{i}} \)</td><td>\( {\mathrm{q}}_{\mathrm{i}} \)</td></tr><tr><td>b</td><td>0</td><td>0</td><td>0</td><td>0</td></tr><tr><td>1</td><td>0</td><td>0</td><td>342</td><td>\( {\mathrm{q}}_{1} \)</td></tr><tr><td>2</td><td>- \( \pi /2 \)</td><td>40</td><td>0</td><td>\( {\mathrm{q}}_{2} - \pi /2 \)</td></tr><tr><td>3</td><td>0</td><td>275</td><td>0</td><td>\( {\mathrm{q}}_{3} \)</td></tr><tr><td>4</td><td>- \( \pi /2 \)</td><td>25</td><td>280</td><td>\( {\mathrm{q}}_{4} \)</td></tr><tr><td>5</td><td>\( \pi /2 \)</td><td>0</td><td>0</td><td>\( {\mathrm{q}}_{5} \)</td></tr><tr><td>e</td><td>- \( \pi /2 \)</td><td>0</td><td>73</td><td>\( {q}_{6} \)</td></tr></table>

<!-- Media -->

where inner(-) represents the inner product,and \( \Delta {s}_{k} \) is the arc length increment, which can be calculated as

\[\Delta {s}_{k} = {v}_{k}{T}_{s} + \frac{{a}_{k}}{2}{T}_{s}^{2} + \frac{{j}_{k}}{6}{T}_{s}^{3} \tag{34}\]

Taken together, the flowchart of the bidirectional interpolation of path segments based on the S-type Acc/Dec control mode is shown in Fig. 4.

It can be seen from the above process that because the robot pose trajectory is acquired under the generated velocity extremum curve, it is approximate time-optimal. In addition, both the velocity extremum curve generation and bidirectional interpolation algorithms can be analytically calculated without optimization process, so the calculation efficiency is high, and the stability and practicality of the planning are good. Comparatively speaking, the existing planning methods based on the intelligent metaheuristic algorithms may not guarantee the optimal solution, and the planning stability is relatively poor, and the performance depends on the specific problems and the experience of the designers. Besides, since the planning process requires iteration, the calculation is complex, and the efficiency needs to be improved. In general, the intelligent metaheuristic optimization methods and the proposed method have their own advantages and disadvantages, that is, the intelligent metaheuristic methods have advantages in dealing with complex trajectory planning problems, but their stability and efficiency may not be as good as the proposed method.

<!-- Media -->

Table 3

Joint position, velocity and acceleration limits of the ROKAE robot.

<table><tr><td>Joint</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td></tr><tr><td>Position ( \( {}^{ \circ  } \) )</td><td>\( \left\lbrack  {-{170},{170}}\right\rbrack \)</td><td>\( \left\lbrack  {-{84},{130}}\right\rbrack \)</td><td>\( \left\lbrack  {-{188},{50}}\right\rbrack \)</td><td>\( \left\lbrack  {-{170},{170}}\right\rbrack \)</td><td>\( \left\lbrack  {-{117},{117}}\right\rbrack \)</td><td>\( \left\lbrack  {-{360},{360}}\right\rbrack \)</td></tr><tr><td>Velocity \( \left( {{}^{ \circ  }/\mathrm{s}}\right) \)</td><td>440</td><td>355</td><td>440</td><td>490</td><td>450</td><td>720</td></tr><tr><td>Acceleration ( \( {}^{ \circ  }/{\mathrm{s}}^{2} \) )</td><td>1500</td><td>1500</td><td>1500</td><td>1750</td><td>1500</td><td>2500</td></tr></table>

<!-- Meanless: 11-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

<!-- figureText: (a) 100 1.5 2 2.5 Time [sec] \( \times  {10}^{4} \) 1 Linear jerk \( \left\lbrack  {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right\rbrack \) 0.5 0 -0.5 - 1 -1.5 0.5 1.5 2 2.5 Time [sec] linear velocity [mm/s] 80 60 40 20 0 0 0.5 (b) 500 Linear acceleration \( \left\lbrack  {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right\rbrack \) -500 0 0.5 1.5 2 2.5 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_11.jpg?x=240&y=148&w=1270&h=1002&r=0"/>

Fig. 14. Actual linear velocity, linear acceleration and linear jerk for the blade-shaped trajectory.

<!-- figureText: (a) 1.5 (b) 10 Angular acceleration \( \left\lbrack  {\mathrm{{rad}}/{\mathrm{s}}^{2}}\right\rbrack \) 5 0 -5 -10 0 0.5 1 1.5 2 2.5 Time [sec] Angular velocity [rad/s] 1 0.5 0 0.5 1 1.5 2 2.5 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_11.jpg?x=240&y=1239&w=1269&h=504&r=0"/>

Fig. 15. Actual angular velocity and angular acceleration for the blade-shaped trajectory.

<!-- Media -->

## 4. Simulation and experimental results

To validate the effectiveness of the proposed planning approach, numerical simulations and real experiments are performed in this section. First, the numerical simulation results of two example curves are given, and then their real experimental results are presented. The proposed approach was coded utilizing MATLAB software and ran on a 3.6- \( \mathrm{{GHz}} \) personal computer with Windows 11 operating system.

<!-- Meanless: 12-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

<!-- Media -->

<!-- figureText: (a) (b) 3 Actual joint velocity [rad/s] Joint 1 Joint 2 Joint 3 Joint 4 Joint 5 - Joint 6 1 0 0.5 1 1.5 2 2.5 Time [sec] (d) 400 Joint 1 Joint 2 - - - Joint 3 Actual joint jerk \( \left\lbrack  {\mathrm{{rad}}/{\mathrm{s}}^{3}}\right\rbrack \) Joint 4 Joint 5 - - - Joint 6 200 0 -200 0 0.5 1 1.5 2 2.5 Time [sec] 4 Joint 1 Joint 2 - Joint 3 Actual joint position [rad] Joint 4 Joint 6 2 0 -2 -4 0.5 1 1.5 2 2.5 Time [sec] (c) Actual joint acceleration \( \left\lbrack  {\mathrm{{rad}}/{\mathrm{s}}^{2}}\right\rbrack \) Joint 1 Joint 2 - Joint 3 10 Joint 4 Joint 5 - Joint 6 5 0 -5 -10 0.5 1 1.5 2 2.5 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_12.jpg?x=240&y=148&w=1269&h=1009&r=0"/>

Fig. 16. Actual joint position, velocity, acceleration and jerk for the blade-shaped trajectory.

<!-- Media -->

### 4.1. Simulation results

#### 4.1.1. Example 1

The first example curve is blade-shaped curve, as illustrated in Fig. 5. The relationship between the curve coordinate system and the robot base coordinate system is set as

\[{\mathbf{T}}_{b}^{c} = \left\lbrack  \begin{matrix} 1 & 0 & 0 &  - {400} \\  0 & 1 & 0 & {400} \\  0 & 0 & 1 & {300} \\  0 & 0 & 0 & 1 \end{matrix}\right\rbrack  \]

where the unit of translational part is \( \mathrm{{mm}} \) . The appropriate velocity planning parameters are given as shown in Table 1, where the interpolation period and commanded velocity are \( 1\mathrm{\;{ms}} \) and \( {80}\mathrm{\;{mm}}/\mathrm{s} \) , respectively.

For the blade-shaped curve, the planned velocity extremum curve, linear velocity, linear acceleration and linear jerk are shown in Fig. 6, and the planned angular velocity and angular acceleration are shown in Fig. 7. It can be found that the planned linear velocity is completely below the velocity extremum curve, which makes the planned linear velocity not violate the given geometric and kinematic constraints. In addition, the linear acceleration, linear jerk and angular velocity are all within the given limits. Consequently, the proposed planning approach can ensure that the linear jerk is limited and the angular velocity is continuous bounded, which demonstrates the effectiveness of the proposed planning approach. It should be noted that because the maximum allowable angular velocity constraint \( {\omega }_{m} \) is directly considered in the velocity extremum curve generation, the planned angular velocity curve will not exceed the maximum angular velocity value. However, because only the maximum angular velocity is considered, only the continuity of the angular velocity curve can be guaranteed. Finally, by employing the bidirectional interpolation algorithm in Section 3.2, the planned position and orientation trajectories of the robot are shown in Fig. 8.

#### 4.1.2. Example 2

The second example curve is fan-shaped curve, as depicted in Fig. 9, and the relationship between the curve coordinate system and the robot base coordinate system is set as

\[{\mathbf{T}}_{b}^{c} = \left\lbrack  \begin{matrix} 1 & 0 & 0 & {328} \\  0 & 1 & 0 & {46} \\  0 & 0 & 1 & {650} \\  0 & 0 & 0 & 1 \end{matrix}\right\rbrack  \]

where the unit of translational part is \( \mathrm{{mm}} \) .

The velocity planning parameters of the fan-shaped curve are the same as those of the blade-shaped curve, except that the maximum angular velocity \( {\omega }_{m} \) is set to 0.5 rad/s instead of \( {1.35}\mathrm{{rad}}/\mathrm{s} \) ,and the planning results are shown in Fig. 10 and Fig. 11. It can be seen that the planned linear velocity is still completely below the velocity extremum curve. Furthermore, the linear acceleration, linear jerk and angular velocity are also still all within the given limits. Thus, for the fan-shaped curve, the linear jerk is limited and the angular velocity is continuous bounded, which not only shows the effectiveness of the proposed planning approach, but also shows that the proposed approach is independent of the curve path type. Similar to the blade-shaped curve, by using the bidirectional interpolation algorithm, the position and orientation trajectories of the robot can be obtained, as shown in Fig. 12.

In order to verify the computational efficiency of the proposed planning approach, the computing time of the blade-shaped curve and fan-shaped curve is measured. The proposed approach is coded with MATLAB programming language, and the computing platform is a 3.6- \( \mathrm{{GHz}} \) personal computer. It can be seen from the statistics that the mean computing time of each interpolation for the blade-shaped and fan-shaped curves is \( {0.0125}\mathrm{\;{ms}} \) and \( {0.0132}\mathrm{\;{ms}} \) ,respectively. Obviously, the computational efficiency of the proposed method is sufficient for modern robot controllers.

<!-- Meanless: 13-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

<!-- Media -->

<!-- figureText: (a) 100 4 Time [sec] \( \times  {10}^{4} \) 1 Linear jerk [mm \( /{\mathrm{s}}^{3} \) ] 0.5 0 -0.5 - 1 -1.5 0 1 3 4 Time [sec] inear velocity [mm/s] 80 60 40 20 0 0 2 (b) 500 Linear acceleration \( \left\lbrack  {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right\rbrack \) -500 0 1 2 3 4 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_13.jpg?x=240&y=151&w=1274&h=998&r=0"/>

Fig. 17. Actual linear velocity, linear acceleration and linear jerk for the fan-shaped trajectory.

<!-- figureText: (a) (b) Angular acceleration \( \left\lbrack  {\mathrm{{rad}}/{\mathrm{s}}^{2}}\right\rbrack \) 0 0 1 3 4 Time [sec] 0.5 Angular velocity [rad/s] 0.4 0.3 0.2 0.1 1 2 3 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_13.jpg?x=244&y=1242&w=1260&h=500&r=0"/>

Fig. 18. Actual angular velocity and angular acceleration for the fan-shaped trajectory.

<!-- Media -->

### 4.2. Experimental results

To validate the effectiveness of the proposed robot pose trajectory planning approach in practical situations, two trajectory tracking experiments are carried out on a robot platform, as shown in Fig. 13. To be specific, the platform is composed of a host computer with MATLAB R2017a/Simulink software, a real-time control board developed by shenyang institute of automation (SIA), Chinese academy of sciences (CAS), and a six-DOF industrial robot produced by ROKAE company. The communication between the control board and the host computer, the control board and the robot driver box is realized through Ethernet. The six joints of the robot are driven by the motors provided by Servo-tronix Motion Control Ltd., and are controlled by PI-type controllers whose gains are adjusted and fixed in advance. In the experiments, the actual joint positions are acquired by the encoders at the back of the motors with a sampling period of \( 1\mathrm{\;{ms}} \) ,and the actual joint velocities are obtained by the time derivative of position increments and suitable moving average low-pass filtering. The actual pose of the robot end-effector is calculated by the actual joint positions and the forward kinematics, and the actual velocity, acceleration and jerk of the robot end-effector are also obtained through the time derivative of pose increments and low-pass filtering. Note that the kinematics modeling of the robot can be achieved through the exponential rotational matrices [51] or DH transformation [52]. Here, the DH transformation is used, and then the forward kinematics of the ROKAE robot can be modeled as \( {\mathbf{T}}_{b}^{e} = \) \( {\mathbf{A}}_{b}^{1}{\mathbf{A}}_{1}^{2}{\mathbf{A}}_{2}^{3}{\mathbf{A}}_{3}^{4}{\mathbf{A}}_{4}^{5}{\mathbf{A}}_{5}^{e} \) ,where \( {\mathbf{A}}_{i - 1}^{i}\left( {i = \mathrm{b},1,2,\ldots ,5,\mathrm{e}}\right) \) is the homogeneous transformation matrix between coordinate systems i-1 and i as

<!-- Meanless: 14-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

\[{\mathbf{A}}_{i - 1}^{i} = \left\lbrack  \begin{matrix} \cos {q}_{i} &  - \cos {\alpha }_{i}\sin {q}_{i} & \sin {q}_{i}\sin {\alpha }_{i} & {a}_{i}\cos {q}_{i} \\  \sin {q}_{i} & \cos {q}_{i}\cos {\alpha }_{i} &  - \cos {q}_{i}\sin {\alpha }_{i} & {a}_{i}\sin {q}_{i} \\  0 & \sin {\alpha }_{i} & \cos {\alpha }_{i} & {d}_{i} \\  0 & 0 & 0 & 1 \end{matrix}\right\rbrack  \]

<!-- Media -->

<!-- figureText: (a) (b) 1.5 Actual joint velocity [rad/s] Joint 1 Joint 2 Joint 3 Joint 4 Joint 5 - Joint 6 1 0.5 -0.5 1 3 Time [sec] (d) 200 Joint 1 Joint 2 Joint 3 Actual joint jerk \( \left\lbrack  {\mathrm{{rad}}/{\mathrm{s}}^{3}}\right\rbrack \) Joint 4 Joint 5 Joint 6 100 0 -100 -200 0 2 3 4 Time [sec] Joint 1 Joint 2 - Joint 3 Actual joint position [rad] 0.5 Joint 4 Joint 5 Joint 6 0 -0.5 - 1 -1.5 -2 1 2 3 Time [sec] (c) Actual joint acceleration \( \left\lbrack  {\mathrm{{rad}}/{\mathrm{s}}^{2}}\right\rbrack \) Joint 1 Joint 2 Joint 3 Joint 4 Joint 5 - Joint 6 5 -5 1 2 3 Time [sec] -->

<img src="https://cdn.noedgeai.com/bo_d2slakbef24c73b2kfm0_14.jpg?x=238&y=147&w=1271&h=1014&r=0"/>

Fig. 19. Actual joint position, velocity, acceleration and jerk for the fan-shaped trajectory.

<!-- Media -->

where \( {\alpha }_{i},{a}_{i},{d}_{i},{q}_{i} \) are the DH parameters as shown in Table 2.

#### 4.2.1. Example 1

The blade-shaped trajectory is adopted for the first tracking experiment. By searching the data manual of the ROKAE robot, the joint position, velocity and acceleration limits of the robot can be obtained as shown in Table 3. Note that the joint jerk limits are not mentioned in the data manual, so they are not given here. By using the platform in Fig. 13 to perform the robot tracking experiment, the results are shown in Fig. 14, Fig. 15 and Fig. 16.

As can be seen from Fig. 14 and Fig. 15 that in the process of robot tracking, the actual linear velocity, linear acceleration and linear jerk of the end-effector almost all meet the given limits, although some areas exceed the limits owing to the noise introduced by the difference (particularly evident in the actual linear jerk). In addition, the actual angular velocity of the robot end-effector also meets the given limit. Obviously, this result shows that the proposed planning approach can guarantee the limited linear jerk and the continuous bounded angular velocity, which verifies the effectiveness of the proposed approach in the practical situation. Meanwhile, it can be seen from Fig. 16 that the actual position, velocity and acceleration of each joint of the robot meet the limits of the robot body in Table 3, and the actual joint jerk is relatively small. Therefore, the robot moves smoothly without impact during the whole blade-shaped trajectory tracking process.

#### 4.2.2. Example 2

The second tracking experiment uses the fan-shaped trajectory, and the tracking results are shown in Fig. 17, Fig. 18 and Fig. 19.

It can be found from Fig. 17 and Fig. 18 that, similar to the results of the blade-shaped trajectory, the actual linear velocity, linear acceleration, linear jerk and angular velocity of the robot end-effector also almost all meet the given limits for the fan-shaped trajectory, although the noise introduced by the difference causes the some areas to exceed the limits. Furthermore, in Fig. 19, the actual position, velocity and acceleration of each joint of the robot also meet the constraints of the robot body, and the jerk is small. Thus, the proposed planning approach can ensure that the trajectory has limited linear jerk and continuous bounded angular velocity, and the stability of the robot motion can also be guaranteed, which further demonstrate that the proposed approach is effective in practical situations. It should be noted that during this trajectory tracking experiment, because the robot has a small velocity at the beginning of the motion, and it needs to overcome the influence of friction, there exist large fluctuations in the actual joint acceleration and jerk at the beginning.

<!-- Meanless: 15-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

## Conclusions and future work

In this article, a novel Cartesian trajectory planning approach for the industrial robots is proposed. Specifically, it can be divided into the following three parts: 1) based on the triple NURBS curves, a synchronous description technique of the robot position and orientation is developed, which has not been reported in the existing literature; 2) by considering the geometric and kinematic constraints, especially the angular velocity constraint, and using the bidirectional interpolation strategy, a robot pose trajectory planning approach with limited linear jerk, continuous bounded angular velocity and approximate time-optimal property is presented, and it does not require the optimization or iterative process; 3) the blade-shaped and fan-shaped curves are used for the simulations, and it can be found that the two curves show the same results, i.e. their linear velocity, linear acceleration, linear jerk and angular velocity are all within the given limits, and the computational efficiency is high, which shows that the proposed planning approach is independent of the curve path type. In addition, based on these two curves, two trajectory tracking experiments are conducted on the ROKAE robot, and it can be seen that the actual linear velocity, linear acceleration, linear jerk and angular velocity of the robot end-effector almost all meet the given limits, and the joint constraints of the robot can also be guaranteed, which confirms the effectiveness of the proposed approach in the practical situations.

In the future, in order to obtain better Cartesian trajectory planning performance, we will further consider the dynamics and control performance constraints on the basis of geometric and kinematic constraints, and explore a new planning method to ensure that both linear jerk and angular jerk are limited.

## CRediT authorship contribution statement

Xiangfei Li: Methodology, Investigation, Writing - original draft. Huan Zhao: Conceptualization, Writing - review & editing. Xianming He: Data curation, Software, Visualization, Validation. Han Ding: Supervision.

## Declaration of Competing Interest

The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper.

## Data availability

Data will be made available on request.

## Acknowledgments

This work was supported by the National Natural Science Foundation of China under Grant Nos. 52188102, 52205521 and 52090054, and the National Key Research and Development Program of China under Grant No. 2022YFB4700304. References

[1] H. Zhao, X. Li, K. Ge, et al., A contour error definition, estimation approach and control structure for six-dimensional robotic machining tasks, Robot. Comput. Integr. Manuf 73 (2022), 102235.

[2] J. Song, Q. Chen, Z Li, A peg-in-hole robot assembly system based on Gauss mixture model, Robot. Comput. Integr. Manuf 67 (2021), 101996.

[3] Z. Hu, L. Hua, X. Qin, et al., Region-based path planning method with all horizontal welding position for robotic curved layer wire and arc additive manufacturing, Robot. Comput. Integr. Manuf 74 (2022), 102286.

[4] A. Gasparetto, P. Boscariol, A. Lanzutti, et al., Path planning and trajectory planning algorithms: a general overview, Motion Oper. Plann. Robot. Syst. (2015) 3-27.

[5] Z. Li, G. Li, Y. Sun, et al., Development of articulated robot trajectory planning, Int. J. Comput. Sci. Math. 8 (1) (2017) 52-60.

[6] Y. Sun, J. Jia, J. Xu, et al., Path, feedrate and trajectory planning for free-from surface machining: a state-of-the-art review, Chin. J. Aeronaut. (2022), https://doi.org/10.1016/j.cja.2021.06.011.

[7] P. Huang, Y. Xu, B. Liang, Tracking trajectory planning of space manipulator for capturing operation, Int. J. Adv. Rob. Syst. 3 (3) (2006) 211-218.

[8] H. Liu, X. Lai, W Wu, Time-optimal and jerk-continuous trajectory planning for robot manipulators with kinematic constraints, Robot. Comput. Integr. Manuf 29 (2) (2013) 309-317.

[9] A. Piazzi, A. Visioli, Global minimum-jerk trajectory planning of robot manipulators, IEEE. Trans. Ind. Electron. 47 (1) (2000) 140-149.

[10] R. Saravanan, S. Ramabalan, C. Balamurugan, Evolutionary optimal trajectory planning for industrial robot with payload constraints, Int. J. Adv. Manuf. Technol 38 (11) (2008) 1213-1226.

[11] J. Pan, L. Zhang, D. Manocha, Collision-free and smooth trajectory computation in cluttered environments, Int. J. Rob. Res 31 (10) (2012) 1155-1175.

[12] S. Kucuk, Maximal dexterous trajectory generation and cubic spline optimization for fully planar parallel manipulators, Commun. Chin. Sci. Abstr. 56 (2016) 634-647.

[13] Ü. Dinçer, M. Cevik, Improved trajectory planning of an industrial parallel mechanism by a composite polynomial consisting of Bézier curves and cubic polynomials, Mech. Mach. Theory 132 (2019) 248-263.

[14] F. Xie, L. Chen, Z. Li, et al., Path smoothing and feed rate planning for robotic curved layer additive manufacturing, Robot. Comput. Integr. Manuf 65 (2020), 101967.

[15] A. Gasparetto, V. Zanotto, A new method for smooth trajectory planning of robot manipulators, Mech. Mach. Theory 42 (4) (2007) 455-471.

[16] J. Huang, P. Hu, K. Wu, et al., Optimal time-jerk trajectory planning for industrial robots, Mech. Mach. Theory 121 (2018) 530-544.

[17] Y. Li, T. Huang, D.G Chetwynd, An approach for smooth trajectory planning of high-speed pick-and-place parallel robots using quintic B-splines, Mech. Mach. Theory 126 (2018) 479-490.

[18] J. Ma, S. Gao, H. Yan, et al., A new approach to time-optimal trajectory planning with torque and jerk limits for robot, Rob. Auton. Syst 140 (2021), 103744.

[19] S. Kucuk, Optimal trajectory generation algorithm for serial and parallel manipulators, Robot. Comput. Integr. Manuf 48 (2017) 219-232.

[20] S. Liu, Y. Wang, X.V. Wang, et al., Energy-efficient trajectory planning for an industrial robot using a multi-objective optimisation approach, Procedia. Manuf. 25 (2018) 517-525.

[21] G. Singh, V.K Banga, Combinations of novel hybrid optimization algorithms-based trajectory planning analysis for an industrial robotic manipulators, J. Field. Rob. (2022), https://doi.org/10.1002/rob.22069.

[22] C. Dai, S. Lefebvre, K.M. Yu, et al., Planning jerk-optimized trajectory with discrete time constraints for redundant robots, IEEE. Trans. Autom. Sci. Eng. 17 (4) (2020) 1711-1724.

[23] H.C. Fang, S.K. Ong, A.Y.C Nee, Interactive robot trajectory planning and simulation using augmented reality, Robot. Comput. Integr. Manuf 28 (2) (2012) 227-237.

[24] A. Olabi, R. Béarée, O. Gibaru, et al., Feedrate planning for machining with industrial six-axis robots, Control Eng. Pract 18 (5) (2010) 471-482.

[25] F. Valero, V. Mata, A. Besa, Trajectory planning in workspaces with obstacles taking into account the dynamic robot behaviour, Mech. Mach. Theory 41 (5) (2006) 525-536.

[26] M. Da Graça Marcos, J.A.T. Machado, Azevedo-Perdicoúlis T P. Trajectory planning of redundant manipulators using genetic algorithms, Commun. Nonlinear. Sci. Numer. Simul. 14 (7) (2009) 2858-2869.

[27] J. Gregory, A. Olivares, E. Staffetti, Energy-optimal trajectory planning for robot manipulators with holonomic constraints, Syst. Control. Lett 61 (2) (2012) 279-291.

[28] K. Zheng, Y. Hu, B. Wu, Trajectory planning of multi-degree-of-freedom robot with coupling effect, J. Mech. Sci. Technol. 33 (1) (2019) 413-421.

[29] S. Macfarlane, E.A. Croft, Jerk-bounded manipulator trajectory planning: design for real-time applications, IEEE. Trans. Rob. Autom 19 (1) (2003) 42-52.

[30] C. Rossi, S. Savino, Robot trajectory planning by assigning positions and tangential velocities, Robot. Comput. Integr. Manuf 29 (1) (2013) 139-156.

[31] G. Trigatti, P. Boscariol, L. Scalera, et al., A new path-constrained trajectory planning strategy for spray painting robots-rev.1, Int. J. Adv. Manuf. Technol 98 (9) (2018) 2287-2296.

[32] L. Lu, J. Zhang, J.Y.H. Fuh, et al., Time-optimal tool motion planning with tool-tip kinematic constraints for robotic machining of sculptured surfaces, Robot. Comput. Integr. Manuf 65 (2020), 101969.

[33] T. Zhang, M. Zhang, Y. Zou, Time-optimal and smooth trajectory planning for robot manipulators, Int. J. Control. Autom. Syst 19 (1) (2021) 521-531.

[34] M. ang, J. Xiao, S. Liu, et al., A third-order constrained approximate time-optimal feedrate planning algorithm, IEEE. Trans. Rob. (2021), https://doi.org/10.1109/ TRO. 2021.3132799.

[35] Z. Li, F. Peng, R. Yan, et al., A virtual repulsive potential field algorithm of posture trajectory planning for precision improvement in robotic multi-axis milling, Robot. Comput. Integr. Manuf 74 (2022), 102288.

[36] X.F Zha, Optimal pose trajectory planning for robot manipulators, Mech. Mach. Theory 37 (10) (2002) 1063-1086.

[37] Y. Sun, Z. Shi, J. Xu, Synchronous feedrate scheduling for the dual-robot machining of complex surface parts with varying wall thickness, Int. J. Adv. Manuf. Technol 119 (3) (2022) 2653-2667.

<!-- Meanless: 16-->




<!-- Meanless: X. Li et al. Robotics and Computer-Integrated Manufacturing 83 (2023) 102576-->

[38] C.G.L. Bianco, F. Ghilardelli, Real-time planner in the operational space for the automatic handling of kinematic constraints, IEEE. Trans. Autom. Sci. Eng. 11 (3) (2014) 730-739.

[39] D. Chen, S. Li, J.F. Wang, et al., A multi-objective trajectory planning method based on the improved immune clonal selection algorithm, Robot. Comput. Integr. Manuf 59 (2019) 431-442.

[40] R. Jin, P. Rocco, Y Geng, Cartesian trajectory planning of space robots using a multi-objective optimization, Aerosp. Sci. Technol. 108 (2021), 106360.

[41] M. Amersdorfer, T. Meurer, Equidistant tool path and cartesian trajectory planning for robotic machining of curved freeform surfaces, IEEE. Trans. Autom. Sci. Eng. (2021), https://doi.org/10.1109/TASE.2021.3117691.

[42] G. Wang, W. Li, C. Jiang, et al., Trajectory planning and optimization for robotic machining based on measured point cloud, IEEE. Trans. Rob. 38 (3) (2022) 1621-1637.

[43] Z. Qiao, Z. Liu, M. Hu, et al., A space cutter compensation method for multi-axis machining using triple NURBS trajectory, Int. J. Adv. Manuf. Technol 103 (9) (2019) 3969-3978.

[44] L. Chen, Z. Wei, L Ma, Five-axis Tri-NURBS spline interpolation method considering compensation and correction of the nonlinear error of cutter contacting paths, Int. J. Adv. Manuf. Technol 119 (3) (2022) 2043-2057.

[45] L. Piegl, W. Tiller, The NURBS Book, Springer Science & Business Media, 1996.

[46] J.S Dai, Finite displacement screw operators with embedded Chasles' motion, J. Mech. Robot 4 (4) (2012), 041002.

[47] R. Campa, H. De La Torre, Pose control of robot manipulators using different orientation representations: a comparative review, IEEE. Am. Control Confer. (2009) 2855-2860.

[48] H. Ni, C. Zhang, S. Ji, et al., A bidirectional adaptive feedrate scheduling method of NURBS interpolation based on S-shaped ACC/DEC algorithm, IEEE. Access 6 (2018) 63794-63812.

[49] F. Luo, Y. You, J. Yin, Research on the algorithm of NURBS curve bidirectional optimization interpolation with S-type acceleration and deceleration control, Chinese. J. Mechan. Engin. 48 (5) (2012) 147-156.

[50] M.C. Tsai, C.W. Cheng, A real-time predictor-corrector interpolator for CNC machining, J. Manuf. Sci. Eng. 125 (3) (2003) 449-460.

[51] C. Ayiz, S. Kucuk, The kinematics of industrial robot manipulators based on the exponential rotational matrices, ISIE. '95,. Proc. IEEE. Int. Symp. Ind. Electron. (2009) 977-982.

[52] S. Kucuk, B.D Gungor, Inverse kinematics solution of a new hybrid robot manipulator proposed for medical purposes, Med. Technol. National. Congress. (TIPTEKNO) (2016) 1-4.

<!-- Meanless: 17-->

