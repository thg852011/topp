

<!-- Meanless: Robotics and Computer-Integrated Manufacturing 75 (2022) 102308 Contents lists available at ScienceDirect Robotics and Computer-Integrated Manufacturing ELSEVIER journal homepage: www.elsevier.com/locate/rcim updates-->

# Global time-optimal B-spline feedrate scheduling for a two-turret multi-axis NC machine tool based on optimization with genetic algorithm

Fusheng Liang \( {}^{a} \) ,Guangpeng \( {\mathrm{{Yan}}}^{a} \) ,Fengzhou Fang \( {}^{a,b, * } \)

\( {}^{a} \) Centre of Micro/Nano Manufacturing Technology (MNMT-Dublin),University College Dublin,Dublin 4,Ireland

\( {}^{b} \) State Key Laboratory of Precision Measuring Technology and Instruments,Laboratory of Micro/Nano Manufacturing Technology (MNMT),Tianjin University,Tianjin 300072, China

## ARTICLEINFO

Keywords:

Feedrate scheduling

Multi-axis machine tool

Numerical control

Genetic algorithm

Concurrent computing

## A B S T R A C T

Feedrate scheduling is of great significance in numerical control machining for the improvement of the surface finish and machining efficiency. For multi-axis machine tools with more than two rotary axes, the feedrate scheduling issue considering the geometry and drive constraints is highly complicated due to the nonlinear kinematic relationship between the cutter location and drive axes. In this paper, a general optimization-based time-optimal feedrate scheduling method for two-turret machine tools with three-rotary axes and one short linear axis is proposed. The closed-form solution for the inverse kinematics of the machine tool is developed to analytically determine the preferable motion of each drive axis for a desired motion of the cutter. By using the B-spline method to represent the feedrate profile, the optimization-based feedrate scheduling model is built, where the control points and knot vector are designated as the decision variables. Two alternative operations, namely the progressive knot insertion and genetic-algorithm-based optimization, are conducted to determine the knot vector and control points for a time-optimal feedrate, respectively. To improve the robustness and computational efficiency of the algorithm, a feedrate profile segmentation criteria is established for concurrent computing. Without any simplification of the constraints, the proposed method can be employed to produce a global time-optimal feedrate efficiently. Typical simulation cases and experiments are carried out to verify the effectiveness and time-optimality of the proposed method.

## 1. Introduction

Multi-axis numerical control (NC) machine tools have been widely used in industry for manufacturing of products with complex surfaces, such as molds, impellers and artificial prosthesis, due to their high motion flexibility and adaptability [1-4]. The NC machining is performed by inversely transforming the via-points of tool path specified by cutter position and orientation in workpiece coordinate system to the reference motion commands of actuators. Smooth motion control plays a crucial role in improving the surface finish, reducing the tool wear, and maximizing the economic benefits of processing, etc. [5]. High-order kinematic behavior of actuators, including the acceleration, jerk, or even jounce, are closely associated with the tracking performance, vibration, and mechanical resonance of machine tools, so as to affect the motion smoothness in high-speed NC machining [6]. Hence, the motion control of the NC machine tools should not only determine the kinematics of cutter, such as the velocity, acceleration, and jerk in the tangential direction of tool path, but also ensure that the corresponding movement of actuators will not violate their drive capabilities. However, due to the nonlinearity induced by the inverse kinematics of multi-axis machine tools, it is a great challenge to produce smooth feed motion directly on tool path while considering high-order drive constraints of actuators. On the other hand, to shorten the machining time, the fee-drate along the tool path should be increased to the utmost. In such high-speed machining, the vibration and chatter of machine tools may occur due to the unrestricted motion of actuators. Therefore, it is necessary to put much particular care in feedrate scheduling with consideration of both machining efficiency and drive constraints for multi-axis machine tools.

In order to minimize the machining time, the drive capability of the machine tools should be fully exploited to achieve the high feedrate as much as possible. To this end, the maximum admissible acceleration/ deceleration (ACC/DEC) of at least one actuator should saturate at any point along a specified tool path [7-9]. The realization of this control method needs to determine the switch points where one of the actively saturated constraints jumps from one extreme to another. The positions and number of these switch points are closely associated with the time-optimality of the derived feedrate. Algorithms to determine the optimal switch points have been investigated a few decades ago, such as phase-plane analysis [7], dynamic programming [8], and direct transcription methods \( \left\lbrack  {{10},{11}}\right\rbrack \) . More challenging works were also conducted to consider higher-order constraints, such as jerk and jounce [12,13]. Although such feedrate derived from the bouncing kinematics is near time-optimal, the jump ACC/DEC or other high-order constraints may excite severe vibration of machine tool, which is unfavorable for improving the machining precision or even likely to damage the structure of drive system.

---

<!-- Footnote -->

* Corresponding author at: Centre of Micro/Nano Manufacturing Technology (MNMT-Dublin), University College Dublin, Dublin 4, Ireland.

E-mail address: fengzhou.fang@ucd.ie (F. Fang).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.rcim.2021.102308 Received 8 July 2021; Received in revised form 9 November 2021; Accepted 21 December 2021 Available online 28 December 2021 0736-5845/(© 2021 Elsevier Ltd. All rights reserved.-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

The smoothness of feed motion in NC machining can be improved by employing the feedrate generated by model-based method, in which the type of velocity profile for feedrate scheduling is preset, while the parameters for the time-optimal results are determined according to the geometry of tool path, boundary conditions, and constraints, etc. One of the most commonly used model is the S-shape feedrate with trapezoid ACC/DEC profile, which has been developed and being intensively studied in recent decades to produce jerk-limited feedrate [6,14-16]. Such kind of feedrate profile which is expressed in parabolic form and typically includes seven phases can realize smooth transition of feed motion between any two points on tool path. The duration and ACC/DEC value for each phase are allocated and assigned based on the travel length and two end velocity of each path segment with consideration of all constraints. To simplify the calculation process, the S-shape feedrate profile can be classified into several predefined types in advance, which will be used for planning of feedrate in different kinds of path segments conveniently \( \left\lbrack  {{14},{17}}\right\rbrack \) . One disadvantage of the S-shape method is the discontinuous jerk profile which may excite very large vibration and snap in machining process. In order to resolve this issue, the higher order polynomials or even sinusoidal curves have been applied for the model-based feedrate scheduling method [18-22]. The time-optimality and feasibility of the model-based method are closely related to the positions and velocity of splitting points on tool path. In general, the splitting points are located at several feature positions where the tool path is either discontinuous or has critical extreme curvature, and then the feedrate scheduling is carried out in each path segment between any two splitting points. Besides, the feasible velocity at these splitting points can be settled by look-ahead method or bi-directional scanning technique to prevent the feedrate from exceeding the constraints. Considering the high computational efficiency of the model-based method, it is possible to conduct the feedrate scheduling on-line for real-time interpolation [23-25]. Although the model-based method is capable of realizing continuous and smooth feed motion, the preset feedrate model cannot flexibly adapt to the ACC/DEC changes in the processing of very complex tool path, which will limit the improvement of machining efficiency.

To maximize the efficiency in NC machining, several works have attempted to explore the time-optimal feedrate using an optimization-based method, in which the feedrate optimization model is established aiming at minimizing the total machining time while respecting the geometric and drive constraints [26-33]. In this method, the feedrate profile can be expressed as a polynomial or spline curve, and an algorithm is developed to solve the optimization problem for the time-optimal solution. The constraints in the feedrate optimization model, in most cases, are nonlinear especially when the high-order derivatives of displacement are involved and the employed machine tools consist of one or more rotary axes. As a consequence, it is a very challenging task for traditional methods to directly find the best solution from numerous feasible ones that exist in the search space. To tackle this issue, several studies have been carried out to transform such complicated nonlinear optimization problem into a linear programming (LP) issue [26-29], which can be solved conveniently and efficiently. Typically, Sun et al. [29] proposed a linearization method to decouple the nonlinear acceleration and jerk constraints, and thus the feedrate optimization problem can be solved simply by a piecewise linear programming scheme. However, the optimal solution derived from the LP model is too conservative to make full use of the drive capacity of the machine tools for the time-optimal machining, given that the transformation may tighten the feasible region defined by the constraints. To improve the performance of the optimization-based method, some other algorithms, such as sequential quadratic programming [30-32] and genetic algorithm (GA) [33], have been developed to deal with the intractable optimization issue with little or no simplification. Although these algorithms have potential to find the solution close to the global optima, the balancing between the stability and optimality is difficult to achieve considering the computational complexity of these algorithms.

Another approach focusing on minimizing the total machining time using the iterative method was put forward in recent years [34-40]. The basic idea of this approach is to iteratively modulate the feedrate profile from an initial smooth one that may conform to all constraints or merely consists of constant value to a desired time-optimal one. In most iteration-based methods, a B-spline curve is usually applied to construct a continuous and smooth feedrate, and then the feedrate scheduling process can be transformed into iterative adjustment of control points of the B-spline curve [35-40]. By approximately increasing the number of knots of the B-spline curve, the near time-optimal feedrate can be obtained [38]. In addition, it is also very convenient to incorporate the high-order constraints in the iterative feedrate scheduling process [39]. Comparing with other methods, such as the model-based and optimization-based methods, the newly developed iteration-based method can be employed to produce a more time-optimal and smooth feedrate robustly. However, the complexity of knot vector determination and a large number of iterative calculations pose great challenges to the application of this method in practice.

Although the feedrate scheduling has been intensively studied in the above mentioned methods, few of them are devoted to resolving the issue on how to generate the time-optimal feedrate for multi-axis machine tools with more than two rotary axes. In this paper, the kinematic analysis of a two-turret CNC machine tool with three rotary axes and one linear axis is taken as the starting point, and a general optimization-based feedrate scheduling method is proposed. This method first adopts a B-spline curve with strong shape adjustment flexibility to represent the feedrate profile. The two unknown factors, i.e., the control points and knot vector, of the B-spline feedrate profile, is then determined in two separated processes. On the one hand, with a preset knot vector, the control points that can minimize the total machining time while considering all constraints in feedrate scheduling are derived by solving a constructed optimization problem using genetic algorithm. On the other hand, the knot vector of the time-optimal feedrate is found according to a developed progressive knot insertion method. The computational efficiency of the proposed method can be greatly improved using a segmentation algorithm in which the B-spline feedrate profile is divided into a series of segments and the feedrate optimization can be performed concurrently. Different from the approaches in previous studies, the proposed optimization-based feedrate scheduling method will not be affected by the nonlinear kinematics of the machine tools. To verify the effectiveness of the proposed method, some simulations and experiments are conducted on the two-turret machine tool.

## 2. Kinematics of the two-turret machine tool

The structure of the two-turret machine tool with three rotary axes and one linear axis is presented in this section. To reveal the motion characteristics of this machine tool, the forward and inverse kinematic models are constructed. Moreover, the kinematic analysis of the machine tool is carried out to demonstrate the significance of feedrate scheduling on such two-turret machine tool.

<!-- Meanless: 2-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

### 2.1. Structure of the two-turret machine tool

The axis configuration of the two-turret machine tool employed in this study is shown in Fig. 1. Different from the traditional four-axis machine tools, this two-turret machine tool incorporates only one short linear axis and three rotary axes. As can be seen, two vertical rotary turrets,i.e.,the axis \( B \) and axis \( D \) ,are mounted on the machine base. The linear axis \( W \) is mounted on the rotary axis \( B \) ,and the directions of the two axes are orthogonal. The rotary axis \( C \) is mounted on the linear axis \( W \) ,and the direction of the rotary axis \( C \) is parallel to that of the linear axis \( W \) and orthogonal to that of the rotary axis \( B \) at one point. Several spindles can be mounted on the rotary axis \( D \) ,and the directions of all spindles are orthogonal to that of the vertical rotary axis \( D \) .

The two-turret machine tool is expected to improve the thermal stability, axis stiffness and damping. Using the two separated vertical axes, it is possible to realize the thermal isolation between the machine base and the processing zone. In addition, the primary motion is provided by the rotary axes instead of the linear axes in the machining process, which contributes to a more compact machine tool without reducing the working range. Therefore, for the same workpiece capacity, this tow-turret machine tool has smaller footprint comparing with the conventional machines.

### 2.2. Forward kinematic transformation

The forward kinematic model of the machine tool is constructed to determine the cutter location and orientation relative to the workpiece for the specified movements of the linear and rotary axes. The kinematic chain and the relevant dimensional parameters of the two-turret machine tool are shown in Fig. 2. The local coordinate systems (CSs) are built on the components of the machine tool, including the four tool axes \( \left( {{\mathrm{{CS}}}_{B},{\mathrm{{CS}}}_{C},{\mathrm{{CS}}}_{D},{\mathrm{{CS}}}_{W}}\right) \) ,workpiece \( \left( {\mathrm{{CS}}}_{P}\right) \) ,and cutter \( \left( {\mathrm{{CS}}}_{T}\right) \) . In addition,a ground coordinate system \( {\mathrm{{CS}}}_{G} \) is attached on the machine base. It is assumed that the initial orientations of all local CSs are aligned with \( {\mathrm{{CS}}}_{G} \) . Moreover,the \( z \) directions of the \( {\mathrm{{CS}}}_{W},{\mathrm{{CS}}}_{C} \) and \( {\mathrm{{CS}}}_{P} \) are collinear,and located in the same height \( h \) with that of \( {\mathrm{{CS}}}_{T} \) . Two kinematic chains,i.e., the kinematic chain 1 and kinematic chain 2, are formed starting from the \( {\mathrm{{CS}}}_{G} \) to the \( {\mathrm{{CS}}}_{P} \) and \( {\mathrm{{CS}}}_{T} \) separately. The transformation matrixes of the two kinematic chains can be calculated as follows.

\[\left\{  \begin{array}{l} {M}_{GP} = {g}_{GB}R\left( B\right) {g}_{BW}T\left( W\right) {g}_{WC}R\left( C\right) {g}_{CP} \\  {M}_{GT} = {g}_{GD}R\left( D\right) {g}_{DT} \end{array}\right.  \tag{1}\]

where \( {M}_{GP} \) and \( {M}_{GT} \) denote the transformation matrixes from \( {\mathrm{{CS}}}_{P} \) and \( {\mathrm{{CS}}}_{T} \) to \( {\mathrm{{CS}}}_{G} \) . \( g \) represent the rigid transformation matrixes of the CSs when they are in their initial status, and the denotation of the subscripts is the same with that of \( M.R \) and \( T \) are used to represent the rotational and translational matrixes in the direction of each tool axis. Thus, the transformation matrix \( {M}_{PT} \) of the whole kinematic chain 3 from the \( {\mathrm{{CS}}}_{P} \) to \( {\mathrm{{CS}}}_{T} \) can be given by,

\[\left\{  \begin{array}{l} {M}_{PT} = {M}_{PG}{M}_{GT} = {M}_{GP}{}^{-1}{M}_{GT} \\  {M}_{GP}{}^{-1} = {g}_{CP}{}^{-1}R{\left( C\right) }^{-1}{g}_{WC}{}^{-1}T{\left( W\right) }^{-1}{g}_{BW}{}^{-1}R{\left( B\right) }^{-1}{g}_{GB}{}^{-1} \end{array}\right.  \tag{2}\]

<!-- Media -->

<!-- figureText: Rotary axis \( B \) Linear axis \( W \) Rotary axis \( C \) Rotary axis \( D \) Workpiece Cutter Machine base -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_2.jpg?x=109&y=1770&w=742&h=371&r=0"/>

Fig. 1. Structure of the two-turret machine tool.

<!-- figureText: \( h \) \( {\Delta x} \) Kinematic chain 2 Kinematic chain 1 Kinematic chain 3 -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_2.jpg?x=900&y=150&w=753&h=683&r=0"/>

Fig. 2. Kinematic chain of the two-turret machine tool.

<!-- Media -->

As shown in Fig. 2,the CL point and orientation of the cutter in \( {\mathrm{{CS}}}_{T} \) are \( \left\lbrack  \begin{array}{lll} 0 & 0 & 0 \end{array}\right\rbrack \) and \( \left\lbrack  \begin{array}{lll} 0 & 0 & 1 \end{array}\right\rbrack \) respectively. Using the transformation matrix \( {M}_{PT} \) ,the position of cutter relative to the workpiece can be calculated by,

\[\left\lbrack  \begin{array}{ll} {P}_{x} & {O}_{i} \\  {P}_{y} & {O}_{j} \\  {P}_{z} & {O}_{k} \end{array}\right\rbrack   = {M}_{PT}\left\lbrack  \begin{array}{ll} 0 & 0 \\  0 & 0 \\  0 & 1 \end{array}\right\rbrack   \tag{3}\]

where \( \mathbf{P} = \left\lbrack  {{P}_{x}{P}_{y}{P}_{z}}\right\rbrack \) and \( \mathbf{O} = \left\lbrack  {{O}_{i}{O}_{j}{O}_{k}}\right\rbrack \) are the CL point and orientation of cutter in \( {\mathrm{{CS}}}_{P} \) . After solving the Eq. (3),the position of cutter in \( {\mathrm{{CS}}}_{P} \) with respect to the movements \( \mathbf{\Theta } = \left\lbrack  \begin{array}{llll} B & C & D & W \end{array}\right\rbrack \) of the four drive axes can be derived by the following closed-form solution.

\[\left\{  \begin{array}{l} {P}_{x} = \cos C\left( {{\Psi \Delta x} + {\Gamma \Delta z} - L\sin B}\right) \\  {P}_{y} =  - \sin C\left( {{\Psi \Delta x} + {\Gamma \Delta z} - L\sin B}\right) \\  {P}_{z} = {\Gamma \Delta x} - {\Psi \Delta z} + L\cos B - W - {l}_{1} - {l}_{2} \\  {O}_{i} =  - \Gamma \cos C \\  {O}_{j} = \Gamma \sin C \\  {O}_{k} = \Psi  \end{array}\right.  \tag{4}\]

where \( {\Delta x} \) and \( {\Delta z} \) are the CL offset relative to \( {\mathrm{{CS}}}_{D} \) in the direction of \( x \) and \( z \) respectively. \( L \) is the distance between axis \( B \) and axis \( D.{l}_{1} \) and \( {l}_{2} \) are the offset of \( {\mathrm{{CS}}}_{C} \) relative to \( {\mathrm{{CS}}}_{W} \) and the offset of \( {\mathrm{{CS}}}_{P} \) relative to \( {\mathrm{{CS}}}_{C} \) in the direction of \( z.\Psi \) and \( \Gamma \) are given by,

\[\left\{  \begin{array}{l} \Psi  = \cos \left( {B - D}\right) \\  \Gamma  = \sin \left( {B - D}\right)  \end{array}\right.  \tag{5}\]

It can be deduced from Eq. (4) that the cutter location and orientation in \( {\mathrm{{CS}}}_{P} \) are correlative,and the relationship can be expressed by,

\[\frac{{P}_{y}}{{P}_{x}} = \frac{{O}_{j}}{{O}_{i}} =  - \tan C \tag{6}\]

According to this correlativity,for any specified CL point \( \mathbf{P} \) ,the tool axis \( \mathbf{O} \) must be restricted in the radial plane that is constructed by the CL point and \( z \) axis of \( {\mathrm{{CS}}}_{P} \) ,as shown in Fig. 3. This restriction limits the degree of freedom of the tool orientation at the CL point, which reduces the ceiling of machine tool processing capabilities.

<!-- Meanless: 3-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

<!-- Media -->

<!-- figureText: \( {\mathrm{{CS}}}_{P} \) Tool axis located radial plane \( O\left( {{O}_{i}{O}_{j}{O}_{k}}\right) \) y \( P\left( {{P}_{x}{P}_{y}{P}_{z}}\right) \) ✘ -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_3.jpg?x=253&y=151&w=450&h=426&r=0"/>

Fig. 3. Relationship between cutter location and orientation in \( {\mathrm{{CS}}}_{P} \) .

<!-- Media -->

### 2.3. Inverse kinematic transformation

The inverse kinematic transformation of the machine tools should be developed to determine the necessary movements of drive axes to achieve the specified cutter position including the CL point and tool orientation in \( {\mathrm{{CS}}}_{P} \) . By solving the Eq. (4) combining with the Eq. (5),the inverse kinematic solution of the two-turret machine tool can be given in the closed-form as follows.

\[\left\{  {\begin{array}{l} C = {k\pi } - \arctan \left( \frac{{P}_{y}}{{P}_{x}}\right) \\  B = \arcsin \left( \frac{{\Delta x}\cos \theta  + {\Delta z}\sin \theta  + {\left( -1\right) }^{k + 1}\sqrt{{P}_{x}^{2} + {P}_{y}^{2}}}{L}\right) \\  D = \theta  + B \\  W =  - {\Delta x}\sin \theta  - {\Delta z}\cos \theta  + L\cos B - {l}_{b} - {l}_{a} - {l}_{c} - {P}_{x} \end{array}\;\left( {k \in  \mathbf{Z}}\right) }\right.  \tag{7}\]

where

\[\theta  = {\left( -1\right) }^{k + 1}\left( {\arccos \left( \frac{{P}_{x}{O}_{i} + {P}_{y}{O}_{j}}{\sqrt{{P}_{x}{}^{2} + {P}_{y}{}^{2}}\begin{Vmatrix}O\end{Vmatrix}}\right) }\right)  \tag{8}\]

where the parameter \( k \) should be an integer.

It can be inferred from the Eq. (7) that for a specified cutter position \( \left\lbrack  {\mathbf{P}\mathbf{O}}\right\rbrack \) ,there are two feasible solutions which correspond to two different configurations of machine tool separately when \( k \) is odd or even. In order to determine the solution to be selected, the movement of each drive axis from the current cutter position to the next one should be calculated. The solution that minimizes the movements of drive axes is generally a favorable choice to realize the smooth and continuous motion of machine tool.

The two-turret machine tool can only perform the machining along the tool path that meets the restriction in Eq. (6). However, the tool path that is generated by CAM cannot conform to this regulation consistently. Therefore, when the tool orientation is not located in the radial plane, a projection operation will be conducted to ensure that the cutter position can be reached by the machine tool. As shown in Fig. 4, the tool orientation \( \mathbf{O} \) outside the radial plane will be projected into this plane along its normal direction using the following equation.

\[{O}^{ * } = O - \frac{\left( O \cdot  n\right) }{\parallel n\parallel } \cdot  n \tag{9}\]

where \( {\mathbf{O}}^{ * } \) is the projected tool orientation in the radial plane,and \( \mathbf{n} \) is the normal vector of the radial plane.

### 2.4. Kinematic analysis of the two-turret machine tool

The unique structure of the two-turret machine tool causes the motions of drive axes in the machine tool space and the feed motion of cutter in the workpiece space to be highly nonlinear, as shown in the Eqs. (4) and (7). In addition, the motions of different drive axes are coupled with each other, which makes it difficult to determine the kinematic behavior of the machine tool under the condition of a specified tool path and scheduled feedrate. Even in a special case where the cutter moves at a constant speed along a linear tool path, the feed speed of each drive axis may change drastically, especially near the singularity.

<!-- Media -->

<!-- figureText: \( {\mathrm{{CS}}}_{F} \) Radial plane y Tool orientation outside Projected tool orientation the radial plane in the radial plane -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_3.jpg?x=1074&y=150&w=392&h=431&r=0"/>

Fig. 4. Projection of tool orientation in the radial plane.

<!-- Media -->

To achieve a smooth machining process, the feed motions of all drive axes need to be monitored and strictly restricted to prevent the drive capabilities of actuators from exceeding their limits. Unlike the feedrate scheduling on three-axis machine tools and traditional five-axis machine tools with three linear axes and two rotary axes, it is a challenging work to produce such smooth feedrate in the workpiece space considering the nonlinear kinematics of the two-turret machine tool. Currently, only a very conservative feedrate along the tool path is adopted to avoid unpredictable vibrations of the machine tool caused by excessively high motion speed changes of drive axes during the machining process. However, such conservative feed motion will inevitably increase the total machining time, which is very detrimental to the improvement of machining efficiency. Therefore, it is necessary to carry out the research on the feedrate scheduling on the two-turret machine tool to minimize the total machining time while meeting the machining smoothness requirements.

## 3. Time-optimal feedrate scheduling method

The motion of each drive axis of the two-turret machine tool along a specified tool path can be determined using the developed inverse kinematic transformation model. To ensure that the cutter can follow the tool path continuously and smoothly, an interpolation algorithm should be built in advance to produce successive reference positions on tool path. For a parametric tool path \( r\left( u\right) \) ,the parameter value of the reference positions can be derived using the following 2nd-order Taylor's expansion interpolator.

\[{\Delta u} = {u}_{i + 1} - {u}_{i} = {\left. \frac{du}{dt}T\right| }_{u = {u}_{i}} + {\left. \frac{1}{2}\frac{{d}^{2}u}{d{t}^{2}}{T}^{2}\right| }_{u = {u}_{i}} + \text{ H.O.T. } \tag{10}\]

where \( {u}_{i} \) and \( {u}_{i + 1} \) are the parameter values of the current and next cutter positions. \( T \) is the interpolation period and \( H.O.T \) . denotes the higher order terms that will be ignored. The derivatives of parameter \( u \) with respect to the time \( t \) can be expressed by,

\[\left\{  \begin{array}{l} \frac{du}{dt} = \frac{du}{ds}\frac{ds}{dt} = \frac{v}{{s}^{\prime }} \\  \frac{{d}^{2}u}{d{t}^{2}} = \frac{{v}^{\prime }{s}^{\prime } - v{s}^{\prime \prime }}{{{s}^{\prime }}^{2}}\frac{du}{dt} \end{array}\right.  \tag{11}\]

where \( s \) and \( v \) denote the arc length and velocity along the cutter location (CL) path respectively. The symbols \( {\Lambda }^{\prime } \) and \( {\Lambda }^{\prime \prime } \) mean the derivatives of \( \Lambda \) with respect to the tool path parameter \( u \) .

<!-- Meanless: 4-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

The interpolator needs to predetermine the velocity and its derivative along the CL path at the current parameter point to obtain the following cutter position. The velocity, also known as the feedrate, is highly related to the machining efficiency and accuracy in CNC machining. Hence, a global optimization-based time-optimal feedrate scheduling method is proposed to minimize the total machining time without compromise on the machining quality in this section.

### 3.1. B-spline feedrate profile definition

The B-spline curve is used in this study to represent the feedrate profile along CL path due to its local support property and flexible shape adjustment capability. A \( p \) th degree B-spline feedrate profile \( v\left( u\right) \) can be given by [41],

\[v\left( u\right)  = \mathop{\sum }\limits_{{i = 0}}^{n}{N}_{i,p}\left( u\right) {V}_{i} \tag{12}\]

where \( {V}_{i} \) are the control points and \( {N}_{i,p}\left( u\right) \) are the B-spline basis function defined on a knot vector \( \mathbf{U} \) ,

\[U = \left( {{\underbrace{{u}_{0} = \ldots , = {u}_{p}}}_{p + 1},\ldots ,{u}_{i},\ldots ,{\underbrace{{u}_{n + 1} = \ldots , = {u}_{n + p + 1}}}_{p + 1}}\right)  \tag{13}\]

where \( {u}_{i}\left( {i = 0 \sim  n + p + 1}\right) \) are a series of non-decreasing knots. The \( p + 1 \) multiplicity of the two end knots can ensure that the B-spline curve can pass through the two end control points,i.e., \( v\left( {u}_{0}\right)  = {V}_{0} \) and \( v\left( {u}_{n + p + 1}\right) \) \( = {V}_{n} \) . The defined parameter range of the B-spline feedrate profile is exactly the same as that of the parametric tool path. In this study, unless stated otherwise,the parameter range will be normalized to \( \left\lbrack  {0,1}\right\rbrack \) .

The local support property of the B-spline curve makes it possible to modify its shape in a small parameter interval. As shown in Fig. 5, the adjustment of control point \( {V}_{i} \) only affects the shape of curve in the parameter position of \( u \in  \left\lbrack  {{u}_{i},{u}_{i + p + 1}}\right\rbrack \) ,where \( \left\lbrack  {V}_{i}\right\rbrack \) is the adjusted new control point. The local shape adjustment of B-spline curve can be much finer when the knot vector becomes much denser.

### 3.2. Constraints in feedrate scheduling

The feedrate scheduling process should fully consider the machining requirements and feed drive capability of the machine tools to improve the quality of the machined surface. The constraints that will be involved in the feedrate scheduling of the two-turret machine tool are presented as follows.

<!-- Media -->

<!-- figureText: ✓ \( \left\lbrack  {V}_{i}\right\rbrack \) U \( {u}_{i + p + 1} \) Adjusted feedrate 0 \( {u}_{i} \) Original feedrate -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_4.jpg?x=188&y=1683&w=577&h=461&r=0"/>

Fig. 5. Local support property of the B-spline curve.

<!-- Media -->

#### 3.2.1. Chord error

The chord error is one of the main sources of error in the interpolation process. For the multi-axis machine tools with the rotational tool center point (RTCP) function, the CL point moves along the straight line between the successive interpolation points and the chord error can be calculated by the maximal deviation between the interpolated linear CL path and designed CL trajectory, as shown in Fig. 6. It can be seen that the chord error is related to the interpolation step length and curvature of tool path. The relationship between them can be approximately expressed as,

\[{\Delta s} = {vT} = 2\sqrt{\frac{2\delta }{\kappa } - {\delta }^{2}} \tag{14}\]

where \( {\Delta s} \) is the step length in one interpolation period; \( \delta \) is the chord error; \( \kappa \) is the curvature of tool path. For a specified tool path,to restrict the chord error within the definite maximal constraint \( {\delta }_{\max } \) ,the feedrate should meet the following constraint.

\[v\left( u\right)  \leq  \frac{2}{T}\sqrt{\frac{2{\delta }_{\max }}{\kappa } - {\delta }_{\max }{}^{2}} \tag{15}\]

#### 3.2.2. Tangential kinematic constraints

The feedrate and its derivatives, such as the acceleration and jerk, along the tool path have a great influence on the cutting force, tool wear, chip formation and surface roughness, etc. In order to achieve a high quality machining, it is necessary to take these tangential kinematic constraints into consideration in the scheduling process of feedrate. Therefore, the following constraints should be satisfied.

\[\left\{  \begin{array}{l} v\left( u\right)  \leq  {V}_{\max } \\  \left| {\dot{v}\left( u\right) }\right|  = \left| \frac{dv}{dt}\right|  = \left| {\frac{dv}{du}\frac{du}{dt}}\right|  = \left| \frac{v{v}^{\prime }}{{s}^{\prime }}\right|  \leq  {A}_{\max } \\  \left| {\ddot{v}\left( u\right) }\right|  = \left| \frac{{d}^{2}v}{d{t}^{2}}\right|  = \left| {\frac{dv}{du}\frac{du}{dt}}\right|  = \left| {\frac{v{v}^{\prime 2} + {v}^{2}{v}^{\prime \prime }}{{{s}^{\prime }}^{2}} - \frac{{s}^{\prime \prime }}{{{s}^{\prime }}^{3}}{v}^{2}{v}^{\prime }}\right|  \leq  {J}_{\max } \end{array}\right.  \tag{16}\]

where \( {V}_{\max },{A}_{\max } \) and \( {J}_{\max } \) are the maximal tangential feedrate,acceleration and jerk constraints. The symbols \( \dot{\Lambda } \) and \( \ddot{\Lambda } \) denote the derivatives of \( \Lambda \) with respect to the time \( t \) .

#### 3.2.3. Drive constraints of tool axes

The drive constraints considered in the feedrate scheduling process are primarily concerned with the restriction of velocity, acceleration and jerk of each drive axis. The kinematic behavior of drive axes is not only related to the geometry of tool path but also closely associated with the inverse kinematic transformation of machine tools and the position of workpiece in the machine coordinate system. For the two-turret machine tool, the nonlinear relationship between the motion of CL point in \( {\mathrm{{CS}}}_{P} \) and the feed motion of each drive axis makes the kinematic behavior of actuators very difficult to predict in the machining process. Although the feedrate along the CL path varies smoothly, the motion of drive axes may exceed their physical limitations drastically, which will result in severe mechanical vibration and large tracking error. To obtain a stable machining process, the kinematics of each drive axis should be restricted, and the constraints can be expressed as follows using the chain rule of derivative

<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

\[\left\{  \begin{array}{l} \left| {{\dot{q}}^{\tau }\left( u\right) }\right|  = \left| {\frac{d{p}^{\tau }}{ds}\frac{ds}{dt}}\right|  = \left| {{q}_{s}^{\tau }v}\right|  \leq  {V}_{\max }^{\tau } \\  \left| {{\ddot{q}}^{\tau }\left( u\right) }\right|  = \left| {\frac{d{\dot{q}}^{\tau }}{ds}\frac{ds}{dt}}\right|  = \left| {{q}_{ss}^{\tau }{v}^{2} + \frac{{q}_{s}^{\tau }}{{s}^{\prime }}v{v}^{\prime }}\right|  \leq  {A}_{\max }^{\tau } \\  \left| {{\ddot{q}}^{\tau }\left( u\right) }\right|  = \left| {\frac{d{\ddot{q}}^{\tau }}{ds}\frac{ds}{dt}}\right|  = \left| {{q}_{sss}^{\tau }{v}^{3} + \frac{3{q}_{ss}^{\tau }{v}^{\prime 2} - {q}_{s}^{\tau }{s}^{\prime 2}}{{s}^{\prime 3}}{v}^{2}{v}^{\prime }}\right|  \end{array}\right. \]

\[ + \frac{{q}_{s}^{\tau }}{{s}^{\prime 2}}\left( {{v}^{2}{v}^{\prime \prime } + v{v}^{\prime 2}}\right)  \mid   \leq  {J}_{\max }^{\tau } \tag{17}\]

<!-- Media -->

<!-- figureText: Interpolation Cutter CL point Chord error Actual CL path with RTCP function step length Designed CL tool path -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_4.jpg?x=1029&y=1722&w=478&h=424&r=0"/>

Fig. 6. Chord error in multi-axis machining with RTCP function.

<!-- Media -->

<!-- Meanless: 5-->


where \( {q}^{\tau }\left( u\right) \) denotes the motion of each drive axis,and the superscript \( \tau \) indicates different drive axis,i.e., \( B,C,D \) ,and \( W \) for the two-turret machine tool. \( {\dot{q}}^{\tau }\left( u\right) ,{\ddot{q}}^{\tau }\left( u\right) \) ,and ... \( {q}^{\ldots }\tau \left( u\right) \) denote the velocity,acceleration and jerk of each drive axis respectively, which are restricted by the maximal limitations of \( {V}_{\max }^{\tau },{A}_{\max }^{\tau } \) ,and \( {J}_{\max }^{\tau } \cdot  {q}_{s}^{\tau },{q}_{ss}^{\tau } \) ,and \( {q}_{sss}^{\tau } \) are the derivatives of each drive axis motion with respect to the arc length \( s \) of CL path. For a specified parametric tool path, \( {q}_{s}^{\tau },{q}_{ss}^{\tau } \) ,and \( {q}_{sss}^{\tau } \) can be determined by,

\[\left\{  \begin{array}{l} {q}_{s}^{\tau } = \frac{d{q}^{\tau }}{du} \cdot  \frac{du}{ds} = \frac{{q}_{u}^{\tau }}{{s}^{\prime }} \\  {q}_{ss}^{\tau } = \frac{d{q}_{s}^{\tau }}{du} \cdot  \frac{du}{ds} = \frac{{q}_{uu}^{\tau }}{{s}^{\prime 2}} + \frac{{q}_{u}^{\tau }}{{s}^{\prime \prime }} \\  {q}_{sss}^{\tau } = \frac{d{q}_{ss}^{\tau }}{du} \cdot  \frac{du}{ds} = \frac{{q}_{uu}^{\tau u}}{{s}^{\prime 3}} + 3\frac{{q}_{uu}^{\tau }}{{s}^{\prime \prime 4}} + \frac{{q}_{u}^{\tau }}{{s}^{\prime \prime 4}} \end{array}\right.  \tag{18}\]

where \( {q}_{u}^{\tau },{q}_{uu}^{\tau } \) ,and \( {q}_{uuu}^{\tau } \) are the derivatives of each drive axis motion with respect to the tool path parameter \( u.{s}^{\prime \prime \prime } \) is the derivative of arc length of CL path with respect to the parameter \( u \) .

### 3.3. Optimization-based feedrate scheduling model

The total interpolation cycle time \( {T}_{tal} \) ,i.e.,the machining time,along the parametric tool path can be calculated by,

\[{T}_{tal} = {\int }_{0}^{1}\frac{dt}{du}{du} = {\int }_{0}^{1}\frac{dt}{ds}\frac{ds}{du}{du} = {\int }_{0}^{1}\frac{{s}^{\prime }\left( u\right) }{v\left( u\right) }{du} \tag{19}\]

where \( v\left( u\right) \) is the feedrate along the CL path. In order to minimize the machining time and ensure the processing requirements, the feedrate should be kept as high as possible while respecting the necessary constraints. The desired time-optimal feedrate can be obtained by solving the following constrained optimization problem (COP).

\[\min {T}_{tal} = {\int }_{0}^{1}\frac{{s}^{\prime }\left( u\right) }{v\left( u\right) }{du} \tag{20}\]

subject to the constraints in Eqs. (15) - (17). Since the magnitude of constraints with different orders varies greatly, to prevent the higher-order constraints, such as jerk and acceleration, from overwhelming the lower-order constraints, such as feedrate and chord error, it is crucial to normalize the constraints into the same level. Therefore, the constraints considered in the feedrate optimization model can be expressed as,

\[\left\{  \begin{array}{l} \frac{v\left( u\right) }{T} - 1 = 0 \\  \frac{1}{T}\sqrt{\frac{2{M}_{\max }}{{M}_{\max }}} - 1 \leq  0 \\  \frac{v\left( u\right) }{{V}_{\max }} - 1 \leq  0 \\  \left| \frac{{w}^{\prime }}{{w}_{\max }}\right|  - 1 \leq  0 \\  \frac{{w}^{\prime }}{{w}^{\prime }} - \frac{1}{{w}^{\prime }} \leq  0 \\  \frac{{w}^{\prime }}{{w}^{\prime }} - \frac{1}{{w}^{\prime }} \leq  0 \\  \frac{{q}_{x}^{\prime }v}{{q}_{x}^{\prime }} - \frac{v{q}_{y}^{\prime }}{{q}_{y}^{\prime }} - \frac{1}{{w}^{\prime }} \leq  0 \\  \frac{{q}_{x}^{\prime }v}{{q}_{x}^{\prime }} - \frac{v{q}_{y}^{\prime }}{{q}_{y}^{\prime }} - \frac{1}{{w}^{\prime }} \leq  0 \\  \frac{{q}_{x}^{\prime }v}{{q}_{x}^{\prime }} - \frac{v{q}_{y}^{\prime }}{{q}_{y}^{\prime }} - \frac{1}{{w}^{\prime }} \leq  0 \\  \frac{{q}_{y}^{\prime }v}{{q}_{y}^{\prime }} - \frac{v{q}_{x}^{\prime }}{{q}_{x}^{\prime }} - \frac{1}{{w}^{\prime }} \leq  0 \end{array}\right.  \tag{21}\]

When the tool path has been specified, the coefficients of the objective function in Eq. (20) and the nonlinear constraints in Eq. (21),such as \( \kappa \) , \( {s}^{\prime } \) ,and \( {s}^{\prime \prime } \) are definite. Thus,the above optimization problem is only related to the feedrate \( v \) and its derivatives with respect to tool path parameter \( u \) ,i.e., \( {v}^{\prime } \) and \( {v}^{\prime \prime } \) along the CL path. The feedrate profile represented by the B-spline curve is completely determined by the knot vector and control points. However, if the knot vector and control points of the B-spline feedrate profile are taken as the optimization variables simultaneously, the optimization problem will become extremely difficult to solve. Therefore, in this study, the determination of knot vector and optimization of control points are conducted in two separate threads.

If the knot vector of the B-spline feedrate profile is predetermined, the control points will be the unique decision variables for the optimization. Since the high-order constraints in Eq. (21) are related to the 2nd-order derivative of feedrate, the constraints are particularly sensitive to the adjustment of control points. Thus, the feasible solutions are scattered irregularly across the searching space of the decision variables and difficult to find. In addition, there are numerous local optimal solutions, which make the traditional optimization solving method much more likely trap into local optima. Considering the above mentioned issues, GA will be employed to resolve the constrained feedrate optimization problem due to the powerful global search capabilities. In GA, a population of candidate solutions is initialized first and several operators, such as mutation, crossover and selection, are then conducted to find the global optimal solution in the entire feasible search space. The population in this algorithm has more opportunities to move away from the local optima by combining the mutation and crossover operators. As a consequence, it can be quite effective for GA to find the generally global optima for the feedrate optimization problem.

As a population-based optimization approach, the GA involves a large amount of repeated evaluation of objective function, especially for complex high-dimensional problem. If the objective function in Eq. (20) is evaluated analytically, the GA-based optimization process will be very time-consuming due to the complex integral computation. Therefore, the exact evaluation of the object function is forgone and a computationally efficient numerical integration method is used to perform an approximate evaluation. By sampling a limited number of positions on tool path, the objective function in Eq. (20) can be approximated as,

\[{T}_{\text{tal }} = \mathop{\sum }\limits_{{i = 0}}^{{N - 1}}\frac{{\Delta s}\left( {u}_{i}\right) }{v\left( {u}_{i}\right) } = \mathop{\sum }\limits_{{i = 0}}^{{N - 1}}\frac{s\left( {u}_{i + 1}\right)  - s\left( {u}_{i}\right) }{v\left( {u}_{i}\right) } \tag{22}\]

where \( {u}_{i}\left( {i = 0 \sim  N}\right) \) are the sampling parameter positions,and \( {\Delta s}\left( {u}_{i}\right) \) denote the arc length of tool path from \( s\left( {u}_{i}\right) \) to \( s\left( {u}_{i + 1}\right) \) ,which should be prepared before the optimization process launches.

<!-- Meanless: 6-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

The constraints violation will be detected and calculated at some parametric checking positions \( {u}_{i}\left( {i = 0 \sim  H}\right) \) on tool path. Although many more checking positions contribute to more reliable constraint restrictions, the excessive calculations will severely reduce the overall efficiency of the optimization-based feedrate scheduling process. Therefore, the number of checking positions should be determined by fully weighting the reliability of constraint checking and the amount of calculations.

### 3.4. Determination of knot vector and concurrent computing

In this section, a progressive knot insertion method is proposed to determine the necessary number of knots to produce the time-optimal B-spline feedrate profile. To improve the computational efficiency of the optimization-based feedrate scheduling process, the overall optimization will be factored into some small problems that can be solved concurrently. By combining the knot insertion method and concurrent computing strategy, the computationally efficient and robust feedrate scheduling method can be developed.

#### 3.4.1. Progressive knot vector determination

The number of knots of the B-spline feedrate profile directly affects the performance of the optimization process. On the one hand, the density of knots should be sufficient enough to adapt to the frequent ACC/DEC of the feed motion, which is also the prerequisite for producing the time-optimal feedrate profile. On the other hand, too dense knots may cause an extremely high-dimensional problem and the optimization processes are more likely to fall into local optima. Therefore, determining the appropriate number of knots is essential to improve the optimization efficiency and guarantee the time-optimality of feedrate.

In order to achieve a robust and efficient optimization process, the knot vector is initialized uniformly and sparsely. The feedrate optimization model is built with a preset sparse knot vector and solved by GA to find the optimal solution. In most cases, the optimized feedrate with the sparse knot vector is far away from the global time-optimal target. To further improve the time-optimality, the knot vector will be refined by inserting new knots in the middle of the original knot vector and a new B-spline feedrate profile is produced. Since the knot insertion has no effect on the shape of B-spline curve, the newly derived B-spline feedrate profile will naturally inherit the constraint satisfaction characteristics of original one. Afterwards, the GA-based feedrate optimization is conducted according to the refined knot vector. Different from the first optimization process in which the initial population is created in the searching space randomly, the new round optimization is carried out staring from the newly derived and constraints satisfied B-spline fee-drate profiles to perform a faster and more efficient searching process. Assuming that the machining time corresponding to the feedrate before knot insertion is \( {f}_{a} \) while the machining time derived from the optimized feedrate after knot insertion is \( {f}_{b} \) ,the machining time reduction caused by the knot insertion should be \( {f}_{a} - {f}_{b} \) . The knot insertion process is repeated and the GA-based feedrate optimization is resumed until such machining time reduction exceeds the predefined threshold \( \Delta {f}_{\max } \) . Otherwise, the knot insertion process terminates and the global time-optimal B-spline feedrate profile is obtained. The flowchart of the progressive knot vector determination process is shown in Fig. 7.

#### 3.4.2. Segmented optimization and concurrent computing

For a long and complex tool path, the total number of knots used for describing the time-optimal B-spline feedrate profile is very large, which leads to a dense and high-dimensional optimization problem. It is computationally expensive for GA to find a global optimal solution for such high-dimensional optimization problem with highly nonlinear constraints. In addition, more decision variables will increase the risk of the optimization process falling into local optima or even failing to find a feasible solution. Although numbers of recent researches have been designed for this high-dimensional optimization problem, few of them can be used to deal with such complex high-order nonlinear constraints.

<!-- Media -->

<!-- figureText: Start \( {f}_{a} = {f}_{b} \) Initialize the knot vector of B-spline feedrate profile Construct the GA-based feedrate optimization model Find the optimal solution. Evaluate the fitness value, i.e.,the machining time \( {f}_{a} \) of the optimized feedrate Refine the knot vector and produce the new B-spline feedrate profiles Resume the GA-based optimization from the new feedrate profiles with the refined knot vector Evaluate the machining time \( {f}_{b} \) of the improved feedrate \( {f}_{a} - {f}_{b} \leq  \Delta {f}_{\max } \) End -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_6.jpg?x=900&y=149&w=756&h=1020&r=0"/>

Fig. 7. Progressive knot vector determination process.

<!-- Media -->

In order to avoid solving such high-dimensional optimization problem, after determining the initial knot vector, the B-spline feedrate profile will be divided into several segments that are connected end to end at some knot positions. By appropriately selecting some control points as decision variables, the feedrate optimization in each segment is independent of each other. Therefore, the high-dimensional feedrate optimization problem will be break down into a series of small problems which can be carried out concurrently. Specifically, if the initial knot vector \( {\mathbf{U}}_{0} \) is determined as,

\[{U}_{0} = \left( {{\underbrace{{u}_{0} = \ldots , = {u}_{p}}}_{p + 1},\ldots ,{u}_{i},\ldots ,{\underbrace{{u}_{n + 1} = \ldots , = {u}_{m + p + 1}}}_{p + 1}}\right)  \tag{23}\]

and the splitting knots are placed at,

\[{U}_{s} = \left( {{u}_{s\left( 0\right) },\ldots ,{u}_{s\left( i\right) },\ldots ,{u}_{s\left( r\right) }}\right)  \tag{24}\]

<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

where \( s\left( 0\right)  = 0 \) and \( s\left( r\right)  = m + p + 1 \) ,the control points relevant to each feedrate segment will be assigned following the instruction in Table 1.

<!-- Media -->

Table 1

Control points assignment in each feedrate segment.

<table><tr><td>Feedrate segments \( {u}_{s\left( i\right) } \leq \) \( u \leq  {u}_{s\left( {i + 1}\right) } \)</td><td>Related control points Head</td><td>Decision variables</td><td>Tail</td></tr><tr><td rowspan="2">\( i = 0 \)</td><td>\( {V}_{s\left( 0\right) } = 0 \)</td><td>\( {V}_{s\left( 0\right)  + 1},\ldots , \)</td><td>\( {V}_{s\left( 1\right)  - p},\ldots ,{V}_{s\left( 1\right)  - 1} \)</td></tr><tr><td/><td>\( {V}_{s\left( 1\right)  - p - 1} \)</td><td>\( = 0 \)</td></tr><tr><td rowspan="2">\( 1 \leq  i \leq  r - 2 \)</td><td>\( {V}_{s\left( i\right)  - p},\ldots , \)</td><td>\( {V}_{s\left( i\right) },\ldots ,{V}_{s(i + } \)</td><td>\( {V}_{s\left( {i + 1}\right)  - p},\ldots ,{V}_{s(i} \)</td></tr><tr><td>\( {V}_{s\left( i\right)  - 1} = 0 \)</td><td>\( 1) \cdot  p \cdot  1 \)</td><td>\( + 1) - 1 = 0 \)</td></tr><tr><td rowspan="2">\( i = r - 1 \)</td><td>\( {V}_{s\left( {r - 1}\right)  - p},\ldots , \)</td><td>\( {V}_{s\left( {r - 1}\right) },\ldots , \)</td><td>\( {V}_{s\left( r\right)  - p - 1} = 0 \)</td></tr><tr><td>\( {V}_{s\left( {r - 1}\right)  - 1} = 0 \)</td><td>\( {V}_{s\left( r\right)  - p - 2} \)</td><td/></tr></table>

<!-- Media -->

<!-- Meanless: 7-->


The control points that influence the shape of each feedrate segment can be divided into three parts, i.e., the head control points, decision variables, and tail control points. By fixing the head and tail control points to 0 , the feed motion in each segment will start from standstill and stop at the end. In order to ensure that at least one decision control variable is included in each feedrate segment,the difference between \( s(i \) \( + 1) \) and \( s\left( i\right) \) must comply with the following conditions,

\[\left\{  \begin{array}{ll} s\left( {i + 1}\right)  - s\left( i\right)  \geq  {2p} + 1 & \left( {i = 0,r - 1}\right) \\  s\left( {i + 1}\right)  - s\left( i\right)  \geq  p + 1 & \left( {1 \leq  i \leq  r - 2}\right)  \end{array}\right.  \tag{25}\]

The progressive knot insertion method can be employed to determine the knot vector of each feedrate segment and the GA-based optimization is carried out to find the optimal solution of the control points. Fig. 8 shows a typical segmented and optimized feedrate profile. It can be seen that the feedrate profile is smooth and all constraint restrictions are ensured even at the splitting parameter positions. Since the adjustment of control points in each divided segment has no effect on the shape of other segments, the feedrate optimization in all segments can be performed concurrently.

The segmented feedrate profile around the splitting parameter positions needs to be further optimized to achieve an uninterrupted and high-efficiency machining. As shown in Fig. 9, the feedrate profile is redivided into a series of transition segments at the middle knots of original segments. Similarly, only part of the control points in each transition segment is selected as the decision variables to ensure that the optimization of feedrate in different transition segments is independent to each other, and other related control points are fixed at the positions specified by the previously optimized feedrate profile. Consequently, the GA-based feedrate optimization in all transition segments can be carried out concurrently. The segmented feedrate profile that meets all constraint restrictions is definitely the feasible solution of the optimization problem and will be selected as an individual of the initial population to accelerate the GA optimization process. Since the knot vector of the B-spline feedrate profile has been refined adequately in the previous segmented optimization, there is no necessary to insert new knots in the transition segments. The results in Fig. 9 show that only a small range of feedrate profile around the splitting parameter positions needs to be adjusted while most of the feedrate is maintained, which allows the GA optimization to find the optimal solution very quickly. Thus, the global time-optimal feedrate can be obtained by combining the feedrate profile in all transition segments.

### 3.5. Global time-optimal feedrate scheduling on multi-axis machine tools

The flowchart of the proposed optimization-based feedrate scheduling method is shown in Fig. 10. For a specified parametric tool path, the knot vector of the B-spline feedrate profile can be initialized in the same parameter range. Since the feedrate scheduling in multi-axis machining with drive constraints is closely related to the structure of machine tools, the kinematics of the employed machine tool should be built and then the GA-based feedrate optimization model considering all constraints can be established. The high-dimensional feedrate optimization issue can be factorized and reorganized into some independent low-dimensional problems by dividing the feedrate profile into small segments and approximately selecting control points as decision variables. The GA-based feedrate optimization in all segments can be conducted concurrently and the knot vector of the B-spline feedrate profile in each segment can be determined using the progressive knot insertion method. Afterwards, the segmented feedrate profile will be re-divided into a series of transition segments, in which the feedrate optimization is performed concurrently to generate the global time-optimal feedrate.

<!-- Media -->

<!-- figureText: Feedrate segments \( {u}_{s\left( {i + 1}\right) } \) ...... \( {u}_{s\left( r\right) } \) \( {u}_{s\left( 0\right) } \) ...... \( {u}_{s\left( i\right) } \) -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_7.jpg?x=109&y=1714&w=727&h=432&r=0"/>

Fig. 8. Concurrent feedrate optimization in the divided segments.

<!-- figureText: ✓ Global time-optimal feedrate \( {u}_{s\left( {i + 1}\right) } \) ...... \( {u}_{s\left( r\right) } \) Segmented feedrate Transition segments \( {u}_{s\left( 0\right) } \) \( {u}_{s\left( i\right) } \) Middle knots -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_7.jpg?x=899&y=152&w=753&h=527&r=0"/>

Fig. 9. Concurrent feedrate optimization in transition segments.

<!-- Media -->

The proposed feedrate scheduling method is applicable for any multi-axis machine tool with complex kinematic structure, including the two-turret machine tool with three rotary axes and robotics. Moreover, this method is robust and computationally efficient for the machining process with high-order constraints and very long tool path. It is also convenient for this method to reconfigure the feedrate optimization model by relaxing or tightening the constraints, or even changing the degree of B-spline feedrate profile.

## 4. Implementations

To verify the feasibility and superiority of the proposed feedrate scheduling method, three case studies are conducted in this section. All employed parametric tool paths are represented by dual NURBS curves to describe the tool motion smoothly and accurately [42]. The dual NURBS tool path is composed by CL path and CL offset path, in which the CL path is used to determine the position of cutter and the CL offset path is derived by offsetting the CL path along the tool axis with a fixed distance. Moreover, the parameter of the CL offset path has been synchronized with that of CL path. Thus, the orientation of tool axis at each parameter position can be determined by joining the two corresponding points from the CL path to the CL offset path. The feedrate scheduling is performed with respect to the NC machining on the two-turret machine tool whose kinematics related parameters are defined in Table 2. The related parameters and constraints in different cases are given in Table 3. The drive constraints of axis \( B \) and \( D \) of the machine tool are exactly the same while different from that of axis \( C \) . All calculations are coded by MATLAB and run on a personal computer with Intel i3 CPU and 8 G Memory. The feedrate optimization model is solved using the built-in GA toolbox in MATLAB and the parameters are set to default options unless otherwise stated.

<!-- Meanless: 8-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

<!-- Media -->

<!-- figureText: Start Conduct the progressive knot insertion and feedrate optimization in all segments concurrently Re-divide the segmented feedrate profile into transition segments Conduct the feedrate optimization in all transition segments concurrently Concurrent feedrate optimization Generate the global time-optimal feedrate End Specify the tool path, machine tool, machining requirements, and feed drive constraints Initialize the knot vector of B- spline feedrate profile Built the inverse kinematics and establish the GA-based feedrate optimization model Divide the feedrate profile into small segments at some parametric knot positions Factorize and reorganize the feedrate optimization into some independent problems Preparations -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_8.jpg?x=296&y=153&w=1156&h=725&r=0"/>

Fig. 10. Flowchart of the optimization-based feedrate scheduling method.

Table 2

Kinematics related parameters of the two-turret machine tool.

<table><tr><td>Parameters</td><td>\( {\Delta x} \)</td><td>\( {\Delta z} \)</td><td>\( {l}_{1} \)</td><td>\( {l}_{2} \)</td><td>\( L \)</td></tr><tr><td>Value (mm)</td><td>-3.37</td><td>233.24</td><td>150</td><td>20</td><td>600.005</td></tr></table>

Table 3

Parameters in different feedrate scheduling cases.

<table><tr><td colspan="3">Parameters</td><td>Case I</td><td>Case II</td><td>Case III</td></tr><tr><td colspan="3">Total length of CL path (mm)</td><td>240.73</td><td>1013.63</td><td>152.50</td></tr><tr><td colspan="3">Degree of the B-spline feedrate profile \( p \)</td><td>3</td><td>3</td><td>3</td></tr><tr><td colspan="3">Interpolation period \( T\left( \mathrm{\;{ms}}\right) \)</td><td>1</td><td>1</td><td>1</td></tr><tr><td colspan="3">Chord error \( \delta \left( \mathrm{{mm}}\right) \)</td><td>0.001</td><td>0.001</td><td>0.001</td></tr><tr><td colspan="3">Threshold \( \Delta {f}_{\max } \) of machining time reduction for progressive knot insertion (s)</td><td>0.1</td><td>0.1</td><td>0.05</td></tr><tr><td rowspan="3">Tangential constraints</td><td colspan="2">Velocity \( {V}_{\max }\left( {\mathrm{{mm}}/\mathrm{s}}\right) \)</td><td>80</td><td>120</td><td>70</td></tr><tr><td colspan="2">Acceleration \( {A}_{\max }\left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) \)</td><td>400</td><td>500</td><td>400</td></tr><tr><td>Jerk \( {J}_{\max }\left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \)</td><td/><td>10,000</td><td>14,000</td><td>12,000</td></tr><tr><td rowspan="9">Drive constraints</td><td rowspan="3">Velocity \( {V}_{\max }^{\tau } \)</td><td>\( B/D \) (rad/s)</td><td>1.75</td><td>1.75</td><td>0.875</td></tr><tr><td>\( C \) (rad/s)</td><td>209</td><td>209</td><td>104.5</td></tr><tr><td>\( W(\mathrm{{mm}}/ \) s)</td><td>650</td><td>650</td><td>325</td></tr><tr><td rowspan="3">Acceleration \( {A}_{\max }^{\tau } \)</td><td>\( B/D \) (rad \( /{\mathrm{s}}^{2} \) )</td><td>60</td><td>60</td><td>30</td></tr><tr><td>\( C \) (rad/ \( {\mathrm{s}}^{2}) \)</td><td>87</td><td>87</td><td>43.5</td></tr><tr><td>\( W(\mathrm{{mm}}/ \) \( \left. {\mathrm{s}}^{2}\right) \)</td><td>500</td><td>500</td><td>250</td></tr><tr><td rowspan="3">Jerk \( {J}_{\max }^{\tau } \)</td><td>\( B/D \) (rad \( /{\mathrm{s}}^{3} \) )</td><td>1000</td><td>1000</td><td>500</td></tr><tr><td>\( C \) (rad/ \( \left. {\mathrm{s}}^{3}\right) \)</td><td>1200</td><td>1200</td><td>600</td></tr><tr><td>\( W(\mathrm{{mm}}/ \) \( {\mathrm{s}}^{3}) \)</td><td>8000</td><td>8000</td><td>4000</td></tr><tr><td/><td>Maximal computation time in each segment (s)</td><td/><td>8</td><td>13</td><td>8</td></tr><tr><td colspan="3">Number of required knots</td><td>163</td><td>167</td><td>247</td></tr></table>

<!-- Media -->

## Case I

A planar rabbit-shaped tool path, as shown in Fig. 11(a), is utilized in this case for interpolation to demonstrate the kinematics performance in the machining process under the scheduled feedrate. This tool path is represented by dual NURBS curves with 46 knots and 42 control points. The motion of each drive axis along the tool path is shown in Fig. 11(b). It can be seen that although the tool orientation is fixed at(0,0,1)in \( {\mathrm{{CS}}}_{P} \) ,the motion of all four drive axes are involved in the machining process. As a main motion participating axis,the rotary axis \( C \) needs to move more than one round, while other axes only swing or reciprocate in very small ranges. The feed direction of each drive axis changes frequently due to the complex shape of tool path, which poses great challenges to the optimization of feedrate.

<!-- Media -->

<!-- figureText: 20 CL path CL offset path Tool axis 10 20 30 ✘ 300 C-axis (deg) 200 100 0 -100 0.5 Parameter u 197 196.5 196 0 0.5 1 Parameter u 10 Feed direction 0 Starting position -10 -20 -20 -10 0 (a) B-axis (deg) 0.5 Parameter u D-axis (deg) W-axis (mm) 0 0.5 Parameter u (b) -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_8.jpg?x=896&y=972&w=764&h=1101&r=0"/>

Fig. 11. Case I, (a) Dual NURBS tool path; (b) Motion of each drive axis.

<!-- Media -->

<!-- Meanless: 9-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

The knot vector of the initial feedrate profile is set to the same as that of the NURBS tool path. Seven splitting parameter positions divide the feedrate profile into 8 segments and the proposed optimization-based feedrate scheduling is conducted in each segment concurrently considering the constraints in Table 3. The knot vector in each segment is refined and determined using the progressive knot insertion method with the terminating condition of predefined machining time reduction threshold. The maximum computation time in each segment is no more than \( 8\mathrm{\;s} \) . Fig. 12(a) shows the optimized segmented feedrate. In each segment, the knot insertion is repeated less than three times and the total number of knots of the B-spline feedrate profile is 163. After further optimizing the feedrate in transition segments concurrently, the smooth and continuous global time-optimal feedrate profile is finally derived. The machining time can be calculated using Eq. (19) and only \( {5.24}\mathrm{\;s} \) is consumed for the whole interpolation process along the complex tool path. The results in Fig. 12(a) also indicate that the feedrate profile needs to be adjusted only in small regions near the splitting positions while remains unchanged in most of the regions. The phenomenon of such local optimization is mainly due to the fact that the segmented feedrate has reached the optima except for the areas near the splitting positions. The feedrate of cutter tip along the CL path is shown in Fig. 12 (b). It can be seen that the feed motion accelerates in low curvature regions and decelerates around high curvature regions smoothly. The acceleration and jerk along the CL path is shown in Fig. 12(c) and (d). The results demonstrate that no constraints violation occurs. Moreover, in the parameter ranges,i.e., \( u \in  \left\lbrack  {{0.275},{0.334}}\right\rbrack  ,\left\lbrack  {{0.731},{0.75}}\right\rbrack \) ,and \( \left\lbrack  {{0.942},{0.981}}\right\rbrack \) ,the acceleration and jerk are fixed at zero,which corresponds to the maximum plateau regions of feedrate profile. Fig. 12(e) shows the derived chord error in the interpolation process. Since the chord error is far from saturated, it is not a factor limiting the improvement of feedrate.

<!-- Media -->

<!-- figureText: Time-optimal feedrate 80 60 40 Feedrate (mm/s) 20 0 (b) \( \times  {10}^{4} \) 1 Jerk (mm/s \( {}^{3} \) ) 0.5 0 -0.5 -1 0.2 0.4 0.6 0.8 Parameter u (d) 0.6 0.8 1 Parameter u Segmented feedrate Splitting parameter positions 80 Feedrate (mm/s) 60 40 20 0.2 0.4 0.6 0.8 1 Parameter u (a) \( \times  {10}^{2} \) 5 Acceleration (mm/s \( {}^{2} \) ) -5 0 0.2 0.4 0.6 0.8 Parameter u (c) \( \times  {10}^{-3} \) 1 Chord error (mm) 0.8 0.6 0.4 0.2 0 0.2 0.4 (e) -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_9.jpg?x=153&y=441&w=1448&h=1677&r=0"/>

Fig. 12. Feedrate scheduling results along the tool path, (a) Optimization-based time-optimal feedrate in parameter space; (b) Feedrate along the CL path, (c) Acceleration, and (d) Jerk in parameter space; (e) Chord error.

<!-- Media -->

<!-- Meanless: 10-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

The velocity, acceleration, and jerk of each drive axis are shown in Fig. 13(a)-(c). The results demonstrate that the drive constraints of axis \( B,D \) ,and \( W \) have no restrictions on the scheduling of feedrate since they are far away from constraint boundaries. On the contrary, the feedrate improvement is limited by the drive constraint of axis \( C \) considering the saturated jerk.

In order to reveal the time-optimality of the optimized feedrate, the constraint saturation \( {C}_{\text{sat }}\left( u\right) \) at each parameter position \( u \) along the tool path is calculated using the following formula.

\[{C}_{\text{sat }}\left( u\right)  = \max \left\lbrack  {\frac{\delta \left( u\right) }{{\delta }_{\max }},\frac{v\left( u\right) }{{V}_{\max }},\frac{\left| \dot{v}\left( u\right) \right| }{{A}_{\max }},\frac{\left| \ddot{v}\left( u\right) \right| }{{j}_{\max }},\frac{\left| {\dot{q}}^{\tau }\left( u\right) \right| }{{V}_{\max }^{\tau }},\frac{\left| \ddot{q}\left( u\right) \right| }{{A}_{\max }^{\tau }},\frac{{\dddot{q}}^{\tau }}{{A}_{\max }^{\tau }}}\right\rbrack   \times  {100}\% \]

(26)

It can be seen that the constraint saturation at each parameter position is expressed as the maximal percentage of all considered constraints to their limits. The magnitude of constraint saturation can be served as an evaluation criterion of the time-optimality of the feedrete optimization results, and also used for indicating how much optimization potential is left for the feedrate. As shown in Fig. 14, the constraints are fully saturated at some discrete or continuous parameter positions, and the mean constraint saturation can reach to 89%. It can be inferred from the results that it is possible to further improve the time-optimality of the feedrate by inserting more knots or adjusting the parameters of GA, such as changing the population diversity or population size. However, these actions will inevitably increase the computation time of the optimization process. Considering the balancing between the time-optimality of the optimized feedrate and the computation time, the constraint saturation around 90% is highly desirable.

Case II

A three-dimensional tool path shaped as a paper crane, as shown in Fig. 15(a), is designed to further verify the effectiveness of the proposed feedrate scheduling method. Both the CL path and CL offset path are represented by third degree NURBS curves, and the parameters of the two paths have been synchronized. The orientation of tool axis varies continuously and smoothly along the CL path in a large range. Using the inverse kinematic model and tool orientation projection method mentioned in Section 2.3, the motion of each drive axis can be obtained, as shown in Fig. 15(b). It can be seen that all four axes are involved in the machining process with large motion ranges and the feed direction changes frequently due to the complex shape of tool path.

<!-- Media -->

<!-- figureText: B-axis (rad/s) 0 C-axis (rad/s) \( \times  {10}^{2} \) 2 0 0 0.5 1 Parameter u W-axis (mm/s) \( \times  {10}^{2} \) 0 -5 0 0.5 Parameter u C-axis (rad/s \( {}^{2} \) ) \( \times  {10}^{2} \) 0 -1 0 0.5 1 Parameter u V-axis (mm/s2) \( \times  {10}^{2} \) 5 0 -5 0.5 1 0 Parameter u C-axis (rad/s \( {}^{3} \) \( \times  {10}^{3} \) 0 -1 0 0.5 1 Parameter u W-axis (mm/s \( {}^{3} \) ) \( \times  {10}^{4} \) 0 -1 0 0.5 1 Parameter u -2 0 0.5 Parameter u D-axis (rad/s) 2 0 -2 0 0.5 1 Parameter u (a) B-axis (rad/s \( {}^{2} \) ) \( \times  {10}^{1} \) 0 0.5 Parameter u D-axis (rad/s2) \( \times  {10}^{1} \) 0 0.5 1 Parameter u (b) B-axis (rad/s \( {}^{3} \) ) \( \times  {10}^{3} \) 0 0.5 1 Parameter u D-axis (rad/s \( {}^{3} \) ) \( \times  {10}^{3} \) -1 0 0.5 Parameter u (c) -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_10.jpg?x=891&y=147&w=764&h=1620&r=0"/>

Fig. 13. Feedrate scheduling results of each drive axis, (a) Velocity, (b) Acceleration, and (c) Jerk.

<!-- Media -->

The dual NURBS tool path employed in this case is constructed on a sparse knot vector with only 27 knots. If the initial knot vector of B-spline feedrate profile is selected directly from that of tool path, the feedrate profile can be divided into 5 segments at most considering the segmentation rule in Eq. (25). The parameters and constraints used for the feedrate scheduling are given in Table 3. In each feedrate segment, the GA-based feedrate optimization is conducted concurrently and the knot vector can be determined after no more than four times knot insertions. As shown in Fig. 16(a), the segmented and optimized feedrate profile is produced with four splitting parameter positions and the total number of knots reaches to 167. Although the length of tool path corresponding to each feedrate segment is long with respect to the constraints (more than \( {190}\mathrm{\;{mm}} \) ),the computation time in each segment is still less than \( {13}\mathrm{\;s} \) due to the use of the progressive knot insertion method. The global time-optimal feedrate is finally derived by further optimizing the segmented feedrate in transition segments concurrently and the total machining time for such complex tool path only needs 14.06 s. Fig. 16(b) shows the feedrate of cutter tip along the CL path. It can be seen that the feedrate can achieve smooth transition in the high curvature areas and rapidly accelerate to the peak value in the low curvature areas. The acceleration and jerk along the CL path are shown in Fig. 16(c) and (d). The results indicate that the constraint boundaries can be reached only near the sharp corners of tool path, and the zero acceleration and jerk are directly related to the constant feedrate. Fig. 16 (e) shows the derived chord error. It can be inferred that the chord error constraint has no effect on the improvement of feedrate since the maximum chord error is much smaller than the predefined limitation.

<!-- Meanless: 11-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

<!-- Media -->

<!-- figureText: 100 0.6 0.8 1 Parameter u Constraints saturation (%) 89.1 80 60 40 Mean constraint saturation 20 0 0 0.2 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_11.jpg?x=104&y=149&w=758&h=489&r=0"/>

Fig. 14. Constraint saturation of the optimized feedrate.

<!-- figureText: CL path Tool axis Feed direction -50 100 50 C-axis (deg) 400 200 0 0.5 1 Parameter u W-axis (mm) 200 150 100 0 0.5 1 Parameter u 80 CL offset path 60 40 20 0 -100 -50 50 (a) B-axis (deg) 10 -10 0 0.5 Parameter u D-axis (deg) 20 -20 0 0.5 Parameter u (b) -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_11.jpg?x=106&y=726&w=758&h=1162&r=0"/>

Fig. 15. Case II, (a) Dual NURBS tool path; (b) Motion of each drive axis.

<!-- Media -->

The kinematic behavior of the four drive axes is given in Fig. 17(a)- (c). It can be seen that the velocity, acceleration and jerk of each drive axis are all confined within their limitations. Only the acceleration and jerk of axis \( W \) ,and the jerk of axis \( C \) around \( u = {0.5} \) ,are saturated. These results indicate that for the current tool path, the drive capability of axis \( W \) is one of the key factors that limit the increase of feedrate while the influence from other axes is negligible.

Fig. 18 shows the constraint saturation derived from the scheduled feedrate. It can be seen that the constraint saturation around the parameter position \( u = {0.5} \) ,i.e.,the sharp corner area of the tail,is low, which is caused by the high ACC/DEC requirements and continuous kinematic characteristics of the feed motion. However, in most areas, the constraints are close to full saturation and the mean value can reach to 92%. Although the feedrate still has the potential to continue to optimize, considering the rapid increase in calculation time, it will not be worthwhile to insert more knots into the feedrate profile for further optimization.

## Case III

To verify the practicability of the proposed feedrate scheduling method, a complex tool path shaped as a dolphin, as shown in Fig. 19(a), is designed and utilized in a milling experiment. This tool path is represented by dual NURBS curves with 67 knots and 63 pairs of control points, and the tool orientation varies along the CL path continuously. In order to intuitively display the position of drive axis during the machining process, the tool orientation corresponding to each CL point is definitely located in the radial plane of CL path. Therefore, it is not necessary to perform the projection of tool orientation in the inverse kinematic transformation process. That is, the direction of tool axis illustrated in Fig. 19(a) can be reached directly. Fig. 19(b) shows the motions of the four axes required for the machining process. Although the CL path is located in one plane, the drive axes need to move in large ranges due to the varied tool orientation. In addition, the feed motion of axis \( C \) changes drastically around the parameter position \( u = {0.5} \) ,which will severely limit the increase of feedrate.

The feedrate optimization related parameters are given in Table 3. The initial knot vector of the B-spline feedrate profile is directly taken from the whole knots of the dual NURBS tool path, and 9 uniformly distributed splitting parameter positions are selected to divide the fee-drate profile into 10 segments. In each segment, the feedrate is optimized by GA and the knot vector is refined progressively considering the threshold of \( \Delta {f}_{\max } \) . The feedrate optimization in all segments is carried out concurrently to reduce the computation time. After no more than three times knot insertions in each segment, the optimized segmented feedrate profile with 247 knots is produced, as shown in Fig. 20(a). The consumed computation time for the most time-consuming segment is less than \( 8\mathrm{\;s} \) . The global time-optimal feedrate can be derived by further optimizing the segmented feedrate profile in all transition segments concurrently. Using the scheduled feedrate,only \( {10.03}\mathrm{\;s} \) cycle time is required to complete the NC interpolation following this complex tool path. Fig. 20(b) shows the feedrate along the CL path. It can be seen that due to the frequent changes of tool orientation and strict drive constraints, the feedrate cannot be maintained at a high level even in the low curvature regions, such as region 1, 2, and 3. Especially in region 2 where the cutter moves near the center of axis \( C \) ,the feedrate is restricted within a low range, which is caused by the rapid feed direction changes of drive axes near the parameter position \( u = {0.5} \) ,as shown in Fig. 19(b). The acceleration and jerk along the CL path are shown in Fig. 20(c) and (d), and the chord error is depicted in Fig. 20(e). The results indicate that the geometric constraints are not the main limiting factors for the improvement of feedrate due to the rare occurrence of constraint saturation. Fig. 20(f)-(h) present the velocity, acceleration, and jerk of each drive axis. The results show that no drive constraint violation happens although the feedrate along the tool path fluctuates drastically. Moreover, it can be inferred that the drive capabilities of axis \( W \) and \( D \) are the main influence factor that restricts the increasing of feedrate considering its frequently saturated acceleration and jerk. As a contrast,other constraints,such as the drive capabilities of axis \( B \) and \( C \) , have little restriction on the feedrate improvement. The overall constraint saturation of the optimized feedrate is shown in Fig. 20(i). In most areas, at least one constraint is nearly fully saturated, which brings the mean constraint saturation to 93.6%.

<!-- Meanless: 12-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

<!-- Media -->

<!-- figureText: Time-optimal feedrate 120 100 80 Feedrate (mm/s) 60 40 20 1 (b) \( \times  {10}^{4} \) 1 Jerk (mm/s \( {}^{3} \) ) 0 -1 0 0.2 0.4 0.6 0.8 Parameter u (d) 0.6 0.8 1 Parameter u Segmented feedrate Splitting parameter position: 120 Feedrate (mm/s) 100 80 60 40 20 0 0.2 0.4 0.6 0.8 Parameter u (a) \( \times  {10}^{2} \) Acceleration (mm/s \( {}^{2} \) ) 0 -5 0 0.2 0.4 0.6 0.8 1 Parameter u (c) 1 Chord error (mm) 0.8 0.6 0.4 0.2 0 0 0.2 0.4 (e) -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_12.jpg?x=223&y=147&w=1309&h=1568&r=0"/>

Fig. 16. Feedrate scheduling results along the tool path, (a) Optimization-based time-optimal feedrate in parameter space; (b) Feedrate along the CI. path, (c) Acceleration, and (d) Jerk in parameter space; (e) Chord error.

<!-- Media -->

The milling experiment is conducted on the two-turret machine tool mentioned in Section 2.1, as shown in Fig. 21(a). The cylinder workpiece made of 6061 aluminum alloy with the diameter of \( {50}\mathrm{\;{mm}} \) is prepared and mounted on axis \( C \) . A two-flute ball-end milling cutter manufactured from high-speed steel is adopted for the cutting process. In order to clearly present the cutting marks at the high curvature areas of tool path, the cutter radius of \( {0.5}\mathrm{\;{mm}} \) is used in the experiment. The milling cutter is mounted on the spindle of the machine, and the kinematic related parameters,such as the CL offset \( {\Delta x} \) and \( {\Delta z} \) ,are all given in Table 2.

<!-- Meanless: 13-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

<!-- Media -->

B-axis (rad/s) 2 C-axis (rad/s) \( \times  {10}^{2} \) 0 -2 0 -2 0 0.5 0.5 1

Parameter u Parameter u

D-axis (rad/s) 2 W-axis (mm/s) \( \times  {10}^{2} \) -5 0.5 1 0 -2 0 0.5

Parameter u Parameter u

(a)

B-axis (rad/s2) \( \times  {10}^{1} \) C-axis (rad/s2) < 10 \( {}^{2} \) 1 0 -1 0 0.5 1 5 0 0.5

Parameter u Parameter u

D-axis (rad/s2) \( \times  {10}^{1} \) W-axis (mm/s2) \( \times  {10}^{2} \) 0 -5 0 0.5 1 5 0 0.5

Parameter u Parameter u

(b)

B-axis (rad/s \( {}^{3} \) ) \( \times  {10}^{3} \) C-axis (rad/s \( {}^{3} \) ) \( \times  {10}^{3} \) 0 0 0.5 1 1 -1 0 0.5

Parameter u Parameter u

D-axis (rad/s \( {}^{3} \) ) \( \times  {10}^{3} \) W-axis (mm/s \( {}^{3} \) ) 0.10 1 0 -1 0.5 1 1 -1 0.5

Parameter u Parameter u

(c)

Fig. 17. Feedrate scheduling results of each drive axis, (a) Velocity, (b) Acceleration, and (c) Jerk.

<!-- Media -->

Using the 2nd-order Taylor's expansion method, the parameter value of cutter location in each interpolation period can be determined according to the designed tool path and the scheduled feedrate. Then, the movement of each drive axis can be derived by the inverse kinematic transformation in Section 2.3, and the machining process is performed with the four axes following their trajectories. Considering the rapid feed motion,the rotational speed of spindle is set to \( {11},{000}\mathrm{r}/\mathrm{{min}} \) for a high surface quality. The cutting depth is set to \( {150\mu }\mathrm{m} \) ,which results in a constant cutting mark width of \( {714\mu }\mathrm{m} \) . Fig. 21(b) shows the machining results on the plane of the cylinder workpiece, and the total machining time is only about \( {10}\mathrm{\;s} \) . It can be seen that the shape of cutting mark is identical with that of the specified CL path even in the sharp corner areas. In addition, although the tool path is very complex, not only for the CL path but also for the tool orientation, the machining process is rather smooth and continuous since the drive constraints of all tool axes are restricted within their limitations. As a consequence, there is no any stalling or defect on the cutting mark even in the high curvature areas, such as the areas in the partially enlarged details. Especially around area 4,although the acceleration and jerk of axis \( C \) and \( W \) are close to saturation, the surface quality of cutting mark is almost the same with that in other areas. In order to further demonstrate the significance of the scheduled smooth feed motion for the improvement of machining accuracy, the tracking error, also called position error, of each drive axis that is defined as the deviation between the actual motion position and the commanded trajectory is monitored and recorded. The maximal position errors of the four drive axes,i.e., \( B,D,C \) ,and \( W \) ,are \( {2.7} \times  {10}^{-4} \) deg, \( {2.6} \times  {10}^{-4}\mathrm{\;{deg}},{16.9} \times  {10}^{-4}\mathrm{\;{deg}} \) ,and \( {0.71\mu }\mathrm{m} \) respectively,as shown in Table 4. It should be noted that all position errors are restricted within the nominal motion accuracy of the two-turret machine tool. Accordingly, it can be inferred that the dynamic capabilities of the machine tool are not violated in the machining process. The above results fully illustrate that using the proposed feedrate scheduling method, the high speed machining can be achieved by maximizing the drive capability of machine tool without sacrificing the machining accuracy.

<!-- Media -->

<!-- figureText: 100 0.6 0.8 1 Parameter u Constraints saturation (%) 92 80 60 40 Mean constraint 20 saturation 0 0 0.2 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_13.jpg?x=924&y=151&w=694&h=445&r=0"/>

Fig. 18. Constraint saturation of the optimized feedrate.

<!-- figureText: CL path Feed direction Starting position 10 20 0 -100 -200 0.5 Parameter u 220 210 200 0.5 1 Parameter u CL offset path 15 Tool axis 10 5 0 -5 -10 -20 -10 (a) B-axis (deg) 10 C-axis (deg) -10 0 0.5 1 Parameter u 20 W-axis (mm) D-axis (deg) 10 -10 0 0.5 1 Parameter u (b) -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_13.jpg?x=925&y=684&w=732&h=1068&r=0"/>

Fig. 19. Case III, (a) Dual NURBS tool path; (b) Motion of each drive axis.

<!-- Meanless: 14-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

<!-- figureText: Time-optimal feedrate 40 \( u = {0.5} \) Feedrate (mm/s) W Center of axis \( C \) 10 0 (b) Jerk (mm/s \( {}^{3} \) ) C 0.2 0.4 0.6 0.8 1 Parameter u (d) 0 0.5 0.5 Parameter u Parameter u 0 0.5 N-axis (mm/s) 0.5 Parameter u Parameter u (f) `-axis (rad/s^) 0.5 Parameter u W-axis (mm/s2) \( \times  {10}^{2} \) 0.5 Parameter u < 10 \( {}^{2} \) 0.5 Parameter u /V-axis (mm/s \( {}^{3} \) ) 0.5 Parameter u saturation 0.6 0.8 Parameter u Segmented feedrate 0.4 Parameter u (a) 5 Acceleration (mm/s \( {}^{2} \) 0.2 0.4 0.6 0.8 Parameter u (c) \( \times  {10}^{-3} \) Chord error (mm) 0.8 0.6 0.2 0 0.2 0.4 0.6 0.8 Parameter u (e) B-axis (rad/s \( {}^{2} \) ) 0.5 Parameter u D-axis (rad/s2) \( \times  {10}^{1} \) 0.5 Parameter u B-axis (rad/s \( {}^{3} \) ) 0.5 Parameter u D-axis (rad/s \( {}^{3} \) ) \( \times  {10}^{2} \) 0.5 Parameter u (h) Constraints saturation (%) 80 60 0.2 0.4 (i) -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_14.jpg?x=442&y=143&w=874&h=1905&r=0"/>

Fig. 20. Feedrate scheduling results, (a) Optimization-based time-optimal feedrate in parameter space; (b) Feedrate along the CL path, (c) Acceleration, and (d) Jerk in parameter space; (e) Chord error; (f) Velocity, (b) Acceleration, and (c) Jerk of each drive axis; (i) Constraint saturation of the optimized feedrate.

<!-- Meanless: 15-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

<!-- figureText: Axis W Workpiece Ball-end milling cutter 3 4 Axis (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2slap77aajc738sf89g_15.jpg?x=365&y=149&w=1023&h=1277&r=0"/>

Fig. 21. Experimental verification, (a) Machining experiment using the scheduled feedrate; (b) Machining results.

Table 4

Maximal position error of each drive axis.

<table><tr><td>Drive axes</td><td>\( B \)</td><td>\( D \)</td><td>\( C \)</td><td>\( W \)</td></tr><tr><td>Maximal position</td><td>\( {2.7} \times \)</td><td>\( {2.6} \times \)</td><td>\( {16.9} \times \)</td><td>\( {0.71\mu }\mathrm{m} \)</td></tr><tr><td>errors</td><td>\( {10}^{-4}\mathrm{\;{deg}} \)</td><td>\( {10}^{-4}\mathrm{\;{deg}} \)</td><td>\( {10}^{-4}\mathrm{\;{deg}} \)</td><td/></tr></table>

<!-- Media -->

## 5. Conclusions

This paper proposed a general optimization-based feedrate scheduling method considering the geometry and high-order drive constraints for multi-axis NC machine tool, typically a two-turret machine tool with three rotary axes and one linear axis. Comparing to other traditional optimization-based algorithms, which are difficult to deal with the highly nonlinear relationship between the motion of cutter tip and the motion of drive axes, the proposed method can produce a global time-optimal feedrate efficiently and robustly without any simplification of constraints. The method development and several implementations are presented. The main contributions made in this work are summarized as follows.

(1) A B-spline curve is used to represent the feedrate profile and the feedrate optimization model considering the geometry and drive constraints is built.

(2) The control points of the time-optimal B-spline feedrate profile are obtained by solving the constructed feedrate optimization model using GA.

<!-- Meanless: 16-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

(3) Using the progressive knot insertion method, the knot vector of the global time-optimal B-spline feedrate profile can be determined efficiently.

(4) The computational efficiency of the proposed feedrate scheduling method can be greatly improved according to the segmented optimization and concurrent computation methods.

(5) The effectiveness and time-optimality of the feedrate scheduling method is verified by several simulation and experimental results.

The proposed optimization-based feedrate scheduling method can also be applied for any other multi-axis machine tools or even industrial robots, for which only the number of constraints may change. Moreover, the time-optimality and computational efficiency of this method have great potential to be improved by optimizing the parameters, such as the degree and initial knot vector of the B-spline feedrate profile and the GA related parameters.

## CRediT authorship contribution statement

Fusheng Liang: Conceptualization, Methodology, Software, Visualization, Methodology, Validation, Writing - original draft, Resources. Guangpeng Yan: Methodology, Writing - review & editing. Fengzhou Fang: Resources, Supervision, Project administration, Writing - review & editing.

## Declaration of Competing Interest

None.

## Acknowledgments

This publication has emanated from research supported in part by a grant from Science Foundation Ireland under Grant number [No. 15/ RP/B3208]. For the purpose of Open Access, the author has applied a CC BY public copyright licenses to any Author Accepted Manuscript version arising from this submission. The authors would like to thank the "111" Project by the State Administration of Foreign Experts Affairs and the Ministry of Education of China [No. B07014].

## Disclosure statement

No potential conflict of interest was reported by the authors. References

[1] M. Langelaar, Topology optimization for multi-axis machining, Comput. Methods Appl. Mech. Eng. 351 (2019) 226-252, https://doi.org/10.1016/j.cma.2019.03.037.

[2] S.S. Chen, C.F. Cheung, F.H. Zhang, M.Y. Liu, Optimization of tool path for uniform scallop-height in ultra-precision grinding of free-form surfaces, Nano Manuf. Metrol. (2019), https://doi.org/10.1007/s41871-019-00048-0.

[3] F.S. Liang, C.W. Kang, F.Z. Fang, A review on tool orientation planning in multi-axis machining, Int. J. Prod. Res. (2020) 1-31, https://doi.org/10.1080/ 00207543.2020.1786187.

[4] F.S. Liang, C.W. Kang, Z.Y. Lu, F.Z. Fang, Iso-scallop tool path planning for triangular mesh surfaces in multi-axis machining, Robot. Comput. Integr. Manuf. 72 (2021), 102206, https://doi.org/10.1016/j.rcim.2021.102206.

[5] Y.S. Lu, Y.Y. Lin, Smooth motion control of rigid robotic manipulators with constraints on high-order kinematic variables, Mechatronics 49 (2018) 11-25, https://doi.org/10.1016/j.mechatronics.2017.11.003.

[6] K. Erkorkmaz, Y. Altintas, High speed CNC system design. Part I: jerk limited trajectory generation and quintic spline interpolation, Int. J. Mach. Tools Manuf. 41 (9) (2001) 1323-1345, https://doi.org/10.1016/S0890-6955(01)00002-5.

[7] J.E. Bobrow, S. Dubowsky, J.S. Gibson, Time-optimal control of robotic manipulators along specified paths, Int. J. Robot. Res. 4 (3) (1985) 3-17, https:// doi.org/10.1177/027836498500400301.

[8] F. Pfeiffer, R. Johanni, A concept for manipulator trajectory planning, IEEE Trans. Robot. Autom. 3 (2) (1987) 115-123, https://doi.org/10.1109/Jra.1987.1087090.

[9] S.D. Timar, R.T. Farouki, T.S. Smith, C.L. Boyadjieff, Algorithms for time-optimal control of CNC machines along curved tool paths, Robot. Comput. Integr. Manuf. 21 (1) (2005) 37-53, https://doi.org/10.1016/j.rcim.2004.05.004.

[10] D. Constantinescu, E.A. Croft, Smooth and time-optimal trajectory planning for industrial manipulators along specified path, J. Robot. Syst. 17 (5) (2000) 233-249, 10.1002/(Sici)1097-4563(200005)17:5<233::Aid-Rob1>3.0.Co;2-Y.

[11] D. Verscheure, B. Demeulenaere, J. Swevers, J. De Schutter, M. Diehl, Time-optimal path tracking for robots: a convex optimization approach, IEEE Trans. Automat. Control 54 (10) (2009) 2318-2327, https://doi.org/10.1109/ Tac. 2009.2028959.

[12] W. Fan, X.S. Gao, W. Yan, C.M. Yuan, Interpolation of parametric CNC machining path under confined jounce, Int. J. Adv. Manuf. Technol. 62 (5-8) (2012) 719-739, https://doi.org/10.1007/s00170-011-3842-0.

[13] L. Lu, J. Zhang, J.Y.H. Fuh, J. Han, H. Wang, Time-optimal tool motion planning with tool-tip kinematic constraints for robotic machining of sculptured surfaces, Robot. Comput. Integr. Manuf. 65 (2020), 101969, https://doi.org/10.1016/j.rcim.2020.101969.

[14] M.T. Lin, M.S. Tsai, H.T. Yau, Development of a dynamics-based NURBS interpolator with real-time look-ahead algorithm, Int. J. Mach. Tools Manuf. 47 (15) (2007) 2246-2262, https://doi.org/10.1016/j.ijmachtools.2007.06.005.

[15] L. Tang, J. Huang, L.M. Zhu, X.Y. Zhu, G.Y. Gu, Path tracking of a cable-driven snake robot with a two-level motion planning method, IEEE Asme. Trans. Mech. 24 (3) (2019) 935-946, https://doi.org/10.1109/Tmech.2019.2909758.

[16] H.P. Ni, C.R. Zhang, Q.Z. Chen, S. Ji, T.L. Hu, Y.A. Liu, A novel time-rounding-up-based feedrate scheduling method based on S-shaped ACC/DEC algorithm, Int. J. Adv. Manuf. Technol. 104 (5-8) (2019) 2073-2088, https://doi.org/10.1007/ s00170-019-03882-0.

[17] L.M. Zhu, J. Huang, X. Du, A complete S-shape feed rate scheduling approach for NURBS interpolator, J. Comput. Des. Eng. 2 (4) (2015) 206-217, https://doi.org/ 10.1016/j.jcde.2015.06.004.

[18] A.C. Lee, M.T. Lin, Y.R. Pan, W.Y. Lin, The feedrate scheduling of NURBS interpolator for CNC machine tools, Comput. Aided Des. 43 (6) (2011) 612-628, https://doi.org/10.1016/j.cad.2011.02.014.

[19] J. Jahanpour, M.R. Alizadeh, A novel ACC-jerk-limited NURBS interpolation enhanced with an optimized S-shaped quintic feedrate scheduling scheme, Int. J. Adv. Manuf. Technol. 77 (9-12) (2014) 1889-1905, https://doi.org/10.1007/ s00170-014-6575-z.

[20] J. Huang, L.M. Zhu, Feedrate scheduling for interpolation of parametric tool path using the sine series representation of jerk profile, P I Mech. Eng. B J Eng. 231 (13) (2016) 2359-2371, https://doi.org/10.1177/0954405416629588.

[21] Y. Fang, J. Qi, J. Hu, W.M. Wang, Y.H. Peng, An approach for jerk-continuous trajectory generation of robotic manipulators with kinematical constraints, Mech. Mach. Theory 153 (2020), 103957, https://doi.org/10.1016/j.mechmachtheory.2020.103957.

[22] Y. Zhang, M.Y. Zhao, P.Q. Ye, H. Zhang, A G4 continuous B-spline transition algorithm for CNC machining with jerk-smooth feedrate scheduling along linear segments, Comput. Aided Des. 115 (2019) 231-243, https://doi.org/10.1016/j.cad.2019.04.004.

[23] D.S. Du, Y.D. Liu, X.G. Guo, K. Yamazaki, M. Fujishima, An accurate adaptive NURBS curve interpolator with real-time flexible acceleration/deceleration control, Robot. Comput. Integr. Manuf. 26 (4) (2010) 273-281, https://doi.org/ 10.1016/j.rcim.2009.09.001.

[24] H. Zhao, L.M. Zhu, H. Ding, A real-time look-ahead interpolation methodology with curvature-continuous B-spline transition scheme for CNC machining of short line segments, Int. J. Mach. Tools Manuf. 65 (2013) 88-98, https://doi.org/ 10.1016/j.ijmachtools.2012.10.005.

[25] J. Huang, Y.A. Lu, L.M. Zhu, Real-time feedrate scheduling for five-axis machining by simultaneously planning linear and angular trajectories, Int. J. Mach. Tools Manuf. 135 (2018) 78-96, https://doi.org/10.1016/j.ijmachtools.2018.08.006.

[26] Y. Alintas, K. Erkormaz, Feedrate optimization for spline interpolation in high speed machine tools, CIRP Ann. Manuf. Technol. 52 (1) (2003) 297-302, https:// doi.org/10.1016/S0007-8506(07)60588-5.

[27] J.F. Zhou, Y.W. Sun, D.M. Guo, Adaptive feedrate interpolation with multiconstraints for five-axis parametric toolpath, Int. J. Adv. Manuf. Technol. 71 (9-12) (2014) 1873-1882, https://doi.org/10.1007/s00170-014-5635-8.

[28] K. Erkorkmaz, Q.G. Chen, M.Y. Zhao, X. Beudaert, X.S. Gao, Linear programming and windowing based feedrate optimization for spline toolpaths, CIRP Ann. Manuf. Technol. 66 (1) (2017) 393-396, https://doi.org/10.1016/j.cirp.2017.04.058.

[29] Y.W. Sun, M.S. Chen, J.J. Jia, Y.S. Lee, D.M. Guo, Jerk-limited feedrate scheduling and optimization for five-axis machining using new piecewise linear programming approach, Sci. China Technol. Sci. 62 (7) (2019) 1067-1081, https://doi.org/ 10.1007/s11431-018-9404-9.

[30] B. Sencer, Y. Altintas, E. Croft, Feed optimization for five-axis CNC machine tools with drive constraints, Int. J. Mach. Tools Manuf. 48 (7-8) (2008) 733-745, https://doi.org/10.1016/j.ijmachtools.2008.01.002.

[31] A. Gasparetto, V. Zanotto, Optimal trajectory planning for industrial robots, Adv. Eng. Softw. 41 (4) (2010) 548-556, https://doi.org/10.1016/j.advengsoft.2009.11.001.

[32] Q. Zhang, S.R. Li, J.X. Guo, Smooth time-optimal tool trajectory generation for CNC manufacturing systems, J. Manuf. Syst. 31 (3) (2012) 280-287, https://doi.org/10.1016/j.jmsy.2012.06.001.

[33] F.B. Xie, L.F. Chen, Z.Y. Li, K. Tang, Path smoothing and feed rate planning for robotic curved layer additive manufacturing, Robot. Comput. Integr. Manuf. 65 (2020), 101967, https://doi.org/10.1016/j.rcim.2020.101967.

[34] K. Erkorkmaz, M. Heng, A heuristic feedrate optimization strategy for NURBS toolpaths, CIRP Ann. Manuf. Technol. 57 (1) (2008) 407-410, https://doi.org/ 10.1016/j.cirp.2008.03.039.

<!-- Meanless: 17-->




<!-- Meanless: F. Liang et al. Robotics and Computer-Integrated Manufacturing 75 (2022) 102308-->

[35] Y.W. Sun, Y. Zhao, Y.R. Bao, D.M. Guo, A novel adaptive-feedrate interpolation method for NURBS tool path with drive constraints, Int. J. Mach. Tools Manuf. 77 (2014) 74-81, https://doi.org/10.1016/j.ijmachtools.2013.11.002.

[36] Y.W. Sun, Y. Zhao, J.T. Xu, D.M. Guo, The feedrate scheduling of parametric interpolator with geometry, process and drive constraints for multi-axis CNC machine tools, Int. J. Mach. Tools Manuf. 85 (5) (2014) 49-57, https://doi.org/ 10.1016/j.ijmachtools.2014.05.001.

[37] Y.W. Sun, Y. Zhao, Y.R. Bao, D.M. Gu, A smooth curve evolution approach to the feedrate planning on five-axis toolpath with geometric and kinematic constraints, Int. J. Mach. Tools Manuf. 97 (2015) 86-97, https://doi.org/10.1016/j.ijmachtools.2015.07.002.

[38] F.S. Liang, J. Zhao, S.J. Ji, An iterative feed rate scheduling method with confined high-order constraints in parametric interpolation, Int. J. Adv. Manuf. Technol. 92 (5-8) (2017) 2001-2015, https://doi.org/10.1007/s00170-017-0249-6.

[39] A. Bharathi, J.Y. Dong, Feedrate optimization for smooth minimum-time trajectory generation with higher order constraints, Int. J. Adv. Manuf. Technol. 82 (5-8) (2016) 1029-1040, https://doi.org/10.1007/s00170-015-7447-x.

[40] G.X. Li, H.T. Liu, W. Yue, J.L. Xiao, Feedrate scheduling of a five-axis hybrid robot for milling considering drive constraints, Int. J. Adv. Manuf. Technol. 112 (11-12) (2021) 3117-3136, https://doi.org/10.1007/s00170-020-06559-1.

[41] L. Piegl, W. Tiller, The NURBS Book, Second ed, Springer, New York, 1997.

[42] J.M. Langeron, E. Duc, C. Lartigue, P. Bourdet, A new format for 5-axis tool path computation, using Bspline curves, Comput. Aided Des. 36 (12) (2004) 1219-1229, https://doi.org/10.1016/j.cad.2003.12.002.

<!-- Meanless: 18-->

