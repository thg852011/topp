

<!-- Meanless: Robotics and Computer-Integrated Manufacturing 86 (2024) 102681 Contents lists available at ScienceDirect Robotics and Computer-Integrated Manufacturing ELSEVIER journal homepage: www.elsevier.com/locate/rcim updates-->

# A novel tool path smoothing algorithm of \( 6\mathrm{R} \) manipulator considering pose-dependent dynamics by designing asymmetrical FIR filters

Hongwei Sun, Jixiang Yang \( {}^{ * } \) , Han Ding

School of Mechanical Science and Engineering, State Key Laboratory of Intelligent Manufacturing Equipment and Technology, Huazhong University of Science and Technology, Wuhan, Hubei 430074, PR China

## ARTICLEINFO

Keywords:

Corner smoothing

FIR filters

Robot manipulator

Dynamic constraints

## A B S T R A C T

The joint dynamic constraints of robot manipulators are normally set to conservative constant values to meet the dynamic tolerances at different poses, which leads to incomplete utilization of the joint drive capability. To fully utilize the drive capability of joint drives, this paper proposes an asymmetrical finite impulse response (FIR) filter-based tool path smoothing algorithm, which considers the pose-dependent dynamics of the robot with 6 rotational (6R) joints. To analyze the pose-dependent motor drive capabilities, a simplified dynamics model is established by considering both the tangential and joint dynamic constraints. An asymmetrical FIR filter-based tool path smoothing algorithm is also proposed to realize different constraints of the acceleration and the deceleration limits at different positions of the linear segments. The tool tip position and the tool orientation commands are respectively smoothed in the Cartesian coordinate system and spherical coordinate system, with the constraints of the tool path deviations in the task frame. The synchronization of the position and the orientation is realized by adjusting the parameters of FIR filters. The simulation and experimental results show that the proposed algorithm guarantees the corner error tolerances of the tool tip position and the tool orientation. Moreover, the proposed method improves the robot motion efficiency by more than 10% through the full utilization of the joint drive capability compared with the traditional tool path smoothing algorithm based on symmetrical FIR filters.

## 1. Introduction

Five-axis computer numerical control (CNC) machine tools play a major role in the traditional manufacturing process due to the high quality and precision of its processing result \( \left\lbrack  {1,2}\right\rbrack \) . With the development of robotics technology, industrial robots are gradually playing significant roles in the manufacturing domain [3,4], such as in polishing [5-7], milling [8-11], grinding [12-15], and deburring [16-20]. Compared with five-axis CNC machine tools, industrial robots have the advantages of high adaptability, large working space, and low cost, thus attracting research interest of scholars in the field of robotic machining. The interpolation and smoothing of robot motion trajectory are important research points in machining tasks. Compared with traditional point-to-point motion tasks, such as palletizing operations, machining tasks have stricter requirements for contour accuracy and continuity of the motion along the whole path. In the point-to-point motion type, only the positional accuracy of the start and end points of the trajectory must be guaranteed, thus, a simple polynomial interpolation algorithm is normally used to plan the feed rate. However, geometrical deviation constraints and kinematic continuity of the tool tip position in the contour motion task, and the tool orientation should be ensured along the trajectory. Thus, interpolation and smoothing are challenging in the contour motion tasks.

The motion drive commands generated in the Computer Aided Manufacturing software are discrete linear segments [21-23], which causes tangential discontinuous motion at the corner of two adjacent linear segments. In this case, the robot would stop at the corners of the trajectory to avoid discontinuous tangential velocity and acceleration shocks, causing motion efficiency loss. Therefore, before being sent to the robot drive controller, discrete motion commands must be interpolated and smoothed under the contour deviation error and the dynamic constraints. The robot tool path smoothing algorithm can be divided into the task space method and the joint space method based on the space in which the tool path commands are smoothed. The joint space method directly smoothens the joint commands with the joint dynamic constraints [24-26]. Its advantages include being able to directly smoothen the joint motion drive commands without inverse kinematics calculation

---

<!-- Footnote -->

* Corresponding author.

E-mail address: jixiangyang@hust.edu.cn (J. Yang).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.rcim.2023.102681 Received 13 July 2023; Received in revised form 6 October 2023; Accepted 16 October 2023 Available online 30 October 2023 0736-5845/(© 2023 Elsevier Ltd. All rights reserved.-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

## Nomenclature

\( {M}_{i}\left( s\right) \) Laplace transform function of the FIR filter \( {T}_{i} \) Time constant of the FIR filter \( {m}_{i}\left( t\right) \) Function of the FIR filter in the time domain \( u\left( t\right) \) Unit step function \( {s}_{p,k} \) Displacement of the \( k \) -th position path \( {v}_{k,\max } \) Velocity limitation of the \( k \) -th position path \( {a}_{k,\max } \) Acceleration limitation of the \( k \) -th corner \( {j}_{\max } \) Jerk limitation of the position path \( {T}_{p,i}^{k,1},{T}_{p,i}^{k,2} \) FIR filter time constants of the \( k \) -th position path \( {T}_{p,\text{total}}^{k} \) Motion time of the \( k \) -th position path \( {s}_{p}^{k,1}\left( t\right) ,{s}_{p}^{k,2}(t \) The \( k \) -th symmetrical interpolated position path \( {s}_{p}^{k}\left( t\right) \) The \( k \) -th asymmetrical interpolated position path \( {\delta }_{p}^{k} \) Time offset of the \( k \) -th position path \( {\varepsilon }_{p}^{k} \) Corner error of the \( k \) -th position path \( {\varepsilon }_{p,\text{tolerance}} \) Corner error tolerance of the position path \( {T}_{v,p}^{k} \) The \( k \) -th overlapping time of the position path \( {s}_{o,k} \) Displacement of the \( k \) -th adjacent tool orientation \( {\omega }_{k,\max } \) Velocity limitation of the \( k \) -th adjacent tool orientation \( {\omega }_{k,\max } \) Acceleration limitation of the \( k \) -th adjacent tool orientation \( {\omega }_{\max } \) Jerk limitation of the \( k \) -th adjacent tool orientation \( {T}_{o,i}^{k,1},{T}_{o,i}^{k,2} \) FIR filter time constants of the \( k \) -th orientation path \( {T}_{o,\text{ total }}^{k} \) Motion time of the \( k \) -th orientation path \( {s}_{o}^{k,1}\left( t\right) ,{s}_{o}^{k,2}\left( t\right) \) The \( k \) -th symmetrical interpolated orientation path

\( {s}_{o}^{k}\left( t\right) \) The \( k \) -th asymmetrical interpolated orientation path

\( {\delta }_{o}^{k} \) Orientation time offset of the \( k \) -th orientation path

\( {\varepsilon }_{o}^{k} \) Orientation corner error of the \( k \) -th orientation path

\( {\mathcal{E}}_{o,\text{ tolerance }} \) Orientation corner error tolerance

\( {T}_{o,p}^{k} \) The \( k \) -th overlapping time of the orientation path

\( {\mathbf{p}}_{k} \) The \( k \) -th position coordinate vector

\( {\mathbf{O}}_{k} \) The \( k \) -th orientation coordinate vector

\( \mathbf{q} \) Vector of robot joint position

\( \dot{\mathbf{q}} \) Vector of robot joint velocity

\( \ddot{\mathbf{q}} \) Vector of robot joint acceleration

\( {q}_{s}^{n},{q}_{ss}^{n} \) First- and second-order derivative of the \( n \) -th joint angular displacements related to the tangential displacement

\( {\dot{q}}_{n,\max },{\ddot{q}}_{n,\max } \) Joint velocity constraint and acceleration constraints of the \( n \) -th joint

RRotation matrix

THomogeneous transformation matrix

\( \alpha ,\beta ,\gamma \) ZYX Euler angles

\( \mathbf{M}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right) \) Equivalent inertial torques on joints

\( \mathbf{G}\left( \mathbf{q}\right) \) Equivalent gravity torques of joints

\( {\mathbf{\tau }}_{f}\left( \dot{\mathbf{q}}\right) \) Friction torques of joints

\( {\mathbf{\tau }}_{m} \) Motor drive torques of the joints

\( {\tau }_{ext} \) Joint additional torques induced by the load

\( {}^{n}{\mathbf{L}}_{r} \) Barycenter coordinate from the z-axis of the \( n \) -th joint

\( {I}_{n}\left( \mathbf{q}\right) \) Equivalent rotating inertia on the axis direction of the \( n \) -th joint

KJoint drive gain vector

\( i \) Joint currents and ensuring the dynamic constraints of each joint. However, because the smoothing algorithm is carried out in the joint space, the deviation errors of the robot end tool path in the workpiece are difficult to control, resulting in the low profile accuracy of machined products. Meanwhile, in the task space smoothing method, the smoothing algorithms are directly applied to plan the tool tip position and orientation in the workpiece coordinate system (WCS) [27-30]. The tool tip position is described using the Cartesian coordinates, while the tool orientation is described by a unit direction vector defined in spherical coordinates. The end pose of robots is often represented by Euler angles, e.g., Qu et al. [31] represented the robot postures by Euler angles and proposed a profile error-oriented optimization model by directly optimizing the feed directions and end-effector postures. The contour deviation errors along the tool path can be evaluated and restricted intuitively by using the task space method.

The robot trajectory smoothing algorithm focuses on the tool path accuracy, the motion continuity, and the dynamic constraints of joint motions. In general, the typical path smoothing methods are divided into two steps named as "two-step" method. Firstly, the tool paths are smoothed with geometric error constraints by applying the analytical parametric splines. Then, the smoothed path is interpolated based on the scheduled feedrate that meets the dynamic constraints. The first step, also called "geometric smoothing" includes the global smoothing method and the local smoothing method. The tool path is directly smoothed by using an entire spline to represent it in the global smoothing method [32-35]. Although the smoothness of the tool path can be easily achieved by applying the global smoothing method, it is difficult to evaluate the constraints of the deviation errors along the entire tool path. Conversely, the local smoothing method only smoothens the corner around the two adjacent segments by a parametric spline, while the rest of the path remains in the linear part [36-40]. In this method, the deviation errors of the tool path are easily evaluated and constrained because only the errors at the corners of adjacent linear segments must be considered. Huang et al. [41] proposed a smoothing method based on a clothoid pair, which synchronously accomplish planning of geometry blending and speed scheduling with high efficiency. There are various types of parametric splines used in the path smoothing methods such as B-spline [42-45], Bezier [46-48], Pythagorean-Hodograph (PH) curves [49-52], and Nurbs curves [53-55]. The two-step method has been well studied in the literature. However, the smoothing and the interpolation of the tool path are implemented separately, which inevitably increases the complexity and calculation burden of the tool path smoothing algorithm. In recent years, the one-step smoothing method based on finite impulse response (FIR) filters has been studied [56], in which the smoothing and the interpolation of the path are completed in one step, and the dynamic constraints are realized by designing the parameters of the FIR filters. Fang et al. [57] proposed a 3-axis path smoothing algorithm based on FIR filtering, which was applied to the blending tool-paths with long segments and micro-segments and effectively controlled the contour error. Tajima et al. [58,59] applied the interpolation method based on FIR filters to the 5-axis machine tools, where the overlapping time was specially designed to achieve continuous motion with the constraints of geometric deviation error tolerances at corners of adjacent segments. Ward [60] proposed a joint workpiece-machine coordinate system interpolation scheme, in which the tool position was interpolated in the workpiece coordinate system (WCS), and the tool orientation was interpolated in the machine coordinate system (MCS) based on FIR filtering. Liu et al. [61] considered both tangential and bounded axial dynamic constraints of the 5-axis machine tool. To smooth segment junctions under the constraint of axis jerk, Tang [62] proposed a filter-based corner smoothing method with double filter (FCSDF) technique to generate interpolated trajectories for high-speed machine tools. Similarly, both Ishizaki [63] and Song [64] studied FIR-based path smoothing algorithm of the 5-axis machine, which improved motion efficiency and constrained trajectory deviation. The above literatures are for FIR-based path smoothing algorithm in 5-axis machine tools. Based on FIR filters, the authors developed a path smoothing method for robots with 6 rotary joints in a previous study [65], in which both the tangential and the joint dynamic limits are constrained.

<!-- Meanless: 2-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

<!-- Media -->

<!-- figureText: Pose-dependent dynamic constraints method Asymmetrical FIR filter-based path smoothing algorithm ③ Linear path Interpolated path time(s) time(s) Interpolation of the disceret path ④ Before smoothing After smoothing \( = {s}_{p}^{k}\left( t\right)  + {s}_{p}^{k + 1}\left( t\right) \) 5 15 Overlapping time \( {T}_{\mathrm{v}} \) Time(s) Time(s) Corner smoothing of the adjacent paths \( {T}_{p,1}^{k,1\prime } = {T}_{p,1}^{k,2\prime } = \left( {{T}_{o,\text{ total }}^{k} - {T}_{p,\text{ total }}^{k}}\right)  + {T}_{p,1}^{k,1},\; \) if \( {T}_{o,\text{ total }}^{k} > {T}_{p,\text{ total }}^{k} \) \( {T}_{o,1}^{k,1\prime } = {T}_{o,1}^{k,2\prime } = \left( {{T}_{p,\text{ total }}^{k} - {T}_{0,\text{ total }}^{k}}\right)  + {T}_{o,1}^{k,1},\; \) if \( {T}_{p,\text{ total }}^{k} > {T}_{o,\text{ total }}^{k} \) \( {T}_{v}^{k} = \min \left\{  {{T}_{v,p}^{k},{T}_{v,o}^{k}}\right\} \) Synchronization of position and orientation motion ① \( M\left( {q,\ddot{q}}\right)  = \mathbf{G}\left( \mathbf{q}\right)  + {\mathbf{\tau }}_{f}\left( \dot{\mathbf{q}}\right)  + {\mathbf{\tau }}_{m} + {\mathbf{\tau }}_{ext} \) RMSE-7.9706 RMSE-3.2274 10 15 35 Modeling of the robot pose-dependent dynamics ② \( \dot{\mathbf{q}} = \frac{d\mathbf{q}}{dt} = \frac{d\mathbf{q}}{ds}\frac{ds}{dt} = {\mathbf{q}}_{s}\dot{\mathbf{s}} \) \( \ddot{\mathbf{q}} = \frac{d\dot{\mathbf{q}}}{dt} = \frac{d\left( {{\mathbf{q}}_{s}\dot{s}}\right) }{dt} = {\mathbf{q}}_{ss}{\dot{s}}^{2} + {\mathbf{q}}_{s}\ddot{s} \) 10 15 \( {T}_{1} = \max \left\{  {\frac{S\left| {q}_{s}^{n}\right| }{{\dot{q}}_{n,\max }},\frac{s}{{\dot{s}}_{\max }}}\right\} \) Joints dynamics performance Hybrid constraints of tangential and joint dynamic characteristics -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_2.jpg?x=185&y=152&w=1374&h=824&r=0"/>

Fig. 1. Overview of the proposed robotic path smoothing algorithm that considers pose-dependent dynamic constraints.

<!-- Media -->

The difference in the path smoothing process of a 6-axis robot and a 5-axis machine tool is mainly caused by different structural characteristics. Normally, 5-axis machine tools have three translation axes and two rotation axes to control the end tool pose, where the tool position is mainly affected by three translation axes and the tool orientation is affected by two rotation axes. Compared with 5-axis machine tools, the 6 degrees of freedom (DOF) industrial robot has an open-chain structure with rotary joints. Both position and orientation of the end tool may be affected by 6 rotary axes based on the robot kinematics. Therefore, the \( 6\mathrm{R} \) robot has more complex forward and inverse kinematics characteristics than the 5-axis machine tool. When implementing the smoothing algorithm, the kinematic constraints of the robot axis are more difficult. Furthermore, the acceleration and deceleration capabilities of the machine tool axis are the same in all machining positions. However, different robot poses result in varying motor drive capabilities for each joint motor due to variances in the gravity moment of the link and the equivalent inertia of the joint. Consequently, different joint dynamic constraints arise under different robot poses. Traditional path interpolation methods often conservatively set fixed values for joint dynamic constraints, leading to a loss of motion efficiency due to insufficient utilization of the joint driving capacity.

To address this problem, this paper presents an asymmetrical FIR filter-based tool path smoothing method that considers the different driving abilities of joint motors under different joint poses. Fig. 1 provides an overview of the proposed method. The hybrid constraints of tangential and joint dynamic characteristics are calculated based on the modeling of the robot pose-dependent dynamics. Subsequently, the pose-dependent dynamic constraints are restricted in the tool path interpolation by applying the asymmetrical FIR filter-based path smoothing algorithm. The paper's novel contributions can be summarized as follows: (1) An asymmetrical trajectory smoothing algorithm based on FIR filters is proposed to make it able to realize different constraints of the acceleration limits at different positions of the tool path; (2) The tool path deviation constraints and the synchronization of the tool tip position and orientation are realized by specially designing time constants of FIR filters; (3) A simplified dynamics model is established to analyze the pose-dependent motor drive capabilities by considering both the tangential and joint dynamic constraints. Experimental results demonstrate that the proposed method improves robot motion efficiency by fully utilizing the joint drive capability compared with the traditional tool path smoothing algorithm based on symmetrical FIR filters.

The remainder of the paper is organized as follows: In Section 2, the asymmetrical FIR filter-based path interpolation algorithm is proposed. In Section 3, pose-dependent dynamic constraints are analyzed based on the robot dynamics model and hybrid constraints of the tangential and joint dynamic characteristics are illustrated. In Section 4, the specific process of the proposed algorithm is discussed. In Section 5, experiments are designed to evaluate the effectiveness of the proposed algorithm. The paper is concluded in Section 6 finally.

## 2. Trajectory smoothing algorithm based on asymmetrical FIR filters

This section first introduces the basic principle of the symmetrical FIR filter-based trajectory smoothing algorithm. Then, an asymmetrical FIR filter-based smoothing algorithm is proposed to overcome the limitations of the equal acceleration and deceleration limits caused by the symmetrical method. The robot tool paths are interpolated and smoothed within the geometric deviation constraints in the WCS based on the proposed algorithm. Thus, the synchronization of the tool tip position and orientation are realized by designing the time constants of the FIR filters.

<!-- Meanless: 3-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

<!-- Media -->

<!-- figureText: 0.6 1.5 Time(s) 0.5 Amplitude 0.4 0.3 Area=1 0.1 0.5 1 -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_3.jpg?x=162&y=150&w=627&h=502&r=0"/>

Fig. 2. Function image in the time domain of \( {m}_{i}\left( t\right) \) .

<!-- Media -->

### 2.1. Symmetrical FIR filter-based trajectory interpolation algorithm

In this section, the mathematical principle of the FIR filter is introduced. According to Ref. [66], its mathematical expression is as follows:

\[{M}_{i}\left( s\right)  = \frac{1}{{T}_{i}}\frac{1 - {e}^{-s{T}_{i}}}{s},i = 1,2,\ldots  \tag{1}\]

where \( {M}_{i}\left( s\right) \) means the Laplace transform function,and \( {T}_{i} \) is the time constant. In particular, \( {M}_{i}\left( s\right) \) is expressed as \( {m}_{i}\left( t\right) \) in the time domain as follows:

\[{m}_{i}\left( t\right)  = \frac{1}{{T}_{i}}\left\lbrack  {u\left( t\right)  - u\left( {t - {T}_{i}}\right) }\right\rbrack  \text{,with}u\left( t\right)  = \left\{  \begin{array}{l} 1,t \geq  0 \\  0,t < 0 \end{array}\right.  \tag{2}\]

The function image of \( {m}_{i}\left( t\right) \) is shown in Fig. 2. As can be seen,the curve of the function \( {m}_{i}\left( t\right) \) and the time axis form a rectangle,with an intercept on the time axis of \( {T}_{i} \) and a function amplitude of \( 1/{T}_{i} \) .

The motion distance \( {s}_{p} \) is expressed as \( {s}_{p}u\left( t\right) \) . The smoothing operation of the trajectory is carried out using the convolution calculation as:

\[{s}_{p}\left( t\right)  = {s}_{p}u\left( t\right)  * {m}_{1}\left( t\right)  * \cdots  * {m}_{i}\left( t\right)  \tag{3}\]

where \( {s}_{p}\left( t\right) \) stands for the filtered trajectory by \( i \) FIR filters. Here, \( i \) takes a value of 3 to obtain the jerk-limited trajectory. The dynamic characteristics of the trajectory filtered for 3 times are shown in Fig. 3.

The dynamic characteristics of the filtered trajectory are determined by the time constants \( \left( {{T}_{1},{T}_{2},{T}_{3}}\right) \) . The time constants are calculated by the dynamic constraints as follows:

\[{T}_{1} = \frac{{s}_{p}}{{v}_{\max }},{T}_{2} = \frac{{v}_{\max }}{{a}_{\max }},{T}_{3} = \frac{{a}_{\max }}{{j}_{\max }} \tag{4}\]

where \( {v}_{\max },{a}_{\max } \) ,and \( {j}_{\max } \) stand for the dynamic constraints (velocity, acceleration, and jerk, respectively) of the path. According to Ref. [66], the time constants should be optimally designed to satisfy the dynamic constraints. Otherwise, the velocity or the acceleration would not reach the set limits, causing motion task efficiency loss. To guarantee time optimality, both Eqs. (5) and (6) should be strictly satisfied:

\[{T}_{1} \geq  {T}_{2} + {T}_{3} \tag{5}\]

\[{T}_{2} \geq  {T}_{3} \tag{6}\]

If only Eq. (5) is violated, \( {v}_{\max } \) should be adjusted using Eq. (7).

\[{v}_{\max } = \frac{1}{2}\left( {-\frac{{a}_{\max }^{2}}{{j}_{\max }} + \sqrt{\frac{{a}_{\max }^{4}}{{j}_{\max }^{2}} + 4{\begin{Vmatrix}{s}_{p}\end{Vmatrix}}_{{a}_{\max }}}}\right)  \tag{7}\]

If both Eqs. (5) and (6) are violated, \( {v}_{\max } \) and \( {a}_{\max } \) are adjusted by Eq. (8).

\[\left\{  \begin{array}{l} {v}_{\max } = \sqrt[3]{\frac{{\begin{Vmatrix}{s}_{p}\end{Vmatrix}}^{2}{j}_{\max }}{4}} \\  {a}_{\max } = \sqrt[3]{\frac{\begin{Vmatrix}{s}_{p}\end{Vmatrix}{j}_{\max }^{2}}{2}} \end{array}\right.  \tag{8}\]

Thereafter, the time constants are updated by Eq. (4) using the adjusted dynamic limitations. The velocity characteristic of the interpolated path is given by Eq. (9).

<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

\[v\left( t\right)  = \frac{{ds}\left( t\right) }{dt} = {s}_{p}{m}_{1}\left( t\right)  * {m}_{2}\left( t\right)  * {m}_{3}\left( t\right) \]

\[\left\{  \begin{array}{l} \frac{1}{2}{v}_{\max },{\gamma }_{0},0 \leq  t < {T}_{3} \\  \frac{1}{2}{v}_{\max },{\gamma }_{0} \leq  t < {T}_{3} \\  \frac{1}{2}{v}_{\max },{\gamma }_{0} \leq  {T}_{3},{\gamma }_{3} \leq  t < {T}_{2} \\  {v}_{\max } - \frac{1}{2}{\gamma }_{0}{v}_{1}\left( {{T}_{2} + {T}_{3}}\right)  - {v}^{2},{T}_{2} \leq  t < {T}_{2} + {T}_{3} \\  {v}_{\max },{T}_{0} \leq  {T}_{3} \leq  t < {T}_{1} \\  {v}_{\max } - \frac{1}{2}\frac{{v}_{\max }\left( {t - {T}_{3}}\right) }{1 - {v}_{\max }},{T}_{1} \leq  t < {T}_{1} + {T}_{3} \\  {v}_{\max } - \frac{1}{2}\frac{{v}_{\max }\left( {t - {T}_{2}}\right) }{1 - {v}_{\max }},{T}_{2} \leq  t < {T}_{1} + {T}_{2} \\  \frac{1}{2}\frac{{v}_{\max }\left( {t - {T}_{3}}\right) }{1 - {v}_{\max }},{T}_{3} \leq  t < {T}_{2} + {T}_{3} + {T}_{2} + {T}_{3} \\  {v}_{\max } - \frac{1}{2}\frac{{v}_{\max }\left( {t - {T}_{2}}\right) }{1 - {v}_{\max }},{T}_{1} \leq  t < {T}_{2} + {T}_{3} + {T}_{2} + {T}_{3} \\  {v}_{\max } - \frac{1}{2}\frac{{v}_{\max }\left( {t - {T}_{3}}\right) }{1 - {v}_{\max }},{T}_{2} \leq  t < {T}_{1} + {T}_{2} + {T}_{3} \end{array}\right.  \tag{9}\]

<!-- Media -->

<!-- figureText: Acceleration ( \( \mathrm{{mm}}/{\mathrm{s}}^{2} \) ) 5 6 7 8 5 6 7 8 4 5 6 7 8 Time (s) Jerk (mm/s \( {}^{3} \) ) 0 1 2 3 -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_3.jpg?x=458&y=1387&w=845&h=669&r=0"/>

Fig. 3. Dynamic characteristics of the trajectory when \( i \) takes 3 .

<!-- Meanless: 4-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

<!-- figureText: Filtered by \( {a}_{k,\max } \) \( {s}_{p}^{k,1}\left( t\right) \) \( {s}_{p}^{k,1}\left( t\right) \) Filtered \( v \) signal 0 5 10 Time (s) Combine \( {s}_{p} \) and \( {s}_{p}{}^{k} \) 10 Time (s) Time (s) Velocity (mm/s) 5 Original \( v \) signal Velocity (mm/s) Time (s) Calculate 0 10 Velocity (mm/s) \( {s}_{p}^{k,2}\left( t\right) \) 0 Time (s) Filtered by \( {a}_{k + 1,\max } \) Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_4.jpg?x=181&y=291&w=1389&h=429&r=0"/>

Fig. 4. The principle of the asymmetrical FIR filter-based smoothing algorithm.

<!-- figureText: \( {s}_{p}^{k,1}\left( t\right) \) Discontinuous velocity in the short segment \( {s}_{p}^{k,1}\left( t\right) \) Filtered \( v \) signal 10 Combine \( {s}_{p}{}^{k,1} \) and \( {s}_{p}{}^{k\text{.}} \) Velocity (mm/s) \( {s}_{p}^{k,2}\left( {t - {\delta }_{p}^{k}}\right) \) 0 5 10 Time (s) Discontinuous velocity Time (s) \( {s}_{p}^{k,1}\left( t\right) \) Filtered \( v \) signal Continuous velocity \( {\delta }_{n}^{k} \) Time (s) Combine \( {s}_{p}{}^{k,1} \) and \( {s}_{p}{}^{k,2} \) \( {s}_{p}^{k,2}\left( {t - {\delta }_{p}^{k}}\right) \) 0 10 Time (s) Time (s) Continuous velocity by adjusting the time constant \( {T}_{p,1}^{k,1} \) Filtered by \( {a}_{k,\max } \) Velocity (mm/s) Time (s) Calculate \( {\delta }_{n}^{k} \) Short segment \( v \) signal Velocity (mm/s) 0 10 elocity (mm/s) \( {s}_{p}^{k,2}\left( t\right) \) Time (s) Filtered by \( {a}_{k + 1,\max } \) 10 Time (s) Adjust Velocity (mm/s) \( {s}_{p}^{k,1}\left( t\right) \) Time (s) Calculate Velocity (mm/s) \( {s}_{p}^{k,2}\left( t\right) \) Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_4.jpg?x=179&y=812&w=1397&h=979&r=0"/>

Fig. 5. Adjustment of the time constants to deal with the velocity discontinuity of short segments.

<!-- Meanless: 5-->


<!-- figureText: (a) Before smoothing After smoothing Velocity (mm/s) \( {s}_{p}^{k}\left( t\right)  + {s}_{p}^{k + 1}\left( {t + {T}_{v,p}^{k}}\right) \) 0 10 15 Overlapping time \( {T}_{v,k} \) Time(s) \( {\theta }_{p}^{k} \) \( {l}_{1} \) \( {\mathbf{P}}_{m}^{k} \) \( {\mathbf{P}}_{k + 1} \) Velocity (mm/s) 6 \( {s}_{p}^{k}\left( t\right)  + {s}_{p}^{k + 1}\left( t\right) \) Apply smoothing 4 algoritm 2 0 5 10 15 Time(s) (b) \( {\mathbf{P}}_{k} \) \( {\theta }_{p}^{k} \) \( {\mathbf{P}}_{m}^{k} \) \( {\overrightarrow{\mathbf{t}}}_{k + 1} \) \( {\varepsilon }_{p} = \begin{Vmatrix}{{\mathbf{P}}_{m} - {\mathbf{P}}_{k + 1}}\end{Vmatrix} \) \( {\mathbf{P}}_{k + 2} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_5.jpg?x=188&y=151&w=1381&h=773&r=0"/>

Fig. 6. The principle of the corner smoothing method: (a) The velocity performance after setting the overlapping time. (b) Corner geometric characteristics after smoothing.

<!-- Media -->

### 2.2. Asymmetrical FIR filter-based path interpolation algorithm

The tool path smoothing and interpolation method in Section 2.1 causes the same acceleration constraints in the acceleration and deceleration phases of the linear segment which exhibits symmetrical dynamic characteristics in Fig. 3. However, the acceleration limitations at the endpoints of the linear segments are actually different because of the pose-dependent robot dynamics. If a conservative value of the acceleration constraint is taken to ensure the joint acceleration constraints at the end points, it would cause a decrease in the motion efficiency. Thus, to deal with this problem, an asymmetrical FIR filter-based interpolation algorithm is presented in this section.

Take the linear segment \( {\mathbf{P}}_{k}{\mathbf{P}}_{k + 1} \) for analysis. The dynamic constraints are the velocity limitation \( {v}_{k,\max } \) along the segment \( {\mathbf{P}}_{k}{\mathbf{P}}_{k + 1} \) ,the acceleration limitations \( {a}_{k,\max } \) and \( {a}_{k + 1,\max } \) at points \( {\mathbf{P}}_{k} \) and \( {\mathbf{P}}_{k + 1} \) ,respectively and the jerk limitation \( {j}_{\max } \) which is set as a fixed value along the path. The displacement of the linear segment \( {\mathbf{P}}_{k}{\mathbf{P}}_{k + 1} \) is \( {s}_{p,k} \) . Firstly,the symmetrical interpolation method is carried out twice using different accelerations \( {a}_{k,\max } \) and \( {a}_{k + 1,\max } \) . In this way,two sets of time constants \( \left( {T}_{p,i}^{k,1}\right. \) and \( \left. {T}_{p,i}^{k,2}\right) \) are obtained by Eqs. (4)-(8). Time constants \( {T}_{p,i}^{k,1}(i = 1,2 \) , 3) are calculated based on the acceleration constraints \( {a}_{k,\max } \) and time constants \( {T}_{p,i}^{k,2}\left( {i = 1,2,3}\right) \) are calculated based on the acceleration constraints \( {a}_{k + 1,\max } \) . In this case,two interpolated paths \( \left( {{s}_{p}^{k,1}\left( t\right) }\right. \) and \( {s}_{p}^{k,2}\left( t\right) \) ) are obtained with different acceleration constraints. Then,the path \( {s}_{p}^{k}\left( t\right) \) is obtained by combining \( {s}_{p}^{k,1}\left( t\right) \) and \( {s}_{p}^{k,2}\left( t\right) \) in the following way:

\[{s}_{p}^{k}\left( t\right)  = {s}_{p}^{k,1}\left( t\right)  \cdot  \left( {1 - {\mu }_{p}^{k}\left( t\right) }\right)  + {s}_{p}^{k,2}\left( {t - {\delta }_{p}^{k}}\right)  \cdot  {\mu }_{p}^{k}\left( t\right)  \tag{10}\]

where \( {\mu }_{p}^{k}\left( t\right) \) is defined as:

\[{\mu }_{p}^{k}\left( t\right)  = \left\{  {\begin{array}{l} 0,t < {t}_{p}^{k} \\  1,{t}_{p}^{k} \leq  t \end{array},\text{ with }{t}_{p}^{k} = \mathop{\sum }\limits_{{i = 1}}^{3}{T}_{p,i}^{k,1}/2}\right.  \tag{11}\]

Half of the displacement is reached at time \( {t}_{p}^{k} \) due to the symmetrical dynamic characteristics of \( {s}_{p}^{k,1}\left( t\right) \) . To achieve continuity for the remaining half of the displacement, \( {s}_{p}^{k,2}\left( t\right) \) is set with a time offset \( {\delta }_{p}^{k} \) , which is calculated based on the time constants \( {T}_{p,i}^{k,1} \) and \( {T}_{p,i}^{k,2} \) :

\[{\delta }_{p}^{k} = \frac{\mathop{\sum }\limits_{{i = 1}}^{3}{T}_{p,i}^{k,1} - \mathop{\sum }\limits_{{i = 1}}^{3}{T}_{p,i}^{k,2}}{2}. \tag{12}\]

The principle of the algorithm is shown in Fig. 4.

The calculations of \( {T}_{p,1}^{k,1} \) and \( {T}_{p,1}^{k,2} \) are all based on the same velocity constraints \( {v}_{k,\max } \) and displacement \( {s}_{p,k} \) according to Eq. (4). Therefore, \( {T}_{p,1}^{k,1} \) and \( {T}_{p,1}^{k,2} \) are equal. Considering that in the case of short line segments, \( {v}_{k,\max } \) may not be reached,and \( {T}_{p,1}^{k,1} = {T}_{p,1}^{k,2} \) would be violated, which may cause discontinuous velocity. As shown in Fig. 5, in the case of short segments,the maximum velocities of \( {s}_{p}^{k,1} \) and \( {s}_{p}^{k,2} \) are different caused by the case that \( {T}_{p,1}^{k,1} \neq  {T}_{p,1}^{k,2} \) . In the case shown in Fig. 5,the maximum velocity of \( {s}_{p}^{k,1}\left( t\right) \) is greater than that of \( {s}_{p}^{k,2}\left( t\right) \) because \( {T}_{p,1}^{k,1} \) \( < {T}_{p,1}^{k,2} \) . Therefore, \( {T}_{p,1}^{k,1} \) and \( {T}_{p,1}^{k,2} \) are adjusted to be the same to avoid discontinuous velocity.

<!-- Meanless: 6-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

\[\left\{  \begin{array}{l} {T}_{p,1}^{k,1} = {T}_{p,1}^{k,2},\text{ if }{T}_{p,1}^{k,1} < {T}_{p,1}^{k,2} \\  {T}_{p,1}^{k,2} = {T}_{p,1}^{k,1},\text{ if }{T}_{p,1}^{k,1} > {T}_{p,1}^{k,2} \end{array}\right.  \tag{13}\]

<!-- Media -->

<!-- figureText: (a) Feed Proposed method (b) Proposed method - Comparison method Acceleration (mm \( /{\mathrm{s}}^{2} \) ) Velocity (mm/s) 0 2 3 5 7 100 -100 5 6 500 Jerk (mm/s \( {}^{3} \) ) -500 0 3 5 7 9 Time (s) direction Compared method Discrect path \( {\mathbf{P}}_{v}\left( \mathrm{{mm}}\right) \) 20 40 \( {\mathbf{P}}_{1}^{1}\left( \mathbf{{mm}}\right) \) \( {\mathrm{P}}_{\mathrm{v}}\left( \mathrm{{mm}}\right) \) (mm) P. (mm) 2.05 59.5 7.95 60.5 \( {\mathbf{P}}_{\mathbf{v}}\left( \mathbf{{mm}}\right) \) \( {\mathrm{P}}_{\mathrm{v}}\left( \mathrm{{mm}}\right) \) P, (mm) \( {\mathrm{P}}_{\mathrm{v}}\left( \mathrm{{mm}}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_6.jpg?x=210&y=152&w=1332&h=507&r=0"/>

Fig. 7. The simulation results using the proposed method and the comparison method (a) The tool path smoothing results (b) The dynamic characteristics (velocity, acceleration and jerk) of the smoothed tool paths.

<!-- Media -->

### 2.3. Corner smoothing method based on the overlapping time

After interpolating each segment, the velocity decreases to zero, as shown in Fig. 3. If the tool tip position adopts the point-point motion, it will stop at each corner, thus causing the loss of motion efficiency. Hence, to accomplish the smoothness transition at the corner of the adjacent linear segments, the corner smoothing algorithm is developed by specially designing the overlapping time \( \left( {T}_{v,p}\right) \) . The principle of the corner smoothing method is shown in Fig. 6(a).

As shown in Fig. 6(b), the actual trajectory after smoothing deviates from the original path due to the overlapping time. By taking the \( k \) -th corner for illustration,the corner error \( {\varepsilon }_{p}^{k} \) is estimated by \( {\mathbf{P}}_{m}^{k}{\mathbf{P}}_{k + 1} \) :

\[{\varepsilon }_{p}^{k} = \begin{Vmatrix}{{\mathbf{P}}_{m}^{k}{\mathbf{P}}_{k + 1}}\end{Vmatrix} = \begin{Vmatrix}{{l}_{1}{\mathbf{t}}_{k} - {l}_{2}{\mathbf{t}}_{k + 1}}\end{Vmatrix} \leq  {\varepsilon }_{p,\text{ tolerance }} \tag{14}\]

where \( {\mathbf{t}}_{k} \) and \( {\mathbf{t}}_{k + 1} \) represent the motion direction of adjacent segments \( \left( {{\mathbf{P}}_{k}{\mathbf{P}}_{k + 1}}\right. \) and \( \left. {{\mathbf{P}}_{k + 1}{\mathbf{P}}_{k + 2}}\right) \) ,respectively,and \( {\varepsilon }_{p,\text{tolerance }} \) is the corner error tolerance expressed as:

\[\left\{  \begin{array}{l} {\mathbf{t}}_{k} = \frac{{\mathbf{P}}_{k}{\mathbf{P}}_{k + 1}}{\begin{Vmatrix}{\mathbf{P}}_{k}{\mathbf{P}}_{k + 1}\end{Vmatrix}} \\  {\mathbf{t}}_{k + 1} = \frac{{\mathbf{P}}_{k + 1}{\mathbf{P}}_{k + 2}}{\begin{Vmatrix}{\mathbf{P}}_{k + 1}{\mathbf{P}}_{k + 2}\end{Vmatrix}} \end{array}\right.  \tag{15}\]

When \( {v}_{k,\max } = {v}_{k + 1,\max },{\mathbf{P}}_{m}^{k} \) is the midpoint of the curve part at the corner. \( {\mathbf{P}}_{k}{\mathbf{P}}_{k + 1} \) and \( {\mathbf{P}}_{k + 1}{\mathbf{P}}_{k + 2} \) share the same acceleration constraint \( {a}_{k,\max } \) resulting in \( {T}_{p,i}^{k,2} = {T}_{p,i}^{k,1}\left( {i = 2,3}\right) \) . In Eq. (14), \( {l}_{1} \) and \( {l}_{2} \) represent the deceleration part of \( {\mathbf{P}}_{k}{\mathbf{P}}_{k + 1} \) and the acceleration part of \( {\mathbf{P}}_{k + 1}{\mathbf{P}}_{k + 2} \) during the overlapping time,respectively. Here, \( {l}_{1} \) and \( {l}_{2} \) are calculated as follows.

\[\left\{  \begin{array}{l} {l}_{1} = {\int }_{{T}_{p,\text{ total }}^{k} - {T}_{v}^{k}/2}^{{T}_{p,\text{ total }}^{k}}{v}_{k}\left( t\right) {dt} \\  {l}_{2} = {\int }_{0}^{{T}_{v}^{k}/2}{v}_{k + 1}\left( t\right) {dt} \end{array}\right.  \tag{16}\]

Combining Eqs. (9),(15),and (16), \( {l}_{1} \) and \( {l}_{2} \) are formulated as follows.

\[{l}_{1} = \left\{  \begin{array}{l} \frac{{T}_{v,p}^{k,3}}{{48}{P}_{p,2}^{k,2}{T}_{p,k}^{k,2}{v}_{k,\max },0 \leq  {T}_{v,p}^{k} \leq  2{T}_{p,3}^{k,2}} \\  \frac{4{\left( {T}_{p,3}^{k,2}\right) }^{2} - 6{T}_{p,3}^{k,2}{T}_{v,p}^{k} + 3{T}_{v,k}^{2}}{{24}{T}_{p,2}^{k,2}}{v}_{k,\max },2{T}_{p,3}^{k,2} \leq  {T}_{v,p}^{k} \leq  {T}_{p,2}^{k,2} + {T}_{p,3}^{k,2} \end{array}\right. \]

\[{l}_{2} = \left\{  \begin{array}{l} \frac{{T}_{v,p}^{k}}{{48}{T}_{p,2}^{k + 1,1}{T}_{p,s}^{k + 1,1}}{v}_{k + 1,\max },0 \leq  {T}_{v,p}^{k} \leq  2{T}_{p,3}^{k + 1,1} \\  \frac{4{\left( {T}_{p,3}^{k + 1,1}\right) }^{2} - 6{T}_{p,3}^{k + 1,1}{T}_{v,p}^{k} + 3{T}_{v,k}^{2}}{{24}{T}_{p,2}^{k + 1,1}}{v}_{k + 1,\max },2{T}_{p,3}^{k + 1,1} \leq  {T}_{v,p}^{k} \leq  {T}_{p,2}^{k + 1,1} + {T}_{p,3}^{k + 1,1} \end{array}\right. \]

(17)

Combining Eqs. (14)-(17) and \( {T}_{p,i}^{k,2} = {T}_{p,i}^{k,1}\left( {i = 2,3}\right) \) ,the maximum corner error \( {\varepsilon }_{p}^{k} \) is formulated as:

\[{\varepsilon }_{p}^{k} = \left\{  \begin{array}{l} \text{ if }0 \leq  {T}_{v,p}^{k} \leq  2{T}_{p,3}^{k,2} \\  \frac{{T}_{v,p}^{k}{}^{3}}{{48}{T}_{p,p}^{k,2}{T}_{p,3}^{k,2}}\sqrt{{\left( {v}_{k,\max }\right) }^{2} + {\left( {v}_{k + 1,\max }\right) }^{2} - {v}_{k,\max }{v}_{k + 1,\max }\cos \left( {\theta }_{p}^{k}\right) }, \\  \text{ if }2{T}_{p,3}^{k} \leq  {T}_{v,p}^{k} \leq  {T}_{p,3}^{k,2} + {T}_{p,2}^{k,2} + {T}_{p,2}^{k,2} \\  \frac{4{\left( {T}_{p,3}^{k,2}\right) }^{2} - 6{T}_{p,3}^{k,2}{T}_{v,p}^{k} + 3{T}_{v,p}^{k,2}}{{24}{T}_{p,2}^{k,2}}\sqrt{{\left( {v}_{k,\max }\right) }^{2} + {\left( {v}_{k + 1,\max }\right) }^{2} - {v}_{k,\max }{v}_{k + 1,\max }\cos \left( {\theta }_{p}^{k}\right) } \end{array}\right.  \tag{18}\]

<!-- Meanless: 7-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

where \( \cos \left( {\theta }_{p}^{k}\right)  = {\mathbf{t}}_{k} \cdot  {\mathbf{t}}_{k + 1} \) .

If \( {v}_{\max }^{k} \neq  {v}_{\max }^{k + 1} \) ,Eq. (18) would overestimate the corner error,because Eq. (18) calculates the corner error at the midpoint on the curve part. In this way, the calculated corner error is greater than or equal to the minimum error. Thus, the corner error constraints can be guaranteed as well. The overlapping time \( {T}_{v,p}^{k} \) that satisfies the geometric error deviation tolerance is obtained by restricting the maximum corner error \( {\varepsilon }_{p}^{k} \) in Eq. (18) to be lower than the error tolerance \( {\varepsilon }_{p,\text{ tolerance }} \) . This is achieved using Eq. (19).

\[{T}_{v,p}^{k} = \left\{  \begin{array}{l} \text{ if }0 \leq  {T}_{v,p}^{k} \leq  2{T}_{p,3}^{k,2} \\  \sqrt{\frac{{48}{T}_{p,2}^{k,2}{T}_{p,3}^{k,2}{T}_{p,k}^{k,2}{p}_{k,p,l}\text{ experience }}{\sqrt{{\left( {V}_{k,\max }^{2}\right) }^{2} + {\left( {V}_{k,\max }^{2}\right) }^{2} - {V}_{k,\max }{V}_{k + 1,\max }\cos \left( {\theta }_{p}^{k}\right) }}} \\  \text{ if }2{T}_{p,3}^{k,2} \leq  {T}_{p,3}^{k,2} \leq  {T}_{p,3}^{k,2} + {T}_{p,3}^{k,2} \\  {T}_{p,3}^{k,2} + \sqrt{{\left( {V}_{k,\max }^{2}\right) }^{2} + {\left( {V}_{k,\max }^{2}\right) }^{2} - {V}_{p,3}^{k,2}{V}_{p,3}^{k,2}\cos \left( {\theta }_{p}^{k}\right) } \leq   - \frac{{T}_{p,3}^{k,2}}{3} \end{array}\right. \]

(19)

By calculating overlapping time \( {T}_{v,p}^{k} \) using the corner error tolerance \( {\varepsilon }_{p,\text{ tolerance }} \) in Eq. (19),there are two benefits obtained while satisfying geometric deviation constraints: (1) The maximum corner error increases as the overlapping time increases, therefore, the full utilization of the geometric error deviation tolerance to design overlapping time \( {T}_{v,p}^{k} \) can reduce the cycle time to increase the motion efficiency. (2) As the overlapping time \( {T}_{v,p}^{k} \) increases,the decrease of the velocity in the overlapping area will be relieved, thereby reducing the velocity fluctuations caused by acceleration and deceleration at corners and eventually improving the smoothness of motion.

Comparisons were conducted using the traditional trajectory smoothing algorithm based on the symmetrical FIR filters [65] to illustrate the advantage of the proposed asymmetrical tool path smoothing method. The discrete three-axis path in Fig. 7(a) is used in the test. The velocity constraint is set as \( {10}\mathrm{\;{mm}}/\mathrm{s} \) . The acceleration constraints of the corners were set as40,20,150,40,and \( {70}\mathrm{\;{mm}}/{\mathrm{s}}^{2} \) at each corner. The jerk limit was set as \( {500}\mathrm{\;{mm}}/{\mathrm{s}}^{3} \) ,and the corner error tolerance is set as \( {0.3}\mathrm{\;{mm}} \) . The acceleration constraint and the deceleration constraint of each linear path should be set the same as the conservative acceleration constraints due to the symmetry of the comparison method. Fig. 7(a) demonstrates that both the proposed method and the comparison method can achieve geometric smoothing at the corners. The dynamic performances of the proposed method and the comparison method are illustrated in Fig. 7(b). In the comparison method, a conservative acceleration constraint is adopted during the acceleration and deceleration phases to ensure the acceleration constraints, which doesn't fully utilize the motion drive capability. However, in the proposed asymmetrical tool path smoothing algorithm, the acceleration and deceleration stages of each linear segment adhere to the given limits at each corner, as evidenced by the planned acceleration curve. Consequently, the smoothed path using the proposed algorithm exhibits higher motion efficiency compared with the path smoothed using the comparison method.

## 3. Pose-dependent dynamic constraints of robot manipulators

The previous section introduced the asymmetrical FIR filter-based path interpolation algorithm to accommodate different acceleration constraints at corners. During the robot's motion, the tangential dynamic constraints are closely related to joint motion constraints. However, robot joints exhibit different dynamic performance under different poses, resulting in varying tangential and joint dynamic constraints. Therefore, this section analyzes the pose-dependent dynamic constraints of robot manipulators. Firstly, Section 3.1 establishes a simplified dynamics model of the robot to analyze the dynamic characteristics of robot joints under different postures. Then, based on the dynamics model, the pose-dependent dynamic constraints of the joints are calculated. In Section 3.3, the relationship between the tangential and joint dynamic characteristics is established. Subsequently, the range of time constants of the FIR filters is analyzed to satisfy the pose-dependent dynamic constraints.

### 3.1. Establishment of the robot dynamics model

To estimate the pose-dependent acceleration constraints of the robot joints, the dynamics model is established in this section. The classical robot dynamics modeling methods are mainly based on the Newton-Euler method \( \left\lbrack  {{67},{68}}\right\rbrack \) . However,the dynamic parameters (velocity and acceleration) of different joints tend to be coupled due to the series structure of multiple links. According to Newton's second theorem, a simplified dynamics model of the robot with \( N \) rotary joints is given by the following equation:

\[\mathbf{M}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right)  = \mathbf{G}\left( \mathbf{q}\right)  + {\mathbf{\tau }}_{f}\left( \dot{\mathbf{q}}\right)  + {\mathbf{\tau }}_{m} + {\mathbf{\tau }}_{ext} \tag{20}\]

where \( \mathbf{M}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right)  \in  {\mathbb{R}}^{N \times  1} \) contains equivalent inertial torques on each joint of the robot, \( \mathbf{G}\left( \mathbf{q}\right)  \in  {\mathbb{R}}^{N \times  1} \) stands for the equivalent gravity torques of each joint,and \( {\mathbf{\tau }}_{f}\left( \dot{\mathbf{q}}\right) \) indicates the friction torques of joints. In addition, \( {\mathbf{\tau }}_{m} \in  {\mathbb{R}}^{N \times  1} \) are motor drive torques of the joints,and \( {\mathbf{\tau }}_{\text{ext }} \in  {\mathbb{R}}^{N \times  1} \) are the joint additional torques induced by the load acting on the end of the robot.

For the purpose of analyzing the terms in Eq. (20),the \( n \) -th joint is taken as an example:

\[{M}_{n}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right)  = {G}_{n}\left( \mathbf{q}\right)  - {\tau }_{f,n}\left( \dot{\mathbf{q}}\right)  + {\tau }_{m,n} + {\tau }_{{ext},n} \tag{21}\]

\[n = 1,2\ldots N\]

where \( {\tau }_{m,n} \) is the motion drive torque of the \( n \) -th joint,and \( {\tau }_{{ext},n} \) is the \( n \) - th term of \( {\tau }_{\text{ext }} \) . In addition, \( {G}_{n}\left( \mathbf{q}\right) \) is the equivalent gravity torque of the \( n \) -th joint and can be calculated by effects of \( n \) -th to \( N \) -th links’ gravity.

\[{G}_{n}\left( \mathbf{q}\right)  = \mathop{\sum }\limits_{{r = n}}^{N}n{G}_{r}\left( \mathbf{q}\right) ,r = n,n + 1\ldots ,N \tag{22}\]

\[\text{with}{}^{n}{G}_{r}\left( \mathbf{q}\right)  = \left( {{m}_{r}\mathbf{g} \times  {}^{n}{\mathbf{L}}_{r}}\right)  \cdot  {\mathbf{z}}_{n}\]

where \( {}^{n}{G}_{r}\left( \mathbf{q}\right) \) is the equivalent torque acting on the \( n \) -th joint acting by the gravity of the \( r \) -th link,and \( {}^{n}{\mathbf{L}}_{r} \) is the barycenter coordinate from the z-axis of the \( n \) -th joint,which is described in the Robot Base Coordinate System (RBCS). \( {\mathbf{z}}_{n} \) is a unit direction vector describing the axis direction of the \( n \) -th joint in the RBCS, \( {m}_{r} \) is the mass of the \( r \) -th link,and \( \mathbf{g} \) is the gravitational acceleration. \( {M}_{n}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right) \) is the inertia torque of the \( n \) -th joint and is calculated by the equivalent rotating inertia \( {I}_{n}\left( \mathbf{q}\right) \) on the axis direction of the \( n \) -th joint and the joint acceleration.

\[{M}_{n}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right)  = {I}_{n}\left( \mathbf{q}\right)  \cdot  \left( {\mathop{\sum }\limits_{{r = 1}}^{n}{\ddot{q}}_{r}{\mathbf{z}}_{r}}\right)  \cdot  {\mathbf{z}}_{n} \tag{23}\]

\[\text{with}{I}_{n}\left( \mathbf{q}\right)  = \mathop{\sum }\limits_{{r = n}}^{N}{m}_{r}{\begin{Vmatrix}{\mathbf{z}}_{n} \times  {}^{n}{\mathbf{L}}_{r}\end{Vmatrix}}^{2}\]

where \( {\ddot{q}}_{r} \) is the joint acceleration of the \( r \) -th joint,and \( {\tau }_{f,n} \) is the friction torque of the \( n \) -th joint. As the direction of friction is opposite to the direction of motion,the sign before the friction term \( \left( {\tau }_{f,n}\right) \) in Eq. (21) is taken as "-". To simplify the modeling of joint friction, Ref. [69] and [70] analyzed the frictional behavior using a nonlinear function of the joint velocity given by:

<!-- Meanless: 8-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

\[{\tau }_{f,n} = {k}_{{fn},1}{\dot{q}}_{n} + {k}_{{fn},2}\sqrt{\left| {\dot{q}}_{n}\right| } + {k}_{{fn},3}\]

\[ = \left\lbrack  {{k}_{{fn},1},{k}_{{fn},2},{k}_{{fn},3}}\right\rbrack  \left\lbrack  \begin{matrix} {\dot{q}}_{n} \\  \sqrt{\left| {\dot{q}}_{n}\right| } \\  1 \end{matrix}\right\rbrack   = {\mathbf{K}}_{f,n}{\mathbf{Y}}_{f,n} \tag{24}\]

\[\text{with}{\mathbf{Y}}_{f,n} = {\left\lbrack  {\dot{q}}_{n},\sqrt{\left| {\dot{q}}_{n}\right| },1\right\rbrack  }^{T},{\mathbf{K}}_{f,n} = \left\lbrack  {{k}_{{fn},1},{k}_{{fn},2},{k}_{{fn},3}}\right\rbrack  \]

where \( {\mathbf{Y}}_{f,i} \) is the matrix combined by the joint velocity \( {\dot{q}}_{n} \) ,and \( {\mathbf{K}}_{f,i} \) is the coefficient matrix.

However, \( {\mathbf{K}}_{f,i} \) may vary when the joint rotates in the forward and backward directions. Therefore, Eq. (24) is further formulated as follows:

\[{\tau }_{f,n} = \left\{  {\begin{array}{l} {\mathbf{K}}_{f,n}^{ + }{\mathbf{Y}}_{f,n},{\dot{q}}_{n} > 0 \\  0,{\dot{q}}_{n} = 0 \\  {\mathbf{K}}_{f,n}^{ - }{\mathbf{Y}}_{f,n},{\dot{q}}_{n} < 0 \end{array}n = 1,2,3}\right.  \tag{25}\]

\[\text{with}{\mathbf{K}}_{f,n}^{ + } = \left\lbrack  {{k}_{{fn},1}^{ + },{k}_{{fn},2}^{ + },{k}_{{fn},3}^{ + }}\right\rbrack  ,{\mathbf{K}}_{f,n}^{ - } = \left\lbrack  {{k}_{{fn},1}^{ - },{k}_{{fn},2}^{ - },{k}_{{fn},3}^{ - }}\right\rbrack  \text{)}\]

where \( {\mathbf{K}}_{f,n}^{ + } \) and \( {\mathbf{K}}_{f,n}^{ - } \) are different coefficient matrices depending on the motion direction of the \( n \) -th joint. In the actual robot movement process, the robot end will generally receive the external load force \( \left( {\mathbf{F}}^{6 \times  1}\right) \) . Thus, according to Ref. [71],joint toques \( \left( {\mathbf{\tau }}_{\text{ext }}\right) \) induced by the external force \( \mathbf{F} \) are calculated by:

\[{\mathbf{\tau }}_{\text{ext }} = {\mathbf{J}}^{T}\mathbf{F} \tag{26}\]

where \( \mathbf{J} \) is the Jacobian matrix of the robot with \( N \) rotary joints expressed as:

\[\mathbf{J}\left( \mathbf{q}\right)  = \left\lbrack  \begin{matrix} {\mathbf{z}}_{1} \times  {}^{1}{\mathbf{p}}_{N}^{0} & {\mathbf{z}}_{2} \times  {}^{2}{\mathbf{p}}_{N}^{0} & \cdots & {\mathbf{z}}_{N} \times  {}^{N}{\mathbf{p}}_{N}^{0} \\  {\mathbf{z}}_{1} & {\mathbf{z}}_{2} & \cdots & {\mathbf{z}}_{N} \end{matrix}\right\rbrack   \tag{27}\]

In Eq. (27), \( {\mathbf{z}}_{n}\left( {n = 1,2,\ldots ,N}\right) \) is a unit direction vector describing the axis direction of the \( n \) -th joint in the RBCS,and \( {}^{n}{\mathbf{p}}_{N}^{0}\left( {n = 1,2,\ldots ,N}\right) \) is the representation of the position vector of the coordinate origin of the robot end relative to the coordinate system described in the RBCS. Therefore, the dynamics model of the \( n \) -th joint is established.

### 3.2. Calculation of the pose-dependent joint dynamic constraints

The driving torques of joints should be restricted to prevent the overload of motors. The acceleration/deceleration capability of the joint motion is pose-dependent due to the strong correlation between gravity and equivalent inertia with robot posture. Therefore, the joint dynamic constraints are pose-dependent.

According to Eq. (21),the motion drive torque of the \( n \) -th joint \( \left( {\tau }_{m,n}\right) \) is expressed as:

\[{\tau }_{m,n} = {M}_{n}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right)  - {G}_{n}\left( \mathbf{q}\right)  - {\tau }_{{ext},n} + {\tau }_{f,n}\left( \dot{\mathbf{q}}\right)  \tag{28}\]

\[n = 1,2\ldots N\]

From the analysis shown in Section 3.1,the terms \( {G}_{n}\left( \mathbf{q}\right) \) and \( {\tau }_{{ext},n} \) are only related to the joint posture of the robot and are respectively calculated by Eqs. (22) and (26). As for inertial torques \( {M}_{n}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right) \) ,the equivalent rotating inertia \( \left( {{I}_{n}\left( \mathbf{q}\right) }\right) \) is affected independently by the robot poses. The term \( \left( {\mathop{\sum }\limits_{{r = 1}}^{n}{\ddot{q}}_{r}{\mathbf{z}}_{r}}\right)  \cdot  {\mathbf{z}}_{n} \) is affected by joint accelerations and can be rewritten as

\[\left( {\mathop{\sum }\limits_{{r = 1}}^{n}{\ddot{q}}_{r}{\mathbf{z}}_{r}}\right)  \cdot  {\mathbf{z}}_{n} = \mathop{\sum }\limits_{{r = 1}}^{n}{\alpha }_{r}{\ddot{q}}_{r}\left( {{\alpha }_{r} = {\mathbf{z}}_{r} \cdot  {\mathbf{z}}_{n}}\right)  \tag{29}\]

where the coefficient \( \left( {{\alpha }_{r} = {\mathbf{z}}_{r} \cdot  {\mathbf{z}}_{n}}\right) \) is affected independently by robot poses. In ensuring motion efficiency, the joint acceleration usually reaches its maximum value during the acceleration phase.

Thus,the maximum value of \( {M}_{n}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right) \) is estimated as

\[\left| {{M}_{n}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right) }\right|  = \left| {{I}_{n}\left( \mathbf{q}\right)  \cdot  \left( {\mathop{\sum }\limits_{{r = 1}}^{n}{\ddot{q}}_{r}{\mathbf{z}}_{r}}\right)  \cdot  {\mathbf{z}}_{n}}\right|  \tag{30}\]

\[ = {I}_{n}\left( \mathbf{q}\right) \left| {\mathop{\sum }\limits_{{r = 1}}^{n}{\alpha }_{r}{\ddot{q}}_{r}}\right|  \leq  {I}_{n}\left( \mathbf{q}\right) \mathop{\sum }\limits_{{r = 1}}^{n}\left| {\alpha }_{r}\right| {\ddot{q}}_{r,\max }\]

where \( {\ddot{q}}_{r,\max } \) is the maximum value of the \( r \) -th joint’s acceleration.

Meanwhile,the motion drive torque of the \( n \) -th joint \( {\tau }_{m,n} \) should be restricted within the following limitation:

\[ - {\tau }_{m,n,\max } \leq  {\tau }_{m,n} \leq  {\tau }_{m,n,\max } \tag{31}\]

\[\text{with}{\tau }_{m,n} = {M}_{n}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right)  - {G}_{n}\left( \mathbf{q}\right)  - {\tau }_{{ext},n} + {\tau }_{f,n}\left( \dot{\mathbf{q}}\right) \]

where \( {\tau }_{m,n,\max } \) is the motion drive torque limit of the \( n \) -th joint. Based on the above analysis,the gravity term \( {G}_{n}\left( \mathbf{q}\right) \) ,the inertia torque \( {M}_{n}\left( {\mathbf{q},\ddot{\mathbf{q}}}\right) \) , and the external payload torque \( {\tau }_{{ext},n} \) can be calculated pose-dependently. Therefore,the limit of the motion drive torque \( {\tau }_{m,n} \) is influenced by the friction term \( {\tau }_{f,n}\left( \dot{\mathbf{q}}\right) \) which is related to joint speed. Hence, the range of friction torque is obtained according to Eqs. (28)- (31).

\[{\tau }_{f,n,\min } \leq  {\tau }_{f,n} \leq  {\tau }_{f,n,\max }\]

\[\left\{  \begin{array}{l} {\tau }_{f,n,\max } = {\tau }_{m,n,\max } + {G}_{n}\left( \mathbf{q}\right)  + {\tau }_{{ext},n} - {I}_{n}\left( \mathbf{q}\right) \mathop{\sum }\limits_{{r = 1}}^{n}\left| {\alpha }_{r}\right| {\ddot{q}}_{r,\max } \\  {\tau }_{f,n,\min } =  - {\tau }_{m,n,\max } + {G}_{n}\left( \mathbf{q}\right)  + {\tau }_{{ext},n} + {I}_{n}\left( \mathbf{q}\right) \mathop{\sum }\limits_{{r = 1}}^{n}\left| {\alpha }_{r}\right| {\ddot{q}}_{r,\max } \end{array}\right.  \tag{32}\]

According to Eq. (25), the friction torque is a monotonic function of joint speed. The range of the joint velocity constraint can be obtained by combining Eqs. (24), (25), and (32) and solving the following inequality:

\[\left\{  \begin{array}{l} {\tau }_{f,n,\min } \leq  {\tau }_{f,n} \leq  {\tau }_{f,n,\max } \\  {\tau }_{f,n} = {k}_{{fn},1}{\dot{q}}_{n} + {k}_{{fn},2}\sqrt{\left| {\dot{q}}_{n}\right| } + {k}_{{fn},3} \end{array}\right.  \tag{33}\]

Furthermore,the joint velocity constraints \( \left( {\dot{q}}_{n,\max }\right) \) are obtained under the given robot pose by solving Eq. (33). The joint velocity constraints are pose-dependent because of the postural independence of \( {\tau }_{f,n,\max } \) and \( {\tau }_{f,n,\min } \) .

### 3.3. Hybrid constraints of tangential and joint dynamic characteristics

The interpolation of the tool paths is accomplished in the WCS. In Section 2, only tangential dynamic constraints are considered to design the time constants of FIR filters, and the joint dynamic constraints are ignored. Therefore, only hybrid constraints of tangential and joint dynamic characteristics are analyzed in this section.

The relationship between tangential and joint dynamic characteristics is obtained by the following:

\[\dot{\mathbf{q}} = \frac{d\mathbf{q}}{dt} = \frac{d\mathbf{q}}{ds}\frac{ds}{dt} = {\mathbf{q}}_{s}\dot{s} \tag{34}\]

\[\ddot{\mathbf{q}} = \frac{d\dot{\mathbf{q}}}{dt} = \frac{d\left( {{\mathbf{q}}_{s}\dot{s}}\right) }{dt} = {\mathbf{q}}_{ss}{\dot{s}}^{2} + {\mathbf{q}}_{s}\ddot{s}\]

where \( \dot{\mathbf{q}} \) and \( \ddot{\mathbf{q}} \) mean the joint velocity and acceleration; \( s,\dot{s} \) ,and \( \ddot{s} \) represent the displacement, velocity, and acceleration of the tangential path,respectively; and \( {\mathbf{q}}_{s} \) and \( {\mathbf{q}}_{ss} \) are the first- and second-order derivatives of joint angular displacements related to the tangential displacement,respectively. Given that \( {\mathbf{q}}_{s} \) and \( {\mathbf{q}}_{ss} \) are only relevant to the geometric information, they can be calculated along the linear segments before interpolation. Therefore, before applying the interpolation algorithm, the joint dynamic constraints are calculated in this section.

The joints’ velocity \( \left( {\dot{\mathbf{q}} = {\left\lbrack  {\dot{q}}_{n}\right\rbrack  }^{T}}\right) \) should satisfy the following inequality:

\[ - {\dot{q}}_{n,\max } \leq  {\dot{q}}_{n} \leq  {\dot{q}}_{n,\max },n = 1,2,\ldots ,N \tag{35}\]

<!-- Meanless: 9-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

where \( {\dot{q}}_{n,\max } \) is the joint velocity constraint obtained by Eqs. (33) and (32) in Section 3.2.

According to Eqs. (34) and (35), the following inequality is obtained:

\[\left| {\dot{q}}_{n}\right|  = \left| {{q}_{s}^{n}\dot{s}}\right|  \leq  \left| {q}_{s}^{n}\right| \dot{s} \leq  \left| {q}_{s}^{n}\right| {\dot{s}}_{\max } \leq  {\dot{q}}_{n,\max },n = 1,2,\ldots ,N \tag{36}\]

According to Eq. (4),the velocity constraint \( {\dot{s}}_{\max } \) of the tangential path is calculated as \( s/{T}_{1} \) ,while \( s \) is the distance of the linear segment. Therefore, \( {T}_{1} \) should satisfy the following:

\[{T}_{1} \geq  \frac{s\left| {q}_{s}^{n}\right| }{{\dot{q}}_{n,\max }},n = 1,2,\ldots ,N \tag{37}\]

where \( s,{q}_{s}^{n} \) ,and \( {\dot{q}}_{n,\max } \) are the known values. The calculation of \( {q}_{s}^{n} \) is achieved by the following numerical discrete equation:

\[{q}_{s}^{n} = \frac{d{q}^{n}}{ds} = \frac{{q}^{n}\left( {s + {ds}}\right)  - {q}^{n}\left( s\right) }{ds},n = 1,2,\ldots ,N \tag{38}\]

The points along the linear segment are selected successively to perform the calculation and calculate the maximum value of \( {q}_{s}^{n} \) on each segment. The maximum value is chosen from the calculation results approximately. Therefore, to satisfy the hybrid constraints of tangential and joint velocity characteristics,the time constant \( {T}_{1} \) is obtained as follows.

\[{T}_{1} = \max \left\{  {\frac{s\left| {q}_{s}^{n}\right| }{{\dot{q}}_{n,\max }},\frac{s}{{\dot{s}}_{\max }}}\right\}  ,n = 1,2,\ldots ,N \tag{39}\]

The joint acceleration should satisfy the following constraints:

\[ - {\ddot{q}}_{n,\max } \leq  {\ddot{q}}_{n} \leq  {\ddot{q}}_{n,\max },n = 1,2,\ldots ,N \tag{40}\]

where \( {\ddot{q}}_{n,\max } \) is the acceleration constraint of the \( n \) -th joint.

Combining Eqs. (34) and (40):

\[\left| {\ddot{q}}_{n}\right|  = \left| {{q}_{ss}^{n}{\dot{s}}^{2} + {q}_{s}^{n}\ddot{s}}\right|  \leq  \left| {q}_{ss}^{n}\right| {\dot{s}}^{2} + \left| {q}_{s}^{n}\right| \left| \ddot{s}\right|  \tag{41}\]

\[ \leq  \left| {q}_{ss}^{n}\right| {\dot{s}}_{\max }^{2} + \left| {q}_{s}^{n}\right| {\ddot{s}}_{\max } \leq  {\ddot{q}}_{n,\max }\]

where \( {\ddot{s}}_{\max } \) is calculated as \( {\dot{s}}_{\max }/{T}_{2} \) . Therefore, \( {T}_{2} \) should satisfy the following equation.

\[{T}_{2} \geq  \frac{\left| {q}_{s}^{n}\right| {\dot{s}}_{\max }}{{\ddot{q}}_{n,\max } - \left| {q}_{ss}^{n}\right| {\dot{s}}_{\max }^{2}},n = 1,2,\ldots ,N \tag{42}\]

In addition, \( {q}_{ss}^{n} \) is also obtained numerically by the following equation:

\[{q}_{ss}^{n} = \frac{d{q}_{s}^{n}}{ds} = \frac{{q}_{s}^{n}\left( {s + {ds}}\right)  - {q}_{s}^{n}\left( s\right) }{ds},n = 1,2,\ldots ,N \tag{43}\]

The acceleration or deceleration stage appears near the endpoint (or corner point) of the linear segment during the motion. Thus, the calculation of \( {q}_{ss}^{n} \) is only considered near the corner to adjust \( {T}_{2} \) . To satisfy the hybrid constraints of tangential and joint acceleration characteristics,the time constant \( {T}_{2} \) is given as:

\[{T}_{2} = \max \left\{  {\frac{\left| {q}_{s}^{n}\right| {\dot{s}}_{\max }}{{\ddot{q}}_{n,\max } - \left| {q}_{ss}^{n}\right| {\dot{s}}_{\max }^{2}},\frac{{\dot{s}}_{\max }}{{\ddot{s}}_{\max }}}\right\}  ,n = 1,2,\ldots ,N \tag{44}\]

Here, Eqs. (37) and (42) are considered before applying the algorithm presented in Section 2. In this way, the hybrid constraints of tangential and joint dynamic characteristics are guaranteed with the consideration of the pose-dependent dynamics of robot manipulators.

## 4. Robotic tool path smoothing algorithm with pose-dependent dynamic constraints

In this section, the robotic path smoothing and interpolation with the consideration of pose-dependent dynamic constraints are introduced. The generation of position and orientation for the robot end poses is illustrated first by designing the FIR filter parameters. Then, the synchronization of the position and orientation are achieved by adjusting the time constants of the FIR filters and overlapping times. The entire execution flow of the proposed algorithm is finally present.

### 4.1. Generation of position and orientation for the robot end tool poses

The trajectory of the robot end tool poses is divided into position and orientation trajectories. In general, the robot end tool pose is described by the homogeneous transformation matrix \( \mathbf{T} \) :

\[\mathbf{T} = \left\lbrack  \begin{array}{ll} \mathbf{R} & \mathbf{p} \\  0 & 1 \end{array}\right\rbrack   = \left\lbrack  \begin{matrix} {r}_{11} & {r}_{12} & {r}_{13} & {p}_{x} \\  {r}_{21} & {r}_{22} & {r}_{23} & {p}_{y} \\  {r}_{31} & {r}_{32} & {r}_{33} & {p}_{z} \\  0 & 0 & 0 & 1 \end{matrix}\right\rbrack   \tag{45}\]

\[\text{,with}\mathbf{R} = \left\lbrack  \begin{array}{lll} {r}_{11} & {r}_{12} & {r}_{13} \\  {r}_{21} & {r}_{22} & {r}_{23} \\  {r}_{31} & {r}_{32} & {r}_{33} \end{array}\right\rbrack  ,\mathbf{p} = \left\lbrack  \begin{array}{l} {p}_{x} \\  {p}_{y} \\  {p}_{z} \end{array}\right\rbrack  \]

where \( \mathbf{R} \) is the rotation matrix that describes the orientation of the robot end tool,and \( \mathbf{p} \) describes the position vector of the robot end tool tip.

The distances between two adjacent robot end tool tip positions are given as follows:

\[{s}_{p,k} = \begin{Vmatrix}{{\mathbf{p}}_{k + 1} - {\mathbf{p}}_{k}}\end{Vmatrix},k = 1,2\ldots  \tag{46}\]

The time constants of the positions \( \left( {T}_{p,i}^{k,1}\right. \) and \( \left. {T}_{p,i}^{k,2}\right) \) are calculated based on Sections 2.1 and 2.2. The overlapping time \( {T}_{v,p}^{k} \) is calculated by the corner error tolerance \( \left( {\varepsilon }_{p,\text{ tolerance }}\right) \) according to Eq. (19).

In general, the orientation of the robot end tool is described by Euler angles. The transformation from the ZYX Euler angles to the rotation matrix \( \mathbf{R} \) is as follows.

\[\mathbf{R} = {\operatorname{Rot}}_{Z}\left( \alpha \right) {\operatorname{Rot}}_{Y}\left( \beta \right) {\operatorname{Rot}}_{X}\left( \gamma \right) \]

\[ = \left\lbrack  \begin{matrix} \cos \alpha &  - \sin \alpha & 0 \\  \sin \alpha & \cos \alpha & 0 \\  0 & 0 & 1 \end{matrix}\right\rbrack  \left\lbrack  \begin{matrix} \cos \beta & 0 & \sin \beta \\  \sin \beta & 1 & 0 \\  0 & 0 & \cos \beta  \end{matrix}\right\rbrack  \left\lbrack  \begin{matrix} 1 & 0 & 0 \\  0 & \cos \gamma &  - \sin \gamma \\  0 & \sin \gamma & \cos \gamma  \end{matrix}\right\rbrack   \tag{47}\]

The orientation of the robot end tool is described as \( \mathbf{O} = {\left\lbrack  {O}_{i},{O}_{j},{O}_{k}\right\rbrack  }^{T} \) in the spherical coordinate system. Next, we calculate the tool orientation \( \mathbf{O} \) in the spherical coordinate system using the rotary transfer matrix \( \mathbf{R} \) and the unit direction vector \( {\mathbf{r}}_{ot} \) of the tool orientation related to the tool frame.

\[\mathbf{O} = \mathbf{R} \cdot  {\mathbf{r}}_{ot}\text{,with}{\mathbf{r}}_{ot} = {\left\lbrack  0,0,1\right\rbrack  }^{T} \tag{48}\]

According to Eqs. (45) and (47), three components of the orientation vector \( \mathbf{O} \) are calculated as follows.

\[\left\{  \begin{array}{l} {O}_{i} = \sin \alpha \sin \gamma  + \cos \alpha \sin \beta \cos \gamma \\  {O}_{j} =  - \cos \alpha \sin \gamma  + \sin \alpha \sin \beta \cos \gamma \\  {O}_{k} = \cos \beta \cos \gamma  \end{array}\right.  \tag{49}\]

The ZYX Euler angles \( \left( {\alpha ,\beta ,\gamma }\right) \) are calculated by inverse calculation using parameters in the matrix \( \mathbf{R} \) .

<!-- Meanless: 10-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

\[\left\{  \begin{array}{l} \beta  = \operatorname{atan}2\left( {-{r}_{31},\sqrt{{r}_{11}^{2} + {r}_{21}^{2}}}\right) \\  \alpha  = \operatorname{atan}2\left( {{r}_{21}/\cos \beta ,{r}_{11}/\cos \beta }\right) \text{ if }\beta  \neq   \pm  {90}^{ \circ  } \\  \gamma  = \operatorname{atan}2\left( {{r}_{32}/\cos \beta ,{r}_{33}/\cos \beta }\right)  \end{array}\right. \]

\[\left\{  \begin{array}{l} \beta  = {90}^{ \circ  } \\  \alpha  = 0\text{ if }\beta  = {90}^{ \circ  } \\  \gamma  = \operatorname{atan}2\left( {{r}_{12},{r}_{22}}\right)  \end{array}\right.  \tag{50}\]

\[\left\{  \begin{array}{l} \beta  =  - {90}^{ \circ  } \\  \alpha  = 0\text{ if }\beta  =  - {90}^{ \circ  } \\  \gamma  = \operatorname{atan}2\left( {-{r}_{12},{r}_{22}}\right)  \end{array}\right. \]

\[{T}_{v,o}^{k} = \left\{  \begin{array}{l} \text{ if }0 \leq  {T}_{v,o}^{k} \leq  2{T}_{o,3}^{k,2} \\  \sqrt[3]{\frac{{48}{T}_{o,2}^{k,2}{T}_{o,3}^{k,2}{\varepsilon }_{o,\text{ tolerance }}}{\sqrt{{\left( {w}_{k,\max }\right) }^{2} + {\left( {w}_{k + 1,\max }\right) }^{2} - {w}_{k,\max }{w}_{k + 1,\max }\cos \left( {\theta }_{o}^{k}\right) }}}, \\  \text{ if }2{T}_{o,3}^{k,2} \leq  {T}_{v,o}^{k,2} + {T}_{o,2}^{k,2} + {T}_{o,2}^{k,2} \\  {T}_{o,3}^{k,2} + \sqrt{\frac{8{T}_{o,4}^{k,2}{T}_{o,3}^{k,2} + {\left( {w}_{k,\max }\right) }^{2} - {w}_{k,\max }{w}_{k + 1,\max }\cos \left( {\theta }_{o}^{k}\right) }{\sqrt{{\left( {w}_{k,\max }\right) }^{2} + {\left( {w}_{k,\max }\right) }^{2} - {w}_{k,\max }{w}_{k + 1,\max }\cos \left( {\theta }_{o}^{k}\right) }}} = \frac{{T}_{o,2}^{k,2}}{3} \end{array}\right. \]

(52)

<!-- Media -->

Algorithm 1: Proposed path smoothing algorithm

---

Input: discrect linear path \( \left\lbrack  {{\mathbf{P}}_{k},{\mathbf{O}}_{k}}\right\rbrack  ,k = 1,2,\ldots ,p + 1 \)
Output: smoothed joint motion commands
Initialization \( k = 1 \) ;
while \( k \leq  p + 1 \) do
		Inverse transformation of robot kinematics : \( \left\lbrack  {{\mathbf{P}}_{k},{\mathbf{O}}_{k}}\right\rbrack   \Rightarrow  {\mathbf{q}}_{k} = {\left\lbrack  {q}_{1},{q}_{2},\ldots ,{q}_{6}\right\rbrack  }^{\mathrm{T}} \) ;
		Establish the dynamics model of robotic joints : \( \mathbf{M}\left( \ddot{\mathbf{q}}\right)  = \mathbf{G}\left( \mathbf{q}\right)  + {\mathbf{\tau }}_{\mathbf{f}}\left( {\mathbf{q},\dot{\mathbf{q}}}\right)  + {\mathbf{\tau }}_{m} + {\mathbf{\tau }}_{\text{ext }} \) ;
		Calculate pose-dependent dynamic constraints by Eqs.(28)-(33) : \( {\tau }_{m} \leq  {\tau }_{m,{lim}} \Rightarrow  {\dot{q}}_{lim} \) ;
		Hybrid constraints of tangential and joint constraisntsby Eqs.(34),(36),(37),(41) and (42):
		\( {\dot{q}}_{lim},{\ddot{q}}_{lim} \Rightarrow  {v}_{k,{lim}},{a}_{k,{lim}}; \)
		\( k = k + 1 \) ；
end
Initialization \( k = 1,{T}_{d} = 0,{s}_{p}\left( t\right)  = 0,{s}_{o}\left( t\right)  = 0 \) ;
while \( k \leq  p \) do
		Tool tip postion interpolation in the WCS of the k-th linear segemnt by Eqs.(4)-(13) and
		(46): \( {s}_{p}^{k}\left( t\right)  = {s}_{p}^{k,1}\left( t\right)  \cdot  \left( {1 - {\mu }_{p}^{k}\left( t\right) }\right)  + {s}_{p}^{k,2}\left( {t - {\delta }_{p}^{k}}\right)  \cdot  {\mu }_{p}^{k}\left( t\right) \) ;
	Tool orientation interpolation in the WCS of the k-th linear segemnt by Eqs.(4)-(13) and
		(51): \( {s}_{o}^{k}\left( t\right)  = {s}_{o}^{k,1}\left( t\right)  \cdot  \left( {1 - {\mu }_{o}^{k}\left( t\right) }\right)  + {s}_{o}^{k,2}\left( {t - {\delta }_{o}^{k}}\right)  \cdot  {\mu }_{o}^{k}\left( t\right) \) ;
		Synchronize the orientation path and the position path by Eq.(54);
		\( {s}_{p}\left( t\right)  = {s}_{p}\left( t\right)  + {s}_{p}^{k}\left( {t - {T}_{d}}\right) ,{s}_{o}\left( t\right)  = {s}_{o}\left( t\right)  + {s}_{o}^{k}\left( {t - {T}_{d}}\right) ; \)
		Calculate overlapping time \( {T}_{v,p}^{k} \) and \( {T}_{o,p}^{k} \) by Eqs.(54) and (55);
		\( {T}_{v}^{k} = \min \left( {{T}_{v,p}^{k},{T}_{o,p}^{k}}\right) ; \)
		\( {T}_{d} = {T}_{d} + {T}_{v}^{k}; \)
		\( k = k + 1 \) ;
	end
Generate the smoothed robot end commands by the interpolation cycle;
Generate smoothed joint motion commands by inverse transformation of robot kinematics.

---

Fig. 8. The framework of the proposed algorithm flow.

<!-- Media -->

In the same way,the distance \( {s}_{o,k} \) of the adjacent tool orientation is calculated using the following equation:

\[{s}_{o,k} = \begin{Vmatrix}{{\mathbf{O}}_{k + 1} \cdot  {\mathbf{O}}_{k}}\end{Vmatrix},k = 1,2\ldots  \tag{51}\]

The orientation trajectory is interpolated in a similar way using FIR filters by referring to the interpolation method of the position trajectory. The time constants \( \left( {T}_{o,i}^{k,1}\right. \) and \( \left. {T}_{o,i}^{k,2}\right) \) are calculated according to the dynamic constraints of the tool orientation \( \left( {{\omega }_{k,\max },{\omega }_{k,\max }^{\prime },{\omega }_{\max }^{\prime \prime }}\right) \) and the distance \( \left( {s}_{o,k}\right) \) of the adjacent tool orientation based on Sections 2.1 and 2.2. In the same way,the overlapping time \( \left( {T}_{v,o}^{k}\right) \) is calculated by Eq. (52):

where \( \cos \left( {\theta }_{o}^{k}\right)  = \frac{\left( {{\mathbf{O}}_{k} \times  {\mathbf{O}}_{k + 1}}\right)  \cdot  \left( {{\mathbf{O}}_{k + 1} \times  {\mathbf{O}}_{k + 2}}\right) }{\begin{Vmatrix}{{\mathbf{O}}_{k} \times  {\mathbf{O}}_{k + 1}}\end{Vmatrix}\begin{Vmatrix}{{\mathbf{O}}_{k + 1} \times  {\mathbf{O}}_{k + 2}}\end{Vmatrix}} \) . In addition, \( {\mathbf{O}}_{k},{\mathbf{O}}_{k + 1} \) ,and \( {\mathbf{O}}_{k + 2} \) are the tool orientation vectors of adjacent robot poses. \( {\varepsilon }_{o,\text{tolerance }} \) is the error tolerance of the tool orientation path.

### 4.2. Synchronization of the position and orientation motions

The time constants should be adjusted to realize the synchronized motion because the poses of the robot end tool are interpolated separately. After being interpolated,the motion time of the \( k \) -th position path \( \left( {T}_{p,\text{ total }}^{k}\right) \) and the motion time of the \( k \) -th orientation path \( \left( {T}_{o,\text{ total }}^{k}\right) \) are obtained based on the time constants.

<!-- Meanless: 11-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

\[\left\{  \begin{array}{l} {T}_{p,\text{ total }}^{k} = \frac{\mathop{\sum }\limits_{{i = 1}}^{3}{T}_{p,i}^{k,1} + \mathop{\sum }\limits_{{i = 1}}^{3}{T}_{p,i}^{k,2}}{2} \\  {T}_{\text{total }}^{k} = \frac{\mathop{\sum }\limits_{{i = 1}}^{3}{T}_{o,i}^{k,1} + \mathop{\sum }\limits_{{i = 1}}^{3}{T}_{o,i}^{k,2}}{2} \end{array}\right.  \tag{53}\]

<!-- Media -->

<!-- figureText: \( {q}_{4} \) Data record Motion commands UR10 robo Payload -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_11.jpg?x=178&y=158&w=1388&h=446&r=0"/>

Fig. 9. Experiment platform based on the UR10 robot.

<!-- Media -->

The synchronization is realized by adjusting \( {T}_{p,i}^{k,1},{T}_{p,i}^{k,2} \) or \( {T}_{o,i}^{k,1},{T}_{o,i}^{k,2} \) .

\[\left\{  \begin{array}{l} {T}_{p,1}^{k,{1}^{\prime }} = {T}_{p,1}^{k,{2}^{\prime }} = \left( {{T}_{o,\text{ total }}^{k} - {T}_{p,\text{ total }}^{k}}\right)  + {T}_{p,1}^{k,1},\text{ if }{T}_{o,\text{ total }}^{k} > {T}_{p,\text{ total }}^{k} \\  {T}_{o,1}^{k,{1}^{\prime }} = {T}_{o,1}^{k,{2}^{\prime }} = \left( {{T}_{p,\text{ total }}^{k} - {T}_{0,\text{ total }}^{k}}\right)  + {T}_{o,1}^{k,1},\text{ if }{T}_{p,\text{ total }}^{k} > {T}_{o,\text{ total }}^{k} \end{array}\right.  \tag{54}\]

The overlapping time \( {T}_{v}^{k} \) at each corner is chosen as

\[{T}_{v}^{k} = \min \left\{  {{T}_{v,p}^{k},{T}_{v,o}^{k}}\right\}   \tag{55}\]

The maximum tangential velocity or angular velocity is changed after the synchronization. Taking the situation \( {T}_{o,\text{ total }}^{k} > {T}_{p,\text{ total }}^{k} \) for analysis,the maximum velocity \( {v}_{k,\max } \) is changed to \( {v}_{k,\max }^{\prime } \) .

\[{v}_{k,\max } = \frac{{s}_{p}^{k}}{{T}_{p,1}^{k,{1}^{\prime }}} = \frac{{T}_{p,1}^{k,1}}{{T}_{p,1}^{k,{1}^{\prime }}}{v}_{k,\max } \tag{56}\]

As can be seen, the velocity constraints are also guaranteed due to the fact that \( {T}_{p,1}^{k,{1}^{\prime }} > {T}_{p,1}^{k,1},{v}_{k,\max } \) is decreased to \( {v}_{k,\max } \) .

### 4.3. Framework of the proposed algorithm flow

The entire algorithm flow of the proposed method is shown in Fig. 8. The specific steps of the flow are described as follows.

Step 1: The input path commands \( \left( \left\lbrack  {{\mathbf{P}}_{k},{\mathbf{O}}_{k}}\right\rbrack  \right) \) are transformed to joint commands \( \mathbf{q} = {\left\lbrack  {q}_{1},{q}_{2},\ldots ,{q}_{6}\right\rbrack  }^{T} \) by inverse kinematics for dynamics analysis.

Step 2: The simplified dynamics model is established first for the purpose of calculating the joint dynamic constraints. Based on the dynamics model, the pose-dependent joint constraints are calculated by Eqs. (28) to (33) in Section 3.2.

Step 3: The tangential dynamic constraints \( \left( {{\dot{s}}_{\text{lim }},{\ddot{s}}_{\text{lim }}}\right) \) of each segment are adjusted to guarantee the joint dynamic constraints \( \left( {{\dot{q}}_{\mathrm{{lim}}},{\ddot{q}}_{\mathrm{{lim}}}}\right) \) . Firstly, the relationship between tangential and joint dynamic characteristics is mapped by Eq.(34). Then, the tangential dynamic constraints should meet Eqs. (36) and (41) under different robot poses to guarantee the joint dynamic constraints. Here, the time constants should be determined by Eqs. (37) and (42) before performing path interpolation because the tangential dynamic performance is determined by the time constants of FIR filters.

Step 4: The tool path interpolation is divided into two components: the tool tip position interpolation and the tool orientation interpolation in the WCS. First, to generate the interpolation path according to Section 2.2, time constants are designed based on dynamic constraints. Then, the overlapping time is designed to meet the corner error tolerance using Eqs. (18) and (52) to ensure the smoothness of corners between adjacent segments.

Step 5: The time constants of the FIR filters and overlapping times must be adjusted to achieve synchronization between tool tip position interpolation and tool orientation interpolation. Time constants of FIR filters in tool tip position interpolation and tool orientation interpolation are adjusted using Eq. (54), and overlapping times are adjusted by Eq. (55).

By implementing the above steps, the smoothed robotic path commands that meet pose-dependent dynamic constraints are obtained.

## 5. Experiment

In this section, experiments are carried out to verify the proposed path smoothing algorithm. The UR10 robot is used in experiments, as shown in Fig. 9. The robot motion control and the data acquisition are implemented by a personal computer using the Python package, which is called "ur_rtde". The kinematic information and current information of the robot joints are collected in a sampling period of \( 8\mathrm{\;{ms}} \) . The torque values of its joints are estimated by collecting and analyzing the currents of the robot joints, because there are no integrated force sensors in the joints of the UR10 robot. The joint torques \( \tau \) is directly proportional to the joint currents \( \mathbf{i} \) [72]; therefore,the joint torques are quantified through joint currents as \( \mathbf{\tau } = \mathbf{{Ki}} \) . Here, \( \mathbf{K} \) is the joint drive gain vector of the UR10 robot,whose value can be referred to Ref. [72] as \( \mathbf{K} = \) \( {\left\lbrack  {14.7336},{14.3300},{11.5476},{11.2487},{11.5000},{11.5000}\right\rbrack  }^{T} \) . The unit of \( \mathrm{K} \) is \( \mathrm{N} \bullet  \mathrm{m}/\mathrm{A} \) . Joints \( 3 - 6 \) are treated as a single unit due to the small torque changes in joints 4-6 with respect to the tool pose variances of the UR10 robot and their minimal impact on joints 1-3 . Detailed descriptions of the modeling process for gravitational torques and equivalent moment of inertia is given in Appendix A and B respectively. The payload is an axisymmetric object weighing \( {8.6}\mathrm{\;{kg}} \) ,which is almost equal to the maximum carrying capacity(10kg)of the UR10 robot.

### 5.1. Experiment result of joint dynamics model establishment

The gravity part has no effect on the torque of joint 1 due to the robot horizontal installation structure shown in Fig. 9. To identify the gravity parameters, the rotary speeds of joint 2 and joint 3 are set to be constant as \( {0.08}\mathrm{{rad}}/\mathrm{s} \) ,while joint torques \( \left( {{\tau }_{m,2}\left( {t}_{i}\right) \text{and}{\tau }_{m,3}\left( {t}_{i}\right) }\right) \) are simultaneously recorded. The frictional torque of the joint is regarded as a constant value, and there is no influence of the inertia torque because the joints are in uniform motion. Therefore, combining Appendix A and Eq. (28), the joint torque is expressed as:

<!-- Meanless: 12-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

\[{\tau }_{m,3} =  - {G}_{3}\left( \mathbf{q}\right)  + {f}_{3} = \left\lbrack  {-{\mathbf{Y}}_{G3},1}\right\rbrack  \left\lbrack  \begin{array}{l} {\mathbf{K}}_{G3} \\  {f}_{3} \end{array}\right\rbrack   = {\mathbf{Y}}_{G3}^{\prime }{\mathbf{K}}_{G3}^{\prime }\]

\[{\tau }_{m,2} =  - {G}_{3}\left( \mathbf{q}\right)  - {G}_{2}\left( \mathbf{q}\right)  + {f}_{2} =  - {\mathbf{Y}}_{G3}{\mathbf{K}}_{G3} + \left\lbrack  {-{\mathbf{Y}}_{G2},1}\right\rbrack  \left\lbrack  \begin{array}{l} {\mathbf{K}}_{G2} \\  {f}_{2} \end{array}\right\rbrack   \tag{57}\]

\[ =  - {\mathbf{Y}}_{G3}{\mathbf{K}}_{G3} + {\mathbf{Y}}_{G2}^{\prime }{\mathbf{K}}_{G2}^{\prime }\]

\[\text{with}\left\{  \begin{array}{l} {\mathbf{Y}}_{G3}^{\prime } = \left\lbrack  {-{\mathbf{Y}}_{G3},1}\right\rbrack  ,{\mathbf{K}}_{G3}^{\prime } = {\left\lbrack  {\mathbf{K}}_{G3},{f}_{3}\right\rbrack  }^{T} \\  {\mathbf{Y}}_{G2}^{\prime } = \left\lbrack  {-{\mathbf{Y}}_{G2},1}\right\rbrack  ,{\mathbf{K}}_{G2}^{\prime } = {\left\lbrack  {\mathbf{K}}_{G2},{f}_{2}\right\rbrack  }^{T} \end{array}\right. \]

<!-- Media -->

Table 1

Experiment data of the joint friction identification.

<table><tr><td colspan="4">Joint 1: \( {\dot{q}}_{1}\left( {\mathrm{{rad}}/\mathrm{s}}\right) ,{\tau }_{f,1}\left( {\mathrm{\;N} \cdot  \mathrm{m}}\right) \)</td><td colspan="4">Joint 2: \( {\dot{q}}_{2}\left( {\mathrm{{rad}}/\mathrm{s}}\right) ,{\tau }_{f,2}\left( {\mathrm{\;N} \cdot  \mathrm{m}}\right) \)</td><td colspan="4">Joint 3: \( {\dot{q}}_{3}\left( {\mathrm{{rad}}/\mathrm{s}}\right) ,{\tau }_{f,3}\left( {\mathrm{\;N} \cdot  \mathrm{m}}\right) \)</td></tr><tr><td>\( {\dot{q}}_{1} > 0 \)</td><td>\( {\tau }_{f,1} > 0 \)</td><td>\( {\dot{q}}_{1} < 0 \)</td><td>\( {\tau }_{f,1} < 0 \)</td><td>\( {\dot{q}}_{2} > 0 \)</td><td>\( {\tau }_{f,2} > 0 \)</td><td>\( {\dot{q}}_{2} < 0 \)</td><td>\( {\tau }_{f,2} < 0 \)</td><td>\( {\dot{q}}_{3} > 0 \)</td><td>\( {\tau }_{f,3} > 0 \)</td><td>\( {\dot{q}}_{3} < 0 \)</td><td>\( {\tau }_{f,3} < 0 \)</td></tr><tr><td>0.074</td><td>12.90</td><td>-0.074</td><td>-13.21</td><td>0.074</td><td>20.03</td><td>-0.074</td><td>-20.48</td><td>0.085</td><td>7.73</td><td>-0.085</td><td>-7.48</td></tr><tr><td>0.099</td><td>13.34</td><td>-0.099</td><td>-13.66</td><td>0.089</td><td>20.16</td><td>-0.089</td><td>-20.69</td><td>0.091</td><td>7.76</td><td>-0.091</td><td>-7.70</td></tr><tr><td>0.119</td><td>13.67</td><td>-0.119</td><td>-13.98</td><td>0.111</td><td>20.19</td><td>-0.111</td><td>-20.76</td><td>0.099</td><td>8.08</td><td>-0.099</td><td>-7.92</td></tr><tr><td>0.148</td><td>14.23</td><td>-0.148</td><td>-14.51</td><td>0.148</td><td>20.38</td><td>-0.149</td><td>-20.99</td><td>0.108</td><td>8.31</td><td>-0.108</td><td>-8.22</td></tr><tr><td>0.198</td><td>15.18</td><td>-0.198</td><td>-15.64</td><td>0.178</td><td>20.55</td><td>-0.178</td><td>-21.21</td><td>0.119</td><td>8.14</td><td>-0.119</td><td>-8.32</td></tr><tr><td>0.247</td><td>16.21</td><td>-0.246</td><td>-16.59</td><td>0.222</td><td>21.09</td><td>-0.223</td><td>-21.71</td><td>0.148</td><td>8.82</td><td>-0.148</td><td>-8.94</td></tr><tr><td>0.297</td><td>16.96</td><td>-0.297</td><td>-17.65</td><td>0.296</td><td>21.83</td><td>-0.297</td><td>-22.68</td><td>0.198</td><td>9.97</td><td>-0.199</td><td>-10.06</td></tr><tr><td>0.564</td><td>21.39</td><td>-0.598</td><td>-22.36</td><td>0.445</td><td>23.92</td><td>-0.444</td><td>-24.65</td><td>0.238</td><td>10.76</td><td>-0.239</td><td>-10.48</td></tr><tr><td>0.621</td><td>22.17</td><td>-0.656</td><td>-23.18</td><td>0.564</td><td>25.05</td><td>-0.560</td><td>-25.73</td><td>0.297</td><td>11.68</td><td>-0.302</td><td>-11.41</td></tr><tr><td>0.690</td><td>23.02</td><td>-0.740</td><td>-24.41</td><td>0.635</td><td>26.04</td><td>-0.638</td><td>-26.97</td><td>0.399</td><td>13.06</td><td>-0.396</td><td>-13.04</td></tr><tr><td>0.886</td><td>25.48</td><td>-0.852</td><td>-25.98</td><td>0.743</td><td>27.19</td><td>-0.748</td><td>-28.07</td><td>0.595</td><td>15.78</td><td>-0.593</td><td>-15.75</td></tr><tr><td>0.993</td><td>27.52</td><td>-0.993</td><td>-27.53</td><td>0.892</td><td>27.98</td><td>-0.890</td><td>-29.58</td><td>0.999</td><td>18.85</td><td>-0.992</td><td>-18.76</td></tr><tr><td>1.185</td><td>29.94</td><td>-1.187</td><td>-30.08</td><td>1.123</td><td>30.53</td><td>-1.110</td><td>-31.36</td><td>1.186</td><td>20.68</td><td>-1.184</td><td>-19.76</td></tr><tr><td>1.480</td><td>33.90</td><td>-1.485</td><td>-32.74</td><td>1.493</td><td>32.71</td><td>-1.479</td><td>-35.03</td><td>1.483</td><td>22.83</td><td>-1.483</td><td>-20.92</td></tr></table>

<!-- Media -->

where \( {\mathbf{Y}}_{G2} \) and \( {\mathbf{Y}}_{G3} \) are the gravity torque regression matrices of joint 2 and joint 3,and \( {\mathbf{K}}_{G2} \) and \( {\mathbf{K}}_{G3} \) are the gravity torque parameter matrices of joint 2 and joint 3,respectively. More details of \( {\mathbf{Y}}_{G2},{\mathbf{Y}}_{G3},{\mathbf{K}}_{G2} \) ,and \( {\mathbf{K}}_{G3} \) are introduced in Appendix A.

The recorded data sets \( \left( {{\mathbf{G}}_{2,m},{\mathbf{G}}_{3,m}}\right) \) of joint 2 and joint 3 are expressed as follows.

\[{\mathbf{G}}_{3,m} = \left\lbrack  \begin{matrix} {\tau }_{m,3}\left( {t}_{1}\right) \\  \vdots \\  {\tau }_{m,3}\left( {t}_{n}\right)  \end{matrix}\right\rbrack   = \left\lbrack  \begin{matrix} {\mathbf{Y}}_{G3}\left( {t}_{1}\right) \\  \vdots \\  {\mathbf{Y}}_{G3}\left( {t}_{n}\right)  \end{matrix}\right\rbrack  {\mathbf{K}}_{G3} = {\mathbf{Y}}_{t,{G3}}{\mathbf{K}}_{G3}\]

\[{\mathbf{G}}_{2,m} = \left\lbrack  \begin{matrix} {\tau }_{m,2}\left( {t}_{1}\right)  + {\mathbf{Y}}_{G3}\left( {t}_{1}\right) {\mathbf{K}}_{G3} \\  \vdots \\  {\tau }_{m,2}\left( {t}_{n}\right)  + {\mathbf{Y}}_{G3}\left( {t}_{n}\right) {\mathbf{K}}_{G3} \end{matrix}\right\rbrack   = \left\lbrack  \begin{matrix} {\mathbf{Y}}_{G2}\left( {t}_{1}\right) \\  \vdots \\  {\mathbf{Y}}_{G2}\left( {t}_{n}\right)  \end{matrix}\right\rbrack  {\mathbf{K}}_{G2}^{\prime } = {\mathbf{Y}}_{t,{G2}}^{\prime }{\mathbf{K}}_{G2}^{\prime } \tag{58}\]

\[\text{with}\left\{  \begin{array}{l} {\mathbf{Y}}_{t,{G3}}^{\prime } = {\left\lbrack  {\mathbf{Y}}_{G3}^{\prime }\left( {t}_{1}\right) ,\ldots ,{\mathbf{Y}}_{G3}^{\prime }\left( {t}_{n}\right) \right\rbrack  }^{T} \\  {\mathbf{Y}}_{t,{G2}}^{\prime } = {\left\lbrack  {\mathbf{Y}}_{G2}^{\prime }\left( {t}_{1}\right) ,\ldots ,{\mathbf{Y}}_{G2}^{\prime }\left( {t}_{n}\right) \right\rbrack  }^{T} \end{array}\right. \]

Firstly,the coefficient matrix \( {\mathbf{K}}_{G3}^{\prime } \) is obtained by the least squares method (LSM). In particular, \( {\mathbf{K}}_{G3} \) is obtained by removing the friction term \( \left( {f}_{3}\right) \) in \( {\mathbf{K}}_{G3}^{\prime } \) . Then, \( {\mathbf{K}}_{G3} \) is substituted into the expression of \( {\mathbf{G}}_{2,m} \) ,and the LSM is used to calculate the coefficient matrix \( {\mathbf{K}}_{G2}^{\prime } \) . In addition, \( {\mathbf{K}}_{G2} \) is obtained by removing the friction term \( \left( {f}_{3}\right) \) in \( {\mathbf{K}}_{G2}^{\prime }.{\mathbf{K}}_{G2}^{\prime } \) and \( {\mathbf{K}}_{G3}^{\prime } \) are obtained by Eq. (59),and results of \( {\mathbf{K}}_{G2} \) and \( {\mathbf{K}}_{G3} \) are shown in Eq. (60).

<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

\[\left\{  \begin{array}{l} {\mathbf{K}}_{G3}^{\prime } = {\left( {\mathbf{Y}}_{t,{G3}}^{\prime T}{\mathbf{Y}}_{t,{G3}}^{\prime }\right) }^{-1}{\mathbf{Y}}_{t,{G3}}^{\prime T}{\mathbf{G}}_{m,3} \\  {\mathbf{K}}_{G2}^{\prime } = {\left( {\mathbf{Y}}_{t,{G2}}^{\prime T}{\mathbf{Y}}_{t,{G2}}^{\prime }\right) }^{-1}{\mathbf{Y}}_{t,{G2}}^{\prime T}{\mathbf{G}}_{m,2} \end{array}\right.  \tag{59}\]

\[\left\{  \begin{array}{l} {\mathbf{K}}_{G2} = {\left\lbrack  {73.9033}, - {1.7083}\right\rbrack  }^{T} \\  {\mathbf{K}}_{G3} = {\left\lbrack  {38.5371},{0.6371}\right\rbrack  }^{T} \end{array}\right.  \tag{60}\]

<!-- Media -->

<!-- figureText: (a) (b) Measured data Predicted data 25 20 15 0 0.5 1 1.5 \( {\dot{\mathbf{q}}}_{1} \) (rad/s) (d) \( {T}_{c} \) , \( \left( {\mathrm{N} \cdot  \mathrm{m}}\right) \) 25 20 0 0.5 1 1.5 (f) \( {\dot{\mathbf{q}}}_{2} \) (rad/s) \( {\tau }_{\varepsilon ,2}\left( {\mathrm{\;N} \cdot  \mathrm{m}}\right) \) 20 15 10 0 0.5 1.5 \( {\dot{\mathbf{q}}}_{3} \) (rad/s) \( {\tau }_{\mathrm{f},1}\left( {\mathrm{N} \cdot  \mathrm{m}}\right) \) -15 -20 -25 -30 -1.5 - 1 -0.5 0 (c) \( {\dot{\mathbf{q}}}_{1} \) (rad/s) \( {\tau }_{\mathrm{f},2}\left( {\mathrm{\;N} \cdot  \mathrm{m}}\right) \) -25 -30 -35 -1.5 - 1 -0.5 0 (e) \( {\dot{\mathbf{q}}}_{2} \) (rad/s) \( {\tau }_{\mathrm{f},3}\left( {\mathrm{N} \cdot  \mathrm{m}}\right) \) -10 -15 -1.5 - 1 -0.5 0 \( {\dot{\mathbf{q}}}_{3} \) (rad/s) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_12.jpg?x=211&y=1402&w=1341&h=699&r=0"/>

Fig. 10. Friction fitting results of joints 1-3 (a) Forward motion of Joint 1 (b) Reverse motion of Joint 1(c) Forward motion of Joint 2. (d) Reverse motion of Joint 2 (e) Forward motion of Joint3 (f) Reverse motion of Joint 3.

<!-- Meanless: 13-->


Table 2

Identification experiment data of joint 2 's equivalent rotating inertia.

<table><tr><td>\( {q}_{3}\left( \mathrm{{deg}}\right) \)</td><td>0</td><td>30</td><td>-30</td><td>45</td><td>-45</td><td>-60</td><td>-90</td></tr><tr><td>\( {I}_{2}\left( {\mathrm{\;{kg}} \cdot  {\mathrm{s}}^{2}}\right) \)</td><td>18.801</td><td>17.382</td><td>17.268</td><td>16.064</td><td>16.064</td><td>13.813</td><td>9.344</td></tr></table>

Table 3

Identification experiment data of joint 1 's equivalent rotating inertia.

<table><tr><td>\( {q}_{3}\left( \mathrm{{deg}}\right) \)</td><td>\( {q}_{2}\left( \mathrm{{deg}}\right) \)</td><td>\( {I}_{1}\left( {\mathrm{\;{kg}} \cdot  {\mathrm{s}}^{2}}\right) \)</td></tr><tr><td>0</td><td>-180</td><td>21.069</td></tr><tr><td>-30</td><td>-120</td><td>11.021</td></tr><tr><td>-30</td><td>-135</td><td>16.119</td></tr><tr><td>-30</td><td>-150</td><td>19.168</td></tr><tr><td>-45</td><td>-90</td><td>4.067</td></tr><tr><td>-45</td><td>-120</td><td>11.625</td></tr><tr><td>-45</td><td>-135</td><td>16.546</td></tr><tr><td>-45</td><td>-150</td><td>19.065</td></tr><tr><td>-60</td><td>-90</td><td>5.230</td></tr><tr><td>-60</td><td>-120</td><td>11.109</td></tr><tr><td>-60</td><td>-135</td><td>13.172</td></tr><tr><td>-60</td><td>-150</td><td>13.732</td></tr><tr><td>-90</td><td>-90</td><td>4.729</td></tr><tr><td>-90</td><td>-120</td><td>9.606</td></tr><tr><td>-90</td><td>-135</td><td>11.257</td></tr></table>

<!-- Media -->

To identify the friction parameters, joints 1-3 are commanded to move at series constant speeds, respectively, and the joint currents are collected simultaneously. Then,the joint frictions \( \left( {{\tau }_{f,n}\left( {\dot{q}}_{n,m}\right) }\right) \) at different speeds are calculated by removing the influence of gravity from the joint torques using the following equation:

\[{\tau }_{f,n}\left( {\dot{q}}_{i,m}\right)  = {G}_{n}\left( q\right)  + {\tau }_{m,n} \tag{61}\]

The experiment data of the joint friction identification is given in Table 1. From Table 1, it can be observed that there is a positive correlation between friction and joint velocity. The frictional torque exhibits a consistent trend in both forward and reverse motion. Based on Eq. (25),the friction coefficient matrix of the \( i \) th joint \( {\mathbf{K}}_{f,i} \) is calculated by the LSM:

\[{\mathbf{K}}_{f,n} = {\left( {\mathbf{Y}}_{f,n}^{\prime T}{\mathbf{Y}}_{f,n}^{\prime }\right) }^{-1}{\mathbf{Y}}_{f,n}^{\prime T} \cdot  {\mathbf{\tau }}_{f,n}\left( {\dot{q}}_{n,k}\right) ,k = 1,2,\ldots ,m\]

\[\text{with}{\mathbf{\tau }}_{f,n}\left( {\dot{q}}_{n,k}\right)  = \left\lbrack  \begin{array}{l} {\tau }_{f,n}\left( {\dot{q}}_{n,1}\right) \\  \vdots \\  {\tau }_{f,n}\left( {\dot{q}}_{n,m}\right)  \end{array}\right\rbrack   = \left\lbrack  \begin{array}{l} {\mathbf{Y}}_{f,n}\left( {\dot{q}}_{n,1}\right) \\  \vdots \\  {\mathbf{Y}}_{f,n}\left( {\dot{q}}_{n,m}\right)  \end{array}\right\rbrack  {\mathbf{K}}_{f,n} = {\mathbf{Y}}_{f,n}^{\prime } \cdot  {\mathbf{K}}_{f,n} \tag{62}\]

where \( {\tau }_{f,n}\left( {\dot{q}}_{n,m}\right) \) means the frictional torque measured of the \( n \) -th joint at speed \( {\dot{q}}_{n,m} \) ,and \( {\tau }_{f,n}\left( {\dot{q}}_{n,k}\right) \) is the combination matrix of \( {\tau }_{f,n}\left( {\dot{q}}_{n,m}\right) \) . The results of the identified \( {\mathbf{K}}_{f,n} \) are as follows.

\[\left\{  \begin{array}{l} {\mathbf{K}}_{f,1}^{ + } = {\left\lbrack  \begin{array}{l} {0.6652},{0.5244},{0.6739} \end{array}\right\rbrack  }^{T} \\  {\mathbf{K}}_{f,1}^{ - } = {\left\lbrack  \begin{array}{l} {0.3503}, - {0.9297}, - {0.5921} \end{array}\right\rbrack  }^{T} \\  {\mathbf{K}}_{f,2}^{ + } = {\left\lbrack  \begin{array}{l} {0.4388},{0.3451},{1.2400} \end{array}\right\rbrack  }^{T} \\  {\mathbf{K}}_{f,2}^{ - } = {\left\lbrack  \begin{array}{l} {0.6015}, - {0.2172}, - {1.3035} \end{array}\right\rbrack  }^{T} \\  {\mathbf{K}}_{f,3}^{ + } = {\left\lbrack  \begin{array}{l} {0.0276},{1.3804},{0.2510} \end{array}\right\rbrack  }^{T} \\  {\mathbf{K}}_{f,3}^{ - } = {\left\lbrack  \begin{array}{l}  - {0.3090}, - {1.7599}, - {0.1525} \end{array}\right\rbrack  }^{T} \end{array}\right.  \tag{63}\]

The fitting results of the friction curve for joints 1-3 are shown in Fig. 10, where it can be observed that the proposed friction model has good fitting performance in both forward and reverse motion of joints.

To identify the parameters of the rotating inertia coefficient matrix \( {\mathbf{K}}_{I,n} \) ,which is introduced in Appendix B,the \( n \) -th joint is set to move at different accelerations, while other joints are stationary. The equivalent rotating inertia \( \left( {{I}_{n}\left( \mathbf{q}\right) }\right) \) is calculated using the following equation:

\[{I}_{n}\left( \mathbf{q}\right)  = {\left( {\mathbf{Q}}_{n}^{T}{\mathbf{Q}}_{n}\right) }^{-1}{\mathbf{Q}}_{n}^{T}{\mathbf{M}}_{m,n}n = 1,2,3\]

\[\text{with}{\mathbf{M}}_{m,n} = \left\lbrack  \begin{array}{l} {\tau }_{m,n}\left( {t}_{1}\right)  + {G}_{n}\left( {t}_{1}\right)  - {\tau }_{f,n}\left( {t}_{1}\right) \\  \vdots \\  {\tau }_{m,n}\left( {t}_{n}\right)  + {G}_{n}\left( {t}_{n}\right)  - {\tau }_{f,n}\left( {t}_{n}\right)  \end{array}\right\rbrack   \tag{64}\]

\[ = \left\lbrack  \begin{array}{l} \mathbf{M}\left( {t}_{1}\right) \\  \vdots \\  \mathbf{M}\left( {t}_{n}\right)  \end{array}\right\rbrack   = \left\lbrack  \begin{array}{l} {\ddot{q}}_{i}\left( {t}_{1}\right) \\  \vdots \\  {\ddot{q}}_{i}\left( {t}_{n}\right)  \end{array}\right\rbrack  {I}_{n}\left( \mathbf{q}\right)  = {\mathbf{Q}}_{n}{I}_{n}\left( \mathbf{q}\right) \]

where \( {\mathbf{M}}_{m,n} \) is the measured data set of inertia torque, \( {\mathbf{Q}}_{n} \) is the combination matrix of joint acceleration,and \( {\tau }_{m,n}\left( {t}_{n}\right) \) is the recorded current during the joint motion.

<!-- Media -->

<!-- figureText: (a) \( {\mathrm{R}}^{2} = {0.9983},\mathrm{{RMSE}} = {0.1179} \) (b) \( {\mathrm{R}}^{2} = {0.9914} \) ,RMSE \( = {0.4754} \) Measured data \( {\mathrm{I}}_{1}\left( {\mathrm{\;{kg}} \cdot  {\mathrm{m}}^{2}}\right) \) Fitted surface \( {\mathrm{I}}_{1}\left( {\mathrm{{kg}} \cdot  {\mathrm{m}}^{2}}\right) \) 1 0.5 - 1 \( {\mathrm{I}}_{1}\left( {\mathrm{{kg}} \cdot  {\mathrm{s}}^{2}}\right) \) \( {\mathbf{q}}_{3}\left( {\mathbf{r}\mathbf{a}\mathbf{d}}\right) \) -1.5 \( {\mathbf{q}}_{2} \) (rad) \( {\mathrm{q}}_{3} \) (rad) Fitting curve Measured data 18 \( {\mathrm{I}}_{2}\left( {\mathrm{\;{kg}}{\mathrm{\;m}}^{2}}\right) \) 16 \( {\mathrm{I}}_{1}\left( {\mathrm{{kg}} \cdot  {\mathrm{m}}^{2}}\right) \) 20 0 1.5 14 12 10 8 -2 - 1 1 2 \( {\mathrm{q}}_{3} \) (rad) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_13.jpg?x=191&y=1629&w=1374&h=503&r=0"/>

Fig. 11. Fitting results of the equivalent rotating inertia. (a) Joint 2. (b) Joint 1.

<!-- Meanless: 14-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

<!-- figureText: (a) (b) RMSE=4.1187 \( {\tau }_{\mathrm{m},1}\left( {\mathrm{N} \cdot  \mathrm{m}}\right) \) 20 Measured data Predictive data 10 25 RMSE=7.9706 \( {\tau }_{\mathrm{m},2}\left( {\mathrm{N} \cdot  \mathrm{m}}\right) \) 100 0 15 20 30 35 RMSE=3.2274 100 30 Time(s) 1.5 10 25 35 \( {q}_{2}\left( {\mathbf{r}\mathbf{a}\mathbf{d}}\right) \) -1.5 -2.5 5 10 15 20 25 35 - 1 \( {\mathrm{q}}_{3} \) (rad) -1.5 Time(s) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_14.jpg?x=196&y=158&w=1324&h=515&r=0"/>

Fig. 12. The test experiment of the established dynamics model (a) Trajectories of joints 1-3. (b) Comparison between measured torque and predicted torques of joints 1-3.

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_14.jpg?x=109&y=805&w=743&h=673&r=0"/>

Fig. 13. The discrete path for testing simulated in RobotDK.

<!-- Media -->

According to Eqs. (B.2) and (B.3) in Appendix B, the equivalent rotating inertia coefficient matrix \( {\mathbf{K}}_{I,n} \) are calculated by Eq. (64):

\[{\mathbf{K}}_{I,n} = {\left( {\mathbf{Y}}_{I,n}^{\prime T}{\mathbf{Y}}_{I,n}^{\prime }\right) }^{-1}{\mathbf{Y}}_{I,n}^{\prime T}{\mathbf{I}}_{n}^{\prime }\left( {n = 1,2,3}\right) \]

\[\text{with}{\mathbf{I}}_{n}^{\prime } = \left\lbrack  \begin{array}{l} {I}_{n}\left( {\mathbf{q}}_{1}\right) \\  \vdots \\  {I}_{n}\left( {\mathbf{q}}_{m}\right)  \end{array}\right\rbrack   = {\mathbf{Y}}_{I,n}^{\prime }{\mathbf{K}}_{I,n},{\mathbf{Y}}_{I,n}^{\prime } = \left\lbrack  \begin{array}{l} {\mathbf{Y}}_{I,n}\left( {\mathbf{q}}_{1}\right) \\  \vdots \\  {\mathbf{Y}}_{I,n}\left( {\mathbf{q}}_{m}\right)  \end{array}\right\rbrack   \tag{65}\]

where \( {\mathbf{I}}_{n}^{\prime } \) is the combination matrix of \( {I}_{n}\left( {\mathbf{q}}_{m}\right) \) .

The experimental data are shown in Table 2 and Table 3. The fitting results of the equivalent rotating inertia of joints 1-2 are shown in Fig. 11. The equivalent rotating inertia of joint 2 is only related to \( {q}_{3} \) ,so the fitting result is a curve shown in Fig. 11(a). The fitting coefficient \( \left( {R}^{2}\right) \) and the root mean square error (RMSE) are 0.9983 and 0.1179, respectively, demonstrating that the fitting results of joint 2 match well with the actual condition. The equivalent rotating inertia of joint 1 is related to \( {q}_{3} \) and \( {q}_{2} \) . Therefore,the fitting result is a three-dimensional surface shown in Fig. 11(b). When \( {q}_{3} \) takes \( - \pi /6, - \pi /4 \) ,and \( - \pi /3 \) , respectively, three curves of equivalent rotating inertia are drawn on the two-dimensional plane to display the fitting results more intuitively. From Fig. 11(b),it can be seen that the predicted values of \( {I}_{1} \) match well with the actual measurement results on the cross-section of the fitted surface. In addition,the values of \( {R}^{2} \) and RMSE of \( {I}_{1} \) are 0.9914 and 0.4754 , respectively, demonstrating that the identified results of joint 1 match well with the actual condition. In the same way, \( {I}_{3} \) is identified as \( {3.431}\mathrm{\;{kg}} \cdot  {\mathrm{s}}^{2} \) .

<!-- Media -->

Table 4

The path data of the robot end tool poses.

<table><tr><td colspan="3">Tool tip position</td><td colspan="3">Tool orientation</td></tr><tr><td>X(mm)</td><td>Y(mm)</td><td>Z(mm)</td><td>\( \alpha \) (rad)</td><td>\( \beta \) (rad)</td><td>\( \gamma \) (rad)</td></tr><tr><td>367.047</td><td>410.172</td><td>275.170</td><td>0.935</td><td>-0.187</td><td>2.969</td></tr><tr><td>476.082</td><td>611.287</td><td>340.064</td><td>0.645</td><td>-0.088</td><td>2.985</td></tr><tr><td>461.082</td><td>786.287</td><td>310.064</td><td>0.928</td><td>-0.220</td><td>2.823</td></tr><tr><td>546.082</td><td>966.287</td><td>220.064</td><td>0.681</td><td>-0.205</td><td>3.045</td></tr><tr><td>466.082</td><td>1081.287</td><td>220.064</td><td>-0.511</td><td>0.109</td><td>3.408</td></tr><tr><td>316.082</td><td>1046.287</td><td>220.064</td><td>0.278</td><td>-0.112</td><td>3.486</td></tr><tr><td>306.082</td><td>886.287</td><td>285.064</td><td>0.256</td><td>0.035</td><td>3.333</td></tr><tr><td>256.082</td><td>661.287</td><td>375.064</td><td>0.256</td><td>0.035</td><td>3.246</td></tr><tr><td>206.082</td><td>511.287</td><td>375.064</td><td>0.290</td><td>0.382</td><td>3.211</td></tr><tr><td>76.082</td><td>511.287</td><td>375.064</td><td>0.256</td><td>-0.009</td><td>3.355</td></tr><tr><td>76.082</td><td>641.287</td><td>375.064</td><td>0.256</td><td>0.035</td><td>3.268</td></tr><tr><td>146.082</td><td>751.287</td><td>375.064</td><td>0.229</td><td>-0.071</td><td>3.487</td></tr><tr><td>91.082</td><td>1011.287</td><td>375.064</td><td>0.256</td><td>0.035</td><td>3.442</td></tr><tr><td>-3.918</td><td>1091.287</td><td>290.064</td><td>0.301</td><td>0.207</td><td>3.404</td></tr><tr><td>-53.918</td><td>951.287</td><td>290.064</td><td>0.255</td><td>0.231</td><td>3.268</td></tr><tr><td>-78.918</td><td>811.287</td><td>345.064</td><td>0.275</td><td>0.340</td><td>3.206</td></tr><tr><td>-118.918</td><td>651.287</td><td>345.064</td><td>0.255</td><td>0.209</td><td>3.290</td></tr><tr><td>-173.918</td><td>531.287</td><td>350.064</td><td>0.260</td><td>0.230</td><td>2.918</td></tr><tr><td>-3.918</td><td>411.287</td><td>350.064</td><td>0.256</td><td>0.035</td><td>3.137</td></tr><tr><td>367.047</td><td>410.172</td><td>275.170</td><td>0.935</td><td>-0.187</td><td>2.969</td></tr></table>

Table 5

Tangential dynamic constraints of the robot end tool path.

<table><tr><td colspan="2">Position dynamic constraints</td><td colspan="2">Orientation dynamic constraints</td></tr><tr><td>Velocity(v)</td><td>125 mm/s</td><td>Velocity \( \left( \omega \right) \)</td><td>25 rad/s</td></tr><tr><td>Acceleration(a)</td><td>\( {1000}\mathrm{\;{mm}}/{\mathrm{s}}^{2} \)</td><td rowspan="2">Acceleration \( \left( {\omega }^{\prime }\right) \) Jerk \( \left( {\omega }^{\prime }\right) \)</td><td>\( {250}\mathrm{{rad}}/{\mathrm{s}}^{2} \)</td></tr><tr><td>Jerk(j)</td><td>\( {40},{000}\mathrm{\;{mm}}/{\mathrm{s}}^{3} \)</td><td>\( {10},{000}\mathrm{{rad}}/{\mathrm{s}}^{3} \)</td></tr></table>

Table 6

Torque constraints of joints 1-3 .

<table><tr><td>Joint 1</td><td>30N·m</td></tr><tr><td>Joint 2</td><td>200N·m</td></tr><tr><td>Joint 3</td><td>100N·m</td></tr></table>

<!-- Meanless: 15-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

<!-- figureText: (a) (b) Error of position (um) 404 30 Comparison method Error tolerance 20 - Proposed method 10 8 12 18 Error of orientation (rad) \( \times  {10}^{-4} \) 6 12 14 16 18 Number of the corner Discrete path Proposed method Feed direction \( {\mathrm{P}}_{7}\left\lbrack  \mathrm{{mm}}\right\rbrack \) 400 300 200 400 1000 200 800 600 \( {\mathbf{P}}_{\mathbf{v}}\left\lbrack  \mathbf{{mm}}\right\rbrack \) P [mm] -173.7 -173.8 -3.92 \( {\mathrm{P}}_{\mathrm{v}}\left\lbrack  \mathrm{{mm}}\right\rbrack \) -173.9 -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_15.jpg?x=198&y=168&w=1318&h=541&r=0"/>

Fig. 14. Path smoothing results by the proposed method and the comparison method. (a) Smoothing result of the discrete path. (b) Corner deviation of the position and the orientation path.

<!-- figureText: (a) (b) Comparison method Proposed method Dynamic constraints 20 10 0 5 10 15 20 25 35 200 -200 5 10 15 20 25 35 Time (s) 150 v (mm/s) 100 \( \omega \left( {\mathrm{{deg}}/\mathrm{s}}\right) \) 10 15 20 25 35 \( \mathbf{{30.47s}} \) \( \mathrm{{34.19s}} \) 1000 a (mm \( /{\mathrm{s}}^{2} \) ) \( {\omega }^{\prime }\left( {\mathrm{{deg}}/{\mathrm{s}}^{2}}\right) \) -1000 5 10 15 20 25 30 35 Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_15.jpg?x=217&y=840&w=1311&h=496&r=0"/>

Fig. 15. Tangential and orientation dynamic characteristics of the proposed method and the comparison method. (a) Tangential dynamic characteristics (b) Orientation dynamic characteristics.

<!-- figureText: (a) (b) Proposed method 200 Comparison method 0 -200 0 5 10 15 20 25 30 35 200 0 -200 5 10 15 20 25 30 35 200 -200 25 30 35 Time (s) Proposed method 20 Comparison method \( {\ddot{\mathbf{q}}}_{1}\left( {\mathbf{{deg}}/{\mathbf{s}}^{2}}\right) \) \( {\dot{\mathbf{q}}}_{1}\left( {\deg /\mathbf{s}}\right) \) 0 20 0 5 10 15 20 25 30 35 \( {\dot{\mathbf{q}}}_{2} \) (deg/s) \( {\ddot{\mathbf{q}}}_{2}\left( {\mathbf{{deg}}/{\mathbf{s}}^{2}}\right) \) 5 10 15 20 25 30 35 \( {\dot{\mathbf{q}}}_{3}\left( {\deg /\operatorname{s}}\right) \) \( {\ddot{\mathbf{q}}}_{3}\left( {\mathbf{{deg}}/{\mathbf{s}}^{2}}\right) \) 10 25 30 Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_15.jpg?x=198&y=1475&w=1346&h=641&r=0"/>

Fig. 16. Joint dynamic characteristics of the proposed method and the comparison method. (a) Velocity characteristics of joints 1-3 (b) Acceleration characteristics of joints 1-3.

<!-- Meanless: 16-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

<!-- figureText: Proposed method Comparison method Torque constraints 20 25 30 35 20 25 30 35 20 25 30 35 Time (s) \( {\tau }_{\mathrm{m},1}\left( {\mathrm{\;N} \cdot  \mathrm{m}}\right) \) 20 0 -20 0 10 15 \( {\tau }_{\mathrm{m},2}\left( {\mathrm{N} \cdot  \mathrm{m}}\right) \) 200 100 0 10 15 m 3 (N·m) 100 50 0 10 15 -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_16.jpg?x=372&y=148&w=1003&h=799&r=0"/>

Fig. 17. Torques of joints 1-3 during the robot's movement by the proposed method and the comparison method.

<!-- Media -->

Joints 1-3 are moved simultaneously to test the accuracy of the dynamics model. The motion path is shown in Fig. 12(a). Joints 1-3 move simultaneously in the testing trajectory. The motion drive torques of joints 1-3 are recorded as shown in Fig. 12(b). Similarly, the joint driving torques calculated using the established dynamic model are also plotted in the figure for comparison. From the test results, it can be seen that the established dynamics model can forecast the torques of joints 1-3 exactly based on a comparison of the measured and predicted data of joints 1-3 .

### 5.2. Experiment results of the trajectory smoothing algorithm

To test the proposed algorithm, the tool path smoothing algorithm in Ref. [65] based on the symmetrical FIR filter is set as the compared method. The discrete path for testing is simulated in the software RobotDK shown in Fig. 13, and the path data are listed in Table 4. The tool positions are described in the Cartesian coordinate system, and the tool orientation is described by ZYX Euler angles. The tangential dynamic constraints and the pose-dependent dynamic constraints of joints 1-3 (replaced by the joint motion drive torque constraints) are listed in Table 5 and Table 6. According to the official technical document of the UR10 robot,the nominal torque constraint of joints \( 1 - 3 \) is \( {200}\mathrm{\;N} \cdot  \mathrm{m} \) . Considering the installation pose of the robot in the experiment, joint 3 drives 3-6, and the actual torque on joint 3 is less than that of joint 2. Similarly, joint 1 only receives inertial torque, while it is not affected by the gravity torque as joint 2 and 3 . Therefore, the actual maximum driving torques that are achieved on joints 1-3 are different. The maximum torques of joints 1 and 3 are multiplied by safety factors to verify the effectiveness of the algorithm in this paper. Torque constraints \( {30}\mathrm{\;N} \cdot  \mathrm{m} \) and \( {100}\mathrm{\;N} \cdot  \mathrm{m} \) have been set for joints 1 and 3,respectively. The corner error tolerances of the tool end paths are set as \( {0.04}\mathrm{\;{mm}} \) and 0.001 rad. The pose-dependent dynamic constraints of joints 1-3 at the corners are calculated by the proposed method introduced in Section 4.3. The tool path smoothing results are shown in Fig. 14. The pose-dependent dynamics model is not considered in the comparison method. Thus, the conservative value is set as the joint constraint along the entire path to ensure the dynamic constraints.

<!-- Media -->

<!-- figureText: \( \times  {10}^{-3} \) Proposed method Comparison method \( \times  {10}^{-3} \) 1 \( {\mathrm{e}}_{2} \) (rad) 0 10 15 20 25 30 35 \( \times  {10}^{-3} \) \( {\mathbf{e}}_{4} \) (rad) -2 5 10 15 20 25 30 35 \( \times  {10}^{-3} \) 4 \( {\mathrm{e}}_{6} \) (rad) 5 10 15 20 25 30 35 Time (s) \( {\mathbf{e}}_{1} \) (rad) 5 10 15 20 25 30 35 \( \times  {10}^{-3} \) \( {\mathrm{e}}_{3} \) (rad) 10 15 20 25 30 35 \( \times  {10}^{-3} \) e \( {}_{5} \) (rad) 5 10 15 20 25 30 35 Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_16.jpg?x=207&y=1513&w=1320&h=632&r=0"/>

Fig. 18. Tracking errors of joint \( 1 \sim  6\left( {{\mathrm{e}}_{1} \sim  {\mathrm{e}}_{6}}\right) \) during the motion by the proposed method and the comparison method.

<!-- Media -->

<!-- Meanless: 17-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

As shown in Fig. 14, the proposed method smooths the corner of the discrete segments as well as the comparison method. The corner deviation can be well constrained within the error tolerance by the proposed method. The tangential dynamic characteristics are shown in Fig. 15, and the dynamic characteristics of joints 1-3 are shown in Fig. 16. The cycle time of the proposed method is 30.47, which is 10.88% more than the cycle time of the comparison method (34.19 s). The efficiency improvement is mainly due to the joint dependent dynamic characteristics. The joint dynamic constraints of the comparison method are set as conservative values to satisfy the limitation of the joint drive torque. However, the proposed method sets the joint dynamic constraints based on the pose-dependent dynamics explained in Section 3, fully utilizing the dynamic ability of the joint drives. As shown in Fig. 17, both the proposed method and the comparison method satisfy the given motion drive torque constraints. However, the joint dynamic constraints of the comparison method use fixed conservative values, which limits the tangential dynamic characteristics. In contrast, the proposed method considers the dynamics of robot joints at different postures to establish dynamic constraints for joints 1-3. Consequently, compared with the comparison method under the tangential dynamic constraints, the proposed method improves the tangential movement efficiency while fully utilizing the joint driving ability. The tracking errors are depicted in Fig. 18, where the comparison method and the proposed method demonstrate joint tracking errors of similar magnitudes. This implies that the proposed method enhances motion efficiency without compromising joint motion control accuracy. Therefore, it can be concluded that, despite marginal disparities in joint tracking errors, the proposed method exhibits superior motion performance than the comparison method.

## 6. Conclusion

This paper presents a novel tool path smoothing algorithm based on asymmetrical FIR filters, considering the pose-dependent dynamics of the robot. In proposed method, the pose-dependent dynamic performance of the robot is considered. A simplified dynamics model is established to analyze dynamic constraints of the robot joints under different poses. The tangential dynamic constraints are adjusted by the pose-dependent joint dynamic constraints. To achieve different acceleration constraints during the acceleration and deceleration stages in the linear segment, an asymmetrical smoothing algorithm based on FIR filters is introduced. The paths of the robot's end tool are individually smoothed in the WCS. The adjacent linear segments are overlapped to ensure smooth transitions at the corners. By setting the overlapping time, the deviation tolerance at each corner can be maintained. Furthermore, the synchronization of the tool tip position and tool orientation is ensured by adjusting the parameters of the asymmetrical FIR filters. Then, through simulations and experiments, the proposed method is compared with the traditional symmetrical FIR filter-based smoothing algorithm. The results demonstrate that the proposed method achieves higher motion efficiency under the same joint dynamic constraints as the comparison method.

## CRediT author statement

Hongwei Sun: Writing-original draft, Formulation, Investigation, Methodology, Formal analysis, Validation. Jixiang Yang: Writing-review & editing, Methodology, Supervision, Project administration. Han Ding: Supervision, Project administration.

## Declaration of Competing Interest

We declare that we do not have any commercial or associative interest that represents a conflict of interest in connection with the work submitted.

## Acknowledgments

This research is supported by the National Natural Science Foundation of China under Grant Nos. 52122512 and 52188102, the Natural Science Foundation of Hubei Province, China, under Grant No. 2021CFA075, and the Fundamental Research Funds for the Central Universities, HUST, under Grant No. 2022JYCXJJ019.

## Appendix A. Modeling of the joint gravity torques for UR10

As shown in Fig. A1, the gravity term has no influence on joint 1 because the rotary axis direction of joint 1 is parallel to the gravity vector direction (g). The joint gravities mainly influence the torques of joints 2 and 3 . (1) Modeling and identification of gravity torque for joint 3

When analyzing the influences of joint gravity on axis torques, joints 4-6 are treated to be an integral part fixed to link 3 because the links of joints 4-6 are relatively much shorter than joints 1-3. As joint 3 only drives link 3,so \( {G}_{3}\left( \mathbf{q}\right) \) is formulated as follow according to Fig. A1 as follows.

\[{G}_{3}\left( \mathbf{q}\right)  = \left\lbrack  {\cos \left( {{q}_{2} + {q}_{3}}\right) ,\sin \left( {{q}_{2} + {q}_{3}}\right) }\right\rbrack  \left\lbrack  \begin{array}{l} {m}_{3}g{l}_{3x} \\  {m}_{3}g{l}_{3y} \end{array}\right\rbrack  \]

\[ = \left\lbrack  {\cos \left( {{q}_{2} + {q}_{3}}\right) ,\sin \left( {{q}_{2} + {q}_{3}}\right) }\right\rbrack  \left\lbrack  \begin{array}{l} {k}_{{G3},1} \\  {k}_{{G3},2} \end{array}\right\rbrack   = {\mathbf{Y}}_{G3}{\mathbf{K}}_{G3} \tag{A.1}\]

\[\text{with}{\mathbf{Y}}_{G3} = \left\lbrack  {\cos \left( {{q}_{2} + {q}_{3}}\right) ,\sin \left( {{q}_{2} + {q}_{3}}\right) }\right\rbrack  ,{\mathbf{K}}_{G3} = {\left\lbrack  {k}_{{G3},1},{k}_{{G3},2}\right\rbrack  }^{T}\]

<!-- Meanless: 18-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

<!-- Media -->

<!-- figureText: (a) \( {m}_{3} \) (b) \( {m}_{3},{G}_{3} \) \( {l}_{2y} \) \( {q}_{1} \) \( {q}_{3} \) \( {l}_{3x} \) \( {q}_{2} \) \( q \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_18.jpg?x=303&y=157&w=1120&h=568&r=0"/>

Fig. A1. Gravity torques modeling the principle of joints 2 and 3. (a) Joint 3. (b) Joint 2.

<!-- Media -->

## (2) Modelling and identification of gravity torque for joint 2

Joint 2 is responsible for driving both links 2 and 3. Here, the gravitational force acting on the barycenter of link 3 is translated to the rotary axis of joint 3 to calculate the impact of gravity on link 3 and its effect on joint 2, thereby resulting in an additional torque acting on joint 2. The mass of link 3 can be equivalent to link 2,as shown in Fig. A1. Therefore, \( {G}_{2} \) is formulated as follows.

\[{G}_{2}\left( \mathbf{q}\right)  = {G}_{3}\left( \mathbf{q}\right)  + \left\lbrack  {\cos \left( {q}_{2}\right) ,\sin \left( {q}_{2}\right) }\right\rbrack  \left\lbrack  \begin{array}{l} {m}_{3}g{l}_{2} + {m}_{2}g{l}_{x2} \\  {m}_{2}g{l}_{y2} \end{array}\right\rbrack  \]

\[ = {\mathbf{Y}}_{G3}{\mathbf{K}}_{G3} + \left\lbrack  {\cos \left( {q}_{2}\right) ,\sin \left( {q}_{2}\right) }\right\rbrack  \left\lbrack  \begin{array}{l} {k}_{{G2},1} \\  {k}_{{G2},2} \end{array}\right\rbrack   \tag{A.2}\]

\[ = {\mathbf{Y}}_{G3}{\mathbf{K}}_{G3} + {\mathbf{Y}}_{G2}{\mathbf{K}}_{G2}\]

\[\text{with}{\mathbf{Y}}_{G2} = \left\lbrack  {\cos \left( {q}_{2}\right) ,\sin \left( {q}_{2}\right) }\right\rbrack  ,{\mathbf{K}}_{G2} = {\left\lbrack  {k}_{{G2},1},{k}_{{G2},2}\right\rbrack  }^{T}\]

The gravity term has no effect on joint 1 ; therefore \( {G}_{1} = 0 \) . The gravity vector \( \mathbf{G}\left( \mathbf{q}\right) \) of joints 1-3 is given by:

\[\mathbf{G}\left( \mathbf{q}\right)  = {\left\lbrack  {G}_{1}\left( \mathbf{q}\right) ,{G}_{2}\left( \mathbf{q}\right) ,{G}_{3}\left( \mathbf{q}\right) \right\rbrack  }^{T} \tag{A.3}\]

## Appendix B. Modeling of joint inertial torques for UR10

The inertia torques \( \mathbf{M}\left( \ddot{\mathbf{q}}\right) \) of the robot joints are formulated as

\[\mathbf{M}\left( \ddot{\mathbf{q}}\right)  = \left\lbrack  \begin{matrix} {M}_{1} \\  {M}_{2} \\  {M}_{3} \end{matrix}\right\rbrack   = \left\lbrack  \begin{matrix} {I}_{1}\left( \mathbf{q}\right)  \cdot  {\ddot{q}}_{1} \\  {I}_{2}\left( \mathbf{q}\right)  \cdot  {\ddot{q}}_{2} \\  {I}_{3}\left( \mathbf{q}\right)  \cdot  \left( {{\ddot{q}}_{2} + {\ddot{q}}_{3}}\right)  \end{matrix}\right\rbrack   \tag{B.1}\]

where \( {M}_{n}\left( {n = 1,2,3}\right) \) represents the inertial torque of the \( n \) -th joint,and \( {I}_{n}\left( \mathbf{q}\right) \) is the equivalent rotating inertia on the \( i \) th joint under different poses. Here, \( {I}_{n}\left( \mathbf{q}\right) \) may vary in different robot poses and is only influenced by such poses. Therefore, \( {I}_{n}\left( \mathbf{q}\right) \) can be modeled as follows.

(1) Modeling of the equivalent rotational inertia of joint 3

Given that joint 3 drives links 3-6,and the effects of links \( 4 - 6 \) on \( {I}_{3}\left( \mathbf{q}\right) \) are very small, \( {I}_{3}\left( \mathbf{q}\right) \) can be regarded as a fixed value.

(2) Modeling of the equivalent rotational inertia of joint 2

As for joint 2,it drives links 2 and 3. The equivalent rotating inertia of link 2 to the joint axis 2 is a fixed value \( \left( {j}_{2}^{2}\right) \) ; thus,the equivalent rotating inertia generated by link 3 to joint 2 is mainly considered. The modeling method of \( {I}_{2}\left( \mathbf{q}\right) \) is shown in Fig. B1(a).

According to the parallel axis theorem, \( {I}_{2}\left( \mathbf{q}\right) \) is calculated by Eq. (B.2):

<!-- Meanless: 19-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

\[{I}_{2}\left( \mathbf{q}\right)  = {j}_{2}^{2} + {j}_{3}^{2} + {m}_{3}{r}_{3}^{2}\]

\[ = {j}_{2}^{2} + {j}_{3}^{2} + {m}_{3}\left( {{l}_{2}^{2} + {l}_{G3}^{2} - 2{l}_{2}{l}_{G3}\cos \left( {\pi /2 + {\varphi }_{3} - {q}_{3}}\right) }\right) \]

\[ =  - 2{m}_{3}{l}_{2}{l}_{G3}\sin \left( {{q}_{3} - {\varphi }_{3}}\right)  + {j}_{2}^{2} + {j}_{3}^{2} + {m}_{3}\left( {{l}_{2}^{2} + {l}_{G3}^{2}}\right) \]

\[ = \left\lbrack  {\sin \left( {{q}_{3} - {\varphi }_{3}}\right) ,1}\right\rbrack  \left\lbrack  \begin{array}{l} {k}_{{I}_{2},1} \\  {k}_{{I}_{2},2} \end{array}\right\rbrack   = {\mathbf{Y}}_{I,2}{\mathbf{K}}_{I,2} \tag{B.2}\]

\[\text{with}\left\{  \begin{array}{l} {\mathbf{Y}}_{I,2} = \left\lbrack  {\sin \left( {{q}_{3} - {\varphi }_{3}}\right) ,1}\right\rbrack  ,{\mathbf{K}}_{I,2} = {\left\lbrack  {k}_{{I}_{2},1},{k}_{{I}_{2},2}\right\rbrack  }^{T},{\varphi }_{3} = \arctan \frac{{l}_{y3}}{{l}_{x3}} \\  {k}_{{I}_{2},1} =  - 2{m}_{3}{l}_{2}{l}_{G3},{k}_{{I}_{2},2} = {j}_{2}^{2} + {j}_{3}^{2} + {m}_{3}\left( {{l}_{2}^{2} + {l}_{G3}^{2}}\right)  \end{array}\right. \]

where \( {j}_{3}^{2} \) is the equivalent rotating inertia about an axis parallel to axis 2 through the barycenter of link 3,and \( {r}_{3} \) means the distance from the barycenter of link 3 to axis 2 . (3) Modeling of the equivalent moment of inertia of joint 1

As joint 1 drives joints 2 and 3,the equivalent rotational inertia to joint 1 is a function of \( {q}_{2} \) and \( {q}_{3} \) . The modeling of the equivalent rotating inertia to joint 1 is shown in Fig. B1(b). Therefore, \( {I}_{1}\left( \mathbf{q}\right) \) is formulated as follows.

\[{I}_{1}\left( \mathbf{q}\right)  = {\left( {l}_{G2}\sin \left( {q}_{2} - {\varphi }_{2}\right) \right) }^{2} + {j}_{2}^{1} + {\left( d + {l}_{2}\sin \left( {q}_{2}\right)  + {l}_{G3}\cos \left( {q}_{2} + {q}_{3} - {\varphi }_{3}\right) \right) }^{2} + {j}_{3}^{1}\]

\[ = {l}_{G2}^{2}{\sin }^{2}\left( {{q}_{2} - {\varphi }_{2}}\right)  + {l}_{2}^{2}{\sin }^{2}\left( {q}_{2}\right)  + {l}_{G3}^{2}{\cos }^{2}\left( {{q}_{2} + {q}_{3} - {\varphi }_{3}}\right)  + 2{l}_{G3}d\cos \left( {{q2} + {q3} - {\varphi }_{3}}\right) \]

\[ + 2{l}_{2}\sin \left( {q}_{2}\right)  + 2{l}_{2}{l}_{G3}\sin \left( {q}_{2}\right) \cos \left( {{q}_{2} + {q}_{3} - {\varphi }_{3}}\right)  + {j}_{2}^{1} + {j}_{3}^{1} + {d}^{2}\]

\[ = {\mathbf{Y}}_{I,1}{\mathbf{K}}_{I,1} \tag{B.3}\]

\[\operatorname{with}\left\{  \begin{array}{l} {\mathbf{Y}}_{I,1} = \left\lbrack  {{\sin }^{2}\left( {{q}_{2} - {\varphi }_{2}}\right) ,{\sin }^{2}\left( {q}_{2}\right) ,{\cos }^{2}\left( {{q}_{2} + {q}_{3} - {\varphi }_{3}}\right) ,\cos \left( {{q2} + {q3} - {\varphi }_{3}}\right) ,\sin \left( {q}_{2}\right) \cos \left( {{q}_{2} + {q}_{3} - {\varphi }_{3}}\right) ,\sin \left( {q}_{2}\right) ,1}\right\rbrack  \\  {\mathbf{K}}_{I,1} = {\left\lbrack  {k}_{{I}_{1},1},{k}_{{I}_{1},2},{k}_{{I}_{1},3},{k}_{{I}_{1},4},{k}_{{I}_{1},5},{k}_{{I}_{1},6},{k}_{{I}_{1},7}\right\rbrack  }^{T},{\varphi }_{2} = \arctan \frac{{l}_{y2}}{{k}_{2}} \end{array}\right. \]

where \( {j}_{k}^{1}\left( {k = 2,3}\right) \) is the equivalent rotating inertia about an axis parallel to axis 1 through the barycenter of the \( k \) -th link.

<!-- Media -->

<!-- figureText: (a) (b) \( {l}_{G3} \) \( {l}_{G2} \) \( {m}_{3} \) \( {q}_{3} \) 10 \( {q}_{2} \) \( {q}_{1} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl96bef24c73b2kegg_19.jpg?x=340&y=1117&w=1077&h=539&r=0"/>

Fig. B1. Modeling of the equivalent inertial moment of joints 2 and 1. (a) Joint 2. (b) Joint 1.

<!-- Media -->

[1] R. Schienbein, F. Fern, R. Theska, S. Supreeti, R. Füßl, E. Manske, Fundamental investigations in the design of five-axis nanopositioning machines for measurement and fabrication purposes, Nanomanufac. Metrol. 4 (3) (2021) 156-164.

[2] Y. Altintas, J. Yang, Z.M. Kilic, Virtual prediction and constraint of contour errors induced by cutting force disturbances on multi-axis CNC machine tools, CIRP Ann. 68 (1) (2019) 377-380.

[3] A. Verl, A. Valente, S. Melkote, C. Brecher, E. Ozturk, L.T. Tunc, Robots in machining, CIRP Ann. 68 (2) (2019) 799-822.

[4] W. Ji, L. Wang, Industrial robotic machining: a review, Int. J. Adv. Manuf. Technol. 103 (1-4) (2019) 1239-1255.

[5] Y. Ding, X. Min, W. Fu, Z. Liang, Research and application on force control of industrial robot polishing concave curved surfaces, Proc.Inst. Mech. Eng. Part B-J. Engineer. Manuf. 233 (6) (2019) 1674-1686.

[6] A. Kharidege, D. Ting, Z. Yajun, A practical approach for automated polishing system of free-form surface path generation based on industrial arm robot, Int. J. Adv. Manuf. Technol. 93 (9-12) (2017) 3921-3934.

[7] Y. Wang, J. Yang, D. Li, H. Ding, Tool path generation with global interference avoidance for the robotic polishing of blisks, Int. J. Adv. Manuf. Technol. 117 (3) (2021) 1223-1232.

[8] M. Cordes, W. Hintze, Y. Altintas, Chatter stability in robotic milling, Robot Comput. Integr. Manuf. 55 (2019) 11-18.

[9] J. Lin, C. Ye, J. Yang, H. Zhao, H. Ding, M. Luo, Contour error-based optimization of the end-effector pose of a 6 degree-of-freedom serial robot in milling operation, Robot Comput. Integr. Manuf. 73 (2022), 102257.

[10] S. Mousavi, V. Gagnol, B.C. Bouzgarrou, P. Ray, Stability optimization in robotic milling through the control of functional redundancies, Robot Comput. Integr. Manuf. 50 (2018) 181-192.

[11] G. Xiong, Y. Ding, L. Zhu, Stiffness-based pose optimization of an industrial robot for five-axis milling, Robot Comput. Integr. Manuf. 55 (2019) 19-28.

<!-- Meanless: References 20-->




<!-- Meanless: H. Sun et al. Robotics and Computer-Integrated Manufacturing 86 (2024) 102681-->

[12] J. Li, T. Yuan, W. Wu, H. Zhu, C. Zhang, J. Xie, Ieee, Automatic programming system for grinding robot of CHSR rail, in: 2018 IEEE International Conference on Robotics and Biomimetics (ROBIO), 2018, pp. 1391-1396.

[13] J. Su, H. Qiao, L. Xu, M. Wang, Ieee, A method of human-robot collaboration for grinding of workpieces, in: 2015 IEEE International Conference on Mechatronics and Automation, 2015, pp. 1156-1161.

[14] X. Zhao, H. Lu, W. Yu, B. Tao, H. Ding, Robotic grinding process monitoring by vibration signal based on LSTM method, IEEE Trans. Instrum. Meas. 71 (2022) \( 1 - {10} \) .

[15] Y. Song, H. Lv, Z. Yang, An adaptive modeling method for a robot belt grinding process, IEEE/ASME Trans. Mechatron. (2012) 309-317.

[16] S. Chen, P. Tung, Trajectory planning for automated robotic deburring on an unknown contour, Int. J. Mach. Tools Manuf 40 (7) (2000) 957-978.

[17] F.Y. Hsu, L.C. Fu, Intelligent robot deburring using adaptive fuzzy hybrid position/ force control, IEEE Transac. Robot.d Autom. 16 (4) (2000) 325-335.

[18] B.K. Pappachan, W. Caesarendra, T. Tjahjowidodo, T. Wijaya, Frequency domain analysis of sensor data for event classification in real-time robot assisted deburring, Sensors 17 (6) (2017) 1024.

[19] H.-C. Song, J.-B. Song, Precision robotic deburring based on force control for arbitrarily shaped workpiece using CAD model matching, Int. J. Precis. Eng. Manuf. 14 (1) (2013) 85-91.

[20] G. Ziliani, A. Visioli, G. Legnani, A mechatronic approach for robotic deburring, Mechatronics 17 (8) (2007) 431-441.

[21] C. Lartigue, E. Duc, A. Affouard, Tool path deformation in 5-axis flank milling using envelope surface, Comput. Aided Des. 35 (4) (2003) 375-382.

[22] T.-C. Lu, S.-L. Chen, E.C.-Y. Yang, Near time-optimal S-curve velocity planning for multiple line segments under axis constraints, IEEE Trans. Ind. Electron. 65 (12) (2018) 9582-9592.

[23] Y. Wang, T. Zhou, J. Zhou, M. Li, Z. Liang, X. Wang, Precision milling of integrated turbine cased on a non-contact on-machine measurement system, Nanomanufac. Metrol. 5 (4) (2022) 394-402.

[24] A. Gasparetto, V. Zanotto, A technique for time-jerk optimal planning of robot trajectories, Robot Comput. Integr. Manuf. 24 (3) (2008) 415-426.

[25] J. Huang, P. Hu, K. Wu, M. Zeng, Optimal time-jerk trajectory planning for industrial robots, Mech. Mach. Theory 121 (2018) 530-544.

[26] H. Liu, X. Lai, W. Wu, Time-optimal and jerk-continuous trajectory planning for robot manipulators with kinematic constraints, Robot Comput. Integr. Manuf. 29 (2) (2013) 309-317.

[27] E. Red, Dynamic optimal trajectory generator for Cartesian path following, Robotica 18 (5) (2000) 451-458.

[28] C.Y. Lin, J.Y. Wu, T.S. Lee, GSC-based frequency-domain equalizer for CP-free OFDM systems, in: IEEE International Conference on Communications, 2005, pp. 1132-1136.

[29] A. Olabi, R. Bearee, E. Nyiri, O. Gibaru, Enhanced trajectory planning for machining with industrial six-axis robots, in: 2010 IEEE International Conference on Industrial Technology, 2010, pp. 500-506.

[30] X. Huang, F. Zhao, T. Tao, X. Mei, A novel local smoothing method for five-axis machining with time-synchronization feedrate scheduling, IEEE Access 8 (2020) 89185-89204.

[31] X. Qu, M. Wan, C.-J. Shen, W.-H. Zhang, Profile error-oriented optimization of the feed direction and posture of the end-effector in robotic free-form milling, Robot Comput. Integr. Manuf. 83 (2023), 102580.

[32] A. Yuen, K. Zhang, Y. Altintas, Smooth trajectory generation for five-axis machine tools, Int. J. Mach. Tools Manuf 71 (2013) 11-19.

[33] Q. Liu, T. Huang, Inverse kinematics of a 5-axis hybrid robot with non-singular tool path generation, Robot Comput. Integr. Manuf. 56 (2019) 140-148.

[34] S. He, C. Yan, Y. Deng, C.-H. Lee, X. Zhou, A tolerance constrained G2 continuous path smoothing and interpolation method for industrial SCARA robots, Robot Comput. Integr. Manuf. 63 (2020), 101907.

[35] D. Yu, Z. Ding, X. Tian, X. Huang, Local corner smoothing algorithm for screw motor high-precision machining, Int. J. Adv. Manuf. Technol. 126 (5-6) (2023) 2117-2127.

[36] Q. Hu, Y. Chen, X. Jin, J. Yang, A real-time C-3 continuous local corner smoothing and interpolation algorithm for CNC machine tools, J. Manuf. Sci. Engin.-Transac. ASME 141 (4) (2019), 041004.

[37] J. Yang, A. Yuen, An analytical local corner smoothing algorithm for five-axis CNC machining, Int. J. Mach. Tools Manuf 123 (2017) 22-35.

[38] W. Wang, C. Hu, K. Zhou, S. He, L. Zhu, Local asymmetrical corner trajectory smoothing with bidirectional planning and adjusting algorithm for CNC machining, Robot Comput Integr Manuf 68 (2021).

[39] X. Zhao, H. Zhao, S. Wan, X. Li, H. Ding, An analytical decoupled corner smoothing method for five-axis linear tool paths, IEEE Access 7 (2019) 22763-22772.

[40] M.T. Lin, J.C. Lee, C.C. Shen, C.Y. Lee, J.T. Wang, Local corner smoothing with kinematic and real-time constraints for five-axis linear tool path, in: 2018 IEEE/ ASME International Conference on Advanced Intelligent Mechatronics (AIM), 2018, pp. 816-821.

[41] X. Huang, F. Zhao, T. Tao, X. Mei, A newly developed corner smoothing methodology based on clothoid splines for high speed machine tools, Robot Comput. Integr. Manuf. 70 (2021), 102106.

[42] H. Zhao, L. Zhu, H. Ding, A real-time look-ahead interpolation methodology with curvature-continuous B-spline transition scheme for CNC machining of short line segments, Int. J. Mach. Tools Manuf 65 (2013) 88-98.

[43] Q. Hu, Y. Chen, J. Yang, D. Zhang, An analytical C-3 continuous local corner smoothing algorithm for four-axis computer numerical control machine tools, J. Manuf. Sci. Engin.-Transac. ASME 140 (5) (2018), 051004.

[44] J. Peng, P. Huang, Y. Ding, H. Ding, An analytical method for decoupled local smoothing of linear paths in industrial robots, Robot Comput Integr Manuf 72 (2021), 102193.

[45] Q. Xiao, M. Wan, X. Qin, Y. Liu, W. Zhang, Real-time smoothing of G01 commands for five-axis machining by constructing an entire spline with the bounded smoothing error, Mech Mach Theory 161 (2021), 104307.

[46] K. Yang, S. Sukkarieh, An analytical continuous-curvature path-smoothing algorithm, IEEE Trans. Rob. 26 (3) (2010) 561-568.

[47] B. Sencer, K. Ishizaki, E. Shamoto, A curvature optimal sharp corner smoothing algorithm for high-speed feed motion generation of NC systems along linear tool paths, Int. J. Adv. Manuf. Technol. 76 (9) (2015) 1977-1992.

[48] Q. Bi, J. Shi, Y. Wang, L. Zhu, H. Ding, Analytical curvature-continuous dual-Bezier corner transition for five-axis linear tool path, Int. J. Mach. Tools Manuf 91 (2015) 96-108.

[49] M. Wan, X.-B. Qin, Q.-B. Xiao, Y. Liu, W.-H. Zhang, Asymmetrical pythagorean-hodograph (PH) spline-based C-3 continuous corner smoothing algorithm for five-axis tool paths with short segments, J. Manuf. Process 64 (2021) 1387-1411.

[50] Q. Hu, Y. Chen, X. Jin, J. Yang, A real-time C-3 continuous tool path smoothing and interpolation algorithm for five-axis machine tools, J. Manuf. Sci. Engin.-Transac. ASME 142 (4) (2020), 041002.

[51] J. Shi, Q. Bi, L. Zhu, Y. Wang, Corner rounding of linear five-axis tool path by dual PH curves blending, Int. J. Mach. Tools Manuf 88 (2015) 223-236.

[52] J. Yang, A. Adili, H. Ding, Real time tool path smoothing of short linear commands for robot manipulator by constructing asymmetrical Pythagoran-hodograph (PH) splines, Technol. Sci. 66 (3) (2023) 674-688.

[53] L. Zhang, K. Zhang, Y. Yan, Local corner smoothing transition algorithm based on double cubic NURBS for five-axis linear tool path, Strojniski Vestnik-J. Mech. Engineer. 62 (11) (2016) 647-656.

[54] M. Duan, C. Okwudire, Minimum-time cornering for CNC machines using an optimal control method with NURBS parameterization, Int. J. Adv. Manuf. Technol. 85 (5-8) (2016) 1405-1418.

[55] M.-T. Lin, M.-S. Tsai, H.-T. Yau, Development of a dynamics-based NURBS interpolator with real-time look-ahead algorithm, Int. J. Mach. Tools Manuf 47 (15) (2007) 2246-2262.

[56] R.-Y. Huang, C.-W. Cheng, A.-C. Lee, Parametric FIR filtering for G-code interpolation with corner smoothing and zero circular contour error for NC systems, Int. J. Adv. Manuf. Technol. 125 (9-10) (2023) 4379-4397.

[57] J. Fang, B. Li, H. Zhang, P. Ye, Real-time smooth trajectory generation for 3-axis blending tool-paths based on FIR filtering, Int. J. Adv. Manuf. Technol. 126 (7-8) (2023) 3401-3416.

[58] S. Tajima, B. Sencer, Accurate real-time interpolation of 5-axis tool-paths with local corner smoothing, Int. J. Mach. Tools Manuf 142 (2019) 1-15.

[59] S. Tajima, B. Sencer, Online interpolation of 5-axis machining toolpaths with global blending, Int. J. Mach. Tools Manuf 175 (2022), 103862.

[60] R.A. Ward, B. Sencer, B. Jones, E. Ozturk, Five-axis trajectory generation considering synchronization and nonlinear interpolation errors, J. Manuf. Sci. Engin.-Transac. ASME 144 (8) (2022), 081002.

[61] Y. Liu, M. Wan, X.-B. Qin, Q.-B. Xiao, W.-H. Zhang, FIR filter-based continuous interpolation of G01 commands with bounded axial and tangential kinematics in industrial five-axis machine tools, Int. J. Mech. Sci. 169 (2020), 105325.

[62] P.-Y. Tang, M.-T. Lin, M.-S. Tsai, C.-C. Cheng, Toolpath interpolation with novel corner smoothing technique, Robot Comput. Integr. Manuf 78 (2022), 102388.

[63] K. Ishizaki, E. Shamoto, A new real-time trajectory generation method modifying trajectory based on trajectory error and angular speed for high accuracy and short machining time, Precis. Eng. 76 (2022) 173-189.

[64] D.-N. Song, D.-W. Zheng, Y.-G. Zhong, J.-W. Ma, J.-S. Li, Non-isometric dual-spline interpolation for five-axis machine tools by FIR filtering-based feedrate scheduling using pseudo curvature under axial drive constraint, J. Manuf. Process 79 (2022) 827-843.

[65] H. Sun, J. Yang, D. Li, H. Ding, An on-line tool path smoothing algorithm for 6R robot manipulator with geometric and dynamic constraints, Technol. Sci. 64 (9) (2021) 1907-1919.

[66] L. Biagiotti, C. Melchiorri, FIR filters for online trajectory planning with time- and frequency-domain specifications, Control Eng. Pract. 20 (12) (2012) 1385-1399.

[67] M. Gautier, A. Janot, P.-O. Vandanjon, A new closed-loop output error method for parameter identification of robot dynamics, IEEE Trans. Control Syst. Technol. 21 (2) (2013) 428-444.

[68] J. Dong, J. Xu, Q. Zhou, J. Zhu, L. Yu, Dynamic identification of industrial robot based on nonlinear friction model and LS-SOS algorithm, IEEE Trans. Instrum. Meas. 70 (2021) 1-12.

[69] X. Tu, P. Zhao, Y.F. Zhou, Parameter identification of static friction based on an optimal exciting trajectory, Mater. Sci. Engin. 280 (1) (2017), 012025.

[70] Y. Han, J. Wu, C. Liu, Z. Xiong, An iterative approach for accurate dynamic model identification of industrial robots, IEEE Trans. Rob. 36 (5) (2020) 1577-1594.

[71] S. Tan, J. Yang, H. Ding, A prediction and compensation method of robot tracking error considering pose-dependent load decomposition, Robot Comput. Integr. Manuf. 80 (2023), 102476.

[72] T. Xu, J. Fan, Q. Fang, Y. Zhu, J. Zhao, Robot dynamic calibration on current level: modeling, identification and applications, Nonlin. Dyn 109 (4) (2022) 2595-2613.

<!-- Meanless: 21-->

