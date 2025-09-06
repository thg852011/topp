

<!-- Meanless: Journal of Computational Design and Engineering, 2022, 9, 2355-2374 OXFORD CDE DOI: 10.1093/jcde/qwac116 Advance access publication date: 1 November 2022 Research Article-->

# Optimal feedrate planning on a five-axis parametric tool path with global geometric and kinematic constraints

Hong-Yu Ma \( {}^{1,2} \) ,Chun-Ming Yuan \( {}^{1,2, * } \) ,Li-Yong Shen \( {}^{1, * } \) and Xiao-Shan Gao \( {}^{2} \)

\( {}^{1} \) School of Mathematical Sciences,University of Chinese Academy of Sciences,100049,Beijing,China

\( {}^{2} \) KLMM,Academy of Mathematics and Systems Sciences,CAS,100190,Beijing,China

*Corresponding author. E-mail: lyshen@ucas.ac.cn, cmyuan@mmrc.iss.ac.cn

## Abstract

The optimal feedrate planning problem for the five-axis parametric tool path remains challenging due to the nonlinear relationships between the joint space and the Cartesian space. We present a novel and complete optimal feedrate planning method for a five-axis parametric tool path by constraining the velocity, acceleration, jerk in the joint space, and the chord error in the Cartesian space. Our method formulates the problem as an optimal control problem, and we propose an iteration control vector parametrization method to compute the optimal solution. Compared with the new development five-axis feedrate planning methods, our solution satisfies "bang-bang" optimal control, and each constraint is strictly under the limits globally. Examples and comparisons with two other methods are demonstrated to show the effectiveness of the algorithm.

Keywords: feedrate planning, five-axis CNC, global constraint, bang-bang control

## 1. Introduction

Five-axis CNC (Computer Numerical Control) machining is becoming more popular in the manufacturing industry. Machining efficiency is one of the critical factors for the competitiveness of CNC machining. The machining time should be minimized as much as possible within the machine limits and quality. Hence, the time optimal feedrate planning for CNC machining is very important and has attracted the attention of both researchers and engineers.

The traditional five-axis linear tool paths (or G01 blocks) are discontinuous at the corners, which leads to feed fluctuation and poor machining quality. To overcome these drawbacks, some commercial CNC systems use parametric curves (such as splines and circular curves) to represent the tool paths. Then, an essential task is to find the minimum time feedrate planning (MTFP) on the five-axis parametric tool path. The goal of MTFP is to drive the machine tool along a predefined path under the constraints determined by machine capabilities and geometric accuracy, i.e., the axis kinematic constraints and the geometric error constraints. These constraints are closely related to the smooth movement of each axis and machining stability and quality. In this paper, we focus on optimal feedrate planning by constraining the velocity, acceleration, and jerk for each axis in the joint space, and constraining the chord error in the Cartesian space. Note that the constraints are in different spaces, and the mapping relationship between the axial joint space and the Cartesian space is nonlinear; hence, the five-axis MTFP problem is challenging.

The methods for the MTFP problem included synchronization methods, phase space analysis methods, and optimization based methods. The synchronization methods consist of two steps: (i) planning the linear trajectory according to linear motion constraints and angular trajectory according to rotation motion constraints separately and (ii) synchronizing the angular trajectory with the linear trajectory according to parameter (Fleisig & Spence, 2001; Jin et al., 2019) or time (Huang et al., 2018; Liu et al., 2020). This kind of method is robust and easy to implement, but may violate the angular kinematic constraints (e.g. the algorithm mentioned will cause the jerk of axis \( C \) to exceed the limit by 16.2% in Huang et al.,2018),which is unacceptable in CNC machining. The phase space analysis methods (Bobrow et al., 1985; Shin & McKay, 1985; Timar & Farouki, 2007; Zhang et al., 2012; Yuan et al., 2013) determine the active axis of each time duration according to the kinematic constraints. Unfortunately, the phase space analysis methods can only handle simple tool paths, such as linear tool paths. As a more general class of methods, the optimization based methods (Chen & Desrochers, 1990; Jamhour & Andrè, 1996; Erkorkmaz & Heng, 2008; Dong et al., 2007) can solve the MTFP problem more precisely. Based on various optimal principles (such as Pontryagin's maximum principle), the MTFP problem can be transformed into a time-optimal control problem with various constraints. However, the above optimization based methods deal with the 3-axis case or the 5-axis case but do not fully consider velocity, acceleration, and jerk constraints for each axis of five-axis machining.

In recent years, much work has focused on time optimal feedrate planning for five-axis parametric tool paths by respecting the kinematic constraints and geometric constraints. Huang et al. (2021) proposed a novel corner smoothing methodology based on clothoid splines for high-speed machine tools, which can effectively eliminate the feedrate fluctuations. However, they only considered the three-axis machine. Lu et al. (2020) mainly considered the tool-tip kinematic constraints in time-optimal tool motion planning; however, their algorithm works for joint robots, which is not often used in CNC machining. Li et al. (2020) proposed a novel neural network-based one-step trajectory smoothing method. Guo et al. (2021) presented a feedrate-planning method based on the critical constraint (curvature) curve of the feedrate. Tajima and Sencer (2020) developed a novel tool-path modification strategy to interpolate around kinematic singularities within part tolerances. However, the above three methods do not fully consider jerk constraints for each axis. Sencer et al. (2008) used the cubic B-Spline to represent the feedrate profile; however, they ignored the chord error constraint in the Cartesian space. Beudaert et al. (2012) and Xu et al. (2020) proposed the near time-optimal feedrate interpolation method with axis jerk constraints, which directly computes the interpolation points by intersecting all the constraints. These two methods strictly meet the kinematic constraints and geometric constraints. Note that the number of iterations of the methods is closely related to the curvature of the tool path, which is not robust and time consuming. Song and Ma (2019) proposed an interval partition-based five-axis feedrate scheduling method for balancing feed-motion efficiency and stability, under velocity, acceleration, and jerk axial constraints. However, none of the algorithms of Beudaert et al. (2012), Xu et al. (2020), and Song and Ma (2019) is time optimal.

---

<!-- Footnote -->

Received: July 6, 2022. Revised: October 16, 2022. Accepted: October 28, 2022

(C) The Author(s) 2022. Published by Oxford University Press on behalf of the Society for Computational Design and Engineering. This is an Open Access article distributed under the terms of the Creative Commons Attribution-NonCommercial License (https://creativecommons.org/licenses/by-nc/4.0/), which permits non-commercial re-use, distribution, and reproduction in any medium, provided the original work is properly cited. For commercial re-use, please contact journals.permissions@oup.com

<!-- Footnote -->

---

<!-- Meanless: ロンラスを見なるオラコスはありますようなこともございますができないです。では違っていることは-->




<!-- Meanless: 356 | Journal of Computational Design and Engineering, 2022, Vol. 9, No. 6 ロションをみなされる「子どなものをすることもらえるかもかものであないです。なかっているようなものだと-->

<!-- Media -->

<!-- figureText: 6000 X axis Y axis Z axis Boundary 0.5 0.6 0.7 0.8 0.9 \( \beta \) (a) Axial jerks just for sampling points. Y axis Z axis Boundary 0.5 0.6 0.7 0.8 0.9 1 \( \beta \) (b) The actual axial jerk curves. 4000 2000 Jerk(mm/s \( {}^{3} \) ) provinces -2000 -4000 -6000 0.1 0.2 0.3 0.4 6000 4000 2000 Jerk(mm/s \( {}^{3} \) ) -2000 -4000 -6000 0.1 0.2 0.3 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_1.jpg?x=379&y=138&w=957&h=1147&r=0"/>

Figure 1: The axial jerks with the method of Li et al. (2012). \( \beta \) is the parameter of the tool path.

<!-- Media -->

Sun et al. (2014) presented a novel feedrate scheduling method by using an initial B-spline curve to represent the feedrate profile, and then optimized and smoothed the feedrate curve by eliminating the regions that violated the prescribed constraints. However, the acceleration/deceleration from/to a specific feedrate at the start/end position is not considered. Fan et al. (2013) converted the MTFP problem to a linear programming (LP) program by using a linear function to approximate the nonlinear jerk constraint. However, they did not consider the velocity constraints for all five axes in the joint space. Li et al. (2012) proved that the optimal solution of the MTFP problem must be "bang-bang" control, and they converted the MTFP problem to a nonlinear programming (NLP) problem by discretizing the tool path into discrete sampling points. Furthermore, Liu et al. (2017) constructed an efficient and robust linear model for feedrate optimization with preset multiple constraints with discrete sampling points. Unfortunately, the methods in Li et al. (2012) and Liu et al. (2017) only constrain the sampling points, which causes the actual axial kinematics (i.e., velocity, acceleration, or jerk) profile to may exceed the preset limits (as shown in Fig. 1, when we use the method given in Li et al., 2012 to process the impeller tool path, the axial jerk curves formed by sampling points do not exceed the limits, while the actual jerk curves greatly exceed the limits, see the red circle). This means that these discretization methods control only the sampling points, not the entire tool path, which is unexpected in CNC machining. Therefore, planning the time optimal feedrate for five-axis parametric tool paths considering the kinematic constraints and geometric constraints is still challenging. In this paper, controlling the geometric and kinematic constraints of the entire parametric tool path is called global constraints.


<!-- Meanless: Journal of Computational Design and Engineering 2357 ロップもないなです。ご存在なものをするかものものですね？ですか？ですが、そのようにすると思いない-->

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_2.jpg?x=617&y=147&w=553&h=555&r=0"/>

Figure 2: The five-axis table-tilting type CNC machine.

<!-- Media -->

This paper studies the MTFP problem by constraining the axial velocity, acceleration, and jerk for each axis in the joint space and constraining the chord error in the Cartesian space. We describe the feedrate planning problem as an optimal control problem with constraints both in joint space and in the Cartesian space, and the optimal solution of this control problem is "bang-bang" control. Here the "bang-bang" control means that at least one of the velocity, acceleration, jerk, and chord error reaches its limit at any time. Then, we divide the parameter interval \( \left\lbrack  {0,1}\right\rbrack \) into several subintervals by fully considering the geometric characteristics of the tool path. Finally, a numerical iteration algorithm based on control vector parametrization (Büskens & Maurer, 2000) is proposed to transform the above control problem into an iterative solvable NLP problem. Meanwhile, the solution of this algorithm is "bang-bang" optimal control. In short, the content of this paper is how to use the optimization method to solve an optimal control problem, which is the main reason we call this work control. The efficiency of our approach is demonstrated with different examples.

In summary, our feedrate planning method for the five-axis parametric tool path gives two main contributions:

(i) We fully consider both geometric characteristics (curvature information), chord error constraint of the whole tool path and the kinematic constraints of the machine when planning the feedrate profile; and

(ii) Compared with the newly developed five-axis feedrate planning methods (Sencer et al., 2008; Beudaert et al., 2012; Li et al., 2012, 2020; Fan et al., 2013; Sun et al., 2014; Liu et al., 2017; Song & Ma, 2019; Tajima & Sencer, 2020; Xu et al., 2020; Guo et al., 2021), our solution satisfies the "bang-bang" optimal control and each constraint is strictly under its limit globally, which means we fully use the capacity of the machine.

The paper is arranged as follows. In Section 2, the MTFP problem is formulated as an optimal control problem. Section 3 presents the iteration control vector parametrization (ICVP) method to compute the optimal solution. Experimental results and a comparison with two methods are shown in Section 4. Finally, we conclude our paper in Section 5.

## 2. Problem Formulation

In this section, we consider the feedrate planning problem as a time optimal control problem. Note that the planning path is generally described in Cartesian space, i.e., the workpiece coordinate system (WCS). However, when we introduce the kinematic constraints of the machine, we have to consider the problem in the joint space, i.e., the machine coordinate system (MCS). Hence, we need to establish the transformation between the MCS and the WCS.

### 2.1. Transformation between the WCS and MCS

In the following paragraphs of this paper, we consider the five-axis table-tilting type CNC machine (Fig. 2) with three translational axes (X,Y,andZ)controlling the spindle position and two rotary axes(AandC)controlling the orientation of the workpiece. The workpiece is fixed onto a rotary table and this machine type is adopted by most CNC machining with the major advantage of having fewer loads being imposed on the spindle, leading to fewer tool vibrations.

The input of our discussion is the parametric tool path \( \mathbf{R}\left( \beta \right)  = \left\lbrack  {\mathbf{P}\left( \beta \right) ,\mathbf{O}\left( \beta \right) }\right\rbrack  ,\beta  \in  \left\lbrack  {0,1}\right\rbrack \) in the Cartesian space with at least \( {\mathrm{C}}^{2} \) continuity, such as NURBS or B-splines. Let \( \mathbf{P} = \left\lbrack  {{P}_{x},{P}_{y},{P}_{z}}\right\rbrack \) be the tool tip position locus and \( \mathbf{O} = \left\lbrack  {{O}_{i},{O}_{j},{O}_{k}}\right\rbrack \) be the locus of the tool axis orientation. Generally, for high-precision machining, people need to consider the axial kinematic constraints in the joint space and consider the chord error constraint in the Cartesian space. Therefore,we need to map the tool motion \( \mathbf{R}\left( \beta \right) \) into the axis drive motion \( \mathbf{Q} = \left\lbrack  {X,Y,Z\text{,}}\right\rbrack \) \( A,C\rbrack \) in the joint space,where \( X,Y \) ,and \( Z \) represent the movements of three translation axes and \( A \) and \( C \) are the two rotations. This transformation \( \varphi  : \mathbf{R} \rightarrow  \mathbf{Q} \) is called the inverse kinematics.

The inverse kinematics is based on the configuration of the selected machine. For the table-tilting machine in Fig. 2, the transformation can be obtained as follows (suppose that the origin of the two coordinate systems coincide and there are no singularity points in the


<!-- Meanless: 358 | Journal of Computational Design and Engineering, 2022, Vol. 9, No. 6 ロッティアをいなです。プラードを供を始めるようなものですね？なのではないのではないので、今に見らってつくてると言いだと-->

inverse kinematics transformation):

\[\left\{  \begin{array}{l} A\left( \beta \right)  = \arctan \left( \frac{\sqrt{{O}_{i}^{2}\left( \beta \right)  + {O}_{j}^{2}\left( \beta \right) }}{{O}_{k}\left( \beta \right) }\right) , \\  C\left( \beta \right)  = \arctan \left( \frac{{O}_{i}\left( \beta \right) }{{O}_{j}\left( \beta \right) }\right) , \end{array}\right.  \tag{1}\]

\[\left\lbrack  \begin{matrix} X\left( \beta \right) \\  Y\left( \beta \right) \\  Z\left( \beta \right)  \end{matrix}\right\rbrack   = \left\lbrack  \begin{matrix} \cos \left( {C\left( \beta \right) }\right) &  - \sin \left( {C\left( \beta \right) }\right) & 0 \\  \cos \left( {A\left( \beta \right) }\right) \sin \left( {C\left( \beta \right) }\right) & \cos \left( {A\left( \beta \right) }\right) \cos \left( {C\left( \beta \right) }\right) &  - \sin \left( {A\left( \beta \right) }\right) \\  \sin \left( {A\left( \beta \right) }\right) \sin \left( {C\left( \beta \right) }\right) & \sin \left( {A\left( \beta \right) }\right) \cos \left( {C\left( \beta \right) }\right) & \cos \left( {A\left( \beta \right) }\right)  \end{matrix}\right\rbrack  \left\lbrack  \begin{matrix} {P}_{x}\left( \beta \right) \\  {P}_{y}\left( \beta \right) \\  {P}_{z}\left( \beta \right)  \end{matrix}\right\rbrack  . \tag{2}\]

### 2.2. Kinematic constraints and chord error constraint

By using the transformation between the WCS and MCS, we give kinematic constraints for each axis in the joint space including the axial velocity, acceleration, and jerk constraints. Adding the chord error limit in the Cartesian space, we obtain all the constraints consisting of the kinematic constraints in the joint space and chord error constraint in the Cartesian space. According to the differential chain rule, the velocity \( \mathbf{v} \) ,acceleration \( \mathbf{a} \) ,and jerk \( \mathbf{j} \) profiles of the five axes can be expressed as

\[\mathbf{v}\left( t\right)  = \dot{\mathbf{Q}}\left( t\right)  = {\mathbf{Q}}^{\prime }\dot{\beta }\left( t\right) ,\]

\[\mathbf{a}\left( t\right)  = \ddot{\mathbf{Q}}\left( t\right)  = {\mathbf{Q}}^{\prime }\ddot{\beta }\left( t\right)  + {\mathbf{Q}}^{\prime \prime }{\dot{\beta }}^{2}\left( t\right) ,\]

\[\mathbf{j}\left( t\right)  = \dddot{\mathbf{Q}}\left( t\right)  = {\mathbf{Q}}^{\prime }\dddot{\beta }\left( t\right)  + 3{\mathbf{Q}}^{\prime \prime }\dot{\beta }\left( t\right) \ddot{\beta }\left( t\right)  + {\mathbf{Q}}^{\prime \prime \prime }{\dot{\beta }}^{3}\left( t\right) ,\]

where \( {\mathbf{Q}}^{\prime } = \frac{d\mathbf{Q}}{d\beta },{\mathbf{Q}}^{\prime \prime } = \frac{{d}^{2}\mathbf{Q}}{d{\beta }^{2}} \) ,and \( {\mathbf{Q}}^{\prime \prime \prime } = \frac{{d}^{3}\mathbf{Q}}{d{\beta }^{3}} \) and \( \dot{\beta }\left( t\right)  = \frac{\mathrm{d}\beta \left( t\right) }{\mathrm{d}t},\ddot{\beta }\left( t\right)  = \frac{{\mathrm{d}}^{2}\beta \left( t\right) }{\mathrm{d}{t}^{2}} \) ,and \( \ddot{\beta }\left( t\right)  = \frac{{\mathrm{d}}^{3}\beta \left( t\right) }{\mathrm{d}{t}^{3}} \) .

Considering the drive and torque performance, the purpose of reducing vibrations and smoothing the generated trajectory, the velocities, accelerations, and jerks of the five axes need to be bounded:

\[ - {\mathbf{v}}_{B} \leq  \mathbf{v} \leq  {\mathbf{v}}_{B}\]

\[ - {\mathbf{a}}_{\mathrm{B}} \leq  \mathbf{a} \leq  {\mathbf{a}}_{\mathrm{B}} \tag{3}\]

\[ - {\mathbf{j}}_{B} \leq  \mathbf{j} \leq  {\mathbf{j}}_{B}\]

where \( {\mathbf{v}}_{B} = {\left\lbrack  {v}_{xB},{v}_{yB},{v}_{zB},{v}_{zB},{v}_{aB},{v}_{cB}\right\rbrack  }^{\mathrm{T}},{\mathbf{a}}_{B} = {\left\lbrack  {a}_{xB},{a}_{yB},{a}_{zB},{a}_{cB},{a}_{cB}\right\rbrack  }^{\mathrm{T}} \) ,and \( {\mathbf{j}}_{B} = {\left\lbrack  {j}_{xB},{j}_{yB},{j}_{zB},{j}_{zB}\right\rbrack  }^{\mathrm{T}} \) are the velocity,acceleration,and jerk bound vectors, respectively. For each axis, the kinematic constraints can be expressed as

\[ - {v}_{lB} \leq  {v}_{l}\left( \beta \right)  = {l}^{\prime }\left( \beta \right) \sqrt{{\dot{\beta }}^{2}} \leq  {v}_{lB},\]

\[ - {a}_{lB} \leq  {a}_{l}\left( \beta \right)  = {l}^{\prime }\left( \beta \right) \ddot{\beta } + {l}^{\prime \prime }\left( \beta \right) {\dot{\beta }}^{2} \leq  {a}_{lB}, \tag{4}\]

\[ - {j}_{lB} \leq  {j}_{l}\left( \beta \right)  = \sqrt{{\dot{\beta }}^{2}}\left( {{l}^{\prime }\left( \beta \right) u + 3{l}^{\prime \prime }\left( \beta \right) \ddot{\beta } + {l}^{\prime \prime \prime }\left( \beta \right) {\dot{\beta }}^{2}}\right)  \leq  {j}_{lB},\]

where \( l = X,Y,Z,A,C \) and \( u = \frac{\widetilde{\beta }}{\beta } \) . In this way,the kinematic constraints can be expressed as inequalities with respect to \( {\dot{\beta }}^{2},\ddot{\beta } \) ,and \( u \) .

For the chord error constraint,we consider the chord error limit along the tool path \( \left\lbrack  {{P}_{x},{P}_{y}\text{,and}{P}_{z}}\right\rbrack \) in the Cartesian space,since the chord error is measured in the WCS and the mapping between the MCS and WCS is nonlinear. In the WCS, the curvature of the tool path is defined as

\[\kappa \left( \beta \right)  = \frac{\left| {\mathbf{P}}^{\prime }\left( \beta \right)  \times  {\mathbf{P}}^{\prime \prime }\left( \beta \right) \right| }{{\left| {\mathbf{P}}^{\prime }\left( \beta \right) \right| }^{3}}, \tag{5}\]

and the radius of curvature is \( \rho \left( \beta \right)  = \frac{1}{\kappa \left( \beta \right) } \) . By the chord error formula (Yeh & Hsu,2002),we can obtain

\[{v}_{\text{feed }}^{2}\left( \beta \right)  \leq  \frac{{8\delta \rho }\left( \beta \right)  - 4{\delta }^{2}}{{\tau }^{2}} \approx  \frac{{8\delta \rho }\left( \beta \right) }{{\tau }^{2}} = \frac{8\delta }{\kappa \left( \beta \right) {\tau }^{2}}, \tag{6}\]

where \( \tau \) is the interpolation period, \( \delta \) is the given chord error bound,and \( {v}_{\text{feed }} \) denotes the feedrate and is defined as

\[{v}_{\text{feed }} = \sqrt{{P}_{x}^{\prime 2} + {P}_{y}^{\prime 2} + {P}_{z}^{\prime 2}}\dot{\beta }. \tag{7}\]

We define the chord error at \( \beta \) as \( d\left( \beta \right) \) . Up to now,we have the chord error constraint

\[d\left( \beta \right)  = \frac{\kappa \left( \beta \right) {\tau }^{2}\left( {{P}_{x}^{\prime 2} + {P}_{y}^{\prime 2} + {P}_{z}^{\prime 2}}\right) {\dot{\beta }}^{2}}{8} \leq  \delta , \tag{8}\]

which can be expressed as inequalities with respect to \( {\dot{\beta }}^{2} \) .

### 2.3. Time optimal control problem

Summarizing the MTFP problem,our goal is to compute a parameter velocity \( \dot{\beta }\left( t\right) \) to minimize the time of tracking the tool path \( \mathbf{R}\left( \beta \right) \) so that the kinematic constraints and geometric constraints are satisfied. Therefore, the MTFP problem can be formulated as follows:

\[\mathop{\min }\limits_{{\ddot{\beta }\left( t\right) }}T = {\int }_{0}^{{t}_{\mathrm{f}}}1\mathrm{\;d}t = {\int }_{0}^{1}\frac{1}{\dot{\beta }}\mathrm{d}\beta \]

\[\text{s.t.}\frac{\mathrm{d}\beta }{\mathrm{d}t} = \dot{\beta }\left( t\right) ,\frac{\mathrm{d}\dot{\beta }}{\mathrm{d}t} = \ddot{\beta }\left( t\right) ,\frac{\mathrm{d}\ddot{\beta }}{\mathrm{d}t} = \dddot{\beta }\left( t\right) \text{,}\]

\[\beta \left( 0\right)  = 0,\dot{\beta }\left( 0\right)  = 0,\ddot{\beta }\left( 0\right)  = 0\text{,} \tag{9}\]

\[\beta \left( {t}_{\mathrm{f}}\right)  = 1,\dot{\beta }\left( {t}_{\mathrm{f}}\right)  = 0,\ddot{\beta }\left( {t}_{\mathrm{f}}\right)  = 0\text{,}\]

\[d\left( t\right)  \leq  \delta ,\; - {\mathbf{v}}_{B} \leq  \mathbf{v}\left( t\right)  \leq  {\mathbf{v}}_{B},\]

\[ - {\mathbf{a}}_{B} \leq  \mathbf{a}\left( t\right)  \leq  {\mathbf{a}}_{B},\; - {\mathbf{j}}_{B} \leq  \mathbf{j}\left( t\right)  \leq  {\mathbf{j}}_{B},\]

where \( {t}_{\mathrm{f}} \) is the total machining time when we drive the machine tool along a predefined path. Since Equation (9) is time dependent,it is difficult to solve this problem directly. For the convenience of calculation, we convert this problem to an equivalent time independent optimal control problem and then solve the problem with nonlinear optimization.


<!-- Meanless: Journal of Computational Design and Engineering 2359 りしますのないです！ヨー롭이후원장소금의 2018년 공장샵원 잊장안전 연구일 연-->

<!-- Media -->

<!-- figureText: Velocity \( {\beta }_{k - 1} \) \( {\beta }_{k} \) Upper bound 0 Lower bound -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_4.jpg?x=560&y=145&w=665&h=339&r=0"/>

Figure 3: The local curvature maximum point \( {\beta }_{c} \) may exceed the limit if we only control \( {\beta }_{k - 1},{\beta }_{k} \) .

<!-- Media -->

Construct a linear system by defining the state vector \( \mathbf{X} = {\left\lbrack  {\dot{\beta }}^{2},\ddot{\beta }\right\rbrack  }^{\mathrm{T}} \) :

\[{\mathbf{X}}^{\prime }\left( \beta \right)  = \left\lbrack  \begin{matrix} 2\ddot{\beta } \\  \frac{\ddot{\beta }}{\beta } \end{matrix}\right\rbrack   = \mathbf{{AX}} + \mathbf{B}u,\mathbf{A} = \left\lbrack  \begin{array}{ll} 0 & 2 \\  0 & 0 \end{array}\right\rbrack  ,\mathbf{B} = \left\lbrack  \begin{array}{l} 0 \\  1 \end{array}\right\rbrack  , \tag{10}\]

where \( u = \frac{\widetilde{\mu }}{\beta } \) is the control variable. According to the linear system (Equation 10),the integral of \( u \) is \( \ddot{\beta } \) ,and the integral of \( 2\ddot{\beta } \) is \( {\dot{\beta }}^{2} \) :

\[\ddot{\beta } = {\int }_{0}^{\beta }u\left( x\right) \mathrm{d}x,{\dot{\beta }}^{2} = {\int }_{0}^{\beta }2\ddot{\beta }\left( x\right) \mathrm{d}x. \tag{11}\]

Then, the kinematic constraints (Equation 4) and chord error constraint (Equation 8) can be expressed as the inequalities only with respect to control variable \( u \) .

Finally, the original problem (Equation 9) is converted into the following equivalent time independent optimal control problem (Li et al., 2012):

\[\mathop{\min }\limits_{u}T = {\int }_{0}^{1}\frac{1}{\dot{\beta }}\mathrm{d}\beta \]

\[\text{s.t.}{\mathbf{X}}^{\prime }\left( \beta \right)  = \mathbf{{AX}} + \mathbf{B}u,\mathbf{X}\left( 0\right)  = {\mathbf{X}}_{0},\mathbf{X}\left( 1\right)  = {\mathbf{X}}_{1}\text{,} \tag{12}\]

\[d\left( u\right)  \leq  \delta ,\; - {v}_{lB} \leq  {v}_{l}\left( u\right)  \leq  {v}_{lB},\]

\[ - {a}_{lB} \leq  {a}_{l}\left( u\right)  \leq  {a}_{lB},\]

\[ - {j}_{lB} \leq  {j}_{l}\left( u\right)  \leq  {j}_{lB},l = X,Y,Z,A,C,\]

where \( {\mathbf{X}}_{0} = {\mathbf{X}}_{1} = {\left\lbrack  0,0\right\rbrack  }^{T} \) . Since the system of this optimization problem is linear,we have the following fact,which is crucial in our optimal velocity planning procedure:

Lemma 1 (Li et al., 2012; Zhang et al., 2012). The optimal solution of this control problem is "bang-bang" control, i.e., at least one of the velocity, acceleration, jerk, and chord error reaches its limit at any time.

## 3. ICVP Method

In this section, we propose an ICVP method to solve the optimal control problem (Equation 12). This method consists of three steps: (i) dividing the parameter interval \( \left\lbrack  {0,1}\right\rbrack \) into several subintervals by fully considering the geometric characteristics of the tool path; (ii) converting the problem (Equation 12) into a solvable nonlinear optimization problem and computing the initial solution; (iii) and computing the optimal solution iteratively, which satisfies the kinematic constraints and chord error constraint.

### 3.1. Parameter subintervals

We first divide the parameter interval \( \left\lbrack  {0,1}\right\rbrack \) into \( {N}_{1} \) equal subintervals densely:

\[0 = {\beta }_{0} < {\beta }_{1} < \cdots  < {\beta }_{{N}_{1} - 1} < {\beta }_{{N}_{1}} = 1. \tag{13}\]

For each subinterval,we use a constant value to approximate the control variable \( u \) . Then,the approximate control \( u\left( \beta \right) ,\beta  \in  \left\lbrack  {0,1}\right\rbrack \) can be formulated as

\[u\left( \beta \right)  \mathrel{\text{:=}} {\widehat{u}}_{k}\text{,if}\beta  \in  \left\lbrack  {{\beta }_{k - 1},{\beta }_{k}}\right\rbrack  \text{.} \tag{14}\]

The above subintervals are trivial without considering the geometrical characteristics of the tool path. When a subinterval \( \left\lbrack  {{\beta }_{k - 1},{\beta }_{k}}\right\rbrack \) contains a local curvature maximum point,controlling the kinematic constraints at the sampling points \( {\beta }_{k - 1},{\beta }_{k} \) cannot control the kinematic constraints at the point that has local maximum curvature, because for this point, the velocity will normally be minimized (Yuan et al., 2013). As shown in Fig. 3, because the kinematic (velocity, acceleration, or jerk) profiles are nonlinear with respect to parameter \( \beta \) , when we only control the kinematic constraints at the sampling points \( {\beta }_{k - 1},{\beta }_{k} \) ,the local curvature maximum point \( {\beta }_{c} \) may exceed the limit.

Hence, we add all the local curvature maximum points of the tool path in the Cartesian space to the above sampling points (Equation 13). Here, we may reduce some extra computations when we consider the whole kinematic and chord error constraints. Assuming that the tool path has \( {N}_{2} \) local curvature maximum points,we rearrange all these sampling points in order by parameters from smallest to largest to obtain \( N \) subintervals:

\[0 = {\beta }_{0} < {\beta }_{1} < \cdots  < {\beta }_{\mathrm{N} - 1} < {\beta }_{\mathrm{N}} = 1, \tag{15}\]




<!-- Meanless: 2360 | Journal of Computational Design and Engineering, 2022, Vol. 9, No. 6 ロップEachをオコーズになるとなるとすることもうことでもございますのであるので、今と見らっつつろうとも-->

where \( N = {N}_{1} + {N}_{2} \) . For each new subinterval \( \left\lbrack  {{\beta }_{k - 1},{\beta }_{k}}\right\rbrack \) ,we give a constant control function \( {\widehat{u}}_{k} \) to describe the velocity profile. In this way, the solution we seek is the piecewise constant function.

For a regular curve,there are always a finite number of local curvature maximum points. The points satisfy the equation \( {\kappa }^{\prime }\left( \beta \right)  = \) 0. However, the curvature expression (Equation 5) is usually highly nonlinear, which makes the computation extremely difficult. Additionally, the root result depends on the initial value selection, and it is difficult to find all the roots. Hence, we compute them by an approximate discrete method.

Specifically,we uniformly sample \( m \) points for each subinterval \( \left\lbrack  {{\beta }_{i - 1},{\beta }_{i}}\right\rbrack \) in Equation (13) and choose the point \( \mathbf{P}\left( \widehat{\beta }\right) \) with the maximum curvature from these \( m \) points. If \( \kappa \left( \widehat{\beta }\right)  > \kappa \left( {\beta }_{i - 1}\right) \) and \( \kappa \left( \widehat{\beta }\right)  > \kappa \left( {\beta }_{i}\right) \) ,then we choose this point as the local curvature maximum point. Then, we traverse each subinterval in Equation (13) to obtain all the local curvature maximum points.

To obtain a good approximation of the local maximum curvature point, we discretize each subinterval into an interpolation level. Assuming that \( \tau \) is the interpolation period, \( {V}_{\text{feed }} \) is the maximum feedrate of the machine and \( {S}_{i} \) is the total length of the tool path in subinterval \( \left\lbrack  {{\beta }_{\mathrm{i} - 1},{\beta }_{\mathrm{i}}}\right\rbrack \) ,then the length of the two adjacent interpolation points can be bounded by \( {V}_{\text{feed }}\tau \) ,and the discrete number \( {m}_{\mathrm{i}} \) can be approximated as

\[{m}_{i} = \frac{{S}_{i}}{{V}_{\text{feed }}\tau }, \tag{16}\]

let \( m = \mathop{\max }\limits_{i}{m}_{i} \) ,then we can see that the length between two discrete adjacent points is bounded by \( {V}_{\text{feed }}\tau \) .

### 3.2. Problem transformation

We now convert the optimal control problem into a NLP problem by the piecewise constant control profile \( u\left( \beta \right) \) in Equation (14). According to differential relations (Equation 10),the \( \widetilde{\beta } \) profile is piecewise linear and the \( {\widehat{\beta }}^{2} \) profile is piecewise quadratic. Then,we give the particular representation of \( \ddot{\beta } \) and \( {\dot{\beta }}^{2} \) .

Let \( \Delta {\beta }_{k} = {\beta }_{k} - {\beta }_{k - 1} \) and \( \Xi  = \{ 1,2,\ldots ,N\} \) . From Equations (10) and (14),the values \( \ddot{\beta } \) and \( {\dot{\beta }}^{2} \) at the sampling point \( {\beta }_{k} \) are

\[{\ddot{\beta }}_{k} = {\widehat{u}}_{k}\left( {{\beta }_{k} - {\beta }_{k - 1}}\right)  + {\ddot{\beta }}_{k - 1} = \mathop{\sum }\limits_{{i = 1}}^{k}{\widehat{u}}_{i}\Delta {\beta }_{i},k \in  \Xi , \tag{17}\]

\[{\dot{\beta }}_{k}^{2} = 2{\int }_{{\beta }_{k - 1}}^{{\beta }_{k}}\left( {{\widehat{u}}_{k}\left( {\beta  - {\beta }_{k - 1}}\right)  + {\ddot{\beta }}_{k - 1}}\right) \mathrm{d}\beta  + {\dot{\beta }}_{k - 1}^{2}\]

\[ = \left\{  \begin{array}{l} {\widehat{u}}_{k}\Delta {\beta }_{k}^{2},k = 1. \\  {\widehat{u}}_{k}\Delta {\beta }_{k}^{2} + {2\Delta }{\beta }_{k}\mathop{\sum }\limits_{{i = 1}}^{{k - 1}}{\widehat{u}}_{i}\Delta {\beta }_{i} + {\dot{\beta }}_{k - 1}^{2},k \in  \Xi  \smallsetminus  \{ 1\} . \end{array}\right.  \tag{18}\]

Then,all the constraints in problem (Equation 12) can be expressed as inequalities with respect to control variables \( \widehat{\mathbf{u}} = \left( {{\widehat{u}}_{1},\ldots ,{\widehat{u}}_{N}}\right) \) .

When we ignore the jerk constraints in the joint space, minimizing the machining time is equivalent to maximizing the fee-drates (Zhang et al., 2013). Inspired by this conclusion, we convert the original optimal control problem (Equation 12) into the following NLP problem, which is also applied in the paper Liu et al. (2017):

\[\mathop{\max }\limits_{\widehat{\mathbf{u}}}{V}_{0} = \mathop{\sum }\limits_{{k = 1}}^{N}{v}_{\text{feed },k}^{2} = \mathop{\sum }\limits_{{k = 1}}^{N}{\sigma }_{k}^{2}{\dot{\beta }}_{k}^{2}\]

\[\text{s.t.} - {v}_{lB} \leq  {v}_{l}\left( {\beta }_{k}\right)  \leq  {v}_{lB}\text{,}\]

\[ - {a}_{lB} \leq  {a}_{l}\left( {\beta }_{k}\right)  \leq  {a}_{lB}\text{,} \tag{19}\]

\[ - {j}_{lB} \leq  {j}_{l}\left( {\beta }_{k}\right)  \leq  {j}_{lB}\text{,}\]

\[k \in  \Xi ,l = X,Y,Z,A,C\text{,}\]

\[d\left( {\beta }_{k}\right)  \leq  \delta ,k \in  \Xi ,\]

\[{\dot{\beta }}_{N}^{2} = 0,{\ddot{\beta }}_{N} = 0,\]

where \( {\sigma }_{k} = \sqrt{{P}_{x}^{\prime 2}\left( {\beta }_{k}\right)  + {P}_{y}^{\prime 2}\left( {\beta }_{k}\right)  + {P}_{z}^{\prime 2}\left( {\beta }_{k}\right) },k \in  \Xi \) . The velocity,acceleration,and jerk of each axis at \( {\beta }_{k} \) are as follows:

\[{v}_{l}\left( {\beta }_{k}\right)  = {l}^{\prime }\left( {\beta }_{k}\right) \sqrt{{\dot{\beta }}_{k}^{2}},\]

\[{a}_{l}\left( {\beta }_{k}\right)  = {l}^{\prime }\left( {\beta }_{k}\right) {\ddot{\beta }}_{k} + {l}^{\prime \prime }\left( {\beta }_{k}\right) {\dot{\beta }}_{k}^{2}, \tag{20}\]

\[{j}_{l}\left( {\beta }_{k}\right)  = \sqrt{{\dot{\beta }}_{k}^{2}\left( {{l}^{\prime }\left( {\beta }_{k}\right) {\widehat{u}}_{k} + 3{l}^{\prime \prime }\left( {\beta }_{k}\right) {\ddot{\beta }}_{k} + {l}^{\prime \prime \prime }\left( {\beta }_{k}\right) {\dot{\beta }}_{k}^{2}}\right) },\]

where \( l = X,Y,Z,A,C \) .

The NLP problem (Equation 19) can be solved numerically by a conventional gradient-based optimization technique such as sequential quadratic programming (SQP). We use SQP method mentioned in paper Büskens and Maurer (2000) to efficiently solve our nonlinear optimization problem. After that,we can get the initial solution \( \widehat{\mathbf{u}} \) of the problem (Equation 12).

#### 3.3.The optimal solution obtained by iterative calculation

In the above subsection, we convert the control problem into an NLP. However, this NLP only constrains the sampling points, not the entire parameter interval. In addition, the kinematic (velocity, acceleration, or jerk) profiles (Equation 20) are nonlinear with respect to parameter \( \beta \) . Therefore,the actual axial kinematic profiles may exceed the preset limits in some parameters,as shown in Fig. 1. Hence, we also need to constrain the points with respect to the maximum and minimum values of the kinematic and geometric error profiles of each subinterval.

We will present an iterative algorithm to achieve the solution. The above subsection has given a set of initial control variables \( \widehat{\mathbf{u}} \) . For each subinterval \( \left\lbrack  {{\beta }_{k - 1},{\beta }_{k}}\right\rbrack \) ,one can obtain the state values \( \widetilde{\beta } \) and \( {\widehat{\beta }}^{2} \) on these subintervals according to the differential relations (Equation 10) and \( \widehat{\mathbf{u}} \) :

\[{\ddot{\beta }}_{k}\left( \beta \right)  = {\widehat{u}}_{k}\left( {\beta  - {\beta }_{k - 1}}\right)  + {\ddot{\beta }}_{k - 1},\beta  \in  \left\lbrack  {{\beta }_{k - 1},{\beta }_{k}}\right\rbrack  , \tag{21}\]




<!-- Meanless: Journal of Computational Design and Engineering 2361 ロッションなどですご了承くなくなりますができずっとなりますができないでしょう。それは、マーマーチをすることがで-->

\[{\dot{\beta }}_{k}^{2}\left( \beta \right)  = 2{\int }_{{\beta }_{k - 1}}^{\beta }\left( {{\widehat{u}}_{k}\left( {\beta  - {\beta }_{k - 1}}\right)  + {\ddot{\beta }}_{k - 1}}\right) \mathrm{d}\beta  + {\dot{\beta }}_{k - 1}^{2}\]

\[ = \left\{  \begin{matrix} {\widehat{u}}_{k}{\beta }^{2}, & \beta  \in  \left\lbrack  {{\beta }_{0},{\beta }_{1}}\right\rbrack  . \\  {\widehat{u}}_{k}{\left( \beta  - {\beta }_{k - 1}\right) }^{2} + 2{\ddot{\beta }}_{k - 1}\left( {\beta  - {\beta }_{k - 1}}\right)  + {\dot{\beta }}_{k - 1}^{2}, & \\   & k \in  \Xi  \smallsetminus  \{ 1\} ,\beta  \in  \left\lbrack  {{\beta }_{k - 1},{\beta }_{k}}\right\rbrack  . \end{matrix}\right.  \tag{22}\]

According to Equations (4) and (8),the axial velocity,acceleration,and jerk profiles and chord error profile on \( \left\lbrack  {{\beta }_{k - 1},{\beta }_{k}}\right\rbrack \) are obtained. We can find the maximum and minimum values of axial velocity: \( \left\{  {{v}_{l}\left( {\beta }_{{vl}\max k}\right) ,{v}_{l}\left( {\beta }_{{vl}\min k}\right) }\right\} \) ,the maximum and minimum values of the axial acceleration: \( \left\{  {{a}_{i}\left( {\beta }_{\text{almak }}\right) ,{a}_{l}\left( {\beta }_{\text{almink }}\right) }\right\} \) ,the maximum and minimum values of the axial jerk: \( \left\{  {{j}_{l}\left( {\beta }_{\text{jimax }}\right) ,{j}_{l}\left( {\beta }_{\text{jimink }}\right) }\right\} \) ,and maximum value of chord error profile \( d\left( {\beta }_{dmaxk}\right) \) in this subinterval,where \( k \in  \Xi \) and \( l = X,Y,Z,A \) ,C. Adding the above parameters to the constraints,we obtain a new nonlinear programming (NNLP) problem (Equation 23):

\[\mathop{\max }\limits_{\overline{\mathbf{u}}}{\mathrm{V}}_{1} = \mathop{\sum }\limits_{{k = 1}}^{N}{\sigma }_{k}^{2}{\dot{\beta }}_{k}^{2}\]

\[\text{s.t.} - {v}_{lB} \leq  {v}_{l}\left( {\beta }_{k}\right)  \leq  {v}_{lB}, - {v}_{lB} \leq  {v}_{l}\left( {\beta }_{vlmaxk}\right)  \leq  {v}_{lB}, - {v}_{lB} \leq  {v}_{l}\left( {\beta }_{vlmink}\right)  \leq  {v}_{lB}\text{,}\]

\[ - {a}_{lB} \leq  {a}_{l}\left( {\beta }_{k}\right)  \leq  {a}_{lB},\; - {a}_{lB} \leq  {a}_{l}\left( {\beta }_{almaxk}\right)  \leq  {a}_{lB},\; - {a}_{lB} \leq  {a}_{l}\left( {\beta }_{almink}\right)  \leq  {a}_{lB}, \tag{23}\]

\[ - {j}_{lB} \leq  {j}_{l}\left( {\beta }_{k}\right)  \leq  {j}_{lB},\; - {j}_{lB} \leq  {j}_{l}\left( {\beta }_{jlmaxk}\right)  \leq  {j}_{lB},\; - {j}_{lB} \leq  {j}_{l}\left( {\beta }_{jlmink}\right)  \leq  {j}_{lB},\]

\[k \in  \Xi ,l = X,Y,Z,A,C\text{,}\]

\[d\left( {\beta }_{k}\right)  \leq  \delta ,d\left( {\beta }_{dmaxk}\right)  \leq  \delta ,k \in  \Xi ,\]

\[{\dot{\beta }}_{N}^{2} = 0,{\ddot{\beta }}_{N} = 0\text{.}\]

One will obtain a control profile \( \overline{\mathbf{u}} \) by solving this NNLP. We obtain a convergence coefficient \( \gamma \) ,which will be considered later. If the absolute value of \( {\mathrm{V}}_{1} \) from NNLP minus \( {\mathrm{V}}_{0} \) from NLP is less than \( \gamma \) ,i.e., \( \left| {{\mathrm{V}}_{1} - {\mathrm{V}}_{0}}\right|  \leq  \gamma \) ,we get the optimal solution \( \overline{\mathbf{u}} \) . If \( \left| {{\mathrm{V}}_{1} - {\mathrm{V}}_{0}}\right|  > \gamma \) ,let \( {\mathrm{V}}_{0} \) \( = {V}_{1} \) and put \( \overline{\mathbf{u}} \) into the Equations (21),(22),(4),and (8) to get the new axial velocity,acceleration,and jerk profiles and chord error profile. Then, we calculate the maximum and minimum values of the axial kinematic profiles and chord error profile for each subinterval. We recalculate NNLP to obtain the new value of the objective function \( {V}_{1} \) and new control profile \( \overline{\mathbf{u}} \) . If \( \left| {{V}_{1} - {V}_{0}}\right|  \leq  \gamma \) ,we obtain the optimal solution \( \overline{\mathbf{u}} \) ; otherwise,repeat the above steps until \( \left| {{V}_{1} - {V}_{0}}\right|  \leq  \gamma \) . The above procedure is given in Algorithm 1.

<!-- Media -->

Algorithm 1: ICVP algorithm.

---

Input:
the parametric tool path \( \mathbf{R}\left( \beta \right)  = \left\lbrack  {\mathbf{P}\left( \beta \right) ,\mathbf{O}\left( \beta \right) }\right\rbrack  ,\beta  \in  \left\lbrack  {0,1}\right\rbrack \) in the Cartesian space;
the kinematic bounds: \( {\mathbf{v}}_{B},{\mathbf{a}}_{B},{\mathbf{j}}_{B} \) . The chord error bound: \( \delta \) .
Output: the control profile: \( \overline{\mathbf{u}} \) .
1: Get the tool path \( Q = \left\lbrack  {X,Y,Z,A,C}\right\rbrack \) in the joint space by inverse kinematics (Equations 1 and 2).
2: Divide the parameter interval \( \left\lbrack  {0,1}\right\rbrack \) according to curvature information (see Section 3.1).
3: Assume that each subinterval has a constant control variable (see Equation 14).
4: Solve the NLP (Equation 19) to obtain the \( \widehat{\mathbf{u}} \) and the value of objective function \( {\mathrm{V}}_{0} \) by SQP.
	Compute the convergence coefficient: \( \gamma  = \frac{{V}_{0}}{{10}^{6}} \) .
		Let \( {V}_{1} \Leftarrow  {V}_{0},\zeta  \Leftarrow  \gamma  + 1 \) ,and \( \overline{\mathbf{u}} \Leftarrow  \widehat{\mathbf{u}} \) .
		while \( \zeta  > \gamma \) do
		Let \( {V}_{0} \Leftarrow  {V}_{1} \) .
9: Obtain the axial kinematic (velocity, acceleration and jerk) profiles (4) and chord error profile (8) by the control profile \( \overline{\mathbf{u}} \)
10: Calculate the maximum and minimum values of axial kinematic profiles and chord error profile for each subinterval.
11: Add the above parameters to the constraints to get a new NNLP problem.
		Solve the NNLP to get the \( \overline{\mathbf{u}} \) and the value of objective function \( {\mathrm{V}}_{1} \) by SQP.
			\( \zeta  \Leftarrow  \left| {{V}_{1} - {V}_{0}}\right| \) .
		end while
	5: return the optimal control profile: \( \overline{\mathbf{u}} \) .

---

<!-- Media -->

For the convergence coefficient \( \gamma \) ,we can obtain the value of objective function \( {V}_{0} \) by solving the NLP. We let \( \gamma  = \frac{{V}_{0}}{{10}^{8}} \) ,which means that if the change in the quadratic sum of the feedrates at all sampling positions is less than 1 over one million of the value of the objective function obtained by the NLP, we can say that the value of the objective function in the NNLP converges. For the step 10 in Algorithm 1, it is very difficult to calculate the maximum and minimum values of the axial kinematic profiles and chord error profile directly because the profiles are highly nonlinear. To simplify the calculation, we can use the Newtonian iterative method or the discrete method mentioned in Section 3.1.

In NLP (Equation 23), the objective function can be rewritten as

\[{V}_{1} = \mathop{\sum }\limits_{{k = 1}}^{N}{\sigma }_{k}^{2}{\dot{\beta }}_{k}^{2} = {\left( QU\right) }^{\mathrm{T}}\left( {QU}\right) , \tag{24}\]

where

\[U = {\left\lbrack  {\widehat{u}}_{1},{\widehat{u}}_{2},{\widehat{u}}_{3},\cdots ,{\widehat{u}}_{N}\right\rbrack  }^{T}, \tag{25}\]

and \( Q \) is the matrix as (Equation 26).


<!-- Meanless: 2362 | Journal of Computational Design and Engineering, 2022, Vol. 9, No. 6 ロップラを取れます。子供を入れる方も言われますのものものですができないでしょうと思っていることをしました-->

<!-- Media -->

<!-- figureText: (a) Start point End point (b) -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_7.jpg?x=599&y=146&w=561&h=742&r=0"/>

Figure 4: (a) The impeller to be processed (Li et al., 2012). (b) The tool path of the impeller.

Table 1. The kinematic limits of the five drive axes.

<table><tr><td>Axis</td><td>Velocity (unit/s)</td><td>Acceleration (unit/s2)</td><td>Jerk (unit/s \( {}^{3} \) )</td></tr><tr><td>\( \mathrm{X}/\mathrm{Y}/\mathrm{Z}\left( \mathrm{{mm}}\right) \)</td><td>250</td><td>500</td><td>3000</td></tr><tr><td>A/C (rad)</td><td>4</td><td>8</td><td>60</td></tr></table>

<!-- figureText: 80 0.5 0.6 0.7 0.9 1 70 Feedrate(mm/s) 40 20 10 0.1 0.2 0.3 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_7.jpg?x=598&y=1264&w=564&h=322&r=0"/>

Figure 5: The feedrate profile of the jet engine impeller curve.

<!-- Media -->

Hence, this problem is quadratic programming with quadratic constraints. The while loop in ICVP converges after finite iterative steps according to the following experiments (see Section 4.4 for details of the iteration data).

Remark 1. Review the five-axis feedrate planning methods of Li et al. (2012, 2020), Guo et al. (2021), Tajima and Sencer (2020), Sencer et al. (2008), Beudaert et al. (2012), Xu et al. (2020), Song and Ma (2019), Sun et al. (2014), and Liu et al. (2017). Methods of Li et al. (2020), Guo et al. (2021), Tajima and Sencer (2020), Sencer et al. (2008), Beudaert et al. (2012), Xu et al. (2020), Song and Ma (2019), and Sun et al. (2014) are not optimal since their solutions are not "bang-bang" control. The methods of Li et al. (2012) and Liu et al. (2017) can get optimal results in some sense; however, their solution may exceed the kinematic constraints (see Fig. 1) because they only control the sampling points. According to our simulations, the solution of our method satisfies "bang-bang" control, and the feedrate obtained by our method is optimal. Moreover, our solution does not exceed the given bounds globally.

To achieve the global control of kinematic capacity of each axis and geometric error, we use an iteration method that adaptively controls the local maximum(minimum) velocity, acceleration, and jerk for each axis and the local maximum chord error, which does not depend on the sampling scale. However, to satisfy the same constraints under given error bounds, the methods of Li et al. (2012) and Liu et al. (2017) need to discretize a very large number of parameter values, which is unacceptable due to the computational cost. Take the jet engine impeller curve (Fig. 4b) as an example,when we set the number of sampling points \( {N}_{1} \) to 200,the number of local curvature maximum points \( {\mathrm{N}}_{2} \) is 6 (see Section 4.2). And assume that we discretize each subinterval by 100 points to find the extreme points of


<!-- Meanless: Journal of Computational Design and Engineering 2363 ロションを見てきます。ご手になくなりますか？ということではないですがですがです。できなくこと・そのようです。そのような-->

<!-- Media -->

<!-- figureText: 300 Y axis Z axis 0.5 0.6 0.7 0.8 0.9 \( \beta \) A axis - - - C axis 0.5 0.6 0.7 0.8 0.9 X axis Y axis Z axis 0.5 0.8 0.9 200 velocity(mm/s) 100 -100 -200 -300 0.1 0.2 0.3 0.4 velocity(rad/s) 0.1 0.2 0.3 600 400 Acceleration(mm/s \( {}^{2} \) 200 200 -600 0.1 0.2 0.3 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_8.jpg?x=605&y=143&w=586&h=992&r=0"/>

Figure 6: Inactive constraints of the jet engine impeller curve.

<!-- Media -->

the kinematic and geometric error profiles. Then the number of constraint inequalities for the NNLP (Equation 23) in our method is at most 9651. However, if methods in Li et al. (2012) and Liu et al. (2017) want to have the same constraint effects, the number of constraints for the optimization problem will be increased to 331296, which is about 34 times larger than our method.

\[Q = \left\lbrack  \begin{matrix} {\sigma }_{1}\Delta {\beta }_{1}^{2} & 0 & 0 & \cdots & 0 \\  {\sigma }_{2}\left( {\Delta {\beta }_{1}^{2} + 2\mathop{\sum }\limits_{{i = 2}}^{3}\Delta {\beta }_{1}\Delta {\beta }_{i}}\right) & {\sigma }_{2}\Delta {\beta }_{2}^{2} & 0 & \cdots & 0 \\  {\sigma }_{3}\left( {\Delta {\beta }_{1}^{2} + 2\mathop{\sum }\limits_{{i = 2}}^{3}\Delta {\beta }_{1}\Delta {\beta }_{i}}\right) & {\sigma }_{3}\left( {\Delta {\beta }_{2}^{2} + 2\mathop{\sum }\limits_{{i = 3}}^{3}\Delta {\beta }_{2}\Delta {\beta }_{i}}\right) & {\sigma }_{3}\Delta {\beta }_{3}^{2} & \cdots & 0 \\  \vdots & \vdots & \vdots &  \ddots  & \vdots \\  {\sigma }_{N}\left( {\Delta {\beta }_{1}^{2} + 2\mathop{\sum }\limits_{{i = 2}}^{N}\Delta {\beta }_{1}\Delta {\beta }_{i}}\right) & {\sigma }_{N}\left( {\Delta {\beta }_{2}^{2} + 2\mathop{\sum }\limits_{{i = 3}}^{N}\Delta {\beta }_{2}\Delta {\beta }_{i}}\right) & {\sigma }_{N}\left( {\Delta {\beta }_{3}^{2} + 2\mathop{\sum }\limits_{{i = 4}}^{N}\Delta {\beta }_{3}\Delta {\beta }_{i}}\right) & \cdots & {\sigma }_{N}\Delta {\beta }_{N}^{2} \end{matrix}\right\rbrack  . \tag{26}\]

## 4. Simulations and Experimental Results

Our algorithms are implemented with MATLAB on a PC with RAM 16G and Intel® Core™ i5-6600k CPU 3.50 GHz, Windows 10, and all the results are obtained in this environment. We use the curve pieces of a jet engine impeller curve and an S-shape test specimen to test the feasibility of our algorithm. We also use our algorithm to interpolate a five-axis dual B-spline tool path (consisting of a bottom curve and a top curve) on a five-axis table-tilting type machine center with rotary axes \( A \) and \( C \) . In the following implementations,the interpolation sampling time \( \tau \) is set to 0.002 s and the chord error limit \( \delta \) is set to 0.125μm. The axis velocities and accelerations are set to zero at the start and end points of the tool path.

### 4.1. Simulations

A jet engine impeller curve and a part of an S-shape test specimen are used to test our algorithm under the constraints of axial velocity, acceleration, jerk constraints, and chord error bound. The kinematic limits of the five axes for these two workpieces are shown in Table 1.

##### 4.1.1.The jet engine impeller curve

The tool path \( \mathbf{R}\left( \beta \right) \) in the Cartesian space is shown in Fig. 4b. The red line shows the tool tip trajectory. The green lines show the tool orientation along the curve and the blue line describes the locus of a second point belonging to the tool cutter.


<!-- Meanless: 364 | Journal of Computational Design and Engineering, 2022, Vol. 9, No. 6 ってきることですご了言葉を取るものをすることものできるかもみですね。まずには、今でくるとは-->

<!-- Media -->

<!-- figureText: 10 C axis 0.5 0.7 0.8 0.9 X axis Y axis Z axis Boundary 0.5 0.7 0.8 0.9 A axis 0.5 0.6 0.7 ---Boundary -10 \( {}^{\prime } \) 0 0.1 0.3 0.4 3000 2000 Jerk(mm/s \( {}^{3} \) ) 1000 -1000 -2000 -3000 0 0.1 0.3 0.4 80 -60 0.3 0.14 0.12 Chord error( \( \mu \mathrm{m} \) ) 0.1 0.08 0.06 0.02 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_9.jpg?x=588&y=142&w=592&h=1323&r=0"/>

Figure 7: Active constraints of the jet engine impeller curve.

<!-- figureText: Start point End point (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_9.jpg?x=652&y=1549&w=458&h=626&r=0"/>

Figure 8: (a) The S-shape test specimen (Wang et al., 2015). (b) The tool path of the S-shape test specimen.




<!-- Meanless: Journal of Computational Design and Engineering 2365 ロションを出来るコープローなどを知らせる方もとなっていることができるのではないでしょうと思ってくるですね。このようです。そのよう-->

<!-- figureText: 120 0.5 0.6 0.7 0.8 0.9 1 \( \beta \) 100 Feedrate(mm/s) 20 0.2 0.3 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_10.jpg?x=610&y=143&w=573&h=326&r=0"/>

Figure 9: The feedrate profile of the S-shape.

<!-- figureText: 300 Z axis β A axis C axis 0.5 0.6 0.7 0.8 0.9 \( \beta \) 200 /elocity(mm/s) 100 100 -200 -300 velocity(rad/s) 0.1 0.2 0.3 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_10.jpg?x=607&y=597&w=579&h=660&r=0"/>

Figure 10: Inactive constraints of the S-shape.

<!-- Media -->

The simulation results are shown in the following figures. Here, active constraint is defined as a constraint that at least one parameter point reaches the boundary of this constraint when machining a tool path represented by parametric curves. On the contrary, inactive constraint is defined as a constraint that never reaches its boundary. And these definitions are used to describe the simulation results. Figure 5 shows the feedrate profile of the impeller curve. Figure 6 shows the inactive constraints (i.e., five axes velocity constraints and \( X,Y \) ,and \( Z \) axis acceleration constraints) that never reach their bounds during the motion. Figure 7 lists the active constraints (i.e.,A and \( C \) axis acceleration,five axes jerk constraints,and chord error constraint),and at least one of them reaches its bound in a moment, i.e., our method satisfies "bang-bang" control.

#### 4.1.2. S-shape test specimen curve

The S-shape test specimen (Fig. 8a) can reflect the comprehensive accuracy and dynamic response characteristics of each moving part in multi-axis CNC machining, and it can be used to test the performance of the five-axis machine (Wang et al., 2015). Hence, we select a portion of the S-shape, represented by cubic B-splines, to test our algorithm. As shown in Fig. 8b, the blue line denotes the tool tip trajectory, and green arrows show the tool orientation along the path.

In the simulation results, Fig. 9 shows the feedrate profile of the S-shape. Figures 10 and 11 show the inactive constraints (i.e., five axes velocity constraints and \( Z \) and \( A \) axis acceleration,jerk constraints) that never reach their bounds during the motion. Figure 12 lists the active constraints (i.e., X, Y, and C axis acceleration, jerk constraints, and chord error constraint) that reach their bounds during the motion.

We can see that none of the velocity, acceleration, and jerk constraints or chord error bound is violated. As shown in Fig. 12, the active constraints show that the solution of our method satisfies "bang-bang" control, which means that we ultimate the capacity of the machine.

### 4.2. Simple criteria for sampling

In this subsection,we determine the number of equal subintervals \( {N}_{1} \) and hence \( {N}_{2} \) . In theory,we always want \( {N}_{1} \) to be as large as possible,so that each subinterval \( \left\lbrack  {{\beta }_{k - 1},{\beta }_{k}}\right\rbrack \) in Equation (13) contains at most one local curvature maximum point. Thus,we can get an accurate number of local curvature maximum points \( {N}_{2} \) through the discrete method in Section 3.1. However,too many sampling points will inevitably increase the computational cost in solving the NLP. Because for each additional sampling point, we need to add at least 16 constraint inequalities to the optimization problem (Equation 23).


<!-- Meanless: 366 | Journal of Computational Design and Engineering, 2022, Vol. 9, No. 6-->

<!-- Media -->

<!-- figureText: 600 Z axis 0.5 0.7 0.8 0.9 A axis 0.5 0.7 0.8 0.9 Z axis -Boundary 0.5 0.6 0.7 400 Acceleration(mm/s \( {}^{2} \) ) 200 200 -400 -600 0 0.1 0.3 0.4 0 0.1 0.3 0.4 4000 3000 2000 Jerk(mm/s \( {}^{3} \) ) 1000 1000 -2000 -300C -4000 0.3 60 20 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_11.jpg?x=586&y=140&w=594&h=1327&r=0"/>

Figure 11: Inactive constraints of the S-shape.

<!-- Media -->

Therefore,we need to select an appropriate number of sampling points \( {N}_{1} \) ,so that we can obtain the accurate \( {N}_{2} \) and computational burden can be avoided. For the above two tool paths, we sample 100, 150, 200, 300, and 400 points for simulation. As shown in Figs. 13 and 14,Tables 2 and \( 3,{N}_{2} \) ,the computational time and machining time are closely related to the number of sampling points. When we set \( {N}_{1} \) to 200,the number of local curvature maximum points \( {N}_{2} \) in both two paths does not increase when we increase \( {N}_{1} \) . When the number of sampling points exceeds 200 , the computational time of the two paths increases rapidly, while the machining time decreases slowly. Considering both accuracy and efficiency,the best number of sampling points \( {N}_{1} \) is the minimum number for which \( {N}_{2} \) does not increase. In our simulation, when we sample 200 points, the average arc length between the sampling points of the jet engine impeller curve and the S-shape curve are 0.6641 and \( {0.6917}\mathrm{\;{mm}} \) ,respectively. Thus,we can set a simple criterion for selecting the number of sampling points such that the average arc length is not greater than \( {0.7}\mathrm{\;{mm}} \) .

As we can see, the average arc length between the sampling points is determined according to the simulation results. This value may not be appropriate for more dramatically changing tool paths. And it is not very closely related to the characteristics of the specific tool path. So, in the future work, we will give a more accurate selection criterion for the number of sampling points.

Firstly, according to chord error constraint, we perform pre-interpolation (Li et al., 2022) using Equation (27), and all the local feedrate minimum points,which are local curvature maximum points,can be determined. Then we can get the accurate \( {\mathrm{N}}_{2} \) for a particular tool

<!-- Meanless: ロッチョンを見てきますよね！私も言うことものできるかものであるかなからないでしょ！！！-->




<!-- Meanless: Journal of Computational Design and Engineering 2367 してもう意味をきっう言われますよう言われますよね！このものですがですか？なので、今回はなりです。ですね。そうです。そうも、-->

<!-- Media -->

<!-- figureText: 600 X axis \( \beta \) -Boundary 0.5 0.6 0.8 0.9 X axis 0.5 0.6 0.8 0.9 0.6 0.8 0.9 ICVP Boundan 400 Acceleration(mm/s \( {}^{2} \) ) 200 -600 Acceleration(rad/s \( {}^{2} \) ) 0.1 0.2 0.4 4000 3000 1000 1000 -2000 -3000 -4000 0.1 0.2 0.4 Jerk(rad/s \( {}^{3} \) ) 20 -20 -40 -80 0.1 0.2 0.4 0.14 0.12 Chord error \( \left( {\mu \mathrm{m}}\right) \) 0.1 0.08 0.06 0.04 0.02 0.2 0.3 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_12.jpg?x=600&y=138&w=596&h=1667&r=0"/>

<!-- Media -->

Figure 12: Active constraints of the S-shape.

<!-- Media -->

Table 2. Planning results for the jet engine impeller curve.

<table><tr><td>Number of points \( {\mathrm{N}}_{1} \)</td><td>\( {\mathrm{N}}_{2} \)</td><td>Computational time (min)</td><td>Machining time (s)</td></tr><tr><td>100</td><td>4</td><td>4.17</td><td>3.426</td></tr><tr><td>150</td><td>5</td><td>9.47</td><td>3.342</td></tr><tr><td>200</td><td>6</td><td>14.62</td><td>3.246</td></tr><tr><td>300</td><td>6</td><td>25.84</td><td>3.197</td></tr><tr><td>400</td><td>6</td><td>40.11</td><td>3.158</td></tr></table>




<!-- Meanless: 2368 | Journal of Computational Design and Engineering, 2022, Vol. 9, No. 6 ロションをなどのコンコーパでもないという方々となることができるかといないのではないことでしましょうってろうところにない-->

Table 3. Planning results for the S-shape curve.

<table><tr><td>Number of points \( {\mathrm{N}}_{1} \)</td><td>\( {\mathrm{N}}_{2} \)</td><td>Computational time (min)</td><td>Machining time (s)</td></tr><tr><td>100</td><td>14</td><td>6.55</td><td>14.71</td></tr><tr><td>150</td><td>18</td><td>10.13</td><td>12.331</td></tr><tr><td>200</td><td>21</td><td>17.87</td><td>11.7</td></tr><tr><td>300</td><td>21</td><td>35.24</td><td>11.61</td></tr><tr><td>400</td><td>21</td><td>55.14</td><td>11.55</td></tr></table>

<!-- figureText: 80 100 2.5 time(s) 70 Feedrate(mm/s) 50 30 20 1 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_13.jpg?x=579&y=522&w=596&h=339&r=0"/>

Figure 13: The feedrate profiles of the jet engine impeller curve at different numbers of sampling points.

<!-- figureText: 120 -200 ...400 15 100 Feedrate(mm/s) -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_13.jpg?x=576&y=977&w=603&h=339&r=0"/>

Figure 14: The feedrate profile of the S-shape curve at different numbers of sampling points.

Table 4. Kinematic limits of the interpolation experiments.

<table><tr><td>Axis</td><td>Velocity (unit/s)</td><td>Acceleration (unit/s2)</td><td>Jerk (unit/s \( {}^{3} \) )</td></tr><tr><td>\( \mathrm{X}/\mathrm{Y}/\mathrm{Z}\left( \mathrm{{mm}}\right) \)</td><td>100</td><td>500</td><td>3000</td></tr><tr><td>A (rad)</td><td>0.4</td><td>0.5</td><td>1.5</td></tr><tr><td>C (rad)</td><td>0.8</td><td>0.5</td><td>1.5</td></tr></table>

<!-- Media -->

path and several parameter intervals.

\[{\beta }_{i + 1} = {\beta }_{i} + \frac{{v}_{i}\tau }{{\begin{Vmatrix}\frac{d\mathbf{P}\left( \beta \right) }{d\beta }\end{Vmatrix}}_{\beta  = {\beta }_{i}}} - \frac{\left( \frac{d\mathbf{P}\left( \beta \right) }{d\beta }\frac{{d}^{2}\mathbf{P}\left( \beta \right) }{d{\beta }^{2}}\right) }{2{\begin{Vmatrix}\frac{d\mathbf{P}\left( \beta \right) }{d\beta }\end{Vmatrix}}_{\beta  = {\beta }_{i}}^{4}}{v}_{i}^{2}{\tau }^{2}, \tag{27}\]

where \( \tau \) is the interpolation period and \( {v}_{i} \) is the feedrate of the current interpolation point \( {\beta }_{i} \) ,which is determined by Equation (6).

Secondly,assuming that the total number of the pre-interpolation points is \( {N}_{t} \) ,in order to control the size of the optimization problem, we set the number of sampling points is about 200 . Then, for each subinterval, we can get the appropriate sampling points by a scale \( {N}_{t}/{200} \) . Assuming the pre-interpolation time of this subinterval is \( {N}_{i}\tau \) ,then the number of sampling points in the subinterval is set to the round of \( {200}{\mathrm{\;N}}_{\mathrm{f}}/{\mathrm{N}}_{\mathrm{f}} \) ,and the sampling points can be given in an average way,that is,the number of pre-interpolation points between sampling points is the same.

For a long tool path, the number of sampling points generated by the simple criterion may be too large, leading to a long computational time. In this case, the long tool path can be divided into several short tool paths. Feedrate planning is performed for each short tool path separately, and then their feedrate curves can be connected together using a parallel windowing algorithm in Erkorkmaz et al. (2017).


<!-- Meanless: Journal of Computational Design and Engineering 2369 ロッションを知ってきました。私もいましょうこともらってもらっていることができる方々にしています。そのよう-->

<!-- Media -->

<!-- figureText: Start point End point -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_14.jpg?x=693&y=146&w=411&h=257&r=0"/>

Figure 15: Flank milling tool path.

<!-- figureText: 20 - -BECKHOFF LP ICVP 15 20 time(s) Feedrate(mm/s) 10 10 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_14.jpg?x=591&y=499&w=609&h=346&r=0"/>

Figure 16: The feedrate profile of the B-spline tool path.

<!-- figureText: (a) (b) (c) (e) (f) (d) -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_14.jpg?x=197&y=946&w=1397&h=658&r=0"/>

Figure 17: Machining experiments of three methods: (a) ICVP; (b) BECKHOFF; (c) LP; and (d-f) magnified views of the three methods.

Table 5. Computational time of the three methods on dual B-spline flank milling.

<table><tr><td/><td>BECKHOFF</td><td>LP</td><td>ICVP</td></tr><tr><td>Computational time (s)</td><td>\( < 1 \)</td><td>1.667</td><td>904.145</td></tr></table>

<!-- Media -->

### 4.3. Experimental results

In this subsection, we compare our method with two other methods given by the commercial software BECKHOFF and the newly developed optimal feedrate method, Liu et al. (2017), based on LP. We use a five-axis dual B-spline tool path, i.e., the flank milling tool path mentioned in Beudaert et al. (2012), to conduct the experiments. The control points and knot vector are given in Appendix 1. As shown in Fig. 15, the red bottom line defines the tool tip trajectory \( \mathbf{P}\left( \beta \right) \) , and the blue top line defines the locus \( \mathbf{H}\left( \beta \right) \) of a second point belonging to the tool cutter. The unit vector \( \mathbf{O}\left( \beta \right) \) of tool orientation can be obtained as follows:

\[\mathbf{O}\left( \beta \right)  = \frac{\mathbf{H}\left( \beta \right)  - \mathbf{P}\left( \beta \right) }{\parallel \mathbf{H}\left( \beta \right)  - \mathbf{P}\left( \beta \right) \parallel }, \tag{28}\]




<!-- Meanless: 2370 70 | Journal of Computational Design and Engineering, 2022, Vol. 9, No. 6 ロションを4年2月ご了にすることを知られるということもございますのであないでしょうか？こうですね。このようです。このよう-->

<!-- Media -->

<!-- figureText: BECKHOFF -BECKHOFF LP velocity(mm/s) LP 50 ICVP Boundary -50 ICVP Boundary -100 15 20 25 0 5 10 15 20 25 time(s) time(s) _____。 Acceleration(mm/s \( {}^{2} \) ) 500 ) ---------------------------------. 500 25 0 10 25 time(s) \( \times  {10}^{4} \) Jerk(mm/s \( {}^{3} \) ) 15 20 25 10 15 20 25 time(s) time(s) 4000 _____ Jerk(mm/s3) 2000 -2000 15 20 25 10 15 20 25 time(s) time(s) (b) \( Y \) -axis. (c) \( z \) -axis. velocity(mm/s) LP velocity(mm/s) 50 -50 ICVP Boundary -50 -100 -100 0 5 10 15 20 25 0 5 10 time(s) Acceleration(mm/s \( {}^{2} \) ) 500 Acceleration(mm/s \( {}^{2} \) ) 500 -500 -500 15 25 0 10 time(s) \( \times  {10}^{4} \) \( \times  {10}^{4} \) Jerk(mm/s \( {}^{3} \) ) Jerk(mm/s \( {}^{3} \) ) 10 15 20 25 10 time(s) 4000 4000 _____ Jerk(mm/s \( {}^{3} \) ) 2000 Jerk(mm/s \( {}^{3} \) ) 2000 -2000 -2000 0 15 20 25 10 time(s) (a) \( x \) -axis. -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_15.jpg?x=199&y=146&w=1334&h=937&r=0"/>

Figure 18: Constraint verification on the \( X,Y \) ,and \( Z \) axis. The fourth row is a partial enlargement of the jerk profile. For simplicity,each column shares the same legend.

<!-- figureText: 0.14 ICVP Boundary the [2]. time(s) 0.12 Chord error \( \left( {\mu \mathrm{m}}\right) \) 0.1 0.04 0.02 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_15.jpg?x=565&y=1204&w=624&h=354&r=0"/>

Figure 19: Chord profile of the B-spline tool path.

<!-- Media -->

which are shown by the green lines in Fig. 15. And the normal in the above equation is the \( {L}_{2} \) normal. The velocity,acceleration,and jerk bounds of the five axes \( X,Y,Z,A \) ,and \( C \) ,are given in Table 4.

When the feedrate profile is obtained offline, to implement our method in CNC controllers, we can use the velocity curves as parts of the input to the CNC controllers to achieve real-time interpolation (see Yuan et al., 2013). We use the BECKHOFF spline interpolation method and LP method for comparison with our method.

The BECKHOFF method is tested under the axial velocity, acceleration constraints in the joint space, and chord error constraint in the Cartesian space. The ICVP method and LP method are tested under the axial velocity, acceleration, and jerk constraints in the joint space and chord error constraints in the Cartesian space. The three methods have the same axial velocity, acceleration constraints (Table 4), and chord error constraint (0.125 \( \mu \mathrm{m} \) ) with the same machine set-up configuration (the type of cutter,speed of main shaft,cutting depth, and cutting width). In addition, the LP method and ICVP method have the same axial jerk constraint (Table 4). Note that the BECKHOFF spline interpolation method has no jerk constraints, which means that the BECKHOFF algorithm is less restrictive than our algorithm and the LP method. For the BECKHOFF method and ICVP method, we only compare the other information except the jerk curves. In the following figures, the results of the three methods (BECKHOFF, LP, and ICVP) are shown as red lines, green lines, and blue lines, respectively. Precisely, Fig. 16 shows the feedrate profiles with respect to machining time; Figs. 18 and 20 show the kinematic (velocity, acceleration,and jerk) profiles of the \( X,Y,Z,A \) ,and \( C \) axes with respect to machining time.

In order to satisfy the "bang-bang" optimal control, our method requires solving the nonlinear optimization problem several times, which leads to a long computational time. As for the other two methods mentioned in this paper, LP method transforms the feedrate planning problem into a LP problem, and BECKHOFF method does not consider the jerk constraints of each axis. So, compared with our approach, the computational time of these two methods is very short (as shown in Table 5); however, the "bang-bang" optimal control cannot be achieved.


<!-- Meanless: Journal of Computational Design and Engineering 2371 してもう意味をきって、言っきりますよね？また、自分を見てきたりますので、今回は4月のお客様のごくだされ、またので、今ですよね-->

<!-- Media -->

<!-- figureText: velocity(rad/s) 0.5 BECKHOFF ICVP — Boundary 20 25 time(s) time(s) 15 20 25 time(s) 20 25 time(s) (a) A-axis. BECKHOFF Boundary 20 25 time(s) time(s) 15 20 25 time(s) 20 25 time(s) (b) \( c \) -axis. 0 -0.5 0 5 10 Acceleration(rad/s \( {}^{2} \) ) 0.5 0 -0.5 500 Jerk(rad/s \( {}^{3} \) ) 0 -500 5 10 Jerk(rad/s \( {}^{3} \) ) 0 5 10 velocity(rad/s) 0.5 0 -1 0 5 10 Acceleration(rad/s \( {}^{2} \) ) 0.5 -0.5 500 Jerk(rad/s \( {}^{3} \) ) 0 5 10 Jerk(rad/s \( {}^{3} \) 0 5 10 -->

<img src="https://cdn.noedgeai.com/bo_d2slb3bef24c73b2kg10_16.jpg?x=675&y=138&w=418&h=1793&r=0"/>

Figure 20: Constraint verification on the \( A \) and \( C \) axis. The fourth row is a partial enlargement of the jerk profile. For simplicity,each axis's line graph shares the same legend.




<!-- Meanless: 2372 | Journal of Computational Design and Engineering, 2022, Vol. 9, No. 6-->

Table 6. The iteration data of the three tool paths.

<table><tr><td rowspan="2">Iteration number</td><td colspan="2">Impeller curve \( \left( {\gamma  = {2.17}}\right) \)</td><td colspan="2">S-shape \( \left( {\gamma  = {3.15}}\right) \)</td><td colspan="2">Dual B-spline \( \left( {\gamma  = {0.04}}\right) \)</td></tr><tr><td>\( {\mathrm{V}}_{\mathrm{i}} \)</td><td>\( \zeta \)</td><td>\( {\mathrm{V}}_{\mathrm{i}} \)</td><td>\( \zeta \)</td><td>\( {\mathrm{V}}_{\mathrm{i}} \)</td><td>\( \zeta \)</td></tr><tr><td>0</td><td>2171661.35</td><td/><td>3154867.92</td><td/><td>44598.121</td><td/></tr><tr><td>1</td><td>2101430.21</td><td>70231.14</td><td>3123347.64</td><td>31520.28</td><td>42078.905</td><td>2519.216</td></tr><tr><td>2</td><td>2101467.11</td><td>36.9</td><td>3099057.92</td><td>24289.72</td><td>42027.188</td><td>51.717</td></tr><tr><td>3</td><td>2101451.27</td><td>15.84</td><td>3108 147.24</td><td>9089.32</td><td>42023.416</td><td>3.772</td></tr><tr><td>4</td><td>2101461.38</td><td>10.11</td><td>3091269.87</td><td>16877.37</td><td>42025.349</td><td>1.933</td></tr><tr><td>5</td><td>2101453.59</td><td>7.79</td><td>3095468.81</td><td>4198.94</td><td>42024.124</td><td>1.225</td></tr><tr><td>6</td><td>2101458.34</td><td>4.75</td><td>3096723.59</td><td>1254.78</td><td>42024.819</td><td>0.695</td></tr><tr><td>7</td><td>2101453.62</td><td>4.72</td><td>3096 357.61</td><td>365.98</td><td>42024.255</td><td>0.564</td></tr><tr><td>8</td><td>2101456.24</td><td>2.62</td><td>3092 278.49</td><td>4079.12</td><td>42024.076</td><td>0.179</td></tr><tr><td>9</td><td>2101455.15</td><td>1.09</td><td>3090705.93</td><td>1572.56</td><td>42023.799</td><td>0.277</td></tr><tr><td>10</td><td/><td/><td>3090 485.38</td><td>220.55</td><td>42023.389</td><td>0.41</td></tr><tr><td>11</td><td/><td/><td>3090 357.12</td><td>128.26</td><td>42022.843</td><td>0.546</td></tr><tr><td>12</td><td/><td/><td>3090 342.18</td><td>14.94</td><td>42022.644</td><td>0.199</td></tr><tr><td>13</td><td/><td/><td>3090 319.22</td><td>22.96</td><td>42022.564</td><td>0.08</td></tr><tr><td>14</td><td/><td/><td>3090327.12</td><td>7.9</td><td>42022.508</td><td>0.056</td></tr><tr><td>15</td><td/><td/><td>3090322.77</td><td>4.35</td><td>42022.554</td><td>0.046</td></tr><tr><td>16</td><td/><td/><td>3090 319.97</td><td>2.8</td><td>42022.618</td><td>0.064</td></tr><tr><td>17</td><td/><td/><td/><td/><td>42022.564</td><td>0.054</td></tr><tr><td>18</td><td/><td/><td/><td/><td>42022.552</td><td>0.012</td></tr></table>

<!-- Media -->

The cutting test was staged on a five-axis table-tilting type machine center with rotary axes A and C (see Fig. 2). The machine is controlled by a TwinCAT-based CNC system developed by the authors of the LP method. The machining experiments of the three methods are shown in Fig. 17. As we can see, all three methods have achieved good processing results, and compared with the other two methods, the result of ICVP has larger spacing (see Fig. 17d) between interpolation points due to the higher feedrates.

The machining time of our method (ICVP) is 9.44 s while those of LP and BECKHOFF are 22.976 and 24.564 s, respectively. The feedrate curve of BECKHOFF has some unnecessary oscillation (see the black dash-dotted circle in Fig. 16), which is not beneficial for machining. The solution of our method satisfies "bang-bang" control, while that of BECKHOFF's algorithm and LP are not, which means that these two methods do not ultimate the capacity of the machine. As shown by the black circle in the second row of Fig. 20a, the acceleration curve of BECKHOFF exceeds the limits. In Figs. 18 and 20, the jerk curves (the third row) of BECKHOFF's method greatly exceed the jerk constraints since they do not control the axial jerk profile. We give the locally enlarged part of the jerk curves in the fourth row. We can see that the jerk profiles of our method strictly satisfy the constraints,while the \( \mathrm{C} \) axis jerk curve of \( \mathrm{{LP}} \) exceeds the limits,as shown in the black circle in the fourth row of Fig. 20b. For the chord error constraint, as shown in Fig. 19, all three methods are under the given limits.

In summary, compared with the newly developed five-axis feedrate planning method and commercial CNC software, the machine time of our method is approximately half of the two methods mentioned above, and the solution of our method satisfies the "bang-bang" optimal control. Moreover, our solution does not exceed the given bounds globally. Therefore, ICVP is significantly superior to the other two methods.

#### 4.4.The iteration data of the three tool paths

The Table 6 shows the iteration data when calculating the optimal feedrate using the ICVP algorithm. The feedrate profile of the tool paths impeller curve,S-shape,and dual B-spline satisfy the convergence condition \( \left( {\zeta  < \gamma }\right) \) after the 9th,16th,and 18th iterations,respectively.

## 5. Conclusions

This paper proposes an optimal feedrate planning on a five-axis parametric tool path with global geometric (chord error) and kinematic constraints (axial velocity, acceleration, and jerk) for the CNC machine tools. Our method formulates the problem as an optimal control problem, and then we use an ICVP method to compute the optimal solution. Compared with the new development five-axis feedrate planning methods, our work can adaptively achieve the global control of kinematic capacity of each axis and geometric error. The results show that the feedrate profiles obtained by our algorithm are optimal with "bang-bang" control, and each constraint is strictly under the limits globally. The machining time of our method is much less than that of the other methods and satisfies the given constraints. On the other hand, our method is offline since the planning time for each example is time consuming. Therefore, in the future work, based on the theory of this paper, we will focus on real-time feedrate planning from the following three aspects:

(i) We will give a more accurate selection criterion for the number of sampling points (see Section 4.2);

(ii) The geometric and kinematic constraints are appropriately scaled so that all the constraints of the optimization problem (Equation 23) can be expressed by linear or quadratic polynomials; and

<!-- Meanless: ってきることです。ご方になりならさったこともことでもございないのですがでいて、どことなり、ママですね。そうなくことは-->




<!-- Meanless: Journal of Computational Design and Engineering 2373 ロッションを知ってきました。私もいましょうこともらってもらっていることができる方々にしています。そのよう-->

(iii) We will use some new solving methods such as Moment-SOS in Josz et al. (2015) and Wang et al. (2021), and heuristic methods (genetic algorithm, ant colony algorithm, etc.) to solve the NLP, so as to shorten the computational time.

## Acknowledgments

We would like to thank Dr Pengpeng Sun for providing us with the interpolation points of the LP methods. This work is partially supported by the National Key Research and Development Program of China under Grant 2020YFA0713703, Beijing Natural Science Foundation under Grant Z190004, NSFC (Nos. 11688101, 61872332, and 12201606), and Fundamental Research Funds for the Central Universities.

## Conflict of interest statement

None declared. References

Beudaert, X., Lavernhe, S., & Tournier, C. (2012). Feedrate interpolation with axis jerk constraints on 5-axis NURBS and G1 tool path. International Journal of Machine Tools and Manufacture, 57, 73-82. https://doi.org/10.1016/j.ijmachtools.2012.02.005.

Bobrow, J. E., Dubowsky, S., & Gibson, J. (1985). Time-optimal control of robotic manipulators along specified paths. International Journal of Robotics Research, 4, 3-17. https://doi.org/10.1177/027836498500400301.

Büskens, C., & Maurer, H. (2000). SQP-methods for solving optimal control problems with control and state constraints: Adjoint variables, sensitivity analysis and real-time control. Journal of Computational and Applied Mathematics, 120(1-2), 85-108. https://doi.org/10.1016/S0377- 0427(00)00305-8.

Chen, B. Y., & Desrochers, A. A. (1990). A proof of the structure of the minimum-time control law of robotic manipulators using a Hamiltonian formulation. IEEE Transactions on Robotics and Automation, 6(3), 388-393. https://doi.org/10.1109/70.56659.

Cripps, R. J., Cross, B., Hunt, M., & Mullineux, G. (2016). Singularities in five-axis machining: Cause, effect and avoidance. International Journal of Machine Tools and Manufacture, 116, 40-51. https://doi.org/10.1016/j.ijmachtools.2016.12.002.

Dong, J., Ferreira, P. M., & Stori,J. A. (2007). Feed-rate optimization with jerk constraints for generating minimum-time trajectories. International Journal of Machine Tools and Manufacture, 47(12-13), 1941-1955. https://doi.org/10.1016/j.ijmachtools.2007.03.006.

Erkorkmaz, K., & Heng, M. (2008). A heuristic feedrate optimization strategy for NURBS tool paths. CIRP Annals- Manufacturing Technology, 57(1), 407-410. https://doi.org/10.1016/j.cirp.2008.03.039.

Erkorkmaz, K., Chen, Q. G., Zhao, M. Y., Beudaert, X., & Gao, X. S. (2017). Linear programming and windowing based feedrate optimization for spline toolpaths. CIRP Annals, 66(1), 393-396. https://doi.org/10.1016/j.cirp.2017.04.058.

Fan, W., Gao, X. S., Yan, W., & Yuan, C. M. (2012). Interpolation of parametric CNC machining path under confined jounce. International Journal of Advanced Manufacturing Technology, 62, 719-739. https://doi.org/10.1007/s00170-011-3842-0.

Fan, W., Gao, X. S., Lee, C. H., Zhang, K., & Zhang, Q. (2013). Time-optimal interpolation for five-axis CNC machining along parametric tool path based on linear programming. International fournal of Advanced Manufacturing Technology, 69, 1373-1388, https://doi.org/10.1007/s00170-013 \( - {5083} - \mathrm{x} \) .

Fleisig, R. V., & Spence, A. D. (2001). A constant feed and reduced angular acceleration interpolation algorithm for multi-axis machining. Computer-Aided Design, 33(1), 1-15. https://doi.org/10.1016/S0010-4485(00)00049-X.

Guo, P., Wu, Y., Yang, G., Shen, Z., Zhang, H., Zhang, P., Lou, F., & Li, H. (2021). A feedrate planning method for the NURBS curve in CNC machining based on the critical constraint curve. Applied Sciences, 11, 4959 https://doi.org/10.3390/app11114959.

He, S., Ou, D., Yan, C. H., & Lee, C. (2015). A chord error conforming tool path B-spline fitting method for NC machining based on energy minimization and LSPIA. Journal of Computational Design and Engineering, (4), 218-232. https://doi.org/10.1016/j.jcde.2015.06.002.

Huang, J., Lu, Y., & Zhu, L. M. (2018). Real-time feedrate scheduling for five-axis machining by simultaneously planning linear and angular trajectories. International Journal of Machine Tools and Manufacture, 135, 78-96. https://doi.org/10.1016/j.ijmachtools.2018.08.006.

Huang, X., Zhao, F., Tao, T., & Mei, X. (2021). A newly developed corner smoothing methodology based on clothoid splines for high speed machine tools. Robotics and Computer-Integrated Manufacturing, 70, 102106. https://doi.org/10.1016/j.rcim.2020.102106.

Jamhour, E., & André, J. P. (1996). Planning smooth trajectories along parametric paths. Mathematics and Computers in Simulation, 41(5-6), 615-626. https://doi.org/10.1016/0378-4754(95)00105-0.

Jin, Y., Zhao, S., & Wang, Y. (2019). An optimal feed interpolator based on \( {\mathrm{G}}^{2} \) continuous Bézier curves for high-speed machining of linear tool path. Chinese Journal of Mechanical Engineering, 32, 43. https://doi.org/10.1186/s10033-019-0360-8.

Josz, C., Maeght, J., Panciatici, P., & Gilbert, J. C. (2015). Application of the moment-SOS approach to global optimization of the OPF problem. IEEE Transactions on Power Systems, 30(1), 463-470. https://doi.org/10.1109/TPWRS.2014.2320819.

Li, B., Zhang, H., Ye, P., & Wang, J. (2020). Trajectory smoothing method using reinforcement learning for computer numerical control machine tools. Robotics and Computer-Integrated Manufacturing, 61, 101847. https://doi.org/10.1016/j.rcim.2019.101847.

i, S., Zhang, Q., Gao, X. S., & Li, H. (2012). Minimum time trajectory planning for five-axis machining with general kinematic constraints. Mathematics Mechanization Research Preprints, 31, 1-20. http://www.mmrc.iss.ac.cn/pub/mm31/01-li.pdf.

Li, H., Jiang, X., Huo, G., Su, C., Wang, B., Hu, Y., & Zheng, Z. (2022). A novel feedrate scheduling method based on Sigmoid function with chord error and kinematic constraints. International Journal of Advanced Manufacturing Technology, 119, 1531-1552. https://doi.org/10.1007/s00170-0 21-08092-1.

Liu, H., Liu, Q., Sun, P., Liu, Q., & Yuan, S. (2017). The optimal feedrate planning on five-axis parametric tool path with geometric and kinematic constraints for CNC machine tools. International Journal of Production Research, 55(13), 3715-3731. https://doi.org/10.1080/00207543.2016.12 54357.


<!-- Meanless: 2374 | Journal of Computational Design and Engineering, 2022, Vol. 9, No. 6 ロップランを知らすこうございました。ここもごと思ったと思っていることができる方々なので、そこもあってってることは、-->

Liu, Y., Wan, M., Qin, X. B., Xiao, Q. B., & Zhang, W. H. (2020). FIR filter-based continuous interpolation of G01 commands with bounded axial and tangential kinematics in industrial five-axis machine tools. International Journal of Mechanical Sciences, 169, 105325. https://doi.org/10.1 016/j.ijmecsci.2019.105325.

Lu, L., Zhang, J., Fuh, J. Y. H., Han, J., & Wang, H. (2020). Time-optimal tool motion planning with tool-tip kinematic constraints for robotic machining of sculptured surfaces. Robotics and Computer-Integrated Manufacturing, 65, 101969. https://doi.org/10.1016/j.rcim.2020.101969.

Sencer, B., Altintas, Y., & Croft, E. (2008). Feed optimization for five-axis CNC machine tools with drive constraints. International Journal of Machine Tools and Manufacture, 48(7-8), 733-745. https://doi.org/10.1016/j.ijmachtools.2008.01.002.

Shin, K., & McKay, N. (1985). Minimum-time control of robotic manipulators with geometric path constraints. IEEE Transactions on Automatic Control, 30, 531-541. https://doi.org/10.1109/TAC.1985.1104009.

Song, D. N., & Ma, J. W. (2019). Interval partition-based feedrate scheduling with axial drive constraints for five-axis spline toolpaths. International Journal of Advanced Manufacturing Technology, 105, 4701-4714. https://doi.org/10.1007/s00170-019-04433-3.

Sun, Y., Zhao, Y., Xu, J., & Guo, D. (2014). The feedrate scheduling of parametric interpolator with geometry, process and drive constraints for multi-axis CNC machine tools. International Journal of Machine Tools and Manufacture, 85, 49-57. https://doi.org/10.1016/j.ijmachtools.2014.0 5.001.

Tajima, S., & Sencer, B. (2020). Real-time trajectory generation for 5-axis machine tools with singularity avoidance. CIRP Annals, 69(1), 349-352. https://doi.org/10.1016/j.cirp.2020.04.050.

Timar, S. D., & Farouki, R. T. (2007). Time-optimal traversal of curved paths by Cartesian CNC machines under both constant and speed-dependent axis acceleration bounds. Robotics and Computer-Integrated Manufacturing, 23(5), 517-532. https://doi.org/10.1016/j.rcim.2006.07.0 02.

Wang, J., Magron, B. V., & Lasserre, J. (2021). TSSOS: A moment-SOS hierarchy that exploits term sparsity. SIAM Journal on Optimization, 31(1), 30-58. https://doi.org/10.1137/19M1307871.

Wang, W, jiang, Z., Tao, W., & Zhuang, W. (2015). A new test part to identify performance of five-axis machine tool-Part I: Geometrical and kinematic characteristics of S part. International Journal of Advanced Manufacturing Technology, 79, 729-738. https://doi.org/10.1007/s00170-0 15-6870-3.

Xu, L., Zhang, W., Geng, Z., & Xu, J. (2020). Crossing principle-based feedrate planning method for 5-axis CNC milling with drive constraints. Journal of Industrial and Production Engineering, 37, 1-12. https://doi.org/10.1080/21681015.2020.1798521.

Yeh, S. S., & Hsu, P. L. (2002). Adaptive-feedrate interpolation for parametric curves with a confined chord error. Computer-Aided Design, 34(3), 229-237. https://doi.org/10.1016/S0010-4485(01)00082-3.

Yuan, M. C., Zhang, K., & Fan, W. (2013). Time-optimal interpolation for CNC machining along curved tool pathes with confined chord error. Journal of Systems Science & Complexity, 26, 836-870. https://doi.org/10.1007/s11424-013-3180-4.

Zhang, K., Gao, X. S., Li, B. H., & Yuan, C. M. (2012). A greedy algorithm for feed-rate planning of CNC machines along curved tool paths with confined jerk for each axis. Robotics and Computer Integrated Manufacturing, 28, 472-483. https://doi.org/10.1016/j.rcim.2012.02.006.

Zhang, K., Yuan, C. M., & Gao, X. S. (2013). Efficient algorithm for time-optimal feedrate planning and smoothing with confined chord error anc acceleration. International Journal of Advanced Manufacturing Technology, 66, 1685-1697. https://doi.org/10.1007/s00170-012-4450-3.

## Appendix 1. Flank Milling Tool Path

B-spline degree: 3.

Knot vector:

<!-- Media -->

\[\left\lbrack  \begin{array}{llllllllllll} 0 & 0 & 0 & 0 & {0.2} & {0.4} & {0.6} & {0.8} & 1 & 1 & 1 & 1 \end{array}\right\rbrack  \text{.}\]

Bottom curve control points:

\[\left\lbrack  \begin{matrix} 5 &  - {10} & {10} & {20} & {30} & {40} & {50} & {55} \\  0 & {20} & {20} & {30} & {30} & {30} & {20} & 0 \\  0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \end{matrix}\right\rbrack  .\]

Top curve control points:

\[\left\lbrack  \begin{matrix} 0 &  - {15} & 5 & {15} & {30} & {45} & {55} & {60} \\  0 & {20} & {25} & {35} & {35} & {35} & {25} & 0 \\  {15} & {15} & {15} & {15} & {15} & {15} & {15} & {15} \end{matrix}\right\rbrack  .\]

<!-- Media -->