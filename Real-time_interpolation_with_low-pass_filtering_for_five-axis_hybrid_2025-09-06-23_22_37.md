

<!-- Meanless: Mechanical Systems and Signal Processing 209 (2024) 111080 Contents lists available at ScienceDirect Mechanical Systems and Signal Processing ELSEVIER journal homepage: www.elsevier.com/locate/ymssp updates-->

# Real-time interpolation with low-pass filtering for five-axis hybrid machining robots

Zikang Shi, Weijia Zhang, Ye Ding \( {}^{ * } \)

State Key Laboratory of Mechanical System and Vibration, School of Mechanical Engineering, Shanghai Jiao Tong University, Shanghai 200240, China

## ARTICLEINFO

Communicated by Yaguo Lei

Keywords:

Low-pass filter

Interpolation

Hybrid robot

Five-axis machining

Geometric programming

Jerk constraints

## A B S T R A C T

This paper proposes a new interpolation approach for five-axis hybrid robots to enhance the surface quality and efficiency of machining processes. Hybrid robots have the potential to manufacture large, complex structural parts with high flexibility and a favorable load-to-weight ratio. To interpolate G01 commands, the proposed method implements finite impulse response (FIR) filters rather than splines. These filters smoothly synchronize the motions of tool center points (TCPs) and tool orientation vectors (TOVs), and are designed for each linear segment online with geometric programming while considering joint constraints. Additionally, the adjacent linear segments are locally blended with bounded geometric errors under kinematic constraints. The proposed method generates time-optimal trajectories with jerk-limited joint motions in real-time, making it more effective than current interpolation methods. The effectiveness of the proposed method is verified through the flank milling operation on aluminum alloy.

## 1. Introduction

Manufacturing high-quality, large structural parts often requires the use of five-axis machine tools [1]. To reduce machine tool costs and increase workspace efficiency, robotic machining systems with a large operational workspace can be utilized [2]. Hybrid robots with a parallel kinematic mechanism (PKM) are more rigid than serial mechanisms \( \left\lbrack  {3,4}\right\rbrack \) ,but their vibration characteristics restrict high-speed machining [5]. Command signal prefiltering is effective in the vibration cancelation [6]. To improve the productivity of five-axis machine tools, this paper proposes a new filtering-based interpolation strategy for hybrid machining robots that maximizes their kinematic capability (Fig. 1).

A Computer-Aided Manufacturing (CAM) systems generates a five-axis machining toolpath by using successive linear segments code with G01 commands [7]. The toolpath is then discretized under chord error tolerance and formatted into an NC file with cutter locations (CLs). Afterward, the NC system interpolates the file while adhering to kinematic constraints. However, the direct interpolation leads to point-to-point (P2P) motion, which causes discontinuities at segment junctions. These discontinuities result in excessive starts and stops, leading to a rough surface finish and longer cycle time [8].

To avoid P2P motion,many techniques use a two-step approach for three-axis machining [9,10]. Firstly, corner smoothing is employed, where high-order splines are fitted at the corners to create a geometrically smooth toolpath. Secondly, feedrate scheduling is used to generate smooth feeds with respect to (w.r.t.) time. Various geometric curves [11-13] are utilized to create smooth corners in

---

<!-- Footnote -->

* Corresponding author.

E-mail address: y.ding@sjtu.edu.cn (Y. Ding).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.ymssp.2023.111080 Received 31 July 2023; Received in revised form 23 November 2023; Accepted 26 December 2023 Available online 5 January 2024 0888-3270/(C) 2023 Elsevier Ltd. All rights reserved.-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

## Nomenclature

\( {\mathbf{P}}_{k}{\mathbf{O}}_{k}{\mathbf{Q}}_{k} \) . The \( {k}^{\text{th }} \) TCP,TOV,and joint position \( {T}_{k,i} \) . Fig. 2 The \( {i}^{\text{th }} \) time constant of the \( {k}^{\text{th }} \) FIR filter \( {T}_{k,c} \) . Fig. 2 Delay time of the \( {k}^{\text{th }} \) corner \( {T}_{k,d} \) . Fig. 2 Delay time of the \( {k}^{\text{th }} \) corner in P2P interpolation \( {\varepsilon }_{\max }^{p}{\varepsilon }_{\max }^{o} \) . Fig. 2 User-defined maximum blending errors \( {\mathbf{V}}_{k}{\mathbf{\Omega }}_{k} \) . Eq. (1) Velocity pulses for the TCP and TOV of the \( {k}^{\text{th }} \) segment \( {T}_{s} \) . Eq. (1) Sampling time of the control loop \( {\mathbf{v}}_{k}\left( {\mathbf{\omega }}_{k}\right) \) . Eq. (2) Filtered (angular) velocity vectors of the \( {k}^{\text{th }} \) segment \( {L}_{k}{\theta }_{k} \) . Eq. (4) Displacement and angle of the \( {k}^{\text{th }} \) segment \( {h}_{\mu ,k} \) . Eq. (4) Displacement of the \( {k}^{\text{th }} \) segment and the \( \mu \) axis \( {q}_{\mu ,k} \) . Eq. (4) Joint position of the \( {k}^{\text{th }} \) segment and the \( \mu \) axis \( {V}_{\max }^{\mu }{V}_{\max }^{p}{V}_{\max }^{o} \) . Eq. (4) Maximum velocity of the \( \mu \) axis,TCP and TOV \( {A}_{\max }^{\mu }{A}_{\max }^{\mu }{A}_{\max }^{o} \) . Eq. (4) Maximum acceleration of the \( \mu \) axis,TCP and TOV \( {J}_{\max }^{\mu }{J}_{\max }^{p}{J}_{\max }^{o} \) . Eq. (4) Maximum jerk of the \( \mu \) axis,TCP and TOV \( \mathrm{s} \) (s’s’s’’). Eq. (10) Arc length of the TCP path (its derivatives w.r.t. time) \( {q}_{\mu }^{\prime }{q}_{\mu }^{\prime \prime }{q}_{\mu }^{\prime \prime } \) . Eq. (10) Joint velocity,acceleration and jerk of the \( \mu \) axis \( {q}_{s}{q}_{ss}{q}_{sss} \) . Eq. (10) Derivatives of the joint \( \mu \) w.r.t. \( s \) \( {\widehat{q}}_{s} \) . Eq. (11) Estimated first-order derivative of the joint \( \mu \) w.r.t. \( s \) \( {\widehat{q}}_{\mu }^{\prime }{\widehat{q}}_{\mu }^{''}{\widehat{q}}_{\mu }^{''} \) . Eq. (12) Estimated velocity,acceleration and jerk of the \( \mu \) axis \( {\varepsilon }_{k}^{p}{\varepsilon }_{k}^{o} \) . Fig. 3 Blending errors of the TCP and TOV of the \( {k}^{\text{th }} \) corner \( {\widehat{\varepsilon }}_{k}^{p}{\widehat{\varepsilon }}_{k}^{o} \) . Fig. 3 Estimated blending errors of the \( {k}^{\text{th }} \) corner \( {v}_{k} \) . Eq. (16) Filtered velocity of the \( {k}^{\text{th }} \) segment \( {v}_{k,i} \) . Eq. (16) The \( {i}^{\text{th }} \) piece of the filtered velocity and the \( {k}^{\text{th }} \) segment \( {t}_{k} \) . Eq. (17) Intersection time between adjacent velocity profiles \( {T}_{n}{T}_{m} \) . Fig. 4 Junction points between pieces of the filtered velocity \( {t}_{s}\left( {t}_{e}\right) \) . Fig. 5 Time when the blending starts (ends) \( {\theta }_{p} \) . Fig. 5 Angle between the adjacent linear segments \( {l}_{dec}\left( {l}_{acc}\right) \) . Eq. (21) Displacement of the deceleration (acceleration) part \( {\theta }_{\text{dec }}\left( {\theta }_{\text{acc }}\right) \) . Eq. (23) Angle of the deceleration (acceleration) part \( {\mathbf{V}}_{m}{\mathbf{V}}_{n} \) . Eq. (23) Vectors on the tangent plane at \( {\mathbf{O}}_{k + 1} \) \( {\widehat{h}}_{\mu }^{\prime }{\widehat{h}}_{\mu }^{\prime \prime }{\widehat{h}}_{\mu }^{\prime \prime } \) . Fig. 7 Estimated derivatives of the \( \mu \) axis in blending \( {h}_{\mu ,k}^{\prime }{h}_{\mu ,k}^{\prime \prime }{h}_{\mu ,k}^{\prime \prime \prime } \) . Fig. 7 Estimated derivatives of the \( {k}^{\text{th }} \) segment and the \( \mu \) axis \( {\mathcal{C}}_{v} \) . Eq. (27) Axes that exceed the velocity constraints \( {\mathcal{C}}_{a}{\mathcal{C}}_{j} \) . Eq. (31) Axes that exceed the acceleration and jerk constraints \( {\widetilde{q}}_{\text{diff }}^{\prime }{\widetilde{q}}_{\text{diff }}{\widetilde{q}}_{\text{diff }}^{\prime \prime } \) . Fig. 9 Differential error of joint velocity,acceleration and jerk \( {\widetilde{q}}_{nl}^{''}{\widetilde{q}}_{nl}^{'''} \) . Fig. 9 Nonlinear error of joint acceleration and jerk \( {p}_{\mu }^{\prime }{p}_{\mu }^{\prime \prime }{p}_{\mu }^{\prime \prime } \) . Fig. 11 TCP’s velocity,acceleration and jerk of the \( \mu \) axis \( {o}_{\mu }^{\prime }{o}_{\mu }^{\prime \prime }{o}_{\mu }^{\prime \prime } \) . Fig. 11 TOV’s velocity,acceleration and jerk of the \( \mu \) axis

the first step. A five-axis toolpath can be represented in either the workpiece coordinate system (WCS) [8,11] or the machine coordinate system (MCS) [12-15]. In WCS-based methods, a toolpath consists of two curves representing tool center points (TCPs) and tool orientation vectors (TOVs). However, inaccurate synchronization between the TCP and TOV curves can lead to feedrate fluctuation or even gouging [16]. Parameter synchronization between the two curves is challenging for hybrid robots with parasitic motion [17]. On the other hand, MCS-based methods represent the toolpath with curves defined in the joint space, which provides a smoother five-axis transition. Although the synchronization of the joints is directly guaranteed, the geometric errors at the corners are non-linear and position-dependent, making it challenging to constrain [12]. In our previous work [15], we addressed the issue of geometric errors for hybrid robots using numerical methods, which demonstrated higher machining efficiency compared to the WCS-based approach [11]. However, our subsequent research on hybrid robots found that real-time trajectory generation poses a challenge for these spline-based methods as the feedrate scheduling and interpolation processes are time-consuming.

In the feedrate scheduling step, a function with time parameters is established under kinematic constraints to generate a smooth trajectory. The time-optimal trajectory planning (TOTP) problem for fully specified paths is well-defined in the robotics literature [18], and various methods have been proposed to address it. The bisection algorithm has been used by Barnett and Gosselin [19] for this problem, and it is compatible with both serial and parallel mechanisms' second-order kinematic and dynamic constraints. When machining, limiting jerk can help to suppress vibrations of the drive system [20]. In three-axis machining, a common strategy is to find critical points with the maximum curvature and conduct a bi-directional scan to determine the feedrate [9]. However, for five-axis machine tools, the feedrate is not solely determined by the TCP path curvature but also by the joints [13]. To address this, various methods have been proposed, such as linearizing the jerk constraints [21] and using heuristic feedrate scheduling methods [22]. These methods consider velocity, acceleration, and jerk limits on the TCP, TOV, and joint motion. Other methods ignore the limits on the joint motion to achieve real-time performance, such as the adaptive interpolator [23] and bi-directional scanning method with time synchronization [24]. However, neglecting the joint constraints could lead to extensive joint tracking errors, which negatively impact the machined surface quality. Therefore, maximizing the kinematic capability of five-axis hybrid machining robots while considering constraints in TCP, TOV, and joint motions remains a challenge for implementing a real-time feedrate scheduling method.

<!-- Meanless: 2-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: (a) (b) MCS: \( {\mathbf{Q}}_{k} = {\left\lbrack  {q}_{1,k},{q}_{2,k},{q}_{3,k},{q}_{4,k},{q}_{5,k}\right\rbrack  }^{\mathrm{T}} \) Inverse kinematics Forward kinematics WCS: \( {\mathbf{P}}_{k},{\mathbf{O}}_{k} \) \( \left( {{\mathbf{P}}_{1},{\mathbf{O}}_{1}}\right) \) \( \left( {{\mathbf{P}}_{2},{\mathbf{O}}_{2}}\right) \) \( \left( {{\mathbf{P}}_{N},{\mathbf{O}}_{N}}\right) \) Tool \( {\mathbf{P}}_{k} = {\left\lbrack  {p}_{x,k},{p}_{y,k},{p}_{z,k}\right\rbrack  }^{\mathrm{T}} \) WCS \( {\mathbf{O}}_{k} = {\left\lbrack  {o}_{x,k},{o}_{y,k},{o}_{z,k}\right\rbrack  }^{\mathrm{T}} \) Linear toolpath \( {q}_{4} \) \( {q}_{3} \) \( {q}_{1} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_2.jpg?x=187&y=155&w=1332&h=619&r=0"/>

Fig. 1. (a) The configuration of the demonstrative hybrid robot. (b) The cutter location representation for five-axis robots (Ref. [15]).

<!-- figureText: (a) (b) (c) \( {\mathbf{O}}_{k + 1} \) \( {\mathbf{P}}_{k + 1} \) \( {\widehat{\varepsilon }}_{k}^{p} \) \( {\mathcal{E}}_{k}^{o} \) 10 \( {\mathbf{P}}_{m} \) \( \mathbf{P}\left( {t}_{k}\right) \) elocity profile \( {v}_{k,4} \) \( {v}_{k} \) \( {t}_{k} \) \( {V}_{k + 1} \) \( {v}_{k,6} \) \( {T}_{k,c} \) \( {v}_{k,1} \) \( {v}_{k,7} \) \( {T}_{k,1} \) \( {T}_{k,d} \) time -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_2.jpg?x=262&y=875&w=1181&h=339&r=0"/>

Fig. 3. (a) The velocity profile blending. (b) TCP and (c) TOV blending errors at the \( {k}^{\text{th }} \) corner.

<!-- figureText: \( \bullet  {t}_{k} \) \( {T}_{n} \) \( {T}_{m} \) \( {v}_{k,7} \) \( {v}_{k + 1,1} \) \( {v}_{k,6} \) \( {v}_{k + 1,2} \) ③ \( {t}_{k} \leq  {T}_{m},{T}_{n} \) ④ \( {T}_{n} \leq  {t}_{k} \leq  {T}_{m} \) ① \( {T}_{m} \leq  {t}_{k} \leq  {T}_{n} \) ② \( {t}_{k} \geq  {T}_{m},{T}_{n} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_2.jpg?x=468&y=1318&w=763&h=356&r=0"/>

Fig. 4. Four intersection cases between adjacent kinematic profiles.

<!-- Media -->

A different approach builds on a one-step strategy. Biagiotti and Melchiorri [20] proposed an online interpolation method that utilizes finite impulse response (FIR) filters. Discrete impulses represent the linear segments, which the FIR filters can smooth and generate a time-optimal trajectory with efficient convolution. Tajima and Sencer [25,26] demonstrated the use of FIR filtering in five-axis toolpath interpolation, synchronously blending the TCP and TOV motion with analytical geometric error models. Liu et al. [27] blended the TOV motion in the MCS to ensure rotational joint constraints, while Ward et al. [28] analyzed the nonlinear orientation error caused by MCS-based interpolation. Compared to the two-step methods, one-step approaches can efficiently generate a jerk-limited trajectory. However, applying these methods to hybrid robots with PKMs presents challenges. Most filter-based methods can only guarantee constraints partially, such as ignoring the joint constraints in Ref. [25,26] since the trajectory is filtered in the TCP and TOV motions. Similarly, Ref. [27,28] do not directly respect TOV and translational joint constraints. Sun et al. [29] addressed this issue by constraining the velocity and acceleration of joints for robot manipulators with six rotational joints. The joint kinematic issue remains crucial for hybrid robots to carefully alter the tool's pose and avoid exceeding joint limitations.

<!-- Meanless: 3-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: (a) Linear path (b) \( \left( {{l}_{acc} + {l}_{dec}}\right) /2 \) \( {l}_{acc} \) \( {\mathbf{P}}_{m} \) \( \mathbf{P}\left( {t}_{k}\right) \) Blending path \( {\mathbf{P}}_{k + 1} \) \( {\mathbf{P}}_{k + 2} \) \( \mathbf{P}\left( {t}_{e}\right) \) \( \mathbf{P}\left( {t}_{k}\right) \) \( \mathbf{P}\left( {t}_{s}\right) \) \( {\mathbf{P}}_{k} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_3.jpg?x=468&y=154&w=768&h=310&r=0"/>

Fig. 5. (a) The corner blending. (b) The determination of \( {\mathrm{P}}_{m} \) .

<!-- Media -->

In this paper, a real-time interpolation technique is presented for five-axis hybrid robots. To ensure a smooth and synchronized tool motion, FIR filtering is applied to TCP and TOV motions. Online design of FIR filters using geometric programming (GP) is implemented to limit the joint velocity, acceleration, and jerk. Overlapping of adjacent kinematic profiles with constrained geometric errors further reduces machining time. Finally, the blending part is adjusted with feed impulses to ensure kinematic constraints are met. The method is applicable to robots with different configurations, including a five-axis machine tool (three orthogonal linear axes and two rotary axes) and a six-axis robot (six rotary axes). The main contributions of this paper are as follows:

(1) A new method is proposed for efficiently interpolating the discrete CLs using third-order FIR filters. The filter-based interpolation for hybrid robots is discussed for the first time in the literature. Compared to the two-step interpolation method for hybrid robots discussed in previous studies \( \left\lbrack  {{11},{21}}\right\rbrack \) ,the proposed method significantly reduces the calculation cost while maintaining similar machining time under the same kinematic constraints.

(2) The process of designing FIR filters is transformed into a GP problem that can be efficiently and robustly solved using the state-of-the-art convex optimizer [30]. By utilizing this method, the filtered trajectory achieves maximum kinematic capability for five-axis hybrid robots, while also adhering to constraints for joints' velocity, acceleration, and jerk. In contrast, the previous FIR filter design strategies [26-29] were unable to directly meet these constraints.

(3) By adapting the feed impulses online, the geometric errors and kinematic constraints of the overlapping parts are effectively managed. To accomplish this, a precise estimate of the geometric errors and kinematics is utilized. Previous methods based on FIR filters [26-29] did not adequately address these issues in the blending part.

The rest of this paper is structured as follows: Section 2 details the interpolation issue for hybrid robots. Section 3 proposes the interpolation approach using local corner blending. The results of the simulation and experiment are presented in Section 4. Finally, Section 5 concludes the paper.

## 2. Interpolation for hybrid robots

Fig. 1(a) presents the configuration of the demonstrative hybrid robot with a 2PRS-PSR parallel manipulator. Here, P, R, and S represent the prismatic,rotational,and spherical joints respectively,where underlined \( \mathrm{P} \) is actuated by three prismatic joints \( \left( {{q}_{3},{q}_{4}}\right. \) , and \( {q}_{5} \) ). The parallel manipulator can achieve one degree of translational and two degrees of rotational motions. The linear guideways \( \left( {q}_{1}\right. \) and \( \left. {q}_{2}\right) \) provide two translational motions. Therefore,the robot can perform five-axis machining. More details of the robot are described in Ref. [15]. The proposed method can be extended to other configurations by replacing the actuated joints in the MCS. In Fig. 1(b),CAM-generated toolpaths for generic five-axis hybrid robots are displayed using discrete CLs. The \( {k}^{\text{th }} \) CL can be presented in either the WCS with TCP as \( {\mathbf{P}}_{k} = {\left\lbrack  {p}_{x,k},{p}_{y,k},{p}_{z,k}\right\rbrack  }^{\mathrm{T}} \) and TOV as \( {\mathbf{O}}_{k} = {\left\lbrack  {o}_{x,k},{o}_{y,k},{o}_{z,k}\right\rbrack  }^{\mathrm{T}} \) ,or the MCS as \( {\mathbf{Q}}_{k} = {\left\lbrack  {q}_{1,k},{q}_{2,k},{q}_{3,k},{q}_{4,k},{q}_{5,k}\right\rbrack  }^{\mathrm{T}} \) . These representations are converted using forward and inverse kinematics. This section introduces the interpolation strategy utilizing third-order FIR filters,illustrated in Fig. 2. The time constants \( {\left\{  {T}_{k,i}\right\}  }_{i = 1}^{3} \) are optimized for each linear segment between the \( {k}^{\text{th }} \) and \( {\left( k + 1\right) }^{\text{th }} \) CLs to obtain time-optimality under the kinematic constraints. A constrained nonlinear optimization problem is proposed for each segment and solved using GP for robust and efficient results. The delay time \( {T}_{k,c} \) at the corner is determined by limiting geometric errors and avoiding velocity violations while blending. Furthermore,the feed duration \( {T}_{k,1} \) are adjusted in case of any acceleration or jerk constraints violations. After optimization, the CLs are interpolated according to the determined time constants and delay time through filtering and blending in the WCS. The joint trajectories are then obtained with inverse kinematics.

<!-- Meanless: 4-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

### 2.1. Filtering based interpolation

This section discusses FIR filtering for the five-axis toolpath of hybrid robots. The starting and end CLs of a \( {k}^{\text{th }} \) linear segment are represented by \( \left\lbrack  {{\mathbf{P}}_{k},{\mathbf{O}}_{k}}\right\rbrack \) and \( \left\lbrack  {{\mathbf{P}}_{k + 1},{\mathbf{O}}_{k + 1}}\right\rbrack \) ,respectively. The TCP and TOV motions of each segment are interpolated with matching time constants \( {\left\{  {T}_{k,i}\right\}  }_{i = 1}^{3} \) . The TCPs are interpolated in the workpiece coordinates,while the TOVs are interpolated on the unit sphere. Direct interpolation on the unit sphere is preferred over MCS-based interpolation of the TOVs [27,28,31] as it provides hybrid robots with greater orientation adjustment flexibility [11].

The velocity pulses for TCP and TOV of each segment are produced in the following manner,

\[{\mathbf{V}}_{k}\left\lbrack  n\right\rbrack   = \left\lbrack  \begin{matrix} {v}_{x,k} \\  {v}_{y,k} \\  {v}_{z,k} \end{matrix}\right\rbrack   = \left\{  \begin{matrix} \frac{{\mathbf{P}}_{k + 1} - {\mathbf{P}}_{k}}{{T}_{k,1}}, & 1 \leq  n < {N}_{k,1} \\  \mathbf{0}, & \text{ otherwise } \end{matrix}\right.  \tag{1}\]

\[{\mathbf{\Omega }}_{k}\left\lbrack  n\right\rbrack   = \left\lbrack  \begin{matrix} {\omega }_{x,k} \\  {\omega }_{y,k} \\  {\omega }_{z,k} \end{matrix}\right\rbrack   = \left\{  {\begin{matrix} \frac{\cos \left( {n{T}_{s}{\theta }_{k}/{T}_{k,1}}\right) {\mathbf{O}}_{k + 1} - \cos \left( {{\theta }_{k} - n{T}_{s}{\theta }_{k}/{T}_{k,1}}\right) {\mathbf{O}}_{k}}{{T}_{k,1}\sin \left( {\theta }_{k}\right) /{\theta }_{k}} & 1 \leq  n < {N}_{k,1} \\  {\mathbf{O}}_{k} & \text{ otherwise } \end{matrix},}\right. \]

where \( {\theta }_{k} = {\cos }^{-1}\left( {{\mathbf{O}}_{k}^{\mathrm{T}}{\mathbf{O}}_{k + 1}}\right) ,{\omega }_{k} = {\theta }_{k}/{T}_{k,1} \) ,and \( {N}_{k,1} = \operatorname{round}\left( {{T}_{k,1}/{T}_{s}}\right) \) . round(-) rounds toward the nearest integer. \( {T}_{s} \) is the sampling time of the control loop. The filtered velocity \( {\mathbf{v}}_{k} \) is computed in real-time at the sampling time \( n \) ,

\[{\mathbf{v}}_{k}\left( n\right)  = 2{\mathbf{v}}_{k}\left( {n - 1}\right)  - {\mathbf{v}}_{k}\left( {n - 2}\right)  + \]

\[\frac{1}{{N}_{k,2}{N}_{k,3}}\left\lbrack  {{\mathbf{V}}_{k}\left( n\right)  - {\mathbf{V}}_{k}\left( {n - {N}_{k,2}}\right)  - {\mathbf{V}}_{k}\left( {n - {N}_{k,3}}\right)  + {\mathbf{V}}_{k}\left( {n - {N}_{k,2} - {N}_{k,3}}\right) }\right\rbrack  , \tag{2}\]

where \( {N}_{k,2} = \operatorname{round}\left( {{T}_{k,2}/{T}_{s}}\right) ,{N}_{k,3} = \operatorname{round}\left( {{T}_{k,3}/{T}_{s}}\right) \) . The filtered angular velocity \( {\omega }_{k} \) is calculated similarly. The final jerk-limit trajectory is interpolated with numerical integration. For further details on FIR filter-based interpolation for five-axis robots, please refer to Ref. [26].

### 2.2. Time constants optimization

When machining, it is important to consider the kinematic constraints from both the MCS and the WCS when generating trajectories. The maximum velocity \( \left( {V}_{\max }^{\mu }\right) \) ,acceleration \( \left( {A}_{\max }^{\mu }\right) \) ,and jerk \( \left( {J}_{\max }^{\mu }\right) \) of the five actuating axes are determined by the kinematic capabilities of the servo drives,where \( \mu \) denotes the joint \( \mu  \in  \{ 1,2,3,4,5\} \) . Additionally,the machining requirements further limit the maximum velocity \( \left( {V}_{\max }^{p}\right) \) ,acceleration \( \left( {A}_{\max }^{p}\right) \) ,and jerk \( \left( {J}_{\max }^{p}\right) \) of the tooltip. In five-axis machining,the TOV variation is restricted under maximum angular velocity \( \left( {V}_{\max }^{o}\right) \) ,acceleration \( \left( {A}_{\max }^{o}\right) \) ,and jerk \( \left( {J}_{\max }^{o}\right) \) . To ensure that these kinematic constraints are met during five-axis motion,time constants \( {\left\{  {T}_{k,i}\right\}  }_{i = 1}^{3} \) are designed using a constrained optimization approach.

The proposed approach utilizes a time-optimal objective as follows,

\[{T}_{k,i}^{ * } = \arg \mathop{\min }\limits_{{T}_{k,i}}\mathop{\sum }\limits_{{i = 1}}^{3}{T}_{k,i} \tag{3}\]

To ensure the user-defined kinematics are met on the given CLs, we have established the following constraints,

\[{T}_{k,1} \geq  \max \left( {\frac{{L}_{k}}{{V}_{\max }^{p}},\frac{{\theta }_{k}}{{V}_{\max }^{o}},\frac{{h}_{\mu ,k}}{{V}_{\max }^{\mu }}}\right) ,\]

\[{T}_{k,1}{T}_{k,2} \geq  \max \left( {\frac{{L}_{k}}{{A}_{\max }^{p}},\frac{{\theta }_{k}}{{A}_{\max }^{o}},\frac{{h}_{\mu ,k}}{{A}_{\max }^{\mu }}}\right) , \tag{4}\]

\[{T}_{k,1}{T}_{k,2}{T}_{k,3} \geq  \max \left( {\frac{{L}_{k}}{{J}_{\max }^{p}},\frac{{\theta }_{k}}{{J}_{\max }^{o}},\frac{{h}_{\mu ,k}}{{J}_{\max }^{\mu }}}\right) .\]

where \( {L}_{k} = \begin{Vmatrix}{{\mathbf{P}}_{k} - {\mathbf{P}}_{k + 1}}\end{Vmatrix},{\theta }_{k} = {\cos }^{-1}\left( {{\mathbf{O}}_{k}^{\mathrm{T}}{\mathbf{O}}_{k + 1}}\right) ,{h}_{\mu ,k} = \left| {{q}_{\mu ,k} - {q}_{\mu ,k + 1}}\right| \) are calculated from the given CLs,i.e., \( {\mathbf{P}}_{k},{\mathbf{O}}_{k} \) ,and \( {\mathbf{Q}}_{k} \) . Here, \( \parallel  \cdot  \parallel \) is the two-norm and \( \left| \cdot \right| \) is the absolute value. As there is no analytical form for the mapping from the filter constants to the actual joint velocity, acceleration, and jerks, we have proposed a kinematic approximation to optimize the filter constants under these constraints in Eq. (4). The accuracy of this approximation is analyzed in Section 2.3. To uniform the analytical expression of the kinematic profile, it is necessary for the time constants to meet the following constraints in every segment,

<!-- Meanless: 5-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

\[{T}_{k,1} \geq  {T}_{k,2} + {T}_{k,3}, \tag{5}\]

In the meantime, \( {T}_{k,1} \) are further constrained to prevent overlapping between adjacent kinematic profiles,

\[{T}_{k,1} \geq  {T}_{k - 1,2} + {T}_{k - 1,3} = {T}_{k - 1,d},k > 1. \tag{6}\]

Eqs. (3)-(6) forms a GP problem, which can be transformed into a convex optimization problem using log mapping [30]. If we assume that \( {T}_{k,1} = \exp \left( x\right) ,{T}_{k,2} = \exp \left( y\right) ,{T}_{k,3} = \exp \left( z\right) \) ,the problem takes an equivalent form,

minimize \( t \)

\[\text{such that}\;x \geq  \log \left\lbrack  {\max \left( {\frac{{L}_{k}}{{V}_{\max }^{p}},\frac{{\theta }_{k}}{{V}_{\max }^{o}},\frac{{h}_{\mu ,k}}{{V}_{\max }^{\mu }},{T}_{k - 1,d}}\right) }\right\rbrack  \text{,}\]

\[x + y \geq  \log \left\lbrack  {\max \left( {\frac{{L}_{k}}{{A}_{\max }^{p}},\frac{{\theta }_{k}}{{A}_{\max }^{o}},\frac{{h}_{\mu ,k}}{{A}_{\max }^{\mu }}}\right) }\right\rbrack  ,\]

\[x + y + z \geq  \log \left\lbrack  {\max \left( {\frac{{L}_{k}}{{J}_{\max }^{p}},\frac{{\theta }_{k}}{{J}_{\max }^{o}},\frac{{h}_{\mu ,k}}{{J}_{\max }^{\mu }}}\right) }\right\rbrack  , \tag{7}\]

\[z - y \leq  0,\]

\[\log \left\lbrack  {\exp \left( x\right)  + \exp \left( y\right)  + \exp \left( z\right) }\right\rbrack   \leq  t\]

\[\log \left\lbrack  {\exp \left( y\right)  + \exp \left( z\right) }\right\rbrack   \leq  x.\]

After calculating the constants from the kinematic constraints and given CLs,i.e., \( \max \left( {\frac{{I}_{k}}{{V}_{\max }^{k}},\frac{{\theta }_{k}}{{V}_{\max }^{k}},\frac{{h}_{k,k}}{{V}_{\max }^{k}},{T}_{k - 1,d}}\right) ,\max \left( {\frac{{I}_{k}}{{A}_{\max }^{k}},\frac{{\theta }_{k}}{{A}_{\max }^{k}},\frac{{h}_{k,k}}{{A}_{\max }^{k}}}\right) \) and \( \max \left( {\frac{{I}_{k}}{{I}_{\max }^{p}},\frac{{\theta }_{k}}{{I}_{\max }^{p}},\frac{{R}_{pk}}{{I}_{\max }^{p}}}\right) \) ,the problem only involves linear and log-sum-exp constraints w.r.t. \( x,y,z \) and \( t \) . The log-sum-exp constraints can be represented using two sets of exponential cones,

\[\left\{  {\begin{matrix} \left( {{u}_{1},1,x - t}\right)  \in  {K}_{\exp }, \\  \left( {{u}_{2},1,y - t}\right)  \in  {K}_{\exp }, \\  \left( {{u}_{3},1,z - t}\right)  \in  {K}_{\exp }, \\  {u}_{1} + {u}_{2} + {u}_{3} = 1. \end{matrix}\text{and}\left\{  \begin{matrix} \left( {{v}_{1},1,y - x}\right)  \in  {K}_{\exp }, \\  \left( {{v}_{2},1,z - x}\right)  \in  {K}_{\exp }, \\  {v}_{1} + {v}_{2} = 1. \end{matrix}\right. }\right.  \tag{8}\]

where \( {K}_{\text{exp }} \) is the primal exponential cone domain defined as,

\[{K}_{\text{exp }} = \left\{  {\left( {{x}_{1},{x}_{2},{x}_{3}}\right)  \in  {\mathbb{R}}^{3} : {x}_{1} \geq  {x}_{2}\exp \left( {{x}_{3}/{x}_{2}}\right) ,{x}_{1},{x}_{2} \geq  0}\right\}  . \tag{9}\]

As a result, the time constants can be efficiently solved through exponential cone optimization, utilizing the state-of-the-art solver, MOSEK [32].

### 2.3. Kinematic approximation

The time constants' kinematic constraints are adopted as Eq. (4), considering both the tool and joint kinematic constraints at the same time. The TCP and TOV motions share the same time constants, guaranteeing multi-axis synchronization, as shown in Fig. 2. The filtered velocity \( {\mathbf{v}}_{k} \) and angular velocity \( {\omega }_{k} \) guarantees the constraints on the tool motion. To ensure joint motion constraints in the optimization process, a kinematic approximation is adopted in Eq. (4), preserving the convexity of Eq. (7). In the following section, we provide an accuracy analysis that demonstrates the effectiveness of the joint kinematic constraints, despite being an approximation.

The joint \( \mu \) trajectory of the \( {k}^{\text{th }} \) segment is parameterized by the arc length \( s \) of the TCP path,i.e., \( {q}_{\mu ,k}\left( s\right) \) . The differentiation w.r.t. time \( t \) yields the joint velocity \( {q}_{\mu }^{\prime } \) ,acceleration \( {q}_{\mu }^{\prime \prime } \) and jerk \( {q}_{\mu }^{\prime \prime } \) ,

\[{q}_{\mu }^{\prime } = {q}_{s}{s}^{\prime }\]

\[{q}_{\mu }^{\prime \prime } = {q}_{s}{s}^{\prime \prime } + {q}_{ss}{\left( {s}^{\prime }\right) }^{2}, \tag{10}\]

\[{q}_{\mu }^{\prime \prime \prime } = {q}_{s}{s}^{\prime \prime \prime } + 3{q}_{ss}{s}^{\prime }{s}^{\prime \prime } + {q}_{sss}{\left( {s}^{\prime }\right) }^{3}.\]

In this paper, \( {\Lambda }^{\prime },{\Lambda }^{''} \) and \( {\Lambda }^{''} \) are the first,second and third order derivatives of \( \Lambda \) w.r.t. time \( t.{q}_{s},{q}_{ss} \) and \( {q}_{sss} \) are the first,second and third order derivatives of the joint \( \mu \) w.r.t. the arc length \( s.{\widehat{q}}_{s} \) is approximated with the two-point-forward-difference formula,

\[{\widehat{q}}_{s} \approx  \frac{{q}_{\mu ,k + 1} - {q}_{\mu ,k}}{{L}_{k}}. \tag{11}\]

Approximation of the joint velocity \( {\widehat{q}}_{\mu } \) ,acceleration \( {\widehat{q}}_{\mu } \) and jerk \( {\widehat{q}}_{\mu } \) with the first linear term yields that,

\[{\widehat{q}}_{\mu }^{\prime } = {\widehat{q}}_{s}{s}^{\prime },{\widehat{q}}_{\mu }^{\prime \prime } = {\widehat{q}}_{s}{s}^{\prime \prime },{\widehat{q}}_{\mu }^{\prime \prime \prime } = {\widehat{q}}_{s}{s}^{\prime \prime \prime }, \tag{12}\]

<!-- Meanless: 6-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

where \( {s}^{\prime },{s}^{\prime \prime } \) and \( {s}^{\prime \prime } \) are bounded by the time constants as follows,

\[s \leq  \frac{{L}_{k}}{{T}_{k,1}},{s}^{\prime \prime } \leq  \frac{{L}_{k}}{{T}_{k,1}{T}_{k,2}},{s}^{\prime \prime \prime } \leq  \frac{{L}_{k}}{{T}_{k,1}{T}_{k,2}{T}_{k,3}}, \tag{13}\]

Therefore, when Eq. (12) is accurately approximated, the maximum joint constraints are bounded in Eq. (4) as follows,

\[\max \left| {\widehat{q}}_{\mu }^{\prime }\right|  \leq  \frac{{h}_{\mu ,k}}{{T}_{k,1}} \leq  {V}_{\max },\]

\[\max \left| {\widehat{q}}_{\mu }^{''}\right|  \leq  \frac{{h}_{\mu ,k}}{{T}_{k,1}{T}_{k,2}} \leq  {A}_{\max }, \tag{14}\]

\[\max \left| {\widehat{q}}_{\mu }^{'''}\right|  \leq  \frac{{h}_{\mu ,k}}{{T}_{k,1}{T}_{k,2}{T}_{k,3}} \leq  {J}_{\max }.\]

The errors come from two factors, the differential accuracy of Eq. (11) and the neglection of the nonlinear terms in Eq. (10). The differential accuracy is respected when the discrete CLs are densely generated in the CAM and the nonlinear terms are limited when the toolpath is smooth. In Section 4, examples are analyzed to determine the impact of these errors.

## 3. Local corner blending

Section 2 presents the P2P interpolation technique for hybrid robots, which ensures accurate geometry at the corners through full-stop motion. To further enhance efficiency, this section proposes a non-stop interpolation method. By blending adjacent kinematic profiles, the machining time is significantly reduced [20]. However, two issues emerge with the blending: geometric errors and kinematic constraints violation. These problems are discussed in Section 3.1 and 3.2.

### 3.1. Geometric errors estimation

P2P interpolation determines the delay time after each velocity pulse as \( {T}_{k,d} = {T}_{k,2} + {T}_{k,3} \) . To achieve non-stop interpolation, adjacent pulses are blended with a smaller delay time \( {T}_{k,c} < {T}_{k,d} \) . This blending results in geometric errors at the corners,as shown in Fig. 3. The actual blending error is defined as \( {\epsilon }_{k}^{p} \) and \( {\epsilon }_{k}^{o} \) ,which is the minimum distance from the trajectory to the CL at the \( {k}^{\text{th }} \) corner. These errors can be reduced by increasing \( {T}_{k,c} \) using an error estimation model \( {\widehat{\varepsilon }}_{k}^{p} \) and \( {\widehat{\varepsilon }}_{k}^{0} \) . The existing model in literature [26,27] is based on symmetric blending curves. To improve accuracy, this paper proposes a general error estimation model that considers different feeds and time constants. The mathematical models are elaborated in detail, with the cases where adjacent kinematic profiles intersect, and the estimation model for position and orientation paths.

The filtered velocity \( {v}_{k} \) is expressed as a piecewise analytic form consisting of seven pieces \( {\left\{  {v}_{k,i}\right\}  }_{i = 1}^{7} \) ,

\[{v}_{k} = \left\{  \begin{matrix} {v}_{k1} & {L}_{k} < {T}_{k}, & {L}_{1} \leq  t < {T}_{k,3} \\  {v}_{k1} = \frac{{L}_{k}}{2{L}_{1}{L}_{2}{L}_{3}} + \frac{{L}_{k}}{{L}_{1}{L}_{1}{L}_{2}}\left( {t - {T}_{k,3}}\right) , & {T}_{k,3} < t < {T}_{k,2} & \\  {v}_{k,3} = \frac{{L}_{k}}{2{L}_{1}{L}_{2}} - \frac{{L}_{k}}{2{L}_{1}{L}_{2}{L}_{3}}\left( {{T}_{k,a} + t}\right) , & {T}_{k,3} < t < {T}_{k,2} & \\  {v}_{k,4} = \frac{{L}_{k,1}}{2{L}_{1}} - \frac{{L}_{k}}{2{L}_{1}{L}_{2}{L}_{3}}\left( {{T}_{k,a} + t}\right) , & {T}_{k,4} < t < {T}_{k,1} & \\  {v}_{k,5} = \frac{{L}_{k}}{{L}_{1}} - \frac{{L}_{k}}{2{L}_{1}{L}_{2}{L}_{3}}\frac{{L}_{k}}{2{L}_{1}{L}_{2}}\left( {t - {T}_{k,1}}\right) , & {T}_{k,4} < t < {T}_{k,1} + {T}_{k,3} & \\  {v}_{k,6} = \frac{{L}_{k}}{{L}_{1}} - \frac{{L}_{k}}{2{L}_{1}{L}_{2}{L}_{3}} - \frac{{L}_{k}}{2{L}_{1}{L}_{2}{L}_{3}}\left( {t - {T}_{k,1}}\right) , & {T}_{k,1} + {T}_{k,2} < t < {T}_{k,1} + {T}_{k,3} & \\  {v}_{k,7} = \frac{{L}_{k,4}}{2{L}_{1}{L}_{2}{L}_{3}}\frac{{L}_{k}}{2{L}_{1}{L}_{2}{L}_{3}}\left( {t - {T}_{k,1}}\right) , & {T}_{k,1} + {T}_{k,2} < t < {T}_{k,1} + {T}_{k,2} & \\  {v}_{k,7} = \frac{{L}_{k,7}}{2{L}_{1}{L}_{2}{L}_{3}}\left( {t - {T}_{k,1}{L}_{3}{L}_{4}}\right) \left( {t - {T}_{k,2}{L}_{3}{L}_{4}}\right) \left( {t - {T}_{k,2}{L}_{3}{L}_{4}}\right) \left( {t - {T}_{k,2}{L}_{3}{L}_{4}}\right) & & \\   & &  \end{matrix}\right.  \tag{15}\]

To approximate the blending error, it is necessary to calculate the intersection time, which occurs at different pieces when the delay time \( {T}_{k,c} \) varies. For instance,Fig. 3(a) shows that \( {v}_{k,7} \) intersects \( {v}_{k + 1,2} \) ,forming a quadratic equation for \( {t}_{k} \) . In particular,when \( {v}_{k,6} \) intersects \( {v}_{k + 1,2} \) ,a linear equation is formed for \( {t}_{k} \) ,which is when the adjacent velocity \( {v}_{k} \) and \( {v}_{k + 1} \) intersect,

\[\frac{{L}_{k}}{{T}_{k,1}} - \frac{{L}_{k}{T}_{k,3}}{2{T}_{k,1}{T}_{k,2}} - \frac{{L}_{k}}{{T}_{k,1}{T}_{k,2}}\left( {{t}_{k} - {T}_{k,1} - {T}_{k,3}}\right)  = \frac{{L}_{k + 1}{T}_{k + 1,3}}{2{T}_{k + 1,1}{T}_{k + 1,2}} + \frac{{L}_{k + 1}}{{T}_{k + 1,1}{T}_{k + 1,2}}\left( {{t}_{k} - {T}_{m}}\right) . \tag{16}\]

<!-- Meanless: 7-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

here, \( {T}_{n} \) is the junction point between \( {v}_{k,6} \) and \( {v}_{k,7} \) ,i.e., \( {T}_{n} = {T}_{k,1} + {T}_{k,2}.{T}_{m} \) is the junction point between \( {v}_{k + 1,1} \) and \( {v}_{k + 1,2} \) ,i.e., \( {T}_{m} = {T}_{k,1} \) \( + {T}_{k,\mathrm{c}} + {T}_{k + 1,3} \) . Solving for \( {t}_{k} \) yields that,

\[{\underset{{T}_{m} \leq  {t}_{k} \leq  {T}_{n}}{\underbrace{\text{ Case }1}}}^{{L}_{k} = \frac{{L}_{k}}{{T}_{k,1}{T}_{k,2}} + \frac{{L}_{k}{T}_{k,3}}{2{T}_{k,1}{T}_{k,2}} + \frac{{L}_{k}}{{T}_{k,2}} + \frac{{L}_{k}{T}_{k,3}}{{T}_{k,1}{T}_{k,2}} - \frac{{L}_{k + 1}{T}_{k + 1,3}}{2{T}_{k + 1,1}{T}_{k + 1,2}} + \frac{{L}_{k + 1}{T}_{m}}{{T}_{k + 1,1}{T}_{k + 1,2}}} \tag{17}\]

If this \( {t}_{k} \) satisfies that \( {T}_{m} \leq  {t}_{k} \leq  {T}_{n} \) ,it is considered valid as Case 1 in Fig. 4. Otherwise,this \( {t}_{k} \) does not accurately represent the intersection time. Hence,to determine the intersection time,three cases are further divided by comparing the initial \( {t}_{k} \) with \( {T}_{n} \) and \( {T}_{m} \) . For instance,if \( {t}_{k} \geq  {T}_{m},{T}_{n} \) ,it falls under Case 2,where \( {v}_{k,7} \) intersects \( {v}_{k + 1,2} \) . In such case, \( {t}_{k}^{ * } \) is recalculated from the following quadratic equation,

\[\frac{{L}_{k}}{2{T}_{k,1}{T}_{k,2}{T}_{k,3}}{\left( {T}_{k,1} + {T}_{k,d} - {t}_{k}^{ * }\right) }^{2} = \frac{{L}_{k + 1}{T}_{k + 1,3}}{2{T}_{k + 1,1}{T}_{k + 1,2}} + \frac{{L}_{k + 1}}{{T}_{k + 1,1}{T}_{k + 1,2}}\left( {{t}_{k}^{ * } - {T}_{m}}\right) . \tag{18}\]

Arranging the terms yields that,

\[\underset{{t}_{k} \geq  {T}_{m},{T}_{n}}{\underbrace{\text{ Case }2}} : {t}_{k}^{ * } = \frac{-b - \sqrt{{b}^{2} - {4ac}}}{2a}\text{,where}\]

\[\left\{  \begin{array}{l} a = \frac{{L}_{k}}{2{T}_{k,1}{T}_{k,2}{T}_{k,3}}, \\  b =  - \frac{{L}_{k}\left( {{T}_{k,1} + {T}_{k,d}}\right) }{{T}_{k,1}{T}_{k,2}{T}_{k,3}} - \frac{{L}_{k + 1}}{{T}_{k + 1,1}{T}_{k + 1,2}}, \\  c = \frac{{L}_{k}{\left( {T}_{k,1} + {T}_{k,d}\right) }^{2}}{2{T}_{k,1}{T}_{k,2}{T}_{k,3}} + \frac{{L}_{k + 1}{T}_{m}}{{T}_{k + 1,1}{T}_{k + 1,2}} - \frac{{L}_{k + 1}{T}_{k + 1,3}}{2{T}_{k + 1,1}{T}_{k + 1,2}}. \end{array}\right.  \tag{19}\]

Similarly,the correct \( {t}_{k}^{ * } \) of the remaining cases are provided below,

\[\underset{{t}_{k} \leq  {T}_{m},{T}_{n}}{\underbrace{\text{ Case }3}} : {t}_{k}^{ * } = \frac{-b + \sqrt{{b}^{2} - {4ac}}}{2a}\text{,where }\]

\[\left\{  \begin{matrix} a = \frac{{L}_{k + 1}}{2{T}_{k + 1,1}{T}_{k + 1,2}{T}_{k + 1,3}}, \\  b = \frac{{L}_{k}}{{T}_{k,1}{T}_{k,2}} - \frac{{L}_{k + 1}\left( {{T}_{k,1} + {T}_{k,c}}\right) }{{T}_{k + 1,1}{T}_{k + 1,2}{T}_{k + 1,3}}, \\  c = \frac{{L}_{k + 1}{\left( {T}_{k,1} + {T}_{k,c}\right) }^{2}}{2{T}_{k + 1,1}{T}_{k + 1,2}{T}_{k + 1,3}} - \frac{{L}_{k}}{{T}_{k,1}} + \frac{{L}_{k}{T}_{k,3}}{2{T}_{k,1}{T}_{k,2}} - \frac{{L}_{k}\left( {{T}_{k,1} + {T}_{k,3}}\right) }{{T}_{k,1}{T}_{k,2}} \end{matrix}\right. \]

\[\underset{{T}_{n} \leq  {t}_{k} \leq  {T}_{m}}{\underbrace{\text{ Case }4}} : {t}_{k}^{ * } = \left\{  {\begin{array}{ll} \frac{-b \pm  \sqrt{{b}^{2} - {4ac}}}{2a}, & a \neq  0 \\   - \frac{c}{b}, & a = 0 \end{array},\text{ where }}\right.  \tag{20}\]

\[\left\{  \begin{matrix} a = \frac{{L}_{k + 1}}{2{T}_{k + 1,1}{T}_{k + 1,2}{T}_{k + 1,3}} - \frac{{L}_{k}}{2{T}_{k,1}{T}_{k,2}{T}_{k,3}}, \\  b =  - \frac{{L}_{k + 1}\left( {{T}_{k,1} + {T}_{k,c}}\right) }{{T}_{k + 1,1}{T}_{k + 1,2}{T}_{k + 1,3}} + \frac{{L}_{k}\left( {{T}_{k,1} + {T}_{k,d}}\right) }{{T}_{k,1}{T}_{k,2}{T}_{k,3}}, \\  c = \frac{{L}_{k + 1}{\left( {T}_{k,1} + {T}_{k,c}\right) }^{2}}{2{T}_{k + 1,1}{T}_{k + 1,2}{T}_{k + 1,3}} - \frac{{L}_{k}{\left( {T}_{k,1} + {T}_{k,d}\right) }^{2}}{2{T}_{k,1}{T}_{k,2}{T}_{k,3}}. \end{matrix}\right. \]

When solving a quadratic equation, the geometry between the velocity pulses determines which of the two roots to select. In Case 2, the smaller root is chosen, while in Case 3, the larger root is chosen. On the other hand, in Case 4, the root that satisfies \( {T}_{k,1} + {T}_{k,c} \leq  {t}_{k}^{ * } \leq  {T}_{k,1} + {T}_{k,d} \) is chosen. A special scenario to note is when both \( a \) and \( b \) are zero in Case 4. This results in \( {T}_{k,c} \) being equal to \( {T}_{k,d} \) ,and therefore causing the error to be zero. For now,our focus is on these four cases as they provide sufficient accuracy. Adding other edge cases will only increase complexity without significant improvement. The error tolerance of the proposed model is verified with examples in Section 4.

The blending path deviates from the \( {k}^{\text{th }} \) segments at the time \( {t}_{\mathrm{s}} = {T}_{k,1} + {T}_{k,c} \) ,and returns to the \( {\left( k + 1\right) }^{\text{th }} \) segment at the time \( {t}_{e} = {T}_{k,1} \) \( + {T}_{k,d} \) ,as shown in Fig. 5(a). To pinpoint \( \mathbf{P}\left( {t}_{k}\right) \) ,the displacement \( {l}_{dec} \) and \( {l}_{acc} \) are found through the integral of the velocity profile at the

<!-- Meanless: 8-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: (a) Linear path (b) \( {\mathbf{O}}_{k + 1} \) \( \mathbf{O}\left( {t}_{s}\right) \) ； \( \mathbf{O}\left( {t}_{k}\right) \) \( \mathbf{O}\left( {t}_{e}\right) \) (c) \( {\mathbf{O}}_{k + 1} \) \( {\mathbf{V}}_{n} \) 0 \( \mathbf{O}\left( {t}_{k}\right) \) \( {O}_{x} \) Om 0 Blending path 1 \( {\mathbf{O}}_{k + 1} \) \( {\mathbf{O}}_{k + 2} \) \( {\mathbf{O}}_{k} \) 0 -1 1 0 - 1 - 1 \( {o}_{y} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_8.jpg?x=169&y=156&w=1368&h=789&r=0"/>

Fig. 6. Orientation blending error estimation. (a) The corner blending on the sphere. (b) The zoomed view. (c) The annotated view.

<!-- Media -->

intersection time \( {t}_{k} \) ,

\[{l}_{dec} = {\int }_{{t}_{k}}^{{t}_{e}}{v}_{k}\mathrm{\;d}t\]

\[ = \left\{  {\begin{matrix} \frac{{L}_{k}}{-6{T}_{k,1}{T}_{k,2}{T}_{k,3}}{\left( {t}_{k} - {T}_{k,1} - {T}_{k,d}\right) }^{3}, & {t}_{k} > {T}_{n} \\  \left( {\frac{{L}_{k}}{{T}_{k,1}} + \frac{{L}_{k}}{{T}_{k,2}} + \frac{{L}_{k}{T}_{k,3}}{2{T}_{k,1}{T}_{k,2}}}\right) \left( {{T}_{n} - {t}_{k}}\right) & \\   - \frac{{L}_{k}}{2{T}_{k,1}{T}_{k,2}}{\left( {T}_{k,1} + {T}_{k,2}\right) }^{2} + \frac{{L}_{k}}{2{T}_{k,1}{T}_{k,2}}{t}_{k}^{2} + \frac{{L}_{k}{T}_{k,3}^{2}}{6{T}_{k,1}{T}_{k,2}}, & {t}_{k} \leq  {T}_{n} \end{matrix}.}\right.  \tag{21}\]

\[{l}_{acc} = {\int }_{{t}_{s}}^{{t}_{k}}{v}_{k + 1}\mathrm{\;d}t\]

\[ = \left\{  {\begin{matrix} \frac{{L}_{k + 1}{T}_{k + 1,3}^{2}}{6{T}_{k + 1,1}{T}_{k + 1,2}} - \frac{{L}_{k + 1}{T}_{k + 1,3}}{2{T}_{k + 1,1}{T}_{k + 1,2}}\left( {{t}_{k} - {T}_{m}}\right) & \\   + \frac{{L}_{k + 1}}{2{T}_{k + 1,1}{T}_{k + 1,2}}\left\lbrack  {{\left( {t}_{k} - {T}_{k,c} - {T}_{k,1}\right) }^{2} - {T}_{k + 1,3}^{2}}\right\rbrack  , & {t}_{k} > {T}_{m} \\  \frac{{L}_{k + 1}}{6{T}_{k + 1,1}{T}_{k + 1,2}{T}_{k + 1,3}}{\left( {t}_{k} - {T}_{k,c} - {T}_{k,1}\right) }^{3}, & {t}_{k} \leq  {T}_{m} \end{matrix}.}\right. \]

In cases of asymmetric blending,estimating the blending error as \( \begin{Vmatrix}{\mathbf{P}\left( {t}_{k}\right)  - {\mathbf{P}}_{k + 1}}\end{Vmatrix} \) may overestimate \( {\varepsilon }_{k}^{p} \) . While the calculation of \( {\varepsilon }_{k}^{p} \) requires numerical iterations [15],we propose to estimate the error analytically with \( {\widehat{\epsilon }}_{k}^{p} = \begin{Vmatrix}{{\mathbf{P}}_{m} - {\mathbf{P}}_{k + 1}}\end{Vmatrix} \) . Here, \( {\mathbf{P}}_{m} \) is the projection of \( \mathbf{P}\left( {t}_{k}\right) \) on the angle bisector as shown in Fig. 5(b). The half angle formula is then used to calculate the estimated blending error \( {\widehat{\varepsilon }}_{k}^{p} \) ,taking into account the property of parallel lines and geometric symmetry,

\[{\widehat{\varepsilon }}_{k}^{p} = \frac{{l}_{acc} + {l}_{dec}}{2}\sqrt{2 - 2\cos \left( {\theta }_{p}\right) }, \tag{22}\]

where \( {\theta }_{p} \) is the angle between the adjacent linear segments.

Meanwhile, \( \mathbf{O}\left( {t}_{k}\right) \) is approximated on the tangent plane at \( {\mathbf{O}}_{k + 1} \) ,as depicted in Fig. 6,

<!-- Meanless: 9-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

\[\mathbf{O}\left( {t}_{k}\right)  = \frac{{\mathbf{O}}_{c}}{\begin{Vmatrix}{\mathbf{O}}_{c}\end{Vmatrix}},{\mathbf{O}}_{c} = {\mathbf{O}}_{k + 1} + {\theta }_{dec}{\mathbf{V}}_{m} + {\theta }_{acc}{\mathbf{V}}_{n},\]

\[{\mathbf{V}}_{m} = {\mathbf{O}}_{k + 1} \times  \left( {{\mathbf{O}}_{k + 1} \times  \left( {{\mathbf{O}}_{k} - {\mathbf{O}}_{k + 1}}\right) }\right) , \tag{23}\]

\[{\mathbf{V}}_{n} = {\mathbf{O}}_{k + 1} \times  \left( {{\mathbf{O}}_{k + 1} \times  \left( {{\mathbf{O}}_{k + 2} - {\mathbf{O}}_{k + 1}}\right) }\right) ,\]

<!-- Media -->

<!-- figureText: (a) Reverse-direction case \( {\mathcal{I}}_{v} = \varnothing \) Same-direction case \( {\mathcal{I}}_{v} = \left\lbrack  {0,{T}_{k,d} - {T}_{k + 1,d}}\right\rbrack \) \( {T}_{k,c} \) velocity \( {T}_{k + 1,d} \) \( {T}_{k,d} \) time Same-direction case \( {\mathcal{I}}_{a} = \varnothing \) acceleration time Same-direction case \( {\mathcal{I}}_{j} = \left\lbrack  {{T}_{k,2} - {T}_{k + 1,3},{T}_{k,d}}\right\rbrack \) \( {T}_{k,c} \) time \( -  - {\widehat{h}}_{\mu ,k}^{\prime } -  - {\widehat{h}}_{\mu ,k + 1}^{\prime } - {\widehat{h}}_{\mu }^{\prime } \) \( \max \left| {\widehat{h}}_{\mu ,k}^{\prime }\right| \) velocity time (b) Reverse-direction case \( {\mathcal{I}}_{a} = \left\lbrack  {0,{T}_{k,d}}\right\rbrack \) acceleration \( - {\widehat{h}}_{\mu ,k}^{\prime \prime } -  - {\widehat{h}}_{\mu ,k + 1}^{\prime \prime } - {\widehat{h}}_{\mu }^{\prime \prime } \) \( \max {\widehat{h}}_{\mu }^{\prime \prime } \) time \( {T}_{k,c} \) (c) Reverse-direction case \( {\mathcal{I}}_{j} = \left\lbrack  {0,{T}_{k,3}}\right\rbrack   \cup  \left\lbrack  {{T}_{k,2} - {T}_{k + 1,d},{T}_{k,d} - {T}_{k + 1,2}}\right\rbrack \) \( -  - {\widehat{h}}_{\mu ,k}^{\prime \prime \prime } -  - {\widehat{h}}_{\mu ,k + 1}^{\prime \prime \prime } - {\widehat{h}}_{\mu }^{\prime \prime \prime } \) jerk time -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_9.jpg?x=165&y=154&w=1370&h=1180&r=0"/>

Fig. 7. Kinematic constraints in blending. (a) Velocity. (b) Acceleration. (c) Jerk.

<!-- Media -->

\( {\theta }_{\text{dec }} \) and \( {\theta }_{\text{acc }} \) are calculated with Eq. (21),where the intersection time \( {t}_{k} \) is adjusted based on the orientation trajectory,and the displacement \( L \) replaced with the angle \( \theta \) . Here, \( \times \) stands for cross product. To estimate the orientation blending error \( {\widehat{\varepsilon }}_{k}^{o} \) ,we use the following approximation,

\[{\widehat{\varepsilon }}_{k}^{o} = {\cos }^{-1}\left( \frac{{\mathbf{O}}_{m}^{\mathrm{T}}{\mathbf{O}}_{k + 1}}{\begin{Vmatrix}{\mathbf{O}}_{m}\end{Vmatrix}}\right) , \tag{24}\]

where \( {\mathbf{O}}_{m} \) is the midpoint on the unit sphere between \( \mathbf{O}\left( {t}_{k}\right) \) and its mirrored counterpart \( {\mathbf{O}}^{ * } \) shown in Fig. 6(c),

\[{\mathbf{O}}_{m} = \frac{{\mathbf{O}}^{ * } + \mathbf{O}\left( {t}_{k}\right) }{2},{\mathbf{O}}^{ * } = \frac{{\mathbf{O}}_{c}^{ * }}{\begin{Vmatrix}{\mathbf{O}}_{c}^{ * }\end{Vmatrix}},{\mathbf{O}}_{c}^{ * } = {\mathbf{O}}_{k + 1} + {\theta }_{acc}{\mathbf{V}}_{k} + {\theta }_{dec}{\mathbf{V}}_{k + 1}. \tag{25}\]

Finally, \( {T}_{k,c} \) is determined with a bisection method proposed in our prior research [15]. This method ensures that the predicted blending errors meet the tolerance level set by the user,i.e., \( {\widehat{\varepsilon }}_{k}^{p} < {\varepsilon }_{\max }^{p} \) and \( {\widehat{\varepsilon }}_{k}^{o} < {\varepsilon }_{\max }^{o} \) . The error constraining robustness is verified in Section 4 with examples.

<!-- Meanless: 10-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: The MCS Coordinates Input G01 Commands \( \left\lbrack  {{\mathbf{P}}_{k},{\mathbf{O}}_{k}}\right\rbrack  ,k = 1,2,\cdots ,N \) \( {\mathbf{V}}_{k}\left\lbrack  n\right\rbrack  ,{\mathbf{\Omega }}_{k}\left\lbrack  n\right\rbrack \) velocity Filtering time acceleration \( {T}_{k,1} + {T}_{k,d} \) \( {T}_{k,3}{T}_{k,2}{T}_{k,d} \) time Blending \( {v}_{k,4} \) velocity profile \( {v}_{k,j} \) \( {v}_{k,i} \) \( {v}_{k,6} \) \( {T}_{k,c} \) \( {v}_{k,6} \) \( {T}_{k,1} \) \( {T}_{k,d} \) time TCP and TOV trajectories \( {\mathbf{Q}}_{k} = {\left\lbrack  {q}_{1,k},{q}_{2,k},{q}_{3,k},{q}_{4,k},{q}_{5,k}\right\rbrack  }^{1} \) \( {L}_{k},{\theta }_{k},{h}_{\mu ,k} \) Optimize time constants for each segment (Section 2.2) \( {T}_{k,i}^{ * } = \arg \mathop{\min }\limits_{{T}_{k,i}}\mathop{\sum }\limits_{{i = 1}}^{3}{T}_{k,i} \) ,s.t. Input: kinematic \( {T}_{k,1} \geq  \max \left( {\frac{{L}_{k}}{{V}_{\max }^{p}},\frac{{\theta }_{k}}{{V}_{\max }^{o}},\frac{{h}_{\mu ,k}}{{V}_{\max }^{\mu }}}\right) , \) constraint \( {V}_{\max }^{p},{A}_{\max }^{p},{J}_{\max }^{p} \) \( {T}_{k,1}{T}_{k,2} \geq  \max \left( {\frac{{L}_{k}}{{A}_{\max }^{p}},\frac{{\theta }_{k}}{{A}_{\max }^{o}},\frac{{h}_{\mu ,k}}{{A}_{\max }^{\mu }}}\right) , \) \( {V}_{\max }^{o},{A}_{\max }^{o},{J}_{\max }^{o} \) \( {V}_{\max }^{\mu },{A}_{\max }^{\mu },{J}_{\max }^{\mu } \) \( {T}_{k,1}{T}_{k,2}{T}_{k,3} \geq  \max \left( {\frac{{L}_{k}}{{J}_{\max }^{p}},\frac{{\theta }_{k}}{{J}_{\max }^{o}},\frac{{h}_{\mu ,k}}{{J}_{\max }^{\mu }}}\right) . \) \( {T}_{k,1} \geq  {T}_{k,2} + {T}_{k,3},{T}_{k,2} \geq  {T}_{k,3} \) . \( {T}_{k,1} \geq  {T}_{k - 1,d},k > 1 \) . Input: blending Calculate delay time \( {T}_{k,c} \) for each corner error tolerance \( {\widehat{\varepsilon }}_{k}^{p} < {\varepsilon }_{\max }^{p},{\widehat{\varepsilon }}_{k}^{o} < {\varepsilon }_{\max }^{o} \) (Section 3.1) \( {\mathcal{E}}_{\max }^{p},{\mathcal{E}}_{\max }^{o} \) Fix kinematic violations in blending \( {T}_{k,c}^{ * } \) and \( {T}_{k,1}^{ * },{T}_{k + 1,1}^{ * } \) (Section 3.2) \( {\left\{  {T}_{k,i}\right\}  }_{\begin{matrix} {i = 1,2,3} \\  {k = 1,\cdots ,N - 1} \end{matrix}},{\left\{  {T}_{k,c}\right\}  }_{k = 1,\cdots ,N - 2} \) Output: joint trajectories -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_10.jpg?x=165&y=150&w=1370&h=1272&r=0"/>

Fig. 2. The interpolation strategy.

<!-- Media -->

### 3.2. Kinematic constraints in blending

Section 2.2 focuses solely on the constraints along the linear toolpath, but when blending occurs, a smooth trajectory is generated at each corner which could potentially violate the kinematic constraints. To ensure these constraints are not violated on the blended trajectory, this section discusses the cases where the constraints can be breached and proposes solutions to fix them.

When \( {T}_{k,c} < {T}_{k,d} - {T}_{k + 1,d} \) ,blending same direction feeds can lead to a breach of velocity constraints,as shown in Fig. 7(a). In such case, the maximum velocity after blending can be approximated as follows,

\[\max \left| {\widehat{h}}_{\mu }^{\prime }\right|  = \frac{{h}_{\mu ,k + 1}}{{T}_{k + 1,1}} + \frac{{h}_{\mu ,k}}{{T}_{k,1}}\frac{{T}_{k,d} - {T}_{k + 1,d} - {T}_{k,c}}{{T}_{k,d}}, \tag{26}\]

where \( {h}_{\mu ,k} \) represents the absolute displacement at the \( {k}^{\text{th }} \) segment of the axis \( \mu \) . To ensure proper blending,it is necessary to check all eleven axes (six in the WCS, \( {\left\{  {p}_{i}\right\}  }_{i = x,y,z},{\left\{  {o}_{i}\right\}  }_{i = x,y,z} \) and five in the MCS, \( {\left\{  {q}_{i}\right\}  }_{i = 1}^{5} \) ) to find axes that would exceed the velocity constraints,

\[{\mathcal{C}}_{v} = \left\{  {\mu \left| \max \right| {h}_{\mu }^{\prime } \mid   > {V}_{\max }^{\mu },{T}_{k,c} \in  {\mathcal{J}}_{v}}\right\}  . \tag{27}\]

To determine the final delay time \( {T}_{k,c}^{ * } \) ,we consider the minimum value allowed by the set \( {\mathcal{E}}_{v} \) and the original value determined by the geometric error,

<!-- Meanless: 11-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

\[{T}_{k,c}^{ * } = \mathop{\max }\limits_{{\mu  \in  {\mathcal{C}}_{v}}}\left\{  {{T}_{\mu ,k,c},{T}_{k,c}}\right\}  , \tag{28}\]

<!-- Media -->

Table 1

The kinematic constraints.

<table><tr><td>Symbol</td><td>Parameter</td><td>Simulation</td><td>Experiment</td></tr><tr><td>\( {V}_{\max }^{p}\left( {\mathrm{{mm}}/\mathrm{s}}\right) \)</td><td>TCP axial velocity</td><td>20</td><td>4</td></tr><tr><td>\( {A}_{\max }^{p}\left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) \)</td><td>TCP axial acceleration</td><td>150</td><td>40</td></tr><tr><td>\( {J}_{\max }^{p}\left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \)</td><td>TCP axial jerk</td><td>3000</td><td>400</td></tr><tr><td>\( {V}_{\max }^{o}\left( {\mathrm{{rad}}/\mathrm{s}}\right) \)</td><td>TOV angular velocity</td><td>0.5</td><td>0.15</td></tr><tr><td>\( {A}_{\max }^{o}\left( {\mathrm{{rad}}/{\mathrm{s}}^{2}}\right) \)</td><td>TOV angular acceleration</td><td>2.5</td><td>1.5</td></tr><tr><td>\( {J}_{\max }^{o}\left( {\mathrm{{rad}}/{\mathrm{s}}^{3}}\right) \)</td><td>TOV angular jerk</td><td>50</td><td>15</td></tr><tr><td>\( {\left\{  {V}_{\max }^{\mu }\right\}  }_{\mu  = 1}^{5}\left( {\mathrm{\;{mm}}/\mathrm{s}}\right) \)</td><td>Joint velocity</td><td>20</td><td>15</td></tr><tr><td>\( {\left\{  {A}_{\max }^{\mu }\right\}  }_{\mu  = 1}^{5}\left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) \)</td><td>Joint acceleration</td><td>100</td><td>75</td></tr><tr><td>\( {\left\{  {J}_{\max }^{\mu }\right\}  }_{\mu  = 1}^{5}\left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \)</td><td>Joint jerk</td><td>2000</td><td>750</td></tr></table>

Table 2

The CLs in the illustrative example.

<table><tr><td>CLs</td><td>TCPs (mm)</td><td>TOVs</td></tr><tr><td>\( {\mathbf{P}}_{0},{\mathbf{O}}_{0} \)</td><td>\( {\left\lbrack  -{143.0}, - {98.0},{13.8}\right\rbrack  }^{\mathrm{T}} \)</td><td>\( {\left\lbrack  -{0.3143}, - {0.1467},{0.9379}\right\rbrack  }^{\mathrm{T}} \)</td></tr><tr><td>\( {\mathbf{P}}_{1},{\mathbf{O}}_{1} \)</td><td>\( {\left\lbrack  -{139.8}, - {97.0},{15.0}\right\rbrack  }^{\mathrm{T}} \)</td><td>\( {\left\lbrack  -{0.3337}, - {0.1402},{0.9322}\right\rbrack  }^{\mathrm{T}} \)</td></tr><tr><td>\( {\mathbf{P}}_{2},{\mathbf{O}}_{2} \)</td><td>\( {\left\lbrack  -{132.0}, - {100.0},{14.0}\right\rbrack  }^{\mathrm{T}} \)</td><td>\( {\left\lbrack  -{0.3526}, - {0.1506},{0.9236}\right\rbrack  }^{\mathrm{T}} \)</td></tr></table>

<!-- Media -->

where the minimum allowable value is calculated from Eq. (26) as follows,

\[{T}_{\mu ,k,c} = {T}_{k,d}\left( {1 - \frac{{T}_{k,1}{T}_{k + 1,1}{V}_{\max }^{\mu } - {T}_{k,1}{h}_{\mu ,k + 1}}{{T}_{k + 1,1}{h}_{\mu ,k}}}\right)  - {T}_{k + 1,d}. \tag{29}\]

Fig. 7(b) and (c) demonstrate that the blending process may result in acceleration and jerk violations even after resolving velocity issues. To address this,the feed duration at the corners is extended by finding new values of \( {T}_{k,1}^{ * } \) and \( {T}_{k + 1,1}^{ * } \) . The extension is minimized by distributing it between the neighboring feeds of the corners while adhering to the constraints. This creates another GP problem with kinematic approximation (Section 2.3).

\[\min {T}_{k,1}^{ * } + {T}_{k + 1,1}^{ * }\]

\[\text{s.t.}{T}_{k,1}^{ * } \geq  {T}_{k,1},{T}_{k + 1,1}^{ * } \geq  {T}_{k + 1,1}\text{,}\]

\[\frac{{h}_{\mu ,k}}{{T}_{k,1}^{ * }{T}_{k,2}} + \frac{{h}_{\mu ,k + 1}}{{T}_{k + 1,1}^{ * }{T}_{k + 1,2}} \leq  {A}_{\max }^{\mu },\mu  \in  {\mathcal{C}}_{a}, \tag{30}\]

\[\frac{{h}_{\mu ,k}}{{T}_{k,1}^{ * }{T}_{k,2}{T}_{k,3}} + \frac{{h}_{\mu ,k + 1}}{{T}_{k + 1,1}^{ * }{T}_{k + 1,2}{T}_{k + 1,3}} \leq  {J}_{\max }^{\mu },\mu  \in  \mathcal{C},\]

The axes that would violate the acceleration or jerk constraints are identified as follows,

\[{\mathcal{C}}_{a} = \left\{  {\mu \left| \max \right| {\widehat{h}}_{\mu }^{''} \mid   > {A}_{\max }^{\mu },{T}_{k,c} \in  {\mathcal{F}}_{a}}\right\}  , \tag{31}\]

\[{\mathcal{C}}_{j} = \left\{  {\mu \left| \max \right| {\widehat{h}}_{\mu }^{'''} \mid   > {J}_{\max }^{\mu },{T}_{k,c} \in  {\mathcal{J}}_{j}}\right\}  .\]

In Fig. 7(b), it is noted that only the cases where the feed direction is reversed across the corner can potentially violate the acceleration constraints, provided that the constraints are already met along the linear segments. Hence, cases with the same direction can be skipped. The maximum blending acceleration is estimated as \( \max \left| {\widehat{h}}^{\prime }\right|  = \max \left| {\widehat{h}}_{\mu ,k}\right|  + \max \left| {\widehat{h}}_{\mu ,k + 1}^{\prime }\right| \) in Eq. (30) from Eq. (14),where \( \max \left| {\widehat{h}}_{\mu ,k}\right|  = \frac{{h}_{\mu ,k}}{{T}_{k,1}{T}_{k,2}} \) . On the other hand,Fig. 7(c) shows that jerk violation occurs when the delay time \( {T}_{k,c} \) triggers the maximum jerk overlap between adjacent segments, regardless of the feed direction. The maximum blending jerk is estimated as max \( {\widehat{h}}_{\mu }\left| { = \max }\right| {\widehat{h}}_{\mu ,k}\left| {+\max }\right| {\widehat{h}}_{\mu ,k + 1}\left| {\text{in Eq. (30),where}\max }\right| {\widehat{h}}_{\mu ,k}\left| { = \frac{{h}_{\nu ,k}}{{T}_{k1}{T}_{k2}{T}_{k3}}\text{. The effectiveness of the extension of feed durations is guaranteed}}\right| \) as it does not affect the determination of the violated axis \( {\mathcal{C}}_{a},{\mathcal{C}}_{j} \) ,allowing the other axis to remain constrained.

Similarly,assuming that \( {T}_{k,1}^{ * } = \exp \left( x\right) ,{T}_{k + 1,1}^{ * } = \exp \left( y\right) \) converts Eq. (30) into a convex problem,

<!-- Meanless: 12-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: (a) 20 15 feedrate (mm/s) 10 -97 -99 -134 -132 \( {p}_{y}\left( \mathrm{{mm}}\right) \) (c) \( {q}_{1} - {q}_{2} - {q}_{3} - {q}_{4} - {q}_{5} \) \( \cdots {\hat{q}}_{1}\cdots {\hat{q}}_{2}\cdots {\hat{q}}_{3}\cdots {\hat{q}}_{4}\cdots {\hat{q}}_{5} \) \( {q}_{\mu }^{\prime }/{V}_{max}^{\mu } \) 0.5 0.5 0 0 -0.5 -0.5 0 1 \( {q}_{\mu }^{\prime \prime }/{A}_{max}^{\mu } \) 0 \( {\widehat{q}}_{\mu }^{\prime \prime }/{A}_{max}^{\mu } \) - 1 0 1 \( {q}_{\mu }^{\prime \prime \prime }/{J}_{max}^{\mu } \) 0 - 1 0 1 time(s) \( {p}_{z}\left( \mathrm{{mm}}\right) \) 15 14 -142 -140 -138 -136 \( {p}_{x}\left( \mathrm{{mm}}\right) \) (b) 1 \( \begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}/{V}_{ma}^{p} \) 0.5 0.5 0 0 0.5 1 \( \parallel {\mathbf{v}}_{k}{\parallel }^{\prime }/{A}_{max}^{p} \) 0 \( {\begin{Vmatrix}{\omega }_{k}\end{Vmatrix}}^{\prime }/{A}_{max}^{o} \) 0 0.5 1 \( {\begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}}^{\prime \prime }/{J}_{max}^{p} \) 0 \( {\begin{Vmatrix}{\omega }_{k}\end{Vmatrix}}^{\prime \prime }/{J}_{max}^{o} \) -1 0 0.5 1 time(s) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_12.jpg?x=302&y=154&w=1105&h=1410&r=0"/>

Fig. 8. (a) The five-axis trajectory. (b) The kinematic saturation of the TCP and TOV motion. (c) The actual and estimated kinematic saturation of the joint motion.

<!-- Meanless: 13-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- figureText: (a) \( {\widetilde{q}}_{\text{diff },1} \) \( {\widetilde{q}}_{\text{diff },2} \) \( {\widetilde{q}}_{\text{diff },3} \) \( {\widetilde{q}}_{\text{diff },4} \) \( {\widetilde{q}}_{\text{diff },5} \) \( {\widetilde{q}}_{\mathrm{{nl}},2} \) \( {\widetilde{q}}_{\mathrm{{nl}},3} \) \( {\widetilde{q}}_{\mathrm{{nl}},4} \) \( {\widetilde{q}}_{\mathrm{{nl}},5} \) 0.6 0.8 1 0.6 0.8 0.6 0.8 1 time(s) \( {\widetilde{q}}_{\mathrm{{nl}},1} \) \( {\widetilde{q}}^{\prime }/{V}_{max}^{\mu }\left( \% \right) \) 1.5 -1.5 -3 0 0.2 0.4 (b) \( {\widetilde{q}}^{\prime \prime }/{A}_{max}^{\mu }\left( \% \right) \) 1.5 0 -1.5 -3 0 0.2 0.4 (c) \( {\widetilde{q}}^{\prime \prime \prime }/{J}_{max}^{\mu }\left( \% \right) \) 1.5 -1.5 -3 0 0.2 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_13.jpg?x=299&y=154&w=1105&h=1188&r=0"/>

Fig. 9. The joint kinematic estimation. The (a) velocity, (b) acceleration and (c) jerk error.

<!-- figureText: 0.4 \( \times  {10}^{-3} \) 1.5 actual estimate \( {\varepsilon }_{1}^{o}\left( {rad}\right) \) 1 0.5 0 0 0.1 0.2 \( {T}_{1,c}\left( s\right) \) actual 0.3 estimate \( {\varepsilon }_{1}^{p}\left( {mm}\right) \) 0.2 0.1 0 0.1 0.2 \( {T}_{1,c}\left( s\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_13.jpg?x=304&y=1440&w=1097&h=505&r=0"/>

Fig. 10. The geometric model accuracy.

<!-- Meanless: 14-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- figureText: (a) TOV path Zoomed view Linear path Blending path -0.14 -0.141 -0.336 -0.334 -0.332 \( {o}_{x} \) 20 feedrate (mm/s) 15 10 -97 -99 -134 -132 \( {p}_{y}\left( \mathrm{{mm}}\right) \) \( {O}_{y} \) \( {O}_{2} \) (d) \( {q}_{1} \) \( {q}_{4} = {q}_{5} \) blending area blending area 0.5 \( {q}_{\mu }^{\prime }/{V}_{max}^{\mu } \) 0 -0.5 - 1 0.5 0.5 1 0.5 \( {q}_{\mu }^{\prime \prime }/{A}_{max}^{\mu } \) 0 -0.5 0.5 1 0.5 0.5 \( {q}_{\mu }^{\prime \prime \prime }/{J}_{max}^{\mu } \) 0 -0.5 -1 0.5 0 0.5 time(s) time(s) -0.14 -0.15 -0.35 -0.34 -0.33 -0.32 \( {o}_{x} \) \( {p}_{z}\left( \mathrm{{mm}}\right) \) 15 14 -142 -140 -138 -136 \( {p}_{x}\left( \mathrm{{mm}}\right) \) (b) \( {p}_{x} \) \( {p}_{y} \) (c) blending area 0.5 0.5 \( {p}_{\mu }^{\prime }/{V}_{max}^{p} \) 0 \( {o}_{\mu }^{\prime }/{V}_{max}^{o} \) -0.5 -0.5 -1 0.5 1 0 0.5 0.5 \( {p}_{\mu }^{\prime \prime }/{A}_{max}^{p} \) 0 \( {o}_{\mu }^{\prime \prime }/{A}_{max}^{o} \) -0.5 -0.5 -1 0 0.5 1 0 0.5 0.5 \( {p}_{\mu }^{\prime \prime \prime }/{J}_{max}^{p} \) 0 \( {o}_{\mu }^{\prime \prime \prime }/{J}_{max}^{o} \) -0.5 -0.5 -1 0.5 1 0 time(s) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_14.jpg?x=189&y=147&w=1332&h=1839&r=0"/>

Fig. 11. The non-stop toolpath example. (a) The feedrate scheduled toolpath and the blended TOV path. The kinematic saturation of (b) TCP, (c) TOV, and (d) joint motion.

<!-- Meanless: 15-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- figureText: TOV path TCP path feed direction (b) 0.1 0 -0.1 -0.2 0 0.2 \( {p}_{z}\left( \mathrm{{mm}}\right) \) 205 195 -40 20 -20 0 0 \( {p}_{x}\left( \mathrm{{mm}}\right) \) 20 -20 \( {p}_{y}\left( \mathrm{{mm}}\right) \) (a) -0.05 100 -0.1 -0.15 -0.3 -0.2 -0.1 \( {o}_{x} \) \( {p}_{z}\left( \mathrm{{mm}}\right) \) \( {p}_{x}\left( \mathrm{{mm}}\right) \) -140 -130 -200 -180 -140 -120 -100 \( {p}_{y}\left( \mathrm{{mm}}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_15.jpg?x=246&y=151&w=1208&h=647&r=0"/>

Fig. 12. Two typical five-axis toolpaths: (a) Case 1, and (b) Case 2.

Table 3

The comparison results.

<table><tr><td rowspan="2">Cases</td><td rowspan="2">Methods</td><td colspan="2">Calculation Speed</td><td colspan="3">Machining Trajectory Result</td></tr><tr><td>Optimization ( \( \mu \mathrm{s}/ \) point)</td><td>Interpolation (μ s/point)</td><td>Machining Time (s)</td><td>Blending Error \( \left( {\mu \mathrm{m}}\right) \)</td><td>Kinematic Saturation</td></tr><tr><td rowspan="6">Case 1 29 CLs</td><td>P2P</td><td>3.43</td><td>0.805</td><td>14.155</td><td>-</td><td>102.7 %</td></tr><tr><td>Ref. [26]</td><td>0.119</td><td>0.589</td><td>8.856</td><td>43.8</td><td>132.7 %</td></tr><tr><td>Ref. [27]</td><td>1.41</td><td>1.17</td><td>9.873</td><td>45.1</td><td>189.2 %</td></tr><tr><td>Ref. [11]</td><td>530</td><td>2390</td><td>10.228</td><td>46.8</td><td>102.6 %</td></tr><tr><td>Ref. [15]</td><td>855</td><td>3313</td><td>8.485</td><td>23.5</td><td>100.8 %</td></tr><tr><td>Proposed</td><td>6.42</td><td>1.02</td><td>8.173</td><td>48.1</td><td>101.2 %</td></tr><tr><td rowspan="6">Case 2 100 CLs</td><td>P2P</td><td>3.15</td><td>0.791</td><td>52.353</td><td>-</td><td>101.3 %</td></tr><tr><td>Ref. [26]</td><td>0.115</td><td>0.522</td><td>25.308</td><td>48.9</td><td>189.1 %</td></tr><tr><td>Ref. [27]</td><td>1.10</td><td>1.25</td><td>31.543</td><td>46.1</td><td>191.6 %</td></tr><tr><td>Ref. [11]</td><td>219</td><td>2763</td><td>28.209</td><td>20.2</td><td>100.7 %</td></tr><tr><td>Ref. [15]</td><td>325</td><td>4231</td><td>27.668</td><td>35.0</td><td>100.0 %</td></tr><tr><td>Proposed</td><td>6.04</td><td>1.03</td><td>28.465</td><td>42.0</td><td>101.1 %</td></tr></table>

<!-- Meanless: 16-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- figureText: Ref. [11] - - - Ref. [15] Proposed (b) 0.07 0.06 0.05 \( \begin{Vmatrix}{\mathbf{\omega }}_{k}\end{Vmatrix}\left( {\mathrm{{rad}}/\mathrm{s}}\right) \) 0.04 0.03 0.02 0.01 0 0.5 1 normalized arc length parameter (a) 25 20 \( \begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}\left( {\mathrm{{mm}}/\mathrm{s}}\right) \) 15 10 5 0 0 0.5 normalized arc length parameter -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_16.jpg?x=306&y=156&w=1089&h=690&r=0"/>

Fig. 13. The feedrate in Case 1. (a) The feedrate of the TCP and (b) TOV.

<!-- Media -->

\( \min t \)

\[\text{s.t.}x \geq  \log \left( {T}_{k,1}\right) \text{,}\]

\[y \geq  \log \left( {T}_{k + 1,1}\right) \]

\[\log \left\lbrack  {\exp \left( x\right)  + \exp \left( y\right) }\right\rbrack   \leq  t \tag{32}\]

\[\log \left\lbrack  {\frac{{h}_{\mu ,k}}{{T}_{k,2}}\exp \left( {-x}\right)  + \frac{{h}_{\mu ,k + 1}}{{T}_{k + 1,2}}\exp \left( {-y}\right) }\right\rbrack   \leq  \log {A}_{\max }^{\mu }\]

\[\log \left\lbrack  {\frac{{h}_{\mu ,k}}{{T}_{k,2}{T}_{k,3}}\exp \left( {-x}\right)  + \frac{{h}_{\mu ,k + 1}}{{T}_{k + 1,2}{T}_{k + 1,3}}\exp \left( {-y}\right) }\right\rbrack   \leq  \log {J}_{\max }^{\mu }\]

The log-sum-log constraints are represented using the following sets of exponential cones,

\[\left\{  \begin{matrix} {u}_{1} + {u}_{2} = 1, \\  \left( {{u}_{1},1,x - t}\right) ,\left( {{u}_{2},1,y - t}\right)  \in  {K}_{\exp } \times  {K}_{\exp }, \end{matrix}\right. \]

\[\left\{  \begin{matrix} \frac{{h}_{\mu ,k}}{{T}_{k,2}}{v}_{1} + \frac{{h}_{\mu ,k + 1}}{{T}_{k + 1,2}}{v}_{2} = {A}_{\max }^{\mu }, \\  \left( {{v}_{1},1, - x}\right) ,\left( {{v}_{2},1, - y}\right)  \in  {K}_{\exp } \times  {K}_{\exp }, \end{matrix}\right.  \tag{33}\]

\[\left\{  \begin{matrix} \frac{{h}_{\mu ,k}}{{T}_{k,2}{T}_{k,3}}{w}_{1} + \frac{{h}_{\mu ,k + 1}}{{T}_{k + 1,2}{T}_{k + 1,3}}{w}_{2} = {J}_{\max }^{\mu }, \\  \left( {{w}_{1},1, - x}\right) ,\left( {{w}_{2},1, - y}\right)  \in  {K}_{\exp } \times  {K}_{\exp }. \end{matrix}\right. \]

#### 3.3.The blending process

In this paper, an approach is proposed for smoothly blending linear segments while ensuring both geometric and kinematic constraints. The process, illustrated in Fig. 2, involves calculating the delay time using a geometric error model, followed by evaluating the kinematic constraints with a prediction model prior to blending. In case of any violation, the velocity constraint is maintained by adding delay time, while the acceleration and jerk constraints are addressed through added feed duration by resolving another GP problem. This section highlights the significance of the blending process.

In terms of the geometric error model, it is common practice [25,26] to use the symmetric blending path assumption to approximate the error and calculate the delay time analytically. However, the blending path are only symmetric when the feeds are identical. This is rarely the case as the lengths of the linear segments and the feedrates often vary. This leads to an overestimation of geometric error, which limits time-optimality. To overcome this limitation, this paper proposes a more general model that provides an analytical determination of geometric errors from delay time. While this makes deriving a close-form for the delay time more complex, a bisection method is utilized to efficiently find the desired delay time. The simulations in Section 4 demonstrate that the proposed method's

<!-- Meanless: 17-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: Ref. [11] Ref. [15] Proposed 0.5 0.6 0.7 0.8 0.9 1 0.5 0.6 0.7 0.8 0.9 1 0.5 0.6 0.7 0.8 0.9 1 0.5 0.6 0.7 0.8 0.9 1 0.5 0.6 0.7 0.8 0.9 1 normalized arc length parameter 1 \( {q}_{1}^{\prime \prime \prime }/{J}_{max}^{1} \) 0 -1 0 0.1 0.2 0.3 0.4 1 \( {q}_{2}^{\prime \prime \prime }/{J}_{ma}^{2} \) 0 -1 0 0.1 0.2 0.3 0.4 1 \( {q}_{3}^{\prime \prime \prime }/{J}_{max}^{3} \) 0 -1 0 0.1 0.2 0.3 0.4 1 \( {q}_{4}^{\prime \prime \prime }/{J}_{ma}^{4} \) 0 -1 0 0.1 0.2 0.3 0.4 1 \( {q}_{5}^{\prime \prime \prime }/{J}_{max}^{5} \) 0 -1 0 0.1 0.2 0.3 0.4 -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_17.jpg?x=188&y=152&w=1331&h=1431&r=0"/>

Fig. 14. The normalized jerk of the joints.

<!-- Media -->

calculation time is adequately efficient with improved accuracy.

The current method [27] in addressing kinematic violation involves fixing velocity and acceleration by adjusting delay time, while tolerating jerk violation. In the proposed method, by adding feed duration instead of delay time, both acceleration and jerk issues can be fixed. This approach also results in less velocity fluctuation [26], thereby improving the quality of the finished surface in the machining process. Moreover, in most situations, the added feed duration can be minimized by distributing it between adjacent feeds, giving priority to the feed with a shorter duration to reduce the overall kinematics.

Our approach follows a step-by-step process that preserves previous constraints in each step. We address two possible concerns in this approach. Firstly, in Section 3.2, the feed durations are adjusted after determining the time constants in Section 2.2. To preserve the constraints, the new feed durations are constrained to be greater than the previously determined values. Secondly, in Section 3.2, the feeds are adjusted after identifying geometric errors in Section 3.1. This adjustment either prolongs the delay time or decreases feedrates by extending the feed duration. The geometric errors can only be reduced after the adjustment. We have verified that all constraints are respected during the blending process with examples in Section 4.

<!-- Meanless: 18-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: (a) Ref.[11] \( {\mathrm{{CL}}}_{27} \) 20 Proposed P2P CL Ref.[11] Ref.[15] (d) \( {\mathrm{{CL}}}_{3} \) 0.025 0.015 \( {p}_{x}\left( {mm}\right) \) -129.98 -129.96 -212.46 -212.44 -212.42 -212.4 \( {p}_{y}\left( {mm}\right) \) (e) \( {\mathrm{{CL}}}_{12} \) 1.815 1.805 feedrate (mm/s) \( {p}_{x}\left( {mm}\right) \) -142.18 -142.16 -164.2 -164.22 -164.28 -164.26 -164.24 \( {p}_{y}\left( {mm}\right) \) (f) \( {\mathrm{{CL}}}_{27} \) 12.675 12.665 \( {p}_{x}\left( {mm}\right) \) -140.79 -140.77 -103.36 -103.34 -103.32 -103.3 -103.28 \( {p}_{y}\left( {mm}\right) \) \( {\mathrm{{CL}}}_{12} \) 18 \( {p}_{x}\left( \mathrm{{mm}}\right) \) -140 16 -130 -100 -140 -120 -200 -180 -160 14 \( {p}_{y}\left( \mathrm{{mm}}\right) \) (b) Ref.[15] \( \downarrow  {\mathrm{{CL}}}_{27} \) \( {\mathrm{{CL}}}_{12} \) \( {\mathrm{{CL}}}_{3} \) \( {p}_{x}\left( \mathrm{{mm}}\right) \) -140 -130 -100 -160 -140 -120 -200 -180 \( {p}_{y}\left( \mathrm{{mm}}\right) \) (c) Proposed \( {\mathrm{{CL}}}_{27} \) \( {\mathrm{{CL}}}_{12} \) 4 \( {p}_{x}\left( \mathrm{{mm}}\right) \) \( {\mathrm{{CL}}}_{3} \) -140 2 -130 -100 -160 -140 -120 -200 -180 \( {p}_{y}\left( \mathrm{{mm}}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_18.jpg?x=165&y=152&w=1372&h=765&r=0"/>

Fig. 15. (a)-(c) The feedrate scheduled toolpaths. (d)-(f) The sampled TCP path.

<!-- Media -->

## 4. Simulations and experiments

The hybrid robot shown in Fig. 1(b) is used for simulations and experiments. Section 4.1 showcases the effectiveness of the proposed method, while Section 4.2 compares it with other interpolation methods. In Section 4.3, a flank milling finishing operation is conducted.

### 4.1. Verification of the proposed method

In this section, the proposed method is verified through a three-CL example to demonstrate the accuracy of each step. The delay time is chosen to guarantee zero geometric error at the corner. Kinematic constraints are listed in Table. 1, and CLs are given in Table. 2. The kinematic constraints are stricter in the experiment due to safety concerns during machining. Time constants are optimized as follows: \( {T}_{1,1} = {234}\mathrm{\;{ms}},{T}_{1,2} = {184}\mathrm{\;{ms}},{T}_{1,3} = {50}\mathrm{\;{ms}},{T}_{2,1} = {421}\mathrm{\;{ms}},{T}_{2,2} = {183}\mathrm{\;{ms}} \) and \( {T}_{2,3} = {50}\mathrm{\;{ms}} \) . The trajectory is shown in Fig. 8 (a).

In the first segment,the second and third time constants \( {T}_{1,2} \) and \( {T}_{1,3} \) are restricted by the kinematic constraints on the motions of TOV and axis \( 2\left( {q}_{2}\right) \) ,while the feed duration \( {T}_{1,1} \) is restricted by Eq. (5). In the second segment,the time constants \( {\left\{  {T}_{2,i}\right\}  }_{i = 1}^{3} \) are restricted by the constraints on the motions of TCP and axis \( 2\left( {q}_{2}\right) \) . To determine the extent to which the kinematic values such as velocity, acceleration and jerk on all axes, including the MCS and WCS, exceed the tolerance values set by user, we introduce the concept of kinematic saturation, which is the ratio of the actual kinematic value to the tolerance value. When the constraints are well-respected,the kinematic saturation reaches \( {100}\% \) . The effectiveness of kinematic constraints can be verified by the maximum of the kinematic saturations across multiple axes. E.g.,the overall kinematic saturation \( \gamma \) across all axes is calculated as,

\[\gamma  = \mathop{\max }\limits_{\substack{{k = 1,\ldots ,N - 1} \\  {\mu  = 1,\ldots ,5} }}\left( {\frac{\begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}}{{V}_{\max }^{p}},\frac{{\begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}}^{\prime }}{{A}_{\max }^{p}},\frac{{\begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}}^{\prime }}{{J}_{\max }^{p}},\frac{\begin{Vmatrix}{\mathbf{\omega }}_{k}\end{Vmatrix}}{{V}_{\max }^{o}},\frac{{\begin{Vmatrix}{\mathbf{\omega }}_{k}\end{Vmatrix}}^{\prime }}{{A}_{\max }^{o}},\frac{{\begin{Vmatrix}{\mathbf{\omega }}_{k}\end{Vmatrix}}^{\prime }}{{J}_{\max }^{o}},\frac{\left| {q}_{\mu }^{\prime }\right| }{{V}_{\max }^{\mu }},\frac{\left| {q}_{\mu }^{\prime }\right| }{{A}_{\max }^{\mu }},\frac{\left| {q}_{\mu }^{\prime \prime }\right| }{{J}_{\max }^{\mu }}}\right)  \tag{34}\]

Here,the overall kinematic saturation is \( {102.0}\% \) ,which is caused by the jerk of the axis \( 2\left( {q}_{2}\right) \) at 1121 ms. Fig. 8(b) illustrates the kinematic saturation of each axis,including the TCP and TOV’s velocity \( \left( {\begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}/{V}_{\max }^{p},\begin{Vmatrix}{\mathbf{\omega }}_{k}\end{Vmatrix}/{V}_{\max }^{p}}\right) \) ,acceleration \( \left( {{\begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}}^{\prime }/{A}_{\max }^{p},{\begin{Vmatrix}{\mathbf{\omega }}_{k}\end{Vmatrix}}^{\prime }/}\right. \) \( {A}_{\max }^{p} \) ) and jerk \( \left( {{\begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}}^{\prime }/{J}_{\max }^{p},{\begin{Vmatrix}{\mathbf{\omega }}_{k}\end{Vmatrix}}^{\prime \prime }/{J}_{\max }^{p}}\right) \) . Fig. 8(c) presents the actual kinematic saturation of the joint motion \( \left( {{q}_{\mu }^{\prime }/{V}_{\max }^{p},{q}_{\mu }^{\prime \prime }/{A}_{\max }^{\mu },{q}_{\mu }^{\prime \prime }}\right. \) \( \left. {J}_{\max }^{\mu }\right) \) and the estimated \( \left( {{\widehat{q}}_{\mu }^{\prime }/{V}_{\max }^{\mu },{\widehat{q}}_{\mu }^{\prime }/{A}_{\max }^{\mu },{\widehat{q}}_{\mu }^{\prime }/{J}_{\max }^{\mu }}\right) \) . Consequently,the proposed interpolation method can generate a time-optimal trajectory adhering to all kinematic constraints.

In Fig. 8(b), it can be seen that the TCP and TOV motions are synchronized and limited by the optimized time constants. On the other hand, Fig. 8(c) compares the predicted and actual joint kinematic constraints. The predicted joint trajectories are approximated with Eq. (12), while the actual joint trajectories are obtained through inverse kinematics and numerical differentiation.

In Fig. 9, the estimation errors are shown to be less than 3 % when compared to the actual value. Fig. 9(a) indicates that the

<!-- Meanless: 19-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: (a) 0.5 \( {T}_{k,1} \) \( {T}_{k,2} \) \( {T}_{k,3} \) Index = 54 Index \( = {84} \) 80 99 segments index TOV velocity TCP velocity TOV acceleration TCP acceleration TOV jerk TCP jerk Index = 54 Index = 84 Proposed Proposed kinematic limit Ref.[27] Ref.[27] Ref.[26] Ref.[26] 2 0 2 saturation saturation 0.4 time constants(s) 0.3 0.2 0.1 Index = 9 Index = 27 1 40 (b) joint velocity joint acceleration joint jerk Index = 9 Index = 27 Proposed kinematic limit Proposed kinematic limit Ref.[27] Ref.[27] Ref.[26] Ref.[26] 0 2 1 saturation saturation -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_19.jpg?x=299&y=155&w=1099&h=1204&r=0"/>

Fig. 16. (a)The time constants. (b) The kinematic saturation of the TCP, TOV and joint motion at four linear segments before blending.

<!-- Media -->

error is caused by the differential error \( {\widetilde{q}}_{\text{diff }} = {q}_{s}\dot{s} - {\widehat{q}}_{\mu } \) . Meanwhile,the acceleration and jerk errors are a combination of both the differential error \( {\widetilde{q}}_{\text{diff }}^{\prime \prime } = {q}_{s}{s}^{\prime } - {\widehat{q}}_{\mu }^{\prime },{\widetilde{q}}_{\text{diff }}^{\prime \prime } = {q}_{s}{s}^{\prime \prime \prime } - {\widehat{q}}_{\mu }^{''} \) and nonlinear error \( {\widetilde{q}}_{nl}^{\prime } = {q}_{ss}{\left( s\right) }^{2},{\widetilde{q}}_{nl}^{\prime \prime } = 3{q}_{ss}s{s}^{\prime } + {q}_{sss}{\left( s\right) }^{3} \) ,as illustrated in Fig. 9(b) and (c). The calculation of arc length \( s\left( n\right) \) involves,

\[s\left( {n + 1}\right)  = s\left( n\right)  + \begin{Vmatrix}{{\mathbf{v}}_{k}\left( n\right) }\end{Vmatrix}{T}_{s},s\left( 0\right)  = 0\mathrm{\;{mm}}. \tag{35}\]

Through the method of two-point differentiation,we acquire \( {q}_{s} \) .

\[{q}_{s}\left( n\right)  = \frac{{q}_{\mu }\left( {n + 1}\right)  - {q}_{\mu }\left( n\right) }{s\left( {n + 1}\right)  - s\left( n\right) }. \tag{36}\]

When \( s\left( {n + 1}\right)  - s\left( n\right)  < 1\mathrm{e} - 6\mathrm{\;{mm}},{q}_{s}\left( n\right) \) is assigned as not a number (NaN). \( {q}_{ss} \) and \( {q}_{sss} \) are calculated similarly. Here,the index “ \( \mathrm{n} \) ” represents the interpolation time.

To verify the intersection cases,we adjust the delay time \( \left( {{T}_{1,c} \in  \left\lbrack  {0,{T}_{1,d}}\right\rbrack  }\right) \) and show the precision of the geometric model in Fig. 10. By selecting an appropriate delay time, we can accommodate different levels of error tolerance. E.g., the P2P motion is smoothed under the geometric constraints of \( {0.3}\mathrm{\;{mm}} \) (or rad),as seen in Fig. 11(a). A delay time of only \( 7\mathrm{\;{ms}} \) is determined,which is significantly less than the 234 ms required in the P2P scenario. The estimated blending error \( \left( {{\widehat{\varepsilon }}_{1}^{p} = {0.299}\mathrm{\;{mm}}}\right. \) and \( {\widehat{\varepsilon }}_{1}^{o} = {1.22}\mathrm{e} - 3 \) rad) is kept below the user-defined value using the bisection method.

Additionally,the velocity is estimated to be within acceptable limits,since \( {T}_{1,c} > {T}_{1,d} - {T}_{2,d}.{T}_{1,c} \) is fully determined based on these estimations,but the acceleration and jerk of axis 5 is expected to exceed the maximum limit \( \left( {{100}\mathrm{\;{mm}}/{\mathrm{s}}^{2}}\right. \) and \( {2000}\mathrm{\;{mm}}/{\mathrm{s}}^{3} \) respectively),with an estimated value of \( {118.5}\mathrm{\;{mm}}/{\mathrm{s}}^{2} \) and \( {2371}\mathrm{\;{mm}}/{\mathrm{s}}^{3} \) . To resolve this,the feed duration is extended to \( {T}_{1,1}^{ * } = {0.309}\mathrm{\;{ms}} \) , \( {T}_{2,1}^{ * } = {0.460}\mathrm{\;{ms}} \) . After blending,the trajectory time improves to 1.009 s from 1.122 s in P2P interpolation. Based on these predictions and corrective measures, the kinematics are fully saturated without any violations during the blending process as shown in Fig. 11(b)- (d). The kinematic saturation of TCP,TOV,and joint motion is calculated as \( {p}_{\mu }^{\prime }/{V}_{\max }^{p},{p}_{\mu }^{\prime }/{A}_{\max }^{p},{p}_{\mu }^{\prime \prime }/{J}_{\max }^{p},{o}_{\mu }^{\prime }/{V}_{\max }^{o},{o}_{\mu }^{\prime }/{A}_{\max }^{o},{o}_{\mu }^{\prime \prime }/{J}_{\max }^{o},\mu  = \) \( x,y,z \) and \( {q}_{\mu }^{\prime }/{V}_{\max }^{\mu },{q}_{\mu }^{\prime }/{A}_{\max }^{\mu },{q}_{\mu }^{\prime }/{J}_{\max }^{\mu },\mu  = 1,\cdots ,5 \) ,respectively. The geometric errors \( \left( {{\varepsilon }_{1}^{p} = {0.248}\mathrm{\;{mm}}}\right. \) and \( {\varepsilon }_{1}^{o} = {9.86}\mathrm{e} - 4 \) rad) have decreased compared to the previous estimate due to the feed extensions, and are still within the user-defined limit. The overall kinematic saturation is \( {100.4}\% \) ,which is caused by the jerk of the axis 5 at \( {358}\mathrm{\;{ms}} \) during blending.

<!-- Meanless: 20-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: Ref. [26] Ref. [27] Proposed (b) 1 0.8 \( {\widehat{\varepsilon }}_{k}^{p}/{\widehat{\varepsilon }}_{k}^{p} \) 0.6 0.4 0.2 0 0 20 40 60 80 100 (d) 1 0.8 \( {\varepsilon }_{k}^{o}/{\widehat{\varepsilon }}_{k}^{o} \) 0.6 0.4 0.2 0 20 40 60 80 100 corner index \( {T}_{k,c} \) \( - {T}_{k,d} \) 50 60 70 80 90 100 corner index 0.012 0.01 \( {\varepsilon }_{k}^{p}\left( {mm}\right) \) 0.008 0.006 0.004 0.002 0 0 20 40 60 80 100 \( \times  {10}^{-3} \) (c) 1.5 \( {\varepsilon }_{k}^{o}\left( {rad}\right) \) 1 0.5 0 20 40 60 80 100 corner index (e) 0.3 delay time(s) 0.2 0.1 0 0 10 20 30 40 -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_20.jpg?x=251&y=154&w=1199&h=1225&r=0"/>

Fig. 17. The geometric errors of (a) position and (c) orientation. The estimation accuracy of (b) position and (d) orientation. (e) The delay time of the proposed method.

<!-- Media -->

### 4.2. Comparisons with other interpolation method

To test the effectiveness of the proposed method, two representative five-axis toolpaths (as shown in Fig. 12) are selected from existing literature. The first case,taken from Ref. [8], consists of 29 CLs with large gaps and a region of high curvature, while the second case involves engraving operation on a sphere with zero side-tilt and 18-degree lead angles.

The kinematic constraints are listed in Table. 1,and the geometric error tolerance is set to \( {0.05}\mathrm{\;{mm}} \) (or rad). The proposed method is compared with two spline-based methods [11,15] and two filter-based methods [26,27]. The results are shown in Table. 3.

The simulations are coded using MATLAB with an i5-13600 K CPU @ 3.50GHz. To assess the real-time performance of trajectory generation methods [33], we measure the calculation speed by taking the average calculation time across the number of calculated interpolation points. A real-time method calculates faster than the controller frequency. We record the calculation time by averaging 100 repetitive calculations. The number of calculated interpolation points is determined by the interpolation interval of \( 1\mathrm{\;{ms}} \) from the machining trajectory result. The optimization time refers to the time taken to determine the feedrate. This includes calculating filter constants and delayed time for filter-based methods, alongside smoothing corners and scheduling feedrate with jerk limitation [21] for spline-based methods. The interpolation time refers to the time taken to determine TCP and TOV trajectories at the interpolation intervals given the feedrate. Therefore, the interpolation includes an additional kinematic transformation for MCS-based methods such as Ref. [15] and Ref. [27]. We calculate the maximum blending error as \( \mathop{\max }\limits_{{k = 1,\ldots ,N - 2}}\left( {\varepsilon }_{k}^{p}\right) \) and the overall kinematic saturation with Eq.

<!-- Meanless: 21-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: Ref. [27] Ref. [26] Proposed P2P (d) calculation speed (ms/point) \( {10}^{-2} \) \( {10}^{-3} \) 0.4 0.08 0.016 0.0032 blending error tolerance \( \left( {\mathrm{{mm}}/\mathrm{{rad}}}\right) \) (e) 60 machining time (s) 50 40 HIMA 30 20 0.4 0.08 0.016 0.0032 blending error tolerance (mm/rad) (f) 8 kinematic saturation 6 4 2 0.4 0.08 0.016 0.0032 blending error tolerance \( \left( {\mathrm{{mm}}/\mathrm{{rad}}}\right) \) (a) calculation speed (ms/point) \( {10}^{-2} \) \( {10}^{-3} \) 0.4 0.08 0.016 0.0032 blending error tolerance (mm/rad) (b) 18 16 machining time (s) 14 12 10 8 6 0.4 0.08 0.016 0.0032 blending error tolerance (mm/rad) (c) 8 kinematic saturation 6 2 0 0.4 0.08 0.016 0.0032 blending error tolerance (mm/rad) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_21.jpg?x=169&y=154&w=1375&h=1510&r=0"/>

Fig. 18. Results with random orientations under four blending error tolerances, 0.4, 0.08, 0.016, and 0.0032 mm for TCP (rad for TOV). (a)-(c) Case 1. (d)-(f) Case 2.

<!-- Media -->

(34). The derivatives of the spline-based methods are estimated using the difference form from Ref. [21] to ensure numerical accuracy. Based on Table. 3, it is clear that the proposed method simultaneously satisfies the blending error tolerances and kinematic limits in both cases. In comparison to the two spline-based methods, the proposed method offers two advantages. Firstly, it significantly reduces the calculation time by optimizing the time constants through convex optimization, allowing for an efficient interpolation of the final trajectory in one step. These optimized time constants guarantee the kinematic and geometric constraints. In contrast, the spline-based methods require three steps for calculation, including spline construction, feedrate scheduling, and interpolation with the scheduled feedrate. The second part of this process involves jerk constraints in TOTP problems, which is complex as the jerk constraints are difficult to represent on the phase plane [19]. Additionally, interpolating multiple spline segments in the third part is time-consuming [25].

<!-- Meanless: 22-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: (a) (b) (c) Fixed frame PKM Work table Guideways -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_22.jpg?x=339&y=158&w=1023&h=600&r=0"/>

Fig. 19. (a) The machining robot. (b) The machining setup. (c) The near net shape stock.

Table 4

The machining parameters.

<table><tr><td>Conditions</td><td>Parameter</td></tr><tr><td>Spindle speed</td><td>8000 rpm</td></tr><tr><td>Depth of cut</td><td>5 mm</td></tr><tr><td>Width of cut</td><td>0.1 mm</td></tr><tr><td>Workpiece material</td><td>Aluminum 6061</td></tr><tr><td>Tool</td><td>D3 flat-end mill (3 teeth)</td></tr><tr><td>Maximum feedrate</td><td>4 mm/s</td></tr></table>

<!-- figureText: TOV path TCP path feed direction 100 80 60 20 40 20 0 \( {p}_{x}\left( \mathrm{{mm}}\right) \) -0.1 feed \( {O}_{x} \) direction \( {p}_{z}\left( \mathrm{{mm}}\right) \) 40 0 0.1 -0.2 0 0.2 \( {p}_{y}\left( \mathrm{{mm}}\right) \) \( {o}_{y} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_22.jpg?x=172&y=1210&w=1365&h=440&r=0"/>

Fig. 20. The CAM-generated flank milling toolpath.

<!-- Media -->

Secondly, despite the limited time for calculations, the proposed method exhibits time-optimality similar to that of the spline-based methods. The geometric path determines the optimal time for the TOTP problem. However, spline-based methods build the splines before optimizing the trajectory time, leading to a suboptimal geometric path. Synchronizing the TCP and TOV motions to reduce trajectory time is crucial, but achieving this during the spline-building stage is challenging due to the nonlinear kinematics of hybrid robots. In contrast, filter-based methods determine the toolpath and the trajectory time simultaneously. As a result, the proposed method generates a smoother geometric path under the user-defined kinematic constraints compared to spline-based methods.

Fig. 13 demonstrates this time-optimality by displaying the feedrate for Case 1. The feedrate of the two splined-based methods is limited at several corners, resulting in unsmooth transitions under kinematic constraints. At these corners, a limiting factor on the feedrate is the joint jerk shown in Fig. 14. Ref. [11] smooths the toolpaths in the WCS, leading to an unsmooth joint path at the linear segment. Ref. [15] improves the trajectory efficiency by smoothing the joint path, but the feedrate is still limited at tight transitions in the WCS at several corners. This is depicted in Fig. 15, where the feedrate is shown along the geometric toolpath. We recommend using the proposed filter-based approach for real-time trajectory generation rather than our previous method outlined in Ref. [15]. However, Ref. [15] is still effective for reducing machining time, especially in situations where computation time is not a concern, as shown in Case 2 of Table 3.

<!-- Meanless: 23-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: (a) feedrate (mm/s) (c) 0.5 1.5 2 2.5 3 3.5 \( {p}_{z}\left( \mathrm{{mm}}\right) \) Ref. [11] 100 60 40 \( {p}_{x}\left( \mathrm{{mm}}\right) \) \( {p}_{y}\left( \overset{20}{\mathrm{{mm}}}\right) \) 20 Proposed \( {p}_{z}\left( \mathrm{{mm}}\right) \) 100 40 \( {p}_{x}\left( \mathrm{{mm}}\right) \) 60 \( {p}_{y}\left( \mathrm{{mm}}\right) \) 20 angular feedrate (rad/s) (d) 0.01 0.02 0.03 0.04 0.05 0.06 -0 . Ref. [11] 0. -0.2 -0.1 0.1 0.2 \( {o}_{y} \) Proposed -0.1 60 -0.2 -0.1 0.1 0.2 \( {o}_{y} \) Ref.[11] Proposed 5 feedrate(mm/s) 3 2 1 0 10 20 30 40 50 time(s) (b) Ref.[11] Proposed 0.07 0.06 angular feedrate(rad/s) 0.05 0.04 0.03 0.02 0.01 0 10 20 30 40 50 time(s) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_23.jpg?x=305&y=154&w=1102&h=1139&r=0"/>

Fig. 21. (a) The feedrate and (b) the angular feedrate. (c) The TCP and (d) TOV trajectories.

<!-- figureText: \( {\delta }_{5}\left( \mathrm{{mm}}\right) {\delta }_{4}\left( \mathrm{{mm}}\right) {\delta }_{3}\left( \mathrm{{mm}}\right) {\delta }_{2}\left( \mathrm{{mm}}\right) {\delta }_{1}\left( \mathrm{{mm}}\right) \) 0.05 Ref. [11] Proposed Increasing 25 30 35 40 45 50 55 time(s) 0 -0.05 0.05 0 -0.05 0.05 0 -0.05 0.05 -0.05 0.05 -0.05 0 5 10 15 20 -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_23.jpg?x=190&y=1392&w=1316&h=656&r=0"/>

Fig. 22. The joint tracking errors of the proposed method and Ref. [11].

<!-- Meanless: 24-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- figureText: (a) Zoomed area (b) actual jerk desired jerk 750 -750 40 50 20 21 22 750 -750 40 50 20 21 22 750 -750 40 50 20 21 22 750 -750 40 50 20 21 22 750 -750 40 50 20 21 22 time(s) \( {q}_{1}^{\prime \prime \prime }\left( {{mm}/{s}^{3}}\right) \) 750 -750 0 10 20 30 \( {q}_{2}^{\prime \prime \prime }\left( {{mm}/{s}^{3}}\right) \) 750 -750 0 10 20 30 \( {q}_{3}^{\prime \prime \prime }\left( {{mm}/{s}^{3}}\right) \) 750 -750 0 10 20 30 \( {q}_{4}^{\prime \prime \prime }\left( {{mm}/{s}^{3}}\right) \) 750 -750 0 10 20 30 \( {q}_{5}^{\prime \prime \prime }\left( {{mm}/{s}^{3}}\right) \) 750 -750 0 10 20 30 time(s) -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_24.jpg?x=217&y=151&w=1269&h=1270&r=0"/>

Fig. 23. (a) The actual and desired jerk of the flank milling operation. (b) The zoomed area.

<!-- Media -->

In comparison to other filter-based methods, the proposed method presents two additional benefits: kinematic saturation and geometric error constraining. The former addresses the issue of hybrid robots' kinematic characteristics not being properly considered by the other filter-based methods, which either ignore (as in Ref. [26]) or simplify (as in Ref. [27]) joint constraints. While this results in satisfactory joint tracking performance for machine tools with orthogonal configurations, it is insufficient for hybrid robots, as the tool motion is affected by multiple chains.

We demonstrate the kinematic saturation through the results in Case 2. Time constants of the proposed method are presented in Fig. 16(a). Fig. 16(b) shows the kinematic saturation of the TCP, TOV and joint motion at four linear segments before blending, which are calculated as \( \max \left( {\begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}/{V}_{\max }^{p}}\right) ,\max \left( {\begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}/{A}_{\max }^{p}}\right) ,\max \left( {\begin{Vmatrix}{\mathbf{v}}_{k}\end{Vmatrix}/{J}_{\max }^{p}}\right) ,\max \left( {\begin{Vmatrix}{\mathbf{\omega }}_{k}\end{Vmatrix}/{V}_{\max }^{o}}\right) ,\max \left( {\begin{Vmatrix}{\mathbf{\omega }}_{k}\end{Vmatrix}/{A}_{\max }^{o}}\right) ,\max \left( {\begin{Vmatrix}{\mathbf{\omega }}_{k}\end{Vmatrix}/{A}_{\max }^{o}}\right) \) , \( \mathop{\max }\limits_{{\mu  = 1\ldots 5}}\left( {{q}_{\mu }^{\prime }/{V}_{\max }^{\mu }}\right) ,\mathop{\max }\limits_{{\mu  = 1\ldots 5}}\left( {{q}_{\max }^{\prime }/{A}_{\max }^{\mu }}\right) \) and \( \mathop{\max }\limits_{{\mu  = 1\ldots 5}}\left( {{q}_{\mu }^{\prime \prime }/{J}_{\max }^{\mu }}\right) \) respectively. \( k \) is the index number. Despite the TCP and TOV change rates satisfying the user-defined tolerance, joint kinematics still exceed the tolerance in comparative methods. This occurs when joint constraints are ignored or simplified, resulting in either under or over-saturation, ultimately leading to sub-optimality or kinematic violation. As a result, Ref. [26] violates the joint constraints, reaching 189.1 % before blending. If the filter remains constant along the toolpath as proposed in Ref. [26], fixing joint violations requires global scaling, which compromises machining efficiency unnecessarily as violations often occur locally. In contrast, Ref. [27]'s maximum saturation is 113.6% before blending, with the final kinematic saturation being 191.6 %, as the prediction and correction scheme fails to fix the violations caused by blending. Ref. [27] provides options for local scaling since the filters adapt to different segment. However, it still faces difficulties due to the use of separate filters for TCP and TOV, which complicates the prediction of higher order derivatives of the joint trajectory. Without an efficient kinematic approximation, finding the optimal delay time to fix violations requires repetitive interpolations, making the process time-consuming. The proposed method optimizes the time constants with the GP solver, fully saturating joint kinematics and ensuring optimal kinematic performance.

<!-- Meanless: 25-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

<!-- Media -->

<!-- figureText: (a) (b) (c) -0.01 0.01 0.1 0.2 mm \( \mathrm{{Ra}} = {2.09\mu }\mathrm{m} \) 4.0 Ra 3.5 3.0 2.5 2.0 1.5 1.0 0.5 0.0 4 mm 0 4 mm Surface profile error (mm) -0.2 -0.1 mm \( \mathrm{{Ra}} = {4.08\mu }\mathrm{m} \) 4.0 3.5 3.5 3.0 3.0 2.5 2.5 2.0 2.0 1.5 1.5 1.0 1.0 0.5 0.5 0.0 0 4 mm 0 -->

<img src="https://cdn.noedgeai.com/bo_d2sla4jef24c73b2kf4g_25.jpg?x=179&y=151&w=1345&h=881&r=0"/>

Fig. 24. The machining result of (a) the P2P method, (b) Ref. [11], and (c) the proposed method.

<!-- Media -->

Another noteworthy improvement is the implementation of geometric error constraining. When the error tolerance is set to 0.01 \( \mathrm{{mm}} \) (or rad),the actual errors,denoted as \( {\epsilon }_{k}^{p} \) and \( {\widehat{\epsilon }}_{k}^{0} \) ,are calculated offline and shown in Fig. 17(a) and (c),respectively. The accuracy of estimation,shown in Fig. 17(b) and (d),is calculated as \( {\varepsilon }_{k}/{\widehat{\varepsilon }}_{k} \) and is considered optimal when it equals to one. Inaccurate estimation can result in a violation of geometric error if \( {\varepsilon }_{k} > {\widehat{\varepsilon }}_{k} \) ,or sub-optimal delay time if \( {\varepsilon }_{k} < {\widehat{\varepsilon }}_{k} \) . In the comparison,all methods can guarantee geometric constraints. According to Ref. [26], geometric errors are estimated based on the same feed premise, as a result, actual geometric errors are observed to be smaller. On the other hand, Ref. [27]'s error models are found to be accurate. Although the proposed method accurately estimates the position error, there is some loss of accuracy during integration on the unit sphere, resulting in inexact orientation error estimation. In Section 2, the necessity of interpolation on the unit sphere for hybrid robots is explained. Despite the slight discrepancy in the estimated orientation error, the proposed estimation scheme is deemed safe for operation as it bounds the actual geometric errors. Fig. 17(e) shows the improvement of delay time compared to the P2P scenario.

When performing multi-axis operations, many factors can affect the interpolation, such as the toolpath's geometry and error tolerance. Furthermore, the result of the proposed method may vary depending on the workpiece's orientation when joint constraints are considered. To investigate the robustness of the proposed method, we rotate the CLs in Cases 1 and 2 with 100 random orientations within 10-degree rolls and pitches. The proposed method and the two filter-based methods [26,27] then interpolate the CLs under various setups. The results are shown in Fig. 18.

In terms of the geometric constraining, all errors are bounded under the tolerance. Meanwhile, the proposed method maintains real-time performance as depicted in Fig. 18(a) and (d),achieving a speed faster than \( {0.04}\mathrm{\;{ms}} \) per point and enabling an interpolation frequency of \( {25}\mathrm{{kHz}} \) ,regardless of the toolpath’s geometry and error tolerance. Most of the time is consumed by the optimization of time constants in comparison to other filter-based methods. As a tradeoff, the machining time of the proposed method is improved compared to Ref. [27] when the geometric error tolerance is above 0.016 mm (or rad),as shown in Fig. 18(b) and (e). Ref. [26] has the least machining time among the three methods. But it violates most kinematic constraints, as shown in Fig. 18(c) and (f), which can dangerously affect joint tracking accuracy. The proposed method generates \( {99}\% \) of the trajectories near full kinematic saturation with only two instances where the maximum violation is 175.6%, making it more reliable than the other two methods.

### 4.3. Experiment results of the flank milling

The flank milling experiment is conducted on the five-axis hybrid machining robot, as shown in Fig. 19(a). The system comprises three parts: a computer for trajectory generation, a Beckhoff real-time multi-axes controller, and a mechanical system that has five Panasonic actuators. The actuators are controlled at 1000 Hz with the standard proportional,integral,and derivative (PID) strategy. The machining parameters are given in Table. 4, and the machining setup is shown in Fig. 19(b). The workpiece is prepared with a rough planar mill operation, as seen in Fig. 19(c).

<!-- Meanless: 26-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

The finishing five-axis toolpath is generated by UG NX 12.0,as shown in Fig. 20,with a tolerance of \( {0.008}\mathrm{\;{mm}} \) . The toolpath is then interpolated with the proposed and spline-based methods from Ref. [11] at the period \( {T}_{s} = 1\mathrm{\;{ms}} \) . The geometric tolerance is \( {0.01}\mathrm{\;{mm}} \) (or rad),and the kinematic constraints are specified in Table. 1. The proposed method generates the trajectory in a mere 654 ms, whereas the spline-based method takes up 227.62 s. The maximum TCP blending error is \( {3.87\mu }\mathrm{m} \) ,and the maximum TOV blending error is 5.82 mrad. The overall kinematic saturation of the generated trajectory is \( {107.4}\% \) ,which is caused by the jerk of axis 4 at \( {26.769}\mathrm{\;s} \) .

In terms of efficiency,the proposed method reduces the machining time of the toolpath to only 43.872 s. This results in a significant improvement of 41.8% in machining efficiency compared to the P2P method. In comparison, the spline-based method from Ref. [11] finishes the toolpath in 53.066 s, as shown in Fig. 21(a) and (b). The TCP and TOV trajectories are shown in Fig. 21(c) and (d), where it is evident that the proposed method maintains the maximum feedrate ( \( 4\mathrm{\;{mm}}/\mathrm{s} \) ),while the spline-based method decelerates between different CLs under the kinematic constraints.

In terms of accuracy, the joint tracking error and the machined surface are presented. The trajectory's smoothness and the controller’s performance under time-variant machining force disturbances determine the joint tracking error \( {\delta }_{\mu } \) . Fig. 22 shows that the joint tracking error of the methods is less than \( {0.05}\mathrm{\;{mm}} \) . Compared to spline-based methods,FIR filters suppress the vibration of the motion system with the limited jerk of the filtered trajectory [20]. The actual jerk of the proposed method is consistent with the desired command, as shown in Fig. 23. The jerk signals are calculated with numerical differentiation after the Gaussian filter from the encoders' recordings.

The machining results of the P2P method, the spline-based method [11], and the proposed method are presented in Fig. 24. The Gocator 3210 scanner’s measurement shows that surface profile errors are limited to \( {0.2}\mathrm{\;{mm}} \) ,which is the positioning accuracy of the experimental platform. Furthermore, the similarity between the error distribution graphs proves that limited geometric error during smoothing \( \left( {{0.01}\mathrm{\;{mm}}/\mathrm{{deg}}\text{in this case}}\right) \) can prevent accuracy loss. The roughness is measured with a ZEISS confocal microscope system according to ISO 4287. The arithmetic mean roughness (Ra) of the machined surface by the P2P method is \( {4.08\mu }\mathrm{m} \) . The proposed and spline-based method improves the Ra to \( {1.63\mu }\mathrm{m} \) and \( {2.09\mu }\mathrm{m} \) ,respectively. The Ra result is consistent with inspection as the tool mark of the P2P method is more visible than the other smoothed toolpaths.

## 5. Conclusions

This paper proposes a real-time interpolation method for five-axis hybrid robots. The proposed method involves utilizing third-order finite impulse response (FIR) filters to interpolate discrete linear toolpaths with jerk-limited trajectory. The filtering guarantees the synchronization of tool center points (TCPs) and tool orientation vectors (TOVs). The time constants that determine the dynamics of the filters are optimized with geometric programming under user-defined constraints. The proposed filter-based interpolation method significantly improves the calculation efficiency using the convex optimizer and can be easily implemented in real-time with concise convolution. The efficiency of the trajectory is further enhanced with blending, where the geometric errors are modeled to determine delay time between velocity pulses. Meanwhile, the edge cases where the kinematic constraints are violated in the blending are addressed in the paper.

Taking a five-axis hybrid robot as an example, simulation on different toolpaths shows that the proposed method generates jerk-limited trajectories with significantly improved efficiency. In comparison to current spline-based smoothing methods [11,15], the proposed method takes less than \( {500}\mathrm{\;{ms}} \) for a \( {30}\mathrm{\;s} \) trajectory with 100 linear segments. Furthermore,the quality of the real-time generated trajectory is comparable to the offline optimized trajectory regarding the machining time, kinematic constraints, and geometric error. The robustness of the proposed method is verified under various conditions. In a cutting experiment, the flank-milled aluminum surface achieves a roughness of \( {1.63\mu }\mathrm{m} \) without any visible tool marks. The joint tracking accuracy reaches \( {0.05}\mathrm{\;{mm}} \) under time-variant machining force, with vibration being suppressed by the filters.

## Declaration of competing interest

The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper.

## Data availability

Data will be made available on request.

## Acknowledgements

This work was supported by the National Natural Science Foundation of China [grant numbers: 51935010, 52275501]. References

<!-- Meanless: 27-->




<!-- Meanless: Z. Shi et al. Mechanical Systems and Signal Processing 209 (2024) 111080-->

[1] S. Osei, W. Wang, Q. Ding, A new method to identify the position-independent geometric errors in the rotary axes of five-axis machine tools, J. Manuf. Process. 87 (2023) 46-53.

[2] A. Verl, A. Valente, S. Melkote, C. Brecher, E. Ozturk, L.T. Tunc, Robots in machining, CIRP Ann. 68 (2) (2019) 799-822.

[3] Z. Xie, F. Xie, X.-J. Liu, J. Wang, H. Su, A parallel machining robot and its control method for high-performance machining of curved parts, Rob. Comput. Inte Manuf. 81 (2023) 102501.

[4] Z. Xie, F. Xie, X.-J. Liu, J. Wang, B. Mei, Tracking error prediction informed motion control of a parallel machine tool for high-performance machining, Int. J. Mach. Tools Manuf. 164 (2021) 103714.

[5] J. Zhang, F. Xie, Z. Ma, X.-J. Liu, H. Zhao, Design of parallel multiple tuned mass dampers for the vibration suppression of a parallel machining robot, Mech. Syst. Signal Process. 200 (2023) 110506.

[6] M.O.T. Cole, P. Shinonawanik, T. Wongratanaphisan, Time-domain prefilter design for enhanced tracking and vibration suppression in machine motion control, Mech. Syst. Signal Process. 104 (2018) 106-119.

[7] C. Lartigue, E. Duc, A. Affouard, Tool path deformation in 5-axis flank milling using envelope surface, Comput-Aided Des. 35 (4) (2003) 375-382.

[8] S. Tulsyan, Y. Altintas, Local toolpath smoothing for five-axis machine tools, Int. J. Mach. Tools Manuf. 96 (2015) 15-26.

[9] H. Zhao, L. Zhu, H. Ding, A real-time look-ahead interpolation methodology with curvature-continuous B-spline transition scheme for CNC machining of short line segments, Int. J. Mach. Tools Manuf. 65 (2013) 88-98.

[10] D.-N. Song, J.-W. Ma, Y.-G. Zhong, J.-J. Yao, Global smoothing of short line segment toolpaths by control-point-assigning-based geometric smoothing and FIR filtering-based motion smoothing, Mech. Syst. Signal Process. 160 (2021) 107908.

[11] H. Liu, G. Li, J. Xiao, A C3 continuous toolpath corner smoothing method for a hybrid machining robot, J. Manuf. Process. 75 (2022) 1072-1088.

[12] Q.Z. Bi, J. Shi, Y.H. Wang, L.M. Zhu, H. Ding, Analytical curvature-continuous dual-Bezier corner transition for five-axis linear tool path, Int. J. Mach. Tool Manuf., 91 (2015) 96-108. https://doi.org/10.1016/j.ijmachtools.2015.02.002.

[13] X. Beudaert, P.-Y. Pechard, C. Tournier, 5-Axis tool path smoothing based on drive constraints, Int. J. Mach. Tools Manuf. 51 (12) (2011) 958-965

[14] M. Müller, G. Erdös, P. Xirouchakis, High accuracy spline interpolation for 5-axis machining, Comput.-Aided Des. 36 (13) (2004) 1379-1393.

[15] Z. Shi, W. Zhang, Y. Ding, A local toolpath smoothing method for a five-axis hybrid machining robot, Sci. China Technol. Sci. 66 (3) (2023) 721-742.

[16] R.J. Cripps, B. Cross, M. Hunt, G. Mullineux, Singularities in five-axis machining: Cause, effect and avoidance, Int. J. Mach. Tools Manuf. 116 (2017) 40-51.

[17] Y.G. Li, H.T. Liu, X.M. Zhao, T. Huang, D.G. Chetwynd, Design of a 3-DOF PKM module for large structural component machining, Mech. Mach. Theory 45 (6) (2010) 941-954.

[18] Q.-C. Pham, A general, fast, and robust implementation of the time-optimal path parameterization algorithm, IEEE Trans. Rob., 30 (2014) 1533-1540. https:// doi.org/10.1109/TRO.2014.2351113.

[19] E. Barnett, C. Gosselin, A bisection algorithm for time-optimal trajectory planning along fully specified paths, IEEE Trans. Rob. 37 (1) (2021) 131–145.

[20] L. Biagiotti, C. Melchiorri, FIR filters for online trajectory planning with time-and frequency-domain specifications, Control Eng. Pract. 20 (12) (2012) 1385-1399.

[21] W. Fan, X.-S. Gao, C.-H. Lee, K.e. Zhang, Q. Zhang, Time-optimal interpolation for five-axis CNC machining along parametric tool path based on linear programming, Int. J. Adv. Manuf. Technol. 69 (5-8) (2013) 1373-1388.

[22] J. Xiao, S. Liu, H. Liu, M. Wang, G. Li, Y. Wang, A jerk-limited heuristic feedrate scheduling method based on particle swarm optimization for a 5-DOF hybrid robot, Rob, Comput. Integr. Manuf. 78 (2022) 102396.

[23] Y. Sun, Y. Bao, K. Kang, D. Guo, An adaptive feedrate scheduling method of dual NURBS curve interpolator for precision five-axis CNC machining, Int. J. Adv. Manuf. Technol. 68 (9-12) (2013) 1977-1987.

[24] J. Huang, Y. Lu, L.-M. Zhu, Real-time feedrate scheduling for five-axis machining by simultaneously planning linear and angular trajectories, Int. J. Mach. Tools Manuf. 135 (2018) 78-96.

[25] S. Tajima, B. Sencer, Online interpolation of 5-axis machining toolpaths with global blending, Int. J. Mach. Tools Manuf. 175 (2022) 103862.

[26] S. Tajima, B. Sencer, Accurate real-time interpolation of 5-axis tool-paths with local corner smoothing, Int. J. Mach. Tools Manuf. 142 (2019) 1-15.

[27] Y. Liu, M. Wan, X-B. Qin, Q.-B. Xiao, W.-H. Zhang, FIR filter-based continuous interpolation of G01 commands with bounded axial and tangential kinematics in industrial five-axis machine tools, Int. J. Mech. Sci. 169 (2020) 105325.

[28] R.A. Ward, B. Sencer, B. Jones, E. Ozturk, Five-axis trajectory generation considering synchronization and nonlinear interpolation errors, J. Manuf. Sci. Eng., 144 (2022) 081002. https://doi.org/10.1115/1.4053460.

[29] H. Sun, J. Yang, D. Li, H. Ding, An on-line tool path smoothing algorithm for 6R robot manipulator with geometric and dynamic constraints, Sci. China Technol. Sci. 64 (9) (2021) 1907-1919.

[30] S. Boyd, S.-J. Kim, L. Vandenberghe, A. Hassibi, A tutorial on geometric programming, Optim. Eng. 8 (1) (2007).

[31] J. Yang, A. Yuen, An analytical local corner smoothing algorithm for five-axis CNC machining, Int. J. Mach. Tools Manuf. 123 (2017) 22-35.

[32] M. ApS, Mosek optimization toolbox for matlab, User's Guide and Reference Manual, Version, 4 (2019) 1.

[33] M.M. Ghazaei Ardakani, B. Olofsson, A. Robertsson, R. Johansson, Model predictive control for real-time point-to-point trajectory generation, IEEE Trans. Autom. Sci. Eng. 16 (2) (2019) 972-983.

<!-- Meanless: 28-->

