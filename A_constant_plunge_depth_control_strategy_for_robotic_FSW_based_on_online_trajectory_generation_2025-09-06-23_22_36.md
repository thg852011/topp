

<!-- Meanless: Robotics and Computer-Integrated Manufacturing 80 (2023) 102479 Contents lists available at ScienceDirect Robotics and Computer-Integrated Manufacturing ELSEVIER journal homepage: www.elsevier.com/locate/rcim updates-->

# A constant plunge depth control strategy for robotic FSW based on online trajectory generation

Juliang Xiao \( {}^{a} \) , Mingli Wang \( {}^{a} \) , Haitao Liu \( {}^{a, * } \) , Sijiang Liu \( {}^{a} \) , Huihui Zhao \( {}^{b} \) , Jiashuang Gao \( {}^{b} \)

\( {}^{a} \) Key Laboratory of Mechanism Theory and Equipment Design of Ministry of Education,Tianjin University,Tianjin 300350,China

\( {}^{b} \) Shanghai Aerospace Equipments Manufacturer Co.,Ltd,Shanghai 200245,China

## ARTICLEINFO

Keywords:

Robotic friction stir welding (RFSW)

Constant plunge depth control

Online trajectory generation

Process constraints

## A B S T R A C T

Robotic friction stir welding (RFSW) usually comes with a huge upsetting force, and the stiffness of the welding system distributes unevenly over the position, which leads to a large deviation of the plunge depth of the tool at the end of the robot. The conventional constant distance tracking control suffers from the problem of unsmooth compensation leading to the vibration of the robot and thus degrading the weld quality. For this problem, a constant plunge depth control based on online trajectory generation for RFSW is studied, which can generate an accurate welding trajectory according to the rough initial reference path and smoothly compensate for the plunge deviation. Initially, three laser-ranging sensors are utilized to measure the pose deviation of the tool in real-time and generate the ideal welding trajectory according to the projection vector method. Then, a deformation compensation model is established to realize the real-time prediction of the correct value. To ensure the smoothness and rapidity of the dynamic tracking process of displacement deviation, we adopt an online trajectory generator as the core of optimization control to meet the process constraints such as speed, acceleration, and jerk during the compensation process. Finally, simulation and experiment are carried out. The results show that the proposed method can effectively reduce the vibration caused by compensation during the welding process and reduce flash, which can improve the welding quality.

## 1. Introduction

Friction stir welding (FSW) is a solid-state welding technology that has been widely used in aerospace, marine equipment, and road traffic industries \( \left\lbrack  {1,2}\right\rbrack \) . With the development of industrial robot,its applications for FSW have received much attention due to its greater flexibility, larger working space, and lower cost relative to traditional welding equipment dedicated to FSW. It is especially suitable for welding large-scale complex parts with three-dimensional curves [2]. Therefore, robotic FSW (RFSW) has great application prospects.

However, RFSW is a complex thermal-mechanical coupling nonlinear process \( \left\lbrack  {3,4}\right\rbrack \) ,and there are numerous difficulties in its practical application. First, inaccurate calibration of the workpiece coordinate system will produce static errors on the weld track [5]. Second, the insufficient stiffness of the robot-workpiece system and excessive forge force will cause deformation of the workpiece and the robot, resulting in transverse and longitudinal dynamic deviations of the expected welding path \( \left\lbrack  {3,6,7}\right\rbrack \) . Third,the welding parameters such as spindle speed, welding speed, and welding pin shape directly affect the metal heat flow and stress change \( \left\lbrack  {3,8}\right\rbrack \) . The above factors impact the quality of RFSW. Uncompensated robot machining trajectories lead to defects such as poor quality of welding joints, low bending strength, and even non-penetrating welds, thus failing to use [9,10].

To overcome the above challenges, notice that the RFSW usually works with a low welding speed [11], which allows manual online intervention [12]. The operator adjusts the compensation value of the robot tool center point (TCP) to improve the welding quality by observing the welding condition and shaking the handwheel. Nevertheless, this manual observation is inaccurate and has large hysteresis, which cannot fundamentally enhance the welding quality. For welding a super-long seam on a large thin-walled tank, this qualitative manual adjustment of the TCP requires many human strengths, which is impractical.

In view of the defects of manual compensation, methods such as constant depth control and constant downforce, torque, or power control [13-15], to adjust the welding process trajectory automatically to ensure RFSW quality. The PID control can meet the requirements of control accuracy and fast response. However, the overly rapidly unsmooth compensation will cause oscillations in the RFSW system. Certain scholars [16] have adapted fuzzy PID to realize constant process force welding to reduce the fluctuation of upsetting force during welding. Since the heat of the weldment is difficult to monitor, and the welding force is related to the welding parameters, the workpiece is heated and softened, resulting in a change in force. When the shoulder fully plunges into the workpiece, if the force sensor detects that it is less than the set value, it will continue to press down, thus failing the downforce control [17]. The constant depth control is different from force control in principle and can avoid the defects of the force control. Therefore, this paper mainly focuses on the constant plunge depth control and will solve the inconsistency of pressing depth caused by insufficient rigidity of the robot-weldment system through trajectory compensation.

---

<!-- Footnote -->

* Corresponding author.

E-mail address: liuht@tju.edu.cn (H. Liu).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.rcim.2022.102479 Received 24 June 2022; Received in revised form 30 September 2022; Accepted 17 October 2022 Available online 29 October 2022 0736-5845/(© 2022 Elsevier Ltd. All rights reserved.-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

To compensate for the error and improve the machining quality, a series of studies have been conducted on each source of deviation in the robot-workpiece system. The sources of deviation are mainly attributed to low robot joint stiffness and workpiece deformation. Based on the calibration of the kinematic parameters of the mechanism [18], some solutions focus on building robot configuration error models and attribute the TCP deformation to insufficient joint stiffness [11]. These solutions mainly include offline or online trajectory corrections [19], for example, deformation prediction from joint stiffness estimation [20-23] and the neural network [10], adopting dual encoders to measure the error of each joint [7], online diagnostics of joint deviations based on information fusion [24] and using sensorless external observers [25]. Guillo and Dubourg [9] compensated the trajectory of the robot by analyzing joint stiffness and applying the feedforward compensation to improve the machining accuracy. Jing et al. [26] introduced an overall flexible stiffness index, which improved the overall stiffness of the serial RFSW process trajectory. Although these studies can predict the joint deformation of a robot and compensate it to some extent, for the parallel robot studied in this paper, it is difficult to establish an accurate stiffness model when the dynamics and stiffness parameters are inaccurate or unknown [9]. As a result, the end deformation cannot be compensated accurately in real-time.

Another approach is to treat the compensation of weld trajectory deviations caused by workpiece deformation as a path or force tracking problem under an unknown surface. Current solutions measure the weld surface to track the welding seam [6,27]. For example, use force sensors [29], distance sensors with 3D vision, and other measurement techniques [28] to perform real-time 3D surface exploration. Most of the solutions aim at arc welding, and their application scenarios are non-contact processing. Their TCPs are subject to zero force, thus they had not considered the end deformation caused by the end forces, which is different from the RFSW. Recently, Amersdorfer et al. [30] adopted three laser-ranging sensors and a force sensor to track an unknown surface and achieve constant force processing. It is worth noting that these sensor-based measurement technologies can realize online compensation control.

Based on the above reasons, to realize the quantitative automatic control of welding compensation, a control strategy evolved from the manual compensation is proposed to study the online error compensation of RFSW. The S-curve velocity profile is adopted to smooth the compensation to avoid the system oscillations. A stable error compensation command can be obtained when applying the S-curve velocity profile in the constant depth control process. There are many studies on the online S-curve velocity planning. Planning in the displacement-velocity phase plane can get a time-optimal S-curve velocity profile that satisfies the specified constraints \( \left\lbrack  {{31},{32}}\right\rbrack \) . However,the calculation accuracy is affected by the number of discrete points. As the number of discrete points increases, the calculation time will also increase. Besides, this method is only applicable to situations where the trajectory does not turn back. Since online trajectory generation (OTG) can perform time-optimal planning under multiple constraints, it can be applied to real-time compensation of deviations. OTG is essentially a procedure of shaping and constraining the jerk input. Liu [33] proposed an online trajectory generator that satisfies the jerk constraint when the target state changes in real-time. However, the target velocity and acceleration are zero. In order to realize online planning for more general problems, considering that the S-curve velocity profile has the uncertainty characteristics of segments, Kröger and Wahl [34] studied a process of determining the type of acceleration by classifying the basic acceleration profiles and solving for each class separately. Recently, in [35], the problem of online trajectory planning under jerk constraint with non-zero target acceleration has been studied.

The above works directly calculate in the continuous-time domain. Certain scholars extend the trajectory planning problem in the time domain to the frequency domain by using filters to realize online planning. For example, Biagiotti and Melchiorri [36], and Besset et al. [37] used multiple FIR filters to achieve velocity, acceleration, and jerk constraints of the planned trajectory. However, the filter methods will cause time lags and difficulty in calculating filter parameters. Therefore, based on the above research, considering that any complex motion can be approximated by limited base trajectory units such as the Taylor series expansion, the online S-curve velocity planning base units can be regarded as the core of the OTG to achieve constant depth control.

In this work, based on previous research [38], an online S-curve generator is proposed to generate RFSW compensation in real time. Unlike traditional solutions, this research performs online error compensation from the perspective of trajectory planning. There is no need to modify the internal algorithm of the robot control system, and the external input is only responsible for compensating the trajectory in the non-feed direction under the welding process constraints. The primary consideration is to make the actual RFSW path converge to the desired welding path as quickly as possible, eventually stabilizing the trajectory deviation within a reasonable range. For the purpose to avoid oscillations during the compensation process, which results in reduced welding quality, the compensation welding speed, acceleration, and jerk are constrained to produce a smooth and stable weld motion. Considering the lower welding speed of RFSW compared with other applications, the influence of joint torque limits is not considered here. The innovations of this paper are as follows:

(1) A method of constant plunge depth control for RFSW based on online trajectory generation is proposed, which can be applied to a closed robot controller without changing the internal program of the controller.

(2) The proposed control strategy can realize the autonomous tracking with a rough reference welding path, which reduces the accuracy requirements of the original welding trajectory.

(3) An online S-curve error compensator is proposed to track changing targets and meet the velocity, acceleration, and jerk constraints. It points out that the core of the online S-curve planning is to calculate the initial jerk of each planning step.

The following of this paper is organized as follows: Section 2 proposes a constant plunge depth control scheme for RFSW, and expounds on the real-time detection of trajectory deviation and autonomous tracking, respectively. Section 3 proposes a compensator based on the online S-curve velocity planner that can generate compensation commands according to the predicted target input. In Section 4, the trajectory tracking simulation of the proposed online S-curve velocity tracking is carried out. Then, a FSW experiment of constant plunge depth control was carried out on the self-developed hybrid robot TriMule 800. Section 5 summarizes the full text and looks forward to future work.

## 2. Constant plunge depth control strategy for RFSW

In this section, a constant plunge depth control scheme is proposed, which can realize adaptive seam tracking based on projection vector method in FSW process. To facilitate the description of the RFSW, the FSW process is first introduced to clarify some basic concepts. Next, a constant plunge depth control scheme for the RFSW is given. An instantaneous end pose detection of TCP is then studied for the feedback detection part of the control link. Finally, based on the instantaneous error detection, the autonomous tracking of a weld seam with an inaccurate reference trajectory is investigated.

<!-- Meanless: 2-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

### 2.1. RFSW process

The basic process of FSW is shown in Fig. 1. First, the FSW tool rotates and slowly plugs into the workpieces to the pre-set plunge depth. Next, the pin rotates and dwells inside the workpieces for a while to reach the desired weld temperature. Then, the FSW tool moves along the seam direction to generate a welded joint between the two workpieces. Finally, pull out the pin to finish the FSW when the weld is complete.

Typically, the FSW process requires the tool to form a certain tilting angle relative to the workpiece surface, and the plunge depth of the shoulder is about \( {0.2}\mathrm{\;{mm}} \) to ensure adequate forging of the material so that the weld material forms a dense joint. Its forge force usually reaches tons level for the thick plate welding process. Considering that the stiffness of the TriMule 800 robot studied in this paper is lower than that of the CNC machine tool, and the welding parts are deformed by the forge force, the above may lead to large fluctuations in the plunge depth of the tool shoulder and reduce the welding quality. Therefore, the deformation compensation of the RFSW system plays a key part in improving welding quality.

We adopt real-time detection of the plunge deviation based on the laser distance measurement sensors to achieve real-time compensation instead of manual and ensure the consistency of the shoulder plunge depth. Since the relationship between plunge deviation and compensation value is nonlinear, a rough model of the deformation-compensation relationship of the RFSW system can be built during the stirring pin immersion phase, which is then refined during the welding process. Therefore, by measuring the plunge deviation, the compensation can be predicted in real-time. The specific implementation details are described below.

### 2.2. Constant plunge depth control for RFSW

The purpose of automatic tracking with constant plunge depth is to keep the tool depth constant during RFSW in case of inaccurate reference paths. This work combines the online S-curve velocity tracking with the real-time compensation controller, which can be applied to Power PMAC to achieve real-time feedback compensation for the deviation and improve the welding quality of the RFSW. The control flow is shown in Fig. 2. The lower-right gray frame in Fig. 2 represents the operation flow without compensation, while the others represent the compensation flow. In the uncompensated flow, the discrete points of the weld track generated by the interpolation module of Power PMAC are sent to the controller at the current control cycle \( k \) . After inverse kinematic calculation, the controller sends the joint trajectories to the motor drivers. Finally, the drivers control the motor so that the manipulator performs the welding action. Due to the influence of the welding force of the robot-workpiece system, the actual welding path will deviate from the ideal path and this deviation needs to be compensated.

The feedback flow of the compensation control uses laser ranging sensors to measure the plunge depth of the pin. Then, the measurement results are digitally filtered to remove random noise. A difference is made with the reference plunge depth to get the compensation deviation. This part will be discussed in this section. Considering that the variation is not equal to the desired compensation, a real-time prediction of compensation from the deviation is required. The above process can also be replaced by manual intervention. The predicted compensation is fed into the online error compensator to achieve real-time compensation for the next cycle and add it to the interpolation position generated by Power PMAC. This part will be discussed in Section 3. The above control process is executed in real-time out of the controller to achieve real-time compensation for deviations in the FSW process.

Unlike the model predictive control that instead of predicting compensation amounts for a fixed period, this scheme predicts variable times or distances in real-time. Its predicted time and distance vary with the control period. This scheme performs new time-optimal S-curve planning with certain constraints in each control cycle, and only the first step is selected to calculate the control quantity. Since the inverse kinematics is called in real-time, the original reference trajectory can be modified by adding this control quantity of the compensation module with the interpolation trajectory generated by \( \mathrm{G} \) code in real-time. Finally, the compensated trajectory was inputted into the inverse kinematics module to achieve a smooth compensation process.

Note that the above control scheme is based on the assumption of small deformation at the robot TCP. Otherwise, if the robot deformation is large when adopted to semi-closed-loop joint control, the accuracy of the inverse kinematic algorithm will lose. The actual joint position of the robot is not the driver-read position, making it difficult to compensate accurately for the end deviation. In this case, grating displacement sensors or encoders need to be installed at the robot joints to measure the end error caused by joint deformation in real time. The key to the deviation compensation control system depicted in Fig. 2 is the theoretical compensation prediction and the online deviation compensation process. In the following, the deviation detection scheme is introduced, based on which the online error compensation process will be discussed in Section 3.

### 2.3. Plunge depth detection and autonomous seam tracking

In this part, three laser ranging sensors are adopted to realize the real-time measurement of weld surface topography and axial deviation at any pose. It is aimed to calculate the ideal frame \( {P}_{\mathrm{c}}^{\prime } \) with a certain distance and tilt angle (such as vertical) from the measured surface. The schematic and structural diagrams of the sensing module mechanism are shown in Fig. 3 and Fig. 4, respectively. It is assumed that the normal intersection \( T \) of the tool frame on the workpiece surface under the current pose is located on the measurement surface composed of three laser measurement points \( {S}_{\mathrm{r}i},\left( {i = 1,2,3}\right) .{P}_{\mathrm{c}} \) is the frame of the TCP, which is a global default reference frame.

<!-- Media -->

<!-- figureText: FSW Shoulder Tilt angle Plunge depth Welding direction Joint (c) (d) tool Pin Workpiece (a) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_2.jpg?x=401&y=1708&w=942&h=438&r=0"/>

Fig. 1. The basic processes of FSW.

<!-- Meanless: 3-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

<!-- figureText: \( \varphi \) \( {d}_{k} \) Section 2. Constant plunge depth control for RFSW \( {d}_{k}^{s} \) Laster ranging \( k = k + 1 \) Flitter sensors Inverse \( {\mathbf{\theta }}_{k + 1} \) Robot \( {y}_{k + } \) kinematics manipulator Workpiece \( {p}_{k + 1}^{\mathrm{r}} \) PMAC interpolator PMAC motion control \( {d}_{\text{ain }} \) Calculate current pose Calculate desired Manual Online error target pose intervention compensator \( {e}_{k} \) Compensation \( {u}_{k}^{ * } \) prediction Section 3. Online S-curve error compensator and compensation prediction -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_3.jpg?x=207&y=154&w=1332&h=490&r=0"/>

Fig. 2. Constant plunge depth control strategy for RFSW.

<!-- Media -->

The coordinates of the measurement points concerning the current frame \( {P}_{\mathrm{c}} \) can be calculated based on the laser ranging sensor mounting dimensions \( {R}_{{\mathrm{S}}_{i},{P}_{\mathrm{c}}} \) and the measured distance vector \( {v}_{{S}_{ri},{S}_{i}} \) as follows:

\[{\mathbf{S}}_{\mathrm{r}i} = {R}_{{\mathrm{S}}_{i},{P}_{\mathrm{c}}}{\mathbf{v}}_{{\mathrm{S}}_{\mathrm{r}i},{\mathrm{\;S}}_{i}},i \in  \{ 1,2,3\}  \tag{1}\]

Then,the homogeneous transformation matrix \( {R}_{T} \) of the frame \( T \) relative to the frame \( {P}_{\mathrm{c}} \) can be calculated.

\[\left\{  \begin{array}{l} {\widehat{\mathbf{z}}}_{T} = \operatorname{norm}\left( {{\mathbf{v}}_{{\mathrm{S}}_{\mathrm{r}2},{\mathrm{\;S}}_{\mathrm{r}3}} \times  {\mathbf{v}}_{{\mathrm{S}}_{\mathrm{r}1},{\mathrm{\;S}}_{\mathrm{r}2}}}\right) \\  {\widehat{\mathbf{x}}}_{T} = {R}_{{P}_{\mathrm{c}},B}^{T}\operatorname{norm}\left( {{\mathbf{v}}_{\text{ref }} - {\left( {\widehat{\mathbf{z}}}_{T}\right) }^{T}{\mathbf{v}}_{\text{ref }}{\widehat{\mathbf{z}}}_{T}}\right)  \end{array}\right.  \tag{2}\]

where \( \operatorname{norm}\left( \bullet \right) \) is a standardized function. It is noticed that when calculating the trajectory direction \( {\widehat{\mathbf{x}}}_{T} \) ,the pose of robot TCP \( {R}_{{P}_{c},B} \) and reference speed direction \( {v}_{\text{ref }} \) are introduced. These parameters are described in the base frame \( B \) so that the actual robot TCP cannot be accurately obtained from the driver actuators. However, as this paper only focuses on the \( z \) -axis direction (deflection direction) of the tool,and the deviation of the \( x \) and \( y \) directions has little effect on the attitude \( {R}_{{P}_{\mathrm{c}},B} \) ,we approximately consider \( {R}_{{P}_{\mathrm{c}},B} \) to be the TCP attitude derived from the forward kinematics. The accuracy of \( {\widehat{\mathbf{x}}}_{T} \) mainly affects the rotation direction of the tool and the steering of the sixth axis, so the influence of its calculation error on the actual processing can be disregarded.

By substituting any point \( {S}_{\mathrm{r}i} \) and normal vector \( {\widehat{\mathbf{z}}}_{T} \) into the point-normal equation for a plane,the coordinates of the intersection \( T \) between the current tool axis and the measuring surface in the frame \( {P}_{\mathrm{c}} \) can be obtained. The position of \( \mathbf{T} \) can be expressed as \( \mathbf{T} = {\left\lbrack  0,0,{z}_{T}\right\rbrack  }^{T} \) ,which can be solved by Eq. (3).

\[{\left( {\widehat{\mathbf{z}}}_{T}\right) }^{T}\left( {\mathbf{T} - {S}_{\mathrm{r}i}}\right)  = 0,\forall i = 1,2,3. \tag{3}\]

<!-- Media -->

<!-- figureText: Servo motor _____ Gear ring Laser ranging sensor -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_3.jpg?x=243&y=1784&w=463&h=360&r=0"/>

Fig. 3. Structure diagram of the laser ranging module.

<!-- figureText: \( {\mathrm{S}}_{2} \) \( V \) , \( {\mathrm{S}}_{\mathrm{r}1} \) \( B \) \( {\mathrm{S}}_{\mathrm{r}} \) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_3.jpg?x=985&y=734&w=563&h=406&r=0"/>

Fig. 4. Determine the TCP error via laser ranging sensors.

<!-- Media -->

Then we can get

\[{R}_{T} = \left\lbrack  \begin{matrix} {\widehat{\mathbf{x}}}_{T} & {\widehat{\mathbf{z}}}_{T} \times  {\widehat{\mathbf{x}}}_{T} & {\widehat{\mathbf{z}}}_{T} & \mathbf{T} \\  0 & 0 & 0 & 1 \end{matrix}\right\rbrack   \tag{4}\]

Finally, according to the requirements of FSW, it should control the tilting angle \( \varphi \) between the ideal configuration \( {P}_{\mathrm{c}}^{\prime } \) and the frame \( T \) ,and the shoulder plunge depth \( {d}_{\text{aim }} \) . Taking these parameters as an input,the homogeneous transformation matrix of the ideal configuration relative to the current configuration \( {R}_{{P}_{\mathrm{c}}^{\prime }{P}_{\mathrm{c}}} \) can be obtained by Eq. (6).

\[{R}_{T,{P}_{\mathrm{c}}^{\prime }} = \operatorname{Roty}\left( \varphi \right) \operatorname{Transz}\left( {d}_{\text{aim }}\right)  \tag{5}\]

\[{R}_{{P}_{\mathrm{c}}^{\prime },{P}_{\mathrm{c}}} = {R}_{T}{\left( {R}_{T,{P}_{\mathrm{c}}^{\prime }}\right) }^{-1} \tag{6}\]

After the above calculation, we can get the pose deviation matrix \( {R}_{{P}_{\mathrm{c}}^{\prime },{P}_{\mathrm{c}}} \) . The process of deviation compensation can be regarded as converting the transforming matrix \( {R}_{{P}_{\mathrm{c}}^{\prime },{P}_{\mathrm{c}}} \) to a unit matrix.

It is worth noting that the above detection process avoids the problem that the terminal TCP rotates around the \( z \) -axis due to the joint error of the manipulator,thereby causing an error in the calculation of \( {R}_{{P}_{\mathrm{c}}^{\prime },{P}_{\mathrm{c}}} \) . However, in practical applications, large rotations of the TCP around the \( z \) -axis should be avoided. For example,the TCP must rotate when the error detection device is mounted on the five-axis, which should be avoided. It is different from the robot machining that FSW requests constant plunge depth, resulting in grooves and flashes on the workpiece surface behind the welding side. If using a five-axis robot for simultaneous processing and detection, the end-mounted sensors will inevitably rotate, which will interfere with the detection and cause a severe offset of the welding track. An auxiliary axis is installed at the end of the robot to address the above problems. Unlike the six-axis machining, the laser ranging sensors are mounted on the sixth axis to detect the distance in real time. The fifth axis is used for direct machining so that the sensors can avoid normal rotation along the trajectory of the welding surface.

<!-- Meanless: 4-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

### 2.4. Autonomous tracking with a constant plunge depth

Based on the previous calculation of the pose deviation at a certain position, this part extends the position error to the trajectory and investigates autonomous tracking with an inaccurate reference seam. Fig. 5 shows a diagram of autonomous tracking of an inaccurate seam based on the projection vector method in one control cycle. The main idea is to estimate the pose deviation and calculate the corresponding ideal pose under the current position at the current position. Then, calculate the pose of the next target point, according to the reference trajectory combined with the current ideal pose. This calculation is performed in each control cycle, enabling autonomous exploration of the final welding trajectory.

As shown in Fig. 5,where \( {P}_{\mathrm{c}} \) represents the end frame of the TCP. \( {P}_{i + 1}^{\mathrm{r}} \) is the \( i + 1 \) th discrete interpolation point of the reference trajectory. \( {P}_{\mathrm{e}} \) represents the corrected target position in the \( i \) th interpolation cycle. \( {d}_{\text{ref }} \) is the unit reference projection direction. \( {v}_{Pc} \) represents the vertical displacement,that is,the projected component of the \( {P}_{\mathrm{c}}{P}_{\mathrm{c}}^{\prime } \) vector on the \( {d}_{\text{ref }} \) . \( {h}_{Pc} \) represents the horizontal displacement,that is,the projection vector of the \( {P}_{\mathrm{c}}{P}_{i + 1}^{\mathrm{r}} \) vector on the \( {d}_{\text{ref }} \) vertical plane. Specifically,the calculation is as follows:

\[{v}_{Pc} = {d}_{\text{ref }}^{\mathrm{T}}{P}_{\mathrm{c}}{P}_{\mathrm{c}}^{\prime } \cdot  {d}_{\text{ref }} \tag{7}\]

\[{h}_{Pc} = {P}_{\mathrm{c}}{P}_{i + 1}^{r} - {d}_{\text{ref }}^{\mathrm{T}}{P}_{\mathrm{c}}{P}_{i + 1}^{r} \cdot  {d}_{\text{ref }} \tag{8}\]

Finally,the target displacement vector of the \( i \) th interpolation period can be obtained by summing the horizontal and vertical displacements, as shown in Eq. (9). The attitude matrix of \( {P}_{\mathrm{e}} \) is equal to that of \( {P}_{\mathrm{e}}^{\prime } \) . The theoretical compensation of the target displacement under the \( i \) th interpolation cycle then can be respected as \( {P}_{i + 1}^{\mathrm{r}}{P}_{\mathrm{e}} \) .

\[{P}_{\mathrm{e}} = {P}_{\mathrm{c}} + {v}_{Pc} + {h}_{Pc} \tag{9}\]

In summary, this section mainly presents the deviation detection of the proposed real-time compensated predictive control. Firstly, an autonomous tracking strategy with a constant plunge depth based on online trajectory generation is proposed. Then, a terminal distance with an attitude detection scheme based on three laser-ranging sensors is studied. Finally, the theoretical target compensation position is generated according to the reference interpolation trajectory point.

<!-- Media -->

<!-- figureText: Ref path \( {P}_{i + 1}^{\mathrm{r}} \) \( {h}_{{P}_{i}} \) \( P \) pin \( B \) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_4.jpg?x=203&y=1775&w=547&h=343&r=0"/>

Fig. 5. Autonomous tracking of an inaccurate seam based on the projection vector method (under the \( i \) th interpolation cycle).

<!-- Media -->

## 3. Online S-curve error compensator and compensation prediction

When an unsmooth target compensation is added directly to the theoretical interpolation position, it will cause the tool to vibrate, thus reducing the welding quality. This section studies a smooth accumulation process of the theoretical compensation position, and the prediction-correction process is carried out to ensure the plunge depth and improve the smoothness of the compensation command. The S-curve velocity profile is used to smoothly compensate for the original reference command to improve the stability of the welding path with compensation. The following describes the basic definition, structure, and real-time calculation process of the input compensation of the online error compensator.

### 3.1. Problem description

Before introducing the online S-curve planning, certain terms should be presented. We assume a general time-discrete state of the S-curve as follows:

\[\varsigma \left( {t}_{i}\right)  = {\varsigma }_{i} = {\left\lbrack  {x}_{i},{\dot{x}}_{i},{\ddot{x}}_{i}\right\rbrack  }^{\mathrm{T}},i = 0,\ldots ,e \tag{10}\]

where \( e \) is the total number of discrete points. \( {x}_{s},{\dot{x}}_{s} \) ,and \( {\ddot{x}}_{s} \) represent initial displacement, velocity, and acceleration, respectively. Similarly, \( {x}_{e},{\dot{x}}_{e} \) ,and \( {\ddot{x}}_{e} \) represent target displacement,velocity,and acceleration, respectively.

To facilitate the description of constraint values, we choose symmetric third-order constraints, i.e., the velocity, acceleration, and jerk bounds are denoted by

\[\bar{\zeta } = {\left\lbrack  {v}_{\max },{a}_{\max },{j}_{\max }\right\rbrack  }^{\mathrm{T}},\underline{\varsigma } =  - \bar{\varsigma } \tag{11}\]

According to the above definitions, the time-optimum S-curve planning problem can be summarized as achieving the target state in the optimal time with the given initial state and constraint bounds.

The displacement, velocity, and acceleration equations corresponding to the S-curve profile [34] can be summarized and written in a matrix form. The state-space representation of any third-order finite trajectory can be expressed as follows:

\[\varsigma \left( {t}_{n + 1}\right)  = K\left( {t}_{n + 1,n}\right) \varsigma \left( {t}_{n}\right)  + \lambda \left( {t}_{n + 1,n}\right) x\left( {t}_{n}\right) . \tag{12}\]

where \( K\left( {t}_{n + 1,n}\right)  \in  {\mathbb{R}}^{3 \times  3},\lambda \left( {{t}_{n + 1},{}_{n}}\right)  \in  {\mathbb{R}}^{3} \) ,and \( {t}_{n + 1,n} = {t}_{n + 1} - {t}_{n} \) ,which satisfy

\[K\left( {t}_{n + 1,n}\right)  = \left\lbrack  \begin{matrix} 1 & {t}_{n + 1,n} & {t}_{n + 1,n}^{2}/2 \\  0 & 1 & {t}_{n + 1,n} \\  0 & 0 & 1 \end{matrix}\right\rbrack  \]

\[\lambda \left( {t}_{n + 1,n}\right)  = {\left\lbrack  {t}_{n + 1,n}^{3}/6,{t}_{n + 1,n}^{2}/2,{t}_{n + 1,n}\right\rbrack  }^{\mathrm{T}}.\]

The complete S-curve velocity profile can be divided into 7 segments, which can represent the planning between any two start and target states. That is, if the zero-acceleration time segment is regarded as one segment, then the non-7-segment S-curve velocity profile, such as any \( 1 \sim  6 \) segment acceleration and deceleration,can be represented by 7 segments. The value of the control quantity \( x \) directly affects the S-curve profile when \( {t}_{n + 1,n} \) is fixed. For a complete 7-section S-curve profile, \( x \) meets the following conditions:

\[x\left( {t}_{n}\right)  = \left\{  \begin{matrix} \operatorname{sign}\left( {x}_{0}\right) {j}_{\max }, & n = 1,7 \\  0, & n = 2,4,6 \\   - \operatorname{sign}\left( {x}_{0}\right) {j}_{\max }, & n = 3,5 \end{matrix}\right.  \tag{13}\]

<!-- Meanless: 5-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

where \( \operatorname{sign}\left( \bullet \right) \) denotes the sign function. The corresponding velocity and acceleration profiles are shown in Fig. 6.

In addition to meeting the above conditions, the time-optimal S-curve trajectory also has the following necessary properties: if \( {t}_{i,i - 1} \neq  0 \) , \( \forall i \in  \{ 2,4,6\} \) ,it must have that:

\[\ddot{\mathrm{x}}\left( \mathrm{t}\right)  = \left\{  \begin{matrix} \operatorname{sign}\left( {\ddot{\mathrm{x}}}_{0}\right) {\mathrm{a}}_{\max }, & \mathrm{t} \in  \left\lbrack  {{\mathrm{t}}_{1},{\mathrm{t}}_{2}}\right\rbrack  \\   - \operatorname{sign}\left( {\ddot{\mathrm{x}}}_{0}\right) {\mathrm{a}}_{\max }, & \mathrm{t} \in  \left\lbrack  {{\mathrm{t}}_{5},{\mathrm{t}}_{6}}\right\rbrack  \\  0, & \mathrm{t} \in  \left\lbrack  {{\mathrm{t}}_{6},{\mathrm{t}}_{7}}\right\rbrack   \end{matrix}\right.  \tag{14}\]

\[\dot{\mathrm{x}}\left( \mathrm{t}\right)  = \operatorname{sign}\left( {\ddot{\mathrm{x}}}_{0}\right) {\mathrm{v}}_{\max }\;\mathrm{t} \in  \left\lbrack  {{\mathrm{t}}_{3},{\mathrm{t}}_{4}}\right\rbrack  \]

The above properties comply with the Bang-Bang control principle. The time-optimal S-curve velocity planning problem can be summarized as the following nonlinear programming:

\[\mathop{\min }\limits_{{x\left( {t}_{n}\right) }}T = \mathop{\sum }\limits_{{n = 0}}^{6}{t}_{n + 1,n}\left( {\varsigma \left( {t}_{s}\right) ,\varsigma \left( {t}_{e}\right) ,\bar{\varsigma },x\left( {t}_{n}\right) }\right) \]

\[\text{s.t.}\varsigma \left( {t}_{n + 1}\right)  = K\left( {t}_{n + 1,n}\right) \varsigma \left( {t}_{n}\right)  + \lambda \left( {t}_{n + 1,n}\right) x\left( {t}_{n}\right) \text{,}\]

\[n = 0,1,\ldots ,6,{t}_{n + 1,n} = {t}_{n + 1} - {t}_{n} \geq  0, \tag{15}\]

\[\varsigma \left( {t}_{0}\right)  = \varsigma \left( {t}_{s}\right) ,\varsigma \left( {t}_{7}\right)  = \varsigma \left( {t}_{e}\right) ,\]

\[x\left( {t}_{n}\right)  \in  \left\{  {-{j}_{\max },0,{j}_{\max }}\right\}  \text{,}\]

\[\ddot{x}\left( t\right)  \in  \left\lbrack  {-{a}_{\max },{a}_{\max }}\right\rbrack  ,\]

\[\dot{x}\left( t\right)  \in  \left\lbrack  {-{v}_{\max },{v}_{\max }}\right\rbrack  ,t \geq  0.\]

It is clear from the above analysis that this issue is difficult to translate into an existing optimization model. When applying S-curve velocity planning to online compensation, we are concerned only with the compensation corresponding to one interpolation point. Only one interpolation cycle needs to be planned forwards for each interpolation point. Therefore, it is necessary to calculate the initial jerk corresponding to the optimal trajectory at each interpolation time step.

### 3.2. Online S-curve error compensator

We have studied the online trajectory generation and time synchronization adjustment method with the zero-target acceleration in [38], and proposed that the initial jerk can be obtained by calculating the displacement corresponding to the target velocity and acceleration from the initial velocity and acceleration in the shortest time. On this basis, an online S-curve error compensator is proposed. As shown in Fig. 7, firstly, the sensing and estimation part generates a target state of the online S-curve planning module. Then, under the given constraints and current states,the initial jerk \( {x}_{0} \) can be calculated from the jerk controller. Finally,the acceleration,velocity,and displacement at time \( i \) +1 are obtained through three integral calculations as shown in Eq. (12), respectively. The real-time output is then set as the initial input for the next jerk controller to generate continuous and smooth reference compensation displacement commands.

<!-- Media -->

<!-- figureText: \( {t}_{0} \) \( {t}_{{}_{1}} \) \( {t}_{2} \) \( {t}_{4} \) time -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_5.jpg?x=203&y=1732&w=547&h=384&r=0"/>

Fig. 6. Acceleration and velocity profiles corresponding to a typical 7-section S-curve profile.

<!-- figureText: Sensing and \( {p}_{i + 1}^{\mathrm{r}} \) Ref interploator position \( {p}_{i + 1}^{\mathrm{r}} \) \( {S}_{i + 1} \) estimation \( {\ddot{x}}_{i + } \) \( {\dot{x}}_{i + 1} \) Jerk controller \( S \) \( {S}_{i} \) \( i = i + 1 \) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_5.jpg?x=907&y=152&w=722&h=337&r=0"/>

Fig. 7. The structure of the online error compensator.

<!-- Media -->

If the target acceleration is zero, the core unit of the online error compensator can be composed of time-optimal velocity profiles that does not consider the target displacement (velocity planning mode). According to whether there is a uniform acceleration segment, the acceleration profiles of the velocity planning mode can be divided into two categories, as shown in Fig. 8. Therefore, compared with the complex problem of piecewise uncertainty (that is, the total number of acceleration segments is uncertainty) corresponding to Eq. (15), this process does not require a detailed classification and solution of the total acceleration profiles [34]. We only need to study a sub-problem of this problem, i.e. using the velocity planning mode to solve the whole online planning problem.

In the velocity planning mode, it is not difficult to obtain the node time of each segment of the acceleration profile \( {t}_{n + 1,n},n = 0,1,2 \) . By comparing the target displacement with the displacement in the velocity planning mode,the initial jerk in each control cycle \( {x}_{i} \) can be determined. If the target displacement is greater, take the maximum jerk value as the initial jerk; if the target displacement is smaller, take the minimum jerk value as the initial jerk; otherwise, if the values are equal, the final planning jerk is the same as that of the velocity planning mode.

An online S-curve error compensator is proposed for variable target tracking that meets the constraints of velocity, acceleration, and jerk. The compensator generates constraint inputs for the proposed compensation control in Section 2, which can avoid oscillation caused by compensation state overshoot in constant distance control during the welding process.

### 3.3. Prediction of compensation

There is a nonlinear relationship between the total system deformation and actual compensation in the RFSW. Thus, it is necessary to study the relationship between deformation and the compensation value of the RFSW system. This part focuses on the input prediction of the online S-curve error compensator.

The deformation of RFSW includes the upsetting deviation and the normal deviation of the welding track. Since the deviation of the upsetting direction has a great influence on the welding quality, it is necessary to analyze the deviation of the upsetting direction. The relationship between the robot-workpiece system compensation and error is shown in Fig. 9.

Fig. 9 shows the relationship between the state that the tool just touches the surface of the workpiece and the actual end position change when it reaches the command position that has not yet been compensated. As shown in Fig. 9, \( r \) is the desired plunge depth; \( e \) and \( u \) represent the upsetting direction deviation and robot compensation, respectively; \( {k}_{1} \) represents the z-direction equivalent stiffness of the robot under the current configuration; \( {k}_{2} \) represents the equivalent stiffness of bending deformation at the contact between the pin and the welding workpiece, which is related to the structure and clamping mode of the welding workpiece; \( {k}_{3} \) represents the z-direction equivalent stiffness caused by the extrusion deformation of the welding workpiece material, which is related to the shape of the pin and produces a circumferential extrusion force \( {F}_{m} \) on the surface of the pin; \( {a}_{1} \) and \( {a}_{2} \) represent the z-direction position deviation when the workpiece surface deformation and the top of the pin are just in contact during the compensation process, respectively. Then,the relationship between compensation \( u \) and deviation \( e \) can be established.

<!-- Media -->

<!-- figureText: 0 time time \( - a \) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_5.jpg?x=1035&y=1947&w=469&h=197&r=0"/>

Fig. 8. Basic types of acceleration profiles in velocity planning mode.

<!-- Meanless: 6-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

<!-- figureText: Work Just touching the surface Actual pose with \( u = 0 \) \( {k}_{3} \) /////////////////////////// piece MIMININ -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_6.jpg?x=146&y=157&w=662&h=435&r=0"/>

Fig. 9. Analysis of upsetting deformation during the pin penetration.

<!-- Media -->

According to practical experience: \( {a}_{1} \leq  {a}_{2} \) . The following constraints can be obtained from Fig. 9 as follows:

\[\left\{  \begin{array}{l} \sigma  = r - {a}_{2} + u \\  e = r + {a}_{1} - {a}_{2} \\  {k}_{1}\sigma  = {k}_{2}{a}_{1} \\  {k}_{1}\sigma  = {k}_{3}\left( {-{a}_{1} + {a}_{2}}\right)  \end{array}\right.  \tag{16}\]

From Eq. (16), we get

\[e =  - \frac{{k}_{1}{k}_{2}}{{k}_{1}{k}_{2} + {k}_{1}{k}_{3} + {k}_{2}{k}_{3}}u + \frac{{k}_{1}{k}_{3} + {k}_{2}{k}_{3}}{{k}_{1}{k}_{2} + {k}_{1}{k}_{3} + {k}_{2}{k}_{3}}r = {Ku} + \left( {1 + K}\right) r \tag{17}\]

where \( K \) represents the instantaneous equivalent stiffness of the welding system,

\[K =  - \frac{1}{1 + {k}_{3}/{k}_{2} + {k}_{3}/{k}_{1}}.\]

Considering that \( {k}_{3} \ll  {k}_{1} \) and \( {k}_{3} \ll  {k}_{2} \) in the actual system,we get -1 \( < K <  - 1/3 \) .

The upsetting deviation can be regarded as a function of theoretical compensation. Therefore, the deviation compensation prediction problem can be transformed into finding a root of Eq. (16) which is a time-varying unknown equation. Note that the collection of function values is time-series (function values can be regarded as time series), so it is not suitable to find the root by dichotomy. Fig. 10 shows the distribution relationship between upsetting deviation along the welding direction and actual compensation. In the welding direction, each welding position has a function curve \( {\xi }_{1} \) of deformation and compensation,and these curves together constitute a deformed surface \( {\Psi }_{1} \) . Since there are different deforming and expected compensation values at different times,there is a deformation compensation curve \( {\xi }_{2} \) on the surface \( {\Psi }_{1} \) that varies with the welding position of the end.

Then,project the compensation curve \( {\xi }_{2} \) on the \( u - e \) plane to obtain Fig. 11.

<!-- Media -->

<!-- figureText: \( e\left( \mathrm{\;{mm}}\right) \) \( u\left( \mathrm{{mm}}\right) \) \( x \) (mm) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_6.jpg?x=952&y=151&w=638&h=463&r=0"/>

Fig. 10. Relationship between compensation \( u \) along feed displacement \( x \) and upsetting direction deviation \( e \) .

<!-- figureText: Actual compensation \( u\left( \mathrm{\;{mm}}\right) \) \( {u}_{k + 1} \) Target compensation \( {u}_{k + 1}^{ * }\left( \mathrm{{mm}}\right) \) \( {u}_{k} \) \( e\left( \mathrm{{mm}}\right) \) \( {e}_{k} \) 0 -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_6.jpg?x=986&y=737&w=567&h=410&r=0"/>

Fig. 11. Relationship between trajectory deviation \( e \) and compensation prediction. The upper horizontal axis represents actual compensation \( u \) ,and the lower horizontal axis represents predicted compensation \( {u}^{ * } \) .

<!-- Media -->

The actual compensation and measurement deviation \( e\left( u\right) \) have the following features:

(1) \( e\left( u\right) \) must have zero point,that is,with the increase of feed-in z-axis direction, there must be a time when the deviation in the z-axis direction is 0 ;

(2) \( e\left( u\right) \) is time-sequential. Only the coordinates of discrete points in the sampling period can be obtained, so its global state cannot be observed;

(3) \( e\left( u\right) \) is non-incremental,that is,the compensation process is always effective for deviation correction, which can be expressed as \( \partial {e}_{k}/\partial {u}_{k} \in  \left( {-1,0}\right) \) . Referring to Eq. (17),due to the influence of elastic deformation, the compensation at one position must be greater than or equal to the reduction. However, this phenomenon only applies to the same location. This feature holds, considering there will be no sudden change of stiffness of the robot-workpiece system in the dynamic welding process.

Based on the above features, the predicted compensation can be calculated with Newton's method as follows:

\[{\mathrm{u}}_{\mathrm{k} + 1}^{ * } = {\mathrm{u}}_{\mathrm{k}} + \lambda {\mathrm{e}}_{\mathrm{k}},\]

\[\lambda  = \left\{  \begin{matrix} 2 & \partial {\mathrm{u}}_{\mathrm{k}}/\partial {\mathrm{e}}_{\mathrm{k}} \leq   - 2 \\   - \partial {\mathrm{u}}_{\mathrm{k}}/\partial {\mathrm{e}}_{\mathrm{k}} & \text{ else } \end{matrix}\right.  \tag{18}\]

This section proposes an online S-curve error compensator that simultaneously satisfies velocity, acceleration, and jerk constraints based on the previous work. The framework realizes the online error detection and provides input constraint control to avoid the overshoot of the compensation state in the subsequent constant distance control, which leads to the oscillation of the welding process. Finally, from the compensation prediction for the input of online trajectory generator estimation, an appropriate aim state can be generated for the online error compensator. Specially, we treat the OTG problem as solving the initial jerk problem in each control period. In this way, the whole OTG problem can be solved from the velocity planning mode.

<!-- Meanless: 7-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

## 4. Simulation and experimental verification

### 4.1. Tracking performance of the S-curve compensator

In order to verify the tracking performance of the proposed online planning algorithm, we first track a segment of a sinusoidal reference trajectory, which equation is as follows:

\[y = {0.5}\sin \left( {1.5t}\right)  + \cos \left( {0.5t}\right) . \tag{19}\]

The tracking constraint parameters are as follows:

\[\bar{\zeta } = {\left\lbrack  2\mathrm{{mm}}/\mathrm{s},{10}\mathrm{\;{mm}}/{\mathrm{s}}^{2},{50}\mathrm{\;{mm}}/{\mathrm{s}}^{3}\right\rbrack  }^{\mathrm{T}}.\]

Then, to verify the tracking ability of the proposed method for discrete signals,the discrete reference time series \( t = {2i},\left( {i = 0,1,2\ldots }\right) \) is taken to discretize the above continuous reference trajectory. Fig. 12(a) and (b) show the tracking results of discrete and continuous reference trajectories, respectively. Both figures show that the proposed online planning method can track the reference trajectory under the set constraint parameters. Tracking precision of \( - {0.03} \sim  {0.02}\mathrm{\;{mm}} \) can be achieved when following the dynamic change reference command, and error-free tracking can be achieved when tracking the discrete signal to a stable state. In addition, the subgraphs show that the velocity, acceleration, and jerk are all within the limit, and the purpose of online planning with process constraints is achieved.

<!-- Media -->

<!-- figureText: Reference trajectory Planned trajectory 2 0 -2 0 5 2 0 -2 10 0 5 10 0 -10 10 0 5 10 0 -50 10 5 0 0.02 -0.5 0 -0.02 0 10 (b) Time (s) Position(mm) - 1 5 2 \( {}^{2}/\mathrm{s} \) ) Velocity (mm/s) -2 5 10 1m \( {}^{3}/\mathrm{s} \) ) Acceleration (mm -10 0 50 -50 1 Error(mm) - 1 0 10 (a) Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_7.jpg?x=107&y=1141&w=746&h=977&r=0"/>

Fig. 12. (a) Tracking the discrete trajectory; (b) tracking the continuous trajectory.

<!-- Media -->

#### 4.2.An experiment for constant plunge depth control

A FSW experiment of constant plunge depth control was carried out on the self-developed 5-axis hybrid robot TriMule 800 whose model and structure sketch are shown in Fig. 13 and Fig. 14, respectively. An auxiliary axis (the sixth axis) is mounted at the end of the TriMule 800, which transmission principle has been shown in Fig. 3. The dimension parameters of the robot are shown in Table 1, and the inverse kinematics is analyzed in Appendix A. In order to reflect the process of autonomous tracking and exploration of the welding path, the experiment is designed to independently generate the welding trajectory with constant plunge depth according to the rough reference welding trajectory.

The experiment consists primarily of two stages. First, assuming that the detailed parameters of the welding seam are inaccurate, roughly set the teaching points on the seam to be processed to form a reference path. Then, according to the return distances of the laser ranging sensors with the projection vector method, the actual processing trajectory can be obtained, so that the tool keeps a constant plunge depth. The actual welding track is formed by the projection of the teaching track along the reference press-down direction on the machined surface, as shown in Fig. 15.

The projected vector method can scale the original reference speed command accordingly. So, when the S-shaped path is traced using a straight reference path, it is difficult to ensure that the final welding speed is stable within the required welding speed range due to the need for a large compensation. Therefore, it is recommended to take approximate points as the reference path over the original seam so that the compensated velocity of TCP is within the required range.

The basic process of path tracking is as follows: First, in the case of no compensation, the robot executes the reference welding trajectory by default. Due to the deviation between the reference and the actual welding track, it is necessary to add distance measuring sensors to return the surface topography information about the processing track. The laser ranging sensors measure the distance and attitude of the TCP and the workpiece surface in real time. Then, we correct the original command online using the proposed method (OTG), PI, and fuzzy PID control, respectively. The principle of PI and fuzzy PID parameter determination is that select its parameters under the same control accuracy. The robot tool and the surface of the weldment always maintain a certain process angle. Set the simulation time step as \( {10}\mathrm{\;{ms}} \) . In order to facilitate the observation of the tracking effect, the inclination angle of the tool is selected as \( {90}^{ \circ  } \) . The speed,acceleration,and jerk constraints corresponding to the TCP and attitude Euler angles in the compensation process are set as follows:

<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

\[\bar{\zeta } = {\left\lbrack  5\mathrm{\;{mm}}/\mathrm{s},{50}\mathrm{\;{mm}}/{\mathrm{s}}^{2},{500}\mathrm{\;{mm}}/{\mathrm{s}}^{3}\right\rbrack  }^{T}\]

\[\bar{w} = {\left\lbrack  {0.5}\mathrm{{rad}}/\mathrm{s},{0.5}\mathrm{{rad}}/{\mathrm{s}}^{2},{0.5}\mathrm{{rad}}/{\mathrm{s}}^{3}\right\rbrack  }^{T}\]

<!-- Media -->

<!-- figureText: Trimule800 \( {J}_{5} \) Laser ranging sensors \( {J}_{4} \) Auxiliary axis \( {J}_{6} \) S-shaped workpiece -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_7.jpg?x=927&y=1390&w=685&h=728&r=0"/>

Fig. 13. The hybrid robot TriMule 800 with an auxiliary axis and distance ranging sensors at the end.

<!-- Meanless: 8-->


<!-- figureText: \( b \) , \( {d}_{1} \) \( A \) . \( {d}_{2} \) a \( {d}_{3} \) \( {b}_{3} \) \( \alpha \) \( {b}_{1} \) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_8.jpg?x=107&y=151&w=741&h=387&r=0"/>

Fig. 14. Schematic diagram of TriMule 800 robot mechanism with an auxiliary joint \( {J}_{6} \) ,where \( {J}_{i} \) represent the active joint, \( i = 1,\ldots 6 \) .

Table 1

Mechanism parameters at home position.

<table><tr><td>\( {a}_{1},{a}_{2},{a}_{3} \)</td><td>\( {b}_{1} \)</td><td>\( {b}_{2},{b}_{3} \)</td><td>\( {d}_{1} \)</td><td>\( {d}_{2} \)</td><td>\( {d}_{3} \)</td><td>\( \alpha ,\beta \)</td></tr><tr><td>220 mm</td><td>850 mm</td><td>450 mm</td><td>1028 mm</td><td>575 mm</td><td>444.18 mm</td><td>\( \pi /2\mathrm{{rad}} \)</td></tr></table>

<!-- Media -->

The results are presented in Figs. 16-19, where Fig. 16(a)-(c) reflects the upsetting distance collected in real-time by three ranging sensors with three different methods (OTG, PI, and Fuzzy PID), where 'S1' represents the distance collected by the first sensor, and the others are the same. The figure shows that the distances gathered by the three laser ranging sensors are relatively close for all autonomous tracking methods with constant plunge depth. Thus, all three methods allow to obtain precise welding inclination angles. Then, we compare the error performance of the proposed OTG method with other different methods in Fig. 16(d), and the Mean Absolute Errors (MAEs) of the three methods are \( {0.0237}\mathrm{\;{mm}} \) (OTG), \( {0.0275}\mathrm{\;{mm}} \) (PI) and \( {0.0255}\mathrm{\;{mm}} \) (Fuzzy PID), respectively. It is concluded that the MAE of the OTG method is slightly smaller than that of the other two methods. Fig. 18 shows the TCP speed generated by a variety of tracking methods, which reflects that the TCP speeds of these three methods are similar and fluctuate around \( 5\mathrm{\;{mm}}/\mathrm{s} \) . Considering the requirement that the distance between the end and the surface of the workpiece is constant, all methods can obtain the indentation deviation errors of \( - {0.05} \sim  {0.05}\mathrm{\;{mm}} \) ,which meets the requirement of \( {0.2}\mathrm{\;{mm}} \) . If a more approximate welding path can be given in the initial stage, only a small error has to be compensated. As a result, the path tracking accuracy can be greater.

<!-- Media -->

reference path.

<!-- figureText: Ref path Joint (a) RFSW simulation with an S-shaped workpiece Ref path Seam 2200 Z (mm) 2150 100 -100 X (mm) 260 220 -200 Y (mm) (b) Generate seam from the reference path -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_8.jpg?x=112&y=1596&w=1526&h=522&r=0"/>

Fig. 15. Explore an unknown surface welding path with the projection vector method. (a) RFSW simulation with an S-shaped workpiece, (b) Generate seam from the

<!-- Media -->

The vibration index of the robot planned trajectory can be accurately reflected in both the time and frequency domains. In the time domain, the higher-order derivative bounded trajectory is smoother; in the frequency domain, the acceleration amplitude of the high frequency is closer to zero can prevent the excitation of the robot natural vibration [39]. As shown in Fig. 18, the left column of Fig. 18 shows the joint velocity curves. To show more details, the right column of Fig. 18 represents a zoomed-in view of the period of the joint velocity. Fig. 19 shows the magnitude-frequency characteristics of the acceleration of the joints. It can be concluded that compared with the PI and fuzzy PID control, the proposed method has a smaller fluctuation frequency and a higher response amplitude in each joint around \( 2\mathrm{\;{Hz}} \) ,while in the frequency range greater than \( 5\mathrm{\;{Hz}} \) ,the proposed method has a relatively small vibration amplitude. This is related to the compensation constraint parameters. The vibration amplitude is significantly reduced and has a tendency of continuous attenuation when frequency increase, whereas the frequency response amplitudes of the other two methods do not have a decreasing trend with frequency. The proposed method effectively avoids the resonant frequency of the robot and reduces the natural vibration due to compensation. The above simulation demonstrates the accuracy of the proposed online bias compensation strategy.

To further verify the effectiveness of the proposed strategy, an FSW experiment was carried out on the TriMule 800 robot welding platform based on the simulation. The welding experiment used an S-shaped plate of 6061-T6 aluminum alloy, which parameters were chosen as follows: pin diameter: \( 5\mathrm{\;{mm}} \) ; welding speed: \( {80}\mathrm{\;{mm}}/\mathrm{{min}} \) ; spindle speed: \( {1000}\mathrm{r}/ \) min; plunge depth: \( {0.1}\mathrm{\;{mm}} \) ,and the tilt angle: \( {2}^{ \circ  } \) . The experimental equipment is shown in Fig. 20, where the robot payload is 3 tons. First, a welding comparison experiment was carried out without and with the compensation method respectively. The results are shown in Fig. 21, where Fig. 21(a) shows the welding result with no compensation, the partially enlarged image indicates that the rear section of the joint is not formed, which results in serious welding defects. Thus, the uncompensated welding trajectory cannot meet the plunge depth requirements. Fig. 21(b) shows the specific welding effect with the OTG and PI control methods. The upper and the lower welded joints are formed by PI control and OTG method respectively. The partially enlarged image shows that the PI method has obvious burrs due to vibration caused by unsmooth compensation. The joint produced by the proposed OTG method has a good welding effect.

<!-- Meanless: 9-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

<!-- Media -->

<!-- figureText: (a) OTG (c) Fuzzy PID 0.05 Errs (mm) -0.05 0 40 60 80 100 0.08 (d) OTG Absolute Errs (mm) PI 0.06 Fuzzy PID 0.04 0.02 0 40 60 80 100 Time (s) 0.05 Errs (mm) -0.05 S1 S2 S3 0 60 80 100 (b) PI 0.05 Errs (mm) -0.05 0 20 40 60 80 100 Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_9.jpg?x=342&y=147&w=1064&h=738&r=0"/>

Fig. 16. The distance errors collected by the displacement sensors.

<!-- figureText: 6 OTG – PI Fuzzy PID 70.5 71 71.5 72 50 60 70 80 90 100 Time (s) 5 \( {V}_{tcp}\left( {\mathrm{{mm}}/\mathrm{s}}\right) \) 3 5.2 5 2 4.8 4.6 1 69 69.5 70 0 0 10 20 30 40 -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_9.jpg?x=95&y=983&w=1546&h=627&r=0"/>

Fig. 17. TCP speed generated by different methods.

<!-- Media -->

As shown in Fig. 21(c), nine representative points on the welded joint were scanned transversely using a white light interferometer to measure the plunge depth, the measurement results are shown in the Table 2. It shows that the plunge depth of the entire joint is around \( {100} \pm  {50}\mathrm{\;{um}} \) .

In summary, both simulation and experimental results show that, compared with other control methods, the proposed method can not only obtain the ideal plunge depth, but also effectively reduce the vibration caused by the compensation process.

## 5. Conclusions

This paper investigates a constant plunge depth control with an adaptive tracking strategy based on online trajectory generation for RFSW. The conclusions are as follows:

(1) A constant plunge depth control scheme is proposed. In the deviation detection part, a terminal distance with an attitude detection scheme based on three laser ranging sensors is adopted. On this basis, based on the projection vector method, the constant indentation welding trajectory is automatically generated according to the rough interpolation trajectory points.

(2) To reduce the vibration generated by the robot compensation process. An online trajectory generation framework that satisfies the compensation command velocity, acceleration, and jerk constraints is proposed. The framework can generate smooth position compensation commands in real time according to the start and end states of any compensation commands. Finally, the deviation compensation control problem is regarded as the compensation prediction problem, and the prediction compensation is given.

<!-- Meanless: 10-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

<!-- Media -->

<!-- figureText: OTG PI Fuzzy PID 0 -5 -10 62 62.5 63 63.5 64 0 -5 -10 -15 62 62.5 63 63.5 64 0 -2 62 62.5 63 63.5 64 -1 62 62.5 63 63.5 64 WALLATAN 62 62.5 63 63.5 64 Time (s) 20 \( {V}_{j1}\mathrm{\;{mm}}/\mathrm{s} \) 0 -20 20 40 60 80 100 20 \( {V}_{j2}\mathrm{\;{mm}}/\mathrm{s} \) 0 -20 20 40 60 80 100 20 \( {V}_{j3}\mathrm{\;{mm}}/\mathrm{s} \) 0 -20 20 40 60 80 100 \( {V}_{j4}\mathrm{{deg}}/\mathrm{s} \) 20 0 -20 20 40 60 80 100 5 \( {V}_{j5}\mathrm{\;{deg}}/\mathrm{s} \) 0 -5 20 40 60 80 100 Time (s) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_10.jpg?x=182&y=137&w=1388&h=1871&r=0"/>

Fig. 18. The joint velocities of TriMule 800 during autonomous execution constant plunge depth task.

<!-- Meanless: 11-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

<!-- figureText: 1.5 1.5 \( \left| {{P}_{j2}\left( f\right) }\right| \) 1 10 50 1.5 \( \left| {{P}_{j4}\left( f\right) }\right| \) 1 0.5 10 20 30 40 50 \( f\left( \mathrm{{Hz}}\right) \) OTG Fuzzy PID \( \left| {{P}_{j1}\left( f\right) }\right| \) 1 0.5 10 20 40 50 1.5 \( \left| {{P}_{j3}\left( f\right) }\right| \) 0.5 10 20 30 40 50 0.3 \( \left| {{P}_{j5}\left( f\right) }\right| \) 0.2 0.1 10 20 30 50 \( f\left( \mathrm{{Hz}}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_11.jpg?x=337&y=146&w=1073&h=913&r=0"/>

Fig. 19. Magnitude-frequency characteristics of robot joint acceleration.

<!-- figureText: TriMule 800 laser ranging sensor S-shaped specimen 元 -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_11.jpg?x=117&y=1153&w=723&h=537&r=0"/>

Fig. 20. The experimental equipment - Trimule 800 with a welded S-shaped specimen.

<!-- Media -->

(3) The simulation and experiment of the robot tracking the inaccurate welding trajectory are carried out. By comparing with the PI and fuzzy PID control, the results show that the proposed method can generate a welding trajectory with a downward pressure error of \( \pm  {0.05}\mathrm{\;{mm}} \) that meets the process requirements under the condition of satisfying the motion constraints. The proposed strategy can effectively reduce the vibration of the robot and avoid the flashing of the joint after welding. Both simulation and experiment have proved the effectiveness.

<!-- Media -->

<!-- figureText: Without compensation PI OTG 6 8 9 (a) PI OTG (b) OTG 2 3 (c) -->

<img src="https://cdn.noedgeai.com/bo_d2slafbef24c73b2kfgg_11.jpg?x=900&y=1154&w=736&h=813&r=0"/>

Fig. 21. Comparison of the welding effect of S-shaped specimens under different control methods.

<!-- Media -->

Since the actual position of the tool is the non-contact indirect measurement, this work assumes that the plane formed by the three measuring points is the measurement surface, and the intersection of this surface and the axis of the tool is used as the action point. Measurements on surfaces with greater curvature will experience large deviation. Therefore, this surface measurement problem can be further discussed to improve the control accuracy.

<!-- Meanless: 12-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479-->

<!-- Media -->

Table 2

Measurement of post-weld plunge in depth of welded specimens (reference depth: 100um).

<table><tr><td>Index</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td><td>8</td><td>9</td></tr><tr><td>Depth(um)</td><td>53.9</td><td>67.2</td><td>102.4</td><td>149.2</td><td>150.6</td><td>143.5</td><td>117.9</td><td>85.5</td><td>58.3</td></tr></table>

<!-- Media -->

## CRediT authorship contribution statement

Juliang Xiao: Conceptualization, Methodology, Resources, Writing

- review & editing, Supervision, Project administration, Funding acquisition. Mingli Wang: Conceptualization, Methodology, Software, Validation, Writing - original draft, Writing - review & editing, Formal analysis. Haitao Liu: Resources, Supervision, Writing - review & editing, Funding acquisition. Sijiang Liu: Validation, Writing - review & editing. Huihui Zhao: Resources. Jiashuang Gao: Resources.

## Declaration of Competing Interest

The authors report no declarations of interest.

## Data availability

Data will be made available on request.

## Acknowledgements

This work is partially supported by National Key R&D Program of China (Grant No. 2019YFA0709004), National Natural Science Foundation of China (grants 52175025 and 91948301).

## Appendix A. Inverse kinematics of TriMule 800

As shown in Fig. 14, \( C \) is the endpoint of the TCP, \( P \) is the intersection of the 4 and 5 joints,and \( B \) is the origin of the base coordinate system. Define the dimension parameter vectors of the TriMule robot mechanism as \( {\mathbf{a}}_{1} = {\left\lbrack  \begin{array}{lll} 0 &  - {a}_{1} & 0 \end{array}\right\rbrack  }^{\mathrm{T}},{\mathbf{a}}_{2} = {\left\lbrack  \begin{array}{lll} {a}_{2} & 0 & 0 \end{array}\right\rbrack  }^{\mathrm{T}},{\mathbf{a}}_{3} = {\left\lbrack  \begin{array}{lll}  - {a}_{3} & 0 & 0 \end{array}\right\rbrack  }^{\mathrm{T}},{\mathbf{b}}_{1} = {\left\lbrack  \begin{array}{lll} 0 &  - {b}_{1} & 0 \end{array}\right\rbrack  }^{\mathrm{T}} \) , \( {\mathbf{b}}_{2} = {\left\lbrack  \begin{array}{lll} {\mathbf{b}}_{2} & 0 & 0 \end{array}\right\rbrack  }^{\mathrm{T}} \) ,and \( {\mathbf{b}}_{3} = {\left\lbrack  \begin{array}{lll}  - {\mathbf{b}}_{3} & 0 & 0 \end{array}\right\rbrack  }^{\mathrm{T}} \) . Then the inverse kinematics of the TriMule 800 robot can be described as calculating each active joint position \( {J}_{i} \) of the robot under the given pose \( {\mathbf{P}}_{w} = {\left\lbrack  \begin{array}{ll} {\mathbf{R}}_{w} & {\mathbf{r}}_{c} \end{array}\right\rbrack  }_{3 \times  4} \) . In this part,we give 6-axis kinematics based on 5-axis kinematics [40].

First,the position vector of \( P \) and its normalization \( {s}_{p} \) can be expressed as

\[{\mathbf{r}}_{p} = {\mathbf{r}}_{c} - {d}_{3}\mathbf{w},{\mathbf{s}}_{p} = \operatorname{norm}\left( {\mathbf{r}}_{p}\right)  \tag{A.1}\]

where \( w \) represents the third column of \( {\mathbf{R}}_{w} \) and norm \( \left( \cdot \right) \) represents the normalized function. Then we get the pitch and yaw angles \( \alpha ,\beta \) of the parallel mechanism

\[\alpha  = \operatorname{atan}\left( {-{\mathbf{s}}_{\mathrm{p}{.2}}/{\mathbf{s}}_{\mathrm{p}{.3}}}\right) ,{\mathbf{s}}_{\mathrm{p}{.3}} \neq  0 \tag{A.2}\]

\[\beta  = \operatorname{asin}\left( {\mathbf{s}}_{\mathrm{p}{.1}}\right) ,\]

where \( {s}_{p,i} \) represents the \( i \) th component of the \( {s}_{p} \) vector. Therefore,the rotate matrix of \( P \) can be expressed in the base coordinate system as

\[{\mathbf{R}}_{p} = \operatorname{Rotx}\left( \alpha \right) \operatorname{Roty}\left( \beta \right)  = \left\lbrack  \begin{matrix} \mathrm{c}\beta & 0 & \mathrm{\;s}\beta \\  \mathrm{s}\alpha \mathrm{s}\beta & \mathrm{c}\alpha &  - \mathrm{s}\alpha \mathrm{c}\beta \\   - \mathrm{c}\alpha \mathrm{s}\beta & \mathrm{s}\alpha & \mathrm{c}\alpha \mathrm{c}\beta  \end{matrix}\right\rbrack   \tag{A.3}\]

where \( \mathrm{s} \) - and \( \mathrm{c} \) - represents the sine and cosine function,respectively.

Then,considering that the 1,2,and 3 motion branches form a closed loop with the middle branch,the joint vectors \( {q}_{i} \) and positions \( {J}_{i}\left( {i = 1,2,3}\right) \) can be solved.

\[{\mathbf{q}}_{\mathrm{i}} = {\mathbf{r}}_{\mathrm{p}} - {\mathbf{b}}_{\mathrm{i}} - {\mathrm{d}}_{2}{\mathbf{s}}_{\mathrm{p}} + {\mathbf{R}}_{\mathrm{p}}{\mathbf{a}}_{\mathrm{i}}\]

\[{\mathbf{J}}_{\mathrm{i}} = \left| {\mathbf{q}}_{\mathrm{i}}\right|  - {\mathbf{l}}_{\mathrm{i}},\mathrm{i} = 1,2,3. \tag{A.4}\]

where \( {l}_{i} \) is the original length of the \( i \) th joint when the robot is in the home position.

With the above analysis,the 4,5 and 6 joint positions can be solved below. Let \( n = \operatorname{norm}\left( {r}_{c}\right) \) ,then we get \( u = w \times  n,v = w \times  u \) . If \( {r}_{p}^{T}v < 0 \) ,then set \( v = \) \( - \gamma \) . We take \( {\mathbf{R}}_{c} \) as the pose of the robot end point \( C \) ,then \( {\mathbf{R}}_{cp} \) which is the orientation of the \( \mathrm{C} \) frame with respect to the \( P \) can be expressed by

\[{\mathbf{R}}_{c} = \left\lbrack  \begin{array}{lll} \mathbf{u} & \mathbf{v} & \mathbf{w} \end{array}\right\rbrack  ,{\mathbf{R}}_{cp} = \left\lbrack  \begin{array}{lll} {\mathbf{u}}_{cp} & {\mathbf{v}}_{cp} & {\mathbf{w}}_{cp} \end{array}\right\rbrack   = {\mathbf{R}}_{p}^{-1}{\mathbf{R}}_{c}. \tag{A.5}\]

Finally, with the above calculation, the 4 and 5th joint position can be calculated as follows

\[{\mathbf{J}}_{5} = \operatorname{atan}2\left( {{\mathbf{v}}_{cp.3},{\mathbf{w}}_{cp.3}}\right) \]

\[{\mathbf{J}}_{4} = \left\{  \begin{matrix}  - \operatorname{atan}2\left( {-{\mathbf{v}}_{{cp},1}/{\mathbf{w}}_{{cp},3},{\mathbf{v}}_{{cp},2}/{\mathbf{w}}_{{cp},3}}\right) & \left| {\mathbf{J}}_{5}\right|  < 1\mathrm{e} - 9 \\   - \operatorname{atan}2\left( {{\mathbf{w}}_{{cp},1}/{\mathbf{v}}_{{cp},3}, - {\mathbf{w}}_{{cp},2}/{\mathbf{v}}_{{cp},3}}\right) & \text{ otherwise } \end{matrix}\right.  \tag{A.6}\]

<!-- Meanless: 13-->




<!-- Meanless: J. Xiao et al. Robotics and Computer-Integrated Manufacturing 80 (2023) 102479 References-->

where \( {v}_{cpl} \) represents the \( i \) th value of the \( {v}_{cp} \) . The others are the same. Finally,the position of the auxiliary axis can be calculated according to the deviation between the pose of the 5 axis and the target point.

\[{\mathbf{R}}_{wc} = \left\lbrack  \begin{array}{lll} {\mathbf{u}}_{wc} & {\mathbf{v}}_{wc} & {\mathbf{w}}_{wc} \end{array}\right\rbrack   = {\mathbf{R}}_{\mathrm{c}}^{-1}{\mathbf{R}}_{\mathrm{w}}\]

(A.7)incremental sheet forming, Robot. Comput. Integr. Manuf. 29 (4) (2013) 58-69, https://doi.org/10.1016/j.rcim.2012.10.008.

[20] U. Schneider, M. Drust, M. Ansaloni, C. Lehmann, M. Pellicciari, F. Leali, et al., Improving robotic machining accuracy through experimental error investigation and modular compensation, Int. J. Adv. Manuf. Technol. 85 (1-4) (2016) 3-15, https://doi.org/10.1007/s00170-014-6021-2.

[21] J.D. Backer, G. Bolmsjö, Detection model for robotic friction stir welding, Ind. Robot 41 (4) (2014) 365-372, https://doi.org/10.1108/IR-01-2014-0301.

[22] S. Zimmer, L. Langlois, J. Laye, R. Bigot, Experimental investigation of the influence of the FSW plunge processing parameters on the maximum generated force and torque, Int. J. Adv. Manuf. Technol. 47 (1-4) (2010) 201-215, https:// doi.org/10.1007/s00170-009-2188-3.

[23] S. Zargarbashi, W. Khan, J. Angeles, The Jacobian condition number as a dexterity index in 6R machining robots, Robot. Comput. Integr. Manuf. 28 (6) (2012) 694-699, https://doi.org/10.1016/j.rcim.2012.04.004.

[24] M Soualhi, K.T.P. Nguyen, et al., Intelligent monitoring of multi-axis robots for online diagnostics of unknown arm deviations, J. Intell. Manuf. (2022), https:// doi.org/10.1007/s10845-021-01882-0.

[25] J. Qin, F. Léonard, G. Abba, Real-time trajectory compensation in robotic friction stir welding using state estimators, in: Proceedings of the IEEE Transactions on Control Systems Technology 24, 2016, pp. 2207-2214, https://doi.org/10.1109/ TCST.2016.2536482. A publication of the IEEE Control Systems Society.

[26] Z.A. Jing, D. Yaxing, X. Biyun, Z. Ziqiang, FSW robot system dimensional optimization and trajectory planning based on soft stiffness indices, J. Manuf. Process. 63 (2021) 88-97, https://doi.org/10.1016/j.jmapro.2020.05.004.

[27] A. Rout, B.B.V.L. Deepak, B.B. Biswal, Advances in weld seam tracking techniques for robotic welding: a review, Robot. Comput. Integr. Manuf. 56 (2019) 12-37.

[28] D. Wertjanz, E. Csencsics, G. Schitter, Three-DoF vibration compensation platform for robot-based precision inline measurements on free-form surfaces, IEEE Trans. Ind. Electron. 69 (1) (2021) 613-621, https://doi.org/10.1109/ TIE.2021.3055132.

[29] A. Rui, U. Nunes, A. Almeida, 3D surface-tracking with a robot manipulator, J. Intell. Robot. Syst. 15 (4) (1996) 401-417, https://doi.org/10.1007/ BF00437604.

[30] M. Amersdorfer, J. Kappey, T. Meurer, Real-time freeform surface and path tracking for force controlled robotic tooling applications, Robot. Comput. Integr. Manuf. 65 (2020), 101955, https://doi.org/10.1016/j.rcim.2020.101955.

[31] J.E. Bobrow, S. Dubowsky, J.S. Gibson, Time-optimal control of robotic manipulators along specified paths, Int. J. Robot. Res. 4 (3) (1985) 3-17, https:// doi.org/10.1177/027836498500400301.

[32] Q. Pham, A general, fast, and robust implementation of the time-optimal path parameterization algorithm, IEEE Trans. Robot. 30 (6) (2013) 1533-1540, https:// doi.org/10.1109/TRO.2014.2351113.

[33] S. Liu, An on-line reference-trajectory generator for smooth motion of impulse-controlled industrial manipulators, in: Proceedings of the International Work shop on Advanced Motion Control, Maribor, Slovenia, 2002, pp. 365-370, https://doi.org/10.1109/AMC.2002.1026947.

[34] T. Kröger, F.M. Wahl, Online trajectory generation: basic concepts for instantaneous reactions to unforeseen events, IEEE Trans. Robot. 26 (1) (2009) 94-111, https://doi.org/10.1109/TRO.2009.2035744.

[35] L. Berscheid, T. Kröger, Jerk-limited real-time trajectory generation with arbitrary target states, Robot. Sci. Syst. 2021 (2021), https://doi.org/10.15607/RSS.2021.XVII.015.

[36] L. Biagiotti, C. Melchiorri, FIR filters for online trajectory planning with time- and frequency-domain specifications, Control Eng. Pract. 20 (12) (2012) 1385-1399, https://doi.org/10.1016/j.conengprac.2012.08.005.

[37] P. Besset, R. Bearee, O. Gibaru, FIR filter-based online jerk-controlled trajectory generation, in: Proceedings of the IEEE International Conference on Industrial Technology, Taipei, Taiwan, 2017, https://doi.org/10.1109/ICIT.2016.7474730.

[38] M. Wang, J. Xiao, F. Zeng, G. Wang, Research on optimized time-synchronous online trajectory generation method for a robot arm, Robot. Auton. Syst. 126 (2020), 103453, https://doi.org/10.1016/j.robot.2020.103453.

[39] L. Biagiotti, C. Melchiorri, Trajectory Planning For Automatic Machines and Robots, Springer, Berlin Heidelberg, 2009.

[40] Q. Liu, T. Huang, Inverse kinematics of a 5-axis hybrid robot with non-singular tool path generation, Robot. Comput. Integr. Manuf. 56 (2019) 140-148.

\[{\mathbf{J}}_{6} = \operatorname{atan}\left( {{\mathbf{u}}_{{wc},2}/{\mathbf{u}}_{{wc},1}}\right) .\]

[1] Thomas, W., Nicholas, E., Needham, J., Murch, M., Temple-Smith, P., Dawes, C., Friction stir butt welding, international patent application no. PCT/GB92 Patent application 1991, (9125978.8).

[2] N. Mendes, P. Neto, A. Loureiro, A.P. Moreira, Machines and control systems for friction stir welding: a review, Mater. Des. 90 (2016) 256-265, https://doi.org/ 10.1016/j.matdes.2015.10.124.

[3] J.D. Backer, A.K. Christiansson, J. Oqueka, G. Bolmsj, Investigation of path compensation methods for robotic friction stir welding, Ind. Robot 39 (6) (2012) 601-608, https://doi.org/10.1108/01439911211268813.

[4] S. Meng, H. Liu, J. Xiao, et al., A method for process parameter optimization of simultaneous double-sided friction stir welding using a heat transfer model, Int. J. Adv. Manuf. Technol. 121 (2022) 3747-3758.

[5] F. Leali, A. Vergnano, F. Pini, M. Pellicciari, G. Berselli, A workcell calibration method for enhancing accuracy in robot machining of aerospace parts, Int. J. Adv. Manuf. Technol. 85 (1-4) (2016) 47-55, https://doi.org/10.1007/s00170-014- 6025-y.

[6] R. Qi, W. Zhou, H. Zhang, W. Zhang, G. Yang, Trace generation of friction stir welding robot for space weld joint on large thin-walled parts, Ind. Robot 43 (6) (2016) 617-627, https://doi.org/10.1108/ir-04-2015-0075.

[7] M. Guillo, L. Dubourg, Dual-loop control force/position with secondary encoders: impact & improvement of industrial robot deviation on FSW quality, in: Proceedings of the 11th International Symposium on Friction Stir Welding, Cambridge, UK, 2016.

[8] L. Shi, C. Wu, H. Liu, The effect of the welding parameters and tool size on the thermal process and tool torque in reverse dual-rotation friction stir welding, Int. J. Mach. Tools Manuf. 91 (2015) 1-11, https://doi.org/10.1016/j.ijmachtools.2015.01.004.

[9] M. Guillo, L. Dubourg, Impact & improvement of tool deviation in friction stir welding: weld quality & real-time compensation on an industrial robot, Robot. Comput. Integr. Manuf. 39 (2016) 22-31, https://doi.org/10.1016/j.rcim.2015.11.001.

[10] Y. Sun, J. Xiao, H. Liu, T. Huang, G. Wang, Deformation prediction based on an adaptive GA-BPNN and the online compensation of a 5-DOF hybrid robot, Ind. Robot 47 (6) (2020) 915-928, https://doi.org/10.1108/IR-01-2020-0016.

[11] M. Soron, I. Kalaykov, A robot prototype for friction stir welding, in: Proceedings of the IEEE Conference on Robotics, Automation & Mechatronics, Bangkok, Thailand, 2006, https://doi.org/10.1109/RAMECH.2006.252646.

[12] E.F. Shultz, A. Fehrenbacher, F.E. Pfeerkorn, M.R. Zinn, N.J. Ferrier, Shared control of robotic friction stir welding in the presence of imperfect joint fit-up, J. Manuf. Process. 15 (1) (2013) 25-33, https://doi.org/10.1016/j.jmapro.2012.07.002.

[13] W.R. Longhurst, A.M. Strauss, G.E. Cook, C.D. Cox, C.E. Hendricks, B.T. Gibson, et al., Investigation of force-controlled friction stir welding for manufacturing and automation, Proc. Inst. Mech. Eng. B J. Eng. 224 (6) (2010) 937-949, https://doi.org/10.1243/09544054JEM1709.

[14] W.R. Longhurst, A.M. Strauss, G.E. Cook, P.A. Fleming, Torque control of friction stir welding for manufacturing and automation, Int. J. Adv. Manuf. Technol. 51 (9-12) (2010) 905-913, https://doi.org/10.1007/s00170-010-2678-3.

[15] T.A. Davis, P.D. Ngo, Y.C. Shin, Multi-level fuzzy control of friction stir welding power, Int. J. Adv. Manuf. Technol. 59 (5-8) (2012) 559-567, https://doi.org/ 10.1007/s00170-011-3522-0.

[16] Q. Wang, Z. Ge, C. Wu, F. Ren, L. Guo, Experimental investigations of axial force control for friction stir welding based on multi-level fuzzy control, in: Proceedings of the International Conference on Control, Automation and Robotics (ICCAR), Beijing, China, 2019, https://doi.org/10.1109/ICCAR.2019.8813466.

[17] L. Han, S. Qingyu, L. Qu, C. Xiong, Z. Yucan, Development of pressure control system for friction stir welding and experiment in simulated non-rigid environment, J. Mech. Eng. 51 (22) (2015) 60-65.

[18] L. Kong, G. Chen, H. Wang, G. Huang, D. Zhang, Kinematic calibration of a 3-PRRU parallel manipulator based on the complete, minimal and continuous error model, Robot. Comput. Integr. Manuf. 71 (2021), 102158, https://doi.org/10.1016/j.rcim.2021.102158.

[19] J. Belchior, M. Guillo, E. Courteille, P. Maurine, L. Leotoing, D. Guines, Off-line compensation of the tool path deviations on robotic machining: application to

<!-- Meanless: 14-->

