

<!-- Meanless: Robotics and Computer-Integrated Manufacturing 85 (2024) 102621 Contents lists available at ScienceDirect Robotics and Computer-Integrated Manufacturing ELSEVIER journal homepage: www.elsevier.com/locate/rcim updates-->

# Advanced adaptive feed control for CNC machining

S.G. Kim \( {}^{a} \) , E.Y. Heo \( {}^{a} \) , H.G. Lee \( {}^{a} \) , D.W. Kim \( {}^{b, * } \) , N.H. Yoo \( {}^{c} \) , T.H. Kim \( {}^{d} \)

\( {}^{a} \) EDIM Inc. 67 Yusang-ro,Deokjin-gu,Jeonju,54852,Republic of Korea

\( {}^{b} \) Department of Industrial and Information Systems Engineering,Jeonbuk National University,Jeonju,54896,Republic of Korea

\( {}^{c} \) School of Computer Engineering,Kyungnam University,Changwon,51767,Republic of Korea

\( {}^{d} \) Department of Aeronautical Mechanical Engineering,Aviation Campus of Korea Polytechnic,Sacheon,52549,Republic of Korea

## ARTICLEINFO

Keywords:

Computer numerical control (CNC) machining

Tool feed rate

Cutting load

Reference load control curve (RLCC)

Advanced adaptive feed control

## A B S T R A C T

In computer numerical control (CNC) machining, the tool feed rate is crucial for determining the machining time. It also affects the degree of tool wear and the final product quality. In a mass production line, the feed rate guides the production cycle. On the other hand, in single-time machining, such as for molds and dies, the tool wear and product quality are influenced by the length of machining time. Accordingly, optimizing the CNC program in terms of the feed rate is critical and should account for various factors, such as the cutting depth, width, spindle speed, and cutting oil. Determining the optimal tool feed rate, however, can be challenging given the various machine tools, machining paths, and cutting conditions involved. It is important to balance the machining load by equalizing the tool's load, reducing the machining time during no-load segments, and controlling the feed rate during high load segments. In this study, an advanced adaptive control method was designed that adjusts the tool feed rate in real time during rough machining. By predicting both the current and future machining load based on the tool position and time stamp, the proposed method combines reference load control curves and cutting characteristics, unlike existing passive adaptive control methods. Four different feed control methods were tested including conventional and proposed adaptive feed control. The results of the comparative analysis was presented with respect to the average machining load and tool wear, the machining time, and the average tool feed speed. When the proposed adaptive control method was used, the production time was reduced up to 12.8% in the test machining while the tool life was increased.

## 1. Introduction

In computer numerical control (CNC) machining, optimization of NC data is crucial for ensuring high-quality machining and reducing the machining time. When generating NC data, the cutting depth and width account for the material removal rate, cutting force, and tool engagement with the material. The feed rate is determined by considering the cutting speed, cutting force of the spindle, and the characteristics of the material. Thus, NC data optimization is conducted with consideration of the features of the parts, the machining paths, and the machining conditions before machining, as well as the machining load, tool life, and machining duration.

Lazoglu et al. [1] optimized the machining path by predicting the machining load for each position of the tool and the machining path in free-form surface machining. Wang et al. [2] optimized the two-dimensional machining path to ensure uniform contact between the tool and material. Meanwhile, cutting load prediction methods have been utilized to optimize NC data. Melkote et al. [3] conducted studies on simulation modeling based on cutting physics and phenomenology in metal cutting, and they experimentally verified the simulation results. Azvar et al. [4] predicted the chip shape and cutting force through a simulation to optimize the cutting force in the gear hobbing process. Berger et al. [5] optimized the chip thickness through a simulation to minimize chip formation and flow in titanium alloy machining, which can cause an uneven cutting force and tool vibration. Lee et al. [6] considered the cross-directional information of the feed direction in machining, including surface quality, construction of the characteristic curve of the tool path, and calculation of the optimal feed rate. This was done to ensure a consistent cutting pattern without defects on the product surface in the cross-direction. Xie et al. [7] developed a power data-based model of the spindle in 3-axis rough milling and optimized the feed rate by applying Artificial Neural Network (ANN) to the spindle power. Budak et al. [8] reduced machining time by optimally scheduling the feed rate based on the predicted cutting force for sculpted surface machining. Erkorkmaz et al. [9] predicted the contact engagement and cutting force between the tool and material through a simulation in free-form machining. They calculated the feed rate for each cutting segment of the free form with consideration of the cutting force limit and control-generated jerk. Rattunde et al. [10] used a Gaussian method to schedule and optimize the feed rate based on spindle power to improve the metal removal rate in milling. They verified their results through a machining experiment on gray cast iron. Liu et al. [11] optimized the feed rate by considering the machine tool geometry and kinematics. Through mathematical modeling of material-tool engagement, the maximum feed rate at various tool positions was derived, and the nonlinear feed rate was transformed into a linear model to produce a smooth spline curve for the feed rate. Ni et al. [12] analysed the characteristics of the S-curve acceleration/deceleration algorithm by setting the jerk of each cutting segment as a variable. They scheduled the feed rate by adjusting the interpolation time and maximum feed rate accordingly. Petráček et al. [13] proposed a linear programming method for the feed rate profile to optimize the machining time, sampling of the curved path, and response time for feed rate override control. Li et al. [14] presented a feed rate scheduling method that increases machining efficiency by dividing the entire curve into several blocks, taking into account the chord error and kinematic error, and generating feed rate profiles for these blocks. Liang et al. [15] optimized the feed rate in the B-spline path by optimizing control points and knot vectors using a genetic algorithm with a turret-type machine. That is, the whole B-spline path based machining area was divided, and the feed rate that minimized the cutting time in the divided area was proposed.

---

<!-- Footnote -->

* Corresponding author.

E-mail address: dwkim@jbnu.ac.kr (D.W. Kim).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.rcim.2023.102621 Received 10 July 2023; Accepted 10 July 2023 Available online 18 July 2023 0736-5845/(© 2023 Elsevier Ltd. All rights reserved.-->




<!-- Meanless: S.G. Kim et al. Robotics and Computer-Integrated Manufacturing 85 (2024) 102621-->

<!-- Media -->

<!-- figureText: actual Feed; Command Feed Upper [Load] Lower [time] -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_1.jpg?x=142&y=152&w=669&h=355&r=0"/>

Fig. 1. Basic concept of adaptive control [21] (F: feed rate).

<!-- Media -->

Meanwhile, some researchers have conducted studies on feed rate scheduling for robotic machining. Xiong et al. [16] proposed a method of offline feed rate optimization for robot milling. The method takes into consideration the cutting force model and part shape and adjusts the feed rate online based on the cutting force. Xiao et al. [17] developed a heuristic-based method of feed rate calculation for 5-axis feed rate scheduling in 5-DOF hybrid robot milling. This method considers feed time and jerk, and further optimizes the control points of the feed rate profile expressed by the parametric analysis formula and the B-spline curve. Li et al. [18] considered the C-axis angle compensation, motor drive, and geometrical accuracy in milling using a 5-DOF hybrid robot. They optimized the feed rate while improving precision at specific locations where precision deteriorated.

Despite the above advances, it remains challenging to account for the unpredictable environment that occurs during actual rough machining when optimizing NC data. If the tools are severely worn or damaged during machining, product defects can occur. Sudden changes in the cutting state, such as thermal deformation of the spindle and tool vibration, not only decrease the efficiency of optimized NC data, but also result in machining time delays and the waste of materials.

Therefore, even with pre-optimized NC data, additional NC control is necessary to handle unexpected situations during cutting to improve NC data efficiency. Adaptive control technology is advancing in accordance with innovations in IT and IoT technology, thereby enabling real-time optimization. It is essential to monitor the machining load in real time during cutting and to balance the machining load by lowering or increasing the feed rate in segments with high or low spindle load through adaptive control that considers machining dynamic characteristics. If tool vibration occurs, an adaptive control strategy that adjusts the main spindle's revolutions is required.

Munoa et al. [19] conducted a review of research papers to tackle the issue of tool vibration causing chatter during machining. They designed a combination of equipment, material, and tools to optimize the machining conditions and prevent chatter. Additionally, they introduced studies that diagnose chatter and control the spindle speed and feed rate in real time to avoid chatter. Altintas et al. [20] produced cutting physics-based data, such as that of cutting force, torque, and chip load, through a preliminary cutting simulation that considered the combination of equipment, material, and tools. This simulation predicted abnormal conditions and tool overload during actual machining. Moreover, the cutting conditions of parts were improved by employing adaptive feed control technology. However, although they predicted the cutting force through the simulation, their method could not actually reflect the dynamic machining load in their feed control, nor verifying the tool wear and breakage in advance.

In this study, we designed a method to synchronize and store the main axis load and NC data. We analyzed the stored data to extract the reference load control curve (RLCC) information. Based on this information, we created an adaptive feed control method that adjusts the tool feed rate during actual rough machining. Unlike passive adaptive control, which relies on the current cutting conditions, our proposed method provides active adaptive control using RLCCs. We conducted experiments to evaluate the effectiveness of the proposed method and presented some of the results at the FAIM2022 international conference [21]. In the present study, we extended a previous study and compared three adaptive feed control methods, including the proposed method using acceleration and deceleration control. The relative performance of the feed control methods, including those providing non-adaptive control, was analyzed through actual machining experiments. The RLCC was continuously updated with consideration of the tool wear and chatter that occur during real-time machining. It could thus be employed for subsequent part machining. Therefore, the proposed advanced adaptive control method can reflect the dynamic machining characteristics of the tool, predicting the tool wear and breakage in advance via RLCCs.

## 2. Adaptive control

NC data is a program written before machining is performed to reflect the machining characteristics, such as those of the tool, material, and spindle performance. CNC reads the NC data and moves the tool to the designated position using a pre-set feed rate. Generally, during the cutting process, CNC cannot change the machining path by considering the machining load, spindle performance (torque and power), and dynamic characteristics of the tool. Nevertheless, it is possible to change the feed rate and spindle speed among cutting conditions through adaptive control, which adjusts the machining load by increasing or decreasing the feed rate.

As shown in Fig. 1, to place the cutting load within the upper and lower limits,the feed rate is increased \( \left( {F + }\right) \) for loads below the lower limit and reduced (F-) for loads above the upper limit to maintain a uniform cutting load. This approach has the advantage of shortening the machining time by increasing the feed rate in the no-load segment. However, conventional adaptive control is passive in nature; it performs the controlling after a machining event (over/no-load) occurs. There are several problems with this passive characteristic. i) Increasing the feed rate in the no-load segment and then inserting the material potentially decreases the tool life or causes damage. ii) Reducing the feed rate in the high cutting load segment decreases productivity. iii) Frequent feed rate increasing or decreasing during uneven cutting of load segments decreases the quality of the machined surface. iv) There are difficulties in selecting the cutting load reference line, which varies depending on the material, tool, and machining stage.

<!-- Meanless: 2-->




<!-- Meanless: S.G. Kim et al. Robotics and Computer-Integrated Manufacturing 85 (2024) 102621-->

<!-- Media -->

<!-- figureText: feedrate=1,000 [mm/min] \( \varepsilon  = {0.22}\left\lbrack  \mathrm{\;A}\right\rbrack \) 12.6 Time [sec] 10.0 depth =0.1[mm] width \( = {7.2}\left\lbrack  \mathrm{\;{mm}}\right\rbrack \) Cutting Load [A] Tool \( = {16}\mathrm{D}{0.8}\mathrm{R} \) 9.0 8.0 7.0 -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_2.jpg?x=142&y=154&w=667&h=476&r=0"/>

Fig. 2. Fluctuation of cutting load on the same cutting condition [21].

<!-- Media -->

## 3. Proposed adaptive feed control

Expensive commercial CAM (Computer Aided Manufacturing) systems are not easily applicable on the machining floor owing to their poor field adaptability and their requirement of correction based on actual machining loads and predicted values. Our proposed advanced adaptive feed control method obtains a reference load control curve (RLCC) through synchronizing the tool position and sensor data during repetitive part machining. This active adaptive control overcomes the limitations of passive adaptive control by searching for a reference curve during machining and using that information for CNC control. As a result, this method enables the diagnosis of tool wear and tear.

### 3.1. Machining history

The term "machining history" refers to a file that records the machining load, vibration levels, and other machining-related information, all synchronized with the tool position during the machining of a specific product [21]. By utilizing machining history during the machining of that part, the limitations of passive adaptive control can be overcome. The machining history is comprised primarily of CNC information and sensor data. CNC information includes precise information on the tool position and speed (such as the feed and rotation speed), while the sensor data include real-time cutting characteristic information. It is crucial to combine both CNC information and sensor data to effectively diagnose the machining state. Sensors are chosen depending on the diagnostic purpose. A current sensor is used to measure the current of the spindle motor for diagnosing the machining load, while an acoustic or acceleration sensor is used to diagnose tool vibrations. A tool dynamometer is employed to measure the cutting force on the material.

If the structure that synchronizes CNC data and sensor data is represented by \( {\operatorname{Rec}}_{i} \) at time \( i \) ,then the machining history can be considered a consecutive set of \( {\operatorname{Rec}}_{i} \) stored between the start and end of machining (Eq. (1)). The machining history includes information on the physical state in which the part has been machined. In this study, a current sensor was used to measure the machining load, and the machining history data, synchronized with the CNC information, were utilized to establish a RLCC for adaptive control.

\[\text{ Machining History } = \left\lbrack  {\mathrm{{Rec}}}_{1}\right\rbrack   + \left\lbrack  {\mathrm{{Rec}}}_{2}\right\rbrack   + \cdots  + \left\lbrack  {\mathrm{{Rec}}}_{n}\right\rbrack  \]

\[ = \mathop{\sum }\limits_{{i = 1}}^{n}{\operatorname{Rec}}_{i} \tag{1}\]

### 3.2. Reference load control curve (RLCC)

The maximum allowable upper limit at a specific machining position is defined as the control load upper limit curve. The minimum load curve for detecting tool breakage is defined as the control load lower limit curve. These two curves are used as RLCCs in the proposed adaptive control method. The machining load \( \left( {\operatorname{Load}}_{i}\right) \) measured at the tool position \( i\left( {\operatorname{Rec}}_{i}\right) \) can be expressed by the linear interpolation (Eq. (2)) of the cutting force \( \left( {F}_{i}\right) \) . The upper limit \( \left( {\operatorname{Load}}_{i,\max }\right) \) can be found in Eq. (3). Let \( {\delta }_{i,u} \) be the difference \( \left( {{\operatorname{Load}}_{i,\max } - {\operatorname{Load}}_{i}}\right) \) between the upper limit and the measured load at the tool position (Eq. (4)) by using Eq. (5) and considering the scenario of no cutting load.

The maximum permissible upper limit at a specific machining position is defined as the control load upper limit curve, and the minimum load curve required to detect tool breakage is defined as the control load lower limit curve. These two curves are used as RLCCs in the proposed adaptive control. The machining load \( \left( {\operatorname{Load}}_{i}\right) \) measured at the tool position \( i\left( {\operatorname{Rec}}_{i}\right) \) can be expressed by linear interpolation Eq. (2)) of the cutting force \( \left( {F}_{i}\right) \) . The upper limit \( \left( {\operatorname{Load}}_{i,\max }\right) \) can be found using Eq. (3). Let \( {\delta }_{i,u} \) be the difference \( \left( {{\operatorname{Load}}_{i,\max } - {\operatorname{Load}}_{i}}\right) \) between the upper limit and the measured load at the tool position (Eq. (4)), and, considering no cutting load, the upper and lower limit control curves at the tool position \( i \) can be obtained as in Eqs. (6) and ((7) by using Eq. (5) [21].

\[{\text{ Load }}_{i} = a{F}_{i} + b \tag{2}\]

\[{\text{ Load }}_{i,\max } = a{F}_{i,\max } + b \tag{3}\]

\[{\delta }_{i,u} = {\text{ Load }}_{i,\max } - {\text{ Load }}_{i} \tag{4}\]

\[{\delta }_{i,l} = \min \left( {{\text{ Load }}_{i} - {\text{ Load }}_{i,\text{ none }},{\text{ Load }}_{i} - {\text{ Load }}_{i,\text{ min }}}\right)  \tag{5}\]

Upper Limit \( = {\text{Load}}_{i,\max } \)

(6)

\[ = \max \left( {{\text{ Load }}_{1} + {\delta }_{1,u},\cdots ,{\text{ Load }}_{n} + {\delta }_{n,u}}\right) \]

\[\text{Lower Limit} = {\text{Load}}_{i,\min } \tag{7}\]

\[ = \min \left( {{\text{ Load }}_{1} - {\delta }_{1,l},\cdots ,{\text{ Load }}_{n} - {\delta }_{n,l}}\right) \]

where \( a \) is the tangential coefficient, \( b \) denotes the intercept value,and \( {F}_{i} \) is the cutting force at tool position \( i \) .

Even when the same material removal rate is geometrically assumed in repetitive machining, deviations in cutting load values as per sensor readings occur Fig. 2). The magnitude of these deviations, which is the difference between the maximum and minimum load, is referred to as noise \( \left( \varepsilon \right) \) . Hence,in the RLCC Eqs.(6)and \( \left( {\left( 7\right) ,{\delta }_{i,u}}\right. \) and \( {\delta }_{i,l} \) must account for the amplitude at tool position \( i \) . As a result,the RLCC equation can be rewritten as Eqs. (8) and (9) [21].

\[\text{Upper Limit} = {\text{Load}}_{i,\max } + \varepsilon  \tag{8}\]

\[\text{Lower Limit} = {\text{Load}}_{i,\min } - \varepsilon  \tag{9}\]

### 3.3. Diagnosis of tool wear/breakage

If the feed rate is increased, the machining load increases as a result of a higher feed rate per tooth. Conversely, if the feed rate is reduced, the machining load decreases. However, excessively worn tools may still exceed the upper control limit even if the feed rate is lowered. If the upper limit is exceeded for a prolonged period of time, even when machining at the lowest possible feed rate \( \left( {F}_{\text{min }}\right) \) ,this could indicate excessive tool wear. On the other hand, if the machining loads remain lower than the lower control limit while using the maximum possible feed rate \( \left( {F}_{\max }\right) \) for a prolonged period of time,tool breakage can be diagnosed.

<!-- Meanless: 3-->




<!-- Meanless: S.G. Kim et al. Robotics and Computer-Integrated Manufacturing 85 (2024) 102621-->

<!-- Media -->

<!-- figureText: Cycle Start ref. curve No alarm or feed-hold Read CNC & sensor data Search \( {Re}{c}_{i} \) according to tool position Compare cutting load with \( {Re}{c}_{i} \) and \( {\Delta t} \) load Is a tool in normal cutting state? Yes Feed override according to the rule No Is it finished? Yes Save the cutting history Cycle End -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_3.jpg?x=406&y=151&w=937&h=746&r=0"/>

Fig. 3. Proposed adaptive control method using RLCCs.

<!-- figureText: Tool details Tool Workpiece -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_3.jpg?x=411&y=1001&w=930&h=404&r=0"/>

Fig. 4. End mill of 16D 0.8R and zig-zag tool path [21].

<!-- Media -->

### 3.4. Proposed adaptive feed control

In the proposed adaptive control method, tool feed control is performed through the application of the following five rules [21].

Rule 1. Acceleration/deceleration uses the upper/lower limit reference control curve.

Rule 2. A maximum feed rate is used in non-load segments.

Rule 3. An approach feed rate is applied when a tool enters a workpiece.

Rule 4. Deceleration occurs due to overload within \( {\Delta t} \) .

Rule 5. A feed-hold is performed in accordance with \( {H}_{TP} \) and \( T{L}_{amp} \) where, \( {H}_{TP} \) is the harmonic of a tooth passing frequency,and \( T{L}_{{amp}.} \) is the amplitude of a cutting load.

The feed rate is controlled by the first upper/lower limit control curve, with acceleration or deceleration taking place according to the load currently being machined (as shown in Fig. 1). If the non-load segment is continuous and the tool feed length is sufficient, the tool is fed at a rapid speed. During the machining process, if the difference between the current machining load and the machining load after \( {\Delta t} \) time is greater than a specified range (indicating that a tool has entered a workpiece),an approach feed rate is applied,where \( {\Delta t} \) is proportional to the cycle of NC control [21].

The flowchart of the proposed adaptive control method using RLCCs is shown in Fig. 3. At the start of machining, the control curve point is identified according to the tool position, and the machining load is determined. If tool wear or damage is detected, an alarm or feed-hold is performed. If the tool is in a normal state, the machining load is compared with the upper/lower limits of the RLCC, and Rules 1 to 5 are applied by increasing or decreasing the feed rate. This proper adaptive control is repeated until the end of the machining process.

## 4. Experimental results

To verify the proposed adaptive control method, the rough machining path depicted in Fig. 4 was used. The cutting conditions comprised a spindle speed of \( {3600}\mathrm{{rpm}} \) ,a feed rate of \( {1000}\mathrm{\;{mm}}/\mathrm{{min}} \) , and a cutting depth of \( {1.0}\mathrm{\;{mm}} \) . A zig-zag path with a varying cutting width was used as the tool path. A 16D0.8R 2-flute insert end-mill was used as the tool and S45C structural steel as the material. A current sensor was used to measure the machining load, while an acceleration or vibration sensor was used to identify the machining or non-machining area. The current sensor is for indirectly measuring the cutting load. Raw data was sampled at a cycle of \( 2\mathrm{{kHz}} \) ,but RMS (Root Mean Square) conversion was performed at intervals of \( {50}\mathrm{\;{ms}} \) . Acceleration or vibration sensor data was sampled at a cycle of \( {16},{384}\mathrm{\;{Hz}} \) ,then converted via FFT (Fast Fourier Transform), and finally the amplitude and magnitude of the tool passing frequency were analysed to separate cutting and non-cutting areas.

<!-- Meanless: 4-->




<!-- Meanless: S.G. Kim et al. Robotics and Computer-Integrated Manufacturing 85 (2024) 102621-->

<!-- Media -->

<!-- figureText: 20 Time [sec] Actual cutting load at \( {14}^{\text{th }} \) layer Actual feedrate Cutting Load [A] 18 16 14 12 10 8 6 2,000 feedrate [mm/min] 1,500 1,000 500 0 Actual cutting load at \( {1}^{\text{st }} \) layer -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_4.jpg?x=361&y=154&w=1020&h=634&r=0"/>

Fig. 5. Cutting load and feed rate during the 13th machining operation.

<!-- figureText: 20 Time [sec] Lower limit Actual feedrate Cutting Load [A] 18 16 14 12 10 8 6 2,000 Feedrate [mm/min] 1,500 1,000 500 0 Actual cutting load Upper limit -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_4.jpg?x=363&y=887&w=1016&h=665&r=0"/>

Fig. 6. Cutting load during the 14th machining operation via conventional adaptive control.

<!-- Media -->

The tool path in Fig. 4 was first machined without any adaptive control strategies for comparison purposes. As shown in Table 1 of Appendix, a total of 13 machining operations were performed, with a machining time for one round of \( 4\mathrm{\;{min}} \) and \( {58}\mathrm{\;s} \) (non-cutting time \( = \) \( {31.8}\mathrm{\;s} \) ,tool engaged time \( = {266.5}\mathrm{\;s} \) ). The average machining load \( \left( {C}_{avg}\right) \) increased with the number of machining operations, as shown in Fig. 5, on account of tool wear.

In this study, three types of adaptive control methods were designed, all with the same cutting path and cutting conditions as in Fig. 4. The first method uses a fixed value as the control curve for adaptive control, the second method uses an envelope curve as a RLCC based on the cutting load profile, and the third method employs both the envelope and cutting property values to create the RLCC.

### 4.1. Conventional adaptive control

In this study, the conventional adaptive control method utilized an upper limit value of \( {130}\% \) of the average cutting load(8.3A)obtained during the initial layer machining, as indicated in Table 1 of the Appendix. Meanwhile, the lower limit was set as the average value. A total of 14 machining operations were conducted using this control method, as shown in Table 2 of the Appendix. The average machining load in the first layer was \( {8.46}\mathrm{\;A} \) ,and the average machining load in the machining area was \( {8.66}\mathrm{\;A} \) . The adaptive control method improved the feed rate,i. e., feed override value (FOV) by 16.5% and reduced the machining time by 14.13%. However, the faster feed rate increased the average machining load by 1.64%.

The average load of the 14th layer machining, when the tool life actually ceased,was \( {8.95}\mathrm{\;A} \) . The machining time was \( {272.7}\mathrm{\;s} \) ,a reduction of 8.59%, while the feed rate was improved by 9.4%. The tool wear deteriorated due to the increase in machining load with the increase in the FOV value until the 14th layer machining, making it difficult to continue rough cutting further. It is important to note that as the number of layers machined using adaptive control increased, the cutting load and time gradually increased, leading to a decrease in the effectiveness of the adaptive control. Specifically, in the machining area where the cutting width is small and the machining load is low (as seen in Fig. 6), only acceleration control was executed since the machining load was lower than the given lower limit, which resulted in severe tool wear.

<!-- Meanless: 5-->




<!-- Meanless: S.G. Kim et al. Robotics and Computer-Integrated Manufacturing 85 (2024) 102621-->

<!-- Media -->

<!-- figureText: 20 Time [sec] Lower limit Actual feedrate Cutting Load [A] 18 16 14 12 10 8 2,000 Feedrate [mm/min] 1,500 1,000 500 0 Actual cutting load Upper limit -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_5.jpg?x=361&y=152&w=1020&h=629&r=0"/>

Fig. 7. Cutting load during the 12th machining operation using adaptive control with RLCCs.

<!-- figureText: 18.57 Cutting Load [A] 17.5 0 12.6 Time [sec] Cutting Load [A] 17.5 16.43 6.8 Cutting width [%] -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_5.jpg?x=359&y=880&w=1024&h=395&r=0"/>

Fig. 8. Linear compensation of cutting load (sensor data [A]) (red line is \( {\operatorname{load}}_{\max } \) and green line is \( {\operatorname{load}}_{\min } \) ).

<!-- figureText: 20 None-cutting control Approach feed control Time [sec] Lower limit Actual feedrate Cutting Load [A] 18 16 14 12 10 8 6 2,000 Feedrate [mm/min] 1,500 1,000 500 0 Actual cutting load Upper limit -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_5.jpg?x=362&y=1371&w=1020&h=657&r=0"/>

Fig. 9. Cutting load during the 17th machining operation with the proposed adaptive control method.

<!-- Meanless: 6-->




<!-- Meanless: S.G. Kim et al. Robotics and Computer-Integrated Manufacturing 85 (2024) 102621-->

<!-- figureText: \( {12}^{\text{th }} \) Layer Original NC code without adaptive control Conventional adaptive control Adaptive control with reference load control curves Proposed advanced adaptive control 15 17 19 21 10.00 9.80 9.60 9.40 9.20 Cutting Load [A] 9.00 8.80 8.40 8.20 8.00 7.80 3 7 9 11 No. of Layer -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_6.jpg?x=285&y=154&w=1184&h=505&r=0"/>

Fig. 10. Comparison of four feed control methods according to average cutting load.

<!-- figureText: 305.00 Original NC code without adaptive control Conventional adaptive control Adaptive control with reference load control curves Proposed advanced adaptive control 13 15 17 19 21 295.00 285.00 Cycle time[second] 275.00 265.00 255.00 245.00 1 3 7 9 11 No. of Layer -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_6.jpg?x=283&y=757&w=1188&h=475&r=0"/>

Fig. 11. Comparison of four feed control methods according to cutting time.

<!-- figureText: 130.00 - Original NC code without adaptive control Conventional adaptive Adaptive control with reference load control curves Proposed advanced adaptive control 15 17 19 21 125.00 120.00 Feed Override [%] 115.00 110.00 105.00 100.00 95.00 90.00 3 5 7 11 No. of Layer -->

<img src="https://cdn.noedgeai.com/bo_d2slbnbef24c73b2kggg_6.jpg?x=283&y=1330&w=1186&h=472&r=0"/>

Fig. 12. Comparison of four feed control methods according to average feed rate.

<!-- Media -->

### 4.2. Adaptive control with RLCCs

In this adaptive control method, the upper/lower limit curves for feed control were applied using the machining load curve generated through the machining in Fig. 4. As shown in Fig. 7 and as introduced in Section 4.1, the machining load data from the first layer of cutting without adaptive control were used to generate the upper/lower limit curve for control. The control of the feed rate was performed by comparing the actual cutting load at the current cutting position with the upper/lower limit values of the upper/lower RLCC. As a result, as shown in Table 3 of the Appendix, 12 machining operations were performed.

<!-- Meanless: 7-->




<!-- Meanless: S.G. Kim et al. Robotics and Computer-Integrated Manufacturing 85 (2024) 102621-->

<!-- Media -->

Table 1

Experimental result without adaptive feed control.

<table><tr><td rowspan="2">Index</td><td rowspan="2">\( {C}_{\text{avg }}\left\lbrack  \mathrm{A}\right\rbrack \)</td><td colspan="2">Tool Engaged Area</td></tr><tr><td>Load avg. [A]</td><td>Load Ratio [%]</td></tr><tr><td>1</td><td>8.35</td><td>8.52</td><td>-</td></tr><tr><td>2</td><td>8.31</td><td>8.42</td><td>-</td></tr><tr><td>3</td><td>8.33</td><td>8.5</td><td>-</td></tr><tr><td>4</td><td>8.35</td><td>8.6</td><td>0.9</td></tr><tr><td>5</td><td>8.36</td><td>8.59</td><td>0.8</td></tr><tr><td>6</td><td>8.38</td><td>8.53</td><td>0.1</td></tr><tr><td>7</td><td>8.50</td><td>8.77</td><td>2.9</td></tr><tr><td>8</td><td>8.58</td><td>8.85</td><td>3.9</td></tr><tr><td>9</td><td>8.80</td><td>9.04</td><td>6.1</td></tr><tr><td>10</td><td>8.80</td><td>9.06</td><td>6.3</td></tr><tr><td>11</td><td>8.90</td><td>9.07</td><td>6.5</td></tr><tr><td>12</td><td>8.87</td><td>9.1</td><td>6.8</td></tr><tr><td>13</td><td>8.98</td><td>9.25</td><td>8.6</td></tr></table>

Table 2

Adaptive feed control with fixed control load limit values.

<table><tr><td rowspan="2">Index</td><td rowspan="2">\( {C}_{avg}\left\lbrack  \mathrm{\;A}\right\rbrack \)</td><td rowspan="2">Cycle Time \( T\left\lbrack  {sec}\right\rbrack \)</td><td colspan="4">Adaptive Control Effect</td></tr><tr><td>FOV avg. [%]</td><td>Load avg. [A]</td><td>Time ratio [%]</td><td>Load ratio [%]</td></tr><tr><td>Org</td><td>8.35</td><td>298.3</td><td>100.0</td><td>8.52</td><td>-</td><td>-</td></tr><tr><td colspan="7">NC</td></tr><tr><td>1</td><td>8.46</td><td>256.1</td><td>116.5</td><td>8.66</td><td>14.13</td><td>1.64</td></tr><tr><td>2</td><td>8.55</td><td>256.8</td><td>116.4</td><td>8.77</td><td>13.90</td><td>2.91</td></tr><tr><td>3</td><td>8.47</td><td>256.7</td><td>116.4</td><td>8.72</td><td>13.95</td><td>2.39</td></tr><tr><td>4</td><td>8.45</td><td>256.7</td><td>116.3</td><td>8.66</td><td>13.94</td><td>1.63</td></tr><tr><td>5</td><td>8.49</td><td>259.2</td><td>115.5</td><td>8.69</td><td>13.11</td><td>1.98</td></tr><tr><td>6</td><td>8.52</td><td>259.4</td><td>115.3</td><td>8.72</td><td>13.04</td><td>2.31</td></tr><tr><td>7</td><td>8.54</td><td>261.0</td><td>114.8</td><td>8.75</td><td>12.50</td><td>2.66</td></tr><tr><td>8</td><td>8.53</td><td>259.6</td><td>114.9</td><td>8.75</td><td>12.97</td><td>2.65</td></tr><tr><td>9</td><td>8.57</td><td>260.0</td><td>114.9</td><td>8.78</td><td>12.83</td><td>3.06</td></tr><tr><td>10</td><td>8.58</td><td>260.1</td><td>114.7</td><td>8.80</td><td>12.79</td><td>3.31</td></tr><tr><td>11</td><td>8.66</td><td>265.4</td><td>112.6</td><td>8.89</td><td>11.03</td><td>4.34</td></tr><tr><td>12</td><td>8.74</td><td>265.9</td><td>112.5</td><td>8.93</td><td>10.85</td><td>4.81</td></tr><tr><td>13</td><td>8.80</td><td>267.9</td><td>111.4</td><td>9.05</td><td>10.20</td><td>6.17</td></tr><tr><td>14</td><td>8.95</td><td>272.7</td><td>109.4</td><td>9.22</td><td>8.59</td><td>8.22</td></tr></table>

Table 3

Adaptive control with reference load control curves (RLCCs).

<table><tr><td rowspan="2">Index</td><td rowspan="2">\( {C}_{avg} \) [A]</td><td rowspan="2">Cycle Time \( T \) [sec]</td><td colspan="4">Adaptive Control Effect</td></tr><tr><td>FOV avg. [%]</td><td>Load avg. [A]</td><td>Time ratio [%]</td><td>Load ratio [%]</td></tr><tr><td>Org</td><td>8.35</td><td>298.3</td><td>100.0</td><td>8.52</td><td>-</td><td>-</td></tr><tr><td>NC</td><td/><td/><td/><td/><td/><td/></tr><tr><td>1</td><td>8.63</td><td>249.4</td><td>119.6</td><td>8.85</td><td>16.39</td><td>3.84</td></tr><tr><td>2</td><td>8.59</td><td>249.4</td><td>119.6</td><td>8.80</td><td>16.40</td><td>3.29</td></tr><tr><td>3</td><td>8.59</td><td>249.3</td><td>119.6</td><td>8.80</td><td>16.41</td><td>3.30</td></tr><tr><td>4</td><td>8.60</td><td>249.5</td><td>119.6</td><td>8.81</td><td>16.35</td><td>3.43</td></tr><tr><td>5</td><td>8.61</td><td>249.3</td><td>119.6</td><td>8.83</td><td>16.41</td><td>3.60</td></tr><tr><td>6</td><td>8.63</td><td>249.8</td><td>119.5</td><td>8.85</td><td>16.27</td><td>3.90</td></tr><tr><td>7</td><td>8.65</td><td>249.9</td><td>119.4</td><td>8.87</td><td>16.24</td><td>4.13</td></tr><tr><td>8</td><td>8.68</td><td>251.1</td><td>118.8</td><td>8.90</td><td>15.82</td><td>4.50</td></tr><tr><td>9</td><td>8.70</td><td>252.9</td><td>117.9</td><td>8.93</td><td>15.23</td><td>4.81</td></tr><tr><td>10</td><td>8.79</td><td>260.4</td><td>114.6</td><td>9.03</td><td>12.72</td><td>6.02</td></tr><tr><td>11</td><td>8.80</td><td>265.7</td><td>112.2</td><td>9.04</td><td>10.94</td><td>6.13</td></tr><tr><td>12</td><td>8.83</td><td>271.0</td><td>110.0</td><td>9.08</td><td>9.15</td><td>6.60</td></tr></table>

<!-- Media -->

Because there were offset upper and lower limits for each of the machining loads over time, the average machining load resulted in a higher value than the average machining load obtained in Section 4.1. Therefore, the feed rate in the first layer machining was improved by 19.6% compared to the previous machining without adaptive control, and the machining time was reduced by 16.39%. On the other hand, because it was machined at a higher feed rate over the entire machining segments, the tool life was reached after 12 machining operations (as seen in Table 3 of the Appendix).

### 4.3. Proposed advanced adaptive control

The machining experiment using the proposed adaptive control method involved the following steps. To observe the effect of adaptive control in the approach feed and non-load segments, the tool path was created by varying the length of the non-load segment (the time when the tool was not engaged). The feed rate was controlled using the feed override (FOV) value, which was set between 80 and 120% of the feed rate specified in the NC part program, according to Rule 1. During the non-load segment,the rapid feed rate \( \left( {F}_{\text{rapid }}\right) \) was set to \( {150}\% \) of the feed rate, as per Rule 2. The approach feed rate was set to 80% of the feed rate, according to Rules 3 and 4.

#### 4.3.1. Derivation of RLCCs

The reference load control curve (RLCC) could be produced using Eqs. (2)-(7). By applying a cutting width of 100% with the same cutting depth,the average cutting load was \( {17.5}\mathrm{\;A} \) . When the tool entered the workpiece,the generated load was \( {6.8}\mathrm{\;A} \) . Thus,by assuming linear interpolation, the slope constant (a) was 0.0098 and the intercept (b) was 6.8 as shown in Fig. 8. At this time, the maximum error amplitude (ε) was \( {0.3}\mathrm{\;A} \) . If the upper limit of control was \( {10}\% \) higher than the cutting force, then the upper limit of the control curve for the measured cutting load of \( {17.5}\mathrm{\;A} \) was \( {18.57} + {0.3} = {18.87}\mathrm{\;A} \) using Eq. (8). Similarly,using Eq. (9),the lower limit of the control curve was \( {16.13}\mathrm{\;A} \) .

#### 4.3.2. Advanced adaptive feed control

Table 4 in the Appendix shows the results of the proposed adaptive feed control method using the same machining path and tool. As with the previous adaptive control approach,the average cutting load \( \left( {C}_{avg}\right) \) increases with the number of cuts, but only slightly. The feed rate is divided into three sections: i) a section with a maximum feed (rapid traverse section at 150% of the given feed rate) in the no-load segment, ii) a section for material entry or deceleration by a look-ahead function with an approach feed rate (80% of the given feed rate), and iii) a section with acceleration/deceleration via RLCCs according to Rule 1. Additionally, 100% feed override (FOV) was applied when the difference between the cutting load after some time and the current cutting load was not large. Even with an increase in the number of machining, the average time in the transfer segment \( \left( {F}_{\text{rapid }}\right) \) and the average time in the approach feed application segment \( \left( {T}_{\text{rapid }}\right) \) remain roughly the same. Actual feed control error can be attributed to the CNC and interface communication delay(0.23s),and this is conventionally at least twice the communication speed between the CNC controller and the interface. According to Rule 1,the average machining time \( \left( {T}_{\text{control }}\right) \) in the feed rate acceleration/deceleraton segment steadily increases, while the FOVavg decrease is inversely proportional. As an effect of the adaptive control using acceleration and deceleration, there is a 4.6 to 12.8% reduction in machining time, while the cutting load ratio increases by 1.4 to 5.3%.

Although the same RLCCs as in Section 4.2 were used, the total machining time has been reduced, and the tool life has been extended up to 17 machining cycles due to the rapid traverse in the non-machining section and the reduction of the impact applied to the tool when entering the workpiece (Fig. 9). This corresponds to a 41.7% increase in tool life compared to the adaptive control method with RLCCs discussed in Section 4.2. Please refer to Tables 3 and 4 in the Appendix for further details.

<!-- Meanless: 8-->




<!-- Meanless: S.G. Kim et al. Robotics and Computer-Integrated Manufacturing 85 (2024) 102621-->

<!-- Media -->

Table 4

Machining performance via proposed adaptive feed control method.

<table><tr><td rowspan="2">Index</td><td rowspan="2">\( {C}_{avg}\left\lbrack  \mathrm{\;A}\right\rbrack \)</td><td colspan="4">Cycle Time</td><td colspan="4">Adaptive Control Effect</td></tr><tr><td>\( T\left\lbrack  {sec}\right\rbrack \)</td><td>\( {T}_{\text{rapid }}\left\lbrack  {sec}\right\rbrack \)</td><td>\( {T}_{\text{app-roach }} \) [sec]</td><td>\( {T}_{\text{control }} \) [sec]</td><td>FOV avg. [%]</td><td>Load avg. [A]</td><td>Time ratio [%]</td><td>Load ratio [%]</td></tr><tr><td>Org NC</td><td>8.35</td><td>298.3</td><td>-</td><td>-</td><td>-</td><td>100.0</td><td>8.52</td><td>-</td><td>-</td></tr><tr><td>1</td><td>8.46</td><td>260.3</td><td>17.27</td><td>14.4</td><td>228.7</td><td>115.4</td><td>8.64</td><td>12.8</td><td>1.4</td></tr><tr><td>2</td><td>8.57</td><td>260.0</td><td>17.47</td><td>13.8</td><td>228.8</td><td>115.6</td><td>8.77</td><td>12.9</td><td>2.9</td></tr><tr><td>3</td><td>8.57</td><td>259.9</td><td>18.13</td><td>13.5</td><td>228.3</td><td>115.4</td><td>8.67</td><td>12.9</td><td>1.8</td></tr><tr><td>4</td><td>8.58</td><td>259.2</td><td>17.77</td><td>13.5</td><td>227.9</td><td>115.7</td><td>8.67</td><td>13.1</td><td>1.8</td></tr><tr><td>5</td><td>8.57</td><td>260.2</td><td>17.98</td><td>13.2</td><td>229</td><td>115.4</td><td>8.66</td><td>12.8</td><td>1.6</td></tr><tr><td>6</td><td>8.58</td><td>260.6</td><td>17.49</td><td>15.1</td><td>228</td><td>115.1</td><td>8.69</td><td>12.7</td><td>2.0</td></tr><tr><td>7</td><td>8.61</td><td>260.6</td><td>17.63</td><td>13.2</td><td>229.7</td><td>115.1</td><td>8.79</td><td>12.7</td><td>3.2</td></tr><tr><td>8</td><td>8.61</td><td>261.5</td><td>17.15</td><td>15.3</td><td>229</td><td>114.9</td><td>8.78</td><td>12.4</td><td>3.1</td></tr><tr><td>9</td><td>8.62</td><td>261.5</td><td>17.91</td><td>14.4</td><td>229.1</td><td>115.1</td><td>8.8</td><td>12.4</td><td>3.3</td></tr><tr><td>10</td><td>8.62</td><td>260.9</td><td>17.16</td><td>14.2</td><td>229.5</td><td>115.0</td><td>8.8</td><td>12.6</td><td>3.3</td></tr><tr><td>11</td><td>8.64</td><td>261.6</td><td>17.35</td><td>14.2</td><td>230</td><td>114.7</td><td>8.75</td><td>12.3</td><td>2.7</td></tr><tr><td>12</td><td>8.70</td><td>262.4</td><td>17.23</td><td>14</td><td>231.2</td><td>114.2</td><td>8.87</td><td>12.1</td><td>4.1</td></tr><tr><td>13</td><td>8.69</td><td>263.4</td><td>17.53</td><td>15.3</td><td>230.6</td><td>114.1</td><td>8.88</td><td>11.7</td><td>4.2</td></tr><tr><td>14</td><td>8.73</td><td>266.8</td><td>17.1</td><td>14.5</td><td>235.2</td><td>112.7</td><td>8.93</td><td>10.6</td><td>4.8</td></tr><tr><td>15</td><td>8.76</td><td>268.2</td><td>17.51</td><td>15.5</td><td>235.2</td><td>111.9</td><td>8.96</td><td>10.1</td><td>5.2</td></tr><tr><td>16</td><td>8.81</td><td>276.8</td><td>17.79</td><td>15.6</td><td>243.4</td><td>108.5</td><td>9.02</td><td>7.2</td><td>5.9</td></tr><tr><td>17</td><td>8.85</td><td>284.6</td><td>17.66</td><td>14.5</td><td>252.4</td><td>105.3</td><td>8.97</td><td>4.6</td><td>5.3</td></tr></table>

<!-- Media -->

### 4.4. Results of comparison of feed control methods

In this study, four different feed control methods were tested during machining. One of these methods uses a fixed feed rate given in the original NC part program; the remaining three methods employ different means of controlling the feed rate. Figs. 10-12 present the results of the comparative analysis of these four feed control types with respect to the average machining load and tool wear, the machining time, and the average tool feed speed. The result of the test machining are summarized as follows.

(1) In the initial stages of machining, all three adaptive control methods demonstrated elevated feed rates and relatively higher cutting loads due to the acceleration/deceleration feed control (refer to Fig. 10 and 12), when compared to the non-adaptive fixed feed rate control. Nevertheless, as the number of machining layers increases, the adaptive control methods managed to maintain a lower cutting load compared to the nonadaptive control method (refer to Fig. 10).

(2) Conversely, in the case of feed controlling with fixed upper and lower limit values, without using cutting history data, as in the conventional adaptive control method (refer to the red line in Figs. 11 and 12), the feed rate increased from the initial layer, resulting in shorter machining cycle times in the initial layers. However, this approach is plagued by a problem of shortening tool life due to high feed machining, especially during the tool approaching stage.

(3) The adaptive feed control method with RLCCs leveraged higher feed rates using RLCCs, resulting in the shortest machining cycle time in the initial layers (refer to the green lines in Figs. 11 and 12). However, like the conventional adaptive control method, this approach also caused a problem of shortening tool life due to high feed machining during the tool approaching stage. It is worth noting that the tool life was reached after 12 cutting operations, indicating the shortest tool exchange cycle among the three adaptive control methods (refer to Table 3 in the Appendix).

(4) The advanced feed rate control method proposed in this paper exhibited the lowest feed rate among the three adaptive control methods as the number of layers increased. This is because the approach feed control reflects the cutting characteristics, thereby delaying the wear of the tool through the reduction of the instantaneous impact applied to the tool when it enters the material. Furthermore, acceleration control was carried out in all machining segments, and rapid traverse was executed in the non-machining section. Consequently, it has the effect of improving tool life while simultaneously enhancing feed speed accordingly (refer to the blue line in Figs. 11 and 12 and Tables 3 and 4 in the Appendix).

## 5. Conclusion

The efficiency of the machining process and the resulting outcomes are highly dependent on the cutting conditions and tool path used in the NC program. Even with pre-optimized NC data, additional feed control is necessary to handle unexpected situations during cutting and improve the NC data efficiency and tool life. However, achieving the desired control effect during actual machining of the same part can be challenging due to the influence of equipment, material, and tool combinations, especially in cutting simulation-based feed rate scheduling methods [3-5] or off-line feed rate scheduling methods considering the machining path [9-18]. To address this issue, adaptive control technology is emerging in response to advancements in IT and IoT, enabling real-time optimization.

Adaptive control methods used in most CNC machining floors typically employ acceleration/deceleration-based feed control with pre-set reference control values, i.e., upper and lower limits, without considering cutting path characteristics. In contrast, the advanced adaptive control method proposed in this study is based on machining history data. This enables prediction of the entry position of the workpiece or cutting segment where the cutting load suddenly goes high, such as corner cutting. Consequently, the feed rate can be controlled accordingly, while increasing the recycling rate of data collected during previous cutting operations, contributing to improving productivity and tool life. In the test machining, the proposed adaptive control method reduced production time by 4.6 to 12.8%, while prolonging tool life by up to 41.7%.

However, this method may not be suitable for low-volume production of molds and dies or mechanical parts. Therefore, future research should try first to optimize the feed rate off-line and then use the feed scheduling result to derive reference control curves, or develop an integrated control scheme that considers tool vibration or condition.

## Declaration of Competing Interest

The authors declare the following financial interests/personal relationships which may be considered as potential competing interests:

S.G. Kim, E.Y. Heo, H.G. Lee reports financial support was provided by Korean Ministry of SMEs and Startups through the P0016443.

<!-- Meanless: 9-->




<!-- Meanless: S.G. Kim et al. Robotics and Computer-Integrated Manufacturing 85 (2024) 102621-->

## Data availability

The data that has been used is confidential.

## Acknowledgement

This paper was supported by the Research Funds of Jeonbuk National University in 2023, and the Promotion of Innovative Businesses for Regulation-Free Special Zones funded by the Ministry of SMEs and Startups through the P0016443, Republic of Korea. The authors appreciate C.S. Im in EDIM Inc., D.H. Kim and S.J. Kim in CAMTIC Co., and J. S. Oh in New Technology Education Center of Korea Polytechnic on their invaluable support for the test machining experiment. They are also grateful to the FAIM2022 academic conference for the opportunity to present our basic research result on the adaptive feed control of CNC machining [21].

## Appendix

## Tables 1-4.

References

[1] I. Lazoglu, C. Manav, Y. Murtezaoglu, Tool path optimization for free form surface machining, CIRP Annals 58 (2009) 101-104, https://doi.org/10.1016/j.cirp.2009.03.054.

[2] H. Wang, P. Jang, J.A. Stori, A Metric-based Approach to two dimensional (2D) tool-path optimization for high-speed machining, J. Manuf. Sci. Eng. 127 (2005) 33-48, https://doi.org/10.1115/1.1830492.

[3] N. Melkote, W. Grzesik, J. Outeiro, J. Rech, V. Schulze, H. Attia, P. Arrazola, R. M'Saoubi, C. Saldana, Advances in material and friction data for modelling of metal machining, CIRP Annals 66 (2017) 731-754, https://doi.org/10.1016/j.cirp.2017.05.002.

[4] M. Azvar, A. Katz, J. Van Dorp, K. Erkorkmaz, Chip geometry and cutting force prediction in gear hobbing, CIRP Annals 70 (2021) 95-98, https://doi.org/ 10.1016/j.cirp.2021.04.082.

[5] S. Berger, G. Brock, D. Biermann, Simulative design of constraints for targeted restriction of chip thickness deviations when machining titanium alloy Ti6Al4V, Procedia CIRP 102 (2021) 85-90, https://doi.org/10.1016/j.procir.2021.09.015.

[6] C.H. Lee, F. Yang, H. Zhou, P. Hu, K. Min, Cross-directional feed rate optimization using tool-path surface, Int. J. Adv. Manuf. Technol. 108 (2020) 2645-2660, https://doi.org/10.1007/s00170-020-05336-4.

[7] J. Xie, P. Zhao, P. Hu, Y. Yin, H. Zhou, J. Chen, J. Yang, Multi-objective feed rate optimization of three-axis rough milling based on artificial neural network, Int. J. Adv. Manuf. Technol. 114 (2021) 1323-1339, https://doi.org/10.1007/s00170- 021-06902-0.

[8] E. Budak, I. Lazoglu, B.U. Guzel, Improving cycle time in sculptured surface machining though force modeling, CIRP Annals 53 (2004) 103-106, https://doi.org/10.1016/S0007-8506(07)60655-6.

[9] K. Erkorkmaz, S.E. Layegh, I. Lazoglu, H. Erdim, Feedrate optimization for freeform milling considering constraints from the feed drive system and process mechanics, CIRP Annals 62 (2013) 395-398, https://doi.org/10.1016/j.cirp.2013.03.084.

[10] L. Rattunde, I. Laptev, E.D. Klenske, H.C. Möhring, Safe optimization for feedrate scheduling of power-constrained milling processes by using Gaussian processes, Procedia CIRP 99 (2021) 127-132, https://doi.org/10.1016/j.procir.2021.03.020.

[11] H. Liu, Q. Liu, S. Yuan, Adaptive feedrate planning on parametric tool path with geometric and kinematic constraints for CNC machining, Int. J. Adv. Manuf. Technol. 90 (2016) 1889-1896, https://doi.org/10.1007/s00170-016-9483-6.

[12] H. Ni, C. Zhang, Q. Chen, S. Ji, T. Hu, Y. Liu, A novel time-rounding-up-based feedrate scheduling method based on S-shaped ACC/DEC algorithm, Int. J. Adv. Manuf. Technol. 104 (2019) 2073-2088, https://doi.org/10.1007/s00170-019- 03882-0.

[13] P. Petráček, B. Vlk, J. Švéda, Linear programming feedrate optimization: adaptive path sampling and feedrate override, Int. J. Adv. Manuf. Technol. 120 (2022) 3625-3646, https://doi.org/10.1007/s00170-022-08708-0.

[14] H. Li, X. Jiang, G. Huo, C. Su, B. Wang, Y. Hu, Z. Zheng, A novel feedrate scheduling method based on Sigmoid function with chord error and kinematic constraints, Int. J. Adv. Manuf. Technol. 119 (2022) 1531-1552, https://doi.org/ 10.1007/s00170-021-08092-1.

[15] F. Liang, G. Yan, F. Fang, Global time-optimal B-spline feedrate scheduling for a two-turret multi-axis NC machine tool based on optimization with genetic algorithm, Robot. Comput. Integr. Manuf. 75 (2022), 102308, https://doi.org/ 10.1016/j.rcim.2021.102308.

[16] G. Xiong, Z.L. Li, Y. Ding, L.M. Zhu, Integration of optimized feedrate into an online adaptive force controller for robot milling, Int. J. Adv. Manuf. Technol. 106 (2020) 1533-1542, https://doi.org/10.1007/s00170-019-04691-1.

[17] J. Xiao, S. Liu, H. Liu, M. Wang, G. Li, Y. Wang, A jerk-limited heuristic feedrate scheduling method based on particle swarm optimization for a 5-DOF hybrid robot, Robot. Comput. Integr. Manuf. 78 (2022), 102396, https://doi.org/10.1016/j.rcim.2022.102396.

[18] G. Li, G. Liu, T. Huang, J. Han, J. Xiao, An effective approach for non-singular trajectory generation of a 5-DOF hybrid machining robot, Robot. Comput. Integr. Manuf. 80 (2023), 102477, https://doi.org/10.1016/j.rcim.2022.102477.

[19] J. Munoa, X. Beudaert, Z. Dombovari, Y. Altintas, E. Budak, C. Brecher, G. Stepan, Chatter suppression techniques in metal cutting, CIRP Annals 65 (2016) 785-808, https://doi.org/10.1016/j.cirp.2016.06.004.

[20] Y. Altintas, D. Aslan, Integration of virtual and on-line machining process control and monitoring, CIRP Annals 66 (2017) 349-352, https://doi.org/10.1016/j.cirp.2017.04.047.

[21] S.G. Kim, E.Y. Heo, H.G. Lee, W. Kim, D.W. Kim, An adaptive control for NC machining using reference control load curves. Flexible automation and intelligent manufacturing: the human-data-technology nexus, in: Proceedings of FAIM 2022, June 19-23, 2022 2, Lecture Notes in Mechanical Engineering, Detroit, Michigan, USA, 2023, pp. 180-190, https://doi.org/10.1007/978-3-031-17629-6_20.

<!-- Meanless: 10-->

