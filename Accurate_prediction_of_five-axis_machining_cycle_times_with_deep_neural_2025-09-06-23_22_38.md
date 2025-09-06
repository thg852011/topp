

<!-- Meanless: CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41 Contents lists available at ScienceDirect CIRP Journal of Manufacturing Science and Technology ELSEVIER journal homepage: www.elsevier.com/locate/cirpj updates-->

# Accurate prediction of five-axis machining cycle times with deep neural networks using Bi-LSTM

Shih-Hsuan Chien \( {}^{a} \) , Burak Sencer \( {}^{a, * } \) , Rob Ward \( {}^{b,c} \)

\( {}^{a} \) School of Mechanical,Industrial and Manufacturing Engineering,Oregon State University,Corvallis,OR 97331,USA

\( {}^{b} \) University of Sheffield,Advanced Manufacturing Research Centre,University of Sheffield,Rotherham S60 5TZ,United Kingdom

\( {}^{c} \) Department of Automatic Control and Systems Engineering,University of Sheffield,Sheffield S1 3JD,United Kingdom

## ARTICLEINFO

Keywords:

Cycle-time prediction

5-Axis machining

Machine learning

## A B S T R A C T

This paper presents a novel machine learning (ML) based approach to predict machining cycle (part program running) times for complex 5-axis machining. Typical 5-axis machining toolpaths consist of short-segmented discrete linear cutter location lines (CL-lines) with simultaneously varying tool center point (TCP) and orientation vectors (ORI). As the 5-axis machine tool NC (numerical control) system tries to interpolate such part programs smoothly, the TCP motion decelerates and accelerates repeatedly causing the actual feedrate to fluctuate. The actual observed (resultant) feedrate can be approximately 30% lower than the user commanded one. Furthermore, if the toolpath requires the 5-axis machine tool to travel closer to its singular point, or if the workpiece placement on the table is not optimal, actual feedrate is even lowered further. This paper presents two ML-based approaches to accurately predict 5-axis machining toolpaths. The first strategy uses a bidirectional long short-term memory (Bi-LSTM) network to model the machine tool behavior and generates a direct final cycle time estimation for any given part program. This approach only uses the toolpath geometry to predict the cycle time, and for training it only needs the "final" cycle times of similar part programs making it practical on today's shop floors. The second strategy requires individual CL-line processing times for training. In return, it can provide highly accurate cycle time predictions. Both strategies can capture the interpolation dynamics of 5-axis machine tool NC systems accurately. Simulation studies and experimental validations are conducted on modern 5-axis machine tools with varying workpiece placements and interpolation parameters. Proposed approaches have shown to predict cycle times with 90-95 % accuracy on real-life complex 5-axis machining toolpaths.

## 1. Introduction

5-axis machine tools are built on 3-axis Cartesian kinematics with the addition of 2 rotary drives that allow simultaneous control of tool axis orientation (ORI) along with the tool center point (TCP) position. This enables 5-axis machine tools to increase productivity in machining complex sculptured surfaces of dies and molds, monolithic aerospace structures and impellers and blades with freeform surfaces. However, simulating actual 5-axis machine tool motion and predicting part program cycle (run) times of complex 5-axis operations has been a long-lasting problem [1]. Due to synchronous 6DOF motion of the tool, singularity in the kinematic chain [2] and the densely discretized linear toolpath format [3] favored in most 5-axis milling operations, machining feedrates in 5-axis part programs fluctuate heavily [4], and as a result the actual motion duration, e.g. practical cycle times, cannot be predicted accurately. This paper presents a machine learning (ML) based strategy to predict 5-axis machining cycle times accurately.

It should be noted that machining cycle time prediction is essential for overall manufacturing process planning and generating accurate manufacturing cost estimates, so-called "part cost quotations". For small to mid-sized metal cutting industries, machining cycle times are used directly for calculating machine and operator time usage and tooling cost predictions as well as to make delivery commitments. For larger enterprises, cycle time predictions play a key role in overall manufacturing process planning, which involves optimizing material flow and machine allocation [5].

Computer Aided Manufacturing (CAM) systems are the state-of-the-art tools for cycle time prediction [6]. They are used for both generating 5-axis machining part programs and predicting the 5-axis machining cycle times. Nevertheless, CAM based cycle time predictions are typically highly inaccurate as they yield \( {30} - {50}\% \) errors for complex machining operations. The fundamental reason for this inaccuracy is the fact that CAM systems use kinematic simulations of the tool and workpiece without considering the machine and the NC system dynamics. In particular, the NC system of the machine tool dictates how the tool motion is generated along a toolpath and the feedrate is planned with respect to the machine's dynamic and kinematic limits.

---

<!-- Footnote -->

* Corresponding author.

E-mail address: Burak.Sencer@oregonstate.edu (B. Sencer).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.cirpj.2023.11.007 Received 6 November 2023; Received in revised form 21 November 2023; Accepted 24 November 2023 Available online 7 December 2023 1755-5817/(C) 2023 CIRP.-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

<!-- Media -->

<!-- figureText: a) TCP Motion in Cartesian Coord. <TCP Motion 60 40 20 -20 x-axis [mm] 0 -40 <ORI Motion 0.5 0 j-axis Time (sec) Actual cycle time 100 80 z-axis [mm] 60 40 20 0 -20 100 Motion Dir. 80 60 y-axis [mm] 40 20 b) ORI Motion in Spherical Coord. k-axis 0.5 Motion Dir. 0 i-axis -0.5 -0.5 -1^-1 c) Resultant Feedrate Profile Commanded Feedrate (F) F Feedrate (mm/sec) CAM-based cycle time -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_1.jpg?x=118&y=148&w=726&h=1639&r=0"/>

Fig. 1. Feedrate profile on complex 5-axis toolpath.

<!-- Media -->

It should be noted that once the part program is optimized with respect to process requirements such as chip load, cutting forces, vibrations, tool life and surface finish, the final part program is obtained and uploaded to the 5-axis machine tool. The NC system of the machine parses this process-optimized toolpath and then interpolates the tool motion. This step determines the final cycle time for a part program. Typical 5-axis machining toolpaths consist of densely discretized linear TCP and ORI motion commands. The NC system utilizes a combination of digital filtering [7], kinematic and geometric smoothing [8] and optimization algorithms [9] to realize a smooth and continuous motion that respects kinematic limits of all the axes.

An example behavior is shown in Fig. 1, the machining toolpath is given in workpiece/tool coordinate system composed of Cartesian TCP data points and ORI vectors. The beginning of the toolpath shows some long CL-lines and feedrate drops at the corners where the feed direction of the TCP is changed from one CL-line to the next. It should be noted that similar behavior is observed when the angular feed direction of the ORI is also altered. Another interesting observation is that the NC system plans the feedrate drop differently when the CL-line lengths are shorter, e.g. along short-segmented sections. Please note that the final tool motion smoothly and accurately follows the interpolated CL data points within a tolerance band/tube. This tolerance band is usually controlled by the end-user depending on the process requirements [10]. The final feedrate profile is highly fluctuating, and it can be observed that feed fluctuations are correlated (linked) to the path geometry; namely, when there are sharp changes in the linear or angular feed direction the fee-drate drops severely. This behavior elongates the cycle time and causes CAM systems to underestimate resultant 5-axis machining cycle times dramatically.

Modeling NC system's interpolation behavior discussed above is key to accurately predict actual machining cycle times in 5-axis operations. Most manufacturing literature has focused on cycle time predictions for 3-axis machining. Since the interpolation algorithms running in the NC system kernels are not disclosed or explained to the end-user, first principle's based modeling is a challenge. Inspired from the observations as presented in Fig. 1, Lee and So tried to find direct correlations between the part geometry and overall cycle time elongation. They reported that the angle between successive CL lines play a critical role [11]. Toolpath geometries with obtuse transitions between CL lines do not cause large feed fluctuations, whereas for acute/sharp changes in the feed direction causes significant elongation in cycle time. More recently, Altintas and Tulysen tried to directly model the NC system's interpolation behavior. They assumed that the NC system uses a low-pass filter to smoothly interpolate the part program and they tried to identify this "smoothing filter" dynamics [12]. The identified filter delay is then used to predict the cycle time. However, their approach worked only when certain conditions were met. For instance, i) the toolpath should not be close to any singularity so that axis kinematic limits are never imposed, and ii) interpolation tolerance is never utilized. In practice these conditions are almost never ever satisfied, rendering the method to be inaccurate and not practical. Ward and Sencer tried to address that by considering the effect of interpolation tolerance on 3-axis machining cycle times [13]. They were able to accurately predict cycle times for simple toolpaths with varying interpolation tolerances. However, their method also failed to provide accurate cycle time predictions when the toolpaths contain short-segmented CL-lines, which is used in most of the 5-axis part programs.

Apart from those first principles-based approaches, ML learning based modeling has been pursued to predict machining cycle times as well. Sun et al. [14] were the first ones to apply ML to predict 3-axis machining cycle time predictions and they showed the potential to generate accurate cycle time predictions for complex machining case scenarios. Later, Endo and Sencer used artificial neural networks (ANNs) to predict how much NC system lowers the feedrate along various CL line transitions, and how it schedules its acceleration and jerk limits accordingly [15]. Once trained with data, the approach can provide very accurate cycle time predictions along complex 3-axis toolpaths for any given blending tolerance. However, their approach required large amounts of data and lengthy pre-processing. Later, Chien and Sencer proposed to use a bidirectional neural network and demonstrated good accuracy in 3-axis machining toolpaths with short-segmented part programs [16].

<!-- Meanless: 29-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

<!-- Media -->

<!-- figureText: a) Non-stop tool motion interpolation Tool-Ti Blending Error ORI Motion in Spherical Coord. k nterpolated ORI Motion \( {F}_{2} \) \( k + 1 \) th block Time \( {\omega }_{2} \) Time TCP Motion in Cartesian Coord. Z Cornering Angle Interpolated \( {\theta }_{TCP} \) TCP Motion y ✘ b) Feedrate profiles \( {F}_{1} \) TCP Feedrate \( {k}^{th} \) block \( {F}_{1} * {F}_{2} * \) Angular Feedrate \( {\omega }_{i} \) \( {\omega }_{1}^{ * } \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_2.jpg?x=108&y=147&w=737&h=1579&r=0"/>

Fig. 2. Non-stop interpolation of 5-axis machining toolpaths.

<!-- Media -->

It should also be mentioned that there are PC-based simulators for NC systems. NC makers such as Siemens, Heidenhain and Mitsubishi provide a copy of their NC kernels for motion simulation on external PCs. These software systems can provide very precise prediction of the cycle times and feed profile for any given toolpath since they simply run a copy of the actual NC kernel from the CNC. However, they come with major drawbacks that limit their practical usage: i) they are very expensive and ii) since they simply run a copy of the actual NC kernel on a PC, it requires the same amount of time as conducting an air-cutting test on an actual machine tool. In other words, for long toolpaths the use of these digital-twins is not practical.

This paper, for the first time proposes a ML-based cycle time prediction for 5-axis part programs. It presents the following the contributions:

- A simple Bi-LSTM neural network model is presented to predict 5- axis machining cycle time, which can handle both simple and complex, short-segmented part programs including simultaneous translational and rotational tool motion.

- The effect of singularity and workpiece location on the worktable on the final cycle time is captured in the model to accurately predict cycle times.

- A practical strategy is presented, which only requires the final cycle time of part programs for training.

## 2. Machine learning-based 5-axis machining cycle time prediction

### 2.1. Effect of path blending on feed drop

This section presents fundamental analysis on NC system's interpolation to determine its key features for accurately prediction the feed drop by an ML-based model.

5-axis part programs command linear motion (interpolation) of the TCP and ORI either in the machine tool, or in the tool/workpiece coordinates. Typical modern accurate 5-axis machining part programs favor tool/workpiece coordinates-based definition where CL data points \( \left( {k = 1\ldots M}\right) \) are defined by Cartesian positions \( {P}_{k} = \left\lbrack  {{P}_{x},{P}_{y},{P}_{z}}\right\rbrack \) and tool axis orientation \( {O}_{k} = \left\lbrack  {{O}_{i},{O}_{j},{O}_{k}}\right\rbrack \) vectors (ORI) are given in spherical coordinates as shown in Fig. 1. The block processing (cycle) time \( {T}_{\text{cycle },k} \) is computed based on the Euclidian TCP motion as:

\[{T}_{\text{cycle },k} = \frac{{L}_{k}}{F} = \frac{\begin{Vmatrix}{P}_{k + 1} - {P}_{k}\end{Vmatrix}}{F} \tag{1}\]

\[{\omega }_{k} = \frac{{\theta }_{k}}{{T}_{\text{cycle },k}} = \frac{{\cos }^{-1}\left( {{O}_{k + 1} \bullet  {O}_{k}}\right) }{{T}_{\text{cycle },k}} \tag{2}\]

where \( {L}_{k} \) is the Euclidian TCP travel length for the \( {k}^{\text{th }} \) CL-line, \( {\theta }_{k} \) is the angular travel distance of the tool orientation vector,and \( {\omega }_{k} \) is the resultant angular velocity of the block. It should be noted that if no translational motion of the TCP is programmed, or if the inverse-time-feed (ITF) programming strategy [17] is used, the feedrate definition must be revised.

Fig. 1c depicts a realistic example of this toolpath definition. It illustrates how the feedrate drops when the TCP or ORI undergoes a feed direction change along successive CL-lines. Notice that it is critical to determine which parameters/features play a key role in this interpolation behavior and how feed-drops so that they can be captured and learned by the ML model. Fig. 2 illustrates a simpler example where 2 long CL-lines are continuously interpolated, and the tool motion transitions from one block to the next.

Please note that non-stop continuous linear interpolation requires the tool motion to deviate from the CL-lines in order to blend the linear moves smoothly [18]. As depicted in Fig. 2, both the TCP and the ORI motion show so-called blending errors from the commanded CL lines. This deviation must be confined within the user-set interpolation tolerance [19]. Although imposing blending tolerance on the TCP interpolation \( {\epsilon }_{TCP} \) is well-known,the angular ORI deviation tolerance \( {\epsilon }_{ORI} \) must be respected especially in flank milling operations [20]. Here, the non-stop interpolation model developed by Shingo and Sencer [21] is adapted to analyze and characterize the impact of the toolpath geometry and the interpolation tolerance on the feed drop, which impacts the final cycle time.

It is reported that modern NC systems utilize a combination of digital filtering and optimization to plan a continuous feed motion blending successive CL-lines [18]. Let us assume that digital filtering [22] is used where each CL-line is presented by a velocity pulse and low pass filtered to interpolate the tool motion smoothly [23]. An example on how fee-drate drops around CL block transitions are critical in controlling the blending tolerance is shown in Fig. 2b. Feed pulse magnitudes are reduced to a level of \( {F}_{1}^{ * } \) ,and \( {F}_{2}^{ * } \) around the junction point of the CL-lines to slow down the tool motion and satisfy the blending tolerance, \( {\epsilon }_{TCP} \) or \( {\epsilon }_{ORI} \) . It is assumed that the pulse widths around the junction point is half of the digital filter delay \( \frac{{T}_{d}}{2} \) ,and the TCP blending tolerance \( {\epsilon }_{TCP} \) is controlled by lowering the speed around the block junction (transition) points from the kinematics of the motion as described in [21]:

<!-- Meanless: 30-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

\[{\epsilon }_{TCP} = \sqrt{{F}_{1}^{{ * }^{2}} + {F}_{2}^{{ * }^{2}} - 2{F}_{1}^{ * }{F}_{2}^{ * }\cos \left( {\theta }_{TCP}\right) }\left( \frac{3{T}_{1}^{2} + {T}_{2}^{2}}{{24}{T}_{1}}\right)  \tag{3}\]

<!-- Media -->

<!-- figureText: a) Tool tip motion Commanded CL-data Points \( k + 4 \) Time Blending Region Interpolated Trajectory b) Velocity pulse overlap Transition between [k-3](3) k to k+1 \( {}^{th} \) CL-lines Velocity k-1 \( k + 1 \) \( {T}_{d}/2 \) Feed Pulses -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_3.jpg?x=202&y=151&w=551&h=574&r=0"/>

Fig. 3. Blending example of short segmented 5-axis tool-paths.

<!-- Media -->

where \( {\theta }_{TCP} \) is the angle between \( {k}^{th} \) and \( {k}^{k + 1} \) blocks as shown in Fig. 2b. The feed drop percentage \( \alpha \) can be computed as:

\[\alpha  = \frac{\frac{{24}{T}_{1}}{3{T}_{1}^{2} + {T}_{2}^{2}}}{\sqrt{{F}_{1}^{2} + {F}_{2}^{2} - 2{F}_{1}{F}_{2}\cos \left( {\theta }_{TCP}\right) }}{\epsilon }_{TCP} \times  {100} \tag{4}\]

As indicated in Eq. (4) the drop in the feedrate along a TCP feed direction change is controlled by the filter time constants \( {T}_{1},{T}_{2} \) ,commanded feedrates \( {F}_{1},{F}_{2} \) ,user-set interpolation tolerance \( {\epsilon }_{TCP} \) and the cornering angle \( {\theta }_{TCP} \) between CL-lines. Since filter time constants are tuned based on machine tool dynamics and fixed by the machine tool manufacturer, the feed drop is controlled by the interpolation tolerance, commanded feedrate and the toolpath geometry, e.g. the cornering angle between successive CL-lines, \( {\theta }_{TCP} \) . These features must be considered in the ML model. It should also be noted that similar observations can be made for the ORI motion where the feedrate is dropped to satisfy the angular blending tolerance \( {\epsilon }_{ORI} \) ,however it is omitted here. Readers should refer to [21] to perform similar analysis and derive relationships between the feed drop due to tool's angular motion. To conclude, the ML model should be trained to predict the actual cycle time of a CL-line as a function of:

\[{T}_{\text{cycle },k} = f\left( {{F}_{k},{\epsilon }_{TCP},{\epsilon }_{ORI},{\theta }_{{TCP},k},{\theta }_{{ORI},k}}\right)  \tag{5}\]

to support the cumulative prediction of the entire machining cycle time.

The above analysis is performed for only long CL-lines. Practical 5- axis machining toolpaths consist of dense short-segmented CL data points as shown in Fig. 3, and the NC system simply blends those CL-lines together. The interpolation strategy along such short-segmented sections is called the "global blending" [24], and it is designed to provide smoother and faster tool motion [25]. When the NC system executes a global smoothing, the final cycle time along those short-segmented blocks is not only controlled by a single \( \left( {k}^{th}\right) \) CL-line but rather by a multiple of them, \( k - n,\ldots ,k,\ldots ,k + m \) . Fig. 3 is used to explain this behavior for digital filtering based interpolation approach. Each CL-line is presented by a short velocity pulse as shown in Fig. 3b, and trapezoidal profiles represent the individual transient feed pulse responses.

<!-- Media -->

<!-- figureText: WCS Tool \( {Y}_{M} \) Table Inverse Kinematics Transformation \( {\left\lbrack  {P}_{x},{P}_{y},{P}_{z},{O}_{i},{O}_{j},{O}_{k}\right\rbrack  }^{T} \) Forward Kinematics Transformation \( {\left\lbrack  X,Y,Z,A,C\right\rbrack  }^{T} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_3.jpg?x=907&y=154&w=722&h=630&r=0"/>

Fig. 4. 5-axis machine tool kinematic model with rotating table configuration.

<!-- Media -->

As shown, as the CL-lines get shorter, their feed pulse response becomes highly overlapped. Depending on the NC system's look-ahead buffer size and the filtering strategy used [23], \( n \) previous and \( m \) next CL lines affect the feedrate at feedrate at the \( {k}^{th} \) segment. Furthermore, the NC system can apply optimization procedures to lower feed fluctuations and generate a smooth feedrate profile [26]. Therefore, cycle times of a short-segmented 5-axis machining toolpath segment should at least consider the feedrate \( F \) and toolpath geometry from neighboring blocks \( {k}^{\text{th }} \) and \( k + {1}^{\text{th }} \) as:

\[{T}_{\text{cycle },k} = f\left( {{F}_{k},{\epsilon }_{TCP},{\epsilon }_{ORI},{\theta }_{{TCP},k},{\theta }_{{ORI},k},{\theta }_{{TCP},k + 1},{\theta }_{{ORI},k + 1}}\right)  \tag{6}\]

Please note that Eq. (6) only considers the effect of current \( {k}^{\text{th }} \) and next \( {k}^{k + 1} \) blocks. It can be extended to consider more blocks for greater accuracy, which will be discussed later in the neural network design section.

### 2.2. Effect of singularity on feed drop

The previous subsection provided basic analysis on the relationship between feed drop and the toolpath geometry, which allows feature selection to build ML models for accurately predicting machining cycle times. This section analyzes the impact of singular points on the cycle time elongation.

Typical 5-axis machine tools contain 2 rotary drives (A (or B) and C) placed either on the tool or the workpiece side generating rotations along the X, Y or the Z-axes [27] [28]. Although there are various combinations possible [29], most common 5-axis machines are built on the AC or BC kinematic structure where the C axis provides a rotation around the \( \mathrm{Z} \) axis,and \( \mathrm{A} \) and \( \mathrm{B} \) axes provide rotations around the \( \mathrm{X} \) or the \( \mathrm{Y} \) axis,respectfully. This allows any desired tool orientation vector (ORI) to be realized. The relationship between the machine coordinate system (MCS) and workpiece coordinate system (WCS) for a five axis AC configuration machine tool is shown in Fig. 4.

<!-- Meanless: 31-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

<!-- Media -->

<!-- figureText: a) Rotary axes definition b) Orientation kinematics definition 1 k-axis -1 -1 - 1 0 j-axis i-axis k-axis 0 \( j \) j-axis i-axis -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_4.jpg?x=334&y=150&w=1079&h=593&r=0"/>

Fig. 5. Tool orientation kinematics.

<!-- Media -->

In Fig. 5, an AC rotary drive configured 5-axis machine tool and the respective commanded ORI \( \left( {O = \left\lbrack  {{O}_{i},{O}_{j},{O}_{k}}\right\rbrack  }\right) \) is shown,and the corresponding forward and inverse kinematics can be derived as [30]:

\[\left\{  \begin{matrix} {O}_{i} = \sin \left( A\right) \sin \left( C\right) \\  {O}_{j} = \sin \left( A\right) \cos \left( C\right) \\  {O}_{k} = \cos \left( A\right)  \end{matrix}\right\}   \rightarrow  \left\{  \begin{array}{l} A = {\cos }^{-1}\left( {O}_{k}\right) . \\  C = {\tan }^{-1}\left( \frac{{O}_{i}}{{O}_{j}}\right) . \end{array}\right.  \tag{7}\]

Notice from Eq. (7) and Fig. 5a that when \( \sin \left( A\right)  = 0 \) ,the tool orientation vector is aligned with the Z-axis of the machine, \( O = \left\lbrack  {{O}_{i},{O}_{j}}\right. \) , \( \left. {O}_{k}\right\rbrack   = \left\lbrack  {0,0,1}\right\rbrack \) ,and the \( C \) axis rotation cannot alter the ORI. Furthermore,at this orientation the \( C \) -axis position cannot be solved since \( {\tan }^{-1}\left( {0/0}\right) \) is undefined. This particular orientation when \( \sin \left( A\right)  = 0 \) (or \( O = \left\lbrack  {0,0,1}\right\rbrack \) ),is known as the singular point of a 5-axis machine tool. Typically, CAM systems generate toolpaths that do not pass through the singular point, or singularity avoidance strategies [23] are utilized by the NC system to circumvent it in real-time.

Although the singular point itself can be avoided by not passing directly through it, it still has an impact on the feed drop and slowing

down the feed motion. It is known that when tool orientation vectors are passing even close to the singular point, rotary axes demand high velocity and acceleration [31]. As a result, the NC system slows down the feed motion to avoid any actuator saturation, leading to an elongated 5-axis machining cycle time. The ML models must capture this characteristic for accurately predicting the cycle times.

Fig. 5b illustrates the differential relationship between the ORI and the rotary drive motion. Angular velocity of ORI can be given as [32]:

\[\frac{\delta \mathbf{O}}{\delta t} = \frac{\delta {\theta }_{v}}{\delta t}\overrightarrow{{\mathbf{t}}_{v}} + \frac{\delta {\theta }_{h}}{\delta t}\overrightarrow{{\mathbf{t}}_{h}} \tag{8}\]

where \( {\theta }_{v} \) and \( {\theta }_{h} \) represent the longitudinal and latitudinal displacement on the unit sphere and \( \overrightarrow{{t}_{v}},\overrightarrow{{t}_{h}} \) are unit vectors as shown in Fig. 5b. The rotary drive speeds can then be solved from Eqs. (7) and (8) as:

\[\left\{  \begin{matrix} \frac{\delta A}{\delta t} = \frac{\delta A}{\delta {\theta }_{v}}\frac{\delta {\theta }_{v}}{\delta t} = \frac{\delta {\theta }_{v}}{\delta t}. \\  \frac{\delta C}{\delta t} = \frac{\delta C}{\delta {\theta }_{h}}\frac{\delta {\theta }_{h}}{\delta t} = \frac{1}{\sin \left( A\right) }\frac{\delta {\theta }_{h}}{\delta t}. \end{matrix}\right.  \tag{9}\]

<!-- Media -->

<!-- figureText: [Xo,Yo,Zo] \( {Y}_{M} \) \( {X}_{W} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_4.jpg?x=329&y=1469&w=1083&h=675&r=0"/>

Fig. 6. Coordinate transformation determining workpiece offset.

<!-- Meanless: 32-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

<!-- figureText: Output Layer: Tcycle,k-n Tcycle, \( k \) ... Tcycle, \( k + m \) \( {bwdk} \) bwdk+m fwdk fwdk+m INk ... \( {INk} + m \) Backward Layer: bwdk-n Forward Layer: fwdk-n Input Layer: INk-n ... -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_5.jpg?x=335&y=151&w=1072&h=394&r=0"/>

Fig. 7. Sequence-to-sequence Bi-LSTM structure.

<!-- Media -->

Eq. (9) presents a crucial relationship between the \( C \) -axis speed and ORI. It indicates that as the tool orientation gets closer to the singular point, or as the tool orientation changes on a longitude close to the north pole of the unit circle \( \left( \left\lbrack  {0,0,1}\right\rbrack  \right) \) ,the C-axis velocity increases. This behavior results in slowing down the overall feed motion to avoid saturating C-axis drives during high-speed 5-axis machining. As a result, it can be concluded that overall cycle time is also affected because of the \( 1/\sin \left( A\right) \) term in Eq. (9).

### 2.3. Effect of workpiece location

In practice it has been observed that workpiece location and tool length also play a role in elongating 5-axis machining cycle times [33, \( {34},{35}\rbrack \) . When the rotary drives are placed on the tool side,the tool length is a critical feature correlated with cycle time variation [36]. When rotary axes are on the workpiece side, the workpiece location on the machine table affects overall machining cycle times. In this work, we assume the rotary drives are on the workpiece side and determine the key factors, i.e. features, that needs to be correlated to the cycle time variation.

Typical 5-axis part programs (CL files) contain the TCP and ORI data given in WCS. Cartesian X,Y,Z axis positions are then interpolated based on the inverse kinematics of the machine tool, and the placement of the WCS \( \left\lbrack  {{X}_{W},{Y}_{W},{Z}_{W}}\right\rbrack \) and MCS \( \left\lbrack  {{X}_{M},{Y}_{M},{Z}_{M}}\right\rbrack \) as shown in Fig. 6. It should be noted that Fig. 6 assumes that origin of the A and C rotary axes coincide, and the origins of the WCS and MCS are offset by \( \left\lbrack  {{X}_{1},{Y}_{1},{Z}_{1}}\right\rbrack \) and \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack \) ,respectively. The Cartesian axis positions can then be computed as [34]:

\[\left\lbrack  \begin{array}{l} {X}_{M} \\  {Y}_{M} \\  {Z}_{M} \end{array}\right\rbrack   = \operatorname{Tran}\left( {{X}_{1},{Y}_{1},{Z}_{1}}\right)  \bullet  \operatorname{Rot}\left( {X,A}\right)  \bullet  \operatorname{Rot}\left( {Z,C}\right)  \bullet  \operatorname{Tran}\left( {{X}_{o},{Y}_{o},{Z}_{o}}\right)  \bullet  \left\lbrack  \begin{array}{l} {X}_{W} \\  {Y}_{W} \\  {Z}_{W} \end{array}\right\rbrack  \]

(10)

where Trans and Rot are the homogeneous translation and rotations.

It should be noted that the workpiece and tool offsets from the origin of the rotary axes affect the Cartesian axis positions of the 5-axis machine tool. Placing the workpiece further away from the rotary axis origin (larger workpiece offset) would elongate the overall Cartesian axis travel and thereby alter the peak velocity and acceleration level. This would all effect the final cycle times and the final cycle time should be presented as a function of:

\[{T}_{\text{cycle },k} = f\left( {\left( {{X}_{o},{Y}_{o},{Z}_{o}}\right) ,{F}_{k},{L}_{k},{\epsilon }_{TCP},{\epsilon }_{ORI},{\theta }_{k},{\theta }_{{TCP},k},{\theta }_{{ORI},k},{\theta }_{{TCP},k + 1},{\theta }_{{ORI},k + 1},\sin \left( A\right) }\right) \]

(11)

Finally, it should also be noted that the TCP location within the workpiece coordinates affects the final cycle time. For large workpieces the average TCP location, \( \left\lbrack  {{\bar{P}}_{x},{\bar{P}}_{y},{\bar{P}}_{z}}\right\rbrack   = \left\lbrack  {\frac{1}{M}\mathop{\sum }\limits_{{k = 1}}^{M}{P}_{x,k},\frac{1}{M}\mathop{\sum }\limits_{{k = 1}}^{M}{P}_{y,k},\frac{1}{M}\mathop{\sum }\limits_{{k = 1}}^{M}{P}_{z,k}}\right\rbrack \) should be added to the workpiece offset \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack \) to increase the accuracy.

A larger workpiece translation distance \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack \) can lead to longer and more rapid MCS movements for the same WCS-defined toolpath, which may increase overall cycle time. Thus, the workpiece translation distance \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack \) is another critical factor in neural network design for cycle time prediction.

### 2.4. Neural network based accurate cycle time prediction

This section presents the design of neural networks to predict 5-axis machining cycle times by considering the effects of NC systems blending behavior, singularity, and workpiece placement. Two approaches are presented; firstly, a block-by-block cycle time prediction strategy, which precisely predicts the cycle time along each CL-line (G-block), and secondly, a direct final cycle-time prediction strategy, which predicts the total (final) cycle time for the entire toolpath.

It should be noted that the direct final cycle time prediction is simply not the summation of the cycle times predicted for each CL-line. The approach is significantly different. The block-by-block cycle time prediction strategy requires cycle times for each CL-line to be measured during the training, which demands the NC system's monitoring function to record those data or external sensors to be attached to the machine tool. However, the direct final cycle time prediction strategy only requires the total (final) cycle time for the part program to be recorded for training purposes, which can be done simply by a stopwatch. Therefore, it is highly practical for shop floor implementation on any machine without any additional instrumentation. The following sections present the neural network (NN) designs and training protocols for these approaches.

Design of the NN plays a key role in accurately predicting the cycle times. Eq. (11) for instance,indicates that the cycle time of the \( {k}^{\text{th }}\mathrm{{CL}} \) - line is influenced by the interpolated motion due to multiple, \( n \) past and \( m \) future,CL-lines. Thus,the Neural Network (NN) architecture must consider the information from multiple previous and future CL-lines for accurately predicting the cycle time. Traditional Artificial Neural Networks (ANN) typically focus on the present time step to make prediction [37]. Although they can be designed to include information from multiple CL-lines [15], it is challenging to tune how many CL-lines to include and determine the number of neurons. While Recurrent Neural Networks (RNNs) can utilize past inputs to influence the current output, they cannot incorporate future data [38]. To learn the relationship between the influence of future and past data information on current block cycle time, bidirectional long short-term memory network (Bi-LSTM) provide a more effective network structure since they inherently incorporate both forward and backward layers for considering past and future inputs simultaneously [39]. This capability makes Bi-LSTM proficient at capturing dependencies between various trajectory segments, ensuring precise block cycle time predictions in scenarios where past and future data both play a crucial role. Furthermore, the Bi-LSTM network can be configured for "block-by-block cycle time prediction", or the "direct final cycle time prediction" by minor alteration of the network structure.

<!-- Meanless: 33-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

<!-- Media -->

<!-- figureText: Backward Layer: bwd1 Output Layer: T bwds bwdm fwd3 fwdm IN3 INM Forward Layer: fwd1 Input Layer: IN1 -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_6.jpg?x=328&y=153&w=1084&h=406&r=0"/>

Fig. 8. Sequence-to-last Bi-LSTM structure.

<!-- figureText: \( {F}_{k} \) \( {T}_{\text{cycle },k} \) (for block-by-block cycle time prediction) or \( T \) (for direct final cycle time prediction) \( {L}_{k} \) \( {\theta }_{{TCP},k} \) \( {\theta }_{{TCP},k + 1} \) \( {\epsilon }_{TCP} \) Output \( {\theta }_{k} \) layer \( {\theta }_{{ORI},k} \) \( {\theta }_{{ORI},k + 1} \) \( {\epsilon }_{ORI} \) \( \sin \left( A\right) \) or sin (B) \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack \) Input Forward Backward -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_6.jpg?x=332&y=647&w=1089&h=709&r=0"/>

Fig. 9. Bi-LSTM structure for cycle time prediction.

<!-- Media -->

The Bi-LSTM network is used for the block-by-block cycle time prediction, which tries to accurately predict the cycle time for each CL-line in the part program. The Bi-LSTM network employing a sequence-to-sequence structure [40] is depicted in Fig. 7. The forward layer processes input data from past to future, while the backward layer operates in the reverse direction, handling input data from the future to the past, as indicated by the arrow direction. Within each layer, multiple hidden neurons are employed as memory units. These hidden neurons are denoted as \( \left( {{fw}{d}_{k - n},\ldots ,{fw}{d}_{k},\ldots ,{fw}{d}_{k + m}}\right) \) in the forward layer,and \( \left( {{bw}{d}_{k - n}}\right. \) , \( \left. {\ldots ,{bw}{d}_{k},\ldots ,{bw}{d}_{k + m}}\right) \) in the backward layer. The subscripts,such as \( k - n,k \) , \( k + m \) ,denote the hidden neurons responsible for processing information of each CL-line. The purpose of the hidden neurons is to capture and learn temporal relationships between the input and the output target, which is the block cycle time of each CL-line. This enables the prediction of block-by-block cycle times \( \left( {{T}_{\text{cycle },k - n},\ldots ,{T}_{\text{cycle },k},\ldots ,{T}_{\text{cycle },k + m}}\right) \) based on inputs from multiple CL-lines \( \left( {I{N}_{k - n},\ldots ,I{N}_{k},\ldots ,I{N}_{k + m}}\right) \) .

Another approach is to utilize the Bi-LSTM network with sequence-to-last structure for directly predicting the final cycle time, \( T \) \( = \mathop{\sum }\limits_{{k = 1}}^{{k = M}}{T}_{\text{cycle },k} \) . The network architecture [41,42] is depicted in Fig. 8. It is similar to the sequence-to-sequence structure used for block-by-block cycle time prediction. The forward layer processes input data from past to future, while the backward layer operates in the reverse direction, handling input data from the future to the past, as indicated by the arrow direction. Each layer features multiple hidden neurons,denoted as \( \left( {{fw}{d}_{1}}\right. \) , \( \left. {\ldots ,{fw}{d}_{3},\ldots ,{fw}{d}_{M}}\right) \) in the forward layer,and \( \left( {{bw}{d}_{1},\ldots ,{bw}{d}_{3},\ldots ,{bw}{d}_{M}}\right) \) in the backward layer. The subscripts,such as \( 1,3,M \) ,denote the hidden neurons responsible for processing information of each CL-line.

<!-- Media -->

Table 1

Bi-LSTM inputs and related variables.

<table><tr><td>Variables - Orientation angles at CL-line end \( \left( {\theta }_{{ORI}.k + 1}\right) \) - ORI interpolation tolerance \( \left( {\epsilon }_{ORI}\right) \)</td><td>Bi-LSTM Inputs</td></tr><tr><td>TCP Information ORI Information</td><td>- Commanded feedrate \( \left( {F}_{k}\right) \) - Nominal CL-line length \( \left( {L}_{k}\right) \) - Cornering angles at CL-line start \( \left( {\theta }_{{TCP}.k}\right) \) - Cornering angles at CL-line end \( \left( {\theta }_{{TCP}.k + 1}\right) \) - TCP interpolation tolerance \( \left( {\epsilon }_{TCP}\right) \) - Orientation arc length \( \left( {\theta }_{k}\right) \) - Orientation angles at CL-line start \( \left( {\theta }_{{ORI},k}\right) \)</td></tr><tr><td>Singularity Effects</td><td>- sin(B) or sin(A) based on rotary axis configuration (BC or \( {AC} \) )</td></tr><tr><td>Workpiece Location Variations</td><td>- WP translation distance to the rotary axis' origin \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack \)</td></tr></table>

<!-- Media -->

The sequence-to-last structure predicts a single output after processing the hidden neurons responsible for all CL-lines. The hidden neurons summarize the network's learned relationship between the input data \( \left( {I{N}_{1},\ldots ,I{N}_{3},\ldots ,I{N}_{M}}\right) \) and the output target,which is the final cycle time \( T \) .

<!-- Meanless: 34-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

<!-- Media -->

<!-- figureText: a) TCP Motion in Cartesian Coord. b) ORI Motion in Spherical Coord. ORI Motion Singular Point 1 k-axis -1 0 - 1 0 1 0 j-axis i-axis -1 TCP Motion local blending z-axis [mm] 280 global blending 260 240 50 0 y-axis [mm] -50 0 50 -100 -50 x-axis [mm] -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_7.jpg?x=341&y=150&w=1066&h=456&r=0"/>

Fig. 10. Mixed blending training toolpaths near singularity.

<!-- figureText: TCP information Part program for training (G-code) Features extraction Measured block cycle time of each G-line Training stage Output target Prediced block cycle time MSE loss between measured and predicted data Meet the number of iterations? Yes, finish training ORI information Singularity effects WP variations Input for BiLSTM LSTM No, update weights -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_7.jpg?x=334&y=717&w=1080&h=723&r=0"/>

Fig. 11. Training process of block-by-block cycle time prediction.

<!-- Media -->

The inputs to both Bi-LSTM networks are based on the key features determined to affect the cycle time presented in the previous section and given in Eq. (11). They are also summarized in Fig. 9 and Table 1. For instance,the commanded feedrate \( {F}_{k} \) ,nominal CL-line length \( {L}_{k} \) ,cornering angles at the start \( {\theta }_{{TCP},k} \) and end \( {\theta }_{{TCP},k + 1} \) of each CL-line,and TCP interpolation tolerance \( {\epsilon }_{TCP} \) collectively affect block cycle times. Notice that smaller \( {\epsilon }_{TCP} \) values yield less deviation from the toolpath but may result in longer block cycle times as the machine needs to greatly slow down to blend the toolpath more accurately. Similarly, ORI arch lengths \( {\theta }_{k} \) ,the angle between successive CL-lines, \( {\theta }_{{ORI},k} \) and \( {\theta }_{{ORI},k + 1} \) ,and the interpolation tolerance \( {\epsilon }_{ORI} \) control the cycle time elongation. Finally, the effect of operating close to singularity, and the workpiece location are modeled by the network by introducing \( \sin \left( B\right) \) (or \( \sin \left( A\right) \) ) terms as well as the workpiece (WP) translation distance to the origin of the rotary axis \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack \) as inputs to the Bi-LSTM.

### 2.5. Network training protocol

The Bi-LSTM cycle time prediction networks are trained by executing dedicated training part programs or alternatively by simply making use of the part programs used in production. The use of dedicated training part programs helps jump-start the network training for higher accuracy and minimize overfitting. These dedicated training part programs involve toolpaths featuring long-segmented CL-lines triggering both local and global blends, deliberately exacerbating the effects of singular point and varying workpiece location.

The training toolpaths consist of CL-line lengths \( {L}_{k} \in  \left( {0,{100}}\right) \mathrm{{mm}} \) and cornering angles \( {\theta }_{TCP} \in  \left( {0,\pi }\right) \) rad. They are comprised of \( {75}\% \) local blend trajectories and \( {25}\% \) global blend trajectories,allowing the Bi-LSTM network to learn behaviors associated with both blending strategies. The training toolpaths designed to investigate the influence of singular points is depicted in Fig. 10. These toolpaths are intentionally positioned close to singular points,corresponding to \( {O}_{k} \in  \left\lbrack  {{0.7},{0.99}}\right\rbrack \) , which is equivalent to \( 1/\sin \left( \mathrm{A}\right)  \in  \left\lbrack  {{1.4},{15}}\right\rbrack \) .

The designed training toolpaths are used to study the effects of different workpiece locations. These toolpaths involve systematic shifts of the workpiece to various positions, with workpiece translation distance ranging from \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack  \left\lbrack  \mathrm{{mm}}\right\rbrack   = \left\lbrack  {{500},{500},{500}}\right\rbrack \) to \( \lbrack {1000},{1000} \) , 1000]. The goal is to enable the Bi-LSTM network to understand and account for the impact of different workpiece translation distances on block cycle time profiles while using the same toolpaths.

<!-- Meanless: 35-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

<!-- Media -->

<!-- figureText: a) TCP Motion in Cartesian Coord. TCP Motion -20 -40 -60 0 [mm] CORI Motion 0 1 i-axis 40 50 60 70 CL-line number z-axis [mm] 280 260 240 220 y-axis [mm] 148 20 136 60 40 b) ORI Motion in Spherical Coord 1 0.5 k-axis 0 -0.5 1 i-axis - 1 - 1 c) Cycle Time Prediction 0.7 Block cycle time (sec) 0.6 Measured 0.5 Predicted CAM 0.4 0.3 0.2 0. 10 20 30 -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_8.jpg?x=119&y=150&w=717&h=1659&r=0"/>

Fig. 12. Simulation test toolpath-I, and cycle time prediction results for \( F = {1000}\mathrm{\;{mm}}/\mathrm{{min}} \) and \( {\epsilon }_{TCP} = {10\mu }\mathrm{m} \) .

<!-- Media -->

Finally, the training procedure for block-by-block cycle time prediction is illustrated in Fig. 11. Toolpath features originating from four key factors, i.e. the TCP motion, ORI motion, singularity effects, and workpiece location effects, are employed as inputs for the Bi-LSTM network. For the Bi-LSTM tuned for block-by-block prediction, the recorded block cycle time for each CL-line block is used in training the network.

The Bi-LSTM network undergoes weight and bias updates during each iteration to minimize the loss function, which quantifies the disparity between predicted and actual times. Adaptive Moment Estimation (ADAM) is selected as the optimization algorithm due to its adaptive learning rate capabilities [43], which enhance training efficiency. The loss function is defined as the Mean Square Error (MSE), and weight calculations are facilitated through backpropagation. This process equips the Bi-LSTM network with the ability to predict total cycle times upon completion of training, enabling testing on various toolpaths.

Previously described training toolpath geometries can also be used for training Bi-LSTM networks for predicting the final cycle time as well. However, care must be taken. Sequence-to-one Bi-LSTM architecture is used for predicting the final cycle time of part programs. Therefore, the sequence length (number of CL-lines) used in the training toolpaths should be consistent. If the sequence lengths are varying, strategies such as zero-padding or cropping may be employed [44]. It should be also noted that padding adds potentially meaningless data to smaller sized toolpaths, which might adversely affect the model performance. Conversely, cropping a training toolpath into a specific duration truncates valuable temporal information, such as the nuanced effects of singularity, that might be crucial for accurate predictions. To ensure effectiveness training toolpath lengths should typically lie within \( 5\% \) of the test toolpath.

## 3. Simulation and experimental results

The proposed cycle time prediction approach is first validated in simulations. The 5-axis trajectory generation algorithm by Tajima and Sencer [23] is used as a simulation kernel before testing the algorithm on actual 5-axis machine tools.

### 3.1. Simulation study

The trajectory generation algorithm assumes that the 5-axis machine tool's NC system uses a filtering-based interpolation. The filter time constants are set to \( {T}_{1} = {70}\mathrm{\;{ms}} \) and \( {T}_{2} = {10}\mathrm{\;{ms}} \) . The kinematic structure of the machine is based on a 3-axis Cartesian equipped with a trunnion A/C table. Block-by-block cycle time prediction Bi-LSTM network is configured with 20 hidden neurons and follows the training methodology outlined previously. Four training toolpaths, each consisting of 10,000 CL-lines are executed. Two of these are regular toolpaths, without consideration of singular effects, while the other two are specifically designed to account for singular effects. All of them are executed at two different feedrates: \( F = {1000}\mathrm{\;{mm}}/\mathrm{{min}} \) and \( F = {3000}\mathrm{\;{mm}}/\mathrm{{min}} \) . Additionally, each feedrate condition is executed under two interpolation tolerances: \( {\epsilon }_{TCP} = {10\mu }\mathrm{m} \) and \( {\epsilon }_{TCP} = {100\mu }\mathrm{m} \) ,while maintaining \( {\epsilon }_{ORI} = 1 \) degree for all cases. In total, each training toolpath is executed under four distinct conditions. Once training is completed and validated with 90% accuracy, tests are conducted on separate toolpaths within the same CL-line length range as the training toolpaths.

The first test toolpath is a simple 5-axis machining toolpath with a workpiece offset of \( \left\lbrack  {0,0,0}\right\rbrack  \mathrm{{mm}} \) . The toolpath features the TCP position following an oval trajectory while the tool orientation moves away from a singular point (see Fig. 12). The measured and predicted block-by-block cycle times are compared for \( F = {1000}\mathrm{\;{mm}}/\mathrm{{min}} \) ,and \( {\epsilon }_{TCP} = {10\mu }\mathrm{m} \) . The proposed Bi-LSTM method predicts block durations and the total cycle time with remarkable accuracy,showing only a 2.4% error. In contrast, the CAM system demonstrates a much larger error, at 31 %. Table 2 summarizes the results across different feedrates and blending tolerances, highlighting the consistent accuracy of the proposed approach and the poor performance of the CAM system, particularly under low interpolation tolerance and high feedrate conditions.

<!-- Media -->

Table 2

Prediction results for simulation test toolpaths I.

<table><tr><td>F [mm/ min]</td><td>\( {\epsilon }_{\text{TCP }} \) [micron]</td><td>Measured Cycle Time [sec]</td><td>Prediction (Proposed) [sec]</td><td>CAM Prediction [sec]</td></tr><tr><td>1000</td><td>10</td><td>42</td><td>41 (err:2.4 %)</td><td>29 (err:31 %)</td></tr><tr><td/><td>100</td><td>41</td><td>40 (err:2.4 %)</td><td>29 (err:29 %)</td></tr><tr><td rowspan="2">3000</td><td>10</td><td>17.34</td><td>16.8 (err:3.1 %)</td><td>12 (err:31 %)</td></tr><tr><td>100</td><td>15.89</td><td>15 (err:5.6 %)</td><td>12 (err:25 %)</td></tr></table>

<!-- Meanless: 36-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

a) TCP Motion in Cartesian Coord.

<!-- figureText: 20 TCP Motion -10 10 y-axis [mm] *ORI Motion 1 0.5 0 j-axis 300 400 500 CL-line number z-axis [mm] 0 -20 20 0 -20 x-axis [mm] b) ORI Motion in Spherical Coord. k-axis 0.5 i-axis -0.5 -0.5 c) Cycle Time Prediction 0.40 Block cycle time (sec) 0.35 0.30 0.25 0.20 Measured Predicted 0.15 CAM 0.10 0.05 0 100 200 -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_9.jpg?x=103&y=199&w=738&h=1723&r=0"/>

Fig. 13. Simulation test toolpath-II and cycle time prediction results for \( F = {3000}\mathrm{\;{mm}}/\mathrm{{min}} \) and \( {\epsilon }_{TCP} = {10\mu }\mathrm{m} \) .

Table 3

Prediction results for simulation test toolpath II.

<table><tr><td>F [mm/ min]</td><td>\( {\epsilon }_{\text{TCP }} \) [micron]</td><td>Measured Cycle Time [sec]</td><td>Prediction (Proposed) [sec]</td><td>CAM Prediction [sec]</td></tr><tr><td>1000</td><td>10</td><td>155</td><td>150 (err: 3.2 %)</td><td>106 (err:31 %)</td></tr><tr><td/><td>100</td><td>154</td><td>150 (err:2.6 %)</td><td>106 (err:31 %)</td></tr><tr><td>3000</td><td>10</td><td>56</td><td>55 (err:1.8 %)</td><td>33 (err:41 %)</td></tr><tr><td/><td>100</td><td>54</td><td>52 (err:3.7 %)</td><td>33 (err:39 %)</td></tr></table>

Table 4

Cycle time prediction performance for various workpiece locations.

<table><tr><td>Workpiece Translational Distance \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack  \left\lbrack  \mathrm{{mm}}\right\rbrack \)</td><td>Measured Cycle Time [sec]</td><td>Prediction (Proposed) [sec]</td><td>CAM Prediction [sec]</td></tr><tr><td>\( \left\lbrack  {0,0,0}\right\rbrack \)</td><td>42</td><td>41 (err:2.4 %)</td><td>29 (err:31 %)</td></tr><tr><td>\( \left\lbrack  {{500},{500},{500}}\right\rbrack \)</td><td>42</td><td>41 (err:2.4 %)</td><td>29 (err:31 %)</td></tr><tr><td>\( \left\lbrack  {{800},{800},{800}}\right\rbrack \)</td><td>43.2</td><td>42 (err: 2.8 %)</td><td>29 (err:33 %)</td></tr><tr><td>\( \left\lbrack  {{1000},{1000},{1000}}\right\rbrack \)</td><td>45</td><td>43.2 (err: 4 %)</td><td>29 (err:36 %)</td></tr></table>

<!-- Media -->

Next, a more complex trochoidal simultaneous 5-axis pocketing toolpath shown in Fig. 13 is tested. Notice from Fig. 13b that tool orientation approaches the singular point very closely. The resultant block-by-block cycle time prediction is shown in Fig. 13c. The proposed approach still provides highly accurate block-by-block cycle time predictions. Table 3 summarizes the result for different conditions on this toolpath. The proposed approach demonstrates cycle time prediction accuracy >96 %. This performance is consistent for various conditions. In stark contrast, the CAM system exhibits a prediction error of more than 40 %.

Next, the effect of workpiece (WP) location on cycle time is investigated in simulations. Varying WP locations are integrated into the training data,and translational distance \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack  \mathrm{{mm}} \) is used as an input. The WP distance is systematically adjusted from [500,500,500] \( \mathrm{{mm}} \) to \( \left\lbrack  {{1000},{1000},{1000}}\right\rbrack  \mathrm{{mm}} \) ,and toolpaths are executed at \( F = {1000}\mathrm{\;{mm}}/\mathrm{{min}},{\epsilon }_{TCP} = {10\mu }\mathrm{m} \) .

Table 4 summarizes the cycle time prediction accuracy for varying WP locations. When the workpiece is initially shifted from \( \left\lbrack  {0,0,0}\right\rbrack  \mathrm{{mm}} \) to \( \left\lbrack  {{500},{500},{500}}\right\rbrack  \mathrm{{mm}} \) ,there is no discernible change in cycle times. However, as the workpiece position shifts further to [800,800,800] mm or [1000,1000,1000] mm, longer cycle times become evident, indicating that larger workpiece shifts result in extended cycle times. The Bi-LSTM network can recognize the impact of different workpiece locations on cycle time after the training phase, whereas CAM systems are not capable of such.

Finally, the performance of the direct cycle time prediction approach is tested. The simulation test toolpath I shown in Fig. 12 is used. It contains 70 CL-lines. Training data used for block-by-block cycle time prediction are chopped and segmented in the range of \( {60} - {80}\mathrm{{CL}} \) lines. The Bi-LSTM network is configured with 50 hidden neurons and 100 training toolpaths are used.

Table 5 provides a summarized overview of the prediction results.

<!-- Meanless: 37-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

<!-- Media -->

Table 5

Cycle time prediction performance for simulation test toolpath-I.

<table><tr><td>\( F\left\lbrack  {\mathrm{{mm}}/\mathrm{{min}}}\right\rbrack \)</td><td>[micron]</td><td>Measured Cycle Time [sec]</td><td>Prediction (Proposed) [sec]</td><td>CAM Prediction [sec]</td></tr><tr><td>1000</td><td>10</td><td>42</td><td>38.8 (err:7.6 %)</td><td>29 (err:31 %)</td></tr><tr><td/><td>100</td><td>41</td><td>37.3 (err:9.0 %)</td><td>29 (err:29 %)</td></tr><tr><td>3000</td><td>10</td><td>17.34</td><td>15.7 (err:9.5 %)</td><td>12 (err:31 %)</td></tr><tr><td/><td>100</td><td>15.89</td><td>14.5 (err:8.7 %)</td><td>12 (err:25 %)</td></tr></table>

Table 6

Prediction accuracy.

<table><tr><td>Number of Training Toolpaths</td><td>Prediction Error (%)</td></tr><tr><td>100</td><td>9.5</td></tr><tr><td>300</td><td>7.5</td></tr><tr><td>500</td><td>4.8</td></tr><tr><td>700</td><td>2.8</td></tr><tr><td>1000</td><td>2.8</td></tr></table>

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_10.jpg?x=227&y=890&w=510&h=460&r=0"/>

Fig. 14. DMG-Mori Seiki eVo 40 5-axis CNC machine.

<!-- Media -->

The proposed direct cycle time prediction approach can predict cycle times at high accuracy consistently. Note that only the final cycle time information is needed for training, which can be easily obtained in practical application. In contrast, the CAM system shows prediction error exceeding \( {30}\% \) even for this simple toolpath.

To better understand the limits of this direct final cycle time method, the number of training toolpaths was expanded, and the influence of additional training toolpaths on the prediction accuracy for test tool-paths was examined. The condition characterized by the highest CAM prediction error,with a commanded feedrate of \( F = {3000} \) and a TCP error tolerance of \( {\epsilon }_{TCP} = {10\mu }\mathrm{m} \) was chosen. The results are presented in Table 6. The prediction performance improves with increased training data. For instance, with 300 training toolpaths, prediction error can be reduced to 7.5 %.

Increasing the number of training toolpaths significantly enhances accuracy but this improvement plateaus at around 700 sets of training data. Beyond this point, the reduction in prediction error becomes less pronounced. This plateauing effect suggests that the model has effectively captured the relevant patterns and relationships within the data for cycle time prediction. In other words, additional data, beyond a certain threshold, doesn't lead to substantial improvements because the model has already learned most of what it needs to make accurate predictions.

<!-- Media -->

<!-- figureText: a) TCP Motion in Cartesian Coord. <TCP Motion 0 -20 -40 -60 x-axis [mm] ORI Motion Measured Predicted CAM 500 600 700 800 900 CL-line number 280 z-axis [mm] 260 240 220 y-axis [mm] -150 -145 -140 60 40 20 b) ORI Motion in Spherical Coord. 0.5 k-axis 0 -0.5 j-axis 0.5 -0.5 -0.5 0 0.5 i-axis c) Cycle Time Prediction 1.5 Block cycle time(sec) 1.2 0.9 0.6 0.3 100 200 300 400 -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_10.jpg?x=902&y=532&w=734&h=1586&r=0"/>

Fig. 15. Experimental test toolpath-I and cycle time prediction results for \( F = {1000}\mathrm{\;{mm}}/\mathrm{{min}} \) and \( {\epsilon }_{TCP} = {10\mu }\mathrm{m} \) .

<!-- Meanless: 38-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

Table 7

Experimental cycle time prediction for test toolpaths-I.

<table><tr><td>\( F \) [mm/min]</td><td>\( {\epsilon }_{TCP} \) [micron]</td><td>Measured Cycle Time [sec]</td><td>Direct Final Cycle Time Prediction [sec]</td><td>Block-by-Block Prediction [sec]</td><td>CAM Prediction [sec]</td></tr><tr><td>1000</td><td>10</td><td>247.84</td><td>228 (err: 7.9 %)</td><td>243.9 (err: 1.6 %)</td><td>212.5 (err: 15%)</td></tr><tr><td/><td>100</td><td>247.15</td><td>228 (err: 7.7 %)</td><td>244 (err: 1.3 %)</td><td>212.5 (err: 15%)</td></tr><tr><td>3000</td><td>10</td><td>90.69</td><td>78 (err:14 %)</td><td>87.6 (err: 3.4 %)</td><td>71 (err: 22%)</td></tr><tr><td/><td>100</td><td>82.12</td><td>76 (err: 7.5 %)</td><td>84.3 (err: 2.6 %)</td><td>71 (err: 14%)</td></tr></table>

<!-- Media -->

### 3.2. Experimental results

A DMG-Mori Seiki eVo 40 5-axis CNC machine tool with the Hei-denhain TNC640 controller was used in experiments (see Fig. 14). The kinematic model of the machine is detailed in [45]. Both block-by-block, and direct final time prediction method are tested experimentally. Training toolpaths commanded at \( F = {1000} \) and \( {3000}\mathrm{\;{mm}}/\mathrm{{min}} \) and with interpolation tolerance parameters \( {\epsilon }_{TCP} = {10} \) and \( {100\mu }\mathrm{m} \) ,and \( {\epsilon }_{ORI} = 1 \) degree are used in training.

For the block-by-block Bi-LSTM network, the training toolpaths include four training toolpaths, each consisting of 10,000 CL-lines. Two of these toolpaths are regular, without consideration of singular effects, while the other two are specifically designed to account for singular effects. The Bi-LSTM network for block-by-block cycle time prediction comprises 20 hidden neurons and follows the training methodology detailed in the previous section. Whereas the Bi-LSTM network for final cycle time prediction comprises 50 hidden neurons and the training protocol follows the methodology presented in Section 2.5. One hundred training toolpaths that cover a range of \( \pm  5\% \) of the CL-line size for the respective test toolpaths are extracted from the four training toolpaths for block-by-block cycle time prediction.

Two different toolpaths are used in tests. Fig. 15 displays the experimental test toolpath I, consisting of 954 CL-lines, and summarizes the block-by-block cycle time prediction results for \( F = {1000}\mathrm{\;{mm}}/\mathrm{{min}} \) and \( {\epsilon }_{TCP} = {10\mu }\mathrm{m} \) . Please refer to Table 7 for a comprehensive summary of all the results. In general, the proposed block-by-block cycle time prediction method demonstrates an accuracy of \( {96}\% \) . The direct final cycle time prediction approach, which only requires the final cycle time during training,achieves a prediction accuracy exceeding \( {86}\% \) . It is worth noting that the predictions based on the proposed approach remain consistent across various conditions, while CAM predictions exhibit significant inaccuracies,with discrepancies exceeding 22%.

The second test toolpath is the fan profile taken from literature [23] and it contains 250 CL-lines (Fig. 16). It is deliberately designed to approach a singular point in the orientation motion. The prediction results for block-by-block cycle time compared with the CAM system is shown in Fig. 16c.

Table 8 presents a comprehensive overview of the prediction results obtained from the test toolpaths II for both direct final cycle time and block-by-block prediction, which are compared with the CAM system. These predictions closely align with the outcomes of simulations. The proposed approach underscores the impressive predictive capability of the block-by-block Bi-LSTM, achieving an accuracy of over 95% for cycle time prediction and exceeding \( {87}\% \) for direct final cycle time prediction. This consistent performance holds true across various fee-drates and tolerance values. In stark contrast, the CAM system exhibits a prediction error exceeding 24% for this realistic and complex toolpath.

Next, the effect of workpiece (WP) location on cycle time is investigated. Varying WP locations are integrated into the training data with translational distance \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack  \mathrm{{mm}} \) used as an input. The WP distance is systematically adjusted from \( \left\lbrack  {0,0,0}\right\rbrack  \mathrm{{mm}} \) to \( \left\lbrack  {{800},{800},{800}}\right\rbrack  \mathrm{{mm}} \) and \( \left\lbrack  {{1000},{1000},{1000}}\right\rbrack  \mathrm{{mm}} \) ,and toolpaths are executed at \( F = {3000}\mathrm{\;{mm}}/ \) min and \( {\epsilon }_{TCP} = {10\mu }\mathrm{m} \) .

Table 9 summarizes the cycle time prediction accuracy for varying WP locations. When the workpiece is initially shifted from \( \left\lbrack  {0,0,0}\right\rbrack  \mathrm{{mm}} \) to \( \left\lbrack  {{800},{800},{800}}\right\rbrack  \mathrm{{mm}} \) ,the measured cycle time remain constant. However, as the workpiece position shifts further to [1000,1000,1000] mm, longer cycle time is measured. The Bi-LSTM network can recognize the impact of different workpiece locations on cycle time after training, whereas the CAM system is not capable of such.

<!-- Media -->

<!-- figureText: a) TCP Motion in Cartesian Coord. TCP Motion 60 40 -20 x-axis [mm] *ORI Motion 0.5 j-axis Measured Predicted CAM 150 200 250 CL-line number 100 80 z-axis [mm] 60 40 20 0 -20 100 80 60 y-axis [mm] 40 20 b) ORI Motion in Spherical Coord. k-axis 0.5 0 i-axis -0.5 -0.5 -1^-1 c) Cycle Time Prediction 0.4 Block cycle time(sec) 0.3 0.2 0.1 50 100 -->

<img src="https://cdn.noedgeai.com/bo_d2sl8nn7aajc738sf7q0_11.jpg?x=905&y=540&w=713&h=1578&r=0"/>

Fig. 16. Experimental test toolpath-II and cycle time prediction results for \( F = {1000}\mathrm{\;{mm}}/\mathrm{{min}} \) and \( {\epsilon }_{TCP} = {10\mu }\mathrm{m} \) .

<!-- Meanless: 39-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

Table 8

Experimental cycle time prediction for test toolpaths-II.

<table><tr><td>\( F\left\lbrack  {\mathrm{{mm}}/\mathrm{{min}}}\right\rbrack \)</td><td>\( {\epsilon }_{TCP} \) [micron]</td><td>Measured Cycle Time [sec]</td><td>Direct Final Cycle time Prediction [sec]</td><td>Block-by-Block Prediction [sec]</td><td>CAM Prediction [sec]</td></tr><tr><td rowspan="4">1000</td><td>10</td><td>55.27</td><td>51</td><td>54.25</td><td>45</td></tr><tr><td/><td/><td>(err: 7.7 %)</td><td>(err: 1.9 %)</td><td>(err: 18 %)</td></tr><tr><td>100</td><td>51.27</td><td>47</td><td>53.53</td><td>45</td></tr><tr><td/><td/><td>(err: 8.3 %)</td><td>(err: 4.4 %)</td><td>(err: 13 %)</td></tr><tr><td>3000</td><td>10</td><td>18.32</td><td>16</td><td>17.68</td><td>14</td></tr><tr><td/><td/><td/><td>(err:12.6 %)</td><td>(err:3.5 %)</td><td>(err: 24 %)</td></tr><tr><td/><td>100</td><td>17.1</td><td>15</td><td>16.35</td><td>14</td></tr><tr><td/><td/><td/><td>(err: 12.3 %)</td><td>(err: 4.4 %)</td><td>(err: 19 %)</td></tr></table>

Table 9

Cycle time prediction performance for test toolpath-I across different WP locations.

<table><tr><td>Workpiece Translational Distance \( \left\lbrack  {{X}_{o},{Y}_{o},{Z}_{o}}\right\rbrack  \left\lbrack  \mathrm{{mm}}\right\rbrack \)</td><td>Measured Cycle Time [sec]</td><td>Prediction (Proposed) [sec]</td><td>CAM Prediction [sec]</td></tr><tr><td>\( \left\lbrack  {0,0,0}\right\rbrack \)</td><td>90.69</td><td>87.6 (err:3.4 %)</td><td>71 (err:21 %)</td></tr><tr><td>\( \left\lbrack  {{800},{800},{800}}\right\rbrack \)</td><td>90.69</td><td>87.6 (err: 3.4 %)</td><td>71 (err:21 %)</td></tr><tr><td>\( \left\lbrack  {{1000},{1000},{1000}}\right\rbrack \)</td><td>93.16</td><td>89 (err: 4.5 %)</td><td>71 (err:24 %)</td></tr></table>

<!-- Media -->

## 4. Conclusion and future work

This paper introduces a data-driven approach for predicting machining cycle times of complex 5-axis machining part programs. Two machine learning-based cycle time prediction method are proposed utilizing the Bi-LSTM networks; namely, a block-by-block and a final cycle time prediction model.

- The block-by-block cycle time prediction model accurately captures the CNC system’s interpolation behavior and achieves \( > {93}\% \) accuracy in prediction cycle times of complex 5-axis machining toolpaths using the toolpath geometry information. To achieve such high accuracy, it requires block-by-block cycle time information during the training phas. Therefore, NC axis motion monitoring function is required to acquire CL block cycle times during interpolation. This information can be easily obtained for newer NC systems that have such functionality where end-users can record the average feedrate for each CL line of a part program. However, for older NC systems, which do not have such functionality, training data collection may need external measurement strategies that may be cumbersome and impractical.

- To deal with that, a second approach, the final cycle time prediction strategy, is proposed. This approach is much more practical where training procedure only requires the end-user to record the final cycle time for a part program, which can be easily obtained. Thus, it does not require any specialized NC function or external sensors. The approach only predicts the final cycle time for 5-axis part programs with >87% accuracy and outperforms CAM systems. This approach can be deployed on shop floors conveniently, and its performance can be even further improved with more training data.

- Both developed methods do not require detailed kinematic model of 5-axis machine tools. They only make use of key kinematic features of the machine such as the rotary axis configuration, i.e. AB, BC or AC, and the location of the workpiece on the machine table.

- Both approaches capture key NC interpolation parameters such as the effect of interpolation tolerance, acceleration, jerk limits and feedrate information, and they utilize the toolpath geometry information to predict cycle times, which involves the TCP position and orientation information. The strategy works most accurately with the XYXIJK 5-axis part program format.

- The developed algorithm can be integrated into commercial CAM systems as a cycle-time prediction module.

- The future work involves extension of the algorithm considering effect of tool length, tool change behavior, and application of the algorithm to predict cycle times during robotic machining.

## CRediT authorship contribution statement

Shih-Hsuan Chien: Conceptualization, Methodology, Software, Validation, Formal analysis, Investigation, Data curation, Writing - Original Draft, Writing - Review & Editing, Burak Sencer: Conceptualization, Methodology, Investigation, Writing - Review & Editing, Project administration, Funding acquisition, Supervision, Visualization. Rob Ward: Writing - Review & Editing, Resources, Data curation.

## Declaration of Competing Interest

The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper.

## Acknowledgment

The authors gratefully thank Oregon Manufacturing Innovation Centre (OMIC) for funding this project. References

[1] Budak E, Ozturk E, Tunc LT. Modeling and simulation of 5-axis milling processes. CIRP Ann Manuf Technol 2009;58(1):347-50.

[2] Cripps RJ, Cross B, Hunt M, Mullineux G. Singularities in five-axis machining: cause, effect and avoidance. Int J Mach Tools Manuf 2017;116:40-51.

[3] Tajima S, Sencer B, Shamoto E. Accurate interpolation of machining tool-paths based on FIR filtering. Precis Eng 2018;52(January):332-44.

[4] Beudaert X, Lavernhe S, Tournier C. 5-Axis local corner rounding of linear tool path discontinuities. Int J Mach Tools Manuf 2013;73:9-16.

[5] Siller H, Rodriguez CA, Ahuett H. Cycle time prediction in high-speed milling operations for sculptured surface finishing. J Mater Process Technol 2006;174 (1-3):355-62.

[6] Coelho RT, De Souza AF, Roger AR, Rigatti AMY, De Lima Ribeiro AA. Mechanistic approach to predict real machining time for milling free-form geometries applying high feed rate. Int J Adv Manuf Technol 2010;46(9-12):1103-11.

<!-- Meanless: 40-->




<!-- Meanless: S.-H. Chien et al. CIRP Journal of Manufacturing Science and Technology 48 (2024) 28-41-->

[7] Tajima S, Sencer B. Accurate real-time interpolation of 5-axis tool-paths with local corner smoothing. Int J Mach Tools Manuf 2019;142:1-15.

[8] Tajima S, Sencer B. Global tool-path smoothing for CNC machine tools with uninterrupted acceleration. Int J Mach Tools Manuf 2017;121:81-95.

[9] Erkorkmaz K, Heng M. A heuristic feedrate optimization strategy for NURBS toolpaths. CIRP Ann Manuf Technol 2008;57(1):407-10.

[10] Tajima S. & Sencer B. "Smooth Cornering Strategy for High Speed CNC Machine Tools With Confined Contour Error." Proceedings of the ASME 2016 11th International Manufacturing Science and Engineering Conference. Volume 2: Materials; Biomanufacturing; Properties, Applications and Systems; Sustainable Manufacturing. Blacksburg, Virginia, USA. June 27-July 1, 2016. -V002T04A034.

[11] So BS, Jung YH, Park JW, Lee DW. Five-axis machining time estimation algorithm based on machine characteristics. J Mater Process Technol 2007;187-188:37-40.

[12] Altintas Y, Tulsyan S. Prediction of part machining cycle times via virtual CNC. CIRP Ann Manuf Technol 2015;64(1):361-4.

[13] Ward R, Sencer B, Jones B, Ozturk E. Accurate prediction of machining feedrate and cycle times considering interpolator dynamics. International Journal of Advanced Manufacturing Technology 2021;116(1-2):417-38.

[14] Sun C, Dominguez-Caballero J, Ward R, Ayvar-Soberanis S, Curtis D. Machining cycle time prediction: data-driven modelling of machine tool feedrate behavior with neural networks. Robot Comput Integr Manuf 2022;75(June 2021):102293.

[15] Endo M, Sencer B. Accurate prediction of machining cycle times by data-driven modelling of NC system's interpolation dynamics. CIRP Ann 2022;71(1):405-8.

[16] Chien SH, Sencer B, Ward R. Accurate prediction of machining cycle times and feedrates with deep neural networks using BiLSTM. J Manuf Syst 2023;68:680-6.

[17] Erkorkmaz K, Layegh SE, Lazoglu I, Erdim H. Feedrate optimization for freeform milling considering constraints from the feed drive system and process mechanics. CIRP Ann 2013;62(1):395-8.

[18] Tajima S, Sencer B. Online interpolation of 5-axis machining toolpaths with global blending. Int J Mach Tools Manuf 2022;175:103862.

[19] Hayasaka T, Minoura K, Ishizaki K, Shamoto E, Burak S. A lightweight interpolation algorithm for short-segmented machining tool paths to realize vibration avoidance, high accuracy, and short machining time. Precis Eng 2019;59: \( 1 - {17} \) .

[20] Yang J, Aslan D, Altintas Y. A feedrate scheduling algorithm to constrain tool tip position and tool orientation errors of five-axis CNC machining under cutting load disturbances. CIRP J Manuf Sci Technol 2018;23:78-90.

[21] Tajima S, Sencer B. Accurate real-time interpolation of 5-axis tool-paths with local corner smoothing. Int J Mach Tools Manuf 2019;142:1-15.

[22] Sencer B, Ishizaki K, Shamoto E. High speed cornering strategy with confined contour error and vibration suppression for CNC machine tools. CIRP Ann 2015;64 (1):369-72.

[23] Tajima S, Sencer B. Real-time trajectory generation for 5-axis machine tools with singularity avoidance. CIRP Ann 2020;69(1):349-52.

[24] Tajima S, Sencer B. Global tool-path smoothing for CNC machine tools with uninterrupted acceleration. Int J Mach Tools Manuf 2017;121:81-95.

[25] Tajima S, Sencer B, Yoshioka H, Shinno H Smooth Path Blending for 5-Axis Machine Tools. Proceedings of the JSME 2020 Conference on Leading Edge Manufacturing/Materials and Processing. JSME 2020 Conference on Leading Edge Manufacturing/Materials and Processing. Virtual, Online September;3:2020.- V001T02A007.

[26] Beudaert X, Lavernhe S, Tournier C. Feedrate interpolation with axis jerk constraints on 5-axis NURBS and G1 tool path. Int J Mach Tools Manuf 2012;57: 73-82.

[27] Tutunea-Fatan OR, Feng HY. Configuration analysis of five-axis machine tools using a generic kinematic model. Int J Mach Tools Manuf 2004;44(11):1235-43.

[28] Ward RA, Sencer B, Jones B, Ozturk E. Five-axis trajectory generation considering synchronization and nonlinear interpolation errors. J Manuf Sci Eng Trans ASME 2022;144(8).

[29] My CA, Bohez ELJ. A novel differential kinematics model to compare the kinematic performances of 5-axis CNC machines. Int J Mech Sci 2019;163:105117.

[30] Cripps RJ, Cross B, Hunt M, Mullineux G. Singularities in five-axis machining: cause, effect and avoidance. Int J Mach Tools Manuf 2017;116:40-51.

[31] Affouard A, Duc E, Lartigue C, Langeron JM, Bourdet P. Avoiding 5-axis singularities using tool path deformation. Int J Mach Tools Manuf 2004;44(4): 415-25.

[32] Cripps RJ, Cross B, Hunt M, Mullineux G. Singularities in five-axis machining: cause, effect and avoidance. Int J Mach Tools Manuf 2017;116:40-51.

[33] Pessoles X, Landon Y, Segonds S, Rubio W. Optimisation of workpiece setup for continuous five-axis milling: application to a five-axis BC type machining centre. Int J Adv Manuf Technol 2013;65(1-4):67-79.

[34] Shaw D, Ou GY. Reducing X, Y and Z axes movement of a 5-axis ac type milling machine by changing the location of the work-piece. CAD Comput Aided Des 2008; 40(10-11):1033-9.

[35] Xu K, Tang K. Optimal workpiece setup for time-efficient and energy- saving five-axis machining of freeform surfaces. J Manuf Sci Eng Trans ASME 2017;139(5).

[36] Lee R-S, She C-H. Developing a postprocessor for three types of five-axis machine tools. Int J Adv Manuf Technol 1997;13(9):658-65.

[37] Kaur M. and Mohta A. "A review of deep learning with recurrent neural network," International Conference on Smart Systems and Inventive Technology (ICSSIT), Tirunelveli, India, 2019, pp. 460-465.

[38] Pirani M., Thakkar P., Jivrani P., Bohara M. H. and Garg, D. "A Comparative Analysis of ARIMA, GRU, LSTM and BiLSTM on Financial Time Series Forecasting," 2022 IEEE International Conference on Distributed Computing and Electrical Circuits and Electronics (ICDCECE), Ballari, India, 2022, pp. 1-6.

[39] Siami-Namini S., Tavakoli N., Namin A. S. The performance of LSTM and BiLSTM in forecasting time series," in. Proc. IEEE Int. Conf. Big Data (Big Data) Dec. 2019; 3285-3292.

[40] Shen SL, Atangana Njock PG, Zhou A, Lyu HM. Dynamic prediction of jet grouted column diameter in soft soil using Bi-LSTM deep learning. Acta Geotech 2021;16 (1):303-15.

[41] Lin WC, Busso C. Chunk-level speech emotion recognition: a general framework of sequence-to-one dynamic temporal modeling. IEEE Trans Affect Comput 2023;14 (2):1215-27.

[42] Ye R, Feng S, Li X, Ye Y, Zhang B, Luo C. SPLNet: a sequence-to-one learning network with time-variant structure for regional wind speed prediction. Inf Sci (N York) 2022;609:79-99.

[43] Kingma DP, Ba J. Adam: a method for stochastic optimization. arXiv Preprint arXiv 2014;1412:6980.

[44] Shi X, Chen Z, Wang H, Yeung D-Y, Wong W-K, Woo W. Convolutional LSTM network: a machine learning approach for precipitation nowcasting. Adv Neural Inf Process Syst 2015:28.

[45] Ward, R., 2022, Smooth Trajectory Generation for 5-Axis CNC Machine Tools (Doctoral Dissertation, University of Sheffield)., Sheffield.

<!-- Meanless: 41-->

