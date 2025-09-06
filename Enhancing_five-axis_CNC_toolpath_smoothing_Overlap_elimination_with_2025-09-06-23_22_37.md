

<!-- Meanless: CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57 Contents lists available at ScienceDirect CIRP Journal of Manufacturing Science and Technology ELSEVIER journal homepage: www.elsevier.com/locate/cirpj updates-->

# Enhancing five-axis CNC toolpath smoothing: Overlap elimination with asymmetrical B-splines

Yifei \( {\mathrm{{Hu}}}^{\mathrm{a},\mathrm{b}} \) ,Xin Jiang \( {}^{\mathrm{a},\mathrm{c}, * } \) ,Guanying \( {\mathrm{{Huo}}}^{\mathrm{a},\mathrm{c},\mathrm{d}} \) ,Cheng Su \( {}^{\mathrm{a},\mathrm{c},\mathrm{d}} \) ,Hexiong Li \( {}^{\mathrm{a},\mathrm{b}} \) ,Li-Yong Shen \( {}^{\mathrm{e}} \) , Zhiming Zheng \( {}^{a,c,d} \)

\( {}^{a} \) LMIB & NLSDE & Institute of Artificial Intelligence,Beihang University,Beijing,China

\( {}^{\mathrm{b}} \) School of Mathematics Science,Beihang University,Beijing,China

\( {}^{\mathrm{c}} \) Zhongguancun Laboratory,Beijing,China

\( {}^{d} \) Zhengzhou Aerotropolis Institute of Artificial Intelligence,Zhengzhou,Henan,China

\( {}^{\mathrm{e}} \) School of Mathematical Sciences,University of Chinese Academy of Sciences,Beijing,China

## ARTICLEINFO

Keywords:

Five-axis CNC machining

Corner smoothing

Tool tip smoothing error

Orientation smoothing error

Overlap elimination

## A B S T R A C T

In the realm of five-axis Computer Numerical Control (CNC) machining, the challenge of minimizing velocity, acceleration, and jerk fluctuations has prompted the development of various local corner smoothing methods. However, existing techniques often rely on symmetrical spline curves or impose pre-set transition length constraints, limiting their effectiveness in reducing curvature and maintaining velocity at critical corners. To comprehensively address these limitations comprehensively, a novel approach for local smoothing of five-axis linear toolpaths is presented in this paper. The proposed method introduces two asymmetrical B-splines at corners, effectively smoothing both the tool-tip position in the workpiece coordinate system and the tool orientation in the machine coordinate system. To fine-tune transition curve scales and minimize velocity disparities between adjacent corners, an overlap elimination scheme is employed. Furthermore, the two-step strategy emphasizes the synchronization of tool-tip position and tool orientation while considering maximal approximation error. The outcome is a blended five-axis toolpath that significantly reduces curvature extremes in the smoothed path,achieving reductions ranging from 36.28% to 45.51%. Additionally,the proposed method streamlines the interpolation process,resulting in time savings ranging from 5.68% to 8.78%,all the while adhering to the same geometric and kinematic constraints.

## 1. Introduction

In the realm of machining 3D products with intricate freeform surfaces, the utilization of five-axis computer numerical controlled (CNC) machine tools has gained widespread traction. These five-axis machines offer distinct advantages, notably in obstacle avoidance and high machining efficiency, when compared to three-axis machines [1]. The design process typically begins in computer-aided design (CAD) systems, followed by the generation of tool paths using computer-aided manufacturing (CAM) software. In the following multi-axis CNC machining, linear interpolation (G01) is a fundamental technique, guiding the movement of machine axes through numerous linear segments along the tool paths [2].

However, challenges arise from tangential discontinuities at the intersections of linear tool paths, which inevitably result in velocity fluctuations and induce structural vibrations in machine tools, especially when encountering sharp geometric variations [3]. As a consequence, feedrates often need to be reduced to adhere to drive constraints, adversely affecting both machining efficiency and quality [4]. Therefore, the smoothing of linear tool paths becomes essential to mitigate these issues and improve the overall machining process.

Indeed, maintaining a smooth feedrate on tool paths is crucial for reducing cycle time, a primary objective for manufacturers [5]. Tool paths with smaller curvature and curvature derivatives enable the implementation of high and smooth feedrate profiles. Therefore, paying meticulous attention to curvature and curvature variation is essential in the process of tool path smoothing. This careful consideration enhances machining efficiency and quality.

The existing tool path smoothing methods can be broadly categorized into two main approaches: global tool path smoothing [6-12] and local corner smoothing \( \left\lbrack  {1,{13} - {33}}\right\rbrack \) . Global smoothing methods employ smooth curves such as polynomial splines, B-splines, and NURBS to interpolate or approximate discrete G01 points. These techniques are particularly effective for rounding a high density of short segments, ensuring high-order continuity of the tool paths [13]. However, challenges such as numerical instability, the absence of chord error constraints, and result uncertainties [8] persist, posing significant difficulties in their comprehensive resolution.

---

<!-- Footnote -->

* Corresponding author at: LMIB & NLSDE & Institute of Artificial Intelligence, Beihang University, Beijing, China.

E-mail address: jiangxin@buaa.edu.cn (X. Jiang).

<!-- Footnote -->

---

<!-- Meanless: https://doi.org/10.1016/j.cirpj.2024.05.013 Received 26 October 2023; Received in revised form 29 April 2024; Accepted 25 May 2024 Available online 4 June 2024 1755-5817/(C) 2024 CIRP.-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

<!-- Media -->

<!-- figureText: \( {0.5}{l}_{1} \) Transition curve Linear tool path segment \( \theta /2 \) \( {l}_{1} \) \( {l}_{2} \) \( {P}_{4} \) \( {P}_{5}{0.5}{l}_{2}{P}_{6} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_1.jpg?x=148&y=150&w=657&h=515&r=0"/>

Fig. 1. The quintic B-spline curve as an example.

<!-- figureText: \( {P}_{i - 1} \) \( {P}_{i + 2} \) \( {C}_{i + 1}\left( u\right) \) \( {P}_{i + 1} \) Overlap \( {2.5}{l}_{1,i + 1} \) \( {C}_{i}\left( u\right) \) \( {P}_{i} \) \( {2.5}{l}_{2,i} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_1.jpg?x=114&y=755&w=726&h=513&r=0"/>

Fig. 2. The overlap between adjacent transition curves.

<!-- Media -->

In contrast, local smoothing methods construct transition curves at corners to round adjacent linear tool path segments. Widely used techniques in this category include Bezier splines [14,15], B-splines [1,13, 16-25,33], Pythagorean-hodograph (PH) splines [26-30], and clothoid splines \( \left\lbrack  {{31},{32}}\right\rbrack \) . These techniques offer the advantage of achieving high-order continuity and precise control over the approximation error [16], garnering significant interest from researchers. However, the integration of synchronous rotary axes in five-axis machining introduces new challenges.

Firstly, there's a need to construct a rounding route with constrained rounding errors for tool orientation. Secondly, parameters between the smoothed position and orientation must be synchronized to maintain tool orientation continuity [1]. Moreover, dealing with short-segmented tool paths raises the risk of overlapping adjacent transition curves. These overlaps can hinder tool path continuity, leading to reduced machining efficiency. Thus, effective avoidance of overlaps is pivotal for both three-axis and five-axis local smoothing techniques.

In recent decades, researchers have proposed various strategies for five-axis local smoothing. Tulsyan et al. [13] suggested inserting quintic and septic B-splines for the tool tip and tool orientation at adjacent linear tool path segments. The control points of these position and orientation splines were optimized using the Newton-Raphson algorithm,ensuring \( {C}^{3} \) continuity at junctions within specified tolerance limits. Bi et al. [14] introduced two cubic Bézier corner transition curves in the machine coordinate system (MCS). In [15], two Bezier splines were used to smooth linear tool path segments in the workpiece coordinate system (WCS), constraining path errors within specified limits. Zhang et al. [17] proposed a symmetric quartic B-spline to smooth the tool tip position in the WCS and an asymmetric quartic B-spline for the tool orientation in the MCS. Yin et al. [1] utilized a circumscribed symmetrical cubic B-spline with micro-splines for the tool tip position in the WCS. An asymmetrical cubic B-spline and two similar micro-splines were introduced to smooth the tool orientation in the MCS. In [26], two \( {C}^{3} \) continuous \( \mathrm{{PH}} \) splines were used to smooth the tool tip position in the WSC and tool orientation in the MCS, ensuring continuity of the motion variance of smoothed tool orientation concerning tool tip displacement. Huang et al. [32] introduced the "bi-airthoid," a novel curve based on clothoid splines, for smoothing the tool position corners in the WCS and tool orientation corners in the MCS. However, to prevent overlaps between adjacent curves, existing methods limited transition lengths to no more than half the linear segment. While effective, this constraint was excessively restrictive, compressing longer transition curves without overlaps. Consequently, this compression increased curvature and reduced velocity along the smooth tool path. Addressing this limitation remains a challenge in current research efforts.

<!-- Media -->

<!-- figureText: \( {v}_{i + 1} \) Keep accelerating \( {B}_{i + 1}\left( {0.5}\right) \) Keep decelerating \( {B}_{i + 1}\left( {0.5}\right) \) (b) \( {v}_{i + 1,{update}} \) \( {v}_{\text{actual }} < {v}_{i + 1} \) \( {v}_{i,{update}} \) \( {v}_{i} \) \( {B}_{i}\left( {0.5}\right) \) (a) \( v \) \( {v}_{i,{update}} \) \( {v}_{\text{actual }} < {v}_{i} \) \( {v}_{i + 1,{update}} \) \( {v}_{i + 1} \) \( {B}_{i}\left( {0.5}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_1.jpg?x=906&y=151&w=725&h=1088&r=0"/>

Fig. 3. The Schematic diagram of feedrate change before and after updating \( {l}_{i} \) and \( {l}_{i + 1} \) .

<!-- Media -->

Except for the half-length constraint, some attempts have introduced

<!-- Meanless: 37-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57 Algorithm 1 to solve the optimization model.-->

Table 1 a length margin to ensure the existence of the remaining linear segment after smoothing. To avoid overlap, the transition length was limited to no more than half of the difference between the linear segment length and the length margin. For instance, Huang et al. [18] employed a pair of cubic B-splines to smooth the tool position and tool orientation paths in the WCS. In [19], two quintic B-splines were inserted at the corners of the linear segments to achieve \( {G}^{3} \) continuity of both the tool position path and tool orientation path. Finally, the entire path was constructed to be \( {C}^{3} \) continuous by converting the remaining linear segments to the specially constructed quintic B-splines, and thesynchronization of the tool orientation path with respect to the tool position path was achieved by employing the same curve parameter. Liu et al. [20] inserted a pair of quintic B-splines to smooth the tool position and orientation paths based on curvature derivative continuity conditions for five-degree of freedom hybrid robots. This methodology was later extended for general five-axis tool paths, where the tool position path was comprised of lines and arcs [21]. Additionally, Yang et al. [25,30] proposed corner smoothing algorithms for robots by constructing specially designed micro-splines, effectively restricting the jerk of the tool path and improving the tracking accuracy of the trajectory. However, due to the introduction of the length margin, the transition curves were compelled to over-approach the corners, leading to a decrease in velocity at the corners, particularly when dealing with short-segmented tool paths [16].

<!-- Media -->

---

Algorithm 1
1: \( {l}_{ - } \) generation \( \left( {{\theta }_{i},{\theta }_{i + 1},{L}_{i},K,{\varepsilon }_{p,\max }}\right) \)
2: If \( \frac{{10}{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i}/2}\right) } \leq  K{L}_{i} \) :
3: \( \;{l}_{i} = \frac{4{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i}/2}\right) },{l}_{i + 1} = \frac{{L}_{i}}{2.5} - {l}_{i} \)
4: Else if \( \frac{{10}{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i + 1}/2}\right) } \leq  K{L}_{i} \) :
	5: \( \;{l}_{i + 1} = \frac{4{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i + 1}/2}\right) },{l}_{i} = \frac{{L}_{i}}{2.5} - {l}_{i + 1} \)
6: Else:
			\( {u}_{l} = \max \left( {K,1 - \frac{{10}{\varepsilon }_{p,\max }}{3\cos \left( \frac{{\theta }_{i}}{2}\right) {L}_{i}}}\right) ,{u}_{r} = \min \left( {1 - K,\frac{{10}{\varepsilon }_{p,\max }}{3\cos \left( \frac{{\theta }_{i + 1}}{2}\right) {L}_{i}}}\right) \) and \( u = \frac{{a}_{i}^{2}}{{a}_{i}^{2} + {a}_{i + 1}^{2}} \)
				If \( u < {u}_{l} : u = {u}_{l} \) End
		If \( u > {u}_{r} : u = {u}_{r} \) End
				\( {l}_{i} = {L}_{i}\frac{\left( 1 - u\right) }{2.5} \) and \( {l}_{i + 1} = {L}_{i}\frac{u}{2.5} \)
11: \( {End} \)

---

<!-- figureText: \( {R}_{i - 1} \) \( k \) \( j \) Equation (1.2) Equation (1.1) \( {R}_{i + 1} \) \( A \) \( {P}_{0,i}^{R} \) \( {0.5}{l}_{1}^{R} \) \( {P}_{1,i}^{R} \) \( \alpha /2 \) \( {l}_{1}^{R} \) \( {P}_{2,i}^{R} \) Transition curve \( {R}_{i} \) \( {l}_{2}^{R} \) \( {l}_{2}^{R} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_2.jpg?x=226&y=1275&w=1294&h=616&r=0"/>

Fig. 4. Corner smoothing of tool orientation.

<!-- Meanless: 38-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

<!-- figureText: Given G01 Toolpath \( i = i + 1 \) \( {l}_{1,i + 1} = \min \left( {\frac{4{\varepsilon }_{p,\max }}{3\cos \left( \frac{{\theta }_{i + 1}}{2}\right) },\frac{{L}_{i}}{2.5},\frac{4{\varepsilon }_{r,\max }}{3{\operatorname{ratio}}_{i}\cos \left( \frac{{\alpha }_{i + 1}}{2}\right) }}\right) \) \( {l}_{1,i + 1}^{R} = {\text{ratio}}_{i}{l}_{1,i + 1} \) \( {l}_{2,i}^{R} = {\text{ratio}}_{i}{l}_{2,i} \) Generate \( {B}_{i}\left( u\right) \) and \( {B}_{i}^{R}\left( u\right) \) by equation (3) and (9) Stop \( {l}_{2,i}^{R} = {\text{ ratio }}_{i}{l}_{2,i} \) \( {l}_{1,i + 1}^{R} = {\text{ratio}}_{i}{l}_{1,i + 1} \) Update \( {\mathbf{l}}_{1,i + 1},{\mathbf{l}}_{2,i},{\mathbf{l}}_{1,i + 1}^{R} \) and \( {\mathbf{l}}_{2,i}^{R} \) according to equation (11-12) when the corresponding conditions are met \( \left\{  {{\mathbf{P}}_{\mathbf{0}},{\mathbf{P}}_{\mathbf{1}},\ldots ,{\mathbf{P}}_{\mathbf{n}}}\right\} \) \( \left\{  {{\mathbf{R}}_{\mathbf{0}},{\mathbf{R}}_{\mathbf{1}},\ldots ,{\mathbf{R}}_{\mathbf{n}}}\right\} \) Set \( i = 0 \) \( i = 0 \) ? \( \rightarrow  i = n - 1 \) ? \( \rightarrow  {l}_{2,i} = \min \left( {\frac{4{\varepsilon }_{p,\max }}{3\cos \left( \frac{{\theta }_{i}}{2}\right) },\frac{{L}_{i}}{2.5},\frac{4{\varepsilon }_{r,\max }}{3{\operatorname{ratio}}_{i}\cos \left( \frac{{\alpha }_{i}}{2}\right) }}\right) \) \( {l}_{2,i} = \frac{4{\varepsilon }_{p,\max }}{3\cos \left( \frac{{\theta }_{i}}{2}\right) } \) \( {l}_{1,i + 1} = \frac{4{\varepsilon }_{p,\max }}{3\cos \left( \frac{{\theta }_{i + 1}}{2}\right) } \) \( {l}_{2,i} + {l}_{1,i + 1} > \frac{{L}_{i}}{2.5} \) ? Generate \( {l}_{2,i} \) and \( {l}_{1,i + 1} \) by Algorithm 1 \( {l}_{2,i}^{R} = {\text{ratio}}_{i}{l}_{2,i} \) \( {\mathbf{l}}_{1,i + 1}^{R} = {\text{ratio}}_{i}{\mathbf{l}}_{1,i + 1} \) Update \( {\mathbf{l}}_{1,i + 1},{\mathbf{l}}_{2,i},{\mathbf{l}}_{1,i + 1}^{R} \) and \( {\mathbf{l}}_{2,i}^{R} \) according to equation (11-12) when the corresponding conditions are met Update \( {\mathbf{l}}_{1,i + 1},{\mathbf{l}}_{2,i},{\mathbf{l}}_{1,i + 1}^{R} \) and \( {\mathbf{l}}_{2,i}^{R} \) according to equation (14) and (16) when the corresponding conditions are met Generate \( {B}_{i}\left( u\right) \) and \( {B}_{i}^{R}\left( u\right) \) by equation (3) and (9) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_3.jpg?x=359&y=148&w=1060&h=1698&r=0"/>

Fig. 5. The flowchart of the overlap elimination scheme for local smoothing.

<!-- Media -->

In addition to methods utilizing the half-length constraint, several other approaches have been proposed to tackle the overlap problem. For instance, Xu et al. [22] proposed a circumscribed corner rounding method based on double cubic B-splines. They employed a proportional adjustment strategy to determine the transition lengths when adjacent transition curves overlapped. In [23], two cubic uniform B-splines with the maximum error were constructed at each corner of the tool tip and the tool orientation path first. Subsequently, when overlaps occurred, the adjacent two splines were proportionally shrunk or replaced by only one uniform cubic B-spline. Finally, two complete B-splines were formed by connecting the pieces of uniform B-splines to smooth the tooltip and the tool orientation. Shi et al. [27] proposed a smoothing method using a pair of quintic \( \mathrm{{PH}} \) curves to round the corners of linear five-axis tool paths in the WCS. To guarantee the continuous variation of the tool orientation and no overlap, the Newton iteration method and an adjustment algorithm based on geometrical relationships were applied to determine control polygon lengths of PH curves. In [28], an asymmetrical PH spline-based \( {C}^{3} \) continuous corner smoothing algorithm was developed for five-axis tool paths with short segments in the WCS. Initially, transition curves were constructed as symmetrical PH splines with maximum error. When overlaps occurred, adjacent two splines were shortened with the same proportion. Zhao et al. [33] adopted a curvature-continuous B-spline with five control points to blend the adjacent straight lines. They addressed considerations such as curvature extrema, approximation error, and overlap elimination by formulating a linear programming problem. An adaptive subdivision scheme was proposed to obtain a suboptimal solution in real-time.

<!-- Meanless: 39-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

<!-- Media -->

<!-- figureText: 70 100 120 60 50 Linear toolpath Y (mm) Yan's method Proposed method 30 20 10 20 40 60 80 X (mm) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_4.jpg?x=332&y=152&w=1084&h=577&r=0"/>

Fig. 6. The butterfly-shaped tool path and its corner smoothed results.

<!-- figureText: 70 (b) Yan's method Proposed method (c) (d) 250 300 350 400 (a) Curvature (mm1) 50 20 10 50 100 150 200 Displacement (mm) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_4.jpg?x=160&y=826&w=1433&h=841&r=0"/>

Fig. 7. The comparison of curvatures among the butterfly-shaped tool path.

<!-- Media -->

In most local smoothing methods, overlaps were typically eliminated by imposing fixed-length or position constraints. Under such constraint, even if the transition length exceeds half the linear tool path segment without overlap, it also has to be shrunk [16], leading to increased curvature of the inserted curve and consequently reduced velocity along it. In contrast to conventional methods that primarily begin from a geometric perspective, this paper proposes an optimization scheme to

<!-- Meanless: 40-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

<!-- Media -->

<!-- figureText: 200 3000 Yan's method Proposed method Tangential acceleration \( \left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) \) 2000 1000 -1000 -2000 -3000 0 5 6 Time (s) (b) 0.12 Yan's method Proposed method 0.1 Tool tip error (mm) 0.08 0.06 0.02 0 5 6 Time (s) (d) Yan's method Proposed method Tangential velocity (mm/s) 150 100 2 3 7 Time (s) (a) \( {10}^{4} \) Yan's method 4 Tangential jerk \( \left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \) 2 -2 -4 -6 2 3 7 Time (s) (c) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_5.jpg?x=221&y=146&w=1304&h=1164&r=0"/>

Fig. 8. The tangential kinematics and the chord error profiles of the three-axis linear toolpath: (a) feedrate profile. (b) acceleration profile. (c) jerk profile. (d) chord error profile.

<!-- figureText: -15 tool orientatin feed direction 40 -10 30 X (mm) -20 Z (mm) -25 -30 -35 -40 tool tip -45 15 Y (mm) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_5.jpg?x=153&y=1442&w=654&h=408&r=0"/>

Fig. 9. One tool path of an impeller.

<!-- Media -->

eliminate overlap in five-axis local smoothing from the perspective of feedrate planning. This approach is motivated by bidirectional scanning in feedrate scheduling [34]. When the velocity variance between two adjacent critical points is too much to result in the violation of kinematic characteristics, the larger velocity has to be reduced. Typically, the curvature extreme point of the transition curve is regarded as a critical point, and the maximum allowable velocity is proportional to the scale of the transition curve. Therefore, the occurrence of the above phenomenon means that one transition curve is far away from the corner, while the other transition curve may be too close to the corner. This implies that the current toolpath smoothing is not optimal, and there is room for improvement by adjusting the scale of the transition curves to increase the velocity on the toolpath. To address this issue, when overlap occurs, the transition length of the two transition curves is allocated to minimize the difference between their minimum velocities.

<!-- Media -->

Table 2

the parameters used in Example 2.

<table><tr><td>Parameters</td><td>Constraints</td><td>Symbols</td><td>Values</td><td>Units</td></tr><tr><td>Smoothing parameters</td><td>Tool tip error</td><td>\( {\varepsilon }_{p,\max } \)</td><td>0.1</td><td>\( {mm} \)</td></tr><tr><td/><td>Ratio threshold</td><td>\( K \)</td><td>1/3</td><td>none</td></tr><tr><td/><td>Tool orientation error</td><td>\( {\varepsilon }_{r,\max } \)</td><td>0.05</td><td>deg</td></tr><tr><td rowspan="7">Feedrate scheduling parameters</td><td>Command feedrate</td><td>\( {F}_{cmd} \)</td><td>100</td><td>mm/s</td></tr><tr><td>Tangential acceleration</td><td>\( {A}_{t,\max } \)</td><td>1000</td><td>\( {mm}/{s}^{2} \)</td></tr><tr><td>Normal acceleration</td><td>\( {A}_{n,\max } \)</td><td>1000</td><td>\( {mm}/{s}^{2} \)</td></tr><tr><td>Tangential acceleration</td><td>\( {J}_{t,\max } \)</td><td>10000</td><td>\( {mm}/{s}^{3} \)</td></tr><tr><td>Normal acceleration</td><td>\( {J}_{n,\text{ max }} \)</td><td>10000</td><td>\( {mm}/{s}^{3} \)</td></tr><tr><td>Chord error</td><td>\( {\delta }_{\max } \)</td><td>0.001</td><td>\( {mm} \)</td></tr><tr><td>Interpolation period</td><td>\( {T}_{s} \)</td><td>1</td><td>ms</td></tr></table>

<!-- Media -->

The main innovation of this paper lies in the overlap elimination scheme. Unlike most existing methods that rely on the half-length constraint or other asymmetric spline-based methods, this approach considers the variation in feedrate on adjacent transition curves during overlap elimination. This consideration leads to more uniform velocities at corners, ultimately enhancing overall machining efficiency. Moreover, this overlap elimination scheme is not limited to the quintic B-spline curve discussed in Section 2.1, it can also be applied to other B-spline curves after simple modification, as demonstrated in Appendix 1. The remainder of this paper is structured as follows: In Section 2, a specific transition curve is presented to illustrate the proposed scheme. This section covers the overlap elimination scheme, tool tip position smoothing, and tool orientation smoothing. Section 3 focuses on simulations, where the results are compared with those obtained using traditional methods. Finally, Section 4 provides the conclusions drawn from the study and outlines potential directions for future research.

<!-- Meanless: 41-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

<!-- Media -->

<!-- figureText: Original toolpath The proposed method 50 60 70 80 90 100 Displacement (mm) ✘ 100 80 60 Axis position (mm/deg) 40 20 0 -40 -60 20 30 40 -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_6.jpg?x=158&y=153&w=1431&h=840&r=0"/>

Fig. 10. The comparison of original and smoothed tool path for each individual drive.

<!-- Media -->

## 2. Tool path smoothing with overlap elimination

In five-axis CNC machining, the coordinates of the tool tip point and tool orientation vector with a unit magnitude of the cutter location (CL) data are usually defined as \( P = {\left\lbrack  {P}_{x},{P}_{y},{P}_{z}\right\rbrack  }^{T} \) and \( O = {\left\lbrack  {O}_{i},{O}_{j},{O}_{k}\right\rbrack  }^{T} \) in the WCS. For the typical 5-axis milling machine tool of table-rotating/tilting (TRT) consisting of three translational axes(X,Y,Z)and two rotary axes (A,C),the tool orientation data \( {O}_{i} \) and the corresponding rotary axes \( {R}_{i} \) \( = {\left\lbrack  {A}_{i},{C}_{i}\right\rbrack  }^{T} \) can be converted to each other by the following equation:

\[\left\{  \begin{matrix} A = {\cos }^{-1}\left( {O}_{k}\right) \\  C = r,\text{ if }A = 0\text{ or }A = \pi \\  C = 2{n}_{1}\pi  + {\cos }^{-1}\left( {-\frac{{O}_{j}}{\sin \left( A\right) }}\right) ,\text{ if }\frac{{O}_{i}}{\sin \left( A\right) } > 0 \\  C = 2{n}_{2}\pi  - {\cos }^{-1}\left( {-\frac{{O}_{j}}{\sin \left( A\right) }}\right) ,\text{ if }\frac{{O}_{i}}{\sin \left( A\right) } < 0 \end{matrix}\right.  \tag{1.1}\]

\[\left\{  \begin{matrix} {O}_{i} = \sin \left( A\right) \sin \left( C\right) \\  {O}_{j} =  - \sin \left( A\right) \cos \left( C\right) \\  {O}_{k} = \cos \left( A\right)  \end{matrix}\right.  \tag{1.2}\]

Where \( r \in  \mathbb{R} \) and \( {n}_{1},{n}_{2} \in  \mathbb{Z} \) ,so \( A \in  \left\lbrack  {0,\pi }\right\rbrack \) and \( C \in  \mathbb{R} \) . In this paper, \( O \) is converted to \( R \) using Eq. (1.1). For the sake of simplicity,when \( \sin \left( A\right)  \neq  0 \) ,select \( {n}_{1} \) or \( {n}_{2} \) such that \( C \in  \left\lbrack  {0,{2\pi }}\right\rbrack \) ; when \( \sin \left( A\right)  = 0 \) ,select \( C \) as the average value of the C-axis coordinates corresponding to the adjacent two tool orientations. Subsequently, considering the predefined error tolerances and parameter synchronization, the proposed method smooths the tool tip position in the WCS and tool orientation in the MCS. Though the smoothing error of the tool orientation vector is controlled in the MCS, the actual smoothing error of the tool orientation vector can also be limited in the WCS [35].

#### 2.1.The inserted smooth curves

To clearly illustrate the proposed overlap elimination scheme, the widely used quintic B-spline curve \( \left\lbrack  {{13},{16},{19},{24}}\right\rbrack \) is selected as an example. The B-spline curve is mathematically represented as follows:

\[B\left( u\right)  = \mathop{\sum }\limits_{{i = 0}}^{6}{N}_{i,p}\left( u\right) {P}_{i} \tag{2}\]

where \( p = 5 \) is the degree of the B-spline curve, \( {N}_{i,p}\left( u\right) \) is the B-spline basis function defined on knot vector \( U = \lbrack 0,0,0,0,0,0,{0.5},1,1,1,1 \) , \( 1,1\rbrack \) ,which can be evaluated by recursive Cox-de Boor formula. The B-spline curve is defined by seven control points \( P = \left\lbrack  {{P}_{0},{P}_{1},{P}_{2},{P}_{3},{P}_{4},{P}_{5}}\right. \) , \( \left. {P}_{6}\right\rbrack \) . While this paper illustrates the subsequent overlap elimination method using this specific B-spline as an example, it is universally applicable to various B-spline curves. The only necessary adjustments involve modifying certain constants in the formula, as detailed in the Appendix 1. As shown in Fig. \( 1,{P}_{a},{P}_{b} \) and \( {P}_{c} \) represent the end points of two adjacent linear tool path segments,with \( \theta \) is the included angle formed by \( \overrightarrow{{P}_{b}{P}_{a}} \) and \( \overrightarrow{{P}_{b}{P}_{c}} \) . Ensuring the \( {C}^{3} \) continuity between the linear segment and the inserted spline curve, the positional relations of the control points can be derived as follows [16]:

<!-- Meanless: 42-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

\[\left\{  \begin{array}{l} {P}_{0} = {P}_{b} + {2.5}{l}_{1}{e}_{1} \\  {P}_{1} = {P}_{b} + 2{l}_{1}{e}_{1} \\  {P}_{2} = {P}_{b} + {l}_{1}{e}_{1} \\  {P}_{3} = {P}_{b} \\  {P}_{4} = {P}_{b} + {l}_{2}{e}_{2} \\  {P}_{5} = {P}_{b} + 2{l}_{2}{e}_{2} \\  {P}_{6} = {P}_{b} + 2{l}_{3}{e}_{3} \end{array}\right.  \tag{3}\]

<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

<!-- Media -->

<!-- figureText: Original toolpath 45 40 35 X (mm) 85 -15 Yan's method Zhang's method -20 -25 Z (mm) -30 -35 -40 -45 20 10 -10 -20 Y (mm) (a) 106 Original toolpath 104 Yan's method 102 Proposed method Zhang's method 100 98 C (deg) 94 92 90 88 86 84 65 70 75 A (deg) (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_7.jpg?x=325&y=151&w=1098&h=1225&r=0"/>

Fig. 11. The comparison of original and smoothed tool paths: (a) Tool tip paths. (b) Tool orientation paths.

<!-- figureText: Yan's method 100 1.8 Proposed method Zhang's method 1.6 1.4 Curvature (mm1) 1.2 0.8 0.6 0.4 0.2 0 0 20 40 60 80 Displacement (mm) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_7.jpg?x=229&y=1471&w=1288&h=629&r=0"/>

Fig. 12. The comparison of curvatures among the smoothed tool paths.

<!-- Meanless: 43-->


<!-- figureText: 120 Yan's method Proposed method Zhang's method 1.5 2.5 Time (s) (a) 1.5 2 2.5 Time (s) (b) 2 2.5 Time (s) (c) 100 Tangential velocity (mm/s) 60 40 0.5 1000 Tangential acceleration \( \left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right. \) 500 -1000 0.5 \( \times  {10}^{4} \) Tangential jerk \( \left( {\mathrm{{mm}}/{\mathrm{s}}^{3}}\right) \) 0.5 0 -0.5 -1 0.5 1 -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_8.jpg?x=152&y=144&w=650&h=1786&r=0"/>

Fig. 13. The tangential kinematic performance: (a) velocity profile. (b) acceleration profile. (c) jerk profile.

<!-- Media -->

In the provided equation, \( {e}_{1} = \frac{\overrightarrow{{P}_{b}{P}_{a}}}{\left| \overrightarrow{{P}_{b}{P}_{a}}\right| } \) and \( {e}_{2} = \frac{\overrightarrow{{P}_{b}{P}_{c}}}{\left| \overrightarrow{{P}_{b}{P}_{c}}\right| },{l}_{1} \) and \( {l}_{2} \) represent the two characteristic lengths of the B-spline curve. The approximation error is defined as the geometric deviation between the transition curve \( B\left( u\right) \) and the polyline segment \( {P}_{0} - {P}_{3} - {P}_{6} \) . When \( {l}_{1} = \) \( {l}_{2} = l \) ,the B-spline curve is symmetrical. The maximum approximation error \( \varepsilon \) occurs at the mid-point \( \left( {\mathrm{u} = {0.5}}\right) \) of the B-spline transition curve and \( \varepsilon  = \frac{3\cos \left( {\theta /2}\right) }{4}l \) . To satisfy the requirement of machining accuracy, \( \varepsilon \) should be less than \( {\varepsilon }_{\max } \) ,leading to \( l \leq  \frac{4{\varepsilon }_{\max }}{3\cos \left( {\theta /2}\right) } \) . Even if \( {l}_{1} \neq  {l}_{2} \) ,as long as \( {l}_{1} \leq  \frac{4{\varepsilon }_{\max }}{3\cos \left( {\theta /2}\right) } \) and \( {l}_{2} \leq  \frac{4{\varepsilon }_{\max }}{3\cos \left( {\theta /2}\right) } \) ,it implies that \( \varepsilon  \leq  {\varepsilon }_{\max } \) [16,28].

The curvature extreme of the inserted curve is a pivotal factor as it directly impacts the velocity along the curve. When the B-spline curve is symmetrical, the curvature extreme is located at the mid-point, which can be analytically derived as follows:

\[{k}_{\max } = \frac{4\sqrt{2}\left| {\sin \left( \theta \right) }\right| }{{5l}{\left( \sqrt{1 - \cos \left( \theta \right) }\right) }^{3}} \tag{4}\]

However,when \( {l}_{1} \neq  {l}_{2} \) ,the calculation process and formula are relatively complex. As an alternative, numerical methods could be implemented. Additionally,a matrix \( M \) similar to the one presented in reference [29] can be pre-constructed to enhance computational efficiency.

### 2.2. Overlap elimination scheme

Given the tool tip position denoted by \( \left\{  {{P}_{0},\;{P}_{1},\;\ldots ,\;{P}_{n}}\right\} \) ,the length of each linear tool path segment \( {P}_{i}{P}_{i + 1}\left( {i = 1,2,\ldots ,n - 2}\right) \) is represented by \( {L}_{i} \) ,and the included angle at corner \( {P}_{i} \) is denoted as \( {\theta }_{i} \) . The transition curves at \( {P}_{i} \) are represented by \( {B}_{i}\left( u\right) \) ,with their characteristic lengths \( {l}_{1,i} \) and \( {l}_{2,i}\left( {i = 1,2,\ldots ,n - 1}\right) \) determined accordingly. When the characteristic lengths of transition curves are solely constrained by the user-specified approximation error, it is possible for adjacent curves to overlap, as shown in Fig. 2. Eq. (3) and Fig. 2 illustrate that within the linear tool path segment \( {P}_{i}{P}_{i + 1} \) ,the transition length of the transition curve \( {B}_{i}\left( u\right) \) only depend on \( {l}_{2,i} \) ,while the transition length of the transition curve \( {B}_{i + 1}\left( u\right) \) only depend on \( {l}_{1,i + 1} \) . Therefore,it’s possible to solve the overlap in the linear segment \( {P}_{i}{P}_{i + 1} \) by adjusting \( {l}_{2,i} \) and \( {l}_{1,i + 1} \) ,while other characteristic lengths such as \( {l}_{1,i} \) and \( {l}_{2,i + 1} \) require no consideration.

Efficiency in machining is directly impacted by the feedrate on the smoothed toolpaths. Therefore, it is crucial to focus on the feedrate on transition curves, particularly the local minimum feedrate. Generally, the local minimum feedrate on the transition curves is determined by the curvature extreme, and is calculated considering expected constraints such as chord error, normal acceleration as well as normal jerk. The calculation formulas are as follows [36]:

<!-- Meanless: 44-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

\[\left\{  \begin{matrix} {v}_{\text{limit },C} = \frac{2}{{T}_{s}}\sqrt{\frac{2\delta }{k} - {\delta }^{2}} \approx  \frac{2}{{T}_{s}}\sqrt{\frac{2\delta }{k}} \\  {v}_{\text{limit },A} = \sqrt{\frac{{A}_{n}}{k}} \\  {v}_{\text{limit },J} = \sqrt[3]{\frac{{J}_{n}^{2}}{k}} \end{matrix}\right.  \tag{5}\]

<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

<!-- Media -->

<!-- figureText: (Proposed method) 40 C (Proposed method C (Zhang's method) 30 20 A/C-velocity (deg/s) 10 0 -10 -20 -30 -40 -50 1.5 2 (b) 400 300 A/C-acceleration (deg \( /{\mathrm{s}}^{2} \) ) 200 100 -100 -200 -300 0 0.5 1.5 2.5 Time (s) (d) 1 -3 0.5 Time (s) (f) (Zhang's method) 40 X/Y/Z-velocity (mm/s) 20 0 -20 -40 0.5 2.5 (a) 500 400 300 \( \mathrm{X}/\mathrm{Y}/\mathrm{Z} \) -acceleration \( \left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) \) 200 100 -200 -300 -400 -500 0 0.5 1.5 2.5 Time (s) (c) \( \times  {10}^{4} \) 6 X/Y/Z-jerk (mm/s \( {}^{3} \) Time (s) (e) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_9.jpg?x=222&y=143&w=1309&h=1725&r=0"/>

Fig. 14. The axial kinematic performance: (a) velocity profile for X/Y/Z axes. (b) velocity profile for A/C axes. (c) acceleration profile for X/Y/Z axes. (d) acceleration profile for \( \mathrm{A}/\mathrm{C} \) axes. (e) jerk profile for \( \mathrm{X}/\mathrm{Y}/\mathrm{Z} \) axes. (f) jerk profile for \( \mathrm{A}/\mathrm{C} \) axes.

<!-- Meanless: 45-->


<!-- figureText: 0.06 0.03 Yan's method Proposed method 0.025 Zhang's method Tool orientation error (deg) 0.02 0.015 0.07 0.005 0.5 1.5 2.5 Time (s) (b) Yan's method Proposed method 0.05 Zhang's method Tool tip error (mm) 0.04 0.03 0.02 0.01 0 0.5 1 2 2.5 Time (s) (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_10.jpg?x=224&y=154&w=1293&h=567&r=0"/>

Fig. 15. The tool tip and tool orientation errors caused by corner smoothing: (a) tool tip position error. (b) tool orientation error.

<!-- figureText: 10 tool tip 5 0 X/mm feed direction Z/mm tool orientation -5 10 5 Y/mm -10 -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_10.jpg?x=150&y=828&w=653&h=411&r=0"/>

Fig. 16. A fan-shaped tool path.

<!-- Media -->

where \( {T}_{s} \) is the interpolation period,and \( k \) is the curvature extreme. \( \delta \) , \( {A}_{n} \) ,and \( {J}_{n} \) are the chord error,normal acceleration,and normal jerk limitations, respectively. From the first two equations of Eq. (5), the local minimum feedrate is inversely proportional to the square root of the curvature extreme,i.e., \( v \propto  \sqrt{\frac{1}{k}} \) . While the last equation shows that it is inversely proportional to the cube root of the curvature extreme, i.e., \( v \propto  \sqrt[3]{\frac{1}{k}} \) . On the one hand,chord error is key to the machining quality, which should pay more attention to. On the other hand, the normal acceleration limit is usually much smaller than the normal jerk limit. Consequently,the feedrate is more likely to be limited by \( {v}_{\text{limit },A} \) rather than \( {v}_{\text{limit },J} \) . Therefore,the local minimum feedrate on the transition curve is simplified to be inversely proportional to the square root of the curvature extreme, that is:

\[v \propto  \sqrt{\frac{1}{k}} \tag{6}\]

For symmetric B-spline curves,where \( l = {l}_{1} = {l}_{2} \) ,substituting Eq. (4)

into Eq. (6) yields:

\[v \propto  \sqrt{\frac{{5l}{\left( \sqrt{1 - \cos \left( \theta \right) }\right) }^{3}}{4\sqrt{2}\left| {\sin \left( \theta \right) }\right| }} \tag{7}\]

As \( \theta \) is fixed for a certain corner,the local minimum feedrate \( v \) is proportional to the square root of the characteristic length \( l \) . When the user-specified approximation error of the tool tip position \( {\varepsilon }_{p,\max } \) is provided,to maximize \( v,l \) can be directly set as \( \frac{4{\varepsilon }_{p,\max }}{3\cos \left( {\theta /2}\right) } \) . However,overlap occurs when \( {2.5} \times  \left( {{l}_{i} + {l}_{i + 1}}\right)  > {L}_{i} \) ,as depicted in Fig. 2. To address this overlap issue,it is necessary to decrease either \( {l}_{i} \) or \( {l}_{i + 1} \) . How to update \( {l}_{i} \) and \( {l}_{i + 1} \) will significantly impact the final machining time.

Let \( {v}_{i} \) and \( {v}_{i + 1} \) represent the local minimum feedrate on \( {B}_{i}\left( u\right) \) and \( {B}_{i + 1}\left( u\right) \) respectively. According to Eq. (7), \( {v}_{i} \) is proportional to \( \sqrt{\frac{5{l}_{i}{\left( \sqrt{1 - \cos \left( {\theta }_{i}\right) }\right) }^{3}}{4\sqrt{2}\left| {\sin \left( {\theta }_{i}\right) }\right| }} \) and \( {v}_{i + 1} \) is proportional to \( \sqrt{\frac{5{l}_{i + 1}{\left( \sqrt{1 - \cos \left( {\theta }_{i + 1}\right) }\right) }^{3}}{4\sqrt{2}\left| {\sin \left( {\theta }_{i + 1}\right) }\right| }} \) . To ensure efficient utilization of the linear segment for local smoothing, we impose a constraint on updating the characteristic lengths: \( {2.5} \times  \left( {{l}_{i} + {l}_{i + 1}}\right)  = {L}_{i} \) . Obviously, \( {v}_{i} \) increases with the increase of \( {l}_{i} \) ,while \( {v}_{i + 1} \) decreases as \( {l}_{i} \) increases. When \( {l}_{i} \) and \( {l}_{i + 1} \) are unevenly allocated,resulting in \( {v}_{i + 1} \) being significantly larger than \( {v}_{i} \) ,even if the tool starts at point \( {B}_{i}\left( {0.5}\right) \) with an initial velocity of \( {v}_{i} \) ,continuously accelerates,it may not reach the velocity of \( {v}_{i + 1} \) by the time it reaches point \( {B}_{i + 1}\left( {0.5}\right) \) . In such cases, appropriately increasing \( {l}_{i} \) can enhance the initial velocity \( {v}_{i} \) at \( {B}_{i}\left( {0.5}\right) \) . Although \( {l}_{i + 1} \) and \( {v}_{i + 1} \) decrease as a consequence,considering \( {v}_{i + 1} \) cannot be achieved, the velocity curve will shift upward throughout the entire motion process,as depicted in Fig. 3(a). On the contrary,if \( {v}_{i} \) is significantly larger than \( {v}_{i + 1} \) ,in order to reduce the velocity to \( {v}_{i + 1} \) when the tool reaches point \( {B}_{i + 1}\left( {0.5}\right) \) ,its initial velocity at point \( {B}_{i}\left( {0.5}\right) \) may need to be significantly lower than \( {v}_{i} \) . Similarly,appropriately reducing \( {l}_{i} \) can shift the velocity curve upward,as depicted in Fig. 3(b). To avoid the occurrence of such extreme situations, our optimization objective is to minimize the gap between \( {v}_{i} \) and \( {v}_{i + 1} \) .

Therefore, the primary objective in the overlap elimination scheme is to minimize the difference between \( {v}_{i} \) and \( {v}_{i + 1} \) . Additionally,to prevent the occurrence of situations where some transition curves are too small, the ratio between the transition lengths with respect to the length of linear segment is constrained to be greater than a constant \( \mathrm{K}\left( {0 \leq  \mathrm{K} \leq  {0.5}}\right) \) . That is, \( \frac{\mathrm{K}{L}_{i}}{2.5} \leq  {l}_{i} \) and \( \frac{\mathrm{K}{L}_{i}}{2.5} \leq  {l}_{i + 1} \) . However,this constraint can be ignored if the approximation error is larger than \( {\varepsilon }_{p,\max } \) . Formally, when overlap occurs,i.e.,when \( \frac{10}{3}\left( {\frac{{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i}/2}\right) } + \frac{{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i + 1}/2}\right) }}\right)  > {L}_{i} \) ,an optimization model with these constraints is constructed as follows:

<!-- Meanless: 46-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

\[\min \left| {{a}_{i}\sqrt{{l}_{i}} - {a}_{i + 1}\sqrt{{l}_{i + 1}}}\right| \]

\[s.t.\left\{  \begin{matrix} {2.5} \times  \left( {{l}_{i} + {l}_{i + 1}}\right)  = {L}_{i} \\  \min \left( {\frac{\mathrm{K}{L}_{i}}{2.5},\frac{4{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i}/2}\right) }}\right)  \leq  {l}_{i} \leq  \frac{4{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i}/2}\right) } \\  \min \left( {\frac{\mathrm{K}{L}_{i}}{2.5},\frac{4{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i + 1}/2}\right) }}\right)  \leq  {l}_{i + 1} \leq  \frac{4{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i + 1}/2}\right) } \end{matrix}\right.  \tag{8}\]

<!-- Media -->

<!-- figureText: Original toolpath The proposed method 25 30 35 40 50 Displacement (mm) 140 120 100 Axis position (mm/deg) 80 60 40 20 20 -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_11.jpg?x=154&y=153&w=1436&h=820&r=0"/>

Fig. 17. The comparison of original and smoothed tool path for each individual drive.

<!-- Media -->

where \( {a}_{i} = \sqrt{\frac{5{\left( \sqrt{1 - \cos \left( {\theta }_{i}\right) }\right) }^{3}}{4\sqrt{2}\left| {\sin \left( {\theta }_{i}\right) }\right| }} \) and \( {a}_{i + 1} = \sqrt{\frac{5{\left( \sqrt{1 - \cos \left( {\theta }_{i + 1}\right) }\right) }^{3}}{4\sqrt{2}\left| {\sin \left( {\theta }_{i + 1}\right) }\right| }} \) . In this optimization model,it is assumed that \( {B}_{i}\left( u\right) \) and \( {B}_{i + 1}\left( u\right) \) are symmetric,providing a simple and rational estimate of the extreme curvature. While this assumption may not accurately reflect the extreme curvature, it serves as a pragmatic approach for estimating curvature extremes within the context of the model. It is important to note that despite this assumption, the subsequent feed rate planning process still involves searching for the true curvature extreme points on transition curves to plan the velocity. The solution to this optimization model can be obtained using Algorithm 1, outlined in Table 1.

### 2.3. Corner smoothing of tool tip path

As previously discussed, the characteristic lengths are initially determined based on the user-specified approximation error. Subsequently, overlaps are eliminated by reducing the corresponding characteristic lengths using the overlap elimination scheme in Section 2.2. In summary,the characteristic lengths \( {l}_{1,i} \) and \( {l}_{2,i}\left( {i = 1,2,\ldots ,n - 1}\right) \) are determined through the following steps, enabling the generation of transition curves \( {B}_{i}\left( u\right) \) using Eq. (3).

Step 1: Set \( i = 1 \) ,calculate \( {l}_{\max ,i} = \frac{4{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i}/2}\right) } \) ,and let \( {l}_{1,i} = \min \left( {l}_{\max ,i}\right. \) , \( \left. \frac{{L}_{i - 1}}{2.5}\right) \) .

Step 2: If \( i < n - 1 \) ,calculate \( {l}_{\max ,i + 1} = \frac{4{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i + 1}/2}\right) } \) ,then set \( {l}_{2,i} = {l}_{\max ,i} \) and \( {l}_{1,i + 1} = {l}_{\max ,i + 1} \) . Otherwise,go to Step 5.

Step 3: If \( {l}_{2,i} + {l}_{1,i + 1} > \frac{{L}_{i}}{2.5} \) ,determine \( {l}_{2,i} \) and \( {l}_{1,i + 1} \) using \( {l}_{ - } \) generation \( \left( {\theta }_{i}\right. \) ,

\( \left. {{\theta }_{i + 1},{L}_{i},K,{\varepsilon }_{p,\max }}\right) \) .

Step 4: Let \( i = i + 1 \) and go to Step 2.

Step 5: Let \( {l}_{2,i} = \min \left( {{l}_{\max ,i},\frac{{L}_{i}}{2.5}}\right) \) and terminate the algorithm.

### 2.4. Corner smoothing of tool orientation

In addition to smoothing the tool tip path, it is also essential to smooth the tool orientation. The tool orientation vector data \( \left\{  {{O}_{0},{O}_{1}}\right. \) , \( \left. {\ldots ,{O}_{n}}\right\} \) has been transformed to \( \left\{  {{R}_{0},{R}_{1},\ldots ,{R}_{n}}\right\} \) ,where \( {R}_{i} = \left( {A}_{i}\right. \) , \( {C}_{i} \) ). The transition curves \( {B}_{i}^{R}\left( u\right) \left( {i = 1,2,\ldots ,n - 1}\right) \) are inserted to facilitate the smoothing of the corresponding rotary axes in the MCS. Similarly,two characteristic lengths \( {l}_{1,i}^{R} \) and \( {l}_{2,i}^{R} \) determine the transition curve \( {B}_{i}^{R}\left( u\right) \) ,as depicted in Fig. 4. Here,the \( A - C \) coordinate system is established to represent the rotary axes,and the \( i - j - k \) coordinate system is established to represent the actual tool orientation vector. The control points of the transition curve \( {B}_{i}^{R}\left( u\right) \) can be calculated as follows:

\[\left\{  \begin{matrix} {P}_{0,i}^{R} = {R}_{i} + {2.5}{l}_{1,i}^{R}{e}_{1,i} \\  {P}_{1,i}^{R} = {R}_{i} + 2{l}_{1,i}^{R}{e}_{1,i} \\  {P}_{2,i}^{R} = {R}_{i} + {l}_{1,i}^{R}{e}_{1,i} \\  {P}_{3,i}^{R} = {R}_{i} \\  {P}_{4,i}^{R} = {R}_{i} + {l}_{2,i}^{R}{e}_{2,i} \\  {P}_{5,i}^{R} = {R}_{i} + 2{l}_{2,i}^{R}{e}_{2,i} \\  {P}_{6,i}^{R} = {R}_{i} + {2.5}{l}_{2,i}^{R}{e}_{2,i} \end{matrix}\right.  \tag{9}\]

where \( {e}_{1,i} = \frac{\overrightarrow{{R}_{i}{R}_{i - 1}}}{\left| \overrightarrow{{R}_{i}{R}_{i - 1}}\right| } \) and \( {e}_{2,i} = \frac{\overrightarrow{{R}_{i}{R}_{i + 1}}}{\left| \overrightarrow{{R}_{i}{R}_{i + 1}}\right| } \) .

Furthermore, in addition to constraining the user-specified approximation error of tool orientation \( {\varepsilon }_{r,\max } \) ,it is essential to ensure the continuity of the angles with respect to tool tip displacements, as the movement of tool orientation is dependent on the displacements of the tool tip. In the derivation process of the synchronization of tool tip position and orientation in prior studies \( \left\lbrack  {{17},{23}}\right\rbrack \) ,it has been demonstrated that continuity can be ensured if:

<!-- Meanless: 47-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

\[\frac{{l}_{1,i}^{R}}{{l}_{1,i}} = \frac{{L}_{i - 1}^{R}}{{L}_{i - 1}} = {\text{ ratio }}_{i - 1}\text{ and }\frac{{l}_{2,i}^{R}}{{l}_{2,i}} = \frac{{L}_{i}^{R}}{{L}_{i}} = {\text{ ratio }}_{i} \tag{10}\]

<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

<!-- Media -->

<!-- figureText: 6 Original toolpath Proposed method Zhang's method 10 5 0 -5 -5 -10 -10 -15 X (mm) Original toolpath Yan's method Proposed method Zhang's method 4 14 16 18 A (deg) 4 Yan's method 2 Z (mm) 0 -2 10 5 Y (mm) (a) 140 130 120 C (deg) 110 90 0 2 (b) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_12.jpg?x=328&y=151&w=1093&h=1229&r=0"/>

Fig. 18. The comparison of original and smoothed tool paths: (a) Tool tip paths. (b) Tool orientation paths.

<!-- figureText: Yan's method (b) 50 Proposed method (a) Zhang's method Curvature (mm1) 4 3 0 10 20 30 40 Displacement (mm) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_12.jpg?x=338&y=1475&w=1078&h=556&r=0"/>

Fig. 19. The comparison of curvatures among the smoothed tool paths.

<!-- Meanless: 48-->


<!-- figureText: 120 Yan's method Proposed method Zhang's method 1.5 Time (s) (a) 1.5 Time (s) (b) 1.5 2 Time (s) (c) 100 Tangential velocity (mm/s) 80 60 40 0.5 1000 Tangential acceleration \( \left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) \) 500 -500 -1000 0.5 Tangential jerk (mm/s \( {}^{3} \) ) 0.5 0 -0.5 0.5 -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_13.jpg?x=153&y=147&w=658&h=1822&r=0"/>

Fig. 20. The tangential kinematic performance: (a) velocity profile. (b) acceleration profile. (c) jerk profile.

<!-- Media -->

where \( {L}_{i}^{R} = \left| {{R}_{i} - {R}_{i + 1}}\right| \) . When consecutive points in the tool path with the same tool tip position but different tool orientation, Eq. (10) is not applicable due to \( {L}_{i} = 0 \) . In this case,the toolpath needs to be processed, such as splitting the path at these two points into two paths or appropriately offsetting these two points to form two distinct tool tip positions. Thus, the characteristic lengths are initially determined as \( {l}_{1,i}^{R} = {\operatorname{ratio}}_{i - 1}{l}_{1,i}\; \) and \( \;{l}_{2,i}^{R} = {\operatorname{ratio}}_{i}{l}_{2,i}.\; \) Since \( \;{l}_{2,i}^{R} + \;{l}_{1,i + 1}^{R} = \) \( {\operatorname{ratio}}_{i}\left( {{l}_{2,i} + {l}_{1,i + 1}}\right)  \leq  {\operatorname{ratio}}_{i}\frac{{L}_{i}}{2.5} = \frac{{L}_{i}^{R}}{2.5} \) ,there won’t be overlaps in the angular space.

In case the approximation error exceeds the specified threshold, the characteristic lengths of \( {B}_{i}^{R}\left( u\right) \) should be reduced. To maintain Eq. (10), the characteristic lengths of \( {B}_{i}\left( u\right) \) also need to be decreased propor-

tionally. Specifically,for \( i = 1,2,\ldots ,n - 1 \) ,if \( {l}_{1,i}^{R} \geq  \frac{4{\varepsilon }_{r,\max }}{3\cos \left( {{\alpha }_{i}/2}\right) } \) ,where \( {\alpha }_{i} = \) \( {\cos }^{-1}\left( \frac{\overline{{R}_{i}{R}_{i - 1}} \cdot  \overline{{R}_{i}{R}_{i + 1}}}{\left| \overline{{R}_{i}{R}_{i - 1}}\right| \left| \overline{{R}_{i}{R}_{i + 1}}\right| }\right) \) represents the included angle formed by \( \overrightarrow{{R}_{i}{R}_{i - 1}} \) and \( \overrightarrow{{R}_{i}{R}_{i + 1}} \) in the angular space, \( {l}_{1,i}^{R} \) and \( {l}_{1,i} \) are updated as follows:

\[\left\{  \begin{matrix} {l}_{1,i}^{R} = \frac{4{\varepsilon }_{r,\max }}{3\cos \left( {{\alpha }_{i}/2}\right) } \\  {l}_{1,i} = \frac{{l}_{1,i}^{R}}{{rati}{o}_{i - 1}} \end{matrix}\right.  \tag{11}\]

If \( {l}_{2,i}^{R} \geq  \frac{4{\varepsilon }_{r,\max }}{3\cos \left( {{\alpha }_{i}/2}\right) },{l}_{2,i}^{R} \) and \( {l}_{2,i} \) are updated as follows:

\[\left\{  \begin{matrix} {l}_{2,i}^{R} = \frac{4{\varepsilon }_{r,\max }}{3\cos \left( {{\alpha }_{i}/2}\right) } \\  {l}_{2,i} = \frac{{l}_{2,i}^{R}}{{ratio}{i}_{i}} \end{matrix}\right.  \tag{12}\]

When the characteristic lengths of \( {B}_{i}\left( u\right) \) are reduced according to Eqs. (11) and (12), some linear tool path segments may remain between the linear tool path segment \( {P}_{i - 1}{P}_{i} \) and \( {P}_{i}{P}_{i + 1} \) . If certain conditions are met,the adjacent transition curves \( {B}_{i - 1}\left( u\right) \) and \( {B}_{i + 1}\left( u\right) \) can be amplified. Specifically,for \( i = 1,2,\ldots ,n - 2 \) ,when \( {l}_{1,i + 1} \) has been reduced to constraint the approximation error of the tool orientation, and

\[{l}_{2,i} < \frac{4{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i}/2}\right) }\text{ and }{l}_{2,i}^{R} < \frac{4{\varepsilon }_{r,\max }}{3\cos \left( {{\alpha }_{i}/2}\right) } \tag{13}\]

\( {l}_{2,i} \) and \( {l}_{2,i}^{R} \) are updated as follows:

\[\left\{  \begin{matrix} {l}_{2,i} = \min \left( {\frac{4{\varepsilon }_{p,\max }}{3\cos \left( \frac{{\theta }_{i}}{2}\right) },\frac{4{\varepsilon }_{r,\max }}{3{\operatorname{ratio}}_{i}\cos \left( \frac{{\alpha }_{i}}{2}\right) },\frac{{L}_{i}}{2.5} - {l}_{1,i + 1}}\right) \\  {l}_{2,i}^{R} = {\operatorname{ratio}}_{i}{l}_{2,i} \end{matrix}\right.  \tag{14}\]

When \( {l}_{2,i} \) has been reduced,and

\[{l}_{1,i + 1} < \frac{4{\varepsilon }_{p,\max }}{3\cos \left( {{\theta }_{i + 1}/2}\right) }\text{ and }{l}_{1,i + 1}^{R} < \frac{4{\varepsilon }_{r,\max }}{3\cos \left( {{\alpha }_{i + 1}/2}\right) } \tag{15}\]

\( {l}_{1,i + 1} \) and \( {l}_{1,i + 1}^{R} \) are updated as follows:

<!-- Meanless: 49-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

\[\left\{  \begin{array}{r} {l}_{1,i + 1} = \min \left( {\frac{4{\varepsilon }_{p,\max }}{3\cos \left( \frac{{\theta }_{i + 1}}{2}\right) },\frac{4{\varepsilon }_{r,\max }}{{3rati}{o}_{i}\cos \left( \frac{{\alpha }_{i + 1}}{2}\right) },\frac{{L}_{i}}{2.5} - {l}_{2,i}}\right) \\  {l}_{1,i + 1}^{R} = {rati}{o}_{i}{l}_{1,i + 1} \end{array}\right.  \tag{16}\]

<!-- Media -->

<!-- figureText: (Proposed method 60 C (Proposed method) C (Zhang's method) 40 A/C-velocity (deg/s) -20 -40 -60 -80 1.5 2 (b) 1500 1000 A/C-acceleration (deg/s \( {}^{2} \) ) 500 0 -1000 -1500 0 0.5 1.5 2.5 Time (s) (d) 2 0.5 Time (s) (f) (Zhang's method) 20 X/Y/Z-velocity (mm/s) 10 -10 -20 -30 -40 0.5 2.5 (a) 600 400 \( \mathrm{X}/\mathrm{Y}/\mathrm{Z} \) -acceleration \( \left( {\mathrm{{mm}}/{\mathrm{s}}^{2}}\right) \) 200 -200 -600 0 0.5 1.5 2.5 Time (s) (c) \( \times  {10}^{5} \) 0.5 X/Y/Z-jerk (mm/s \( {}^{3} \) ) -0.5 -1 -1.5 Time (s) (e) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_14.jpg?x=222&y=140&w=1306&h=1728&r=0"/>

Fig. 21. The axial kinematic performance: (a) velocity profile for X/Y/Z axes. (b) velocity profile for A/C axes. (c) acceleration profile for X/Y/Z axes. (d) acceleration profile for \( \mathrm{A}/\mathrm{C} \) axes. (e) jerk profile for \( \mathrm{X}/\mathrm{Y}/\mathrm{Z} \) axes. (f) jerk profile for \( \mathrm{A}/\mathrm{C} \) axes.

<!-- Media -->

### 2.5. Summary of the proposed method

In summary of this section, the flowchart of the proposed method is presented in Fig. 5. Since the characteristic lengths of \( {B}_{i}\left( u\right) \) and \( {B}_{i}^{R}\left( u\right) \) can be determined analytically without any iteration, the proposed method can be achieved in real-time.

<!-- Meanless: 50-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

<!-- Media -->

<!-- figureText: 0.1 0.05 0.045 0.04 Tool orientation error (deg) 0.03 0.025 0.02 0.015 0.01 Yan's method 0.005 Proposed method Zhang's method 0 ANTANNANTA 0.5 1 2.5 Time (s) (b) Yan's method 0.09 Proposed method Zhang's method 0.08 0.07 Tool tip error (mm) 0.06 0.05 0.04 0.02 0.01 0 0.5 Time (s) (a) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_15.jpg?x=226&y=152&w=1291&h=563&r=0"/>

Fig. 22. The tool tip and tool orientation errors caused by corner smoothing: (a) tool tip position error. (b) tool orientation error.

<!-- Media -->

## 3. Simulation

In this section, simulations are conducted for both three-axis and five-axis linear tool paths across three distinct examples. During the simulations, the tool paths are smoothed using the proposed method and the local smoothing algorithms from Ref. [16] (hereafter referred to as Yan's method) and Ref. [17] (hereafter referred to as Zhang's method). The reason for choosing these two methods as the comparison methods is that Yan's method is asymmetric, and Zhang's method is symmetric. To fairly compare the effectiveness of smoothing algorithms, the same feedrate scheduling method [37] is utilized to plan the feedrate, and Newton's method [38] is employed for the calculation of interpolation points. Appendix 2 shows that there is only one curvature extreme point on each inserted quintic B-spline spline, facilitating the straightforward application of the feedrate scheduling method [37]. The first example involves the butterfly-shaped three-axis tool path, a typical example for three-axis local smoothing. Yan's method and the proposed method are applied to the tool path. In the second and third examples, the impeller tool path in Ref. [17] and a fan-shaped tool path are examined. To demonstrate the applicability of the proposed smoothing scheme for five-axis tool paths, the proposed method, Yan's method, and Zhang's method are applied to these tool paths. Finally, the smoothed tool paths are interpolated under the same constraints, and a comprehensive comparison is made in terms of machining time, velocity, acceleration, jerk, tool tip error, and tool orientation error to validate the efficacy of the proposed corner smoothing method.

### 3.1. Example 1

In this section, both the proposed smoothing method and Yan's method are applied to smooth the corners in the butterfly-shaped three-axis linear tool path, as depicted in Fig. 6. The maximum approximation error constraint is set to \( {\varepsilon }_{p,\max } = {0.1}\mathrm{\;{mm}} \) . It is evident that the linear toolpaths near the corners have been replaced by the quintic B-spline curves,rendering the tool path \( {G}^{3} \) continuous. To provide a detailed view of the generated corner transition curve, a specific smoothed detail is also provided. The comparison of curvatures between the two smoothed tool path is shown in Fig. 7. In regions with high curvature extremes, such as regions (a) and (c), the curvatures of these two tool-paths are consistent. This is because the same transition curves are adopted in both methods, and the same characteristic lengths are determined by the maximum approximation error constraint. In fact, even if the curvature here are halved, the absolute increment of the maximum allowable velocity would still be quite small due to the high curvature extreme. In regions where the curvature extreme is low, such as regions (b) and (d), the curvatures differ. Compared with Yan's method, the curvature extreme has been decreased, and furthermore, the curvature changes more gently in the proposed method.

Utilizing the jerk-continuous feedrate scheduling method, the fee-drate, acceleration, jerk, and approximation error profiles of both the proposed and Yan's methods are depicted in Fig. 8. These profiles are planned under identical constraints, where the command feedrate, tangential/normal acceleration, tangential/normal jerk, and chord error limits are set to \( F = {200}\mathrm{\;{mm}}/\mathrm{s},{A}_{t} = {A}_{n} = {3000}\mathrm{\;{mm}}/{\mathrm{s}}^{2},{J}_{t} = {J}_{n} = \) \( {60000}\mathrm{\;{mm}}/{\mathrm{s}}^{3} \) ,and \( \delta  = {0.001}\mathrm{\;{mm}} \) . The interpolation period is \( {T}_{s} = 1\mathrm{\;{ms}} \) . Fig. 8 illustrates that the velocity, acceleration, jerk, and chord error are all well within the specified limitations for both methods. Moreover, the total cycle time of the proposed method is \( {5.695}\mathrm{\;s} \) ,while Yan’s method takes \( {6.243}\mathrm{\;s} \) ,resulting in an \( {8.78}\% \) reduction in interpolation time compared to Yan's method. Therefore, the proposed method not only achieves consistent contour performance but also demonstrates higher machining efficiency for three-axis linear tool paths.

### 3.2. Example 2

In this section, an impeller tool path, as shown in Fig. 9, is smoothed by various smoothing methods to demonstrate the effectiveness of the proposed method in five-axis CNC machining. Subsequently, the fee-drate scheduling method is employed to interpolate the rounded paths. The relevant parameters used in this example are listed in Table 2.

In Fig. 10, the proposed method effectively smooths the tool path for each individual drive, as demonstrated by the well-smoothed paths. Fig. 11 compares the original tool path and the smoothed tool paths obtained by the proposed method, Yan's method, and Zhang's method. Upon closer inspection in the enlarged views, it is evident that all three methods successfully smooth both the tool tip and tool orientation paths. Fig. 12 illustrates the comparison of curvatures among three smoothed toolpaths. While the proposed method may not achieve the smallest maximum curvature for every corner, it significantly reduces the overall maximum curvature of the smoothed tool path. The maximum curvature of these three methods are \( {1.0081}{\mathrm{\;{mm}}}^{-1},{1.5822}{\mathrm{\;{mm}}}^{-1} \) ,and \( {1.8499}{\mathrm{\;{mm}}}^{-1} \) ,respectively. Clearly,compared with Yan’s method,the maximum curvature has been reduced by \( {36.28}\% \) and it has been reduced by \( {45.51}\% \) compared with Zhang’s method. Given the relatively small curvatures of the smoothed tool paths (in comparison with Example 1), this reduction in maximum curvature is highly beneficial, facilitating a larger absolute increment in the maximum allowable velocity.

The kinematics and approximation error profiles of these three methods are presented in Figs. 13-15. Fig. 13 displays the tangential velocity, acceleration, and jerk profiles, indicating that all methods adhere to the respective limits within the ranges \( \left\lbrack  {0\mathrm{\;{mm}}/\mathrm{s},{100}\mathrm{\;{mm}}/\mathrm{s}}\right\rbrack  ,\lbrack  - \) \( {1000}\mathrm{\;{mm}}/{\mathrm{s}}^{2},{1000}\mathrm{\;{mm}}/{\mathrm{s}}^{2}\rbrack \) ,and \( \left\lbrack  {-{10000}\mathrm{\;{mm}}/{\mathrm{s}}^{3},{10000}\mathrm{\;{mm}}/{\mathrm{s}}^{3}}\right\rbrack \) . Fig. 14 illustrates the velocity, acceleration, and jerk profiles for each individual drive. Although the axial kinematics are not particularly constrained in the feedrate scheduling method, there are no sudden changes in axial velocity, acceleration, or jerk. This indicates that both the tool tip and tool orientation paths have been sufficiently smoothed. Fig. 15 depicts the actual tool tip position and orientation error profiles, revealing that errors in tool tip position and tool orientation produced by these three methods remain within the user-defined tolerances. The total cycle time of the proposed method, Yan's method, and Zhang's method are 2.188 s, \( {2.307}\mathrm{\;s} \) ,and \( {2.370}\mathrm{\;s} \) ,respectively. This implies a \( {5.16}\% \) reduction in interpolation time compared to Yan’s method,and a 7.68% reduction compared to Zhang's method. In summary, the reduction in interpolation time achieved through the proposed method does not compromise kinematic performance or increase tool tip and tool orientation errors.

<!-- Meanless: 51-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

### 3.3. Example 3

In this section, a fan-shaped tool path with varying tool orientation, as shown in Fig. 16, is provided as an example. The original and smoothed tool paths for each individual drive are visually presented in Fig. 17, showcasing that the smoothed tool path on each individual drive is sufficiently smooth and closely resembles the original tool path. Fig. 18 provides the comparison of the original tool path and the smoothed tool paths obtained by the proposed method, Yan's method, and Zhang's method. Upon closer inspection in the enlarged views, it is evident that all three methods effectively smooth both the tool tip and tool orientation paths. The curvatures comparison of the three smoothed tool paths is illustrated in Fig. 19. The maximum curvature values for these three methods are \( {5.7625}{\mathrm{\;{mm}}}^{-1},{5.7625}{\mathrm{\;{mm}}}^{-1} \) and \( {8.5244}{\mathrm{\;{mm}}}^{-1} \) , respectively. It is noteworthy that despite a 32.39% reduction in maximum curvature compared to Zhang's method, the proposed method and Yan's method yield the same maximum curvature. This similarity arises because the same quintic B-spline curve is applied in these two methods, resulting in the generation of identical smooth toolpaths for certain linear segments, such as region (a). Moreover, in the domain with relatively small maximum curvature, such as region (b), the proposed method exhibits a more uniform curvature distribution, facilitating the maintenance of a high feedrate.

The kinematics and approximation error profiles of these three methods are illustrated in Figs. 20-22. Fig. 20 displays continuous and confined tangential velocity, acceleration, and jerk profiles, all within predetermined maximum values. Fig. 21 presents the velocity, acceleration, and jerk profiles for each individual drive. Due to the significant changes in tool orientation along the fan-shaped tool path and the relatively unconstrained axial dynamics in the feedrate scheduling method, although there are no sudden changes in axial jerk, its maximum value is relatively high, necessitating consideration in future work. Fig. 22 depicts the actual tool tip position and orientation error profiles. The graphs reveal that the errors in the tool tip position and tool orientation by these three methods remain well within the user-defined tolerances. The total cycle time of the proposed method, Yan's method, and Zhang’s method are \( {2.050}\mathrm{\;s},{2.178}\mathrm{\;s} \) ,and \( {2.196}\mathrm{\;s} \) ,respectively. This indicates a 5.88 % reduction in interpolation time compared to Yan's method and a 6.65% reduction compared to Zhang's method. Thus, the reduction in interpolation time achieved through the proposed method does not compromise kinematic performance or increase tool tip and tool orientation errors.

## 4. Conclusions

In conclusion, this paper presents a novel local corner smoothing method with overlap elimination for five-axis linear tool paths. Unlike traditional methods that focus on curvature minimization, the proposed method directly optimizes the local minimum velocity at corners. By minimizing differences in local minimum velocities between adjacent corners, the proposed method ensures a more uniform curvature distribution and a higher feedrate profile on the smoothed tool path, consequently reducing interpolation time. The effectiveness of the proposed method is demonstrated through computer simulations, where it outperforms two traditional local smoothing methods. The key advantages of our method can be summarized as follows:

(1) The corner transition method ensures curvature continuity, synchronization of the tool tip position and tool orientation, and maximal approximation error for the smoothed tool path.

(2) The novel velocity-based optimization scheme effectively eliminates potential overlaps, enhancing machining efficiency by minimizing differences in velocities at adjacent critical points.

(3) The proposed method substantially reduces the curvature extreme of the smoothed tool path, achieving a reduction of 36.28 % and 45.51 % compared to two existing methods. It also creates a more uniform curvature distribution, improving the velocity profile on the smoothed tool path.

(4) Simulation results showcase enhanced processing efficiency, with the proposed method reducing interpolation time by \( {5.16}\% \) to 8.78 % compared to conventional methods.

Considering that axial kinematics are not currently incorporated into the feedrate scheme, their maximum values remain unconstrained. Future work will focus on incorporating axial kinematics constraints into the feedrate scheme to further enhance machining quality. Additionally, our research efforts will be dedicated to developing a one-step smoothing interpolation method tailored specifically for five-axis tool paths, with the goal of streamlining and optimizing machining processes.

## Compliance with ethical standards

The authors declare that they have no conflict of interest that could have direct or potential influence or impart bias on the research reported in this paper. Consent to submit the paper for publication has been received explicitly from all co-authors.

## Declaration of Competing Interest

The authors declare that they have no conflict of interest that could have direct or potential influence or impart bias on the research reported in this paper. Consent to submit the paper for publication has been received explicitly from all co-authors. During the preparation of this work the authors used ChatGPT in order to polish the sentences. After using this tool, the authors reviewed and edited the content as needed and take full responsibility for the content of the publication.

## Acknowledgements

This work has been supported by National Key Research and Development Program of China (Grant No.2020YFA0713700), National Natural Science Foundation of China (Grants No. 12171023, No. 12001028 and No. 62102013) and International Joint Doctoral Education Fund of Beihang University.

<!-- Meanless: 52-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

## Appendix A

To show the overlap elimination method is also applicable to some other B-spline curves, the modified version applied to the B-spline curve in Ref. [17] is given. The B-spline curve is represented by the following equation:

\[B\left( u\right)  = \mathop{\sum }\limits_{{i = 0}}^{6}{N}_{i,p}\left( u\right) {P}_{i} \tag{A.1}\]

where \( p = 4 \) is the degree and the knot vector \( U = \left\lbrack  {0,0,0,0,0,{0.5},{0.5},1,1,1,1,1}\right\rbrack \) ,then the transition curve is determined by seven control points \( P \) \( = \left\lbrack  {{P}_{0},{P}_{1},{P}_{2},{P}_{3},{P}_{4},{P}_{5},{P}_{6}}\right\rbrack \) . As shown in Fig. A.1,the positional relations of the control points are derived as follows:

\[\left\{  \begin{matrix} {P}_{0} = {P}_{b} + 2{l}_{1}{e}_{1} \\  {P}_{1} = {P}_{b} + {1.5}{l}_{1}{e}_{1} \\  {P}_{2} = {P}_{b} + {l}_{1}{e}_{1} \\  {P}_{3} = {P}_{b} \\  {P}_{4} = {P}_{b} + {l}_{2}{e}_{2} \\  {P}_{5} = {P}_{b} + {1.5}{l}_{2}{e}_{2} \\  {P}_{6} = {P}_{b} + 2{l}_{3}{e}_{3} \end{matrix}\right.  \tag{A.2}\]

When \( {l}_{1} = {l}_{2} = l \) ,the maximum approximation error \( \varepsilon  = \frac{\cos \left( {\theta /2}\right) }{2}l \) ,we arrive at \( l \leq  \frac{2{\varepsilon }_{\max }}{\cos \left( {\theta /2}\right) } \) . The curvature extreme \( {k}_{\max } = \frac{3\sqrt{2}\sin \left( \theta \right) }{2{\left( \sqrt{1 - \cos \left( \theta \right) }\right) }^{3}} \) ,so the local minimum feedrate \( v \propto  \sqrt{\frac{{2l}{\left( \sqrt{1 - \cos \left( \theta \right) }\right) }^{3}}{3\sqrt{2}\left| {\sin \left( \theta \right) }\right| }} \) .

<!-- Media -->

<!-- figureText: \( {P}_{a} \) \( {0.5}{l}_{1} \) Transition curve Linear tool path segment \( {P}_{c} \) \( \theta /2 \) \( {0.5}{l}_{1} \) \( {P}_{2} \) \( {l}_{1} \) \( {l}_{2} \) \( {P}_{4} \) \( {P}_{5} \) \( {P}_{6} \) \( {0.5}{l}_{2}\;{0.5}{l}_{2} \) -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_17.jpg?x=549&y=983&w=646&h=562&r=0"/>

Fig. A.1. The B-spline curve in Ref. [17].

<!-- Media -->

In Section 2.2, the optimization model in Eq. (8) is modified as follows:

\[\min \left| {{a}_{i}\sqrt{{l}_{i}} - {a}_{i + 1}\sqrt{{l}_{i + 1}}}\right| \]

\[s.t.\left\{  \begin{matrix} 2 \times  \left( {{l}_{i} + {l}_{i + 1}}\right)  = {L}_{i} \\  \min \left( {\frac{\mathrm{K}{L}_{i}}{2},\frac{2{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i}/2}\right) }}\right)  \leq  {l}_{i} \leq  \frac{2{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i}/2}\right) } \\  \min \left( {\frac{\mathrm{K}{L}_{i}}{2},\frac{2{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i + 1}/2}\right) }}\right)  \leq  {l}_{i + 1} \leq  \frac{2{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i + 1}/2}\right) } \end{matrix}\right. \]

where \( {a}_{i} = \sqrt{\frac{2{\left( \sqrt{1 - \cos \left( {\theta }_{i}\right) }\right) }^{3}}{3\sqrt{2}\left| {\sin \left( {\theta }_{i}\right) }\right| }} \) and \( {a}_{i + 1} = \sqrt{\frac{2{\left( \sqrt{1 - \cos \left( {\theta }_{i + 1}\right) }\right) }^{3}}{3\sqrt{2}\left| {\sin \left( {\theta }_{i + 1}\right) }\right| }} \) ,and the algorithm to solve this optimization model is modified as follows:

<!-- Meanless: 53-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

Table A1

<!-- Media -->

Algorithm A. 1 to solve the optimization model. Algorithm A.1

---

1: \( {l}_{ - } \) generation \( \left( {{\theta }_{i},{\theta }_{i + 1},{L}_{i},K,{\varepsilon }_{p,\max }}\right) \)
2: If \( \frac{4{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i}/2}\right) } \leq  K{L}_{i} \) :
			\( {l}_{i} = \frac{2{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i}/2}\right) },{l}_{i + 1} = \frac{{L}_{i}}{2} - {l}_{i} \)
4: Else if \( \frac{4{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i + 1}/2}\right) } \leq  K{L}_{i} \) :
	\( : \;{l}_{i + 1} = \frac{2{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i + 1}/2}\right) },{l}_{i} = \frac{{L}_{i}}{2} - {l}_{i + 1} \)
6: Else:
				\( {u}_{l} = \max \left( {K,1 - \frac{4{\varepsilon }_{p,\max }}{\cos \left( \frac{{\theta }_{i}}{2}\right) {L}_{i}}}\right) ,{u}_{r} = \min \left( {1 - K,\frac{4{\varepsilon }_{p,\max }}{\cos \left( \frac{{\theta }_{i + 1}}{2}\right) {L}_{i}}}\right) \) and \( u = \frac{{a}_{i}^{2}}{{a}_{i}^{2} + {a}_{i + 1}^{2}} \)
				If \( u < {u}_{l} : u = {u}_{l} \) End
				If \( u > {u}_{r} : u = {u}_{r} \) End
				\( {l}_{i} = {L}_{i}\frac{\left( 1 - u\right) }{2} \) and \( {l}_{i + 1} = {L}_{i}\frac{u}{2} \)
11: \( {End} \)

---

<!-- Media -->

In Section 2.3,the steps to determine the characteristic lengths \( {l}_{1,i} \) and \( {l}_{2,i}\left( {i = 1,2,\ldots ,n - 1}\right) \) are modified as follows:

Step 1: Set \( i = 1 \) ,calculate \( {l}_{\max ,i} = \frac{2{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i}/2}\right) } \) and let \( {l}_{1,i} = \min \left( {{l}_{\max ,i},\frac{{L}_{i - 1}}{2}}\right) \) .

Step 2: If \( i < n - 1 \) ,calculate \( {l}_{\max ,i + 1} = \frac{2{e}_{p,\max }}{\cos \left( {{\theta }_{i + 1}/2}\right) } \) ,let \( {l}_{2,i} = {l}_{\max ,i} \) and \( {l}_{1,i + 1} = {l}_{\max ,i + 1} \) ; otherwise,go to Step 5 .

Step 3: If \( {l}_{2,i} + {l}_{1,i + 1} > \frac{{L}_{i}}{2} \) ,determine \( {l}_{2,i} \) and \( {l}_{1,i + 1} \) by \( l \) generation \( \left( {{\theta }_{i},{\theta }_{i + 1},{L}_{i},K,{\varepsilon }_{p,\max }}\right) \) .

Step 4: Let \( i = i + 1 \) ,go to Step 2.

Step 5: Let \( {l}_{2,i} = \min \left( {{l}_{\max ,i},\frac{{L}_{i}}{2}}\right) \) ,generate \( {B}_{i}\left( u\right) \) by Eq. (3) and terminate the algorithm.

In Section 2.4,the control points of the transition curve \( {B}_{i}^{R}\left( u\right) \) are modified as follows:

\[\left\{  \begin{matrix} {P}_{0,i}^{R} = {R}_{i} + 2{l}_{1,i}^{R}{e}_{1,i} \\  {P}_{1,i}^{R} = {R}_{i} + {1.5}{l}_{1,i}^{R}{e}_{1,i} \\  {P}_{2,i}^{R} = {R}_{i} + {l}_{1,i}^{R}{e}_{1,i} \\  {P}_{3,i}^{R} = {R}_{i} \\  {P}_{4,i}^{R} = {R}_{i} + {l}_{2,i}^{R}{e}_{2,i} \\  {P}_{5,i}^{R} = {R}_{i} + {1.5}{l}_{2,i}^{R}{e}_{2,i} \\  {P}_{6,i}^{R} = {R}_{i} + 2{l}_{3,i}^{R}{e}_{3,i} \end{matrix}\right. \]

<!-- Meanless: (A.4) 54-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

If \( {l}_{1,i}^{R} \geq  \frac{2{\varepsilon }_{r,\max }}{\cos \left( {{\alpha }_{i}/2}\right) },{l}_{1,i}^{R} \) and \( {l}_{1,i} \) are update as follows:

\[\left\{  \begin{matrix} {l}_{1,i}^{R} = \frac{2{\varepsilon }_{r,\max }}{\cos \left( {{\alpha }_{i}/2}\right) } \\  {l}_{1,i} = \frac{{l}_{1,i}^{R}}{{rati}{o}_{i - 1}} \end{matrix}\right.  \tag{A.5}\]

If \( {l}_{2,i}^{R} \geq  \frac{2{\varepsilon }_{r,\max }}{\cos \left( {{\alpha }_{i}/2}\right) },{l}_{2,i}^{R} \) and \( {l}_{2,i} \) are update as follows:

\[\left\{  \begin{array}{r} {l}_{2,i}^{R} = \frac{2{\varepsilon }_{r,\max }}{\cos \left( {{\alpha }_{i}/2}\right) } \\  {l}_{2,i} = \frac{{l}_{2,i}^{R}}{{rati}{o}_{i}} \end{array}\right.  \tag{A.6}\]

Eqs. (13-16) are modified as follows:

\[{l}_{2,i} < \frac{2{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i}/2}\right) }\text{ and }{l}_{2,i}^{R} < \frac{2{\varepsilon }_{r,\max }}{\cos \left( {{\alpha }_{i}/2}\right) } \tag{A.7}\]

\[\begin{cases} {l}_{2,i} &  = \min \left( \frac{2{\varepsilon }_{p,\max }}{\cos \left( \frac{{\theta }_{i}}{2}\right) ,\frac{2{\varepsilon }_{r,\max }}{{rati}{o}_{i}\cos \left( \frac{{\alpha }_{i}}{2}\right) },\frac{{L}_{i}}{2} - {l}_{1,i + 1}}\right) \\  {l}_{2,i}^{R} &  = {rati}{o}_{i}{l}_{2,i} \end{cases} \tag{A.8}\]

\[{l}_{1,i + 1} < \frac{2{\varepsilon }_{p,\max }}{\cos \left( {{\theta }_{i + 1}/2}\right) }\text{ and }{l}_{1,i + 1}^{R} < \frac{2{\varepsilon }_{r,\max }}{\cos \left( {{\alpha }_{i + 1}/2}\right) } \tag{A.9}\]

\[\left\{  \begin{matrix} {l}_{1,i + 1} = \min \left( {\frac{2{\varepsilon }_{p,\max }}{\cos \left( \frac{{\theta }_{i + 1}}{2}\right) },\frac{2{\varepsilon }_{r,\max }}{\operatorname{rati}{o}_{i}\cos \left( \frac{{\alpha }_{i + 1}}{2}\right) },\frac{{L}_{i}}{2} - {l}_{2,i}}\right) \\  {l}_{1,i + 1}^{R} = \operatorname{ratio}{i}_{0}{l}_{1,i + 1} \end{matrix}\right.  \tag{A.10}\]

## Appendix B

As outlined in Eq. (3),quintic B-spline curves are determined by characteristic lengths \( {l}_{1} \) and \( {l}_{2} \) ,the corner point \( {P}_{b} \) and two unit vectors \( {e}_{1} \) and \( {e}_{2} \) . Importantly, curvature is an intrinsic geometric property that remains unaffected by the selection of coordinate systems. Thus, the curvature of the quintic B-spline curve is solely determined by the characteristic lengths \( {l}_{1},{l}_{2} \) and the corner angle \( \theta \) . Without loss of generality,it can be assumed that \( {l}_{2} \geq  {l}_{1} \) . Furthermore,when the corner angle \( \theta \) and the ratio \( {l}_{2}/{l}_{1} \) are fixed,the curvature of each point at the quintic B-spline curve is inversely proportional to \( {l}_{1} \) . Consequently,the value of \( {l}_{1} \) does not affect the curvature trend on the quintic B-spline curve,and without loss of generality, \( {l}_{1} \) is assumed to be \( 1\mathrm{\;{mm}} \) in our discussion.

Moreover,through computer simulations and analysis,we have demonstrated that regardless of variations in \( \theta \) and \( {l}_{2} \) ,there remains only one extreme point of curvature on the quintic B-spline curve. The following Fig. A. 2 illustrates the curvature on the quintic B-spline spline curve when \( \theta \) equals to \( \pi /4,\pi /2 \) and \( {3\pi }/4 \) ,while \( {l}_{2} \) varies from \( {1mm} \) to \( {3mm} \) . It is shown that when \( \theta \) and \( {l}_{2} \) are fixed,there is only one extreme point of curvature on the quintic B-spline curve. Although only partial situations are depicted here, the curvature trend on the quintic B-spline curve is applicable across various \( \theta \) and \( {l}_{2} \) values. References

<!-- Meanless: 55-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

<!-- Media -->

<!-- figureText: Curvature (mm \( {}^{-1} \) ) 2 2.5 1.5 2.5 1.5 0.8 I \( {}_{2} \) (mm) (a) 0.6 0.4 0.2 1.5 0.8 1, (mm) (b) 0.35 0.3 0.25 0.2 0.15 0.1 2.5 2 0.05 1.5 I, (mm) (c) 0.2 0.4 0.6 Curvature \( \left( {\mathrm{{mm}}{}^{-1}}\right) \) 0.8 0.4 0.2 0.2 0.4 0.6 U 0.4 Curvature (mm \( {}^{-1} \) ) 0.3 0.2 0.1 0.2 0.6 0.8 U -->

<img src="https://cdn.noedgeai.com/bo_d2sl91bef24c73b2kea0_20.jpg?x=546&y=151&w=670&h=1779&r=0"/>

Fig. A2,the curvature of the quintic B-spline spline with various characteristic length \( {l}_{2} \) and the corner angle. (a) \( \theta  = \pi /4 \) . (b) \( \theta  = \pi /2 \) . (c) \( \theta  = {3\pi }/4 \) .

<!-- Media -->

<!-- Meanless: 56-->




<!-- Meanless: Y. Hu et al. CIRP Journal of Manufacturing Science and Technology 52 (2024) 36-57-->

[1] Yin X, Guan J, Chen M, Sun Y. An analytical corner rounding algorithm with G01 shape preserving for five-axis computer numerical controlled machining. J Manuf Sci Eng 2022;144:081013. https://doi.org/10.1115/1.4053923.

[2] Zhong W, Luo X, Chang W, Cai Y, Ding F, Liu H, Sun Y. Toolpath interpolation and smoothing for computer numerical control machining of freeform surfaces: a review. Int J Autom Comput 2021;17(1):1-16. https://doi.org/10.1007/s11633- 019-1190-y.

[3] Wang W, Hu C, Zhou K, He S, Zhu L. Local asymmetrical corner trajectory smoothing with bidirectional planning and adjusting algorithm for CNC machining. Robot Comput Integr Manuf 2021;68:102058. https://doi.org/10.1016/j.rcim.2020.102058.

[4] Hashemian A, Bo P, Bartoñ M. Reparameterization of ruled surfaces: toward generating smooth jerk-minimized toolpaths for multi-axis flank CNC milling. Comput-Aided Des 2020;127:102868. https://doi.org/10.1016/j.cad.2020.102868.

[5] Kornmaneesang W, Chen S. Time-optimal feedrate scheduling with actuator constraints for 5-axis machining. Int J Adv Manuf Technol 2022;119:6789-807. https://doi.org/10.1007/s00170-021-08033-y.

[6] Bouchenitfa H, Julien CJ, Linares JM, Sprauel JM, Azzam N, Boukebbab S. Improvement of toolpath quality combining polynomial interpolation with reduction of toolpath points. Int J Adv Manuf Technol 2015;78(5-8):875-83. https://doi.org/10.1007/s00170-014-6696-4.

[7] Zhang J, Zhang LQ, Zhang K, Mao J. Double NURBS trajectory generation and synchronous interpolation for five-axis machining based on dual quaternion algorithm. Int J Adv Manuf Technol 2016;83:2015-25. https://doi.org/10.1007/ s00170-015-7723-9.

[8] He S, Ou D, Yan C, Lee C. A chord error conforming tool path B-spline fitting method for NC machining based on energy minimization and LSPIA. J Comput Des Eng 2015;2(4):218-32. https://doi.org/10.1016/j.jcde.2015.06.002.

[9] Deng C, Lin H. Progressive and iterative approximation for least squares B-spline curve and surface fitting. Comput-Aided Des 2015;47:32-44. https://doi.org/ 10.1016/j.cad.2013.08.012.

[10] Yang Z, Shen L, Yuan C, Gao X. Curve fitting and optimal interpolation for CNC machining under confined error using quadratic B-splines. Comput-Aided Des 2015;66:62-72. https://doi.org/10.1016/j.cad.2015.04.010.

[11] Lin F, Shen L, Yuan C, Mi Z. Certified space curve fitting and trajectory planning for CNC machining with cubic B-splines. Comput-Aided Des 2019;106:13-29. https:// doi.org/10.1016/j.cad.2018.08.001.

[12] Beudaert X, Pechard PY, Tournier C. 5-Axis tool path smoothing based on drive constraints. Int J Mach Tools Manuf 2011;51(12):958-65. https://doi.org/ 10.1016/j.ijmachtools.2011.08.014.

[13] Tulsyan S, Altintas Y. Local toolpath smoothing for five-axis machine tools. Int J Mach Tools Manuf 2015;96:15-26. https://doi.org/10.1016/j.ijmachtools.2015.04.014.

[14] Bi Q, Shi J, Wang Y, Zhu L, Ding H. Analytical curvature-continuous dual-Bézier corner transition for five-axis linear tool path. Int J Mach Tools Manuf 2015;91: 96-108. https://doi.org/10.1016/j.ijmachtools.2015.02.002.

[15] Sun S, Altintas Y. A G3 continuous tool path smoothing method for 5-axis CNC machining. CIRP J Manuf Sci Technol 2021;32:529-49. https://doi.org/10.1016/j.cirpj.2020.11.002.

[16] Yan G, Zhang Y, Li C, Xu J. Asymmetrical transition-based corner rounding method driven by overlap elimination for CNC machining of short-segmented tool path. J Manuf Process 2022;76:624-37. https://doi.org/10.1016/j.jmapro.2022.02.022.

[17] Zhang Y, Wang T, Dong J, Peng P, Liu Y, Ke R. An analytical G3 continuous corner smoothing method with adaptive constraints adjustments for five-axis machine tool. Int J Adv Manuf Technol 2020;109:1007-26. https://doi.org/10.1007/ s00170-020-05402-x.

[18] Huang J, Du X, Zhu L. Real-time local smoothing for five-axis linear toolpath considering smoothing error constraints. Int J Mach Tools Manuf 2018;124:67-79. https://doi.org/10.1016/j.ijmachtools.2017.10.001.

[19] Peng J, Huang P, Ding Y, Ding H. An analytical method for decoupled local smoothing of linear paths in industrial robots. Robot Comput-Integr Manuf 2021; 72:102193. https://doi.org/10.1016/j.rcim.2021.102193.

[20] Liu H, Li G, Xiao J. A C3 continuous toolpath corner smoothing method for a hybrid machining robot. J Manuf Process 2022;75:1072-88. https://doi.org/ 10.1016/j.jmapro.2021.12.057.

[21] Li G, Liu H, Liu S, Xiao J. A general C 2 continuous toolpath corner smoothing method for a 5-DOF hybrid robot. Mech Mach Theory 2022;169:104640. https:// doi.org/10.1016/j.mechmachtheory.2021.104640.

[22] Xu F, Sun Y. A circumscribed corner rounding method based on double cubic B-splines for a five-axis linear tool path. Int J Adv Manuf Technol 2018;94:451-62. https://doi.org/10.1007/s00170-017-0869-x.

[23] Xiao Q, Wan M, Qin X. Real-time smoothing of G01 commands for five-axis machining by constructing an entire spline with the bounded smoothing error. Mech Mach Theory 2021;161:1-30. https://doi.org/10.1016/j.mechmachtheory.2021.104307.

[24] Yang J, Yuen A. An analytical local corner smoothing algorithm for five-axis CNC machining. Int J Mach Tools Manuf 2017;123:22-35. https://doi.org/10.1016/j.ijmachtools.2017.07.007.

[25] Yang J, Li D, Ye C, Ding H. An analytical C3 continuous tool path corner smoothing algorithm for 6R robot manipulator. Robot Comput-Integr Manuf 2020;64:101947. https://doi.org/10.1016/j.rcim.2020.101947.

[26] Hu Q, Chen Y, Jin X, Yang J. A real-time C3 continuous tool path smoothing and interpolation algorithm for five-axis machine tools. J Manuf Sci Eng 2020;142(4): 041002. https://doi.org/10.1115/1.4046091.

[27] Shi J, Bi Q, Zhu L, Wang Y. Corner rounding of linear five-axis tool path by dual PH curves blending. Int J Mach Tools Manuf 2015;88:223-36. https://doi.org/ 10.1016/j.ijmachtools.2014.09.007.

[28] Wan M, Qin X, Xiao Q, Liu Y, Zhang W. Asymmetrical pythagorean-hodograph (PH) spline-based C3 continuous corner smoothing algorithm for five-axis tool paths with short segments. J Manuf Process 2021;64(3):1387-411. https://doi.org/10.1016/j.jmapro.2021.02.059.

[29] Jiang X, Hu Y, Huo G, Su C, Wang B, Li H, Shen L, Zheng Z. Asymmetrical Pythagorean-hodograph spline-based C4 continuous local corner smoothing method with jerk-continuous feedrate scheduling along linear toolpath. The International. J Adv Manuf Technol 2022;volume 121:5731-54. https://doi.org/ 10.1016/j.jmapro.2021.02.059.

[30] Yang J, Adili A, Ding H. Real time tool path smoothing of short linear commands for robot manipulator by constructing asymmetrical Pythagoran-hodograph (PH) splines. Sci China Technol Sci 2023:1-15. https://doi.org/10.1007/s11431-022- 2280-1.

[31] Huang X, Zhao F, Tao T, Mei X. A newly developed corner smoothing methodology based on clothoid splines for high speed machine tools. Robot Comput-Integr Manuf 2021;70:102106. https://doi.org/10.1016/j.rcim.2020.102106.

[32] Huang X, Zhao F, Tao T, Mei X. A novel local smoothing method for five-axis machining with time-synchronization feedrate scheduling. IEEE Access 2020;8: 89185-204. https://doi.org/10.1109/ACCESS.2020.2992022.

[33] Zhao H, Zhu L, Ding H. A real-time look-ahead interpolation methodology with curvature-continuous B-spline transition scheme for CNC machining of short line segments. Int J Mach Tools Manuf 2013;65:88-98. https://doi.org/10.1016/j.ijmachtools.2012.10.005.

[34] Fan W, Lee C, Chen J. A realtime curvature-smooth interpolation scheme and motion planning for CNC machining of short line segments. Int J Mach Tools Manuf 2015;96:27-46. https://doi.org/10.1016/j.ijmachtools.2015.04.009.

[35] Zhao H, Zhang YH, Ding H. A corner rounding and trajectory generation algorithm for five-axis linear toolpath. J Mech Eng 2018;54(3):108-16. https://doi.org/ 10.3901/JME.2018.03.108.

[36] Lee A, Lin M, Pan Y. The feedrate scheduling of NURBS interpolator for CNC machine tools. Comput Aided Des 2011;43(6):612-28. https://doi.org/10.1016/j.cad.2011.02.014.

[37] Hu Y, Jiang X, Huo G, Su C, Wang B, Li H, Zheng Z. A novel S-shape based NURBS interpolation with acc-jerk-Continuity and round-off error elimination. J Comput Des Eng 2023;10:294-317. https://doi.org/10.1093/jcde/qwad004.

[38] Zhao H, Zhu L, Ding H. A parametric interpolator with minimal feed fluctuation for CNC machine tools using arc-length compensation and feedback correction. Int J Mach Tools Manuf 2013;75:1-8. https://doi.org/10.1016/j.ijmachtools.2013.08.002.

<!-- Meanless: 57-->

