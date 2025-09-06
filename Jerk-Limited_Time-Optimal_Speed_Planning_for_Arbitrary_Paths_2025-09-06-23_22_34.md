

<!-- Meanless: 8194 IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, VOL. 23, NO. 7, JULY 2022-->

# Jerk-Limited Time-Optimal Speed Planning for Arbitrary Paths

Antonio Artuñedo \( {}^{\square } \) ,Jorge Villagra \( {}^{\square } \) ,and Jorge Godoy \( {}^{\square } \)

Abstract-Both path and speed planning play key roles in automated driving, specially when a certain comfort level is required. This paper proposes a human-like speed planning method for already defined paths while minimizing travel time. The proposed method is able to compute a time-optimal speed profile that meet the given constrains with regard to speed, acceleration and jerk. For this purpose, an initial acceleration-limited approach is introduced. This algorithm serves as a starting point for the subsequent jerk-limited speed planning. Moreover, fallback strategies are included to manage critical driving situations where initial or final conditions cannot be met. The proposed approach has been tested and validated in an experimental platform through extensive trials in real environments. Its performance has been evaluated both in terms of quality of the computed speed profiles and with respect the required computing time.

Index Terms-Autonomous vehicles, motion planning, jerk-limited speed planning, comfortable automated driving.

## I. INTRODUCTION

IN RECENT decades, research interest in autonomous driving has increased considerably and significant progress has been made in the technologies involved in this area. In addition to ensuring safe driving, it is expected that the driving of autonomous vehicles will be similar to that of human drivers, i.e. smooth and comfortable for the vehicle occupants. Although enormous research efforts have been made on motion planning techniques to enable autonomous vehicles to reach a given destination safely, the smooth behavior of automated vehicles remains a challenge, especially in medium and high speed environments.

State-of-the-art decision-making architectures for automated vehicles are typically structured into global route planning, behavioural/manoeuvre planning and local motion planning. Among these functionalities, the latter is of utmost importance as it has to generate safe, feasible and comfortable trajectories to be tracked by the vehicle control system [24].

Automated driving requires parameterizable planning methods that enable the adaptability of the vehicle speed profile based on the driving scene understanding and predictions of the nearby environment. Although safety is the most important aspect when designing planning algorithms, comfort should also be considered. In this regard, it is important to note that the main factors contributing to uncomfortable driving are high levels of jerk and acceleration [7], [31], although jerk has a stronger influence than acceleration [8]. Moreover, jerk-limited trajectories increase the similarity between automated and human-driven driving. Although several algorithms for obtaining jerk-limited profiles can be found in the literature, most of them are either limited to unmanned aerial vehicles or to low speed ground vehicles, or apply complex and computationally expensive optimization processes to obtain near-optimum speed profiles, or can only be applied to path defined through specific geometric primitives. Moreover, one of the limitations of the approaches found in the literature is the inability to handle contexts where the initial and/or final required state of the vehicle cannot be met considering the given constraints on speed, acceleration and jerk. Therefore, the variability of the application contexts is reduced to rather conservative cases.

The growing need of comfortable, reactive and efficient motion planning methods for automated driving applications motivates this work. To cope with this challenging problem, a jerk-limited speed planning method for already determined paths is introduced. The proposed algorithm is able to compute time-optimal speed profiles that meet constraints on speed, accelerations and jerk along the trajectory. Moreover, fallback strategies to address the speed planning problem when initial and/or final speed and acceleration constraints cannot be satisfied, are also proposed.

The main contributions of this work are threefold:

- The proposed time-optimal speed planning algorithm is path primitive agnostic while considering kinodynamic constraints of the vehicle.

- Neither time-displacement interpolations nor computationally expensive optimization algorithms are required, resulting in a computational efficient solution.

- The proposed approach introduces mechanisms to handle design constraints when initial and final conditions cannot be met, as shown in extensive experimental results.

The paper is structured as follows: In section II similar state-of-the-art approaches are reviewed. In section III the interplay between path geometry and speed profile constraints is introduced. Section IV describes the acceleration-limited approach while section \( \mathrm{V} \) focuses on the jerk-limited speed profile generation algorithm. In section VI, the results of the trials carried out in a real environment are presented and analyzed. Finally, section VII draws some concluding remarks and future works.

---

<!-- Footnote -->

Manuscript received 26 October 2020; revised 16 March 2021; accepted 27 April 2021. Date of publication 12 May 2021; date of current version 8 July 2022. This work was supported in part by the Spanish Ministry of Science and Innovation with National Projects Programmable Systems for Intelligence in Automobiles (PRYSTINE) and Integrated, Fail-Operational, Cognitive Perception, Planning and Control Systems for Highly Automated Vehicles (NEWCONTROL) under Grant PCI2018-092928 and Grant PCI2019-103791, in part by the Community of Madrid through Seguridad de vehículos para una movilidad inteligente, sostenible, segura e integradora (SEGVAUTO) 4.0-CM Programme under Grant S2018-EMT-4362, and in part by the European Commission and Electronic Components and Systems for European Leadership (ECSEL) Joint Undertaking through the Project PRYSTINE under Grant 783190 and the Project NEWCONTROL under Grant 826653. The Associate Editor for this article was L. Li. (Corresponding author: Antonio Artuñedo.)

The authors are with the Centre for Automation and Robotics (CSIC-UPM), 28500 Madrid, Spain (e-mail: antonio.artunedo@csic.es; jorge.villagra@csic.es; jorge.godoy@csic.es).

Digital Object Identifier 10.1109/TITS.2021.3076813

<!-- Footnote -->

---

<!-- Meanless: 1558-0016 (C) 2021 IEEE. Personal use is permitted, but republication/redistribution requires IEEE permission. See https://www.ieee.org/publications/rights/index.html for more information. Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: ARTUÑEDO et al.: JERK-LIMITED TIME-OPTIMAL SPEED PLANNING FOR ARBITRARY PATHS 8195-->

## II. RELATED WORK

Motion planning problem has been one of the main focus of the robotics research in last decades. Although some existing approaches jointly address both path and speed planning [37], the complexity in obtaining safe and feasible trajectories makes this problem often divided into two different procedures: (i) path planning in order to compute collision free paths and feasible in terms of kinematic constraints, and (ii) speed planning for computing speed profiles for a given path that satisfies dynamic feasibility and comfort requirements, taking into account vehicle constraints.

Rather than speed planning, most of the attention in motion planning research has focused on different aspects of path planning such as obtaining optimal paths with regard to path length or curvature [2], [10], [24]. Nevertheless, speed planning approaches for on-road autonomous driving are recently gaining considerable attention.

Most of the speed planning approaches that can be found in the automated driving literature deal with speed and acceleration constraints. In [21], an algorithm to find an achievable acceleration-limited speed profile considering the dynamics of the vehicle is applied to obtain the minimum time traversal of a given path, using optimization techniques. The authors obtained speed profiles each \( {10}\mathrm{\;{ms}} \) on paths defined through 50 points. A similar acceleration-limited speed planning approach is also used in [36], where a strategy to find a minimum time speed profile traveling along a fixed path subject to the vehicle dynamics constraints, slip circle constraints, and actuator limits is proposed. In [19], authors use quintic Bézier curves to generate speed profiles considering bounding speed and acceleration.

Unlike acceleration-limited speed planning approaches, fewer methods are found in the state of the art for automated driving proposing methods to obtain jerk-limited speed profiles. The approach proposed in [35] is able to compute jerk-limited speed profiles given piecewise paths composed of clothoids, arcs of circles and straight lines. S-curve equations [22] are applied to each section of the path to achieve speed and acceleration continuity. In [25], the jerk-limited speed planning problem is stated as a non-linear and non-convex one. To efficiently solve it, authors propose to use linear approximations at different stages that can be easily solved. Speed planning with jerk constraints is also formulated as a non-linear optimization problem in [5], which is solved using the interior point optimizer with the goal of minimizing the travel time. In [33], a set of speed profiles is generated using 3rd order polynomial spline in the time domain. Then, a selection procedure discards those speed profiles that does not meet the speed and longitudinal acceleration limits. A recent work [27] proposes an heuristic approach for jerk-limited speed planning in laser guided vehicles for warehouse environments. This algorithm needs to be run recursively to reach a near time-optimal solution and it is limited to 9 constant-jerk intervals in a trajectory. In addition, some jerk-limited speed planning methods can be found in the literature for different robotic applications such as manipulators [15], [22], [28], or unmanned aerial vehicles [18]. As can be seen, most of the reviewed jerk-limited speed planning approaches make use of costly optimization processes whose convergence to a global minimum is not guaranteed when the computation time is limited. This makes them difficult to apply in applications where the computation time of a speed profile is critical.

Instead of generating trajectories for subsequent tracking, other existing approaches rely on model-predictive control (MPC) techniques. These approaches jointly solve the problem of planning and control by generating a set of future control actions that satisfy a set of constraints [4], [9], [23], [32] with regard to safety and comfort. The computational complexity of MPC-based approaches is highly dependent on the model used, optimization constraints, the definition of the optimization problem, but even more on the planning horizon. In this context, a balance among these factors must be found to achieve acceptable performance in terms of closed-loop behavior, stability, robustness and computation time [34]. In some cases, the planning horizon is reduced (e.g. 2 seconds in [14], 5 seconds in [16]), thus reducing the anticipation capability of the trajectory. To address these limitations, other approaches [3], [12], [20], [33], [36], based on long-term motion planning, are often used to provide larger trajectories that increase vehicle anticipation and thus prevent sudden events from abruptly changing the trajectory.

As discussed above, the reviewed speed planning approaches for autonomous driving that limit speed, accelerations and jerk, do so (i) by applying speed planning over paths generated by specific geometric primitives, (ii) obtain non-time-optimal solutions or (iii) often apply computationally expensive methods. Besides, the anticipation capability of MPC-based approaches is often reduced to cope with jerk-limited speed profiles generation for predefined paths in a real-time setting.

## III. Path Geometry and Speed Profile Constraints

In contrast to many other approaches, that explicitly take advantage of path primitives to generate (quasi-)optimal speed profiles, this work allows an arbitrary path as input. However, kinematic and dynamic vehicle constraints needs to be imposed to that path, so that a suitable speed profile can be proposed. The work from [17] evidences that, although in principle a bicycle dynamic model is supposed to better reproduce a vehicle behaviour, when it is compared with a discretised kinematic bicycle model over short periods of time, their forecast errors are similar under average driving conditions (urban-like scenarios). As a result, a kinematic bicycle model is used here to justify that a path is trackable if and only if the control actions (speed and steering angle) are continuous and explicitly bounded with the maximum lateral acceleration, the longitudinal acceleration and the longitudinal jerk.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 8196 IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, VOL. 23, NO. 7, JULY 2022-->

The considered model has a body frame attached to a reference point with coordinates \( \xi  = {\left( x,y\right) }^{T} \) in a global coordinate frame,and orientation \( \theta \) with respect to the global frame. The vehicle is assumed to have planar motion, moves with velocity \( v \) and is equipped with a front-centred steerable wheel (whose angle is represented by \( \delta \) ) and fixed parallel rear wheels. The velocity vector at the centre of gravity has an orientation \( \beta \) ,called slip angle,with respect to the longitudinal axis of the vehicle. To account for the physical constraints on the steering angle rate \( \eta \) ,the longitudinal acceleration \( {\gamma }_{x} \) and the longitudinal jerk \( j \) ,three additional states have been added to the pose variables define the generalised coordinates of the system \( \left( {x,y,\theta ,\delta ,v,{\gamma }_{x}}\right) \) :

\[\left\lbrack  \begin{matrix} \dot{x} \\  \dot{y} \\  \dot{\theta } \\  \dot{\delta } \\  \dot{v} \\  {\dot{y}}_{x} \end{matrix}\right\rbrack   = \left\lbrack  \begin{matrix} v\cos \left( {\theta  + \beta }\right) \\  v\sin \left( {\theta  + \beta }\right) \\  \frac{v}{{L}_{r}}\sin \beta \\  0 \\  0 \\  0 \end{matrix}\right\rbrack   + \left\lbrack  \begin{array}{l} 0 \\  0 \\  0 \\  1 \\  0 \\  0 \end{array}\right\rbrack  \eta  + \left\lbrack  \begin{array}{l} 0 \\  0 \\  0 \\  0 \\  1 \\  0 \end{array}\right\rbrack  \]

\[ + \left\lbrack  \begin{array}{l} 0 \\  0 \\  0 \\  0 \\  0 \\  1 \end{array}\right\rbrack  j,\;\beta  = \arctan \left( {\frac{{L}_{r}}{{L}_{r} + {L}_{f}}\tan \delta }\right)  \tag{1}\]

where \( {L}_{f} \) and \( {L}_{r} \) represent the distance from the center of the mass of the vehicle to the front and rear axles, respectively. Note that the steering angle, steering angle speed, longitudinal velocity, acceleration and jerk are subject to the constraints \( \left| \delta \right|  \leq  {\delta }_{\max },\left| \eta \right|  \leq  {\eta }_{\max },v \leq  {v}_{\max },{\gamma }_{x} \in  \left\lbrack  {{\gamma }_{{x}_{\min }},{\gamma }_{{x}_{\min }}}\right\rbrack  ,j \in \) \( \left\lbrack  {{j}_{min},{j}_{min}}\right\rbrack \) ,respectively.

Definition 1: A parametric curve \( \mathbf{p}\left( u\right) \) has first order geometric continuity and is a \( {G}^{1} \) if it is regular and its unit tangent vector is a continuous function along the curve. The curve \( \mathbf{p}\left( u\right) \) has second order geometric continuity \( \left( {G}^{2}\right) \) if it is a \( {G}^{1} \) -curve and its curvature vector is continuous along the curve.

Definition 2: The arc length \( {s}_{r} \) of a planar curve \( \{ x,y\} \) can be expressed as

\[f : \left\lbrack  {{u}_{0},{u}_{1}}\right\rbrack   \rightarrow  \left\lbrack  {0,f\left( u\right) }\right\rbrack  \]

\[u \mapsto  {s}_{r} = {\int }_{{u}_{0}}^{u}\sqrt{\dot{x}{\left( \xi \right) }^{2} + \dot{y}{\left( \xi \right) }^{2}}{d\xi }\]

Proposition 1: A path \( \Gamma \) in the \( \{ x,y\} \) plane is generated by model (1) with continuous inputs:

- \( \delta \left( \cdot \right)  \in  {\mathcal{C}}^{1},\left| \delta \right|  \leq  {\delta }_{\max },\eta  \leq  {\eta }_{\max },\forall t \geq  0 \)

- \( v\left( \cdot \right)  \in  {\mathcal{C}}^{1},v \in  \left\lbrack  {0,{v}_{\text{max }}}\right\rbrack  ,\dot{v} \in  \left\lbrack  {{\gamma }_{{x}_{\text{min }}},{\gamma }_{{x}_{\text{min }}}}\right\rbrack  ,\ddot{v} \in \) \( \left\lbrack  {{J}_{{x}_{min}},{J}_{{x}_{min}}}\right\rbrack  ,\forall t \geq  0 \)

if and only if \( \Gamma \) is a \( {G}^{2} \) path and its curvature \( \kappa  \in  {\mathcal{C}}^{1} \) , \( \kappa  \in  \left\lbrack  {0,{\kappa }_{\max }}\right\rbrack \) and curvature derivative \( \sigma  \in  \left\lbrack  {-{\sigma }_{\max },{\sigma }_{\max }}\right\rbrack \) are bounded.

The proof can be sketched as follows. Given any \( {C}^{2} \) -curve \( \mathbf{p}\left( u\right) \) with \( u \in  \left\lbrack  {{u}_{0},{u}_{1}}\right\rbrack \) ,the inverse arc length function \( {s}_{r}^{-1} \) is defined, and is, by definition, a continuous function. Moreover, the scalar curvature \( \kappa \left( u\right) \) is also continuous over \( \left\lbrack  {{u}_{0},{u}_{1}}\right\rbrack \) because \( \mathbf{p}\left( u\right) \) is a \( {C}^{2} \) -curve. Consider the curvature bound to be such that the steering angle is \( \left| \delta \right|  \leq  {\delta }_{\max } \) and the curvature derivative bound to be such that the steering angle derivative is \( \left| \eta \right|  \leq  {\eta }_{\max } \) .

If the side-slip angle \( \beta \) is considered small, \( \tan \delta  = \) \( \left( {{L}_{r} + {Lf}}\right) \kappa \left( {s}_{r}\right) \) and therefore \( {\kappa }_{\max } = \frac{\tan {\delta }_{\max }}{{L}_{r} + {L}_{f}} \) . Likewise, the curvature derivative bound \( {\sigma }_{\max } \) can be expressed in terms of the speed, the steering angle and its derivative:

\[\frac{d\kappa }{dt} = \frac{d\kappa }{d{s}_{r}}\frac{d{s}_{r}}{dt} \Rightarrow  {\sigma }_{\max } = {\left. \frac{d\kappa }{d{s}_{r}}\right| }_{\max } = {\left. \frac{\dot{\delta }}{{\dot{s}}_{r}L{\cos }^{2}\delta }\right| }_{\max } \tag{2}\]

being \( {\dot{s}}_{r} = v > 0 \) . Since the speed is \( {\mathcal{C}}^{1} \) ,with bounds in its first and second derivatives ( \( {\gamma }_{{x}_{\max }} \) and \( {j}_{\max } \) ,respectively), it allows a continuous mapping in (2), which, in addition can be expressed as a function of the longitudinal and lateral accelerations by integrating \( v \) in the following equation:

\[\dot{v} = \dot{\theta }{v\beta } + {\gamma }_{x},\beta  = \arctan \left( {\frac{{L}_{r}}{{L}_{r} + {L}_{f}}\tan \delta }\right)  \tag{3}\]

\[{\dot{v}}_{y} =  - \dot{\theta }v + {\gamma }_{y}\]

Note that introducing \( \dot{\theta } = {v\kappa } \) and considering a small vehicle side-slip \( \left( {\kappa  = \tan \delta /L}\right) \) ,eq. (3) yields \( \dot{v} = {g}_{1}\left( {v,\delta ,{\gamma }_{x}}\right) \) , \( {\dot{v}}_{y} = {g}_{2}\left( {v,\delta ,{\gamma }_{y}}\right) \) ,and therefore the maximum speed \( {v}_{\max } \) is a function of design constraints \( {\gamma }_{{x}_{\max }},{\gamma }_{{y}_{\max }} \) and \( {\delta }_{\max } \) . Remark also that under neglecting slip conditions, eq. (3) becomes \( {\gamma }_{y} = {v}^{2}\kappa \) and the maximum speed \( {v}_{\max } \) can be expressed in terms of \( {\gamma }_{{y}_{\max }} \) (assumption used in (7)). As a result, a direct link can be established between the maximum longitudinal and lateral acceleration constraints and the maximum curvature derivative.

At the initial time \( {t}_{0} \) ,consider the state of model (1) given by \( {\left\lbrack  \begin{array}{llllll} x\left( {u}_{0}\right) & y\left( {u}_{0}\right) & \theta \left( {u}_{0}\right) & \delta \left( {u}_{0}\right) & v\left( {u}_{0}\right) & {\gamma }_{x}\left( {u}_{0}\right)  \end{array}\right\rbrack  }^{T} \) . Then,applying the continuous input

\[\delta \left( t\right)  = \arctan \left( \frac{\left( {L}_{r} + {L}_{f}\kappa \left( {s}_{r}^{-1}\left( v\left( t - {t}_{0}\right) \right) \right) \right) }{{\left( 1 - \left( {L}_{r}\kappa \left( {s}_{r}^{-1}\left( v\left( t - {t}_{0}\right) \right) \right) \right) \right) }^{\left( 1/2\right) }}\right)  \tag{4}\]

the vehicle’s motion from \( {t}_{0} \) to \( {t}_{0} + \frac{s\left( {u}_{1}\right) }{n} \) exactly matches the path of the given curve if \( v\left( t\right)  \leq  {v}_{\max },\left| {\dot{v}\left( t\right) }\right|  \leq  {\gamma }_{{x}_{\max }} \) and \( \ddot{v}\left( t\right)  \in  \left\lbrack  {{J}_{{x}_{\min }},{J}_{{x}_{\min }}}\right\rbrack \) . Indeed,let consider a steering angle \( \delta \) such that the side-slip of system (1) leads to \( \dot{\theta } = {v\kappa } \) :

\[\beta  = \arctan \left( {\frac{{L}_{r}}{{L}_{f} + {L}_{r}}\tan \delta }\right)  = \arcsin \left( {{L}_{r}\kappa }\right) ) \tag{5}\]

Eq. (5) is then solved for \( \delta \) ,yielding:

\[\delta  = \arctan \left( {\frac{{L}_{f} + {L}_{r}}{{L}_{r}}\tan \left( {\arcsin \left( {{L}_{r}\kappa }\right) }\right) }\right) \]

\[ = \arctan \left( \frac{\left( {{L}_{r} + {L}_{f}}\right) \kappa }{{\left( 1 - {\left( {L}_{r}\kappa \right) }^{2}\right) }^{\left( 1/2\right) }}\right) \]

which is a continuous function in the operational range \( \left| \kappa \right|  \leq  {\kappa }_{\max } \) if \( \left| \kappa \right|  < 1/{L}_{r} \) . This condition,when small side-slips are considered,is equivalent to \( \left| \delta \right|  \leq  1 + {L}_{f}/{L}_{r} \) , which always holds in a passenger vehicle,where \( {\delta }_{\max } < 1 \) . As a result,since the scalar curvature \( \kappa \) is continuous over \( \left\lbrack  {{u}_{0},{u}_{1}}\right\rbrack \) because \( p\left( u\right) \) is a \( {G}^{2} \) curve,(4) is also continuous and therefore Proposition 1 can be proved.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: ARTUÑEDO et al.: JERK-LIMITED TIME-OPTIMAL SPEED PLANNING FOR ARBITRARY PATHS 8197-->

## IV. ACCELERATION-LIMITED TIME-OPTIMAL SPEED PLANNING

Before calculating a jerk-limited speed profile for a given path, the proposed approach in this work computes an acceleration-limited speed for each point of the trajectory. This auxiliary speed profile will be used both by the jerk-limited speed planning algorithm and to perform anticipatory reachability estimation, as explained in section V-C. Indeed, in addition to the speed planning approach proposed in [1], this algorithm includes a fallback strategy to avoid speed profile discontinuities when initial and final speeds cannot be met with specified acceleration constraints.

Based on Pontryagin's Maximum Principle, time-optimal trajectories are assumed to be bang-bang in this work (see e.g. [26] for a similar consideration). As a result, the time-optimal speed profile for a predefined path follows successively maximum and minimum acceleration profiles while considering the given speed and acceleration constraints.

The acceleration-limited speed planning strategy comprises the computation of a speed limit curve and a subsequent algorithm for acceleration limiting. Both steps are detailed in the following subsections.

## A. Speed Limit Curve

The speed limit curve sets a maximum velocity value at each point of the path taking into account different speed and acceleration constraints that can come from regulatory limits (maximum speed), comfort constraints (maximum comfort speed, acceleration and jerk) and dynamic constraints (available engine torque, sliding, ground contact and tip-over constraints) [30].

In this work, the speed limit curve has been used to consider the maximum speed and maximum lateral acceleration. However, the remaining constraints mentioned above can be also used to impose further limits to be taken into account in the proposed speed planning algorithm.

## B. Acceleration-Limited Speed Planning Algorithm

The speed profile is calculated over a given path \( P\left( {x,y,\kappa }\right) \) , composed of a coordinate list of \( n \) two-dimensional points and curvature. This speed profile must comply with the constraints listed in table I, i.e. limit both longitudinal and lateral accelerations and linear speed, as well as impose initial and final speed. In other words, the goal of this step is to find a mapping of a linear speed \( {v}_{i} \) and a longitudinal acceleration \( {a}_{i} \) for each point \( i \) of \( P \) ,taking into account the following constraints:

\[0 < {v}_{i} < {v}_{\max }\]

\[{a}_{\text{min }}^{\text{long }} < {a}_{i} < {a}_{\text{max }}^{\text{long }}\]

\[\left| {a}_{i}^{\text{lat }}\right|  < {a}_{\max }^{\text{lat }}\]

\[{v}_{0} = {v}_{\text{ini }}\]

\[{v}_{n} = {v}_{\text{end }} \tag{6}\]

Note that in the previous section the longitudinal and lateral accelerations were expresses as \( {\gamma }_{x} \) and \( {\gamma }_{y} \) ,respectively,for compactness. Henceforth,they are notated as \( {a}^{\text{long }} \) and \( {a}^{\text{lat }} \) .

<!-- Media -->

TABLE I

PARAMETERS FOR ACCELERATION-LIMITED SPEED PROFILE GENERATION

<table><tr><td>Symbol</td><td>Description</td></tr><tr><td>\( {v}_{ini} \)</td><td>Initial speed</td></tr><tr><td>\( {v}_{end} \)</td><td>Final speed</td></tr><tr><td>\( {v}_{max} \)</td><td>Maximum allowed speed along the path</td></tr><tr><td>\( {a}_{max}^{lat} \)</td><td>Maximum lateral acceleration</td></tr><tr><td>long \( {a}_{max} \)</td><td>Maximum positive longitudinal acceleration</td></tr><tr><td>long \( {a}_{min} \)</td><td>Maximum negative longitudinal acceleration</td></tr></table>

Algorithm 1 Acceleration-Limited Speed Planning Algorithm

---

: procedure AL-SPEEDPLANNING \( (P,{v}_{ini},{v}_{end},{v}_{max}, \)
\( {a}_{min}^{long},{a}_{max}^{long},{a}_{max}^{lat}) \)
	\( {v}^{SLC} \leftarrow  f\left( {P\left( \kappa \right) ,{v}_{\text{ini }},{v}_{\text{end }},{v}_{\max },{a}_{\max }^{\text{lat }}}\right) \)
	\( {\Delta s} \leftarrow  f\left( {P\left( {x,y}\right) }\right) \; \vartriangleright \) Distances between path points
	\( {a}_{1,\ldots ,n} \leftarrow  f\left( {{v}^{SLC},{\Delta s}}\right) \; \vartriangleright \) Obtain acceleration
	for \( i \leftarrow  1,n - 1 \) do \( \; \vartriangleright \) Limit maximum positive
acceleration
		if \( {a}_{i} > {a}_{\max }^{\text{long }} \) then
				\( {v}_{i + 1} \leftarrow  f\left( {{v}_{i},{a}_{i},{a}_{i + 1},\Delta {s}_{i}}\right) \)
				\( {a}_{i} \leftarrow  {a}_{\max }^{\operatorname{long}} \)
				\( {a}_{i + 1} \leftarrow  f\left( {{v}_{i + 2},{v}_{i + 1},\Delta {s}_{i + 1}}\right) \)
			end if
		end for
		if \( {v}_{n} < {v}_{\text{end }} \) then
			\( {a}_{fb} \leftarrow  f\left( {\Delta {v}_{0fb},\Delta {v}_{ffb},\Delta {s}_{fb}}\right)  \vartriangleright  {a}_{fb} \) is the long.
acceleration satisfying the initial and final speeds \( \left( {\Delta {v}_{0fb}}\right. \) ,
	\( \left. {\Delta {v}_{0fb}}\right) \) of the fallback section with a length of \( \Delta {s}_{fb} \) .
			FALLBACKFINALSECTION \( \left( {a}_{fb}\right) \)
		end if
	for \( i \leftarrow  n - 1,1 \) do \( \; \vartriangleright \) Limit maximum negative
acceleration
			if \( {a}_{i} < {a}_{\min }^{\text{long }} \) then
				\( {v}_{i} \leftarrow  f\left( {{v}_{i + 1},{a}_{i},{a}_{i + 1},\Delta {s}_{i}}\right) \)
				\( {a}_{i} \leftarrow  {a}_{\min }^{\operatorname{long}} \)
				\( {a}_{i - 1} \leftarrow  f\left( {{v}_{i},{v}_{i - 1},\Delta {s}_{i - 1}}\right) \)
			end if
		end for
		if \( {v}_{0} < {v}_{\text{ini }} \) then
			\( {a}_{fb} \leftarrow  f\left( {\Delta {v}_{0fb},\Delta {v}_{ffb},\Delta {s}_{fb}}\right) \)
			FALLBACKINITIALSECTION \( \left( {a}_{fb}\right) \)
		end if
	return \( v,a\; \vartriangleright \) Return speed and acceleration for \( P \)
end procedure

---

<!-- Media -->

Algorithm 1 shows the complete procedure to obtain an acceleration-limited speed profile associated to a given path. The speed profile calculation is performed in several stages:

1) The first step of algorithm 1 is to compute the speed limit curve,based on the maximum speed \( {v}_{\max } \) and the maximum lateral acceleration \( {a}_{max}^{lat} \) . The maximum allowed speed \( {v}^{SLC} \) is computed using centripetal force equation

\[{v}_{i}^{SLC} = \min \left\{  {{v}_{\max },\sqrt{\frac{{a}_{\max }^{\text{lat }}}{\left| {\kappa }_{i}\right| }}}\right\}   \tag{7}\]

where \( \left( {\kappa }_{i}\right) \) is the curvature of the given path(P)at a specific point \( i \) .

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 8198 IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, VOL. 23, NO. 7, JULY 2022-->

<!-- Media -->

<!-- figureText: \( \times  {10}^{6} \) Final pose UTM easting (m) 4.46258 UTM northing (m) 4.46257 4.46256 4.46255 4.46254 Initial pose 4.46253 -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_4.jpg?x=134&y=162&w=754&h=419&r=0"/>

Fig. 1. Example of acceleration-limited speed planning.

<!-- Media -->

2) After that,initial and final speeds \( \left( {v}_{ini}\right. \) and \( \left. {v}_{end}\right) \) are imposed and the acceleration profile is computed assuming uniform acceleration between two consecutive points of the path:

\[{v}_{i} = \sqrt{{v}_{i - 1}^{2} + 2{a}_{i}\Delta {s}_{i}} \tag{8}\]

where \( \Delta {s}_{i} \) is the distance between points \( i - 1 \) and \( i \) of the path \( P \) .

3) Then, the accelerations computed for each path point are traversed forward in order to verify that they are lower than the maximum acceleration value \( \left( {a}_{\max }^{\text{long }}\right) \) . In case the acceleration at point \( i \) overcomes the limit,it is bounded to the maximum value and the speed at point \( i + 1 \) is recalculated using Eq. (8).

4) Finally, the same procedure followed in the previous step is performed backwards imposing a deceleration limit of \( {a}_{\text{min }}^{\text{long }} \) along the whole path.

Fig. 2 shows an example of the acceleration-limited speed planning over a \( {289.2}\mathrm{\;m} \) path (see Fig 1) with the following constraints: \( {a}_{\text{max }}^{\text{lat }} = {a}_{\text{max }}^{\text{long }} = {1.8}\mathrm{\;m}/{\mathrm{s}}^{2},{a}_{\text{min }}^{\text{long }} =  - {2.5}\mathrm{\;m}/{\mathrm{s}}^{2} \) , \( {v}_{\text{ini }} = {v}_{\text{end }} = 0\mathrm{\;{km}}/\mathrm{h} \) and \( {v}_{\max } = {50}\mathrm{\;{km}}/\mathrm{h} \) . As can be seen, a smooth speed profile is generated while limiting longitudinal and lateral accelerations, as well as guaranteeing initial, final and maximum speed.

## C. Fallback Strategy for Acceleration-Limited Speed Planning

Lines 12-15 and 23-26 of Alg. 1 are devoted to verify if initial an final speeds can be reached with the given maximum acceleration constraints, avoiding thus discontinuities at the beginning or the end of the speed profile.

In the critical cases where initial and final speeds cannot be met, the longitudinal acceleration that allows to satisfy these end-point speed constraints \( \left( {a}_{fb}\right) \) is found applying Eq. 8:

\[{a}_{fb} = \frac{{v}_{ffb}^{2} - {v}_{0fb}^{2}}{{2\Delta }{s}_{fb}} \tag{9}\]

where \( {v}_{0fb},{v}_{ffb} \) and \( \Delta {s}_{fb} \) are the initial and final speeds and the length of the fallback section, respectively. Then a new acceleration-limited speed profile is computed using the calculated fallback acceleration.

In the schematic example of Fig. \( 3,{v}_{\text{ini }} \) cannot be met with the given acceleration constraints. In this case, \( {a}_{min}^{long} \) does not allow \( {v}_{\text{ini }} \) to be reached when performing the backwards correction of the algorithm. To guarantee speed continuity, \( {a}_{fb} \) is found and the initial section of the speed profile is computed. A analogous procedure is applied when \( {v}_{\text{end }} \) cannot be met.

Information about whether the initial and/or final section have been recomputed is stored and given an additional output of the algorithm. This information is helpful to identify fallback actions in the jerk-limited speed planning, as is explained in section V-C.

## V. JERK-LIMITED TIME-OPTIMAL SPEED PLANNING

The acceleration limitation in the speed planning as presented in the previous section is a fast and effective method for speed planning while meeting kinodynamic constraints that can be considered in the speed limit curve. However, abrupt changes in acceleration could cause uncomfortable behaviour inside the vehicle. In these cases when comfort becomes more relevant (e.g. in passenger transportation), the jerk limitation becomes a need.

To meet the requirement of time-optimality when jerk is constrained, time-optimal speed profile must be generated successively applying maximum and minimum jerk profiles [26], while considering the given speed, acceleration and jerk constraints along the trajectory. However, it is reported in the literature [26], [29] that bang-bang approaches based on Pontryagin's Maximum Principle fails at singular arcs in which large amounts of continuous jerk switches are required, thus generating inaccurate results. The proposed approach deals with this kind of singularities by applying the maximum possible jerk, acceleration and speed at each of the points defining the trajectory.

In this section, an algorithm for jerk-limited speed planning is proposed. The objective of the algorithm is to find a mapping of speed \( {v}_{i} \) and acceleration \( {a}_{i} \) for each point \( i \) of the path \( P \) ,taking into account the following constraints,whose parameters are listed in Table II:

\[0 < {v}_{i} < {v}_{\max }\]

\[{a}_{\text{min }}^{\text{long }} < {a}_{i}^{\text{long }} < {a}_{\text{max }}^{\text{long }}\]

\[\left| {a}_{i}^{\text{lat }}\right|  < {a}_{\max }^{\text{lat }}\]

\[{j}_{\text{min }} < {j}_{i} < {j}_{\text{max }}\]

\[{v}_{0} = {v}_{\text{ini }}\]

\[{v}_{n} = {v}_{\text{end }}\]

\[{a}_{0} = {a}_{\text{ini }}\]

\[{a}_{n} = {a}_{\text{end }} \tag{10}\]

The following subsections focus on introducing the jerk-limited motion equations used, the detailed description of the algorithm and the explanations of the fallback strategy used when initial conditions cannot be met.

## A. Constant Jerk Motion

Since jerk must be limited along the whole path, a constant jerk motion is considered between two consecutive points (i - 1andi)of the given path \( P \) . This constant jerk motion along time is described,for each point \( i \) ,by the following expressions:

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: ARTUÑEDO et al.: JERK-LIMITED TIME-OPTIMAL SPEED PLANNING FOR ARBITRARY PATHS 8199-->

\[{s}_{i} = {s}_{i - 1} + {v}_{i - 1}t + \frac{1}{2}{a}_{i - 1}{t}^{2} + \frac{1}{6}{j}_{i}{t}^{3} \tag{11}\]

\[{v}_{i} = {v}_{i - 1} + {a}_{i - 1}t + \frac{1}{2}{j}_{i}{t}^{2} \tag{12}\]

\[{a}_{i} = {a}_{i - 1} + {j}_{i}t \tag{13}\]

<!-- Media -->

<!-- figureText: 50 0 Acceleration., \( a\left( {\mathrm{\;m}/{\mathrm{s}}^{2}}\right) \) 200 250 300 Speed, \( v\left( {\mathrm{\;{km}}/\mathrm{h}}\right) \) 40 30 Speed 20 Speed limit curve Long .acceleration 10 Lat. acceleration 0 50 100 150 Displacement, \( s\left( \mathrm{\;m}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_5.jpg?x=144&y=161&w=1505&h=377&r=0"/>

Fig. 2. Example of acceleration-limited speed planning for path of Fig. 1.

TABLE II

PARAMETERS FOR JERK-LIMITED SPEED PROFILE GENERATION

<table><tr><td>Symbol</td><td>Description</td></tr><tr><td>\( {v}_{ini} \)</td><td>Initial speed</td></tr><tr><td>\( {v}_{end} \)</td><td>Final speed</td></tr><tr><td>\( {a}_{ini} \)</td><td>Initial acceleration</td></tr><tr><td>\( {a}_{end} \)</td><td>Final acceleration</td></tr><tr><td>\( {v}_{max} \)</td><td>Maximum speed</td></tr><tr><td>\( {a}_{max}^{lat} \)</td><td>Maximum lateral acceleration</td></tr><tr><td>long \( {a}_{max} \)</td><td>Maximum positive longitudinal acceleration</td></tr><tr><td>long \( {a}_{min} \)</td><td>Maximum negative longitudinal acceleration</td></tr><tr><td>Imin</td><td>Minimum jerk</td></tr><tr><td>\( j\max \)</td><td>Maximum jerk</td></tr></table>

<!-- figureText: \( {V}_{int} \) \( {a}_{fb} \) Speed limite curve \( {a}_{min}^{long} \) Displacement, \( s \) Speed, \( v \) -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_5.jpg?x=185&y=1031&w=649&h=300&r=0"/>

Fig. 3. Schematic example where \( {v}_{\text{ini }} \) cannot be met.

<!-- Media -->

where \( s,v,a \) and \( j \) are the displacement,speed,acceleration and jerk,respectively,and \( t \) the time. Nevertheless,these equations cannot be directly applied to the stated problem as they are defined in function of time and time independence is required since the position of every point in path \( P \) is invariable (it is an input). Moreover, the speed limit curve also contains speed constraints associated to each point.

Hence, to avoid a explicit time dependence, Eq. (11) can be rearranged replacing time \( t \) by its value in (13):

\[{\Delta s} = \frac{{\left( {a}_{i} - {a}_{i - 1}\right) }^{3}}{6{j}_{i}^{2}} + \frac{{a}_{i - 1}{\left( {a}_{i} - {a}_{i - 1}\right) }^{2}}{2{j}_{i}^{2}} + \frac{{v}_{i - 1}\left( {{a}_{i} - {a}_{i - 1}}\right) }{{j}_{i}}\]

(14)

The problem is reduced to find a value \( {a}_{i} \) in (14) for each pair of points(i - 1andi),given: \( {\Delta s} = {s}_{i} - {s}_{i - 1},{v}_{i - 1} \) , \( {a}_{i - 1} \) and \( {j}_{i} \) ,such that the constraints specified in (10) are met.

Rectilinear motion is assumed between \( {P}_{i - 1} \) and \( {P}_{i} \) : \( i \in  \left\lbrack  {1,n}\right\rbrack \) . As a result,the path \( P \) is handled as a set of \( n - 1 \) rectilinear motions with constant jerk,subject to speed, acceleration and jerk constraints.

## B. Jerk-Limited Speed Planning Algorithm

To solve the stated problem, algorithm 2 is proposed.

<!-- Media -->

Algorithm 2 Jerk-Limited Speed Planning Algorithm

---

\( 1 : \) procedure JL-SPEEDPLANNING \( (P,{v}_{ini},{v}_{end},{v}_{max},{a}_{ini}, \)
	\( {a}_{end},{a}_{min}^{long},{a}_{max}^{long},{a}_{max}^{lat},{j}_{max},{j}_{min}) \)
			\( v,a \leftarrow \) AL-SPEEDPLANNING \( \left( {P,{v}_{\text{ini }},{v}_{\text{end }},{v}_{\max },{a}_{\min }^{\text{long }}}\right. \) ,
	\( \left. {{a}_{\max }^{\text{long }},{a}_{\max }^{\text{lat }}}\right) \)
			AL-CONSTRAINTSCHECK \( \left( {v,{v}_{\text{ini }},{v}_{\text{end }}}\right) \)
			\( j \leftarrow  f\left( {v,a,{\Delta s}}\right) \)
			\( {i}_{\text{local-min }} \leftarrow \) COMPUTELOCALMINIMA(v)
			for all \( i \in  {i}_{\text{local } - \min } \) do
					\( v,a,j \leftarrow \) FIXLOCALMINIMUM(v,a,j,i)
			end for
			\( {i}_{\text{local-max }} \leftarrow \) COMPUTELOCALMAXIMA(v)
			for all \( i \in  {i}_{\text{local-max }} \) do
					\( v,a,j \leftarrow \) FIXLOCALMAXIMUM(v,a,j,i)
			end for
			return \( v,a,j\; \vartriangleright \) Return speed,acceleration and jerk
	for \( P \)
	end procedure

---

<!-- Media -->

The first step of the proposed algorithm is to find local minima of the speed limit curve in order to build up a jerk-limited sections of the speed profile. To that end, the acceleration-limited speed planner (Alg. 1) is initially used, leveraging on its simplicity and computational efficiency. This step is not strictly necessary, as local minima could be extracted directly from the speed limit curve. However, the geometry of the path can result in a great variety of possibilities, leading to significant complexity differences when using the speed limit curve. The output of Alg. 1 typically provides a lower amount of local minima and consequently a lower and more stable total execution time while achieving the same results. To illustrate that effect, Fig. 2 shows a case in which 10 local minima would be obtained from the speed limit curve (red line), while after applying Alg. 1, only 4 local minima are attained (blue line).

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 8200 IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, VOL. 23, NO. 7, JULY 2022-->

<!-- Media -->

---

Algorithm 3 Procedure for Local Minimum Fix
	procedure FixLOCALMINIMUM \( \left( {v,a,j,{i}_{min}}\right) \)
		\( {\text{flag}}_{\text{sol }} \leftarrow \) false
		while \( {fla}{g}_{\text{sol }} = \) false do
				for \( i \leftarrow  {i}_{\min },n - 2 \) do
					if \( {a}_{i} < {a}_{\max }^{\text{long }} \) then
						\( {j}_{aux} \leftarrow  {j}_{max} \vartriangleright  {j}_{aux},{a}_{aux},{v}_{aux} \) are the aimed
	jerk, acc. and speed values, respectively.
						\( {a}_{aux},{v}_{aux} \leftarrow  f\left( {{j}_{i},{a}_{i},{v}_{i},\Delta {s}_{i}}\right) \)
					else
						\( {j}_{\text{aux }} \leftarrow  0 \)
						\( {a}_{aux} \leftarrow  {a}_{i} \)
						\( {v}_{\text{aux }} \leftarrow  f\left( {{v}_{i},{ai} + 1,\Delta {s}_{i}}\right) \)
					end if
					if \( {v}_{\text{aux }} > {v}_{i + 1} \) then \( \vartriangleright \) Speed profile reached
						break
					else
						\( {v}_{i + 1} \leftarrow  {v}_{\text{aux }} \)
						\( {a}_{i + 1} \leftarrow  {a}_{aux} \)
						\( {j}_{i} \leftarrow  {j}_{aux} \)
						\( {j}_{j + 1} \leftarrow  f\left( {{a}_{i + 1},{a}_{i + 2},{v}_{i + 1},\Delta {s}_{i + 1}}\right) \)
					end if
				end for
				if \( i = n - 2\& {v}_{i + 1} < {v}_{\text{end }} \) then \( \vartriangleright  {v}_{\text{end }} \) not met
					\( {j}_{\max } \leftarrow  {j}_{\max } + \Delta {j}_{fb} \)
				else
					flagsol \( \leftarrow \) true
				end if
			end while
			\( {\text{flag}}_{\text{sol }} \leftarrow \) false
			while \( {fla}{g}_{\text{sol }} = \) false do
				for \( i \leftarrow  {i}_{\min },2 \) do
					if \( {a}_{i} > {a}_{\min }^{\text{long }} \) then
						\( {j}_{\text{aux }} \leftarrow  {j}_{\max } \)
						\( {a}_{aux},{v}_{aux} \leftarrow  f\left( {{j}_{i},{a}_{i + 1},{v}_{i + 1},\Delta {s}_{i + 1}}\right) \)
					else
						\( {j}_{\text{aux }} \leftarrow  0 \)
						\( {a}_{aux} \leftarrow  {a}_{i + 1} \)
						\( {v}_{\text{aux }} \leftarrow  f\left( {{v}_{i + 1},{ai},\Delta {s}_{i}}\right) \)
					end if
					if \( {v}_{i} > {v}_{i}^{SLC} \) then \( \vartriangleright \) Speed profile reached
						break
					else
						\( {v}_{i} \leftarrow  {v}_{aux} \)
						\( {a}_{i} \leftarrow  {a}_{aux} \)
						\( {j}_{i} \leftarrow  {j}_{aux} \)
						\( {j}_{i - 1} \leftarrow  f\left( {{a}_{i},{a}_{i - 1},{v}_{i},\Delta {s}_{i}}\right) \)
					end if
				end for
				if \( i = 0\& {v}_{i} < {v}_{\text{ini }} \) then \( \; \vartriangleright  {v}_{\text{ini }} \) not met
					\( {j}_{\min } \leftarrow  {j}_{\min } - \Delta {j}_{fb} \)
				else
					flagsol \( \leftarrow \) true
				end if
			end while
		return \( v,a,j \)
	end procedure

---

<!-- figureText: 20 Speed limit curve Long acc. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right)  \mid \) Jerk \( \left( {\mathrm{m}/{\mathrm{s}}^{3}}\right) \) - Long. acceleration Jerk - Before Alg. 3 25 30 Speed, \( v\left( {\mathrm{\;{km}}/\mathrm{h}}\right) \) 15 10 5 10 15 20 Displacement, \( s\left( \mathrm{\;m}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_6.jpg?x=908&y=159&w=754&h=360&r=0"/>

Fig. 4. Acceleration discontinuity fix at speed local minimum.

<!-- Media -->

Once every local minima is identified, they are used to generate jerk and acceleration limited sections of the speed profile (Alg. 3). An illustrative example of this procedure is shown in the speed profile in Fig. 4, where dashed lines shows the speed profile before computing Alg. 3 on the minimum at \( {20.5}\mathrm{\;m} \) ,and the result after this procedure is shown in continuous lines. This procedure sets null acceleration at the location of each local minimum speed (except at the initial and final point of the path) and computes two speed profile sections from local speed minimum, one forward and another one backward. The maximum positive allowed jerk \( \left( {j}_{max}\right) \) is applied until the maximum allowed accelerations are reached (maximum positive acceleration \( {a}_{\max } \) in forward section and maximum negative acceleration \( {a}_{\min } \) in backward section) or the acceleration limited speed profile is reached. Thus, the maximum jerk of the whole trajectory is limited to \( {j}_{max} \) . In the example in Fig. 4,the maximum positive allowed jerk \( \left( {j}_{max}\right) \) is set to \( {0.5}\mathrm{\;m}/{\mathrm{s}}^{3},{a}_{\max } = {1.8}\mathrm{\;m}/{\mathrm{s}}^{2} \) and \( {a}_{\text{min }} =  - {1.8}\mathrm{\;m}/{\mathrm{s}}^{2} \) . It can be seen that,on the forward speed profile computation, the maximum positive acceleration is reached at \( {31}\mathrm{\;m} \) and the speed reaches acceleration limited speed profile at \( {32}\mathrm{\;m} \) . Starting from local minima at \( {20.5}\mathrm{\;m} \) , the backward computation is carried out until the computed speed reaches acceleration limited speed profile at \( {8.6}\mathrm{\;m} \) . In this case, the maximum negative acceleration is reached at \( {10.5}\mathrm{\;m} \) . In this example it can also be seen that,although the acceleration has been limited to the maximum allowed values when computing Alg. 1,the positive jerk values exceeded \( {j}_{max} \) before computing Alg. 3. Nevertheless, as a result of applying this algorithm, it is ensured that the maximum jerk obtained in this section is limited to \( {j}_{\max } \) . By applying this procedure to all identified local minima, it is consequently ensured that \( {j}_{max} \) is not exceeded across the entire speed profile.

Once the maximum jerk has been limited, a subsequent computation is carried out to find local maxima of the speed profile. Alg. 4 is then applied at each local maximum (labelled as \( {i}_{max} \) ) to correct the speed profile by limiting the jerk to the minimum allowed jerk \( \left( {j}_{min}\right) \) .

Alg. 4 is illustrated in Fig. 5, which depicts the speed, longitudinal acceleration and jerk before (in dashed lines) and after (continuous lines) applying Alg. 4. This algorithm consist of an iterative procedure that, starting from point \( {i}_{\max } - 1 \) ,imposes the minimum allowed jerk \( {j}_{\min } \) to find the closest point before \( {i}_{\max } \) where to start applying \( {j}_{\min } \) while satisfying acceleration and speed continuity at the end of the section being computed (index \( {i}_{k} \) in Alg. 4). Three iterations of this procedure are represented in Fig. 5. The first iteration shown starts from point \( {i}_{o}^{\prime } \) and reach the current speed profile at \( {i}_{k}^{\prime } \) (dotted line),where an acceleration discontinuity is found. A subsequent iteration starts from \( {i}_{o}^{\prime \prime } \) and reach the current speed profile at \( {i}_{k}^{\prime \prime } \) but still the acceleration presents a discontinuity. Finally, a solution that meets the acceleration and speed continuity is found from \( {i}_{o} \) to \( {i}_{k} \) and the final jerk, acceleration and speed for that section is computed. Thus, it is guaranteed that \( {j}_{min} \) is not exceeded when a solution for that section is found.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: ARTUÑEDO et al.: JERK-LIMITED TIME-OPTIMAL SPEED PLANNING FOR ARBITRARY PATHS 8201-->

<!-- Media -->

Algorithm 4 Procedure for Local Maximum Fix

---

procedure FIXLOCALMAXIMUM \( \left( {v,a,j,{i}_{max}}\right) \)
	\( k \leftarrow  {i}_{\max } + 1 \)
	for \( {i}_{o}^{\prime } \leftarrow  {i}_{\max } - 1,2 \) do
		while \( {i}_{k} \in  \left\lbrack  {{i}_{\max },n}\right\rbrack \) do
			\( {a}_{aux} \leftarrow  f\left( {{j}_{min}\ldots }\right) \; \vartriangleright  {a}_{aux},{v}_{aux} \) are the aimed
acc. and speed values, respectively.
			\( {v}_{aux} \leftarrow  f\left( {{v}_{i + 1},{a}_{aux},{j}_{i}}\right) \)
			if \( {v}_{aux} < {v}_{k}^{SLC}\& {a}_{aux} < {a}_{k} \) then
				flagsol \( \leftarrow \) true
				\( {i}_{o} = {i}_{o}^{\prime } \)
				break
			end if
			\( {i}_{k} \leftarrow  {i}_{k} + 1 \)
		end while
	end for
	if \( {fla}{g}_{sol} \) then
		COMPUTESECTION \( \left( {{i}_{o},{i}_{k}}\right) \)
	else \( \; \vartriangleright \) Initial/final conditions cannot be met
		\( {j}_{fb} \leftarrow  j + \Delta {j}_{fb} \)
		\( \operatorname{FIXLOCALMAXIMUM}\left( {v,a,{j}^{\prime },{i}_{max}}\right) \)
	end if
	return \( v,a,j \)
end procedure

---

<!-- figureText: 40 Acceleration discontinuity 30 40 50 60 Displacement, \( s\left( \mathrm{\;m}\right) \) Speed, \( v\left( {\mathrm{\;{km}}/\mathrm{h}}\right) \) 20 Speed Long. acceleration Jerk - Before Alg.4 10 20 -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_7.jpg?x=133&y=1112&w=756&h=381&r=0"/>

Fig. 5. Acceleration discontinuity fix at speed local maximum.

<!-- Media -->

Fig. 6 shows an example of the jerk-limited speed planning over the same path used in the acceleration-limited speed profile in Fig. 2. The following constraints were used to compute this speed profile: \( {j}_{\max } = 1\mathrm{\;m}/{\mathrm{s}}^{3},{j}_{\min } =  - 1\mathrm{\;m}/{\mathrm{s}}^{3} \) , \( {a}_{\text{max }}^{\text{lat }} = {a}_{\text{max }}^{\text{long }} = {1.8}\mathrm{\;m}/{\mathrm{s}}^{2},{a}_{\text{min }}^{\text{long }} =  - {2.5}\mathrm{\;m}/{\mathrm{s}}^{2},{v}_{\text{ini }} = \) \( {v}_{\text{end }} = 0\mathrm{\;{km}}/\mathrm{h} \) and \( {v}_{\max } = {50}\mathrm{\;{km}}/\mathrm{h} \) . As can be observed, all constraints are satisfied along the entire trajectory.

<!-- Media -->

TABLE III

Parameters for Each of the Four Zones of Speed Profile in Fig. 7

<table><tr><td rowspan="2">Zone</td><td>Imax</td><td>Imin</td><td>\( {a}_{max}^{long} \)</td><td>\( {a}_{min}^{long} \)</td><td>\( {a}_{max}^{lat} \)</td><td>\( {v}_{max} \)</td></tr><tr><td colspan="2">\( \left\lbrack  {m/{s}^{3}}\right\rbrack \)</td><td colspan="3">\( \lbrack m/{s}^{2} \)</td><td>\( \left\lbrack  {{km}/h}\right\rbrack \)</td></tr><tr><td>I</td><td>0.5</td><td>-0.8</td><td>0.8</td><td>-1.2</td><td>1.5</td><td>25</td></tr><tr><td>II</td><td>0.3</td><td>-0.3</td><td>1.0</td><td>-1.0</td><td>1.0</td><td>15</td></tr><tr><td>III</td><td>1.3</td><td>-1.5</td><td>2.0</td><td>-3.0</td><td>2.5</td><td>60</td></tr><tr><td>IV</td><td>0.7</td><td>-0.7</td><td>1.8</td><td>-2.0</td><td>2.0</td><td>35</td></tr></table>

<!-- Media -->

Note that it is possible to adapt the speed planning behaviour over the path by varying the speed planner parameters \( \left( {{j}_{max},{j}_{min},{a}_{max}^{lat},{a}_{max}^{long},{a}_{min}^{long}\text{and}{v}_{max}}\right) \) . This adaptation can be based on comfort (e.g. considering passengers driving preferences allowing them to choose the driving smoothness level), safety criteria (e.g. considering high deceleration values to automatically avoid collisions) or the road context taking into account the limits of adhesion of the vehicle in specific areas that could be altered by road conditions, weather, etc.

To show the flexibility of the proposed method with respect to the variation of these parameters, Fig. 7 shows an example where four zones (I - IV) with different parameters are traversed by the trajectory. These sets of parameters are shown in Table III and the initial and final speeds and accelerations are equal to 0 in this example. It can be observed that neither speed, accelerations and jerk exceed the bounds imposed in each of the zones. Moreover, the proposed method is able to successfully compute the transitions needed to guarantee that the constraints are satisfied from the beginning of each zone. For example,at \( s = {347}\mathrm{\;m} \) (end of zone III),the longitudinal acceleration (green line) starts increasing to ensure that the minimum longitudinal acceleration of zone IV is not exceeded. A similar case regarding speed limits can be seen at the end of zone \( \mathrm{I}\left( {s = {121}\mathrm{\;m}}\right) \) ,where speed starts decreasing to ensure that the maximum allowed speed at zone II \( \left( {{15}\mathrm{\;{km}}/\mathrm{h}}\right) \) is not exceeded.

In some cases, initial and/or final conditions may not be satisfied depending on the given constraints for jerk-limited speed planning. In these cases, fallback constraints are used to compute the initial/final section. The fallback strategy is detailed in the following subsection.

## C. Fallback Strategy in Jerk-Limited Speed Planning

As indicated in line 3 of Alg. 2, the jerk-limited planning algorithm runs Alg. 1 at the beginning, thus ensuring that initial and final speeds are met. However, in some specific case initial/final speeds and accelerations could not be met after applying jerk limitations. Thus, when the initial or final path point is reached while calculating a speed profile section with positive jerk (Alg. 3), it is firstly checked that the section being computed has not been previously computed by applying a fallback acceleration as described in section IV-C. If it is not the case,it is assumed that initial acceleration \( \left( {a}_{ini}\right) \) and jerk limits cannot be met in this section and the speed profile in this section remains the same. In other words, if a less restrictive acceleration had been imposed in the fallback calculation when limiting acceleration (as shown in Fig. 3), lower speeds would be computed if jerk were limited in that section, making it impossible to reach the initial/final speed. Otherwise, the jerk-limited calculation is carried out. After that, it is checked that initial and final speeds are greater than \( {v}_{\text{ini }},{v}_{\text{end }} \) ,respectively. If not, \( {fla}{g}_{\text{sol }} \) in Alg. 3 is not activated and a fallback maximum jerk is used to recompute that section. The fallback maximum jerk is computed by increasing the maximum allowed jerk a given amount \( \Delta {j}_{fb} \) so that \( {j}_{max} = {j}_{max} + \Delta {j}_{fb} \) . This fallback procedure is iteratively computed until a solution is found or a maximum \( {j}_{ax} \) is reached \( \left( {j}_{jb}^{\max }\right) \) . In the latter case,the acceleration-limited profile for this section is conserved, assuming that jerk cannot be limited in this section.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 8202 IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, VOL. 23, NO. 7, JULY 2022-->

<!-- Media -->

<!-- figureText: 50 0 Accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right)  \mid \) Jerk \( \left( {\mathrm{m}/{\mathrm{s}}^{3}}\right) \) 200 250 300 Speed, \( v\left( {\mathrm{\;{km}}/\mathrm{h}}\right) \) 40 30 20 Speed Speed limit curve Long .acceleration 10 Lat. acceleration Jerk 0 50 100 150 Displacement, \( s\left( \mathrm{\;m}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_8.jpg?x=142&y=161&w=1517&h=376&r=0"/>

Fig. 6. Example of jerk-limited speed planning for path of Fig. 1.

<!-- figureText: 60 III IV Acceleration \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right)  \mid \) Jerk \( \left( {\mathrm{m}/{\mathrm{s}}^{3}}\right) \) 300 350 400 450 500 Speed Long. acceleration 50 Speed limit curve Lat. acceleration Jerk Speed, v (km/h) 40 30 20 10 II 0 50 100 150 200 250 Displacement, s (m) -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_8.jpg?x=131&y=591&w=1533&h=390&r=0"/>

Fig. 7. Example of jerk-limited speed planning with different parameters in each of the four zones.

<!-- figureText: Speed limit curve Speed limit curve Acc.-limited profile Jmin \( {v}_{ini} \) Local Speed, \( v \) min \( = {j}_{\min }^{\prime } - \Delta {j}_{fb} \) Jerk-limited profile Displacement, \( s \) Acc.-limited profile \( {v}_{ini} \) \( {j}_{max}^{\prime } = {j}_{max} + \Delta {j}_{fb} \) Local Speed, \( v \) limin. Jerk-limited profile Displacement, \( s \) -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_8.jpg?x=133&y=1052&w=751&h=342&r=0"/>

(a) Fallback strategy to increase \( {j}_{max}\left( \mathrm{b}\right) \) Fallback strategy to decrease \( {j}_{min} \)

Fig. 8. Fallback strategies in jerk-limited speed planning.

<!-- Media -->

An illustrative example of this strategy is shown in the speed profile section in Fig. 8a. It can be seen that the initial attempt to compute the backward section of the local minimum end with an initial speed lower than \( {v}_{ini} \) . Then,a fallback maximum jerk \( {j}_{max}^{\prime } \) is used to recompute that section but again the initial speed is lower than \( {v}_{\text{ini }} \) . A second fallback maximum jerk \( {j}_{\max }^{\prime \prime } \) is once again increased and the \( {v}_{\text{ini }} \) is met.

While computing the procedure for local maximum fix (Alg. 3), it is also possible that a solution cannot be found with the given jerk and acceleration limits. The strategy in this case is to decrease the minimum allowed jerk a given amount \( \Delta {j}_{fb} \) ,so that \( {j}_{min} = {j}_{min} - \Delta {j}_{fb} \) and recompute it again. The procedure is recursively called (line 19 in Alg. 4) until a solution is found or a minimum \( {j}_{min} \) value is reached \( \left( {j}_{jb}^{\min }\right) \) .

An example of this strategy is shown in Fig. 8b. In this case, the first three represented iterations,where \( {j}_{min} \) is applied, are part of the normal operation of the algorithm. However, the first point of the trajectory is reached and a solution is not found using that value for \( {j}_{min} \) ,meaning that a lower value is needed. The first fallback minimum jerk \( {j}_{\text{min }}^{\prime } \) reaches the speed profile but acceleration continuity is not found in that point. A second iteration reduces again the minimum allowed jerk \( \left( {j}_{\text{min }}^{\prime \prime }\right) \) and a solution is found.

As in the previous case, when a solution is not found and \( {j}_{jb}^{\min } \) is reached,the acceleration-limited profile for this section is conserved, assuming that jerk cannot be limited in this section. Please note that, for the sake of clarity, this mechanism is not included in the pseudocode in Alg. 4. Also note that, although the examples in Fig. 8 show the fallback strategies applied to the initial section when the initial conditions cannot be met, an analogous mechanism is applied when the final conditions cannot be met with the original jerk limits.

## VI. EXPERIMENTAL RESULTS

The performance of the proposed speed planning algorithm has been extensively evaluated by computing speed profiles for a large set of paths with different geometries and lengths. Moreover, some of the generated trajectories have been tested on an instrumented vehicle.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: ARTUÑEDO et al.: JERK-LIMITED TIME-OPTIMAL SPEED PLANNING FOR ARBITRARY PATHS 8203-->

<!-- Media -->

<!-- figureText: AUTOPIA's vehicle equipment Trunk A/D I/O Touch Actuators switches ② LIDAR ④ IMU ⑤ PC -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_9.jpg?x=215&y=167&w=598&h=478&r=0"/>

Fig. 9. Experimental platform.

<!-- Media -->

The experimental platform used for testing and the evaluation details are respectively described in the following subsections.

## A. Experimental Platform

The proposed algorithm has been validated by performing different experiments using one of the automated vehicles of the AUTOPIA Program (see Fig. 9) at the test track of the Centre for Automation and Robotics (CSIC-UPM) in Arganda del Rey, Spain.

The localization of the vehicle used in these tests relies on a RTK-DGPS receiver and on-board sensors to measure vehicle speed, accelerations and yaw rate. The vehicle also includes a computer with an Intel Core i7-3610QE and 8Gb RAM, which is used to run the planning and high-level control algorithms [1]. This software architecture runs on a soft real-time Linux-based operating system and dedicated digital positioning controllers are in charge of low-level control tasks.

The trajectory tracking system used in this work, relying on fuzzy logic, is designed and behaves in a decoupled manner: on the one hand, the lateral controller uses both lateral and angular errors measured from the ego-vehicle pose with respect to the reference path, to compute the steering wheel position. On the other hand, the longitudinal controller computes the positions of throttle and brake pedals from the reference speed profile and the speed error (resulting from the difference of that reference and the measured speed of the vehicle). Further details about the trajectory tracking can be found in [11].

## B. Trials on Test Track

Two different paths were used to test the proposed algorithm in real scenarios (see Fig. 10). While Path 1 includes a route with a high concentration of curves in a section of 100 meters, Path 2 (of 200 meters) includes a straight section of 180 meters that allows to reach higher speeds. Both paths were generated by applying the approach described in [3], i.e. path points, orientation and curvature are obtained from the analytic equations of the Bézier curves.

<!-- Media -->

<!-- figureText: \( \times  {10}^{6} \) Final pose Path 1 Path 2 4.5896 4.5898 4.59 4.5902 4.5904 4.5906 JTM easting (m) \( \times  {10}^{5} \) 4.46258 Initial pose UTM northing (m) 4.46256 4.46254 4.46252 4.4625 4.5888 4.589 4.5892 4.5894 -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_9.jpg?x=906&y=161&w=756&h=433&r=0"/>

Fig. 10. Paths used in trials.

TABLE IV

RESULTS OBTAINED FOR ALL SETUPS AND PATHS

<table><tr><td rowspan="2">Setup</td><td rowspan="2">\( {j}_{max} \)</td><td rowspan="2">\( {j}_{\min } \)</td><td colspan="2">Path 1</td><td colspan="2">Path 2</td></tr><tr><td>MSJ</td><td>\( {MS}{E}_{a} \)</td><td>MSJ</td><td>\( {MS}{E}_{a} \)</td></tr><tr><td>1</td><td>0.1</td><td>-0.1</td><td>1.52</td><td>0.0510</td><td>1.87</td><td>0.0425</td></tr><tr><td>2</td><td>0.2</td><td>-0.2</td><td>1.79</td><td>0.0920</td><td>2.30</td><td>0.0787</td></tr><tr><td>3</td><td>0.3</td><td>-0.3</td><td>2.07</td><td>0.0870</td><td>2.56</td><td>0.0839</td></tr><tr><td>4</td><td>0.5</td><td>-0.5</td><td>2.46</td><td>0.0899</td><td>2.93</td><td>0.1236</td></tr><tr><td>5</td><td>0.8</td><td>-0.8</td><td>2.81</td><td>0.1804</td><td>3.98</td><td>0.2670</td></tr><tr><td>6</td><td>1.0</td><td>-1.0</td><td>3.07</td><td>0.2594</td><td>4.42</td><td>0.3144</td></tr><tr><td>7</td><td>-</td><td>-</td><td>4.21</td><td>0.5703</td><td>4.75</td><td>0.4937</td></tr></table>

<!-- Media -->

Aiming at quantifying the jerkiness to compare the resulting vehicle performance while tracking the generated trajectories, the mean square jerk metric \( \left( {MSJ}\right) \left\lbrack  6\right\rbrack \) is used to represent the jerkiness of each test. Note that this metric is one of the most relevant ones in terms of quantifying the ride comfort [13]. It is defined as follows:

\[{MSJ} = \frac{1}{{t}_{f} - {t}_{0}}\mathop{\sum }\limits_{{t}_{0}}^{{t}_{f}}\frac{\Delta {\widehat{a}}_{long}^{2}}{\Delta t} \tag{15}\]

where \( {\widehat{a}}_{\text{long }} \) is the longitudinal acceleration measured in the vehicle during the trials and \( {t}_{0} \) and \( {t}_{f} \) are the initial and final time of the test.

For each of the paths defined, a set of 7 speed profiles were generated considering different speed planning parameters, as indicated in Table IV. While the maximum speed allowed in Path 1 was \( {v}_{\max } = {40}\mathrm{\;{km}}/\mathrm{h},{v}_{\max } = {50}\mathrm{\;{km}}/\mathrm{h} \) was set for Path 2 trajectories. In all setups the following values have been the same: \( {a}_{\max }^{\text{long }} = {1.2}\mathrm{\;m}/{\mathrm{s}}^{2},{a}_{\min }^{\text{long }} =  - 2\mathrm{\;m}/{\mathrm{s}}^{2} \) and \( \left| {a}_{\max }^{\text{lat }}\right|  = \) \( {1.2}\mathrm{\;m}/{\mathrm{s}}^{2} \) . In addition,the following values used in fallback strategy (section V-C) were considered: \( \Delta {j}_{fb} =  - {0.5}\mathrm{\;m}/{\mathrm{s}}^{3} \) and \( {j}_{jb}^{\max } =  - {3m}/{s}^{3} \) .

For the sake of clarity, only some representative setups are depicted for each path. The generated speed profiles for setups \( 5\left( {{j}_{\max } = {0.8}\mathrm{\;m}/{\mathrm{s}}^{3}}\right) \) and 7 (no jerk bounds) are shown in Fig. 11. Fig. 12 shows the speed profile computed in setups \( 3\left( {{j}_{\max } = {0.3}\mathrm{\;m}/{\mathrm{s}}^{3}}\right) \) and 7 . In both figures,continuous blue, green and yellow lines show the jerk-limited speed, acceleration and jerk, respectively, while dashed lines refer to acceleration-limited speed profile. As can be seen, the constraints are respected along both speed profiles. Note that no abrupt acceleration changes occurs in jerk-limited profiles in contrast to just acceleration-limited profiles. For example, at \( {205}\mathrm{\;m} \) in the speed profile in Fig. 11,an acceleration discontinuity in the acceleration-limited profile is corrected by the jerk-limited speed planning algorithm while respecting the maximum accelerations: starting from the maximum positive acceleration at \( {187}\mathrm{\;m} \) ,the minimum jerk is applied until the maximum negative acceleration is reached at \( {219}\mathrm{\;m} \) . Likewise, note that jerk-limited speed profiles have a smoother speed evolution along the path, which makes driving more comfortable when compared to acceleration-limited profiles.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 8204 IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, VOL. 23, NO. 7, JULY 2022-->

<!-- Media -->

<!-- figureText: 40 0 Long acc. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right)  \mid \) Jerk \( \left( {\mathrm{m}/{\mathrm{s}}^{3}}\right) \) Speed Long .acceleration Speed limit curve Jerk 150 200 250 Speed, \( v\left( {\mathrm{\;{km}}/\mathrm{h}}\right) \) 30 20 10 0 0 50 100 Displacement, \( s\left( \mathrm{\;m}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_10.jpg?x=131&y=162&w=1535&h=362&r=0"/>

Fig. 11. Speed profiles of Path 1. Continuous lines: Setup \( 5\left( {{j}_{max} = {0.8m}/{s}^{3},{j}_{min} =  - {0.8m}/{s}^{3}}\right. \) ,Dashed lines: setup 7 (no jerk limited).

<!-- figureText: 50 1.5 0.5 Speed limit curve Long .acceleration -1.5 300 350 400 450 500 40 Speed, \( v\left( {\mathrm{\;{km}}/\mathrm{h}}\right) \) 30 20 Speed 10 Jerk 0 50 100 150 200 250 Displacement, \( s\left( \mathrm{\;m}\right) \) -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_10.jpg?x=142&y=594&w=1510&h=384&r=0"/>

Fig. 12. Speed profiles of Path 2. Continuous lines: Setup \( 3\left( {{j}_{max} = {0.3m}/{s}^{3},{j}_{min} =  - {0.3m}/{s}^{3}}\right. \) ,). Dashed lines: setup 7 (no jerk limited).

<!-- Media -->

The proposed approach does not necessarily impose maximum, minimum or null jerk on each point of the trajectory but intermediate values can be obtained in sections where the imposed jerk limits are not exceeded. This effect can be observed around \( {25}\mathrm{\;m} \) and \( {55}\mathrm{\;m} \) of the Path 1 results shown in Fig. 11, where the speed limit curve in the first two path curves present almost flat sections that do not cause high jerk values exceeding the imposed limits.

## C. Performance Evaluation

All the setups listed in Table IV were tested for both paths in the experimental platform. The values of \( {MSJ} \) obtained for each of the performed trials (7 different setups and 2 paths) are shown in columns 4 (for in Path 1) and 6 (for trials in Path 2) of Table IV. As can be observed, the jerkiness in the vehicle increases when the maximum allowed jerk grows consequently decreasing the ride comfort. Moreover, it is worth noting that the maximum values of \( {MSJ} \) are reached in setup 7 (acceleration-limited speed profile) for both paths, i.e. when there is no jerk constraint in the speed profile.

This effect can be graphically seen in Fig. 13. This figure shows density graphs of the actual longitudinal and lateral accelerations measured along the trials on Path 2. It can be noticed that greater negative (acceleration applied in left direction) than positive lateral acceleration values are measured. This is due to the fact that the vehicle must negotiate more right than left turns to reach the destination. Moreover, it is also observed that a broader spectrum is covered by the measured acceleration when jerk is limited (setups 1-6) in contrast to setup 7 (acceleration-limited speed profile). For example, in setup 5 (Fig. 13-e), intermediate acceleration values are measured between maximum allowed longitudinal accelerations \( \left( {{a}_{\max }^{\text{long }} = {1.2}\mathrm{\;m}/{\mathrm{s}}^{2},{a}_{\min }^{\text{long }} =  - 2\mathrm{\;m}/{\mathrm{s}}^{2}}\right) \) unlike setup 7, causing a smoother driving. This effect shows a clear benefit of using jerk-limited speed planning over acceleration-limited approaches.

It can be noticed that while most of the measured acceleration values are under the limits imposed in the speed planning, some of them go further. This effect is mainly caused by the lack of gravity compensation in accelerometers measurements. Indeed, road camber and longitudinal slope affect lateral and longitudinal acceleration measurements, respectively. The section of the test track traversed by Path 1 and 2 presents negligible longitudinal slopes. However, road camber is noticeable (around \( 6\% \) ),causing lateral acceleration reach up to \( {0.6}\mathrm{\;m}/{\mathrm{s}}^{2} \) of measurements when the vehicle remains stopped in the steepest section of the track. Furthermore, lateral acceleration measurements are also influenced by vibrations induced by road imperfections and by positive speed tracking errors when driving on curves, thus generating greater centrifugal forces than planned. Consequently, greater disturbances are perceived in lateral acceleration measurements rather than in longitudinal acceleration ones.

When maximum allowed jerk is set to higher values (Figs. 13e-g), some positive longitudinal accelerations beyond the allowed limit can be found. These higher acceleration values are originated by the longitudinal controller reaction to the speed error caused by that the lack of acceleration during the shift from first to second gear. This effect can be easily seen in the temporal evolution of speed and accelerations of the trial on Path 2 with setup 7 depicted in Fig. 14 (at time \( t = {6s},t = {20s} \) and \( t = {45s} \) ). However,when the maximum allowed jerk is below \( {0.8}\mathrm{\;m}/{\mathrm{s}}^{3} \) ,the vehicle is able to smoothly perform the gear shift as can be observed in the speed and acceleration evolution of setup 3 in Path 2 shown in Fig. 15. In order to quantify this effect, the mean squared error of the acceleration tracking \( \left( {\mathrm{{MSE}}}_{a}\right) \) is shown in Table IV. As reflected,the \( {MS}{E}_{a} \) increases when the allowed jerk does. Moreover, the greatest value is obtained for setup when no jerk limited. This highlights the positive impact of jerk bounding in the performance of trajectory tracking controllers with respect to acceleration-limited speed planning approaches.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: ARTUÑEDO et al.: JERK-LIMITED TIME-OPTIMAL SPEED PLANNING FOR ARBITRARY PATHS 8205-->

<!-- Media -->

<!-- figureText: Time (s) Time (s) Long. accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) Time (s) Time (s) 4.4 Long. accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) 3.48 1.74 -1 2.2 0 -2 C -2 Lateral accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) -2 Lateral accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) (c) Setup 3 (d) Setup 4 Time (s) Time (s) 3.04 Long. accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) 3.76 0 1.88 - 1 0 -2 1.52 -2 Lateral accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) Lateral accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) (g) Setup 7 Long. accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) 10.6 5.68 0.5 5.3 2.84 -0.5 -2 2 0 Lateral accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) Lateral accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) (a) Setup 1 (b) Setup 2 Long. accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) 2.76 Long. accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) 1.38 -1 -2 -2 Lateral accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) (e) Setup 5 (f) Setup 6 -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_11.jpg?x=144&y=165&w=1509&h=542&r=0"/>

Fig. 13. Density graph of measured acceleration in the vehicle during the trials in Path 2.

<!-- figureText: 60 40 50 60 70 80 Time (s) Long. acceleration Lateral acceleration 40 50 60 70 80 Time (s) (b) Longitudinal and lateral accelerations Reference speed Speed (km/h) Vehicle speed 20 10 20 30 (a) Reference and measured speed - Reference long. acceleration Acceleration ( \( \mathrm{m}/{\mathrm{s}}^{2} \) ) 1 -1 0 10 20 30 -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_11.jpg?x=166&y=793&w=693&h=672&r=0"/>

Fig. 14. Speed and accelerations during trial on Path 2 with setup 7 (no jerk limited).

<!-- figureText: 40 Reference speed 50 60 70 80 Time (s) (a) Reference and measured speed Long. acceleration Lateral acceleration 50 60 70 Time (s) Speed \( \left( {\mathrm{{km}}/\mathrm{h}}\right) \) Vehicle speed 30 20 10 0 30 40 Acceleration ( \( \mathrm{m}/{\mathrm{s}}^{2} \) ) 2 1 0 - 1 Reference long. acceleration 10 20 30 40 (b) Longitudinal and lateral accelerations -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_11.jpg?x=941&y=787&w=690&h=645&r=0"/>

Fig. 15. Speed and accelerations during trial on Path 2 with setup 3.

<!-- Media -->

Finally, an additional trial was carried out with the trajectory in Fig. 7. This trajectory comprises four different sets of parameters specified in Table III. The resulting speed and acceleration during this trial are shown in Fig. 16. As can be observed, both planned speed (Fig. 16a) and acceleration (Fig. 16b) are tracked with small errors in each of the four zones. The main differences between them are due to gear shifting: at the beginning, when first gear is being engaged \( \left( {\mathrm{t} = 2\mathrm{\;s}}\right) \) ,or in the gear shift at \( \mathrm{t} = {50}\mathrm{\;s} \) . Nevertheless, the tracking smoothness considerably increases with respect to speed profiles that do not limit the jerk. Note also that the transitions between two zones is performed smoothly as planned, thus guaranteeing the stated constraints in each zone are not exceeded.

## D. Comparison With a Similar Approach

With the aim of comparing the proposed jerk-limited speed planning approach with other state-of-the-art methods, a test was carried out to compare the proposed method with the one proposed in [35]. This test consisted of computing the speed profiles using both approaches under the same path and speed planning constraints. The path used was Path 1 (see Fig. 10) and the speed planning constraints were: \( {v}_{\max } = {40}\mathrm{\;{km}}/\mathrm{h} \) , \( {a}_{\max }^{\text{long }} = {1.2}\mathrm{\;m}/{\mathrm{s}}^{2},{a}_{\min }^{\text{long }} =  - 2\mathrm{\;m}/{\mathrm{s}}^{2},\left| {a}_{\max }^{\text{lat }}\right|  = {1.2}\mathrm{\;m}/{\mathrm{s}}^{2},{j}_{\min } = \) \( - {0.3m}/{s}^{3} \) and \( {j}_{\max } = {0.3m}/{s}^{3} \) .

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 8206 IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, VOL. 23, NO. 7, JULY 2022-->

<!-- Media -->

<!-- figureText: 60 40 50 60 70 80 Time (s) Long. acceleration Lateral acceleration 40 50 60 70 80 Time (s) Speed (km/h) Reference speed 40 Vehicle speed 20 0 10 20 30 (a) Reference and measured speed Acceleration ( \( \mathrm{m}/{\mathrm{s}}^{2} \) ) 4 Reference long. acceleration 0 0 10 20 30 (b) Longitudinal and lateral accelerations -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_12.jpg?x=166&y=161&w=693&h=612&r=0"/>

Fig. 16. Speed and accelerations during trial with trajectory in Fig. 7: Path 2 with 4 different sets of parameters for speed planning (see Table III).

<!-- Media -->

The speed planning results of both approaches are shown in Fig 17. As can be observed, the implementation of the method in [35] is able compute a speed profile that satisfies all constraints. However, this approach is very conservative and far from being time-optimal when compared with the proposed in this work. Note that the resulting travelling time of the speed profile computed with the method in [35] is \( {95.44s} \) ,while \( {63.46s} \) are obtained with our algorithm. Looking at Fig. 17a, it can be noticed that the approach in [35] (orange line) computes a maximum speed based on the maximum allowed lateral acceleration (at \( s = {163}\mathrm{\;m} \) ) and applies it to the whole path, even if higher speeds can be achieved without exceeding the lateral acceleration limit on other path sections. In Figs. 17b and 17c, it can be seen that this algorithm computes jerk-limited transitions from the initial to the aforementioned speed and, finally, a deceleration section until final speed is reached. In contrast to the method in [35], the approach proposed in this work is able to ensure that constraints are not exceeded while computing a jerk-limited time-optimal speed profile by applying the maximum allowed jerk, accelerations and speed whenever possible.

Besides the limitations described above, it is worth mentioning that, in contrast to [35], the proposed method in this work is able to handle different sets of speed planning parameters for the same path as explained at the end of section V-B.

## E. Computation Time Analysis

To perform a computation time analysis of the proposed speed planning algorithm, a set of 8 long paths with lengths between \( {100}\mathrm{\;m} \) and \( {500}\mathrm{\;m} \) were generated over the test track at the Centre for Automation and Robotics. From these long paths, a big set of shorter paths were automatically generated by splitting each of them into path sections with incremental lengths by \( 1\mathrm{\;m} \) . Thus,a total amount of 423622 paths were generated. Finally, speed profiles were computed for each of the generated paths and the computing times were saved. The speed planning parameters for these tests were: \( {a}_{\max }^{\text{long }} = {1.2}\mathrm{\;m}/{\mathrm{s}}^{2},{a}_{\min }^{\text{long }} =  - 2\mathrm{\;m}/{\mathrm{s}}^{2},\left| {a}_{\max }^{\text{lat }}\right|  = {1.2}\mathrm{\;m}/{\mathrm{s}}^{2} \) , \( {j}_{\max } = {0.5}\mathrm{\;m}/{s}^{3} \) and \( {j}_{\min } =  - {0.5}\mathrm{\;m}/{s}^{3} \) .

<!-- Media -->

<!-- figureText: Speed (km/h) Proposed approach 150 200 250 Displacement, \( s\left( \mathrm{\;m}\right) \) 150 200 250 Displacement, \( s\left( \mathrm{\;m}\right) \) 150 200 250 Displacement, \( s\left( \mathrm{\;m}\right) \) 20 Approach in [35] 10 50 100 (a) Planned speed Long. accel. \( \left( {\mathrm{m}/{\mathrm{s}}^{2}}\right) \) 50 100 (b) Planned acceleration 0.5 Jerk ( \( \mathrm{m}/{\mathrm{s}}^{3} \) ) 0 -0.5 0 50 100 (c) Planned jerk -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_12.jpg?x=940&y=159&w=692&h=882&r=0"/>

Fig. 17. Planned speed, accelerations and jerk with both approaches (blue lines: proposed approach, orange lines: approach in [35]).

<!-- Media -->

As expected, the run time of the speed planning algorithm highly depends on the amount of points of the path. This effect is shown in Fig. 18a, where it can be noticed a proportional relationship between the amount of points in the path and the run time of the jerk-limited speed planning algorithm. In Fig. 18 the microseconds per path point according to the number of points of each path is shown. A mean value of \( {22.71\mu s}/p \) is obtained.

The separation of points of the paths \( \left( {\Delta s}\right) \) used for the this timing analysis is fixed along the paths and is set to \( {\Delta s} = {0.1}\mathrm{\;m} \) . Considering this separation,a mean computation time according to the path length of about \( {0.23}\mathrm{\;{ms}}/\mathrm{m} \) is obtained. Based on these results, and taking into account that the real-time planning architecture in which the proposed speed planning algorithm is integrated typically generates trajectories with path lengths around \( {50}\mathrm{\;m} \) ,an approximated average computing time for the speed planning is \( {11.35}\mathrm{{ms}} \) .

It is important to note that \( {\Delta s} = {0.1m} \) is a generous discretization value for the path points even at low speed scenarios. In most situations \( {\Delta s} = {0.2m} \) ,or even greater values,are assumable. If \( {\Delta s} = {0.2m} \) is adopted,the number of path points is halved and so is the computation time consequently,thus obtaining \( {5.68}\mathrm{\;{ms}} \) for the speed planning computation on a \( {50m} \) length path.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: ARTUÑEDO et al.: JERK-LIMITED TIME-OPTIMAL SPEED PLANNING FOR ARBITRARY PATHS 8207-->

<!-- Media -->

<!-- figureText: 150 2500 3000 3500 4500 (a) Run time according to number of points of the path mean \( = {22.74\mu }\mathrm{s}/\mathrm{p} \) 2500 3000 3500 4500 Number of points along the path (b) Run time per point according to number of points of the path Time (ms) 100 50 500 1000 1500 2000 100 Time per path point \( \left( {\mu \mathrm{s}/\mathrm{p}}\right) \) 80 60 500 1000 1500 2000 -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_13.jpg?x=184&y=158&w=651&h=709&r=0"/>

Fig. 18. Computation time results.

<!-- Media -->

Finally, the results of the proposed jerk-limited speed planning method in terms of computation time were compared with the state of the art approach proposed in [27]. From the experiments reported in [27], a mean computation time of \( {59.2}\mathrm{\;{ms}} \) is needed to obtain the speed profiles of a set of paths with a mean length of \( {9.56m} \) ,estimated from the path shapes used in that work. Based on these values, a mean time according to the path length of \( {6.19}\mathrm{{ms}}/\mathrm{m} \) is obtained. Considering the mean computation time of our method \( \left( {{0.23}\mathrm{\;{ms}}/\mathrm{m}}\right) \) ,a speedup of \( {26.9}\mathrm{x} \) with respect to the approach in [27] is achieved. Nevertheless, note that the CPU used in [27] is a i7-2670QM while we used a i7-3610QE, which is slightly more powerful. Moreover, note that our approach computes a time-optimal solution instead a near-optimal one.

## VII. CONCLUSION

The proposed algorithm is able to compute time-optimal speed profiles for given paths while meeting speed, acceleration and jerk constraints. This method is formulated in the spatial domain so that time-displacement interpolations are not needed. Moreover, optimization algorithms are not involved either, allowing low computational cost of the proposed method. Furthermore, the proposed fallback strategies allows to maintain speed profile continuity in critical driving situations where initial or final conditions cannot be met with the given acceleration and jerk constraints.

An experimental platform has been used to test and validate the proposed approach through different trials in real environments. Moreover, the computing time required by the proposed algorithm has been analysed. The results show the capability of the proposed approach to be used in real automated driving applications.

Future work will focus on increasing the responsiveness of the speed planning in critical driving situations, further fallback strategies and their integration with a high-level decision-making system will be explored. Moreover, motivated by the application of this jerk-limited speed planning method in aggressive driving scenarios, the combined acceleration constraint can be also considered. Finally, an in-depth comparison study of jerk-limited speed planning algorithms for automated driving will be carried out, including MPC-based approaches. REFERENCES

[1] A. Artuñedo, J. Godoy, and J. Villagra, "A decision-making architecture for automated driving without detailed prior maps," in Proc. IEEE Intell. Vehicles Symp. (IV), 2019, pp. 1645-1652, doi: 10.1109/IVS.2019.8814070.

[2] A. Artuñedo, J. Godoy, and J. Villagra, "A primitive comparison for traffic-free path planning," IEEE Access, vol. 6, pp. 28801-28817, 2018, doi: 10.1109/ACCESS.2018.2839884.

[3] A. Artuñedo, J. Villagra, and J. Godoy, "Real-time motion planning approach for automated driving in urban environments," IEEE Access, vol. 7, pp. 180039-180053, 2019, doi: 10.1109/ACCESS.2019.2959432.

[4] M. Babu, Y. Oza, A. K. Singh, K. M. Krishna, and S. Medasani, Model Predictive Control for Autonomous Driving Based on Time Scaled Collision Cone. Piscataway, NJ, USA: Institute of Electrical and Electronics Engineers, 2018, pp. 641-648.

[5] I. Bae, J. Moon, and J. Seo, "Toward a comfortable driving experience for a self-driving shuttle bus," Electronics, vol. 8, no. 9, p. 943, Aug. 2019. [Online]. Available: https://www.mdpi.com/2079- 9292/8/9/943

[6] S. Balasubramanian, A. Melendez-Calderon, and E. Burdet, "A robust and sensitive metric for quantifying movement smoothness," IEEE Trans. Biomed. Eng., vol. 59, no. 8, pp. 2126-2136, Aug. 2012.

[7] I. Batkovic, M. Zanon, M. Ali, and P. Falcone, "Real-time constrained trajectory planning and vehicle control for proactive autonomous driving with road users," in Proc. 18th Eur. Control Conf. (ECC), 2019, pp. 256-262.

[8] H. Bellem, B. Thiel, M. Schrauf, and J. F. Krems, "Comfort in automated driving: An analysis of preferences for different automated driving styles and their dependence on personality traits," Transp. Res. F, Traffic Psychol. Behav., vol. 55, pp. 90-100, May 2018. [Online]. Available: http://www.sciencedirect.com/science/article/pii/S1369847817301535

[9] G. Cesari, G. Schildbach, A. Carvalho, and F. Borrelli, "Scenario model predictive control for lane change assistance and autonomous driving on highways," IEEE Intell. Transp. Syst. Mag., vol. 9, no. 3, pp. 23-35, Aug. 2017.

[10] L. Claussmann, M. Revilloud, D. Gruyer, and S. Glaser, "A review of motion planning for highway autonomous driving," IEEE Trans. Intell. Transp. Syst., vol. 21, no. 5, pp. 1826-1848, May 2020.

[11] J. Godoy, J. Pérez, E. Onieva, J. Villagrá, V. Milanés, and R. Haber, "A driverless vehicle demonstration on motorways and in urban environments," Transport, vol. 30, no. 3, pp. 253-263, 2015, doi: 10.3846/16484142.2014.1003406.

[12] T. Gu, J. M. Dolan, and J.-W. Lee, "On-road trajectory planning for general autonomous driving with enhanced tunability," in Proc. Adv. Intell. Systems and Computing, 2016, pp. 247-261. [Online]. Available: http://link.springer.com/10.1007/978-3-319-08338-4_19

[13] Q. Huang and H. Wang, "Fundamental study of Jerk: Evaluation of shift quality and ride comfort," in Proc. SAE Tech. Papers, May 2004, p. 724. [Online]. Available: https://www.sae.org/content/2004-01-2065/

[14] J. Ji, A. Khajepour, W. W. Melek, and Y. Huang, "Path planning and tracking for vehicle collision avoidance based on model predictive control with multiconstraints," IEEE Trans. Veh. Technol., vol. 66, no. 2, pp. 952-964, Feb. 2017.

[15] D. Kaserer, H. Gattringer, and A. Müller, "Nearly optimal path following with jerk and torque rate limits using dynamic programming," IEEE Trans. Robot., vol. 35, no. 4, pp. 521-528, Apr. 2019.

[16] T. S. Kim, C. Manzie, and R. Sharma, "Model predictive control of velocity and torque split in a parallel hybrid vehicle," in Proc. IEEE Int. Conf. Syst., Man Cybern., Oct. 2009, pp. 2014-2019.

[17] J. Kong, M. Pfeiffer, G. Schildbach, and F. Borrelli, "Kinematic and dynamic vehicle models for autonomous driving control design," in Proc. IEEE Intell. Vehicles Symp. (IV), Jun. 2015, pp. 1094-1099.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->




<!-- Meanless: 8208 IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, VOL. 23, NO. 7, JULY 2022-->

[18] S.-P. Lai, M.-L. Lan, Y.-X. Li, and B. M. Chen, "Safe navigation of quadrotors with jerk limited trajectory," Frontiers Inf. Technol. Electron. Eng., vol. 20, no. 1, pp. 107-119, Jan. 2019.

[19] R. Lattarulo, E. Marti, M. Marcano, J. Matute, and J. Perez, "A speed planner approach based on Bézier curves using vehicle dynamic constrains and passengers comfort," in Proc. IEEE Int. Symp. Circuits Syst. (ISCAS), May 2018, pp. 1-5. [Online]. Available: https://ieeexplore.ieee.org/document/8351307/

[20] X. Li, Z. Sun, D. Cao, Z. He, and Q. Zhu, "Real-time trajectory planning for autonomous urban driving: Framework, algorithms, and verifications," IEEE/ASME Trans. Mechatronics, vol. 21, no. 2, pp. 740-753, Apr. 2016.

[21] T. Lipp and S. Boyd, "Minimum-time speed optimisation over a fixed path," Int. J. Control, vol. 87, no. 6, pp. 1297-1311, Jun. 2014, doi: 10.1080/00207179.2013.875224.

[22] S. Liu, "An on-line reference-trajectory generator for smooth motion of impulse-controlled industrial manipulators," in Proc. Int. Workshop Adv. Motion Control, 2002, pp. 365-370.

[23] M. Nolte, M. Rose, T. Stolte, and M. Maurer, Model Predictive Control Based Trajectory Generation for Autonomous Vehicles-an Architectural Approach. Piscataway, NJ, USA: Institute of Electrical and Electronics Engineers, 2017, pp. 798-805.

[24] B. Paden, M. Cáp, S. Z. Yong, D. Yershov, and E. Frazzoli, "A survey of motion planning and control techniques for self-driving urban vehicles," IEEE Trans. Intell. Veh., vol. 1, no. 1, pp. 33-55, Mar. 2016. [Online]. Available: http://ieeexplore.ieee.org/document/7490340/

[25] S. Perri, C. Guarino Lo Bianco, and M. Locatelli, "Jerk bounded velocity planner for the online management of autonomous vehicles," in Proc. IEEE Int. Conf. Autom. Sci. Eng. (CASE), Aug. 2015, pp. 618-625.

[26] H. Pham and Q.-C. Pham, "On the structure of the time-optimal path parameterization problem with third-order constraints," in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), May 2017, pp. 679-686.

[27] M. Raineri and C. G. L. Bianco, "Jerk limited planner for real-time applications requiring variable velocity bounds," in Proc. IEEE Int. Conf. Autom. Sci. Eng., Aug. 2019, pp. 1611-1617.

[28] P. Reynoso-Mora, W. Chen, and M. Tomizuka, On the Time-Optimal Trajectory Planning and Control of Robotic Manipulators Along Predefined Paths. Piscataway, NJ, USA: Institute of Electrical and Electronics Engineers, 2013, pp. 371-377.

[29] Z. Shiller and S. Dubowsky, "On computing the global time-optimal motions of robotic manipulators in the presence of obstacles," IEEE Trans. Robot. Automat., vol. 7, no. 6, pp. 785-797, Oct. 1991, doi: 10.1109/70.105387.

[30] Z. Shiller and Y.-R. Gwo, "Dynamic motion planning of autonomous vehicles," IEEE Trans. Robot. Autom., vol. 7, no. 2, pp. 241-249, Apr. 1991, doi: 10.1109/70.75906.

[31] C. Sohn, J. Andert, and R. N. Manfouo, "A driveability study on automated longitudinal vehicle control," IEEE Trans. Intell. Transp. Syst., vol. 21, no. 8, pp. 3273-3280, Jun. 2020, doi: 10.1109/TITS.2019.2925193.

[32] J. Suh, H. Chae, and K. Yi, "Stochastic model-predictive control for lane change decision of automated driving vehicles," IEEE Trans. Veh. Technol., vol. 67, no. 6, pp. 4771-4782, Jun. 2018, doi: 10.1109/TVT. 2018.2804891.

[33] J. P. Talamino and A. Sanfeliu, "Anticipatory kinodynamic motion planner for computing the best path and velocity trajectory in autonomous driving," Robot. Auto. Syst., vol. 114, pp. 93-105, Apr. 2019. [Online]. Available: https://www.sciencedirect.com/ science/article/pii/S0921889018301957?via%3Dihub

[34] M. Turki, N. Langlois, and A. Yassine. (2017). An Analytical Tuning of MPC Control Horizon Using the Hessian Condition Number. [Online]. Available: https://hal-normandie-univ.archives-ouvertes.fr/hal-02282128

[35] J. Villagra, V. Milanés, J. Pérez, and J. Godoy, "Smooth path and speed planning for an automated public transport vehicle," Robot. Auton. Syst., vol. 60, no. 2, pp. 252-265, 2012. [Online]. Available: https://www.sciencedirect.com/science/article/pii/S092188901100203X

[36] Y. Zhang et al., "Hybrid trajectory planning for autonomous driving in highly constrained environments," IEEE Access, vol. 6, pp. 32800-32819, 2018. [Online]. Available: https://ieeexplore.ieee.org/document/8375948/

[37] J. Ziegler and C. Stiller, "Spatiotemporal state lattices for fast trajectory planning in dynamic on-road driving scenarios," in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2009, pp. 1879-1884, doi: 10.1109/IROS.2009.5354448.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_14.jpg?x=908&y=173&w=227&h=284&r=0"/>

<!-- Media -->

Antonio Artuñedo received the B.Sc. degree in electrical engineering from the Universidad de Castilla-La Mancha, Spain, in 2011, the M.Sc. degree in industrial engineering from the Uni-versidad Carlos III de Madrid in 2014, and the Ph.D. degree in automation and robotics from the Technical University of Madrid (UPM), Spain, in 2019, with the AUTOPIA Program. During his Pre-Doctoral period, he made a research stay at the Integrated Vehicle Safety Group at TNO, The Netherlands, in 2017. He is currently a Post-Doctoral Researcher with the Centre for Automation and Robotics (CSIC-UPM), AUTOPIA Group, Madrid, Spain, which he joined in 2013. He has been working on both national and European research projects in the scope of autonomous vehicles. He has published and peer-reviewed multiple journals and conference papers focused in this field. His research interests include system modeling and simulation, intelligent control, motion planning, and decision-making systems. His thesis, awarded with the distinction "Cum Laude" and the International Mention, won the following prizes, such as the Extraordinary Ph.D. Award of the Technical University of Madrid, the Best Ph.D. Thesis on Intelligent Transportation Systems 2020 by the Spanish Chapter of the IEEE-ITS Society, and the IDOM Prize for Best Ph.D. Thesis on Intelligent Control 2020, and the Finalist of the Robotnik Prize for the Best Ph.D. Thesis on Robotics 2020 by the Spanish Committee of Automatics.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_14.jpg?x=909&y=914&w=226&h=283&r=0"/>

<!-- Media -->

Jorge Villagra graduated in industrial engineering from the Universidad Politécnica of Madrid in 2002. He received the Ph.D. degree in real-time computer science, robotics and automatic control from the École des Mines de Paris, France, in 2006. From 2007 to 2009, he was a Visiting Professor with the University Carlos III of Madrid, Spain. From August 2013 to August 2016, he led the Department of ADAS and Highly Automated Driving Systems at Ixion Industry and Aerospace SL, where he also coordinated all the activities in the EU Research and Development funding programmes. He has been leading AUTOPIA Program at CSIC, since October 2016. He has developed his research activity in six different entities with a very intense activity in project setup and management, through over 30 international and national Research and Development projects, where he is or has been IP of ten of these projects. He has published over 85 articles in international journals and over 85 papers in international conferences on autonomous driving, intelligent transportation systems, dmodel-free control, and new probabilistic approaches for embedded components in autonomous vehicles. He was first granted with a three years CIFRE Program in PSA-Peugeot-Citroën and then with a PostDoctoral Fellowship with the Joint Research Unit INRIA-Mines ParisTech, France. The results of the Ph.D. were granted with the Prize for the Best Dissertation in Automatic Control in France, in 2006. He, then received the three year JAEDoc Fellowship at the AUTOPÍA Program from the Center for Automation and Robotics UPM-CSIC, Spain, where he spent one additional year funded by a research contract.

<!-- Media -->

<img src="https://cdn.noedgeai.com/bo_d2slc0v7aajc738sf8k0_14.jpg?x=911&y=1732&w=222&h=280&r=0"/>

<!-- Media -->

Jorge Godoy was born in Maracay, Venezuela, in 1986. He received the degree in electronics engineering from Universidad Simón Bolívar in 2008, and the M.E. and Ph.D. degrees in automation and robotics from the Universidad Politécnica de Madrid in 2011 and 2013, respectively. From 2013 to 2017, he was the Technical Coordinator of the AUTOPIA Program funded by research contracts from National and European research projects. His research interests include intelligent transportation systems, autonomous driving, path planning, and embedded AI-based control for autonomous vehicles. In 2009, he was granted with a Pre-Doctoral JAE Fellowship from CSIC for researching on autonomous vehicles at the Centre of Automation and Robotics (UPM-CSIC). In November 2017, he was granted with a Juan de la Cierva Fellowship for Post-Doctoral Research at the Universidad Politécnica de Madrid.

<!-- Meanless: Authorized licensed use limited to: TIANJIN UNIVERSITY. Downloaded on November 07,2024 at 06:16:44 UTC from IEEE Xplore. Restrictions apply.-->

