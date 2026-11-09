import { scatePaper as scateUr5Paper } from '../../scate/ur5/src/paper.js';
import { scatePaper as scateUr10ePaper } from '../../scate/ur10e/src/paper.js';
import { urdfPaper } from '../../urdf/src/paper.js';
// Presentation only: numerical settings and results remain in the recordings.
const contact = {
  id: 'contact', name: 'Contact-pose observation',
  description: 'A ContactPoseFactor connects a root or foot-link pose to one world-frame sole-pose landmark per uninterrupted stance. Its six-dimensional residual compares the predicted root-relative or link-relative sole pose with the measured transform. Translation and rotation have separate noise scales. Lift-off ends a landmark; recontact creates another.',
};
const upright = {
  id: 'upright', name: 'Upright prior · optional',
  description: 'A root-orientation prior penalizes pelvis tilt relative to the world vertical. It leaves heading free. The rotation standard deviation, specified in degrees, sets the strength of this modeling assumption.',
};
const height = {
  id: 'height', name: 'Contact-height prior · optional',
  description: 'A scalar ContactHeightFactor on each sole landmark penalizes its vertical distance from the selected ground height. Horizontal position and yaw remain free. The Gaussian prior assigns a finite cost to height errors, including penetration below the ground.',
};
const motion = {
  id: 'motion', name: 'Weak motion prior · optional in initialization',
  description: 'A BetweenFactor<Pose3> with identity relative motion weakly discourages large changes between consecutive root poses. It regularizes poorly constrained contact chains. Inertial measurements enter through separate factors when enabled.',
};
const normal = {
  id: 'normal', name: 'Sole-normal alignment · sequence dependent',
  description: 'A ground-normal objective aligns the sole normal with the world vertical while leaving yaw free. This prior is available for the non-SSIO recordings; the SSIO recordings instead expose the pelvis-upright prior. The saved graph identifies which ground objectives are present.',
};
const kinematics = {
  id: 'kinematics', name: 'Articulated kinematic equalities',
  description: 'Within each slice, link COM poses and joint angles must satisfy the model’s joint transforms. CM-Opt enforces these as equality constraints. Their violation is reported separately from the sum of soft factor errors.',
};
const encoders = {
  id: 'encoders', name: 'Joint-angle measurements',
  description: 'Unary priors compare each estimated joint angle with its measured encoder angle. These soft costs permit joint corrections while the articulated kinematic relationships remain exact.',
};
const imu = {
  id: 'imu', name: 'Gal3 IMU preintegration · optional',
  description: 'A GalileanImuFactor connects root COM poses Xₖ and Xₖ₊₁, their world-frame linear velocities vₖ and vₖ₊₁, and one shared accelerometer/gyroscope bias b. Each pim integrates measurements over the actual irregular interval. The COM-to-IMU transform and gravity direction determine the measurement frame; propagated covariance weights the nine-dimensional residual.',
};
const inertialPriors = {
  id: 'inertial-priors', name: 'Velocity and shared-bias priors · with IMU',
  description: 'A prior on the first world-frame COM velocity anchors the velocity chain. One ConstantBias prior regularizes the shared sensor-axis bias; the gyro mean comes from the initial stationary interval. The bias remains constant throughout the recording. Noise assumptions are recorded below with the numerical results.',
};
const loop = {
  id: 'loop', name: 'Final-position prior · optional',
  description: 'PoseTranslationPrior3D places the final root COM near the start anchor, with position standard deviations of 30 cm horizontally and 5 cm vertically. Orientation remains free, and the graph retains its chain structure. The measurement expresses an assumed return to the starting position.',
};
const gauge = {
  id: 'gauge', name: 'World-frame gauge',
  description: 'The first root pose fixes the otherwise arbitrary world frame. Initialization fixes that pose; smoothing uses a tight first-root prior. Subsequent LiDAR-inertial reference poses are reserved for evaluation.',
};

export const papers = {
  urdf: urdfPaper,
  'scate/ur5': { ...scateUr5Paper, next: 'scate/ur10e', nextTitle: 'SCATE UR10e' },
  'scate/ur10e': scateUr10ePaper,
  'contact-labels': {
    abstract: 'This app infers left, right, and double support from G1 foot motion using a discrete factor graph. Max-product combines displacement and joint-velocity evidence with walking priors; the resulting phases supply the contact schedule for initialization. For the default 24–44 s clip with velocity aiding and walking constraints, recorded max-product inference took 0.113 s.',
    motivation: 'A velocity threshold alone can produce isolated labels or contradictory support assignments. Contact timing matters because it determines which observations share a stationary foothold. Combining local measurements with temporal and cross-foot assumptions makes that timing explicit and inspectable.',
    approach: 'At each 50 Hz slice k, each foot has its own binary stance/swing variable. Measurement potentials and discrete priors multiply, and exact max-product returns the joint MAP sequence. The alternative window model labels stationarity from translation and rotation. These are privileged labels: SuperOdom LiDAR-inertial poses contribute to world-frame foot motion.',
    graph: 'labels',
    factors: [
      { id: 'displacement', name: 'Per-foot binary displacement measurement', description: 'A factor on c_f,k and c_f,k+1 observes two incoming translation differences, d_f,k and d_f,k+1, each obtained from successive world-frame sole origins. A narrow half-normal density favors stance and a broad density permits swing. Endpoint weights avoid counting the same observation twice. The implemented potential is separable; temporal dependence comes from the other factors.' },
      { id: 'velocity', name: 'Per-foot unary joint-velocity measurement · optional', description: 'The sole’s forward velocity relative to the pelvis, computed from measured joint velocities, gives stance/swing weights through a bounded logistic potential. Backward motion favors stance; forward motion favors swing. Each factor is agnostic about the other foot. This heuristic is most appropriate for forward walking.' },
      { id: 'support', name: 'Within-slice support coupling', description: 'A binary factor on the left and right variables penalizes simultaneous swing while remaining neutral between the three supported configurations. Its soft form permits both-swing assignments when other evidence dominates.' },
      { id: 'transition', name: 'Four-variable transition prior', description: 'One factor connects both feet at slices k and k+1. Its table assigns relative weights to the sixteen possible transitions between joint configurations (WW, WS, SW, SS); W means swing and S means stance, with the left foot listed first.' },
      { id: 'duration', name: 'Per-foot three-slice duration prior', description: 'A factor on three consecutive labels penalizes an isolated one-slice stance or swing event. At 50 Hz this discourages 20 ms flicker. Short phases remain possible when measurement evidence outweighs the finite penalty.' },
      { id: 'walking', name: 'Hard walking constraints · optional, enabled by default', description: 'Additional zero/one factors forbid simultaneous swing and forbid direct left-only ↔ right-only exchanges. A support exchange must pass through at least one double-stance slice. These constraints multiply with the existing soft factors, whose tables remain unchanged.' },
      { id: 'initial', name: 'Initial configuration prior', description: 'A uniform prior on the first left/right pair supplies no preference among the four joint configurations. Context samples outside the displayed snippet reduce boundary effects.' },
    ],
    discussion: 'The inferred support sequence depends on the motion evidence and walking assumptions. Agreement with the window model measures consistency between two inference methods; validating contact accuracy would require independent contact measurements. Contact Initialization uses these phases to connect root poses through shared sole-pose landmarks.',
    next: 'contact-initialization', nextTitle: 'Contact Initialization',
  },
  'contact-initialization': {
    abstract: 'Given contact phases, this app initializes a walking trajectory with a continuous pose factor graph. Root-relative sole observations connect pelvis poses to shared stance landmarks; independently selectable priors show how regularization affects the estimate. For the default 24–44 s clip with motion, height, and upright priors, native graph construction and optimization took a median 36.49 ms over eleven timed runs, excluding dense replay expansion.',
    motivation: 'A stance foot provides observations of the same foothold from multiple root poses. A shared landmark couples those poses and can initialize global motion from contact and encoder observations. Weakly constrained directions still need a gauge and carefully chosen priors.',
    approach: 'Measured joint configurations provide root-relative sole poses. The graph estimates one root pose at each retained slice and one world-frame contact pose per uninterrupted stance. Short double-stance phases contribute a central slice; long phases retain buffered endpoints and evenly distributed slices with at most 200 ms between them. The browser selects results from recorded native solves.',
    graph: 'initialization',
    factors: [contact, gauge, motion, height, upright, normal],
    discussion: 'Trajectory accuracy depends on contact timing, sole rocking, pose noise, and the chosen priors. Contact residuals report how well the estimate fits those assumptions. Contact vs IK compares this joint contact initialization with independent inverse kinematics before applying the same smoothing graph.',
    next: 'contact-vs-ik', nextTitle: 'Contact vs IK',
  },
  'contact-vs-ik': {
    abstract: 'Contact and inverse-kinematics initializers feed the same G1 smoothing factor graph. Recorded iterations, constraint violations, and residuals on identical walking snippets show how initialization affects convergence. On the recorded 204–214 s turn with IMU aiding, smoothing took 0.389 s from contact initialization and 1.066 s from IK, including diagnostic and replay capture but excluding initialization and graph construction.',
    motivation: 'A nonlinear smoother can depend strongly on its starting point. Per-slice IK is simple, while shared contact landmarks couple observations over time. Comparing both on identical slices and objectives isolates the effect of initialization.',
    approach: 'The contact initializer jointly estimates root and stance poses; IK solves retained slices independently. CM-Opt then smooths both trajectories using exact articulated kinematics, encoder measurements, and stance landmarks. The IMU switch selects an inertially aided graph. The two viewers share time selection and expose their own recorded optimizer iterations.',
    graph: 'smoothing',
    factors: [kinematics, encoders, contact, gauge, imu, inertialPriors,
      { ...motion, name: 'Contact-initializer motion prior', description: `${motion.description} This prior is active only during initialization.` },
      { ...height, name: 'Contact-initializer height prior' },
      { ...upright, name: 'Contact-initializer upright prior' },
      { id: 'ik', name: 'IK initialization objectives', description: 'Each retained slice uses a nominal root-pose prior, joint-angle measurements, articulated kinematic factors, active sole-height and sole-normal objectives, and a foot-link pose prior toward a stance target frozen at its first observation. Root guesses follow a commanded nominal path. The IK stance targets remain fixed at their input values. The subsequent smoother estimates shared landmarks for both methods.' },
    ],
    discussion: 'Trajectory quality is assessed through equality violation, physical residuals, reference disagreement, and stopping conditions. The objective measures the fit of the weighted factors. Kinematics extends contact-initialized smoothing to the full recording and compares prior combinations.',
    next: 'kinematics-inspector', nextTitle: 'Kinematics',
  },
  'kinematics-inspector': {
    abstract: 'This app estimates the full G1 trajectory with a factor graph combining articulated kinematics, shared stance poses, and optional IMU aiding. Recorded prior combinations expose convergence, contact slip, and disagreement with the LiDAR-inertial reference. For the full 220 s clip with IMU aiding and loop closure, the recorded solve took 105.83 s for 715 slices and reached the 50-iteration limit. This includes manifold setup and factor-error recording, but excludes initialization, graph construction, and replay export.',
    motivation: 'Contact and inertial observations constrain motion across multiple slices. Smoothing combines this evidence over the trajectory while representing sensor uncertainty and enforcing exact robot kinematics.',
    approach: 'Initialization and smoothing use exactly the same contact-selected slices, with a default maximum spacing of 200 ms inside long double-stance phases. ContactPoseFactor observations share landmarks across each stance. Gal3 pims span the actual irregular intervals. CM-Opt enforces articulated kinematics. Interpolation is confined to playback, and reference alignment changes evaluation and overlays only.',
    graph: 'smoothing',
    factors: [kinematics, encoders, contact, gauge, imu, inertialPriors, loop, upright, height],
    discussion: 'SuperOdom LiDAR-inertial odometry supplies the reference trajectory and privileged motion used for contact inference. IMU noise scales are chosen modeling parameters whose calibration remains to be established. Error budgets, stopping conditions, and aligned position error describe different aspects of the solution. Interpolation fits velocities using this saved kinematic trajectory and dense encoder measurements before Dynamics estimates accelerations and forces.',
    next: 'interpolation', nextTitle: 'Interpolation',
  },
  dynamics: {
    abstract: 'This app estimates accelerations, ground reaction wrenches, and joint torques from GP-fitted G1 motion over 78–98 s. A constrained factor graph combines native link dynamics with one wrench per sole and acceleration observations. For the 57 recorded knots, native graph construction and solving took 0.745 s; GP graph construction and solving took another 0.177 s, excluding data loading, FK sampling, and export.',
    motivation: 'Inverse dynamics needs both motion derivatives and a contact-force allocation. Double support leaves that allocation underdetermined. A factor graph combines the dynamics equations with contact-wrench feasibility and explicit effort and temporal regularization.',
    approach: 'The full-trajectory Gaussian-process fit supplies configurations and velocities, fixed using ConstVarFactor. The unknowns are link body-twist derivatives, joint accelerations, internal link wrenches, actuator torques, and sole-frame contact wrenches. Acceleration observations come from local differences of GP velocities. CMC-Opt enforces exact articulated acceleration kinematics, force/moment balance, and linear wrench inequalities using the affine ReducedAL retraction. The recording retains the original double-support knots; intervening single-support intervals remain unsampled.',
    graph: 'dynamics',
    factors: [
      { id: 'fixed', name: 'Fixed configurations and velocities', description: 'ConstVarFactor substitutes the saved knot poses, joint angles, GP-fitted body twists, and joint rates. The fixed inputs determine the coefficients of the affine dynamics equations. Accelerations remain adjustable variables.' },
      { id: 'acceleration', name: 'Root and joint acceleration observations', description: 'Soft priors compare root body-twist derivatives and joint accelerations with local velocity differences of the GP fit, using its full-trajectory context. Default standard deviations are 90 °/s² for root angular components, 3 m/s² for root linear components, and 100 °/s² for revolute-joint accelerations. Body-twist derivatives use moving COM axes; physical linear acceleration also includes the angular-velocity cross linear-velocity term.' },
      { id: 'acceleration-kinematics', name: 'Articulated acceleration equalities', description: 'TwistAccelFactor relates parent/child body-twist derivatives and joint accelerations using the fixed configuration and velocity inputs. These exact constraints ensure that every estimated link acceleration belongs to the same articulated motion.' },
      { id: 'balance', name: 'Link force and moment balance', description: 'Native Newton–Euler factors relate each link’s inertia, gravity, estimated acceleration, joint wrenches, and external contact wrench. The published recording enforces exact balance for all links, including the floating base. An explicit soft-base diagnostic mode permits unexplained residual force and moment; its noise scales are recorded with that run.' },
      { id: 'joint', name: 'Joint wrench and actuator-torque constraints', description: 'Adjacent link wrenches satisfy action/reaction transformations. Torque projection relates the joint-axis moment to actuator torque, including configured armature. These equalities use link COM frames and the model’s joint-axis sign convention.' },
      { id: 'contact-map', name: 'Sole-to-link wrench transformation', description: 'ContactWrenchFactor maps the sole-frame external wrench to the foot-link COM with the dual adjoint, including the moment lever arm. Wrenches are ordered [moment; force], in Nm and N; the force is ground-on-robot.' },
      { id: 'cone', name: 'Rectangular contact-wrench inequalities', description: 'Linear inequalities constrain tangential friction, center of pressure, and coupled yaw moment for a rectangular sole patch. An explicit nonnegative normal-force row is redundant but retained. Swing contact wrenches are fixed to zero. Patch dimensions and the friction-pyramid convention are recorded in the settings.' },
      { id: 'effort', name: 'Effort and wrench regularization', description: 'Zero-centered soft priors penalize actuator effort and sole-wrench magnitude, choosing among otherwise feasible double-support force allocations. Moment scales include a sole length scale. Published actuator effort limits are diagnostic unless the recording explicitly enables hard bounds.' },
      { id: 'rate', name: 'Torque and wrench temporal regularization', description: 'Between factors penalize changes between retained samples, with standard deviations scaled by the square root of elapsed time. Wrench-rate factors connect only observations in the same uninterrupted stance; the full source phase sequence preserves lift-off and recontact across unsampled intervals. Torque-rate factors connect consecutive retained knots.' },
    ],
    discussion: 'The estimated forces and torques depend on fixed GP-fitted poses and velocities, estimated accelerations, and the contact schedule. Exact dynamics and wrench feasibility permit acceleration corrections relative to the GP-derived priors. Residual stance-foot speed in the fixed velocity fit still limits the rigid-contact interpretation. Agreement with simulator equations checks the dynamics calculation for these inputs; independent force and torque measurements would support physical validation. Kinodynamics next estimates motion and forces together at every original 50 Hz slice.',
    next: 'kinodynamics', nextTitle: 'Kinodynamics',
  },
  kinodynamics: {
    abstract: 'This app jointly estimates G1 motion, velocities, accelerations, sole wrenches, and joint torques on sparse single- and double-support slices of the 78–98 s turns. A constrained factor graph couples contact and inertial observations to articulated mechanics and temporal collocation, starting from contact-pose initialization, a floating-base GP velocity fit, and an affine linear dynamics seed. Contact-only and fixed-pelvis FK initializations remain available for comparison. The recording reports its initialization strategy, timing and stopping condition.',
    motivation: 'A sequential pipeline fixes earlier motion estimates before calculating forces, so the dynamics cannot correct those estimates. A coupled trajectory solve allows the motion and contact-force allocation to adjust together. The recorded stage timings separate sequential initialization from the coupled optimization.',
    approach: 'Each retained support slice contains free link COM poses, body twists and their derivatives, joint angles/rates/accelerations, reaction wrenches, actuator torques, and sole wrenches. Shared stance-pose landmarks and one IMU bias connect slices. CMC-Opt enforces articulated mechanics and contact-wrench feasibility exactly; encoder/contact/IMU observations, trapezoidal collocation, and regularization form the soft objective. The default contact initializer fits selected single- and double-support slices, using a central slice in short phases and buffered slices at most 200 ms apart inside long phases, then fits GP velocities on those same knots and obtains accelerations, wrenches and torques from one affine Gaussian dynamics solve. Stance wrenches start with positive normal gravity support; exact mechanics determine the remaining dynamics variables. Initialization performs no dynamics manifold construction or iterative optimization. The contact-only alternative supplies FK configurations at the retained knots with zero motion and forces. The optional FK seed uses measured joint angles at a fixed upright pelvis and shares gravity-support forces across stance feet. All initialization strategies use the same optimization graph. Each constraint-connected component eliminates its linearized equalities through B; GTSAM or the optional ProxQP backend solves the coupled reduced QP with active boundary inequalities, followed by component-local nonlinear retractions. Unit-standard-deviation translation and wrapped-yaw costs choose the world frame at the first selected supported slice. All six root-pose coordinates remain adjustable; gravity and IMU observations estimate roll and pitch.',
    graph: 'kinodynamics',
    factors: [
      { ...kinematics, name: 'Articulated pose and velocity equalities', description: 'Pose constraints connect parent/child COM poses and their joint angle; TwistFactor connects their body twists and joint rate at each slice. The canonical articulated model determines the transforms. Both are exact constraints, with configuration and velocity variables free.' },
      encoders, contact,
      { ...imu, name: 'Gal3 IMU preintegration', description: 'A GalileanImuFactor connects two root COM poses, their world-frame COM linear velocities, and one shared sensor-axis bias. Each pim integrates the native IMU samples over the actual interval between retained support slices. Its propagated covariance weights the nine-dimensional residual; gravity and the recorded COM-to-IMU transform determine the frames.' },
      { ...inertialPriors, name: 'Initial world velocity and shared-bias priors', description: 'A zero-mean prior regularizes the first world-frame COM velocity. The constant IMU bias has one accelerometer/gyroscope prior; the gyro prior mean comes from the stationary interval. These are soft measurements. The default contact → GP → linear dynamics seed pipeline supplies estimated motion, forces and torques. The shared bias starts at zero. Contact-only initialization starts motion and dynamics at zero.' },
      { id: 'world-velocity', name: 'Body-to-world COM velocity equality', description: 'WorldVelocityFactor connects the root pose, root body twist [ω; u], and world linear velocity v. Its residual is R u − v, using the COM-to-world rotation R. This exact equality lets the IMU and articulated mechanics share the same physical velocity.' },
      { id: 'collocation', name: 'Configuration and velocity collocation', description: 'Four-variable trapezoidal factors connect root poses/body twists at adjacent slices and connect root twists/their body-coordinate derivatives. Scalar trapezoidal factors perform the same integration for every joint angle/rate and rate/acceleration. These are finite-noise temporal costs, with intervals from the original timestamps.' },
      { id: 'stance-velocity', name: 'Stance sole-origin velocity objective', description: 'ContactKinematicsTwistFactor penalizes the three-dimensional linear velocity of each active sole origin, transformed from its foot-link COM body twist. Its zero target allows finite slip; it does not constrain sole angular velocity.' },
      { id: 'acceleration-kinematics', name: 'Articulated acceleration equalities', description: 'TwistAccelFactor connects parent/child body-twist derivatives and joint acceleration, with their estimated configurations and velocities. These exact constraints include the velocity-dependent terms and keep all link accelerations consistent with one articulated motion.' },
      { id: 'balance', name: 'Newton–Euler, reaction-wrench and motor-torque equalities', description: 'Each link satisfies exact force/moment balance under inertia, gravity and contact/joint wrenches. Adjacent reaction wrenches obey action/reaction transforms. The motor torque equals the joint-axis projection of the child reaction wrench plus rotor armature times joint acceleration. All wrench transforms use COM frames and the model joint-axis convention.' },
      { id: 'contact-map', name: 'Sole-to-COM wrench equality', description: 'ContactWrenchFactor maps each ground-on-robot sole wrench to its foot-link COM through the dual adjoint, including the moment lever arm. Wrenches are [moment; force] in Nm and N. Each canonical sole frame has +x forward, +y left, +z up.' },
      { id: 'cone', name: 'Rectangular sole-wrench constraints', description: 'Seventeen linear inequalities per stance sole constrain nonnegative normal force, tangential friction, center of pressure, and coupled yaw moment within its support rectangle. The friction pyramid is inscribed in the circular Coulomb cone. Swing wrenches are exactly zero. Actuator effort limits are reported as diagnostics.' },
      { id: 'effort', name: 'Effort, wrench and acceleration regularization', description: 'Zero-centered unary Gaussian costs penalize actuator torques, sole-wrench magnitudes, root body-twist derivatives and joint accelerations. Wrench moments use the square root of patch half-length times half-width as a characteristic length. Broad acceleration priors regularize the coupled solve without importing GP-derived derivatives.' },
      { id: 'rate', name: 'Torque and same-stance wrench rate objectives', description: 'Between factors penalize changes of actuator torque at consecutive slices and of sole wrench during an uninterrupted stance. Standard deviations scale with the square root of elapsed time. Lift-off ends a wrench-rate chain; recontact starts another.' },
      { id: 'gauge', name: 'Translation and yaw gauge costs', description: 'At the first contact-initialization slice, PoseTranslationPrior and a wrapped-yaw factor penalize displacement from the initial translation and heading, with standard deviations of 1 m and approximately 57.3°. Roll and pitch remain free for gravity and IMU observations to estimate. The initializer supplies a pose guess; its upright and height priors are not observations in the coupled solve. Later SuperOdom poses are reserved for evaluation.' },
    ],
    discussion: 'The recorded run stops at its configured iteration limit, so its feasible mechanics and reduced objective describe an intermediate coupled estimate. Force and torque accuracy depends on the contact labels, sensor assumptions, collocation weights and canonical robot model. Contact labels use privileged SuperOdom motion; independent force and torque measurements would be needed for physical validation. Motion Planning applies related factor-graph components to a separate Franka trajectory problem.',
    next: 'motion-planning', nextTitle: 'Motion Planning',
  },
  'motion-planning': {
    abstract: 'This app optimizes a Franka arm trajectory between two hand poses using a kinematic factor graph. Robot states and temporal factors expose how the same GTDynamics formulation can describe desired motion after the estimation examples. The recorded 12-state problem took 25.0 ms for graph construction and eight optimizer iterations, including snapshot serialization and IPC but excluding process startup, model loading, and browser playback delays.',
    motivation: 'A motion plan must satisfy requested hand poses while keeping joint angles and velocities consistent over time. Trajectory optimization expresses these requirements as factors and uses regularization to resolve redundant arm motion.',
    approach: 'Each slice contains joint angles q, joint velocities q̇, link COM poses T, and link twists V. Within-slice factors express articulated kinematics; temporal factors discretize integration and smooth joint velocity. GTSAM Levenberg–Marquardt minimizes their weighted residuals. The browser replays a recorded native solve; a local backend enables endpoint editing and new solves.',
    graph: 'planning',
    factors: [
      { id: 'kinematics', name: 'Position and velocity kinematics', description: 'qFactors and vFactors relate joint coordinates and rates to link COM poses and twists, including the fixed-base relationships. This LM example represents those relationships as strongly weighted soft factors.' },
      { id: 'endpoint', name: 'Endpoint hand-pose priors', description: 'Pose3 priors fit the start and goal hand frames. Translation and rotation residuals are tightly weighted; displayed endpoint residuals report metres and degrees independently.' },
      { id: 'rest', name: 'Endpoint rest priors', description: 'Unary joint-velocity priors impose near-zero velocity at the first and last slices, encouraging the arm to start and finish at rest.' },
      { id: 'joint-integration', name: 'Joint trapezoidal integration', description: 'A four-variable factor penalizes qₖ₊₁ − qₖ − Δt(q̇ₖ + q̇ₖ₊₁)/2, coupling joint angles and velocities at consecutive slices.' },
      { id: 'pose-integration', name: 'Pose trapezoidal integration', description: 'A four-variable pose/twist factor compares the relative link COM pose with the motion predicted by the two endpoint body twists, transporting them into a common tangent frame.' },
      { id: 'smoothness', name: 'Joint-velocity smoothness', description: 'Between factors penalize consecutive joint-velocity differences with variance proportional to Δt. The summed cost approximates an integral of squared joint acceleration.' },
      { id: 'posture', name: 'Weak posture priors', description: 'Joint-angle priors toward the home configuration select among redundant arm solutions. Their strength scales with the number of slices so changing discretization does not simply multiply the posture preference.' },
      { id: 'limits', name: 'Joint-angle limit costs', description: 'Limit factors penalize angles outside the configured joint range. Collision avoidance, wrench feasibility, and full dynamic trajectory optimization remain extensions to this kinematic example.' },
    ],
    discussion: 'MuJoCo tracks the optimized kinematic trajectory with position control, which introduces execution error. The Franka problem uses its own robot model and endpoint targets. Extending the G1 estimation pipeline toward dynamic motion planning requires contact consistency, actuator limits, and collision constraints in the optimization problem.',
    next: 'scate/ur5', nextTitle: 'SCATE UR5',
  },
};
