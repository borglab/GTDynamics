/** The formulation follows the two native GTDynamics IK paths in ik_backend.py. */
export const urdfPaper = {
  abstract: 'Inverse kinematics finds joint coordinates that move a robot’s tool to a requested position or pose. This demo compares GTDynamics’ factor graph, which couples link poses and joint coordinates, with four joint-space solvers on the same URDF model.',
  motivation: 'A tool target may admit several joint configurations or lie beyond the robot’s reach. Joint-space methods use forward kinematics to evaluate candidate configurations. A factor graph estimates link poses alongside the joint coordinates, which exposes how the target, joint geometry, and priors compete.',
  approach: 'The GTDynamics scheme solves one configuration at a time with Levenberg–Marquardt, starting from the current joint coordinates and their forward kinematics. The selected end-effector frame may have a fixed offset from its carrier link. The diagram shows one joint and the factors for the selected goal type.',
  graph: 'urdf-pose',
  factors: [
    { id: 'kinematics', name: 'K · Articulated joint geometry', description: 'A PoseFactor connects the parent and child world-frame centre-of-mass (COM) poses to their joint coordinate. Its six-dimensional pose residual compares the relative COM transform with the one predicted by the URDF joint. Full-pose IK also includes the joint equality residual as a penalty in the default optimizer.' },
    { id: 'base', name: 'B · Fixed-link anchor', description: 'A unary residual compares a fixed link’s COM pose with its rest pose and anchors the world frame. Position-only IK uses a Gaussian pose prior. Full-pose IK adds the fixed-link equality residual as a penalty.' },
    { id: 'pose', name: 'T · Tool pose target · full-pose mode', description: 'A unary Pose3 prior on the tool’s carrier-link pose compares it with the desired world-frame tool pose composed with the inverse of the fixed COM-to-tool transform. Its tangent-space residual includes rotation and translation.' },
    { id: 'point', name: 'P · Tool position target · position-only mode', description: 'A unary PointGoalFactor transforms the tool origin from the carrier COM frame into world coordinates and subtracts the requested world point in metres. The isotropic position standard deviation is 0.1 mm.' },
    { id: 'prior', name: 'Q · Joint prior', description: 'A unary Gaussian prior subtracts the warm-start coordinate from the estimated joint coordinate. The default standard deviation is 0.5 in joint units, radians for revolute joints and metres for prismatic joints.' },
    { id: 'limit', name: 'L · Joint limits · full-pose mode', description: 'A unary hinge penalizes excursions outside the URDF interval reduced by a 0.04 margin in joint units. The finite weight permits violations. The position-only graph omits this factor.' },
  ],
  discussion: 'The returned joint coordinates drive forward kinematics for the displayed tool pose. The graph’s link poses can disagree with that reconstruction because the solver trades joint geometry against target error. Local minima, singular configurations, and joint bounds can all limit convergence. Contact Labels uses a discrete factor graph for a different robotics inference problem.',
  next: 'contact-labels', nextTitle: 'Contact Labels',
};
