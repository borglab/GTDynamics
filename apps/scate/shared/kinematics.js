/**
 * Forward and inverse kinematics of a live-app robot description
 * (scate_live describe): link frames at rest and the joints' space-frame
 * screw axes, posed by the product of exponentials
 * T_l(q) = exp(S_1 q_1) ... exp(S_l q_l) T_l(0). Matrices are 4x4 arrays of
 * 16 numbers, row-major. No three.js, so it runs in tests too.
 */

/** 4x4 identity. */
export const identity = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

/** a * b for 4x4 rigid transforms. */
export function multiply(a, b) {
  const out = new Array(16);
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      out[4 * r + c] = a[4 * r] * b[c] + a[4 * r + 1] * b[4 + c] + a[4 * r + 2] * b[8 + c] + a[4 * r + 3] * b[12 + c];
    }
  }
  return out;
}

/** Transform of a [x, y, z, qx, qy, qz, qw] pose. */
export function poseMatrix([x, y, z, qx, qy, qz, qw]) {
  return [
    1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw), x,
    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw), y,
    2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy), z,
    0, 0, 0, 1,
  ];
}

/** The [x, y, z, qx, qy, qz, qw] pose of a transform. */
export function matrixPose(m) {
  const trace = m[0] + m[5] + m[10];
  let qx, qy, qz, qw;
  if (trace > 0) {
    const s = 2 * Math.sqrt(trace + 1);
    qw = s / 4; qx = (m[9] - m[6]) / s; qy = (m[2] - m[8]) / s; qz = (m[4] - m[1]) / s;
  } else if (m[0] > m[5] && m[0] > m[10]) {
    const s = 2 * Math.sqrt(1 + m[0] - m[5] - m[10]);
    qw = (m[9] - m[6]) / s; qx = s / 4; qy = (m[1] + m[4]) / s; qz = (m[2] + m[8]) / s;
  } else if (m[5] > m[10]) {
    const s = 2 * Math.sqrt(1 + m[5] - m[0] - m[10]);
    qw = (m[2] - m[8]) / s; qx = (m[1] + m[4]) / s; qy = s / 4; qz = (m[6] + m[9]) / s;
  } else {
    const s = 2 * Math.sqrt(1 + m[10] - m[0] - m[5]);
    qw = (m[4] - m[1]) / s; qx = (m[2] + m[8]) / s; qy = (m[6] + m[9]) / s; qz = s / 4;
  }
  return [m[3], m[7], m[11], qx, qy, qz, qw];
}

/**
 * exp of the twist screw * theta, screw = [w; v] with |w| = 1 (a revolute
 * joint): rotation by theta about w, and the translation of the matching
 * screw motion.
 */
export function screwExp(screw, theta) {
  const [wx, wy, wz, vx, vy, vz] = screw;
  const s = Math.sin(theta), c = 1 - Math.cos(theta);
  // R = I + sin W + (1 - cos) W^2, with W the cross-product matrix of w.
  const W = [0, -wz, wy, wz, 0, -wx, -wy, wx, 0];
  const W2 = [
    -(wy * wy + wz * wz), wx * wy, wx * wz,
    wx * wy, -(wx * wx + wz * wz), wy * wz,
    wx * wz, wy * wz, -(wx * wx + wy * wy)];
  const R = [0, 1, 2, 3, 4, 5, 6, 7, 8].map(i => (i % 4 === 0 ? 1 : 0) + s * W[i] + c * W2[i]);
  // t = (I theta + (1 - cos) W + (theta - sin) W^2) v
  const G = [0, 1, 2, 3, 4, 5, 6, 7, 8].map(i => (i % 4 === 0 ? theta : 0) + c * W[i] + (theta - s) * W2[i]);
  const t = [0, 1, 2].map(r => G[3 * r] * vx + G[3 * r + 1] * vy + G[3 * r + 2] * vz);
  return [R[0], R[1], R[2], t[0], R[3], R[4], R[5], t[1], R[6], R[7], R[8], t[2], 0, 0, 0, 1];
}

/** World transform of every link (chain order, link 0 the base) at joint angles q. */
export function linkTransforms(robot, q) {
  if (!robot._rest) robot._rest = robot.links.map(link => poseMatrix(link.rest));
  const out = [robot._rest[0]];
  let motion = identity();
  robot.joints.forEach((joint, j) => {
    motion = multiply(motion, screwExp(joint.screw, q[j]));
    out.push(multiply(motion, robot._rest[j + 1]));
  });
  return out;
}

/** m applied to point p. */
export function transformPoint(m, [x, y, z]) {
  return [m[0] * x + m[1] * y + m[2] * z + m[3], m[4] * x + m[5] * y + m[6] * z + m[7], m[8] * x + m[9] * y + m[10] * z + m[11]];
}

/** Point p in the frame of rigid transform m: m^{-1} p. */
export function inversePoint(m, [x, y, z]) {
  const dx = x - m[3], dy = y - m[7], dz = z - m[11];
  return [m[0] * dx + m[4] * dy + m[8] * dz, m[1] * dx + m[5] * dy + m[9] * dz, m[2] * dx + m[6] * dy + m[10] * dz];
}

/** World centers of the collision spheres at q: [{center, radius}]. */
export function sphereCenters(robot, q, transforms = linkTransforms(robot, q)) {
  return robot.spheres.map(sphere => ({ center: transformPoint(transforms[sphere.link], sphere.center), radius: sphere.radius }));
}

/** Clamp each joint angle to its limits. */
export function clampJoints(robot, q) {
  return q.map((value, j) => Math.min(robot.joints[j].upper, Math.max(robot.joints[j].lower, value)));
}

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

/** Solve the 3x3 system A x = b (Cramer's rule; A symmetric positive definite here). */
function solve3(A, b) {
  const det = m => m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
  const d = det(A);
  return [0, 1, 2].map(col => det(A.map((value, i) => (i % 3 === col ? b[Math.floor(i / 3)] : value))) / d);
}

/**
 * Joint angles that move a point fixed on one link toward a world target:
 * damped, weighted least squares on the point's position, from q0, within
 * the joint limits. weights > 1 make a joint move less (default: the wrists).
 * @returns the new joint angles
 */
export function solvePointIK(robot, q0, link, pointInLink, target, {
  iterations = 30, damping = 0.05, weights = [1, 1, 1, 3, 3, 3], tolerance = 1e-4,
} = {}) {
  let q = q0.slice();
  for (let iteration = 0; iteration < iterations; iteration++) {
    const transforms = linkTransforms(robot, q);
    const p = transformPoint(transforms[link], pointInLink);
    const error = [target[0] - p[0], target[1] - p[1], target[2] - p[2]];
    if (Math.hypot(...error) < tolerance) break;
    // Point Jacobian columns from the current space screws: v + w x p.
    const columns = [];
    let motion = identity();
    for (let j = 0; j < robot.joints.length; j++) {
      if (j >= link) { columns.push([0, 0, 0]); continue; }
      const [wx, wy, wz, vx, vy, vz] = robot.joints[j].screw;
      const R = [motion[0], motion[1], motion[2], motion[4], motion[5], motion[6], motion[8], motion[9], motion[10]];
      const t = [motion[3], motion[7], motion[11]];
      const w = [R[0] * wx + R[1] * wy + R[2] * wz, R[3] * wx + R[4] * wy + R[5] * wz, R[6] * wx + R[7] * wy + R[8] * wz];
      const v0 = [R[0] * vx + R[1] * vy + R[2] * vz, R[3] * vx + R[4] * vy + R[5] * vz, R[6] * vx + R[7] * vy + R[8] * vz];
      const v = v0.map((value, i) => value + cross(t, w)[i]);  // adjoint of the prefix motion
      const pointVelocity = cross(w, p).map((value, i) => value + v[i]);
      columns.push(pointVelocity);
      motion = multiply(motion, screwExp(robot.joints[j].screw, q[j]));
    }
    // dq = W^-1 J^T (J W^-1 J^T + damping^2 I)^-1 e
    const A = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    columns.forEach((col, j) => {
      for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) A[3 * r + c] += col[r] * col[c] / weights[j];
    });
    A[0] += damping ** 2; A[4] += damping ** 2; A[8] += damping ** 2;
    const y = solve3(A, error);
    const step = columns.map((col, j) => (col[0] * y[0] + col[1] * y[1] + col[2] * y[2]) / weights[j]);
    const scale = Math.min(1, 0.5 / Math.max(...step.map(Math.abs)));  // at most 0.5 rad per iteration
    q = clampJoints(robot, q.map((value, j) => value + scale * step[j]));
  }
  return q;
}

/**
 * Smallest gap between the arm's spheres at q and spheres at the given
 * centers (negative = overlap), and the obstacle it is to.
 */
export function armClearance(robot, q, obstacles) {
  let best = Infinity, index = -1;
  for (const sphere of sphereCenters(robot, q)) {
    obstacles.forEach((obstacle, j) => {
      const gap = Math.hypot(sphere.center[0] - obstacle.center[0], sphere.center[1] - obstacle.center[1],
        sphere.center[2] - obstacle.center[2]) - sphere.radius - obstacle.radius;
      if (gap < best) { best = gap; index = j; }
    });
  }
  return { gap: best, obstacle: index };
}
