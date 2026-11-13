/** Pose, joint and solve-status helpers for the URDF IK app (no DOM, no scene). */
import { Euler, MathUtils, Quaternion } from 'three';

/** Filter model-specific solvers while retaining compatibility with browser IK. */
export function availableSchemes(schemes, endEffector, orientation) {
  return schemes.filter(scheme =>
    (endEffector?.schemes ? endEffector.schemes.includes(scheme.id) : !scheme.requires_orientation) &&
    (!scheme.requires_orientation || orientation));
}

export const POSE_FIELDS = [
  ['x', 'm'], ['y', 'm'], ['z', 'm'], ['roll', '°'], ['pitch', '°'], ['yaw', '°'],
];

// URDF roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll), i.e. three.js 'ZYX'.
export function rpyDegFromQuaternion(q) {
  const e = new Euler().setFromQuaternion(q, 'ZYX');
  return [e.x, e.y, e.z].map(MathUtils.radToDeg);
}

export function quaternionFromRpyDeg(rpyDeg) {
  const [r, p, y] = rpyDeg.map(MathUtils.degToRad);
  return new Quaternion().setFromEuler(new Euler(r, p, y, 'ZYX'));
}

/** Slider bounds: the joint limits, or ±π rad / ±1 m (widened to include value) if unlimited. */
export function sliderRange(joint, value) {
  let lo = joint.lower;
  let hi = joint.upper;
  if (lo === null || hi === null) {
    const span = joint.type === 'prismatic' ? 1 : Math.PI;
    lo = Math.min(-span, value);
    hi = Math.max(span, value);
  }
  return [lo, hi];
}

/** Zero, or mid-range for joints whose limits exclude zero. */
export function homeConfiguration(joints) {
  const q = {};
  for (const j of joints) {
    const lo = j.lower ?? -Infinity;
    const hi = j.upper ?? Infinity;
    q[j.name] = lo <= 0 && 0 <= hi ? 0 : (lo + hi) / 2;
  }
  return q;
}

export function endEffectorLabel(e) {
  return (e.link === e.name ? e.name : `${e.name} (on ${e.link})`) + ` · ${e.dof} DoF`;
}

/** Status panel contents for an /api/ik response. */
export function solveSummary(res, orientation) {
  const meta = [];
  if (res.pos_err !== undefined) {
    meta.push(`${(res.pos_err * 1000).toFixed(2)} mm`);
    if (orientation) meta.push(`${res.rot_err_deg.toFixed(2)}°`);
  }
  if (res.iterations !== null && res.iterations !== undefined) meta.push(`${res.iterations} it`);
  meta.push(`${res.time_ms.toFixed(1)} ms`);
  if (res.dof !== undefined) meta.push(`${res.dof} DoF`);
  return {
    cls: { solved: 'ok', failed: 'failed', error: 'error' }[res.status],
    label: { solved: 'Solved', failed: 'Not solved', error: 'Solver error' }[res.status],
    meta: meta.join(' · '),
    items: [
      ...res.reasons.map((text) => ({ text, kind: 'reason' })),
      ...res.warnings.map((text) => ({ text, kind: 'warning' })),
    ],
  };
}
