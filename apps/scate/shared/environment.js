/**
 * Static walls and floors (half-spaces) for the SCATE apps: clearance of the
 * arm's spheres to them and request checks (no Three.js, so the dev server
 * can import it); planePanels.js draws them.
 *
 * A plane is {name, normal: [nx, ny, nz], offset}: free space is where
 * normal . p >= offset (normal has unit length and points into free space),
 * the same convention as HalfSpace in scate_online.h.
 */
import { sphereCenters } from './kinematics.js';

/** Default drawing bounds: the UR10e SDF grid (x, y in [-1.5, 1.5], z in [-0.45, 1.65] m). */
export const DEFAULT_BOUNDS = { min: [-1.5, -1.5, -0.45], max: [1.5, 1.5, 1.65] };

/** Signed distance of point p to a plane, positive in free space. */
export function planeDistance(plane, p) {
  const n = plane.normal;
  return n[0] * p[0] + n[1] * p[1] + n[2] * p[2] - plane.offset;
}

/**
 * Smallest gap between the arm's spheres at q and the planes (negative =
 * through it), with the plane it is to; {gap: Infinity, plane: -1} with none.
 */
export function planeClearance(robot, q, planes) {
  let gap = Infinity, plane = -1;
  if (!planes?.length) return { gap, plane };
  for (const sphere of sphereCenters(robot, q)) {
    planes.forEach((p, j) => {
      const g = planeDistance(p, sphere.center) - sphere.radius;
      if (g < gap) { gap = g; plane = j; }
    });
  }
  return { gap, plane };
}

/** "plane name nx ny nz offset" request fields, checked: throws on anything malformed. */
export function checkPlane(plane, index = 0) {
  const what = `plane ${index + 1}`;
  if (typeof plane?.name !== 'string' || !/^[A-Za-z0-9_-]{1,32}$/.test(plane.name)) {
    throw new Error(`${what} needs a name of letters, digits, _ or -`);
  }
  const n = plane.normal;
  if (!Array.isArray(n) || n.length !== 3 || !n.every(Number.isFinite)) throw new Error(`${what} normal needs 3 numbers`);
  if (Math.abs(Math.hypot(...n) - 1) > 1e-3) throw new Error(`${what} normal must have unit length`);
  if (!Number.isFinite(plane.offset)) throw new Error(`${what} offset must be a number`);
  return plane;
}
