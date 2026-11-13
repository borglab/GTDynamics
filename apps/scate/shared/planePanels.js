/**
 * Translucent panels for the static walls and floors of environment.js.
 */
import * as THREE from 'three';
import { DEFAULT_BOUNDS } from './environment.js';

/**
 * A translucent panel for each plane, clipped to the bounds box: centered on
 * the projection of the box center, spanning the box's extent along two
 * in-plane axes, with an outline. Returns a THREE.Group.
 */
export function makePlanePanels(planes, { bounds = DEFAULT_BOUNDS, color = '#8a96a3', opacity = 0.22 } = {}) {
  const group = new THREE.Group();
  group.name = 'environment';
  const lo = new THREE.Vector3(...bounds.min), hi = new THREE.Vector3(...bounds.max);
  const center = lo.clone().add(hi).multiplyScalar(0.5);
  const corners = [];
  for (let i = 0; i < 8; i++) {
    corners.push(new THREE.Vector3(i & 1 ? hi.x : lo.x, i & 2 ? hi.y : lo.y, i & 4 ? hi.z : lo.z));
  }
  for (const plane of planes || []) {
    const n = new THREE.Vector3(...plane.normal).normalize();
    const origin = center.clone().addScaledVector(n, plane.offset - n.dot(center));
    // In-plane axes: the world axis least aligned with n, made orthogonal.
    const seed = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)]
      .reduce((a, b) => (Math.abs(a.dot(n)) <= Math.abs(b.dot(n)) ? a : b));
    const u = seed.clone().addScaledVector(n, -seed.dot(n)).normalize();
    const v = n.clone().cross(u);
    let uMin = Infinity, uMax = -Infinity, vMin = Infinity, vMax = -Infinity;
    for (const c of corners) {
      const d = c.clone().sub(origin);
      uMin = Math.min(uMin, d.dot(u)); uMax = Math.max(uMax, d.dot(u));
      vMin = Math.min(vMin, d.dot(v)); vMax = Math.max(vMax, d.dot(v));
    }
    const width = uMax - uMin, height = vMax - vMin;
    const mid = origin.clone().addScaledVector(u, (uMin + uMax) / 2).addScaledVector(v, (vMin + vMax) / 2);
    const basis = new THREE.Matrix4().makeBasis(u, v, n).setPosition(mid);
    const panel = new THREE.Mesh(new THREE.PlaneGeometry(width, height),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false }));
    panel.applyMatrix4(basis);
    panel.name = plane.name;
    panel.userData.environment = true;
    panel.userData.ownMaterial = true;
    const outline = new THREE.LineSegments(new THREE.EdgesGeometry(panel.geometry),
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: Math.min(1, 3 * opacity) }));
    outline.applyMatrix4(basis);
    outline.userData.ownMaterial = true;
    group.add(panel, outline);
  }
  return group;
}

/** Dispose the geometries and materials of a group made by makePlanePanels. */
export function disposePlanePanels(group) {
  group?.traverse(object => {
    object.geometry?.dispose();
    object.material?.dispose();
  });
}
