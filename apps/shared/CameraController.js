import * as THREE from 'three';

const directions = {
  perspective: new THREE.Vector3(.8, -1, .8),
  top: new THREE.Vector3(0, -.001, 1),
  front: new THREE.Vector3(1, .001, 0),
  side: new THREE.Vector3(.001, -1, 0),
};

/** Z-up camera operations, independent of robot model, physics, or recordings. */
export class CameraController {
  constructor(camera, controls) {
    this.camera = camera;
    this.controls = controls;
    this.following = false;
    this.target = new THREE.Vector3();
  }

  /** Metre-valued world target and camera offset; stop following. */
  lookAt(target, offset) {
    this.following = false;
    this.fittedBounds = null;
    this.target.copy(target);
    this.controls.target.copy(target);
    this.camera.position.copy(target).add(offset);
    this.controls.update();
  }

  /** Start following at this world position using a specified viewing offset. */
  follow(target, offset = new THREE.Vector3(2.6, -3.4, 1.25)) {
    this.lookAt(target, offset);
    this.following = true;
  }

  /** Follow translation while preserving the user's orbit, pan, and zoom. */
  updateTarget(target) {
    if (this.following) {
      const delta = target.clone().sub(this.target);
      this.camera.position.add(delta);
      this.controls.target.add(delta);
    }
    this.target.copy(target);
    this.controls.update();
  }

  /** Frame a world-space Box3 using both horizontal and vertical field of view. */
  fit(bounds, { view = 'perspective', direction, padding = 1.1, minDistance = 2 } = {}) {
    if (bounds.isEmpty()) return;
    const sphere = bounds.getBoundingSphere(new THREE.Sphere());
    const vertical = THREE.MathUtils.degToRad(this.camera.fov) / 2;
    const horizontal = Math.atan(Math.tan(vertical) * this.camera.aspect);
    const distance = Math.max(minDistance, sphere.radius / Math.sin(Math.min(vertical, horizontal)) * padding);
    const offset = (direction || directions[view] || directions.perspective).clone().normalize().multiplyScalar(distance);
    this.controls.maxDistance = Math.max(this.controls.maxDistance, distance * 2);
    this.camera.far = Math.max(this.camera.far, distance * 4);
    this.camera.updateProjectionMatrix();
    this.lookAt(sphere.center, offset);
    this.fittedBounds = bounds.clone();
    this.fitOptions = { view, direction: direction?.clone(), padding, minDistance };
  }

  /** Preserve a fitted view when the canvas aspect changes; leave follow mode alone. */
  refit() { if (this.fittedBounds) this.fit(this.fittedBounds, this.fitOptions); }
}
