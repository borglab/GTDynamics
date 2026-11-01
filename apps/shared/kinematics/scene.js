import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { createRobotVisual } from '../mujocoRobot.js';
import { bodyOffset } from './recording.js';

const robotTemplates = new Map();

function robotVisual(model) {
  if (!robotTemplates.has(model)) {
    const visual = createRobotVisual(model);
    const geometries = new Set();
    visual.root.traverse(object => { if (object.geometry) geometries.add(object.geometry); });
    robotTemplates.set(model, {...visual, geometries});
  }
  const template = robotTemplates.get(model);
  const root = template.root.clone(true);
  // Body transforms and materials are independent; mesh geometry is immutable.
  root.traverse(object => { if (object.material) object.material = object.material.clone(); });
  return {root, bodies: root.children};
}

export function disposeRobotTemplates() {
  for (const template of robotTemplates.values()) {
    template.geometries.forEach(geometry => geometry.dispose());
    template.root.traverse(object => { if (object.material) object.material.dispose(); });
  }
  robotTemplates.clear();
}

/** Render stored body transforms directly. No MuJoCo, FK, or optimizer in-browser. */
export class ReplayScene {
  constructor(container, model, record, floats, color) {
    this.record = record; this.floats = floats; this.container = container;
    this.scene = new THREE.Scene(); this.scene.background = new THREE.Color('#f1f3ef');
    this.camera = new THREE.PerspectiveCamera(35, 1, 0.02, 80); this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
    this.renderer.outputEncoding = THREE.sRGBEncoding;
    this.renderer.domElement.setAttribute('aria-label', 'Recorded G1 robot poses; drag to orbit, scroll to zoom');
    container.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.minDistance = 0.5; this.controls.maxDistance = 20;
    this.controls.maxPolarAngle = Math.PI * .49;
    this.controls.addEventListener('change', () => this.render());
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x788679, 1.1));
    const light = new THREE.DirectionalLight(0xffffff, 1.2); light.position.set(2, -3, 6); this.scene.add(light);
    this.grid = new THREE.GridHelper(40, 160, '#bac7bb', '#dce2d9'); this.grid.rotation.x = Math.PI / 2; this.scene.add(this.grid);
    this.robot = robotVisual(model); this.scene.add(this.robot.root);
    this.reference = robotVisual(model); this.scene.add(this.reference.root);
    this.reference.root.traverse(object => {
      if (!object.material) return;
      object.material.color.set('#8b96a6'); object.material.opacity = .18;
      object.material.transparent = true; object.material.depthWrite = false;
    });
    this.path = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color }));
    this.scene.add(this.path);
    // Compare XY ground projections in the same recorded world frame.
    const referencePoints = Array.from({ length: record.samples }, (_, k) => {
      const offset = bodyOffset(record, record.reference, k, record.rootBody);
      return new THREE.Vector3(floats[offset], floats[offset + 1], .014);
    });
    this.referencePath = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(referencePoints),
      new THREE.LineDashedMaterial({ color: '#536b89', dashSize: .10, gapSize: .07 }),
    );
    this.referencePath.computeLineDistances();
    this.scene.add(this.referencePath);
    this.slipPoints = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({
      size: 5, sizeAttenuation: false, vertexColors: true, transparent: true,
      opacity: .85, depthWrite: false, depthTest: false,
    }));
    this.slipPoints.visible = false; this.slipPoints.renderOrder = 3;
    this.scene.add(this.slipPoints);
    this.slipArrows = [0, 1].map(() => {
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), .1, '#ce5827');
      arrow.visible = false; this.scene.add(arrow); return arrow;
    });
    this.observer = new ResizeObserver(() => this.resize()); this.observer.observe(container);
    this.reset(); this.resize();
    container.dataset.ready = 'true';
  }
  reset() {
    this.followRoot = true;
    const offset = bodyOffset(this.record, this.currentState || this.record.reference, this.currentFrame || 0, this.record.rootBody);
    this.focus = new THREE.Vector3().fromArray(this.floats, offset);
    this.controls.target.copy(this.focus); this.camera.position.copy(this.focus).add(new THREE.Vector3(2.6, -3.4, 1.25));
    this.controls.update(); this.render();
  }
  fitPaths() {
    if (!this.path.geometry.attributes.position) return;
    this.path.geometry.computeBoundingBox();
    const bounds = this.path.geometry.boundingBox.clone();
    if (this.referencePath.visible) {
      this.referencePath.geometry.computeBoundingBox();
      bounds.union(this.referencePath.geometry.boundingBox);
    }
    // Include robot height above the XY paths, even for a stationary recording.
    bounds.max.z = Math.max(bounds.max.z, 1.5);
    const sphere = bounds.getBoundingSphere(new THREE.Sphere());
    const vertical = THREE.MathUtils.degToRad(this.camera.fov) / 2;
    const horizontal = Math.atan(Math.tan(vertical) * this.camera.aspect);
    const distance = Math.max(2, sphere.radius / Math.sin(Math.min(vertical, horizontal)) * 1.1);
    const direction = new THREE.Vector3(.8, -1, 1.3).normalize();
    this.followRoot = false;
    this.controls.target.copy(sphere.center);
    this.camera.position.copy(sphere.center).addScaledVector(direction, distance);
    this.controls.maxDistance = Math.max(20, distance * 2);
    this.camera.far = Math.max(80, distance * 4);
    this.camera.updateProjectionMatrix(); this.controls.update(); this.render();
  }
  place(visual, state, frame) {
    visual.bodies.forEach((body, i) => {
      const offset = bodyOffset(this.record, state, frame, i);
      body.position.fromArray(this.floats, offset); body.quaternion.fromArray(this.floats, offset + 3);
    });
  }
  update(state, frame, showReference, pathState, options = {}) {
    this.currentState = state; this.currentFrame = frame;
    const offset = bodyOffset(this.record, state, frame, this.record.rootBody);
    const focus = new THREE.Vector3().fromArray(this.floats, offset);
    const delta = focus.clone().sub(this.focus);
    if (this.followRoot) { this.camera.position.add(delta); this.controls.target.add(delta); }
    this.focus.copy(focus);
    this.grid.position.set(Math.round(focus.x / 5) * 5, Math.round(focus.y / 5) * 5, 0);
    this.controls.update();
    this.place(this.robot, state, frame);
    this.place(this.reference, this.record.reference, frame); this.reference.root.visible = showReference;
    this.referencePath.visible = showReference;
    if (this.pathState !== pathState) {
      this.pathState = pathState;
      const points = Array.from({ length: this.record.samples }, (_, k) => {
        const offset = bodyOffset(this.record, pathState, k, this.record.rootBody);
        return new THREE.Vector3(this.floats[offset], this.floats[offset + 1], .012);
      });
      this.path.geometry.dispose(); this.path.geometry = new THREE.BufferGeometry().setFromPoints(points);
    }
    this.updateSlip(state, pathState, frame, options);
    this.render();
  }
  updateSlip(state, pathState, frame, {showSlip = false, slipThreshold = .005}) {
    this.slipPoints.visible = showSlip && pathState.contactSlipOffset !== undefined;
    if (!this.slipPoints.visible) {
      this.slipArrows.forEach(arrow => { arrow.visible = false; });
      this.container.dataset.slipCount = '0'; return;
    }
    if (this.slipState !== pathState || this.slipThreshold !== slipThreshold) {
      this.slipState = pathState; this.slipThreshold = slipThreshold;
      const positions = [], colors = [], yellow = new THREE.Color('#e8b32e'), red = new THREE.Color('#c94125');
      for (let k = 0; k < this.record.samples; k++) for (let foot = 0; foot < 2; foot++) {
        const offset = pathState.contactSlipOffset + (k*2 + foot)*7;
        const distance = this.floats[offset + 6];
        if (distance < slipThreshold) continue;
        positions.push(...this.floats.subarray(offset + 3, offset + 6));
        const color = yellow.clone().lerp(red, Math.min(1, (distance-slipThreshold)/.05));
        colors.push(color.r, color.g, color.b);
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
      geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
      this.slipPoints.geometry.dispose(); this.slipPoints.geometry = geometry;
    }
    this.container.dataset.slipCount = String(this.slipPoints.geometry.attributes.position.count);
    const selected = state.contactSlipOffset === undefined ? pathState : state;
    this.slipArrows.forEach((arrow, foot) => {
      const offset = selected.contactSlipOffset + (frame*2 + foot)*7;
      const distance = this.floats[offset + 6];
      arrow.visible = distance >= slipThreshold;
      if (!arrow.visible) return;
      const previous = new THREE.Vector3().fromArray(this.floats, offset);
      const current = new THREE.Vector3().fromArray(this.floats, offset + 3);
      arrow.position.copy(previous); arrow.setDirection(current.sub(previous).normalize());
      const length = Math.min(.5, Math.max(.08, distance*5));
      arrow.setLength(length, Math.min(.08, length*.35), Math.min(.045, length*.22));
    });
  }
  resize() {
    const width = this.container.clientWidth, height = this.container.clientHeight;
    this.renderer.setSize(width, height); this.camera.aspect = width / Math.max(1, height);
    this.camera.updateProjectionMatrix();
    if (this.followRoot === false) this.fitPaths();
    else this.render();
  }
  render() { this.renderer.render(this.scene, this.camera); }
  dispose() {
    this.observer.disconnect(); this.controls.dispose();
    const geometries = new Set(), materials = new Set();
    this.scene.traverse(object => { if (object.geometry) geometries.add(object.geometry); if (object.material) materials.add(object.material); });
    // Release renderer allocations/listeners. The shared CPU mesh buffers remain
    // available in the template for the next renderer to upload.
    geometries.forEach(g => g.dispose());
    materials.forEach(m => m.dispose());
    this.renderer.dispose(); this.renderer.domElement.remove();
  }
}
