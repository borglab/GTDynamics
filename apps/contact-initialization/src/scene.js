import * as THREE from 'three';
import { RobotViewport, disposeObject3D } from '../../shared/RobotViewport.js';
import { MujocoG1 } from './mujocoG1.js';

// Everything in this viewer stays in the robot's native, Z-up coordinate system.
const COLORS = { ink: '#243438', silver: '#94a4a3', left: '#13836e', right: '#df785c', paper: '#f0f2ed', path: '#778b87', reference: '#a6a7b6' };
const vec = (a) => new THREE.Vector3(...(a ?? [0, 0, 0]));

function material(color, opacity = 1, extras = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.12,
    transparent: opacity < 1, opacity, depthWrite: opacity > 0.7, ...extras });
}

const release = disposeObject3D;

function empty(group) {
  for (const child of [...group.children]) { group.remove(child); release(child); }
}

/** Accepts the solver's row-major rotation matrix, or dataset wxyz quaternions. */
function poseMatrix(pose) {
  if (!pose) return new THREE.Matrix4();
  const p = pose.translation ?? pose.position ?? pose.p ?? [0, 0, 0];
  if (pose.rotation?.length === 9) {
    const r = pose.rotation;
    return new THREE.Matrix4().set(r[0], r[1], r[2], p[0], r[3], r[4], r[5], p[1], r[6], r[7], r[8], p[2], 0, 0, 0, 1);
  }
  const q = pose.quaternion ? new THREE.Quaternion(...pose.quaternion)
    : pose.q ? new THREE.Quaternion(pose.q[1], pose.q[2], pose.q[3], pose.q[0]) : new THREE.Quaternion();
  return new THREE.Matrix4().compose(vec(p), q, new THREE.Vector3(1, 1, 1));
}

function line(points, color, opacity = 1, dashed = false) {
  const geometry = new THREE.BufferGeometry().setFromPoints(points.map((p) => p.isVector3 ? p : vec(p)));
  const style = dashed ? new THREE.LineDashedMaterial({ color, transparent: true, opacity, dashSize: 0.035, gapSize: 0.025 })
    : new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity });
  const result = new THREE.Line(geometry, style);
  if (dashed) result.computeLineDistances();
  return result;
}

function sphere(at, radius, style) {
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius, 12, 8), style);
  mesh.position.copy(at);
  mesh.castShadow = style.opacity > 0.7;
  return mesh;
}

function caption(value, color = COLORS.ink, scale = 0.22) {
  const canvas = document.createElement('canvas');
  canvas.width = 512; canvas.height = 128;
  const context = canvas.getContext('2d');
  context.clearRect(0, 0, 512, 128);
  context.font = '600 44px "SFMono-Regular", Consolas, monospace';
  context.textAlign = 'center'; context.textBaseline = 'middle';
  context.fillStyle = 'rgba(240,242,237,0.94)';
  const width = context.measureText(value).width + 44;
  context.beginPath(); context.roundRect(256 - width / 2, 26, width, 76, 24); context.fill();
  context.fillStyle = color; context.fillText(value, 256, 67);
  const texture = new THREE.CanvasTexture(canvas);
  texture.encoding = THREE.sRGBEncoding;
  const result = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false, toneMapped: false }));
  result.scale.set(scale * 4, scale, 1); result.renderOrder = 30;
  return result;
}

function footprint(contact, active = false) {
  if (!contact.pose) return null;
  const group = new THREE.Group();
  group.applyMatrix4(poseMatrix(contact.pose));
  const color = COLORS[contact.foot] ?? COLORS.left;
  const axes = new THREE.AxesHelper(active ? .16 : .10);
  axes.material.depthTest = false; axes.renderOrder = 5;
  group.add(axes);
  group.add(sphere(new THREE.Vector3(), active ? .014 : .008, material(color, active ? 1 : .65)));
  return group;
}

/**
 * Rendering contract:
 *  model: {jointNames}; samples: [{jointAngles}]
 *  frames: [{sampleIndex, observations:[{local,contactId}]}]
 *  poses: solver {translation, rotation:rowMajor9}; referencePoses: same, or {p,q:wxyz}
 *  contacts: [{id,foot,pose:{rotation,translation}}]; selectedIndex indexes frames.
 */
export class ContactScene {
  constructor(container, { onSelect } = {}) {
    this.container = container; this.onSelect = onSelect; this.state = {}; this.disposed = false;
    this.assetController = new AbortController();
    this.viewport = new RobotViewport(container, {
      background: COLORS.paper, far: 60, shadows: true, exposure: .95,
      controls: {minDistance: .8, maxDistance: 18, maxPolarAngle: Math.PI * .488, screenSpacePanning: false},
      ariaLabel: 'Interactive 3D reconstruction of the G1 walking sequence',
    });
    const {scene, camera, renderer, controls, cameraController} = this.viewport;
    Object.assign(this, {scene, camera, renderer, controls, cameraController});
    this.ground = this.viewport.addStage().ground;
    this.pathGroup = new THREE.Group(); this.contactGroup = new THREE.Group(); this.ghostGroup = new THREE.Group();
    this.selectedGroup = new THREE.Group(); this.labelGroup = new THREE.Group();
    this.scene.add(this.pathGroup, this.contactGroup, this.ghostGroup, this.selectedGroup, this.labelGroup);
    this.raycaster = new THREE.Raycaster(); this.pointer = new THREE.Vector2(); this.hitTargets = [];
    this.pointerDown = (event) => { this.down = [event.clientX, event.clientY]; };
    this.pointerUp = (event) => this.pick(event);
    this.renderer.domElement.addEventListener('pointerdown', this.pointerDown);
    this.renderer.domElement.addEventListener('pointerup', this.pointerUp);
    this.bounds = new THREE.Box3(new THREE.Vector3(-1, -1, 0), new THREE.Vector3(1, 1, 1.2));
    this.reset(); this.resize();
  }

  async init() {
    const robot = await MujocoG1.load(this.assetController.signal);
    if (this.disposed) { robot.dispose(); return; }
    this.robot = robot;
    this.container.dataset.renderer = 'mujoco';
    this.container.dataset.joints = String(this.robot.joints.size);
    this.container.dataset.meshes = String(this.robot.model.nmesh);
  }

  update(next) {
    if (this.disposed || !this.robot) return;
    const previous = this.state;
    this.state = { ...this.state, ...next };
    const state = this.state;
    const changed = ['model', 'samples', 'frames', 'poses', 'contacts', 'referencePoses', 'mode','floorConstrained'].some((key) => previous[key] !== state[key]);
    if (changed) this.rebuild();
    this.ghostGroup.visible = state.showGhosts !== false;
    this.drawSelected(); this.render();
  }

  sample(frame) { return this.state.samples?.[frame?.sampleIndex ?? frame?.index ?? 0] ?? {}; }

  referencePose(frame, i) {
    const poses = this.state.referencePoses ?? this.state.reference?.poses;
    if (!poses) return this.sample(frame).reference;
    return poses.length === this.state.frames?.length ? poses[i] : poses[frame?.sampleIndex ?? frame?.index ?? i];
  }

  rebuild() {
    for (const group of [this.pathGroup, this.contactGroup, this.ghostGroup, this.labelGroup]) empty(group);
    this.hitTargets = [];
    const { frames = [], poses = [], contacts = [], model } = this.state;
    const positions = poses.map((p) => new THREE.Vector3().setFromMatrixPosition(poseMatrix(p)));
    if (!positions.length) return;
    // Fit the actual trajectory envelope, including robot height and visible
    // reference poses. A cube based on the longest axis zooms out excessively.
    const box = new THREE.Box3().setFromPoints(positions);
    this.bounds = box.clone();
    for (const contact of contacts)
      this.bounds.expandByPoint(new THREE.Vector3().setFromMatrixPosition(poseMatrix(contact.pose)));
    if (this.state.showReference !== false)
      frames.forEach((frame, i) => {
        const pose = this.referencePose(frame, i);
        if (pose) this.bounds.expandByPoint(new THREE.Vector3().setFromMatrixPosition(poseMatrix(pose)));
      });
    this.bounds.expandByVector(new THREE.Vector3(.35, .35, .8));
    this.ground.visible=this.state.floorConstrained!==false;
    if (!this.hasData) { this.hasData = true; this.reset(); }
    const components=this.state.components||[positions.map((_,i)=>i)];
    for(const indices of components){
      const sequence=indices.slice().sort((a,b)=>a-b).map(i=>positions[i]);
      if(sequence.length<2)continue;
      const groundPath=sequence.map(p=>new THREE.Vector3(p.x,p.y,0.015));
      this.pathGroup.add(line(groundPath,COLORS.path,0.7,true));
      this.pathGroup.add(line(sequence,COLORS.ink,0.34));
    }
    const ref = frames.map((frame, i) => this.referencePose(frame, i)).filter(Boolean).map((pose) => new THREE.Vector3().setFromMatrixPosition(poseMatrix(pose)));
    if (ref.length > 1) {
      const referenceLines=new THREE.Group();referenceLines.name='reference-path';
      for(const indices of components){
        const sequence=indices.slice().sort((a,b)=>a-b).map(i=>ref[i]);
        if(sequence.length>1)referenceLines.add(line(sequence,COLORS.reference,0.5,true));
      }
      referenceLines.visible=this.state.showReference!==false;this.pathGroup.add(referenceLines);
    }
    const step = Math.max(1, Math.ceil(frames.length / 9));
    frames.forEach((frame, index) => {
      if (!poses[index]) return;
      const dot = sphere(positions[index], 0.015, material(COLORS.ink, 0.7)); dot.userData.frameIndex = index;
      this.hitTargets.push(dot); this.pathGroup.add(dot);
      if (index % step !== 0 && index !== frames.length - 1) return;
      const ghost = this.robot.create(this.sample(frame), model, { opacity: 0.055, color: COLORS.path }); ghost.applyMatrix4(poseMatrix(poses[index])); this.ghostGroup.add(ghost);
      const number = caption(String(index).padStart(2, '0'), '#73827b', 0.12);
      number.position.set(positions[index].x, positions[index].y - 0.12, 0.035); this.labelGroup.add(number);
    });
    for (const contact of contacts) { const patch = footprint(contact,false,this.state.floorConstrained!==false); if (patch) this.contactGroup.add(patch); }
    // A small scale bar gives metric meaning without turning the scene into a diagram.
    const min = box.min; const barY = Math.min(-0.43, min.y - 0.4);
    const x = min.x; const bar = [[x, barY, 0.012], [x + 0.5, barY, 0.012]];
    this.pathGroup.add(line(bar, '#7c8e84', 0.8));
    for (const offset of [0, 0.5]) this.pathGroup.add(line([[x + offset, barY - 0.025, 0.012], [x + offset, barY + 0.025, 0.012]], '#7c8e84', 0.8));
    const scale = caption('0.5 m', '#75867b', 0.11); scale.position.set(x + 0.25, barY - 0.1, 0.04); this.labelGroup.add(scale);
  }

  drawSelected() {
    empty(this.selectedGroup);
    const { frames = [], poses = [], contacts = [], model, selectedIndex = 0 } = this.state;
    const index = Math.max(0, Math.min(frames.length - 1, selectedIndex));
    const frame = frames[index]; if (!frame || !poses[index]) return;
    const matrix = poseMatrix(poses[index]);
    const body = this.robot.create(this.sample(frame), model); body.applyMatrix4(matrix); this.selectedGroup.add(body);
    const root = new THREE.Vector3().setFromMatrixPosition(matrix);
    const floorCenter = root.clone().setZ(0.008);
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.20, 0.207, 64), new THREE.MeshBasicMaterial({ color: COLORS.ink, transparent: true, opacity: 0.15, side: THREE.DoubleSide, depthWrite: false }));
    ring.position.copy(floorCenter); ring.visible=this.state.floorConstrained!==false; this.selectedGroup.add(ring);
    const axes = new THREE.AxesHelper(0.16); axes.applyMatrix4(matrix); this.selectedGroup.add(axes);
    const rootNode = sphere(root, 0.027, material('#fffef7')); this.selectedGroup.add(rootNode);
    const label = caption(`K${String(index).padStart(2, '0')}`, COLORS.ink, 0.14);
    label.position.copy(root).add(new THREE.Vector3(0, 0, 0.72)); this.selectedGroup.add(label);
    const active = new Set((frame.observations ?? []).map((observation) => observation.contactId));
    for (const id of frame.contacts ?? []) active.add(id);
    for (const contact of contacts) {
      if (!active.has(contact.id)) continue;
      const patch = footprint(contact, true,this.state.floorConstrained!==false); if (patch) this.selectedGroup.add(patch);
      const center = vec(contact.pose.translation);
      const side = contact.foot ?? contact.side; const color = COLORS[side] ?? COLORS.left;
      this.selectedGroup.add(line([root, center], color, 0.55, true));
      const note = caption(`${side === 'right' ? 'R' : 'L'}${String(contact.id).replace(/^[LRlr]/, '')}`, color, 0.12);
      note.position.copy(center).add(new THREE.Vector3(0, side === 'right' ? -0.13 : 0.13, 0.055)); this.selectedGroup.add(note);
    }
    if (this.state.showReference !== false) {
      const reference = this.referencePose(frame, index);
      if (reference) { const body = this.robot.create(this.sample(frame), model, { opacity: 0.2, color: COLORS.reference }); body.applyMatrix4(poseMatrix(reference)); this.selectedGroup.add(body); }
    }
    const referencePath = this.pathGroup.getObjectByName('reference-path');
    if (referencePath) referencePath.visible = this.state.showReference !== false;
  }

  pick(event) {
    if (!this.down || Math.hypot(event.clientX - this.down[0], event.clientY - this.down[1]) > 5) return;
    const bounds = this.renderer.domElement.getBoundingClientRect();
    this.pointer.set((event.clientX - bounds.left) / bounds.width * 2 - 1, -(event.clientY - bounds.top) / bounds.height * 2 + 1);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const hits = this.raycaster.intersectObjects(this.hitTargets);
    if (hits.length) {
      const index = hits[0].object.userData.frameIndex; this.onSelect?.(index);
      this.container.dispatchEvent(new CustomEvent('frame-select', { detail: index }));
    }
  }

  reset() { this.setView('perspective'); }

  /** Explicitly frame the current clip without changing the viewing direction. */
  fit() {
    const direction = this.camera.position.clone().sub(this.controls.target);
    this.cameraController.fit(this.bounds, {direction});
    this.render();
  }

  setView(view) {
    this.cameraController.fit(this.bounds, {view});
    this.render();
  }

  resize() { this.viewport.resize(); }

  render() { if (!this.disposed) this.viewport.render(); }

  dispose() {
    if (this.disposed) return;
    this.assetController.abort();
    this.disposed = true;
    this.renderer.domElement.removeEventListener('pointerdown', this.pointerDown);
    this.renderer.domElement.removeEventListener('pointerup', this.pointerUp);
    this.viewport.dispose(); this.robot?.dispose();
  }
}
