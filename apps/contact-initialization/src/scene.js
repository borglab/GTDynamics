import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

// Everything in this viewer stays in the robot's native, Z-up coordinate system.
const COLORS = { ink: '#243438', silver: '#94a4a3', left: '#13836e', right: '#df785c', paper: '#f0f2ed', path: '#778b87', reference: '#a6a7b6' };
const Z = new THREE.Vector3(0, 0, 1);
const Y = new THREE.Vector3(0, 1, 0);
const vec = (a) => new THREE.Vector3(...(a ?? [0, 0, 0]));
const mean = (points) => points.reduce((a, b) => a.add(vec(b)), new THREE.Vector3()).multiplyScalar(1 / Math.max(1, points.length));

function material(color, opacity = 1, extras = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.12,
    transparent: opacity < 1, opacity, depthWrite: opacity > 0.7, ...extras });
}

function release(object) {
  const geometries = new Set();
  const materials = new Set();
  object.traverse((child) => {
    if (child.geometry) geometries.add(child.geometry);
    if (child.material) for (const item of [child.material].flat()) materials.add(item);
  });
  for (const geometry of geometries) geometry.dispose();
  for (const item of materials) { item.map?.dispose(); item.dispose(); }
}

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

function capsuleBetween(a, b, radius, style, radial = 8) {
  const distance = a.distanceTo(b);
  if (distance < 1e-5) return new THREE.Group();
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius * 0.86, radius, distance, radial), style);
  mesh.position.copy(a).add(b).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(Y, b.clone().sub(a).normalize());
  mesh.castShadow = style.opacity > 0.7;
  return mesh;
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

function samplePoints(sample, model) {
  const result = {};
  if (sample?.bodyPoints) (model?.bodyNames ?? []).forEach((name, i) => { result[name] = vec(sample.bodyPoints[i]); });
  for (const [name, value] of Object.entries(sample?.points ?? sample?.joints ?? {})) result[name] = vec(value);
  return result;
}

function bodyPoint(points, names, fallback) {
  for (const name of names) if (points[name]) return points[name].clone();
  return vec(fallback);
}

/** A mesh-free G1 silhouette whose joints are placed using the recorded FK. */
function robot(sample, model, opacity = 1, reference = false) {
  const group = new THREE.Group();
  const points = samplePoints(sample, model);
  const ink = material(reference ? COLORS.reference : COLORS.ink, opacity,{wireframe:reference});
  const silver = material(reference ? COLORS.reference : COLORS.silver, opacity,{wireframe:reference});
  const light = material(reference ? COLORS.reference : '#d8dfd9', opacity,{wireframe:reference});
  const pelvis = bodyPoint(points, ['pelvis', 'pelvis_link', 'base'], [0, 0, 0]);
  const chest = bodyPoint(points, ['torso_link', 'torso', 'waist_roll_link'], [0, 0, 0.04]);
  // The G1 torso link is the waist origin. Shoulder FK also supplies the shell's
  // orientation, including waist bends, without requiring visual mesh assets.
  const leftShoulder = bodyPoint(points, ['left_shoulder_pitch_link', 'left_shoulder_roll_link'], [chest.x, 0.19, chest.z + 0.255]);
  const rightShoulder = bodyPoint(points, ['right_shoulder_pitch_link', 'right_shoulder_roll_link'], [chest.x, -0.19, chest.z + 0.255]);
  const shoulderCenter = leftShoulder.clone().add(rightShoulder).multiplyScalar(0.5);
  const torsoUp = shoulderCenter.clone().sub(chest).normalize();
  const torsoAcross = leftShoulder.clone().sub(rightShoulder).normalize();
  const torsoForward = new THREE.Vector3().crossVectors(torsoAcross, torsoUp).normalize();
  torsoAcross.crossVectors(torsoUp, torsoForward).normalize();
  const torsoOrientation = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(torsoForward, torsoAcross, torsoUp));
  const upperBody = new THREE.Group(); upperBody.position.copy(chest); upperBody.quaternion.copy(torsoOrientation); group.add(upperBody);
  const shoulderHeight = shoulderCenter.distanceTo(chest);
  const hipShell = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.235, 0.145), silver);
  hipShell.position.copy(pelvis).add(new THREE.Vector3(0, 0, 0.025)); group.add(hipShell);
  const torso = new THREE.Mesh(new THREE.BoxGeometry(0.165, 0.24, Math.max(0.18, shoulderHeight)), ink);
  torso.position.set(0, 0, shoulderHeight / 2); torso.castShadow = opacity > 0.7; upperBody.add(torso);
  const plate = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.165, 0.165), light);
  plate.position.set(0.087, 0, shoulderHeight / 2 + 0.015); upperBody.add(plate);
  const neck = new THREE.Vector3(0, 0, shoulderHeight + 0.055);
  upperBody.add(capsuleBetween(new THREE.Vector3(0, 0, shoulderHeight), neck, 0.035, silver));
  const head = new THREE.Mesh(new THREE.SphereGeometry(0.09, 16, 12), light);
  head.scale.set(0.8, 0.85, 1.06); head.position.copy(neck).add(new THREE.Vector3(0.005, 0, 0.067)); upperBody.add(head);
  const visor = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.123, 0.053), ink);
  visor.position.copy(head.position).add(new THREE.Vector3(0.065, 0, 0.005)); upperBody.add(visor);

  for (const [side, sign] of [['left', 1], ['right', -1]]) {
    const accent = material(reference ? COLORS.reference : COLORS[side], opacity);
    const hip = bodyPoint(points, [`${side}_hip_roll_link`, `${side}_hip_pitch_link`, `${side}_hip`], [0, sign * 0.095, -0.045]);
    const knee = bodyPoint(points, [`${side}_knee_link`, `${side}_knee`], [0, sign * 0.095, -0.34]);
    const ankle = bodyPoint(points, [`${side}_ankle_roll_link`, `${side}_ankle_pitch_link`, `${side}_ankle`], [0, sign * 0.095, -0.67]);
    group.add(capsuleBetween(hip, knee, 0.048, silver));
    group.add(capsuleBetween(knee, ankle, 0.036, ink));
    group.add(sphere(hip, 0.053, ink), sphere(knee, 0.043, accent), sphere(ankle, 0.029, silver));
    const shinPanel = knee.clone().lerp(ankle, 0.42).add(new THREE.Vector3(0.025, 0, 0));
    group.add(capsuleBetween(shinPanel.clone().lerp(knee, 0.5), shinPanel.clone().lerp(ankle, 0.6), 0.019, accent));
    const foot = new THREE.Mesh(new THREE.BoxGeometry(0.19, 0.075, 0.045), ink);
    const footState = sample?.feet?.[side];
    if (footState?.q) foot.quaternion.set(footState.q[1], footState.q[2], footState.q[3], footState.q[0]);
    foot.position.copy(footState?.p ? vec(footState.p) : ankle).add(new THREE.Vector3(0.04, 0, -0.026).applyQuaternion(foot.quaternion));
    foot.castShadow = opacity > 0.7; group.add(foot);
    const shoulder = bodyPoint(points, [`${side}_shoulder_roll_link`, `${side}_shoulder_pitch_link`, `${side}_shoulder`], [chest.x, sign * 0.19, chest.z + 0.23]);
    const elbow = bodyPoint(points, [`${side}_elbow_link`, `${side}_elbow`], [chest.x, sign * 0.22, chest.z + 0.03]);
    const wrist = bodyPoint(points, [`${side}_wrist_yaw_link`, `${side}_wrist_roll_link`, `${side}_wrist`], [chest.x + 0.04, sign * 0.22, chest.z - 0.12]);
    group.add(capsuleBetween(shoulder, elbow, 0.027, silver), capsuleBetween(elbow, wrist, 0.024, ink));
    group.add(sphere(shoulder, 0.04, ink), sphere(elbow, 0.03, silver), sphere(wrist, 0.025, ink));
  }
  return group;
}

function contactPoints(contact) {
  if (contact.points?.length) return contact.points.map((p) => p.position ?? p.point ?? p);
  if (contact.position || contact.p) return [contact.position ?? contact.p];
  return [];
}

function footprint(contact, active = false, projectToFloor = true) {
  const points = contactPoints(contact);
  if (!points.length) return null;
  const group = new THREE.Group();
  const color = COLORS[contact.foot ?? contact.side] ?? COLORS.left;
  const center = mean(points);
  // Markers stay visible on the floor; the actual 3D landmarks are small dots.
  const outline = points.length > 2 ? points.map((p) => [p[0], p[1], projectToFloor ? 0.006 : p[2]])
    : [[center.x - 0.08, center.y - 0.035, 0.006], [center.x + 0.10, center.y - 0.035, 0.006], [center.x + 0.10, center.y + 0.035, 0.006], [center.x - 0.08, center.y + 0.035, 0.006]];
  // The graph needs only three noncollinear sole landmarks. Complete their
  // parallelogram for the footprint while retaining exactly three point dots.
  if (outline.length === 3) outline.push(outline[0].map((v,i)=>v+outline[2][i]-outline[1][i]));
  // Corner order is geometrical, so graph point IDs can be in any order.
  if(projectToFloor) {
    const shape = new THREE.Shape(outline.map((p) => new THREE.Vector2(p[0], p[1])));
    const patch = new THREE.Mesh(new THREE.ShapeGeometry(shape), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: active ? 0.23 : 0.09, depthWrite: false, side: THREE.DoubleSide }));
    patch.position.z = 0.004; group.add(patch);
  }
  group.add(line([...outline, outline[0]], color, active ? 1 : 0.55));
  for (const point of points) group.add(sphere(projectToFloor?vec(point).setZ(Math.max(0.009,point[2])):vec(point), active ? 0.013 : 0.008, material(color, active ? 1 : 0.65)));
  return group;
}

/**
 * Rendering contract:
 *  model: {bodyNames}; samples: [{bodyPoints, feet:{left:{p,q},right:{p,q}}}]
 *  frames: [{sampleIndex, observations:[{local,contactId}]}]
 *  poses: solver {translation, rotation:rowMajor9}; referencePoses: same, or {p,q:wxyz}
 *  contacts: [{id,foot,points:[[x,y,z],...]}]; selectedIndex indexes frames.
 */
export class ContactScene {
  constructor(container, { onSelect } = {}) {
    this.container = container; this.onSelect = onSelect; this.state = {}; this.disposed = false;
    this.scene = new THREE.Scene(); this.scene.background = new THREE.Color(COLORS.paper);
    this.scene.fog = new THREE.Fog(COLORS.paper, 9, 24);
    this.camera = new THREE.PerspectiveCamera(35, 1, 0.02, 60); this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputEncoding = THREE.sRGBEncoding;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping; this.renderer.toneMappingExposure = 0.95;
    this.renderer.shadowMap.enabled = true; this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.domElement.setAttribute('aria-label', 'Interactive 3D reconstruction of the G1 walking sequence');
    this.renderer.domElement.style.display = 'block';
    this.container.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = false; this.controls.minDistance = 0.8; this.controls.maxDistance = 18;
    this.controls.maxPolarAngle = Math.PI * 0.488; this.controls.screenSpacePanning = false;
    this.controls.addEventListener('change', () => this.render());
    this.scene.add(new THREE.HemisphereLight('#f7fbff', '#a4b0a3', 1.5));
    const key = new THREE.DirectionalLight('#fff8e9', 2.1); key.position.set(0, -3, 7); key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048); key.shadow.bias = -0.0005; key.shadow.normalBias = 0.015;
    Object.assign(key.shadow.camera, { left: -6, right: 6, bottom: -5, top: 5, near: 0.1, far: 20 });
    this.scene.add(key);
    const fill = new THREE.DirectionalLight('#d6e9ed', 0.8); fill.position.set(-4, 3, 4); this.scene.add(fill);
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(70, 70), material('#edf0e9', 1, { metalness: 0, roughness: 1 }));
    ground.position.z = -0.01; ground.receiveShadow = true; this.scene.add(ground); this.ground=ground;
    const grid = new THREE.GridHelper(30, 120, '#b5c2b7', '#d6ddd3'); grid.rotation.x = Math.PI / 2;
    grid.position.z = -0.007; grid.material.transparent = true; grid.material.opacity = 0.6; this.scene.add(grid);
    this.pathGroup = new THREE.Group(); this.contactGroup = new THREE.Group(); this.ghostGroup = new THREE.Group();
    this.selectedGroup = new THREE.Group(); this.labelGroup = new THREE.Group();
    this.scene.add(this.pathGroup, this.contactGroup, this.ghostGroup, this.selectedGroup, this.labelGroup);
    this.raycaster = new THREE.Raycaster(); this.pointer = new THREE.Vector2(); this.hitTargets = [];
    this.pointerDown = (event) => { this.down = [event.clientX, event.clientY]; };
    this.pointerUp = (event) => this.pick(event);
    this.renderer.domElement.addEventListener('pointerdown', this.pointerDown);
    this.renderer.domElement.addEventListener('pointerup', this.pointerUp);
    this.resizeObserver = new ResizeObserver(() => this.resize()); this.resizeObserver.observe(container);
    this.bounds = { center: new THREE.Vector3(0, 0, 0.6), span: 2 }; this.reset(); this.resize();
  }

  update(next) {
    if (this.disposed) return;
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
    const box = new THREE.Box3().setFromPoints(positions); const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    center.z = this.state.floorConstrained === false ? center.z + 0.4 : 0.55;
    this.bounds = { center, span: Math.max(1.7, size.x, size.y, size.z+1.1) };
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
      const ghost = robot(this.sample(frame), model, 0.07); ghost.applyMatrix4(poseMatrix(poses[index])); this.ghostGroup.add(ghost);
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
    const body = robot(this.sample(frame), model); body.applyMatrix4(matrix); this.selectedGroup.add(body);
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
      const points = contactPoints(contact); if (!points.length) continue;
      const center = mean(points); if(this.state.floorConstrained!==false)center.z = Math.max(0.025, center.z);
      const side = contact.foot ?? contact.side; const color = COLORS[side] ?? COLORS.left;
      this.selectedGroup.add(line([root, center], color, 0.55, true));
      const note = caption(`${side === 'right' ? 'R' : 'L'}${String(contact.id).replace(/^[LRlr]/, '')}`, color, 0.12);
      note.position.copy(center).add(new THREE.Vector3(0, side === 'right' ? -0.13 : 0.13, 0.055)); this.selectedGroup.add(note);
    }
    if (this.state.showReference !== false) {
      const reference = this.referencePose(frame, index);
      if (reference) { const body = robot(this.sample(frame), model, 0.18, true); body.applyMatrix4(poseMatrix(reference)); this.selectedGroup.add(body); }
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

  setView(view) {
    const { center, span } = this.bounds; const distance = Math.max(2.7, span * 1.1);
    this.controls.target.copy(center);
    const offset = view === 'top' ? new THREE.Vector3(0, -0.001, distance * 1.6)
      : view === 'side' ? new THREE.Vector3(0.001, -distance * 1.5, 0.3)
        : view === 'front' ? new THREE.Vector3(distance * 1.5, 0.001, 0.3)
          : new THREE.Vector3(distance * 0.7, -distance * 1.25, distance * 0.62);
    this.camera.position.copy(center).add(offset); this.controls.update(); this.render();
  }

  resize() {
    const width = Math.max(1, this.container.clientWidth); const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height; this.camera.updateProjectionMatrix(); this.renderer.setSize(width, height, false); this.render();
  }

  render() { if (!this.disposed) this.renderer.render(this.scene, this.camera); }

  dispose() {
    this.disposed = true; this.resizeObserver.disconnect(); this.controls.dispose();
    this.renderer.domElement.removeEventListener('pointerdown', this.pointerDown);
    this.renderer.domElement.removeEventListener('pointerup', this.pointerUp);
    release(this.scene); this.renderer.dispose(); this.renderer.domElement.remove();
  }
}
