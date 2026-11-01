import * as THREE from 'three';
import { RobotViewport, disposeObject3D } from '../../shared/RobotViewport.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import loadMujoco from 'mujoco-js';
import { createRobotVisual, updateRobotBodies } from '../../shared/mujocoRobot.js';

THREE.ColorManagement.enabled = true;

// Geometry comes from the compiled MuJoCo model in native Z-up coordinates.
const MODEL_ROOT = '/working';
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const vec = (p) => new THREE.Vector3().fromArray(p);
const poseOf = (object) => ({
  position: object.position.toArray(), quaternion: object.quaternion.toArray(),
});

const releaseTree = disposeObject3D;

function label(text, color) {
  const canvas = document.createElement('canvas');
  canvas.width = 384;
  canvas.height = 96;
  const context = canvas.getContext('2d');
  context.fillStyle = color;
  context.font = '600 38px system-ui, sans-serif';
  context.textAlign = 'center';
  context.fillText(text, 192, 58);
  const texture = new THREE.CanvasTexture(canvas);
  texture.encoding = THREE.sRGBEncoding;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, toneMapped: false }));
  sprite.scale.set(0.25, 0.0625, 1);
  return sprite;
}

function frame(color, name) {
  const group = new THREE.Group();
  const material = new THREE.MeshBasicMaterial({ color, depthTest: false, toneMapped: false });
  const center = new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 12), material);
  center.userData.target = name;
  center.renderOrder = 10;
  group.add(center);
  const axes = new THREE.AxesHelper(0.12);
  axes.material.depthTest = false;
  axes.renderOrder = 10;
  group.add(axes);
  // The ring distinguishes start and goal even when RGB axes overlap.
  const ring = new THREE.Mesh(new THREE.TorusGeometry(0.035, 0.003, 6, 40), material);
  ring.userData.target = name;
  ring.renderOrder = 10;
  group.add(ring);
  const caption = label(name.toUpperCase(), `#${new THREE.Color(color).getHexString()}`);
  caption.position.set(0, 0, 0.16);
  group.add(caption);
  group.userData.caption = caption;
  return group;
}

// Arrival is measured against the draggable world frame while physics keeps
// integrating. A bounded hold makes unreachable/contact-blocked targets clear.
const ENDPOINT_HOLD = Object.freeze({
  positionTolerance: 1e-4, rotationTolerance: 1e-3, velocityTolerance: 0.005,
  stableDuration: 0.1, maxDuration: 2,
});

/** Hermite position and its analytic time derivative, in trajectory seconds. */
export function sampleTrajectoryState(snapshot, time) {
  const states = snapshot?.states;
  if (!states?.length) return null;
  if (states.length === 1) return { q: states[0].q.slice(), v: states[0].q.map(() => 0) };
  const duration = Math.max(0.001, Number(snapshot.duration));
  const dt = duration / (states.length - 1);
  const scaled = clamp(time / dt, 0, states.length - 1);
  const i = Math.min(states.length - 2, Math.floor(scaled));
  const s = scaled - i;
  const a = states[i];
  const b = states[i + 1];
  const h00 = 2 * s ** 3 - 3 * s ** 2 + 1;
  const h10 = s ** 3 - 2 * s ** 2 + s;
  const h01 = -2 * s ** 3 + 3 * s ** 2;
  const h11 = s ** 3 - s ** 2;
  return {
    q: a.q.map((q, j) => h00 * q + h01 * b.q[j]
      + dt * (h10 * (a.v?.[j] ?? 0) + h11 * (b.v?.[j] ?? 0))),
    v: a.q.map((q, j) => 6 * s * (1 - s) * (b.q[j] - q) / dt
      + (3 * s ** 2 - 4 * s + 1) * (a.v?.[j] ?? 0)
      + (3 * s ** 2 - 2 * s) * (b.v?.[j] ?? 0)),
  };
}

/** Joint-space interpolation uses the velocities actually optimized by C++. */
export function sampleTrajectory(snapshot, time) {
  return sampleTrajectoryState(snapshot, time)?.q ?? null;
}

export class MotionPlanningViewer {
  constructor(container, { onTargetChange, onTimeChange, onSelectionChange } = {}) {
    this.container = container;
    this.container.motionPlanningViewer = this;
    this.onTargetChange = onTargetChange;
    this.onTimeChange = onTimeChange;
    this.onSelectionChange = onSelectionChange;
    this.playing = false;
    this.time = 0;
    this.speed = 1;
    this.direction = 1;
    this.endpointHold = null;
    this.lastEndpointArrival = null;
    this.accumulator = 0;
    this.disposed = false;
    this.lastUiUpdate = 0;
    this.viewport = new RobotViewport(container, {
      background: '#101b25', fog: [4, 9], fov: 38, near: .01, far: 30,
      shadows: true, exposure: .9,
      controls: {enableDamping: true, maxDistance: 5, maxPolarAngle: Math.PI * .48},
    });
    const {scene, camera, renderer, controls, cameraController} = this.viewport;
    Object.assign(this, {scene, camera, renderer, controls, cameraController});
    this.cameraController.lookAt(new THREE.Vector3(.25, 0, .45), new THREE.Vector3(1.55, -2.1, 1.1));
    this.viewport.addStage({
      hemisphere: [0xcfe7ff, 0x33444b, .5],
      key: {color: 0xffffff, intensity: 1, position: [2, -3, 5]},
      fill: {color: 0x83adff, intensity: .3, position: [-2, 1, 2]},
      shadow: {left: -1.6, right: 1.6, top: 1.6, bottom: -1.6, near: .1, far: 10, bias: -.0003},
      ground: {size: 20, color: 0x15232f, z: -.002},
      grid: {size: 4, divisions: 40, colors: [0x51616b, 0x293d4d]},
    });
    const worldAxes = new THREE.AxesHelper(0.2);
    worldAxes.position.set(-0.36, -0.36, 0.002);
    this.scene.add(worldAxes);
    this.frames = { start: frame('#ff5e66', 'start'), goal: frame('#52d997', 'goal') };
    this.scene.add(this.frames.start, this.frames.goal);
    this.transform = new TransformControls(this.camera, this.renderer.domElement);
    this.transform.setSize(0.8);
    this.transform.setSpace('world');
    this.transform.addEventListener('dragging-changed', ({ value }) => {
      this.controls.enabled = !value;
    });
    this.transform.addEventListener('objectChange', () => {
      if (this.endpointHold?.name === this.selection) this.restartEndpointHold();
      this.onTargetChange?.({ start: poseOf(this.frames.start), goal: poseOf(this.frames.goal) });
    });
    this.scene.add(this.transform);
    this.setSelection('goal');
    this.targetClick = (event) => {
      if (this.transform.dragging || this.transform.axis) return;
      const rect = this.renderer.domElement.getBoundingClientRect();
      const ray = new THREE.Raycaster();
      ray.setFromCamera(new THREE.Vector2(
        (event.clientX - rect.left) / rect.width * 2 - 1,
        -(event.clientY - rect.top) / rect.height * 2 + 1), this.camera);
      const hit = ray.intersectObjects(Object.values(this.frames), true)
        .find((entry) => entry.object.userData.target);
      if (hit) this.setSelection(hit.object.userData.target);
    };
    this.renderer.domElement.addEventListener('pointerdown', this.targetClick);
    this.trajectoryGroup = new THREE.Group();
    this.ghostGroup = new THREE.Group();
    this.scene.add(this.trajectoryGroup, this.ghostGroup);
    this.resize();
    this.lastFrame = performance.now();
    this.viewport.start(now => this.update(now));
  }

  async init(description) {
    this.description = description;
    const response = await fetch('./assets/manifest.json');
    if (!response.ok || !response.headers.get('content-type')?.includes('json')) {
      throw new Error('Panda assets are missing. Run python apps/motion-planning/scripts/bootstrap_models.py, then reload.');
    }
    const manifest = await response.json();
    const mujoco = await loadMujoco();
    if (this.disposed) return;
    this.mujoco = mujoco;
    mujoco.FS.mkdir(MODEL_ROOT);
    // Download a bounded number at a time; large CAD meshes otherwise exhaust
    // browser connections and create excessive temporary ArrayBuffers.
    let next = 0;
    await Promise.all(Array.from({ length: 6 }, async () => {
      while (next < manifest.files.length) {
        const file = manifest.files[next++];
        const asset = await fetch(`./assets/${file}`);
        if (!asset.ok) throw new Error(`Could not load Panda asset ${file}: ${asset.status}`);
        const path = `${MODEL_ROOT}/${file}`;
        mujoco.FS.mkdirTree(path.slice(0, path.lastIndexOf('/')));
        const bytes = new Uint8Array(await asset.arrayBuffer());
        // MuJoCo counts gravity-compensated bodies at compile time. Enable it
        // in the in-memory XML, leaving the pinned source assets untouched.
        const contents = file.endsWith('/panda.xml')
          ? new TextDecoder().decode(bytes).replace(/<body(?=\s)/g, '<body gravcomp="1"')
          : bytes;
        mujoco.FS.writeFile(path, contents);
      }
    }));
    if (this.disposed) return;
    this.model = mujoco.MjModel.loadFromXML(`${MODEL_ROOT}/${manifest.scene}`);
    this.data = new mujoco.MjData(this.model);
    this.targetData = new mujoco.MjData(this.model);
    this.timestep = this.model.opt.timestep;
    this.joints = (description.joint_names ?? Array.from({ length: 7 }, (_, j) => `joint${j + 1}`))
      .map((name) => this.findName('jnt', this.model.njnt, name));
    this.qposAddresses = this.joints.map((j) => this.model.jnt_qposadr[j]);
    this.dofAddresses = this.joints.map((j) => this.model.jnt_dofadr[j]);
    this.actuators = this.joints.map((joint) => {
      for (let a = 0; a < this.model.nu; a += 1) {
        if (this.model.actuator_trnid[2 * a] === joint) return a;
      }
      throw new Error(`Panda joint ${joint} has no position actuator`);
    });
    this.handBody = this.findName('body', this.model.nbody, 'hand');
    // The simulated arm uses its real position actuators with gravity
    // compensation, as a Franka position interface does. mj_step integrates
    // inertia, joint limits, damping and contacts during replay.
    this.createRobotMeshes();
    this.instantPose(description.home ?? [0, -0.7854, 0, -2.3562, 0, 1.5708, 0.7854]);
    this.setTargets(description.start, description.goal);
    this.emitTime();
  }

  findName(kind, count, wanted) {
    const names = new Uint8Array(this.model.names);
    const decoder = new TextDecoder();
    for (let i = 0; i < count; i += 1) {
      const begin = this.model[`name_${kind}adr`][i];
      let end = begin;
      while (end < names.length && names[end]) end += 1;
      if (decoder.decode(names.subarray(begin, end)) === wanted) return i;
    }
    throw new Error(`MuJoCo Panda does not contain ${kind} ${wanted}`);
  }

  createRobotMeshes() {
    const visual = createRobotVisual(this.model);
    this.bodies = visual.bodies;
    this.robotRoot = visual.root;
    this.scene.add(this.robotRoot);
    this.tcpAxes = new THREE.AxesHelper(0.09);
    this.scene.add(this.tcpAxes);
  }

  instantPose(q) {
    if (!this.data) return;
    this.data.qvel.fill(0);
    q.forEach((value, i) => {
      this.data.qpos[this.qposAddresses[i]] = value;
      this.data.ctrl[this.actuators[i]] = value;
    });
    // Open the fingers; these are not variables in the seven-DOF graph.
    if (this.model.nq >= 9) {
      this.data.qpos[7] = 0.035;
      this.data.qpos[8] = 0.035;
    }
    if (this.model.nu > 7) this.data.ctrl[7] = 255;
    this.mujoco.mj_forward(this.model, this.data);
    this.command = q.slice();
  }

  setTargets(start, goal) {
    for (const [name, pose] of Object.entries({ start, goal })) {
      if (!pose) continue;
      const previous = poseOf(this.frames[name]);
      if (this.endpointHold?.name === name && (pose.position.some((v, i) => Math.abs(v - previous.position[i]) > 1e-12)
          || pose.quaternion.some((v, i) => Math.abs(v - previous.quaternion[i]) > 1e-12))) {
        this.restartEndpointHold();
      }
      this.frames[name].position.fromArray(pose.position);
      this.frames[name].quaternion.fromArray(pose.quaternion).normalize();
    }
  }

  setSelection(which) {
    if (this.frames[which]) {
      this.selection = which;
      this.onSelectionChange?.(which);
      if (this.editable !== false) this.transform.attach(this.frames[which]);
    }
  }

  setEditable(editable) {
    this.editable = editable;
    if (!editable) this.transform.detach();
    else this.setSelection(this.selection);
  }

  setTransformMode(mode) {
    this.transform.setMode(mode === 'rotate' ? 'rotate' : 'translate');
    this.transform.setSpace(mode === 'rotate' ? 'local' : 'world');
  }

  setTrajectory(snapshot) {
    if (!snapshot?.states?.length) return;
    this.snapshot = snapshot;
    this.duration = Number(snapshot.duration);
    this.time = this.endpointHold ? (this.endpointHold.name === 'goal' ? this.duration : 0)
      : clamp(this.time, 0, this.duration);
    this.restartEndpointHold();
    releaseTree(this.trajectoryGroup);
    releaseTree(this.ghostGroup);
    this.trajectoryGroup.clear();
    this.ghostGroup.clear();
    const points = snapshot.states.map((state) => vec(state.ee.position));
    const path = new THREE.BufferGeometry().setFromPoints(points);
    this.trajectoryGroup.add(new THREE.Line(path, new THREE.LineBasicMaterial({ color: '#64d9eb' })));
    const positions = new THREE.BufferGeometry().setFromPoints(points);
    const colors = new Float32Array(points.length * 3);
    points.forEach((_, i) => {
      new THREE.Color('#ff5e66').lerp(new THREE.Color('#52d997'), i / Math.max(1, points.length - 1))
        .toArray(colors, 3 * i);
    });
    positions.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    this.trajectoryGroup.add(new THREE.Points(positions,
      new THREE.PointsMaterial({ size: 0.012, vertexColors: true, depthTest: false })));
    // Show every slice's pose estimate, including temporary disagreement with
    // joint FK during optimization. These are the actual graph variables.
    snapshot.states.forEach((state, i) => {
      const links = state.links?.map((pose) => vec(pose.position)) ?? [];
      if (links.length) {
        links.push(vec(state.ee.position));
        this.ghostGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(links),
          new THREE.LineBasicMaterial({ color: 0x7bbccf, transparent: true,
            opacity: 0.08 + 0.20 * (i / Math.max(1, snapshot.states.length - 1)), depthWrite: false })));
      }
      const axes = new THREE.AxesHelper(0.035);
      axes.position.fromArray(state.ee.position);
      axes.quaternion.fromArray(state.ee.quaternion);
      axes.material.transparent = true;
      axes.material.opacity = 0.5;
      this.ghostGroup.add(axes);
    });
    if (!this.playing) this.instantPose(sampleTrajectory(snapshot, this.time));
    this.emitTime();
  }

  setPlaying(playing) {
    this.playing = Boolean(playing && this.snapshot?.states?.length);
    this.accumulator = 0;
    if (this.playing && this.endpointHold?.timedOut) this.restartEndpointHold();
    this.emitTime();
  }

  setTime(seconds) {
    this.endpointHold = null;
    this.time = clamp(Number(seconds) || 0, 0, this.duration || 0);
    this.instantPose(sampleTrajectory(this.snapshot, this.time) ?? this.description?.home ?? []);
    this.emitTime();
  }

  setSpeed(speed) { this.speed = clamp(Number(speed) || 1, 0.1, 3); }
  setGhosts(visible) { this.ghostGroup.visible = Boolean(visible); }

  restartEndpointHold() {
    if (!this.endpointHold) return;
    if (!this.playing) this.endpointHold = null;
    else Object.assign(this.endpointHold, { elapsed: 0, stable: 0, timedOut: false });
  }

  endpointError(name) {
    const actual = this.tcpPose(this.data);
    const target = this.frames[name];
    const orientation = new THREE.Quaternion().fromArray(actual.quaternion);
    return {
      position: vec(actual.position).distanceTo(target.position),
      rotation: orientation.angleTo(target.quaternion),
      velocity: Math.max(...this.dofAddresses.map((address) => Math.abs(this.data.qvel[address]))),
    };
  }

  step() {
    if (!this.endpointHold) {
      this.time += this.direction * this.speed * this.timestep;
      if (this.time >= this.duration || this.time <= 0) {
        const name = this.direction > 0 ? 'goal' : 'start';
        this.time = name === 'goal' ? this.duration : 0;
        this.endpointHold = { name, elapsed: 0, stable: 0, timedOut: false };
      }
    }
    const desired = sampleTrajectoryState(this.snapshot, this.time);
    desired.q.forEach((value, i) => {
      const actuator = this.actuators[i];
      // Menagerie's position servo computes kp*(ctrl-q) - kd*qdot. Adding
      // kd/kp * desired_qdot gives kp*(q_desired-q) + kd*(qdot_desired-qdot),
      // removing its otherwise approximately 100 ms velocity-dependent lag.
      const kp = this.model.actuator_gainprm[10 * actuator];
      const kd = -this.model.actuator_biasprm[10 * actuator + 2];
      const velocity = this.endpointHold ? 0 : desired.v[i] * this.direction * this.speed;
      const ctrl = value + (kp > 0 ? kd / kp * velocity : 0);
      // Retargeting and new optimizer snapshots keep physical qpos/qvel intact.
      // The same actuator command slew limit applies to feedforward as position.
      this.command[i] += clamp(ctrl - this.command[i], -2.5 * this.timestep, 2.5 * this.timestep);
      this.command[i] = clamp(this.command[i],
        this.model.actuator_ctrlrange[2 * actuator], this.model.actuator_ctrlrange[2 * actuator + 1]);
      this.data.ctrl[actuator] = this.command[i];
    });
    this.mujoco.mj_step(this.model, this.data);
    // mj_step leaves xpos at the pre-integration configuration. Refresh just
    // kinematics so arrival tests, render poses and diagnostics use current qpos.
    this.mujoco.mj_kinematics(this.model, this.data);
    if (this.endpointHold) {
      const hold = this.endpointHold;
      const error = this.endpointError(hold.name);
      hold.elapsed += this.timestep;
      const reached = error.position <= ENDPOINT_HOLD.positionTolerance
        && error.rotation <= ENDPOINT_HOLD.rotationTolerance
        && error.velocity <= ENDPOINT_HOLD.velocityTolerance;
      hold.stable = reached ? hold.stable + this.timestep : 0;
      if (hold.stable >= ENDPOINT_HOLD.stableDuration) {
        this.lastEndpointArrival = { name: hold.name, elapsed: hold.elapsed,
          positionError: error.position, rotationError: error.rotation };
        this.direction = hold.name === 'goal' ? -1 : 1;
        this.endpointHold = null;
      } else if (hold.elapsed >= ENDPOINT_HOLD.maxDuration) {
        hold.timedOut = true;
        this.playing = false;
      }
    }
  }

  emitTime() {
    const target = sampleTrajectory(this.snapshot, this.time);
    const trackingError = target && this.data
      ? Math.sqrt(target.reduce((sum, q, i) => sum + (q - this.data.qpos[this.qposAddresses[i]]) ** 2, 0) / 7)
      : 0;
    const diagnostics = this.data && target ? this.getDiagnostics() : null;
    this.onTimeChange?.({ time: this.time, playing: this.playing, trackingError,
      tcpPositionError: diagnostics?.tcpPositionError ?? 0,
      endpointStatus: diagnostics?.endpointStatus ?? 'moving',
      endpointName: diagnostics?.endpointName ?? null,
      endpointPositionError: diagnostics?.endpointPositionError ?? null,
      endpointRotationError: diagnostics?.endpointRotationError ?? null });
  }

  tcpPose(data) {
    const b = this.handBody;
    const quaternion = new THREE.Quaternion(data.xquat[4 * b + 1], data.xquat[4 * b + 2],
      data.xquat[4 * b + 3], data.xquat[4 * b]);
    const position = new THREE.Vector3(0, 0, 0.1034).applyQuaternion(quaternion)
      .add(new THREE.Vector3().fromArray(data.xpos, 3 * b));
    return { position: position.toArray(), quaternion: quaternion.toArray() };
  }

  getDiagnostics() {
    if (!this.data) return null;
    const targetQ = sampleTrajectory(this.snapshot, this.time) ?? this.command;
    this.targetData.qpos.set(this.data.qpos);
    targetQ.forEach((q, i) => { this.targetData.qpos[this.qposAddresses[i]] = q; });
    this.mujoco.mj_forward(this.model, this.targetData);
    const actualTcp = this.tcpPose(this.data);
    const targetTcp = this.tcpPose(this.targetData);
    const endpointError = this.endpointHold ? this.endpointError(this.endpointHold.name) : null;
    return {
      time: this.time, simulationTime: this.data.time, playing: this.playing,
      qpos: this.qposAddresses.map((address) => this.data.qpos[address]),
      ctrl: this.actuators.map((actuator) => this.data.ctrl[actuator]),
      targetQ, actualTcp, targetTcp,
      tcpPositionError: vec(actualTcp.position).distanceTo(vec(targetTcp.position)),
      qposAddresses: this.qposAddresses.slice(), actuators: this.actuators.slice(),
      gravityCompensatedBodies: this.model.ngravcomp,
      endpointStatus: this.endpointHold ? (this.endpointHold.timedOut ? 'unreached' : 'settling') : 'moving',
      endpointName: this.endpointHold?.name ?? null,
      endpointPositionError: endpointError?.position ?? null,
      endpointRotationError: endpointError?.rotation ?? null,
      endpointHoldTime: this.endpointHold?.elapsed ?? 0,
      lastEndpointArrival: this.lastEndpointArrival ? { ...this.lastEndpointArrival } : null,
      trajectoryStates: this.snapshot?.states?.length ?? 0,
      targets: { start: poseOf(this.frames.start), goal: poseOf(this.frames.goal) },
    };
  }

  resize() { this.viewport.resize(); }

  update(now) {
    if (this.disposed) return;
    const elapsed = clamp((now - this.lastFrame) / 1000, 0, 0.05);
    this.lastFrame = now;
    if (this.playing && this.data && this.snapshot) {
      this.accumulator += elapsed;
      while (this.playing && this.accumulator >= this.timestep) {
        this.step();
        this.accumulator -= this.timestep;
      }
    }
    if (this.data) {
      updateRobotBodies(this.bodies, this.data);
      const hand = this.bodies[this.handBody];
      this.tcpAxes.quaternion.copy(hand.quaternion);
      this.tcpAxes.position.set(0, 0, 0.1034).applyQuaternion(hand.quaternion).add(hand.position);
    }
    for (const frame of Object.values(this.frames)) {
      frame.userData.caption.position.set(0, 0, 0.16)
        .applyQuaternion(frame.quaternion.clone().invert());
    }
    if (now - this.lastUiUpdate > 60) {
      this.emitTime();
      this.lastUiUpdate = now;
    }
  }

  dispose() {
    this.disposed = true;
    this.renderer.domElement.removeEventListener('pointerdown', this.targetClick);
    this.transform.detach();
    this.transform.dispose();
    this.viewport.dispose();
    this.targetData?.delete();
    this.data?.delete();
    this.model?.delete();
    this.targetData = null;
    delete this.container.motionPlanningViewer;
    this.data = null;
    this.model = null;
  }
}
