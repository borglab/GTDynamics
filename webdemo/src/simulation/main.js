import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import {
  downloadExampleScenesFolder,
  getPosition,
  getQuaternion,
  reloadScene
} from './mujocoUtils.js';
import { ClipPlayer } from './clipPlayer.js';

function toFloatArray(value, size, fallback) {
  const array = new Float64Array(size);
  for (let i = 0; i < size; i += 1) {
    const candidate = Array.isArray(value) ? Number(value[i]) : Number(value);
    array[i] = Number.isFinite(candidate) ? candidate : fallback;
  }
  return array;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export class MuJoCoClipDemo {
  constructor(mujoco) {
    this.mujoco = mujoco;
    mujoco.FS.mkdir('/working');
    mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');

    this.model = null;
    this.data = null;
    this.simulation = null;
    this.bodies = {};
    this.lights = {};
    this.mujocoRoot = null;
    this.lastSimState = {
      bodies: new Map(),
      lights: new Map(),
      tendons: {
        numWraps: { count: 0 },
        matrix: new THREE.Matrix4()
      }
    };

    this.container = document.getElementById('mujoco-container');
    if (!this.container) {
      throw new Error('Expected a #mujoco-container element in the page');
    }

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0.14, 0.18, 0.22);

    this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.001, 100);
    this.camera.position.set(3.0, 2.0, 3.0);
    this.scene.add(this.camera);

    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.2);
    this.scene.add(this.ambientLight);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderScale = 1.5;
    this.renderer.setPixelRatio(this.renderScale);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.8, 0);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;
    this.controls.update();

    this.reloadScene = reloadScene.bind(this);
    this.clipPlayer = new ClipPlayer(60);

    this.clipTime = 0;
    this.playing = false;
    this.loop = true;
    this.alive = false;

    this.clipJointNames = [];
    this.ctrlAdr = [];
    this.qposAdr = [];
    this.qvelAdr = [];
    this.clipJointMap = [];
    this.numActions = 0;

    this.kp = new Float64Array();
    this.kd = new Float64Array();
    this.defaultJointPos = new Float64Array();
    this.defaultRootPos = [0, 0, 0.8];
    this.defaultRootQuat = [1, 0, 0, 0];

    this.timestep = 1 / 60;
    this.decimation = 1;

    this.simStepHz = 0;
    this._stepFrameCount = 0;
    this._stepLastTime = performance.now();

    this.onWindowResize = this.onWindowResize.bind(this);
    window.addEventListener('resize', this.onWindowResize);
    this.renderer.setAnimationLoop(this.render.bind(this));
  }

  async init(defaultRobot = 'g1') {
    await downloadExampleScenesFolder(this.mujoco);
    await this.loadRobot(defaultRobot);
    this.alive = true;
    this.main_loop();
  }

  async loadRobot(robotName) {
    const configUrl = `./examples/configs/${robotName}.json`;
    const response = await fetch(configUrl);
    if (!response.ok) {
      throw new Error(`Failed to load robot config from ${configUrl}: ${response.status}`);
    }
    const config = await response.json();

    const scenePath = config.scene ?? `${robotName}/${robotName}.xml`;
    await this.reloadScene(scenePath);

    this.clipJointNames = Array.isArray(config.clip_joint_names) ? config.clip_joint_names.slice() : [];
    if (this.clipJointNames.length === 0) {
      this.clipJointNames = this.inferActuatedJointNames();
    }

    this.configureJointMappings(this.clipJointNames);
    this.kp = toFloatArray(config.stiffness, this.numActions, 30.0);
    this.kd = toFloatArray(config.damping, this.numActions, 1.0);

    if (Array.isArray(config.default_joint_pos) && config.default_joint_pos.length === this.numActions) {
      this.defaultJointPos = new Float64Array(config.default_joint_pos);
    } else {
      this.defaultJointPos = new Float64Array(this.numActions);
      this.defaultJointPos.fill(0.0);
    }

    if (Array.isArray(config.default_root_pos) && config.default_root_pos.length === 3) {
      this.defaultRootPos = config.default_root_pos.map((v) => Number(v));
    }
    if (Array.isArray(config.default_root_quat) && config.default_root_quat.length === 4) {
      this.defaultRootQuat = config.default_root_quat.map((v) => Number(v));
    }

    this.currentRobot = robotName;
    this.clipTime = 0;
    this.playing = false;
    this.applyDefaultPose();
  }

  inferActuatedJointNames() {
    const names = [];
    for (let i = 0; i < this.model.nu; i += 1) {
      const jointId = this.model.actuator_trnid[2 * i];
      names.push(this.jointNamesMJC[jointId]);
    }
    return names;
  }

  configureJointMappings(jointNames) {
    const jointTransmission = this.mujoco.mjtTrn.mjTRN_JOINT.value;
    const actuatorToJoint = [];

    for (let i = 0; i < this.model.nu; i += 1) {
      if (this.model.actuator_trntype[i] !== jointTransmission) {
        throw new Error(`Actuator ${i} transmission type is not mjTRN_JOINT`);
      }
      actuatorToJoint.push(this.model.actuator_trnid[2 * i]);
    }

    this.ctrlAdr = [];
    this.qposAdr = [];
    this.qvelAdr = [];

    for (const name of jointNames) {
      const jointIdx = this.jointNamesMJC.indexOf(name);
      if (jointIdx < 0) {
        throw new Error(`Joint '${name}' was not found in MuJoCo model`);
      }
      const actuatorIdx = actuatorToJoint.findIndex((id) => id === jointIdx);
      if (actuatorIdx < 0) {
        throw new Error(`No actuator mapped to joint '${name}'`);
      }
      this.ctrlAdr.push(actuatorIdx);
      this.qposAdr.push(this.model.jnt_qposadr[jointIdx]);
      this.qvelAdr.push(this.model.jnt_dofadr[jointIdx]);
    }

    this.numActions = jointNames.length;
    this.clipJointMap = Array.from({ length: this.numActions }, (_, i) => i);
  }

  async loadClipFromUrl(clipUrl, clipName = 'default') {
    const response = await fetch(clipUrl);
    if (!response.ok) {
      throw new Error(`Failed to load motion clip from ${clipUrl}: ${response.status}`);
    }
    const payload = await response.json();
    this.loadClipPayload(payload, clipName);
  }

  loadClipPayload(payload, clipName = 'default') {
    this.clipPlayer.loadClip(payload, { clipName });
    this.configureClipMapping();
    this.setTime(0, true);
  }

  configureClipMapping() {
    const clipJointNames = this.clipPlayer.jointNames;
    if (!Array.isArray(clipJointNames) || clipJointNames.length === 0) {
      this.clipJointMap = Array.from({ length: this.numActions }, (_, i) => i);
      return;
    }

    this.clipJointMap = this.clipJointNames.map((jointName, idx) => {
      const clipIdx = clipJointNames.indexOf(jointName);
      return clipIdx >= 0 ? clipIdx : idx;
    });
  }

  applyDefaultPose() {
    if (!this.simulation) {
      return;
    }
    const qpos = this.simulation.qpos;
    const qvel = this.simulation.qvel;

    if (qpos.length >= 7) {
      qpos[0] = this.defaultRootPos[0];
      qpos[1] = this.defaultRootPos[1];
      qpos[2] = this.defaultRootPos[2];
      qpos[3] = this.defaultRootQuat[0];
      qpos[4] = this.defaultRootQuat[1];
      qpos[5] = this.defaultRootQuat[2];
      qpos[6] = this.defaultRootQuat[3];
    }

    for (let i = 0; i < this.numActions; i += 1) {
      qpos[this.qposAdr[i]] = this.defaultJointPos[i] ?? 0.0;
      qvel[this.qvelAdr[i]] = 0.0;
    }

    this.simulation.forward();
  }

  applyClipPoseInstant(targets) {
    if (!targets || !this.simulation) {
      return;
    }

    const qpos = this.simulation.qpos;
    const qvel = this.simulation.qvel;

    if (Array.isArray(targets.rootPos) && targets.rootPos.length === 3 && qpos.length >= 3) {
      qpos[0] = Number(targets.rootPos[0]);
      qpos[1] = Number(targets.rootPos[1]);
      qpos[2] = Number(targets.rootPos[2]);
    }
    if (Array.isArray(targets.rootQuat) && targets.rootQuat.length === 4 && qpos.length >= 7) {
      qpos[3] = Number(targets.rootQuat[0]);
      qpos[4] = Number(targets.rootQuat[1]);
      qpos[5] = Number(targets.rootQuat[2]);
      qpos[6] = Number(targets.rootQuat[3]);
    }

    for (let i = 0; i < this.numActions; i += 1) {
      const clipIdx = this.clipJointMap[i];
      const desired = Number(targets.jointPos?.[clipIdx]);
      qpos[this.qposAdr[i]] = Number.isFinite(desired) ? desired : (this.defaultJointPos[i] ?? 0.0);
      qvel[this.qvelAdr[i]] = 0.0;
    }

    this.simulation.forward();
  }

  applyRootTarget(targets) {
    if (!targets || !this.simulation) {
      return;
    }

    const qpos = this.simulation.qpos;
    const qvel = this.simulation.qvel;
    let changed = false;

    if (Array.isArray(targets.rootPos) && targets.rootPos.length === 3 && qpos.length >= 3) {
      qpos[0] = Number(targets.rootPos[0]);
      qpos[1] = Number(targets.rootPos[1]);
      qpos[2] = Number(targets.rootPos[2]);
      changed = true;
    }
    if (Array.isArray(targets.rootQuat) && targets.rootQuat.length === 4 && qpos.length >= 7) {
      qpos[3] = Number(targets.rootQuat[0]);
      qpos[4] = Number(targets.rootQuat[1]);
      qpos[5] = Number(targets.rootQuat[2]);
      qpos[6] = Number(targets.rootQuat[3]);
      changed = true;
    }

    if (changed) {
      for (let i = 0; i < Math.min(6, qvel.length); i += 1) {
        qvel[i] = 0.0;
      }
      this.simulation.forward();
    }
  }

  applyPdControl(targets) {
    if (!this.simulation) {
      return;
    }

    const qpos = this.simulation.qpos;
    const qvel = this.simulation.qvel;
    const ctrl = this.simulation.ctrl;
    const ctrlRange = this.model?.actuator_ctrlrange;

    for (let i = 0; i < this.numActions; i += 1) {
      const clipIdx = this.clipJointMap[i];
      const targetJpos = Number(targets?.jointPos?.[clipIdx]);
      const desired = Number.isFinite(targetJpos) ? targetJpos : (this.defaultJointPos[i] ?? 0.0);
      const qposAdr = this.qposAdr[i];
      const qvelAdr = this.qvelAdr[i];
      const ctrlAdr = this.ctrlAdr[i];

      const torque = this.kp[i] * (desired - qpos[qposAdr]) + this.kd[i] * (0 - qvel[qvelAdr]);
      let ctrlValue = torque;

      if (ctrlRange && ctrlRange.length >= (ctrlAdr + 1) * 2) {
        const min = ctrlRange[ctrlAdr * 2];
        const max = ctrlRange[(ctrlAdr * 2) + 1];
        if (Number.isFinite(min) && Number.isFinite(max) && min < max) {
          ctrlValue = clamp(ctrlValue, min, max);
        }
      }

      ctrl[ctrlAdr] = ctrlValue;
    }
  }

  stepClipTime() {
    const controlDt = this.timestep * this.decimation;
    if (!this.playing) {
      return;
    }

    this.clipTime += controlDt;
    const duration = this.clipPlayer.duration;
    if (duration <= 0) {
      this.clipTime = 0;
      return;
    }

    if (this.loop) {
      this.clipTime = this.clipTime % duration;
    } else {
      this.clipTime = Math.min(this.clipTime, duration);
      if (this.clipTime >= duration) {
        this.playing = false;
      }
    }
  }

  async main_loop() {
    while (this.alive) {
      const loopStart = performance.now();

      if (this.model && this.data && this.simulation) {
        this.stepClipTime();
        const targets = this.clipPlayer.getTargets(this.clipTime);
        this.applyRootTarget(targets);

        for (let substep = 0; substep < this.decimation; substep += 1) {
          this.applyPdControl(targets);
          this.simulation.step();
        }

        this._stepFrameCount += 1;
        const now = performance.now();
        const elapsedStep = now - this._stepLastTime;
        if (elapsedStep >= 500) {
          this.simStepHz = (this._stepFrameCount * 1000) / elapsedStep;
          this._stepFrameCount = 0;
          this._stepLastTime = now;
        }
      }

      const loopElapsedSec = (performance.now() - loopStart) / 1000;
      const target = this.timestep * this.decimation;
      const sleepMs = Math.max(0, target - loopElapsedSec) * 1000;
      await new Promise((resolve) => setTimeout(resolve, sleepMs));
    }
  }

  setPlaying(flag) {
    this.playing = Boolean(flag);
  }

  setLoop(flag) {
    this.loop = Boolean(flag);
  }

  setTime(timeSec, applyImmediately = false) {
    const duration = this.clipPlayer.duration;
    const clampedTime = duration > 0 ? clamp(timeSec, 0, duration) : 0;
    this.clipTime = Number.isFinite(clampedTime) ? clampedTime : 0;

    if (applyImmediately) {
      const targets = this.clipPlayer.getTargets(this.clipTime);
      this.applyClipPoseInstant(targets);
    }
  }

  getDuration() {
    return this.clipPlayer.duration;
  }

  getCurrentTime() {
    return this.clipTime;
  }

  getCurrentFrame() {
    return this.clipPlayer.currentFrameIndex;
  }

  getSimStepHz() {
    return this.simStepHz;
  }

  setRenderScale(scale) {
    const clamped = clamp(scale, 0.5, 2.0);
    this.renderScale = clamped;
    this.renderer.setPixelRatio(this.renderScale);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setPixelRatio(this.renderScale);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  render() {
    if (!this.model || !this.simulation) {
      return;
    }

    this.controls.update();

    for (let b = 0; b < this.model.nbody; b += 1) {
      if (!this.bodies[b]) {
        continue;
      }
      getPosition(this.simulation.xpos, b, this.bodies[b].position);
      getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
      this.bodies[b].updateWorldMatrix();
    }

    for (let l = 0; l < this.model.nlight; l += 1) {
      if (!this.lights[l]) {
        continue;
      }
      const position = new THREE.Vector3();
      const direction = new THREE.Vector3();
      getPosition(this.simulation.light_xpos, l, position);
      getPosition(this.simulation.light_xdir, l, direction);
      this.lights[l].position.copy(position);
      this.lights[l].lookAt(direction.clone().add(position));
    }

    this.renderer.render(this.scene, this.camera);
  }

  destroy() {
    this.alive = false;
    window.removeEventListener('resize', this.onWindowResize);
    this.renderer.setAnimationLoop(null);
    this.controls.dispose();
    this.renderer.dispose();

    if (this.simulation) {
      this.simulation.free();
      this.simulation = null;
    }
  }
}
