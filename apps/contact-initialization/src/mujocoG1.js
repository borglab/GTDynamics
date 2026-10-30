import loadMujoco from 'mujoco-js';
import { createRobotVisual, updateRobotBodies } from '../../shared/mujocoRobot.js';

/** Menagerie G1 meshes, positioned with measured joints by MuJoCo FK. */
export class MujocoG1 {
  static async load(signal) {
    const response = await fetch(`${import.meta.env.BASE_URL}assets/manifest.json`, { signal });
    if (!response.ok || !response.headers.get('content-type')?.includes('json')) {
      throw new Error('G1 model assets are missing. Run npm run models from apps, then reload.');
    }
    const manifest = await response.json();
    const mujoco = await loadMujoco();
    signal.throwIfAborted();
    const directory = '/contact-viewer';
    let next = 0;
    await Promise.all(Array.from({ length: 6 }, async () => {
      while (next < manifest.files.length) {
        const file = manifest.files[next++];
        const asset = await fetch(`${import.meta.env.BASE_URL}assets/${file}`, { signal });
        if (!asset.ok) throw new Error(`Could not load G1 asset ${file} (${asset.status}).`);
        const path = `${directory}/${file}`;
        mujoco.FS.mkdirTree(path.slice(0, path.lastIndexOf('/')));
        mujoco.FS.writeFile(path, new Uint8Array(await asset.arrayBuffer()));
      }
    }));
    signal.throwIfAborted();
    const model = mujoco.MjModel.loadFromXML(`${directory}/${manifest.scene}`);
    try {
      return new MujocoG1(mujoco, model);
    } catch (error) {
      model.delete();
      throw error;
    }
  }

  constructor(mujoco, model) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = new mujoco.MjData(model);
    this.visual = createRobotVisual(model);
    this.visual.root.traverse(object => {
      // Snapshot clones share the CAD buffers; only the owner disposes them.
      if (object.geometry) object.geometry.userData.sharedMujoco = true;
    });
    const names = new Uint8Array(model.names);
    const decoder = new TextDecoder();
    this.joints = new Map();
    for (let i = 0; i < model.njnt; i += 1) {
      if (model.jnt_type[i] !== 3) continue;
      const begin = model.name_jntadr[i];
      let end = begin;
      while (end < names.length && names[end]) end += 1;
      this.joints.set(decoder.decode(names.subarray(begin, end)), model.jnt_qposadr[i]);
    }
    // The event transform is applied by ContactScene. FK stays pelvis-local.
    this.data.qpos.fill(0);
    this.data.qpos[3] = 1;
  }

  create(sample, description, { opacity = 1, color } = {}) {
    if (sample.jointAngles?.length !== description.jointNames.length) {
      throw new Error('The recorded G1 joint angles do not match the model description.');
    }
    description.jointNames.forEach((name, i) => {
      const address = this.joints.get(name);
      if (address === undefined) throw new Error(`The MuJoCo G1 model has no joint named ${name}.`);
      this.data.qpos[address] = sample.jointAngles[i];
    });
    // Replay the measured configuration; the initialization remains a GTSAM solve.
    this.mujoco.mj_kinematics(this.model, this.data);
    updateRobotBodies(this.visual.bodies, this.data);
    const root = this.visual.root.clone(true);
    root.traverse(object => {
      if (!object.isMesh) return;
      object.material = object.material.clone();
      if (color) object.material.color.set(color);
      object.material.opacity *= opacity;
      object.material.transparent = object.material.opacity < 1;
      object.material.depthWrite = opacity > 0.7;
      object.castShadow = opacity > 0.7;
    });
    return root;
  }

  dispose() {
    const geometries = new Set();
    this.visual.root.traverse(object => {
      if (object.geometry) geometries.add(object.geometry);
      object.material?.dispose();
    });
    for (const geometry of geometries) geometry.dispose();
    this.data.delete();
    this.model.delete();
  }
}
