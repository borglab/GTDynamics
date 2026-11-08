import { MathUtils, Quaternion, Vector3 } from 'three';
import { homeConfiguration, rpyDegFromQuaternion } from './pose.js';

const defaultTool = 'tool0';
const initialBend = {
  shoulder_lift_joint: -0.65, elbow_joint: 1.2, wrist_1_joint: -0.55, wrist_2_joint: -0.5,
};
const maxBrowserIterations = 200;

/** Shortest rotation from the current orientation to the target, in world axes. */
function rotationError(target, current) {
  const delta = target.clone().multiply(current.clone().invert());
  const vector = new Vector3(delta.x, delta.y, delta.z);
  const length = vector.length();
  if (length === 0) return vector;
  const angle = 2 * Math.atan2(length, Math.abs(delta.w));
  return vector.multiplyScalar((delta.w < 0 ? -angle : angle) / length);
}

/** Kinematics for the two bundled fixed-base, revolute UR arms. */
export class BrowserRobot {
  static load(mujoco, name, urdf) {
    const path = '/browser-ik.urdf';
    mujoco.FS.writeFile(path, urdf);
    try {
      const model = mujoco.MjModel.loadFromXML(path);
      try {
        return new BrowserRobot(mujoco, name, model);
      } catch (error) {
        model.delete();
        throw error;
      }
    } finally {
      mujoco.FS.unlink(path);
    }
  }

  constructor(mujoco, name, model) {
    this.mujoco = mujoco;
    this.name = name;
    this.model = model;
    const { mjOBJ_BODY, mjOBJ_JOINT } = mujoco.mjtObj;
    this.joints = Array.from({ length: model.njnt }, (_, id) => {
      if (model.jnt_type[id] !== mujoco.mjtJoint.mjJNT_HINGE.value || !model.jnt_limited[id]) {
        throw new Error('Browser IK requires bounded revolute joints.');
      }
      return {
        name: mujoco.mj_id2name(model, mjOBJ_JOINT.value, id),
        type: 'revolute',
        lower: model.jnt_range[2 * id],
        upper: model.jnt_range[2 * id + 1],
        positionAddress: model.jnt_qposadr[id],
        velocityAddress: model.jnt_dofadr[id],
      };
    });
    this.links = Array.from({ length: model.nbody }, (_, id) => {
      const name = mujoco.mj_id2name(model, mjOBJ_BODY.value, id);
      let dof = 0;
      let reach = 0;
      for (let body = id; body > 0; body = model.body_parentid[body]) {
        dof += model.body_jntnum[body];
        reach += Math.hypot(...model.body_pos.subarray(3 * body, 3 * body + 3));
      }
      return { id, name, dof, reach };
    });
    this.data = new mujoco.MjData(model);
    this.jacPosition = new mujoco.DoubleBuffer(3 * model.nv);
    this.jacRotation = new mujoco.DoubleBuffer(3 * model.nv);
    this.normal = new mujoco.DoubleBuffer(model.nv ** 2);
    this.step = new mujoco.DoubleBuffer(model.nv);
  }

  describe() {
    // Both arms are singular at the zero pose. Start bent for interactive dragging.
    const initial = { ...homeConfiguration(this.joints), ...initialBend };
    return {
      base: 'base_link',
      joints: this.joints.map(({ name, type, lower, upper }) => ({ name, type, lower, upper })),
      end_effectors: this.links.filter(link => link.dof).map(({ name, dof }) => ({ name, link: name, dof })),
      default_ee: defaultTool,
      ...this.forward(defaultTool, initial),
    };
  }

  link(name) {
    const link = this.links.find(link => link.name === name);
    if (!link) throw new Error(`Unknown end-effector frame '${name}'.`);
    return link;
  }

  evaluate(link, angles) {
    const qpos = this.data.qpos;
    this.joints.forEach(({ positionAddress }, index) => { qpos[positionAddress] = angles[index]; });
    this.mujoco.mj_kinematics(this.model, this.data);
    const { id } = link;
    const [w, x, y, z] = this.data.xquat.subarray(4 * id, 4 * id + 4);
    return {
      position: new Vector3().fromArray(this.data.xpos, 3 * id),
      rotation: new Quaternion(x, y, z, w),
    };
  }

  angles(q) {
    return this.joints.map(({ name, lower, upper }) => MathUtils.clamp(q[name] ?? 0, lower, upper));
  }

  configuration(angles, pose) {
    const positions = this.data.xpos;
    return {
      q: Object.fromEntries(this.joints.map(({ name }, index) => [name, angles[index]])),
      ee_pose: {
        position: pose.position.toArray(),
        quat_xyzw: pose.rotation.toArray(),
        rpy_deg: rpyDegFromQuaternion(pose.rotation),
      },
      skeleton: this.links.map(({ id, name }) => ({
        link: name,
        parent: id ? this.links[this.model.body_parentid[id]].name : null,
        position: Array.from(positions.subarray(3 * id, 3 * id + 3)),
      })),
      at_limits: this.joints.filter(({ lower, upper }, i) =>
        Math.min(angles[i] - lower, upper - angles[i]) < 1e-6).map(({ name }) => name),
    };
  }

  forward(name, q) {
    const angles = this.angles(q);
    return this.configuration(angles, this.evaluate(this.link(name), angles));
  }

  dampedStep(body, damping, error, rotationWeight) {
    const { mujoco, model, data } = this;
    const { nv } = model;
    mujoco.mj_comPos(model, data);
    mujoco.mj_jacBody(model, data, this.jacPosition, this.jacRotation, body);
    const position = this.jacPosition.GetView();
    const rotation = this.jacRotation.GetView();
    const jacobian = error.length === 3 ? position : [...position, ...rotation.map(value => value * rotationWeight)];
    const normal = this.normal.GetView();
    const rhs = new Float64Array(nv);
    normal.fill(0);
    for (let col = 0; col < nv; col++) {
      normal[col * nv + col] = damping ** 2;
      for (let row = 0; row < error.length; row++) {
        const offset = row * nv;
        const entry = jacobian[offset + col];
        rhs[col] += entry * error[row];
        for (let j = 0; j < nv; j++) normal[col * nv + j] += entry * jacobian[offset + j];
      }
    }
    mujoco.mju_cholFactor(this.normal, 1e-12);
    mujoco.mju_cholSolve(this.step, normal, rhs);
    const step = this.step.GetView();
    const scale = 0.5 / Math.max(0.5, ...step.map(Math.abs));
    return this.joints.map(({ velocityAddress }) => scale * step[velocityAddress]);
  }

  solve({ ee, q, target, orientation, pos_tol, rot_tol_deg, max_iters }) {
    const start = performance.now();
    const link = this.link(ee);
    const goal = new Vector3().fromArray(target.position);
    const goalRotation = new Quaternion().fromArray(target.quat_xyzw).normalize();
    const rotationWeight = MathUtils.clamp(link.reach / 2, 0.05, 1);
    const positionTolerance = pos_tol > 0 ? pos_tol : 0.001;
    const rotationTolerance = MathUtils.degToRad(rot_tol_deg > 0 ? rot_tol_deg : 0.5);
    const limit = MathUtils.clamp(Math.floor(max_iters ?? 60), 1, maxBrowserIterations);
    const measure = pose => {
      const translation = goal.clone().sub(pose.position);
      const rotation = rotationError(goalRotation, pose.rotation);
      const error = translation.toArray();
      if (orientation) error.push(...rotation.clone().multiplyScalar(rotationWeight).toArray());
      return { error, position: translation.length(), rotation: rotation.length(), norm: Math.hypot(...error) };
    };
    const converged = error => error.position <= positionTolerance &&
      (!orientation || error.rotation <= rotationTolerance);

    let angles = this.angles(q);
    let pose = this.evaluate(link, angles);
    let error = measure(pose);
    let damping = 0.05;
    let iterations = 0;
    while (iterations < limit && damping <= 1e4 && !converged(error)) {
      iterations++;
      const step = this.dampedStep(link.id, damping, error.error, rotationWeight);
      const candidate = this.joints.map(({ lower, upper }, i) =>
        MathUtils.clamp(angles[i] + step[i], lower, upper));
      const candidatePose = this.evaluate(link, candidate);
      const candidateError = measure(candidatePose);
      if (candidateError.norm < error.norm) {
        angles = candidate;
        pose = candidatePose;
        error = candidateError;
        damping = Math.max(0.5 * damping, 1e-4);
      } else {
        this.evaluate(link, angles);
        damping *= 3;
      }
    }

    const result = this.configuration(angles, pose);
    const solved = converged(error);
    const reasons = [];
    if (!solved) {
      if (goal.length() > link.reach + positionTolerance) {
        reasons.push('The target is beyond the maximum reach of this tool frame.');
      } else if (link.dof < (orientation ? 6 : 3)) {
        reasons.push('This frame has too few movable joints for the requested goal.');
      } else if (iterations === limit) {
        reasons.push('The browser solver reached its iteration limit.');
      } else {
        reasons.push('The solve stalled before reaching the target. Try smaller moves or bend the arm with the joint sliders.');
      }
    }
    return {
      ...result, status: solved ? 'solved' : 'failed', reasons, iterations,
      warnings: result.at_limits.length ? [`At joint limits: ${result.at_limits.join(', ')}.`] : [],
      pos_err: error.position, rot_err_deg: MathUtils.radToDeg(error.rotation),
      time_ms: performance.now() - start, dof: link.dof,
    };
  }

  dispose() {
    for (const buffer of [this.jacPosition, this.jacRotation, this.normal, this.step]) buffer.delete();
    this.data.delete();
    this.model.delete();
  }
}
