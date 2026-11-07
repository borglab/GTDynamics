import * as THREE from 'three';
import { RobotViewport } from '../../shared/RobotViewport.js';
import { createRobotVisual } from '../../shared/mujocoRobot.js';
import { segmentAtFrame } from './recording.js';

// Arm 1 carries the gripper and arm 2 the nailer; colors echo the other apps' left/right palette.
const ARM_COLORS = ['#d44747', '#219469'];
const WOOD = '#d8b779';
const WOOD_ACTIVE = '#f0a43a';
const WOOD_FIXED = '#b99556';
const TABLE = '#8f9a96';
const CABLE = '#4c6fb2';
const REPLAN = '#d4883a';
const MOVING = 0.02;
const CABLE_SAMPLES = 57;
// Nail heads are drawn larger than life so they read at the scale of the cell.
const NAIL = '#23272b';
const NAIL_HEAD_RADIUS = .008;
const NAIL_HEAD_HEIGHT = .004;
const NAIL_SHANK_RADIUS = .002;
const NAIL_LENGTH = .083;

function line(color, dashed = false) {
  const material = dashed
    ? new THREE.LineDashedMaterial({ color, dashSize: .12, gapSize: .08, transparent: true, opacity: .8 })
    : new THREE.LineBasicMaterial({ color });
  const object = new THREE.Line(new THREE.BufferGeometry(), material);
  object.visible = false;
  object.frustumCulled = false;
  return object;
}

function setLine(object, points) {
  object.geometry.dispose();
  object.geometry = new THREE.BufferGeometry().setFromPoints(points.map(p => new THREE.Vector3(p[0], p[1], p[2])));
  if (object.material.isLineDashedMaterial) object.computeLineDistances();
  object.visible = points.length > 1;
}

function pathLength(points) {
  let total = 0;
  for (let i = 1; i < points.length; i += 1) total += Math.hypot(...points[i].map((v, k) => v - points[i - 1][k]));
  return total;
}

/** Saved link poses, wood, tabletops, planned/replanned/executed tool paths and dress cables. */
export class CmrgScene {
  constructor(container, model, record) {
    this.container = container;
    this.record = record;
    this.following = false;
    this.viewport = new RobotViewport(container, {
      ariaLabel: 'BAR Lab CMRG building a post-and-lintel frame; drag to orbit, scroll to zoom',
      far: 150, controls: { maxDistance: 60 },
    });
    this.viewport.addStage({ ground: { size: 80, color: '#edf0e9', z: -.01 },
      grid: { size: 40, divisions: 80, colors: ['#bac7bb', '#dce2d9'] } });
    this.robot = createRobotVisual(model);
    record.bodies.static.forEach((pose, i) => {
      this.robot.bodies[i].position.fromArray(pose);
      this.robot.bodies[i].quaternion.fromArray(pose, 3);
    });
    this.viewport.scene.add(this.robot.root);
    this.addTable();
    this.addMembers();
    this.addNails();
    this.addOverlays();
    this.update(0, {});
    this.fit();
    container.dataset.ready = 'true';
  }

  addTable() {
    for (const table of this.record.structure.tables) {
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(...table.half.map(h => 2 * h)),
        new THREE.MeshStandardMaterial({ color: TABLE, roughness: .85 }));
      mesh.position.fromArray(table.center);
      this.viewport.scene.add(mesh);
    }
  }

  addMembers() {
    this.members = this.record.structure.members.map(member => {
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(...member.size),
        new THREE.MeshStandardMaterial({ color: member.fixed ? WOOD_FIXED : WOOD, roughness: .8 }));
      mesh.userData.member = member;
      this.viewport.scene.add(mesh);
      return mesh;
    });
  }

  addNails() {
    const material = new THREE.MeshStandardMaterial({ color: NAIL, roughness: .4, metalness: .6 });
    const head = new THREE.CylinderGeometry(NAIL_HEAD_RADIUS, NAIL_HEAD_RADIUS, NAIL_HEAD_HEIGHT, 16);
    const shank = new THREE.CylinderGeometry(NAIL_SHANK_RADIUS, NAIL_SHANK_RADIUS, NAIL_LENGTH, 8);
    this.nails = this.record.nails.map(nail => {
      const group = new THREE.Group();
      const headMesh = new THREE.Mesh(head, material);
      headMesh.position.y = -NAIL_HEAD_HEIGHT / 2;
      const shankMesh = new THREE.Mesh(shank, material);
      shankMesh.position.y = NAIL_LENGTH / 2;
      group.add(headMesh, shankMesh);
      group.position.fromArray(nail.head);
      group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3().fromArray(nail.direction));
      group.userData.frame = nail.frame;
      this.viewport.scene.add(group);
      return group;
    });
  }

  addOverlays() {
    const scene = this.viewport.scene;
    this.planned = ARM_COLORS.map(color => line(color, true));
    this.replan = ARM_COLORS.map(() => line(REPLAN));
    this.trail = ARM_COLORS.map(color => line(color));
    // Each cable is the chain of equal spheres the planner's constraint checks, drawn as a solid.
    const radius = this.record.settings.cableRadius;
    this.cables = ARM_COLORS.map(() => {
      const chain = new THREE.InstancedMesh(new THREE.SphereGeometry(radius, 14, 10),
        new THREE.MeshStandardMaterial({ color: CABLE, roughness: .55, metalness: .1 }), CABLE_SAMPLES);
      chain.count = 0;
      chain.frustumCulled = false;
      return chain;
    });
    this.tips = ARM_COLORS.map(color => {
      const marker = new THREE.Mesh(new THREE.SphereGeometry(.05, 14, 10),
        new THREE.MeshBasicMaterial({ color, depthTest: false }));
      marker.renderOrder = 10;
      return marker;
    });
    for (const object of [...this.planned, ...this.replan, ...this.trail, ...this.tips]) {
      if (object.material.depthTest !== false && object.isLine) object.material.depthTest = false;
      if (object.isLine) object.renderOrder = 9;
      scene.add(object);
    }
    this.cables.forEach(chain => scene.add(chain));
  }

  /** Apply one recorded frame and the current display options. */
  update(frame, options = {}) {
    const { showPlanned = true, showReplan = true, showCable = true } = options;
    const record = this.record, data = record.frames;
    this.frame = frame;
    const poses = data.bodyPoses[frame];
    record.bodies.movable.forEach((bodyIndex, row) => {
      this.robot.bodies[bodyIndex].position.fromArray(poses[row]);
      this.robot.bodies[bodyIndex].quaternion.fromArray(poses[row], 3);
    });
    const segmentIndex = segmentAtFrame(record.segments, frame), segment = record.segments[segmentIndex];
    data.members[frame].forEach((pose, i) => {
      const mesh = this.members[i];
      mesh.position.fromArray(pose);
      mesh.quaternion.fromArray(pose, 3);
      mesh.material.color.set(mesh.userData.member.fixed ? WOOD_FIXED
        : mesh.userData.member.name === segment.member ? WOOD_ACTIVE : WOOD);
    });
    this.nails.forEach(nail => { nail.visible = frame >= nail.userData.frame; });
    const substeps = segment.subFrames, local = frame - segment.frameStart;
    const step = Math.min(segment.planned.length - 1, Math.floor(local / substeps));
    for (let arm = 0; arm < 2; arm += 1) {
      this.tips[arm].position.fromArray(data.tips[frame][arm]);
      const planned = segment.planned.map(state => state[arm]);
      const moving = pathLength(planned) > MOVING;
      setLine(this.planned[arm], showPlanned && moving ? planned : []);
      setLine(this.replan[arm], showReplan && moving ? segment.replans[step].map(state => state[arm]) : []);
      const trail = [];
      for (let i = segment.frameStart; i <= frame; i += 1) trail.push(data.tips[i][arm]);
      setLine(this.trail[arm], moving ? trail : []);
      this.setCable(this.cables[arm], showCable ? data.cable[frame][arm] : []);
    }
    if (this.following) this.viewport.cameraController.updateTarget(new THREE.Vector3().fromArray(data.tips[frame][0]));
    this.container.dataset.frame = String(frame);
    this.container.dataset.segment = segment.key;
    this.viewport.render();
  }

  setCable(chain, points) {
    const matrix = new THREE.Matrix4();
    chain.count = Math.min(points.length, CABLE_SAMPLES);
    for (let i = 0; i < chain.count; i += 1) {
      matrix.makeTranslation(points[i][0], points[i][1], points[i][2]);
      chain.setMatrixAt(i, matrix);
    }
    chain.instanceMatrix.needsUpdate = true;
    chain.visible = chain.count > 0;
  }

  fit() {
    this.following = false;
    this.viewport.cameraController.fit(new THREE.Box3(new THREE.Vector3(2.5, .2, 0), new THREE.Vector3(12.5, 7.2, 4.2)),
      { view: 'perspective', direction: new THREE.Vector3(.35, -1, .6) });
    this.viewport.render();
  }

  follow() {
    this.following = true;
    this.viewport.cameraController.follow(new THREE.Vector3().fromArray(this.record.frames.tips[this.frame][0]));
    this.viewport.render();
  }

  dispose() { this.viewport.dispose(); }
}
