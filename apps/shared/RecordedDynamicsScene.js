import * as THREE from 'three';
import { RobotViewport } from './RobotViewport.js';
import { createRobotVisual } from './mujocoRobot.js';

const colors = ['#d44747', '#219469'];

/** Saved poses and world-frame force arrows. The browser performs no dynamics. */
export class RecordedDynamicsScene {
  constructor(container, model, record) {
    this.container = container;
    this.record = record;
    this.viewport = new RobotViewport(container, {ariaLabel:'G1 ground reaction forces; drag to orbit, scroll to zoom'});
    this.viewport.addStage({ground:false, grid:{size:40, divisions:160, colors:['#bac7bb','#dce2d9']}});
    this.robot = createRobotVisual(model);
    this.viewport.scene.add(this.robot.root);
    const points = record.bodyPoses.map(sample => new THREE.Vector3(...sample[record.rootBody].slice(0,3)));
    this.path = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), new THREE.LineBasicMaterial({color:'#627a83'}));
    this.viewport.scene.add(this.path);
    this.arrows = colors.map(color => {
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(0,0,1), new THREE.Vector3(), 1, color, .12, .065);
      // Near-vertical GRFs run through the leg meshes; keep the diagnostic
      // overlay visible while its base remains the true sole-frame origin.
      for (const object of [arrow.line,arrow.cone]) {
        object.material.depthTest = false; object.material.depthWrite = false;
        object.renderOrder = 10;
      }
      this.viewport.scene.add(arrow); return arrow;
    });
    this.soles = colors.map(color => {
      const sole = new THREE.AxesHelper(.13);
      const marker = new THREE.Mesh(new THREE.SphereGeometry(.025,12,8), new THREE.MeshBasicMaterial({color}));
      sole.add(marker); this.viewport.scene.add(sole); return sole;
    });
    this.update(0, .002, true);
    this.follow();
    container.dataset.ready = 'true';
  }

  /** Swap saved estimates while preserving the current orbit/follow camera. */
  setRecord(record) {
    this.record = record;
    this.path.geometry.dispose();
    this.path.geometry = new THREE.BufferGeometry().setFromPoints(record.bodyPoses.map(sample =>
      new THREE.Vector3(...sample[record.rootBody].slice(0,3))));
  }

  update(frame, scale, showForces) {
    this.frame = frame;
    const sample = this.record.bodyPoses[frame];
    this.robot.bodies.forEach((body, index) => {
      body.position.fromArray(sample[index]); body.quaternion.fromArray(sample[index],3);
    });
    this.viewport.cameraController.updateTarget(new THREE.Vector3().fromArray(sample[this.record.rootBody]));
    this.soles.forEach((sole, foot) => {
      sole.position.fromArray(this.record.contactPoses[frame][foot]);
      sole.quaternion.fromArray(this.record.contactPoses[frame][foot],3);
    });
    this.arrows.forEach((arrow, foot) => {
      const force = new THREE.Vector3().fromArray(this.record.worldForces[frame][foot]);
      const magnitude = force.length(), length = magnitude * scale;
      arrow.visible = showForces && magnitude > 1e-6;
      arrow.position.copy(this.soles[foot].position);
      if (arrow.visible) {
        arrow.setDirection(force.normalize());
        arrow.setLength(length, Math.min(.12,length*.3), Math.min(.065,length*.16));
      }
    });
    this.container.dataset.frame = String(frame);
    this.container.dataset.forceArrows = String(this.arrows.filter(arrow=>arrow.visible).length);
    this.viewport.render();
  }

  follow() {
    this.viewport.cameraController.follow(new THREE.Vector3().fromArray(this.record.bodyPoses[this.frame][this.record.rootBody]));
    this.viewport.render();
  }

  fit() {
    const bounds = new THREE.Box3();
    this.record.bodyPoses.forEach(sample => bounds.expandByPoint(new THREE.Vector3().fromArray(sample[this.record.rootBody])));
    bounds.expandByScalar(.8);
    this.viewport.cameraController.fit(bounds, {direction:new THREE.Vector3(.8,-1,.7)});
    this.viewport.render();
  }

  dispose() { this.viewport.dispose(); }
}
