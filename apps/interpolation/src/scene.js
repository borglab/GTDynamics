import * as THREE from 'three';
import { RobotViewport } from '../../shared/RobotViewport.js';
import { createRobotVisual } from '../../shared/mujocoRobot.js';

/** Saved world body poses, source knots, and world sole-velocity overlays. */
export class InterpolationScene {
  constructor(element, data) {
    this.data = data;
    this.viewport = new RobotViewport(element, {
      far: 1000, controls: { maxDistance: 400 },
    });
    this.stage = this.viewport.addStage();
    this.robot = createRobotVisual(data.model);
    this.viewport.scene.add(this.robot.root);
    const { record, arrays } = data;
    const point = frame => new THREE.Vector3().fromArray(
      arrays.bodyPoses, (frame * record.bodies + record.rootBody) * 7,
    );
    this.path = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(record.times.map((_, i) => point(i))),
      new THREE.LineBasicMaterial({ color: '#087f71' }),
    );
    this.knots = new THREE.Points(
      new THREE.BufferGeometry().setFromPoints(record.knots.frames.map(point)),
      new THREE.PointsMaterial({ color: '#ad5c28', size: .055 }),
    );
    this.feet = [0, 1].map(() => new THREE.Mesh(
      new THREE.SphereGeometry(.027, 12, 10),
      new THREE.MeshStandardMaterial({ color: 'black' }),
    ));
    this.arrows = [0, 1].map(() => new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0), new THREE.Vector3(), .1, '#bd622d',
    ));
    this.viewport.scene.add(this.path, this.knots, ...this.feet, ...this.arrows);
  }

  update(frame, showKnots, showVelocities) {
    const { record, arrays } = this.data;
    this.robot.bodies.forEach((body, i) => {
      const offset = (frame * record.bodies + i) * 7;
      body.position.fromArray(arrays.bodyPoses, offset);
      body.quaternion.fromArray(arrays.bodyPoses, offset + 3);
    });
    const focus = this.robot.bodies[record.rootBody].position;
    this.stage.grid.position.set(Math.round(focus.x / 5) * 5, Math.round(focus.y / 5) * 5, -.007);
    this.stage.ground.position.set(focus.x, focus.y, -.01);
    this.viewport.cameraController.updateTarget(focus);
    this.knots.visible = showKnots;
    this.feet.forEach((foot, i) => {
      foot.position.fromArray(arrays.contactPoses, (frame * 2 + i) * 7);
      foot.material.color.set(record.contacts[frame][i] ? 'black' : 'white');
      const velocity = new THREE.Vector3().fromArray(arrays.footTwists, (frame * 2 + i) * 6 + 3);
      const speed = velocity.length();
      const arrow = this.arrows[i];
      arrow.visible = showVelocities && speed > .001;
      arrow.position.copy(foot.position);
      if (speed > .001) {
        arrow.setDirection(velocity.normalize());
        arrow.setLength(speed * .3, .04, .025);
      }
    });
    this.viewport.render();
  }

  fit(first, last) {
    const { record, arrays } = this.data;
    const bounds = new THREE.Box3();
    for (let frame = first; frame <= last; frame++) {
      bounds.expandByPoint(new THREE.Vector3().fromArray(
        arrays.bodyPoses, (frame * record.bodies + record.rootBody) * 7,
      ));
    }
    bounds.expandByScalar(.7);
    this.viewport.cameraController.fit(bounds, { direction: new THREE.Vector3(.8, -1, .7) });
    this.viewport.render();
  }

  follow(frame) {
    const { record, arrays } = this.data;
    this.viewport.cameraController.follow(new THREE.Vector3().fromArray(
      arrays.bodyPoses, (frame * record.bodies + record.rootBody) * 7,
    ));
    this.viewport.render();
  }

  dispose() {
    this.viewport.dispose();
  }
}
