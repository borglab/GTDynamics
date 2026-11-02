import * as THREE from "three";
import { RobotViewport } from "../../shared/RobotViewport.js";
import { createRobotVisual } from "../../shared/mujocoRobot.js";
import { markerColors, footLabels } from "./recording.js";

/** Recorded real-model geometry and reference-world sole motion. No browser FK. */
export class ContactScene {
  constructor(element, model, record) {
    this.element = element;
    this.record = record;
    this.viewport = new RobotViewport(element, {
      controls: { maxDistance: 80 },
    });
    this.viewport.addStage({
      ground: false,
      grid: {
        size: 60,
        divisions: 120,
        colors: ["#b7c4bc", "#dce4df"],
        z: -0.01,
      },
    });
    this.robot = createRobotVisual(model);
    this.viewport.scene.add(this.robot.root);
    this.markers = {};
    this.axes = {};
    this.trails = {};
    for (const side of ["left", "right"]) {
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.015, 12, 8),
        new THREE.MeshBasicMaterial({ depthTest: false }),
      );
      const outline = new THREE.Mesh(
        new THREE.SphereGeometry(0.017, 12, 8),
        new THREE.MeshBasicMaterial({ color: "#000000", depthTest: false }),
      );
      outline.renderOrder = 3;
      marker.add(outline);
      marker.renderOrder = 4;
      this.viewport.scene.add(marker);
      this.markers[side] = marker;
      this.axes[side] = new THREE.AxesHelper(0.1);
      this.axes[side].material.depthTest = false;
      this.axes[side].renderOrder = 5;
      this.viewport.scene.add(this.axes[side]);
      this.trails[side] = new THREE.Points(
        new THREE.BufferGeometry(),
        new THREE.PointsMaterial({
          size: 2.5,
          sizeAttenuation: false,
          vertexColors: true,
          transparent: false,
          opacity: 1,
        }),
      );
      this.viewport.scene.add(this.trails[side]);
    }
    const path = record.poses.map((p) =>
      new THREE.Vector3().fromArray(p, record.rootBody * 7),
    );
    this.path = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(path),
      new THREE.LineDashedMaterial({
        color: "#6e83a0",
        dashSize: 0.15,
        gapSize: 0.09,
      }),
    );
    this.path.computeLineDistances();
    this.viewport.scene.add(this.path);
    this.preset = null;
    this.following = false;
    this.frame = 0;
    this.fit();
    element.dataset.ready = "true";
  }
  fit() {
    this.following = false;
    const points = this.record.poses.map((p) =>
      new THREE.Vector3().fromArray(p, this.record.rootBody * 7),
    );
    const box = new THREE.Box3().setFromPoints(points);
    box.min.z -= 1;
    box.max.z += 0.5;
    this.viewport.cameraController.fit(box);
    this.viewport.render();
  }
  follow() {
    this.following = true;
    this.viewport.cameraController.follow(
      new THREE.Vector3().fromArray(
        this.record.poses[this.frame],
        this.record.rootBody * 7,
      ),
      new THREE.Vector3(2.2, -3.2, 1.7),
    );
    this.viewport.render();
  }
  update(
    frame, preset, showTrails, walkingPrior, method, velocityAiding,
    enforceWalkingSequence,
  ) {
    this.frame = frame;
    const transforms = this.record.poses[frame];
    this.robot.bodies.forEach((body, i) => {
      body.position.fromArray(transforms, i * 7);
      body.quaternion.fromArray(transforms, i * 7 + 3).normalize();
    });
    if (
      this.preset !== preset ||
      this.walkingPrior !== walkingPrior ||
      this.method !== method ||
      this.velocityAiding !== velocityAiding ||
      this.enforceWalkingSequence !== enforceWalkingSequence
    ) {
      this.preset = preset;
      this.walkingPrior = walkingPrior;
      this.method = method;
      this.velocityAiding = velocityAiding;
      this.enforceWalkingSequence = enforceWalkingSequence;
      this.labels = Object.fromEntries(
        ["left", "right"].map((side) => [
          side,
          footLabels(
            this.record,
            side,
            preset,
            walkingPrior,
            method,
            velocityAiding,
            enforceWalkingSequence,
          ),
        ]),
      );
      for (const side of ["left", "right"]) {
        const foot = this.record.feet[side],
          positions = [],
          colors = [];
        foot.worldContactPoses.forEach((p, k) => {
          positions.push(...p.slice(0, 3));
          const color = new THREE.Color(markerColors[this.labels[side][k]]);
          colors.push(color.r, color.g, color.b);
        });
        this.trails[side].geometry.dispose();
        this.trails[side].geometry = new THREE.BufferGeometry();
        this.trails[side].geometry.setAttribute(
          "position",
          new THREE.Float32BufferAttribute(positions, 3),
        );
        this.trails[side].geometry.setAttribute(
          "color",
          new THREE.Float32BufferAttribute(colors, 3),
        );
      }
    }
    for (const side of ["left", "right"]) {
      const foot = this.record.feet[side],
        pose = foot.worldContactPoses[frame];
      this.markers[side].position.fromArray(pose);
      this.markers[side].material.color.set(
        markerColors[this.labels[side][frame]],
      );
      this.axes[side].position.fromArray(pose);
      this.axes[side].quaternion.fromArray(pose, 3).normalize();
      this.trails[side].visible = showTrails;
    }
    if (this.following)
      this.viewport.cameraController.updateTarget(
        new THREE.Vector3().fromArray(transforms, this.record.rootBody * 7),
      );
    this.element.dataset.method = method;
    this.element.dataset.velocityAiding = velocityAiding;
    this.element.dataset.walkingConstraints = enforceWalkingSequence;
    this.element.dataset.walkingPrior = walkingPrior;
    this.element.dataset.frame = frame;
    this.element.dataset.preset = preset;
    this.viewport.render();
  }
  dispose() {
    this.viewport.dispose();
    delete this.element.dataset.ready;
  }
}
