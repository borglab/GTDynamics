import assert from "node:assert/strict";
import test from "node:test";
import * as THREE from "three";
import { ContactScene } from "../src/scene.js";
import { CameraController } from "../../shared/CameraController.js";

// Exercise scene updates without a WebGL renderer; labels only need this canvas API.
test("changing recorded geometry preserves the camera; Fit reframes without rotating", () => {
  const previousDocument = globalThis.document;
  globalThis.document = {
    createElement: () => ({
      getContext: () => ({
        clearRect() {},
        beginPath() {},
        roundRect() {},
        fill() {},
        fillText() {},
        measureText: () => ({ width: 40 }),
      }),
    }),
  };
  try {
    const viewer = Object.create(ContactScene.prototype);
    const camera = new THREE.PerspectiveCamera(35, 2, 0.02, 60);
    camera.position.set(1, -5, 3);
    const controls = {
      target: new THREE.Vector3(),
      maxDistance: 18,
      update() {},
    };
    Object.assign(viewer, {
      camera,
      controls,
      cameraController: new CameraController(camera, controls),
      robot: { create: () => new THREE.Group() },
      state: {},
      hasData: true,
      ground: {},
      hitTargets: [],
      render() {},
      drawSelected() {},
    });
    for (const name of [
      "pathGroup",
      "contactGroup",
      "ghostGroup",
      "labelGroup",
    ])
      viewer[name] = new THREE.Group();
    const originalPosition = camera.position.clone();
    const originalTarget = controls.target.clone();
    const pose = (translation) => ({
      translation,
      rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1],
    });
    const frames = [0, 1].map((sampleIndex) => ({
      sampleIndex,
      observations: [],
    }));
    viewer.update({
      frames,
      model: {},
      samples: [{}, {}],
      contacts: [],
      poses: [pose([0, 0, 0.8]), pose([6, 9, 0.8])],
      showReference: false,
    });
    assert(camera.position.equals(originalPosition));
    assert(controls.target.equals(originalTarget));
    viewer.update({ poses: [pose([4, 1, 0.8]), pose([10, 10, 0.8])] });
    assert(camera.position.equals(originalPosition));
    assert(controls.target.equals(originalTarget));
    const direction = camera.position.clone().sub(controls.target).normalize();
    viewer.fit();
    assert(!camera.position.equals(originalPosition));
    assert(
      controls.target.distanceTo(viewer.bounds.getCenter(new THREE.Vector3())) <
        1e-12,
    );
    assert(
      camera.position
        .clone()
        .sub(controls.target)
        .normalize()
        .distanceTo(direction) < 1e-12,
    );
    assert(viewer.bounds.getSize(new THREE.Vector3()).z < 2);
  } finally {
    globalThis.document = previousDocument;
  }
});
