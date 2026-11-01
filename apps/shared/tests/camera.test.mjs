import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { CameraController } from '../CameraController.js';
import { disposeObject3D } from '../RobotViewport.js';

function fixture(aspect = 1) {
  const camera = new THREE.PerspectiveCamera(35, aspect, .02, 80);
  camera.up.set(0, 0, 1);
  const controls = {
    target: new THREE.Vector3(), maxDistance: 20,
    update() { camera.lookAt(this.target); camera.updateMatrixWorld(true); },
  };
  return { camera, controls, controller: new CameraController(camera, controls) };
}
const close = (actual, expected) => assert(actual.distanceTo(expected) < 1e-9);

test('trajectory fitting keeps all bounds visible in portrait and landscape views', () => {
  for (const aspect of [.3, 3]) {
    const {camera, controller} = fixture(aspect);
    const bounds = new THREE.Box3(new THREE.Vector3(-300, -100, -2), new THREE.Vector3(300, 100, 2));
    controller.fit(bounds);
    for (const x of [-300, 300]) for (const y of [-100, 100]) for (const z of [-2, 2]) {
      const ndc = new THREE.Vector3(x, y, z).project(camera);
      assert(Math.abs(ndc.x) <= 1 && Math.abs(ndc.y) <= 1 && Math.abs(ndc.z) <= 1);
    }
    assert(camera.far > 80, 'long trajectories need an expanded far plane');
  }
});

test('following preserves orbit, pan, and zoom as the robot translates', () => {
  const {camera, controls, controller} = fixture();
  controller.follow(new THREE.Vector3(2, 3, 1));
  // User orbits/zooms and pans while following.
  camera.position.add(new THREE.Vector3(-1, .4, 2));
  controls.target.add(new THREE.Vector3(.3, .2, .1));
  const previousCamera = camera.position.clone(), previousTarget = controls.target.clone();
  const delta = new THREE.Vector3(5, -2, .5);
  controller.updateTarget(new THREE.Vector3(7, 1, 1.5));
  close(camera.position, previousCamera.add(delta));
  close(controls.target, previousTarget.add(delta));
  const stopped = camera.position.clone();
  controller.updateTarget(new THREE.Vector3(7, 1, 1.5));
  close(camera.position, stopped);
});

test('fitting leaves the camera fixed when subsequent replay positions change', () => {
  const {camera, controls, controller} = fixture();
  controller.follow(new THREE.Vector3());
  controller.fit(new THREE.Box3(new THREE.Vector3(-1, -2, 0), new THREE.Vector3(1, 2, 1)));
  const position = camera.position.clone(), target = controls.target.clone();
  controller.updateTarget(new THREE.Vector3(20, 40, 2));
  assert.equal(controller.following, false);
  close(camera.position, position); close(controls.target, target);
});

test('named views preserve Z-up and empty bounds leave the camera unchanged', () => {
  const {camera, controller} = fixture();
  const bounds = new THREE.Box3(new THREE.Vector3(-1, -1, -1), new THREE.Vector3(1, 1, 1));
  for (const [view, axis] of [['top','z'], ['front','x'], ['side','y']]) {
    controller.fit(bounds, {view});
    assert(Math.abs(camera.position[axis]) > camera.position.length() * .99);
    close(camera.up, new THREE.Vector3(0, 0, 1));
  }
  const position = camera.position.clone();
  controller.fit(new THREE.Box3()); close(camera.position, position);
});

test('cleanup frees owned resources once and leaves shared model geometry to its owner', () => {
  const root = new THREE.Group(), owned = new THREE.BoxGeometry(), shared = new THREE.BoxGeometry();
  shared.userData.sharedMujoco = true;
  const material = new THREE.MeshBasicMaterial(), texture = new THREE.Texture();
  material.map = texture;
  let ownedDisposed = 0, sharedDisposed = 0, materialDisposed = 0, textureDisposed = 0;
  owned.addEventListener('dispose', () => ownedDisposed++);
  shared.addEventListener('dispose', () => sharedDisposed++);
  material.addEventListener('dispose', () => materialDisposed++);
  texture.addEventListener('dispose', () => textureDisposed++);
  root.add(new THREE.Mesh(owned, material), new THREE.Mesh(owned, material), new THREE.Mesh(shared, material));
  disposeObject3D(root);
  assert.deepEqual([ownedDisposed, sharedDisposed, materialDisposed, textureDisposed], [1,0,1,1]);
});


test('refitting after a portrait resize updates distance without changing the chosen bounds', () => {
  const {camera, controls, controller} = fixture(3);
  const bounds = new THREE.Box3(new THREE.Vector3(-10, -2, 0), new THREE.Vector3(10, 2, 1));
  controller.fit(bounds);
  const target = controls.target.clone(), distance = camera.position.distanceTo(target);
  camera.aspect = .3; controller.refit();
  assert(camera.position.distanceTo(target) > distance);
  close(controls.target, target);
  for(const x of [-10,10])for(const y of [-2,2])for(const z of [0,1]){
    const ndc = new THREE.Vector3(x,y,z).project(camera);
    assert(Math.abs(ndc.x)<=1&&Math.abs(ndc.y)<=1);
  }
  controller.follow(new THREE.Vector3(1,2,3));
  const following = camera.position.clone();
  camera.aspect = 1; controller.refit();
  close(camera.position, following); assert.equal(controller.following, true);
});
