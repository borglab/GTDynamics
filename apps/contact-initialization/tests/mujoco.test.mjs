import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { after, before, test } from 'node:test';
import loadMujoco from 'mujoco-js';
import { MujocoG1 } from '../src/mujocoG1.js';

const publicRoot = new URL('../public/', import.meta.url);
let robot, bodyIds;

before(async () => {
  const mujoco = await loadMujoco();
  const manifest = JSON.parse(await readFile(new URL('assets/manifest.json', publicRoot)));
  for (const file of manifest.files) {
    const path = `/working/${file}`;
    mujoco.FS.mkdirTree(path.slice(0, path.lastIndexOf('/')));
    mujoco.FS.writeFile(path, new Uint8Array(await readFile(new URL(`assets/${file}`, publicRoot))));
  }
  robot = new MujocoG1(mujoco, mujoco.MjModel.loadFromXML(`/working/${manifest.scene}`));
  const names = new Uint8Array(robot.model.names), decoder = new TextDecoder();
  bodyIds = new Map(Array.from({ length: robot.model.nbody }, (_, i) => {
    const begin = robot.model.name_bodyadr[i];
    let end = begin;
    while (names[end]) end += 1;
    return [decoder.decode(names.subarray(begin, end)), i];
  }));
});

after(() => robot?.dispose());

function disposeMaterials(root) {
  root.traverse(object => object.material?.dispose());
}

for (const file of ['g1-walk.json', 'g1-steptouch-30s.json']) {
  test(`${file}: MuJoCo visual FK agrees with the recorded body frames`, async () => {
    const dataset = JSON.parse(await readFile(new URL(file, publicRoot)));
    for (const index of [0, 24, Math.floor(dataset.samples.length / 2), dataset.samples.length - 1]) {
      const sample = dataset.samples[index];
      const snapshot = robot.create(sample, dataset.model);
      dataset.model.bodyNames.forEach((name, i) => {
        const actual = snapshot.children[bodyIds.get(name)].position.toArray();
        const distance = Math.hypot(...actual.map((value, axis) => value - sample.bodyPoints[i][axis]));
        assert(distance < 2e-6, `${name} at sample ${index}: ${distance} m`);
      });
      disposeMaterials(snapshot);
    }
  });
}

test('joint names determine the pose even when the recording orders them differently', async () => {
  const { model, samples } = JSON.parse(await readFile(new URL('g1-walk.json', publicRoot)));
  const original = robot.create(samples[24], model);
  const reordered = robot.create(
    { ...samples[24], jointAngles: samples[24].jointAngles.slice().reverse() },
    { ...model, jointNames: model.jointNames.slice().reverse() });
  original.children.forEach((body, i) => {
    assert(body.position.distanceTo(reordered.children[i].position) < 1e-12);
    assert(body.quaternion.angleTo(reordered.children[i].quaternion) < 1e-7);
  });
  disposeMaterials(original);
  disposeMaterials(reordered);
});
