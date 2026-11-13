// Checks the checked-in base-disturbance comparison and meshes that the hosted viewer loads.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const readJson = path => JSON.parse(readFileSync(new URL(path, import.meta.url)));
const replay = readJson('../public/replay.json');
const meshes = readJson('../public/meshes.json');
const N = replay.num_poses - 1;
const torque = frame => Math.hypot(frame.base[0], frame.base[1], frame.base[2]);
const run = name => replay.runs.find(r => r.name === name);

test('replay compares the base cost off with sigma 3, 0.3 and 0.1 on the UR10e', () => {
  assert.equal(replay.format, 'scate-replay');
  assert.equal(replay.q_start.length, 6);
  assert.match(replay.source, /scate_ur10e_base_wrench \(GTDynamics UR10e\)/);
  assert.deepEqual(replay.runs.map(r => r.name), ['online', 'cost_off', 'sigma_0.3', 'sigma_0.1']);
  assert.match(run('online').label, /σ = 3/);
  assert.equal(replay.obstacles.length, 2);
  for (const obstacle of replay.obstacles) assert(Math.hypot(...obstacle.velocity) > 0, 'moving obstacles');
});

test('every run has the full executed motion with a base wrench and obstacle centers per frame', () => {
  for (const r of replay.runs) {
    assert.equal(r.u.length, N);
    assert.equal(r.steps.length, N + 1);
    assert.equal(r.frames.at(-1).t, N * replay.dt);
    for (const frame of r.frames) {
      assert.equal(frame.base.length, 6);
      assert.equal(frame.obstacle_centers.length, 2);
      assert.equal(frame.links.length, replay.links.length);
    }
    assert.equal(r.steps.at(-1).obstacle_estimates.length, 2);
  }
});

test('the base cost lowers the peak base torque, and the default keeps clear', () => {
  const peak = name => Math.max(...run(name).frames.map(torque));
  for (const name of ['online', 'sigma_0.3', 'sigma_0.1']) assert(peak(name) < 0.5 * peak('cost_off'), name);
  assert.equal(run('online').summary.collided, false);
  assert.equal(run('cost_off').summary.collided, false);
});

test('every recorded base wrench is finite, and small once the arm settles at the goal', () => {
  for (const r of replay.runs) for (const frame of r.frames) assert(frame.base.every(Number.isFinite));
  assert(torque(run('online').frames.at(-1)) < 0.5);
});

test('visual meshes cover every link the replay draws, with colored parts', () => {
  const meshForLink = {
    base_link: 'base', shoulder_link: 'shoulder', upper_arm_link: 'upperarm',
    forearm_link: 'forearm', wrist_1_link: 'wrist1', wrist_2_link: 'wrist2',
    wrist_3_link: 'wrist3',
  };
  assert.equal(meshes.kind, 'visual');
  for (const link of replay.links) {
    const mesh = meshes.meshes[meshForLink[link]];
    assert(mesh && mesh.parts.length > 0, link);
    for (const part of mesh.parts) assert.equal(part.color.length, 3);
  }
});
