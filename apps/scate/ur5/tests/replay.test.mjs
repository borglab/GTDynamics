// Checks the checked-in recording and meshes that the hosted viewer loads.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const readJson = path => JSON.parse(readFileSync(new URL(path, import.meta.url)));
const replay = readJson('../public/replay.json');
const meshes = readJson('../public/meshes.json');
const N = replay.num_poses - 1;

test('replay is a SCATE replay of the UR5 with online and blind runs', () => {
  assert.equal(replay.format, 'scate-replay');
  assert.equal(replay.version, 1);
  assert.equal(replay.q_start.length, 6);
  assert.deepEqual(replay.runs.map(run => run.name), ['online', 'blind']);
});

test('every run has the full executed motion and one step per planner call', () => {
  for (const run of replay.runs) {
    assert.equal(run.u.length, N);
    assert.equal(run.steps.length, N + 1);
    assert.equal(run.frames.length % N, 1, 'N intervals of substeps plus the final state');
    assert.equal(run.frames.at(-1).t, N * replay.dt);
    for (const frame of run.frames) {
      assert.equal(frame.links.length, replay.links.length);
      assert.equal(frame.spheres.length, replay.sphere_radii.length);
    }
  }
});

test('online run avoids the obstacle and the blind run hits it', () => {
  const [online, blind] = replay.runs;
  assert.equal(online.summary.collided, false);
  assert(online.summary.first_detection > 0, 'obstacle discovered online, not at the start');
  assert.equal(blind.summary.collided, true);
  assert.equal(blind.summary.first_detection, -1);
});

test('meshes cover every link the replay draws', () => {
  const meshForLink = {
    base_link: 'base', shoulder_link: 'shoulder', upper_arm_link: 'upperarm',
    forearm_link: 'forearm', wrist_1_link: 'wrist1', wrist_2_link: 'wrist2',
    wrist_3_link: 'wrist3',
  };
  for (const link of replay.links) assert(meshes.meshes[meshForLink[link]], link);
});
