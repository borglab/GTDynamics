import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';
import {
  buildGraph, contactPoseRank, fitRigidTransform, graphObjective,
  identityPose, inversePose, optimizeGraph, quaternionToRotation, transformPoint,
} from '../src/graph.js';

const distance = (a, b) => Math.hypot(...a.map((v, i) => v - b[i]));
const assertVector = (a, b, epsilon = 1e-9) => assert.ok(distance(a, b) < epsilon, `${a} differs from ${b}`);
const clone = (value) => JSON.parse(JSON.stringify(value));

/** FK-only fixture: the hidden true root never appears in the input data. */
function synthetic() {
  const quaternions = [
    [1, 0, 0, 0], [0.9950041653, 0, 0, 0.0998334166],
    [0.9887710779, 0, 0.1494381325, 0], [0.9800665778, 0.1986693308, 0, 0],
    [0.9689124217, 0, 0, 0.2474039593],
  ];
  const truth = quaternions.map((q, i) => ({ rotation: quaternionToRotation(q), translation: [i * 0.1, i * 0.012, Math.sin(i) * 0.02] }));
  const contactWorld = { L0: [0, -0.12, -0.8], R0: [0.23, 0.12, -0.8], L1: [0.46, -0.12, -0.8] };
  const active = [['L0'], ['L0', 'R0'], ['R0'], ['R0', 'L1'], ['L1']];
  const data = {
    model: { soleCorners: [[-0.06, -0.03, 0], [0.09, -0.03, 0], [0.09, 0.03, 0]], soleCenter: [0, 0, 0] },
    contacts: [{ id: 'L0', foot: 'left' }, { id: 'R0', foot: 'right' }, { id: 'L1', foot: 'left' }],
    frames: active.map((contacts, i) => ({ sampleIndex: i, t: i * 0.1, contacts })),
    samples: truth.map((pose, i) => {
      const inverse = inversePose(pose);
      const foot = (id) => ({ p: transformPoint(inverse, contactWorld[id]), q: [quaternions[i][0], ...quaternions[i].slice(1).map((v) => -v)] });
      return { t: i * 0.1, feet: { left: foot(i < 3 ? 'L0' : 'L1'), right: foot('R0') } };
    }),
  };
  return { data, truth, contactWorld };
}

test('three noncollinear sole points recover full SE(3), including large rotations', () => {
  const local = [[0, 0, 0], [0.1, 0, 0], [0, 0.06, 0], [-0.02, 0.01, 0.03]];
  for (const q of [[0.8, 0.2, 0.3, 0.4], [0, 1, 0, 0], [0, 0, 0.6, 0.8]]) {
    const expected = { rotation: quaternionToRotation(q), translation: [1, -2, 0.7] };
    const fitted = fitRigidTransform(local, local.map((p) => transformPoint(expected, p)));
    assertVector(fitted.translation, expected.translation);
    assertVector(fitted.rotation, expected.rotation);
  }
});

test('FK event chain reconstructs hidden poses and new stance landmarks', () => {
  const { data, truth, contactWorld } = synthetic();
  const graph = buildGraph(data);
  const result = optimizeGraph(graph);
  assert.equal(optimizeGraph(graph, { iterations: 1 }).converged, true, 'Convergence at the iteration cap must be reported.');
  assert.equal(optimizeGraph(graph, { iterations: 0 }).converged, false, 'An unevaluated stopping criterion is not convergence.');
  assert.equal(graph.frames.length, truth.length);
  assert.equal(graph.observability.componentCount, 1);
  assert.equal(graph.observability.fullyConstrained, true);
  assert.equal(graph.dimensions.jointVariables, 0);
  assert.ok(result.residualRms < 1e-10);
  result.poses.forEach((pose, i) => {
    assertVector(pose.translation, truth[i].translation);
    assertVector(pose.rotation, truth[i].rotation);
  });
  for (const contact of graph.contacts) contact.landmarkIndices.forEach((index, j) => assertVector(result.landmarks[index], data.model.soleCorners[j].map((v, k) => v + contactWorld[contact.id][k])));
});

test('graph construction and solve never access the reference trajectory', () => {
  const { data } = synthetic();
  Object.defineProperty(data, 'reference', { get() { throw new Error('Reference leaked into solver'); } });
  for (const groundPlane of [false, true]) {
    const a = buildGraph(data, { groundPlane });
    const b = buildGraph({ ...data, reference: { poses: Array(5).fill({ p: [999, -999, 500], q: [0, 1, 0, 0] }) } }, { groundPlane });
    assert.deepEqual(a.initialPoses, b.initialPoses);
    assert.deepEqual(optimizeGraph(a).poses, optimizeGraph(b).poses);
  }
});

test('explicit level-floor constraints fix every landmark height and preserve objective descent', () => {
  const { data } = synthetic();
  const graph = buildGraph(data, { groundPlane: true, noise: 0.001 });
  assert.equal(graph.groundPlane.enabled, true);
  assert.equal(graph.observability.gaugeDofs, 3);
  assert.equal(graph.dimensions.floorConstraints, graph.landmarks.length);
  const initial = clone(graph.initialPoses);
  initial.forEach((pose, i) => { if (i) pose.translation[2] += 0.03 * Math.sin(i); });
  const result = optimizeGraph(graph, { initialPoses: initial, iterations: 100 });
  assert.ok(result.objectiveFinal < result.objectiveInitial);
  for (const entry of result.history) for (const landmark of entry.landmarks) assert.equal(landmark[2], graph.groundPlane.height);
  for (let i = 1; i < result.history.length; i++) assert.ok(result.history[i].objective <= result.history[i - 1].objective + 1e-12);
  assert.deepEqual(result.poses[0], graph.seedPoses[0]);
});

test('level-floor seed derives the anchor tilt from local sole FK', () => {
  const { data } = synthetic();
  const q = [Math.cos(0.15), Math.sin(0.15), 0, 0];
  const inverseTilt = { rotation: quaternionToRotation([q[0], -q[1], 0, 0]), translation: [0, 0, 0] };
  for (const foot of Object.values(data.samples[0].feet)) {
    foot.p = transformPoint(inverseTilt, foot.p);
    foot.q = [q[0], -q[1], 0, 0];
  }
  const graph = buildGraph(data, { groundPlane: true });
  assertVector(graph.seedPoses[0].rotation, quaternionToRotation(q));
  const heights = graph.frames[0].observations.map((o) => transformPoint(graph.seedPoses[0], o.local)[2]);
  heights.forEach((height) => assert.ok(Math.abs(height - graph.groundPlane.height) < 1e-12));
});

test('contact minimization decreases the true objective and keeps the anchor fixed', () => {
  const graph = buildGraph(synthetic().data);
  const initial = clone(graph.initialPoses);
  initial.forEach((pose, i) => {
    pose.translation[0] += 0.04 * Math.sin(i + 1);
    pose.translation[1] += 0.03 * Math.cos(i + 1);
    pose.translation[2] += 0.02 * Math.sin(i * 2 + 1);
    if (i) pose.rotation = quaternionToRotation([1, 0.03 * Math.sin(i), 0.025 * Math.cos(i), 0.02]);
  });
  const result = optimizeGraph(graph, { initialPoses: initial, iterations: 300, tolerance: 0 });
  assert.deepEqual(result.poses[0], identityPose());
  assert.ok(result.objectiveFinal < result.objectiveInitial * 0.02, `${result.objectiveInitial} -> ${result.objectiveFinal}`);
  for (let i = 1; i < result.history.length; i++) assert.ok(result.history[i].objective <= result.history[i - 1].objective + 1e-12);
  assert.equal(result.history[0].poses.length, graph.frames.length);
  assert.ok(Math.abs(graphObjective(graph, result.poses, result.landmarks) - result.objectiveFinal) < 1e-15);
  assert.deepEqual(graph.initialPoses, buildGraph(synthetic().data).initialPoses, 'Solver must not mutate graph seeds.');
});

test('point mode exposes local rotational ambiguities and an actual alternative solution', () => {
  assert.equal(contactPoseRank([[0, 0, 0]]), 3);
  assert.equal(contactPoseRank([[0, 0, 0], [1, 0, 0]]), 5);
  assert.equal(contactPoseRank([[0, 0, 0], [1, 0, 0], [0, 1, 0]]), 6);
  const graph = buildGraph(synthetic().data, { model: 'point' });
  const result = optimizeGraph(graph);
  assert.equal(graph.observability.fullyConstrained, false);
  assert.ok(graph.observability.freeDofsLowerBound > 0);
  const alternative = clone(result.poses);
  const frame = graph.frames[2]; // An unanchored single-support event.
  assert.equal(frame.observations.length, 1);
  const observation = frame.observations[0];
  alternative[2].rotation = quaternionToRotation([0.8, 0.2, 0.3, 0.4]);
  const rotated = transformPoint({ rotation: alternative[2].rotation, translation: [0, 0, 0] }, observation.local);
  alternative[2].translation = result.landmarks[observation.landmarkIndex].map((v, i) => v - rotated[i]);
  assert.ok(distance(alternative[2].rotation, result.poses[2].rotation) > 0.1);
  assert.ok(Math.abs(graphObjective(graph, alternative, result.landmarks) - result.objectiveFinal) < 1e-12);
});

test('flight/disjoint support components are never joined by an invented motion factor', () => {
  const { data } = synthetic();
  data.contacts.push({ id: 'R1', foot: 'right' });
  data.frames[3].contacts = ['R1', 'L1'];
  const graph = buildGraph(data);
  assert.equal(graph.observability.componentCount, 2);
  assert.equal(graph.observability.gaugeDofs, 12);
  assert.deepEqual(graph.anchorFrameIndices, [0, 3]);
  const result = optimizeGraph(graph);
  assert.ok(Number.isFinite(result.residualRms));
  graph.anchorFrameIndices.forEach((index) => assert.deepEqual(result.poses[index], graph.seedPoses[index]));
});

test('deterministic FK noise creates a reproducible finite measurement problem', () => {
  const { data } = synthetic();
  const graph = buildGraph(data, { noise: 0.002 });
  assert.deepEqual(graph.frames, buildGraph(data, { noise: 0.002 }).frames);
  const result = optimizeGraph(graph);
  assert.ok(result.residualRms > 0);
  assert.ok(result.objectiveFinal < result.objectiveInitial);
});

test('recorded G1 events produce a connected finite contact-only solve', async () => {
  const data = JSON.parse(await readFile(new URL('../public/g1-walk.json', import.meta.url), 'utf8'));
  for (const [model, groundPlane] of [['patch', false], ['patch', true], ['point', false]]) {
    const graph = buildGraph(data, { model, groundPlane });
    const result = optimizeGraph(graph, { iterations: 80, snapshots: false });
    assert.equal(graph.frames.length, 75);
    assert.equal(graph.contacts.length, 38);
    assert.equal(graph.observability.componentCount, 1);
    assert.ok(Number.isFinite(result.residualRms));
    assert.ok(result.objectiveFinal <= result.objectiveInitial + 1e-12);
    for (let i = 1; i < result.history.length; i++) assert.ok(result.history[i].objective <= result.history[i - 1].objective + 1e-12, `${model} objective increased at iteration ${i}`);
    assert.ok(result.poses.every((pose) => [...pose.rotation, ...pose.translation].every(Number.isFinite)));
    assert.deepEqual(result.poses[0], graph.seedPoses[0]);
    assert.equal(graph.dimensions.jointVariables, 0);
    assert.equal(graph.observability.fullyConstrained, model === 'patch');
  }
});
