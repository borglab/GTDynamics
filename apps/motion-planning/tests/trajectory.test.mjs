import test from 'node:test';
import assert from 'node:assert/strict';
import { sampleTrajectory, sampleTrajectoryState } from '../src/viewer.js';

const position = t => [0.3 * t ** 3 - 0.4 * t ** 2 + 0.1 * t + 0.5,
  -0.2 * t ** 3 + 0.2 * t ** 2 + 0.4 * t - 0.1];
const velocity = t => [0.9 * t ** 2 - 0.8 * t + 0.1,
  -0.6 * t ** 2 + 0.4 * t + 0.4];
const snapshot = { duration: 4.5,
  states: [0, 1.5, 3, 4.5].map(t => ({ q: position(t), v: velocity(t) })) };

function close(actual, expected, tolerance = 1e-11) {
  actual.forEach((value, i) => assert(Math.abs(value - expected[i]) < tolerance,
    `${value} differs from ${expected[i]}`));
}

test('Hermite feedforward recovers a cubic trajectory and its physical time derivative', () => {
  // Different joint polynomials exercise interior samples, both endpoints,
  // and continuity across all collocation boundaries.
  for (const t of [0, 0.17, 0.9, 1.5, 2.21, 2.99, 3, 4.34, 4.5]) {
    const sample = sampleTrajectoryState(snapshot, t);
    close(sample.q, position(t));
    close(sample.v, velocity(t));
    close(sampleTrajectory(snapshot, t), sample.q);
  }
});

test('Hermite desired velocity agrees with playback displacement in both directions and at different speeds', () => {
  for (const direction of [-1, 1]) {
    for (const speed of [0.5, 1, 2, 3]) {
      const t = 1.4, dt = 1e-5;
      const before = sampleTrajectory(snapshot, t - direction * speed * dt);
      const after = sampleTrajectory(snapshot, t + direction * speed * dt);
      const observedVelocity = after.map((q, i) => (q - before[i]) / (2 * dt));
      const feedforward = sampleTrajectoryState(snapshot, t).v.map(v => v * direction * speed);
      close(feedforward, observedVelocity, 1e-8);
    }
  }
});

test('a single-state trajectory is stationary and absent trajectories have no target', () => {
  assert.deepEqual(sampleTrajectoryState({ states: [{ q: [1, 2], v: [3, 4] }], duration: 4 }, 2),
    { q: [1, 2], v: [0, 0] });
  assert.equal(sampleTrajectory(null, 0), null);
});
