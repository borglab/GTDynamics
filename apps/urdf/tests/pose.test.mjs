import assert from 'node:assert/strict';
import test from 'node:test';
import { Quaternion, Vector3 } from 'three';
import { api, latestOnly } from '../src/api.js';
import {
  availableSchemes, endEffectorLabel, homeConfiguration, quaternionFromRpyDeg, rpyDegFromQuaternion,
  sliderRange, solveSummary,
} from '../src/pose.js';

const close = (actual, expected, tol = 1e-9) =>
  actual.forEach((v, i) => assert(Math.abs(v - expected[i]) < tol, `${actual} vs ${expected}`));

test('Panda IKFast is offered only for supported frames and pose goals', () => {
  const schemes = [{ id: 'dls' }, { id: 'panda_ikfast', requires_orientation: true }];
  const hand = { schemes: ['dls', 'panda_ikfast'] };
  assert.deepEqual(availableSchemes(schemes, hand, true), schemes);
  for (const frame of [undefined, { schemes: ['dls'] }]) {
    assert.deepEqual(availableSchemes(schemes, frame, true), [schemes[0]]);
  }
  assert.deepEqual(availableSchemes(schemes, hand, false), [schemes[0]]);
});

test('URDF rpy follows R = Rz(yaw) Ry(pitch) Rx(roll) and round-trips', () => {
  const rpy = [10, -20, 30];
  close(rpyDegFromQuaternion(quaternionFromRpyDeg(rpy)), rpy);
  // Yaw alone rotates x toward y about z.
  const x = new Vector3(1, 0, 0).applyQuaternion(quaternionFromRpyDeg([0, 0, 90]));
  close(x.toArray(), [0, 1, 0]);
  const zyx = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 6)
    .multiply(new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), -Math.PI / 9))
    .multiply(new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 18));
  close(quaternionFromRpyDeg(rpy).toArray(), zyx.toArray());
});

test('slider range uses limits, or a default span that includes the value', () => {
  assert.deepEqual(sliderRange({ type: 'revolute', lower: -1, upper: 2 }, 0), [-1, 2]);
  assert.deepEqual(sliderRange({ type: 'continuous', lower: null, upper: null }, 5), [-Math.PI, 5]);
  assert.deepEqual(sliderRange({ type: 'prismatic', lower: null, upper: null }, 0), [-1, 1]);
});

test('home is zero, or mid-range when the limits exclude zero', () => {
  assert.deepEqual(homeConfiguration([
    { name: 'a', lower: -1, upper: 1 },
    { name: 'b', lower: 0.5, upper: 1.5 },
    { name: 'c', lower: null, upper: null },
  ]), { a: 0, b: 1, c: 0 });
});

test('end-effector labels name the carrying link for merged frames', () => {
  assert.equal(endEffectorLabel({ name: 'wrist_3_link', link: 'wrist_3_link', dof: 6 }), 'wrist_3_link · 6 DoF');
  assert.equal(endEffectorLabel({ name: 'tool0', link: 'wrist_3_link', dof: 6 }), 'tool0 (on wrist_3_link) · 6 DoF');
});

test('solve summary reports errors, effort and reasons', () => {
  const res = {
    status: 'failed', pos_err: 0.0123, rot_err_deg: 1.5, iterations: 40, time_ms: 3.21, dof: 6,
    reasons: ['Out of reach'], warnings: ['Near a singularity'],
  };
  assert.deepEqual(solveSummary(res, true), {
    cls: 'failed', label: 'Not solved', meta: '12.30 mm · 1.50° · 40 it · 3.2 ms · 6 DoF',
    items: [{ text: 'Out of reach', kind: 'reason' }, { text: 'Near a singularity', kind: 'warning' }],
  });
  assert.equal(solveSummary({ ...res, status: 'solved', iterations: null }, false).meta, '12.30 mm · 3.2 ms · 6 DoF');
});

test('api reports server errors and non-JSON responses', async () => {
  const reply = (status, body) => async () => ({ status, ok: status < 400, json: async () => JSON.parse(body) });
  assert.deepEqual(await api('api/schemes', undefined, reply(200, '[1]')), [1]);
  await assert.rejects(api('api/load', {}, reply(422, '{"error": "No URDF"}')), /No URDF/);
  // A static host (GitHub Pages) answers with an HTML 404 page.
  await assert.rejects(api('api/schemes', undefined, reply(404, '<html>')), /HTTP 404/);
});

test('latestOnly runs one request at a time and keeps only the newest pending', async () => {
  const seen = [];
  let release;
  const send = latestOnly(async (job) => {
    seen.push(job);
    if (job === 1) await new Promise((resolve) => { release = resolve; });
  });
  send(1); send(2); send(3);
  release();
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.deepEqual(seen, [1, 3]);
});
