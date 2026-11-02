import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { createHash } from 'node:crypto';
import { decodeStates } from '../../shared/kinematics/recording.js';
import { validateRecording, frameAtTime } from '../src/recording.js';

const json = path => JSON.parse(readFileSync(new URL(path, import.meta.url)));
const record = JSON.parse(gunzipSync(readFileSync(new URL('../public/recording.json.gz', import.meta.url))));
const bytes = gunzipSync(readFileSync(new URL('../public/states.bin.gz', import.meta.url)));
const floats = decodeStates(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), record.storage);

test('Interpolation follows Kinematics independently of Dynamics', () => {
  const apps = json('../../registry.json');
  assert.equal(apps.at(-1).id, 'interpolation');
  assert.equal(apps.at(-2).id, 'kinematics-inspector');
  assert(!apps.some(app => app.id === 'dynamics'));
});

test('one offline recording uses the actual comparison winner and original knots', () => {
  validateRecording(record, floats);
  const comparison = json('../../kinematics-inspector/public/comparison.json');
  assert.equal(record.source.variant, comparison.bestVariant);
  assert.equal(record.knots.times.length, comparison.variants[0].samples);
  assert.equal(record.jointNames.length, 29);
  assert.equal(new Set(record.jointNames).size, 29);
  assert(record.times.length > 10000);
  assert(record.diagnostics.solveSeconds > 0);
  assert(record.source.checksums.denseJointSource.match(/^[0-9a-f]{64}$/));
  for (const [path, expected] of Object.entries(record.provenance.sources)) {
    const bytes = readFileSync(new URL(`../../../${path}`, import.meta.url));
    assert.equal(createHash('sha256').update(bytes).digest('hex'), expected, path);
  }
  assert(record.diagnostics.mujocoOracle.maxBodyPoseError < 1e-8);
  assert(record.diagnostics.mujocoOracle.maxSoleTwistError < 1e-8);
  const spec = record.storage.arrays.jointAngles;
  for (let k = 0; k < record.knots.times.length; k++) {
    for (let j = 0; j < record.jointNames.length; j++) {
      const value = floats[spec.offset + record.knots.frames[k] * 29 + j];
      assert(Math.abs(value - record.knots.jointAngles[k][j]) < 1e-6);
    }
  }
});

test('encoder residuals and stance diagnostics describe the published arrays', () => {
  const estimated = record.storage.arrays.jointAngles;
  const measured = record.storage.arrays.denseAngles;
  let square = 0;
  for (let i = 0; i < record.times.length * 29; i++) {
    square += (floats[estimated.offset + i] - floats[measured.offset + i]) ** 2;
  }
  const rmsDegrees = Math.sqrt(square / (record.times.length * 29)) * 180 / Math.PI;
  assert(Math.abs(rmsDegrees - record.diagnostics.jointMeasurementRmsDegrees) < 1e-5);
  const feet = record.storage.arrays.footTwists;
  for (const [foot, side] of ['left', 'right'].entries()) {
    let count = 0, sum = 0;
    record.contacts.forEach((contact, frame) => {
      if (!contact[foot]) return;
      const offset = feet.offset + (frame * 2 + foot) * 6 + 3;
      sum += floats[offset] ** 2 + floats[offset + 1] ** 2 + floats[offset + 2] ** 2;
      count++;
    });
    assert.equal(count, record.diagnostics.stance[side].samples);
    assert(Math.abs(Math.sqrt(sum / count) - record.diagnostics.stance[side].linearRmsMetersPerSecond) < 1e-6);
  }
  assert.equal(record.settings.contactConstraintsApplied, false);
});

test('truncated storage and invalid knot mappings are rejected', () => {
  assert.throws(() => validateRecording(record, floats.subarray(1)));
  const invalid = structuredClone(record);
  invalid.knots.frames[0] = -1;
  assert.throws(() => validateRecording(invalid, floats));
  assert.equal(frameAtTime(record.times, -1), 0);
  assert.equal(frameAtTime(record.times, Infinity), record.times.length - 1);
});
