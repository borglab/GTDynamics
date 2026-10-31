import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { bodyOffset, jointValue, historyFor, validateRecording, decodeStates } from '../src/recording.js';

const record = JSON.parse(readFileSync(new URL('../public/recording.json', import.meta.url)));
const raw = gunzipSync(readFileSync(new URL('../public/states.bin.gz', import.meta.url)));
const floats = decodeStates(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.byteLength), record.storage);

test('complete real recording, feasible final states, and monotone objectives', () => {
  validateRecording(record, floats);
  assert.equal(record.samples, 637);
  assert.equal(record.dt, 1/60);
  assert.equal(record.maxIterations, 10);
  assert.equal(record.phases.length, 21);
  for (const method of ['contact', 'ik']) {
    const run = record.runs[method];
    assert.equal(run.solver.iterations, 10);
    assert.equal(historyFor(record, method, 'smoothing', 0).at(-1).iteration, 10);
    assert.equal(historyFor(record, method, 'smoothing', 0)[0].iteration, 0);
    assert(run.smoothing.at(-1).metrics.kinematic_equality_l2 < 1e-5);
    assert(run.smoothing.at(-1).metrics.cost < run.smoothing[0].metrics.cost);
    const histories = [run.smoothing.slice(1), ...run.ikFrames];
    if (method === 'contact') histories.push(run.initializer);
    for (const history of histories) {
      assert.equal(history[0].iteration, 0);
      for (let i=1;i<history.length;i++) {
        assert(history[i].iteration >= history[i-1].iteration);
        assert(history[i].iteration <= history[i-1].iteration + 1);
        const before=history[i-1].error ?? history[i-1].metrics.cost;
        const after=history[i].error ?? history[i].metrics.cost;
        assert(after <= before + Math.max(1,before)*1e-9);
      }
    }
  }
});

test('pose storage is finite and unit quaternion, and stage/frame selection is exact', () => {
  for (const method of ['contact', 'ik']) {
    const run = record.runs[method];
    for (const state of [...run.smoothing, ...run.initializer, ...run.ikFrames.flat()]) {
      for (let frame=0;frame<state.frames;frame++) {
        for (let body=0;body<record.bodies;body++) {
          const offset=bodyOffset(record,state,frame,body);
          const p=floats.subarray(offset,offset+7);
          assert(p.every(Number.isFinite));
          assert(Math.abs(Math.hypot(...p.subarray(3))-1)<1e-5);
        }
      }
    }
  }
  for (const frame of [0,70,record.samples-1]) {
    assert.strictEqual(historyFor(record,'ik','initializer',frame),record.runs.ik.ikFrames[frame]);
    const ikFinal=record.runs.ik.ikFrames[frame].at(-1), seed=record.runs.ik.smoothing[0];
    for(let j=0;j<record.jointNames.length;j++) {
      assert.equal(jointValue(record,floats,ikFinal,frame,j),jointValue(record,floats,seed,frame,j));
    }
    assert.deepEqual(floats.subarray(bodyOffset(record,ikFinal,frame),bodyOffset(record,ikFinal,frame)+record.bodies*7),
      floats.subarray(bodyOffset(record,seed,frame),bodyOffset(record,seed,frame)+record.bodies*7));
  }
  assert.throws(()=>validateRecording(record,new Float32Array(10)), /Truncated/);
});

test('JazzWalk source, attribution, and original arm motion are retained', () => {
  const source = record.motionSource;
  assert.equal(source.sequenceId, 'g1-jazzwalk');
  assert.equal(source.sourceRevision, '895d064b2385725e6aabf540461c219afc3e0916');
  assert.equal(source.sourceSha256, 'f8f61ea840a38da7a7c9ce468dcdf4643d47ad1e055e7292a278735e60ae1309');
  assert.equal(source.sourceWindow.startSample, 0);
  assert.equal(source.sourceWindow.endExclusiveSample, record.samples);
  assert.equal(source.licenseNotes[0].license, 'CC-BY-4.0');
  assert.equal(source.licenseNotes[0].performer, 'Jasmine Coro');
  for (const name of ['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint']) {
    const joint = record.jointNames.indexOf(name);
    const angles = Array.from({length:record.samples}, (_,k) => jointValue(record,floats,record.reference,k,joint));
    assert(Math.max(...angles)-Math.min(...angles) > 1.8);
  }
});

test('contact history ends at the exact dense initializer, all phases cover the walk', () => {
  const run = record.runs.contact;
  assert.equal(run.initializer.length,run.initialization.iterations+1);
  const last=run.initializer.at(-1), seed=run.smoothing[0];
  const count=record.samples*record.bodies*7;
  for(let i=0;i<count;i++) assert(Math.abs(floats[last.poseOffset+i]-floats[seed.poseOffset+i])<2e-6);
  assert.deepEqual(record.phases.flatMap(p=>Array.from({length:p.end-p.start+1},(_,i)=>p.start+i)),Array.from({length:record.samples},(_,i)=>i));
});
