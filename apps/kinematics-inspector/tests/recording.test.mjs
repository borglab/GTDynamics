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
  assert.equal(record.samples, 500);
  assert.equal(record.dt, 1/50);
  assert.equal(record.maxIterations, 10);
  assert.equal(record.graph.contactSlipSigmaMeters, 0.05);
  assert.equal(record.phases.length, 15);
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

test('physical G1 source, LIO reference, mapping, and attribution are retained', () => {
  const source = record.motionSource;
  assert.equal(source.sequenceId, '10871_humanoid_20260520_223451');
  assert.equal(source.sourceRevision, 'db621c57833e08a812b2ec8491aa0ca81f0ecb33');
  assert.equal(source.sourceChecksums['joints.npy'], '2f0784ebea10c8a9cc2b0827e4bd80015bb2b235fc14ec34c289053bd237a154');
  assert.equal(source.sourceWindow.startSeconds, 204);
  assert.equal(source.sourceWindow.endExclusiveSeconds, 214);
  assert.equal(source.sourceSampleCount, 43991);
  assert.equal(source.referenceLabel, 'SuperOdom reference');
  assert(source.mappingValidation.maxFootPositionErrorMeters < 1e-4);
  assert(source.mappingValidation.maxFootRotationErrorRadians < 1e-4);
  assert.equal(source.licenseNotes[0].license, 'CC-BY-4.0');
  const notice = readFileSync(new URL('../public/OPENIO_ATTRIBUTION.txt', import.meta.url), 'utf8');
  assert(notice.includes(source.sequenceId));
  assert.match(notice, /Yuheng Qiu.*Guanya Shi/s);
  const roots = Array.from({length:record.samples}, (_,k) => {
    const offset = bodyOffset(record, record.reference, k, record.rootBody);
    return floats.subarray(offset, offset+7);
  });
  const yaw = p => Math.atan2(2*(p[6]*p[5]+p[3]*p[4]), 1-2*(p[4]*p[4]+p[5]*p[5]));
  const delta = yaw(roots.at(-1))-yaw(roots[0]);
  const turn = Math.atan2(Math.sin(delta),Math.cos(delta))*180/Math.PI;
  const distance = roots.slice(1).reduce((sum,p,i) => sum+Math.hypot(p[0]-roots[i][0],p[1]-roots[i][1]),0);
  assert(turn < -140 && turn > -155);
  assert(distance > 2.5 && distance < 3.5);
});

test('IMU contains actual pelvis measurements and preserves nonuniform timestamps', () => {
  const imu = JSON.parse(readFileSync(new URL('../public/imu.json', import.meta.url)));
  assert.equal(imu.clip, record.motionSource.sequenceId);
  assert.equal(imu.frame, 'pelvis');
  assert.equal(record.motionSource.imu.usedBySmoother, false);
  assert.equal(imu.timeSeconds.length, 2000);
  assert.equal(imu.timeOffsetNs[0], 0);
  assert(BigInt(imu.timeOriginNs) > 0n);
  const deltas = [];
  for (let i=0; i<imu.timeSeconds.length; i++) {
    for (const channel of [imu.accelerometerMetersPerSecondSquared, imu.gyroscopeRadiansPerSecond]) {
      assert.equal(channel[i].length, 3);
      assert(channel[i].every(Number.isFinite));
    }
    assert(Math.abs(imu.timeSeconds[i]-imu.timeSeconds[0]-imu.timeOffsetNs[i]*1e-9) < 1e-12);
    if (i) {
      assert(imu.timeOffsetNs[i] > imu.timeOffsetNs[i-1]);
      deltas.push(imu.timeOffsetNs[i]-imu.timeOffsetNs[i-1]);
    }
  }
  assert(new Set(deltas).size > 1);
});

test('contact history ends at the exact dense initializer, all phases cover the walk', () => {
  const run = record.runs.contact;
  assert.equal(run.initializer.length,run.initialization.iterations+1);
  const last=run.initializer.at(-1), seed=run.smoothing[0];
  const count=record.samples*record.bodies*7;
  for(let i=0;i<count;i++) assert(Math.abs(floats[last.poseOffset+i]-floats[seed.poseOffset+i])<2e-6);
  assert.deepEqual(record.phases.flatMap(p=>Array.from({length:p.end-p.start+1},(_,i)=>p.start+i)),Array.from({length:record.samples},(_,i)=>i));
});

test('entire trajectory at 5 Hz has complete histories and original IMU coverage', () => {
  const full = JSON.parse(readFileSync(new URL('../public/full/recording.json', import.meta.url)));
  const bytes = gunzipSync(readFileSync(new URL('../public/full/states.bin.gz', import.meta.url)));
  const values = decodeStates(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), full.storage);
  validateRecording(full, values);
  assert.equal(full.samples, 1100);
  assert.equal(full.dt, .2);
  assert.equal(full.graph.contactSlipSigmaMeters, 0.05);
  assert.equal(full.motionSource.fullTrajectory, true);
  assert.equal(full.motionSource.sourceWindow.startSample, 0);
  assert.equal(full.motionSource.sourceWindow.endExclusiveSample, 43991);
  assert.equal(full.motionSource.detector.detectionRateHz, 50);
  assert(full.motionSource.detector.contactComponents > 1);
  assert.equal(full.motionSource.contactInitialization.addBetweenFactors, true);
  for (const method of ['contact', 'ik']) {
    const run = full.runs[method];
    assert.equal(run.solver.iterations, 10);
    assert(run.smoothing.at(-1).metrics.kinematic_equality_l2 < 1e-5);
    assert(run.smoothing.at(-1).metrics.cost < run.smoothing[0].metrics.cost);
  }
  assert.deepEqual(full.phases.flatMap(p => Array.from({length:p.end-p.start+1}, (_,i) => p.start+i)), Array.from({length:1100}, (_,i) => i));
  const imu = JSON.parse(readFileSync(new URL('../public/full/imu.json', import.meta.url)));
  assert.equal(imu.timeSeconds.length, 43991);
  assert.equal(imu.timeSeconds[0], 0);
  assert(Math.abs(imu.timeSeconds.at(-1) - full.motionSource.sourceDurationSeconds) < 1e-9);
});

for (const [baselineDirectory, aidedDirectory] of [['', 'imu/'], ['full/', 'full-imu/']]) {
  test(`Gal3 aiding preserves inputs and initializers: ${aidedDirectory}`, () => {
    const load = directory => {
      const metadata = JSON.parse(readFileSync(new URL(`../public/${directory}recording.json`, import.meta.url)));
      const bytes = gunzipSync(readFileSync(new URL(`../public/${directory}states.bin.gz`, import.meta.url)));
      const values = decodeStates(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), metadata.storage);
      validateRecording(metadata, values);
      return {metadata, values};
    };
    const baseline = load(baselineDirectory), aided = load(aidedDirectory);
    const {metadata, values} = aided;
    assert.equal(metadata.imuAiding.enabled, true);
    assert.equal(metadata.graph.contactSlipSigmaMeters, baseline.metadata.graph.contactSlipSigmaMeters);
    assert.equal(metadata.imuAiding.factor, 'GalileanImuFactor');
    assert.equal(metadata.imuAiding.intervals, metadata.samples - 1);
    assert.equal(metadata.imuAiding.biasVariables, 1);
    assert.equal(metadata.imuAiding.poseFrame, 'world_T_linkCOM');
    assert.deepEqual(metadata.imuAiding.stationaryCalibration.sourceWindowSeconds, [0, 25]);
    assert.equal(metadata.imuAiding.stationaryCalibration.sampleCount, 5000);
    assert.deepEqual(metadata.imuAiding.biasPriorSigmas, [.5, .5, .5, .0001, .0001, .0001]);
    const gyroMean = [-.002148401415348053, -.00467177232503891, -.0020597515225410463];
    assert.deepEqual(metadata.imuAiding.biasPriorMean, [0, 0, 0, ...gyroMean]);
    assert.deepEqual(metadata.imuAiding.stationaryCalibration.gyroMeanRadiansPerSecond, gyroMean);
    assert.equal(metadata.motionSource.imu.usedBySmoother, true);
    assert.equal(metadata.graph.costFactors - baseline.metadata.graph.costFactors, metadata.samples + 1);
    assert.deepEqual(metadata.phases, baseline.metadata.phases);
    assert.equal(metadata.provenance.inputs['imu.json'], baseline.metadata.provenance.inputs['imu.json']);
    for (const method of ['contact', 'ik']) {
      const run = metadata.runs[method], original = baseline.metadata.runs[method];
      assert(run.solver.iterations > 0 && run.solver.iterations <= metadata.maxIterations);
      assert(run.smoothing.at(-1).metrics.kinematic_equality_l2 < 1e-5);
      assert(run.smoothing.at(-1).metrics.cost < run.smoothing[0].metrics.cost);
      const history = [...run.initializer, ...run.ikFrames.flat()];
      const originalHistory = [...original.initializer, ...original.ikFrames.flat()];
      assert.equal(history.length, originalHistory.length);
      for (let k=0; k<history.length; k++) {
        const state = history[k], other = originalHistory[k], size = state.frames * metadata.bodies * 7;
        assert.equal(state.iteration, other.iteration);
        assert.equal(state.velocityOffset, undefined);
        assert.deepEqual(values.subarray(state.poseOffset, state.poseOffset+size),
          baseline.values.subarray(other.poseOffset, other.poseOffset+size));
      }
      for (const state of run.smoothing) {
        assert.equal(state.bias.length, 6);
        assert(state.bias.every(Number.isFinite));
        assert(Number.isInteger(state.velocityOffset));
        assert(values.subarray(state.velocityOffset, state.velocityOffset+3*metadata.samples).every(Number.isFinite));
      }
      assert.deepEqual(run.smoothing[0].bias, metadata.imuAiding.biasPriorMean);
      run.smoothing.at(-1).bias.slice(3).forEach((value, axis) => {
        assert(Math.abs(value - gyroMean[axis]) < 3*metadata.imuAiding.biasPriorSigmas[axis+3]);
      });
    }
    const invalid = structuredClone(metadata);
    invalid.runs.contact.smoothing.at(-1).velocityOffset = values.length;
    assert.throws(() => validateRecording(invalid, values), /Invalid IMU/);
  });
}

for (const base of ['full', 'full-imu']) {
  test(`position closure preserves reference and initialization: ${base}`, () => {
    const load = directory => {
      const metadata = JSON.parse(readFileSync(new URL(`../public/${directory}/recording.json`, import.meta.url)));
      const raw = gunzipSync(readFileSync(new URL(`../public/${directory}/states.bin.gz`, import.meta.url)));
      const values = decodeStates(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.byteLength), metadata.storage);
      validateRecording(metadata, values);
      return {metadata, values};
    };
    const baseline = load(base), closed = load(`${base}-loop`);
    const {metadata, values} = closed;
    assert.equal(metadata.loopClosure.enabled, true);
    assert.deepEqual(metadata.loopClosure.positionSigmasMeters, [.30, .30, .05]);
    assert.equal(metadata.loopClosure.factor, 'PoseTranslationPrior3D');
    assert.equal(metadata.loopClosure.sample, metadata.samples-1);
    assert.equal(metadata.loopClosure.orientationConstraint, 'none');
    assert.equal(metadata.graph.costFactors, baseline.metadata.graph.costFactors + 1);
    assert.deepEqual(metadata.provenance.inputs, baseline.metadata.provenance.inputs);
    const poses = (data, state) => data.values.subarray(state.poseOffset, state.poseOffset + state.frames*data.metadata.bodies*7);
    assert.deepEqual(poses(closed, metadata.reference), poses(baseline, baseline.metadata.reference));
    for (const method of ['contact', 'ik']) {
      const run = metadata.runs[method], original = baseline.metadata.runs[method];
      assert.equal(run.solver.iterations, 10);
      assert(run.smoothing.at(-1).metrics.kinematic_equality_l2 < 1e-5);
      assert.deepEqual(poses(closed, run.smoothing[0]), poses(baseline, original.smoothing[0]));
      const histories = [...run.initializer, ...run.ikFrames.flat()];
      const originals = [...original.initializer, ...original.ikFrames.flat()];
      assert.equal(histories.length, originals.length);
      for (let k=0; k<histories.length; k++) assert.deepEqual(poses(closed, histories[k]), poses(baseline, originals[k]));
      // At the identical seed, the only additional objective is the world-position closure.
      const sigma = metadata.loopClosure.positionSigmasMeters;
      const closureCost = run.smoothing[0].closurePositionErrorWorld.reduce((cost, x, i) => cost + .5*(x/sigma[i])**2, 0);
      const difference = run.smoothing[0].metrics.cost - original.smoothing[0].metrics.cost;
      assert(Math.abs(difference - closureCost) < 1e-6*Math.max(1, closureCost));
      for(let i=2; i<run.smoothing.length; i++) assert(run.smoothing[i].metrics.cost <= run.smoothing[i-1].metrics.cost + 1e-6);
      assert(run.smoothing.at(-1).endpointDisplacementWorld.every(Number.isFinite));
      if (metadata.imuAiding.enabled) assert(run.smoothing.at(-1).bias.every(Number.isFinite));
    }
  });
}

for (const directory of ['', 'imu/', 'full/', 'full-imu/', 'full-loop/', 'full-imu-loop/']) {
  test(`smoothing bundle preserves every iterate and defers initializer history: ${directory}`, () => {
    const load = (metadataName, stateName) => {
      const file = readFileSync(new URL(`../public/${directory}${metadataName}`, import.meta.url));
      const metadata = JSON.parse(metadataName.endsWith('.gz') ? gunzipSync(file) : file);
      const bytes = gunzipSync(readFileSync(new URL(`../public/${directory}${stateName}`, import.meta.url)));
      const values = decodeStates(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength), metadata.storage);
      validateRecording(metadata,values);
      return {metadata,values};
    };
    const original = load('recording.json','states.bin.gz');
    const packed = load('smoothing.json.gz','smoothing.bin.gz');
    assert.deepEqual(packed.metadata.availableStages,['smoothing']);
    assert(packed.values.length < original.values.length);
    const sameState = (a,b) => {
      assert.deepEqual(a.metrics,b.metrics);
      assert.deepEqual(a.bias,b.bias);
      for (const [field,size] of [['poseOffset', a.frames*original.metadata.bodies*7],
          ['jointOffset', a.frames*original.metadata.jointNames.length], ['velocityOffset', a.frames*3]]) {
        if(a[field] === undefined) continue;
        assert.deepEqual(packed.values.subarray(a[field],a[field]+size),original.values.subarray(b[field],b[field]+size));
      }
    };
    sameState(packed.metadata.reference, original.metadata.reference);
    for(const method of ['contact','ik']) {
      const a=packed.metadata.runs[method], b=original.metadata.runs[method];
      assert.equal(a.initializer.length,0); assert.equal(a.ikFrames.length,0);
      assert.equal(a.smoothing.length,b.smoothing.length);
      a.smoothing.forEach((state,index)=>sameState(state,b.smoothing[index]));
      assert.throws(()=>historyFor(packed.metadata,method,'initializer',0),/not been loaded/);
    }
    const complete=JSON.parse(gunzipSync(readFileSync(new URL(`../public/${directory}recording.json.gz`,import.meta.url))));
    assert.deepEqual(complete.runs,original.metadata.runs);
    assert.deepEqual(complete.availableStages,['smoothing','initializer']);
  });
}
