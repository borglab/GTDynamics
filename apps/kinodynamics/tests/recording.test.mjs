import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {validateRecording,jointGroups} from '../src/recording.js';
const read=name=>JSON.parse(gunzipSync(readFileSync(new URL(`../public/${name}.json.gz`,import.meta.url))));
const record=read('recording'),model=read('model');

test('the full mid-clip solve retains every original 50 Hz slice',()=>{
  validateRecording(record,model);
  assert.equal(record.times.length,1000);
  assert.equal(record.times[0],78);
  assert.equal(record.times.at(-1),97.98);
  record.times.slice(1).forEach((time,index)=>assert(Math.abs(time-record.times[index]-.02)<1e-10));
  assert.equal(record.initial.bodyPoses.length,1000);
  assert(record.settings.kinematicsEstimated);
  assert(record.settings.accelerationsEstimated);
});

test('the measured run reduces cost and satisfies the exact mechanics and sole cones',()=>{
  const d=record.diagnostics;
  assert(d.feasible);
  assert(d.objectiveFinal<d.objectiveInitial);
  assert(d.equalityL2<1e-5);
  assert(d.inequalityL2<1e-5);
  assert(d.initialEqualityL2<1e-5);
  assert.equal(d.initializationStrategy,'contact-gp-dynamics');
  assert(d.dynamicsInitializationSeconds>0);
  assert(d.dynamicsInitializationAssemblySeconds>0);
  assert(d.dynamicsInitializationAnchorSeconds>0);
  assert(d.dynamicsInitializationLinearizationSeconds>0);
  assert(d.dynamicsInitializationLinearSolveSeconds>0);
  assert(d.dynamicsInitializationValueAssemblySeconds>0);
  assert(d.dynamicsInitializationValidationSeconds>0);
  assert(d.dynamicsInitializationLinearizationSeconds+d.dynamicsInitializationLinearSolveSeconds+d.dynamicsInitializationValueAssemblySeconds<=d.dynamicsInitializationAnchorSeconds);
  assert.equal(d.dynamicsInitializationObjectiveInitial,d.dynamicsInitializationObjectiveFinal);
  assert.equal(d.dynamicsInitializationIterations,0);
  assert.equal(d.dynamicsInitializationManifoldSeconds,0);
  assert.equal(d.dynamicsInitializationOptimizationSeconds,0);
  assert(d.dynamicsInitializationFeasible);
  assert(d.initializationSeconds>0);
  assert(d.optimizationSeconds>0);
  assert(d.configurationInitializationSeconds>0);
  assert(Math.abs(d.errorHistory.at(-1).reduce((a,b)=>a+b,0)-d.objectiveFinal)<1e-5);
  assert.equal(d.errorHistory.length,d.wallSeconds.length);
  assert(d.factors.some(factor=>factor.type==='Configuration collocation'&&factor.count===29970));
  assert(d.factors.some(factor=>factor.type==='Velocity collocation'&&factor.count===29970));
  assert(record.worldVelocities.some(v=>Math.hypot(...v)>.1));
  assert(record.worldForces.some(sample=>sample.some(v=>Math.hypot(...v)>100)));
  record.contact.forEach((sample,k)=>sample.forEach((stance,foot)=>{
    const wrench=record.wrenches[k][foot];
    if(!stance)assert(Math.hypot(...wrench)<1e-7);
    else assert(wrench[5]>-1e-7);
  }));
  assert.equal(new Set(jointGroups(record.jointNames).flatMap(g=>g.indices)).size,29);
});


test('the recorded gauge is a translation/yaw cost and leaves tilt free',()=>{
  const d=record.diagnostics;
  assert.deepEqual(d.gauge.coordinates,['world_x','world_y','world_z','yaw']);
  assert.deepEqual(d.gauge.rotationFree,['roll','pitch']);
  assert.equal(d.gauge.translationSigmaMeters,1);
  assert.equal(d.gauge.yawSigmaRadians,1);
  assert(d.errorNames.includes('Translation/yaw gauge'));
  assert(d.factors.some(f=>f.type==='Translation/yaw gauge'&&f.count===2));
  assert.equal(d.solverSettings.historyWindow,1);
  assert.equal(d.iterations,d.maxIterations);
});


test('initialization sampling and timing comparisons match the replay',()=>{
  const sampling=record.source.initializationSampling;
  const selected=record.diagnostics.initializationSlices;
  assert.equal(sampling.selectedSlices,selected.length);
  assert.equal(sampling.singleSupportSlices,selected.filter(k=>record.contact[k].filter(Boolean).length===1).length);
  assert.equal(sampling.doubleSupportSlices,selected.filter(k=>record.contact[k].filter(Boolean).length===2).length);
  assert(sampling.singleSupportSlices>0);
  const comparison=record.initializationComparison;
  assert.equal(comparison.runsPerStrategy,1);
  assert.deepEqual(comparison.runs.map(run=>run.initializationStrategy),['contact-gp-dynamics','contact','fk']);
  const contact=comparison.runs[0];
  for(const key of ['objectiveFinal','iterations','optimizationSeconds','totalInitializationAndSolveSeconds'])assert.equal(contact[key],record.diagnostics[key]);
  assert.equal(comparison.samples,record.times.length);
});


test('the default pipeline replays GP motion and the affine dynamics seed',()=>{
  assert.equal(record.initial.jointVelocities.length,record.times.length);
  assert.equal(record.initial.jointTorques.length,record.times.length);
  assert(record.initial.jointVelocities.some(sample=>sample.some(value=>Math.abs(value)>.01)));
  assert(record.initial.jointTorques.some(sample=>sample.some(value=>Math.abs(value)>1)));
  assert(record.initial.worldVelocities.some(sample=>Math.hypot(...sample)>.1));
  assert(record.initial.worldForces.some(sample=>sample.some(force=>Math.hypot(...force)>100)));
});
