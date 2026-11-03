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
  assert(d.initialEqualityL2>1);
  assert(d.initializationSeconds>0);
  assert(d.optimizationSeconds>0);
  assert(d.contactInitializationSeconds>0);
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
