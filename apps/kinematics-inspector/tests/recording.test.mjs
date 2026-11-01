import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync,existsSync} from 'node:fs';
import {readRecording} from '../../shared/kinematics/recordingTestUtils.mjs';
import {validateRecording,historyFor} from '../../shared/kinematics/recording.js';
const variants=new Map();
for(const sigma of [50,20,5])for(const imu of [false,true])for(const loop of [false,true]){
 const name=`${sigma}${imu?'-imu':''}${loop?'-loop':''}`;
 variants.set(name,readRecording(new URL(`../public/variants/${name}/`,import.meta.url),true));
}
test('Kinematics is last, and Contact vs IK is a separate tab',()=>{
 const apps=JSON.parse(readFileSync(new URL('../../registry.json',import.meta.url)));
 assert.equal(apps.at(-1).id,'kinematics-inspector');assert.equal(apps.at(-1).title,'Kinematics');
 assert.equal(apps.at(-2).title,'Contact vs IK');
});
for(const [name,data]of variants){
 test(`complete contact-only graph, factor budget, and slip data: ${name}`,()=>{
  const {record:r,floats}=data,run=r.runs.contact;
  assert.deepEqual(Object.keys(r.runs),['contact']);assert.deepEqual(r.availableStages,['smoothing']);
  assert.equal(r.samples,1100);assert.equal(r.dt,.2);assert.equal(r.maxIterations,10);
  assert.equal(r.motionSource.sequenceId,'10871_humanoid_20260520_223451');
  assert.equal(r.motionSource.fullTrajectory,true);assert.equal(r.graph.equalities,31900);
  assert.equal(r.graph.equalityDimension,6*r.graph.equalities);
  assert.equal(r.graph.factorGroups.reduce((n,g)=>n+g.count,0),r.graph.costFactors);
  assert.equal(r.graph.factorGroups.length,5+(r.imuAiding.enabled?3:0)+(r.loopClosure.enabled?1:0));
  assert.equal(r.graph.contactSlipSigmaMeters,Number(name.split('-')[0])/1000);
  assert.equal(run.initializer.length,0);assert.equal(run.ikFrames.length,0);
  const noInitializerFields=structuredClone(r);delete noInitializerFields.runs.contact.initializer;delete noInitializerFields.runs.contact.ikFrames;
  validateRecording(noInitializerFields,floats);
  assert(run.solver.iterations>0&&run.solver.iterations<=10);
  assert.equal(run.smoothing.length,run.solver.iterations+2);
  assert(run.smoothing.at(-1).metrics.kinematic_equality_l2<1e-5);
  assert.deepEqual(r.phases.flatMap(p=>Array.from({length:p.end-p.start+1},(_,i)=>p.start+i)),Array.from({length:r.samples},(_,i)=>i));
  for(const s of run.smoothing){
   assert(s.factorErrors&&s.contactSlipOffset!==undefined);
   const sum=Object.values(s.factorErrors).reduce((n,v)=>{assert(Number.isFinite(v)&&v>=0);return n+v;},0);
   assert(Math.abs(sum-s.metrics.cost)<Math.max(1,sum)*1e-8);
   for(let k=0;k<r.samples*2;k++){
    const p=s.contactSlipOffset+k*7,d=floats[p+6];assert(d===-1||d>=0);
    if(d>=0)assert(Math.abs(Math.hypot(floats[p+3]-floats[p],floats[p+4]-floats[p+1],floats[p+5]-floats[p+2])-d)<1e-5);
   }
  }
  const history=historyFor(r,'contact','smoothing',0);
  for(let i=1;i<history.length;i++)assert(history[i].metrics.cost<=history[i-1].metrics.cost+1e-6);
  assert.throws(()=>historyFor(r,'contact','initializer',0),/not been loaded/);
  const invalid=structuredClone(r);invalid.runs.contact.smoothing[0].contactSlipOffset=floats.length;
  assert.throws(()=>validateRecording(invalid,floats),/Invalid contact slip/);
  if(r.imuAiding.enabled){assert.equal(r.imuAiding.intervals,1099);assert.deepEqual(r.imuAiding.biasPriorSigmas.slice(3),[.0001,.0001,.0001]);assert(run.smoothing.at(-1).bias.every(Number.isFinite));}
  assert(existsSync(new URL(`../public/variants/${name}/${r.storage.model}`,import.meta.url)));
 });
}
test('tightening only stationary displacement weights reduces measured slip in all four modes',()=>{
 for(const imu of [false,true])for(const loop of [false,true]){
  const get=sigma=>variants.get(`${sigma}${imu?'-imu':''}${loop?'-loop':''}`).record;
  const records=[50,20,5].map(get),errors=records.map(r=>r.runs.contact.smoothing.at(-1).metrics.stance_step_slip_rms_m);
  assert(errors[2]<errors[1]&&errors[1]<errors[0]);
  for(const r of records){assert.deepEqual(r.phases,records[0].phases);assert.deepEqual(r.graph.factorGroups.map(g=>[g.id,g.count,g.dimension]),records[0].graph.factorGroups.map(g=>[g.id,g.count,g.dimension]));}
 }
});
