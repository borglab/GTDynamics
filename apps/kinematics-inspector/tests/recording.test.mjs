import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync,existsSync} from 'node:fs';
import {readRecording} from '../../shared/kinematics/recordingTestUtils.mjs';
import {validateRecording,historyFor,bodyOffset,sliceTime,frameAtTime} from '../../shared/kinematics/recording.js';
import {Quaternion,Vector3} from 'three';
const variants=new Map();
for(const sigma of [50,20,5])for(const imu of [false,true])for(const loop of [false,true]){
 const name=`${sigma}${imu?'-imu':''}${loop?'-loop':''}`;
 variants.set(name,readRecording(new URL(`../public/variants/${name}/`,import.meta.url),'final'));
}
test('Kinematics is last, and Contact vs IK is a separate tab',()=>{
 const apps=JSON.parse(readFileSync(new URL('../../registry.json',import.meta.url)));
 assert.equal(apps.at(-1).id,'kinematics-inspector');assert.equal(apps.at(-1).title,'Kinematics');
 assert.equal(apps.at(-2).title,'Contact vs IK');
});
for(const [name,data]of variants){
 test(`complete contact-only graph, factor budget, and slip data: ${name}`,()=>{
  const {record:r,floats}=data,run=r.runs.contact;
  assert.deepEqual(Object.keys(r.runs),['contact']);assert.deepEqual(r.availableStages,['final']);
  assert.equal(r.samples,1135);assert.equal(r.dt,.02);assert.equal(r.maxIterations,10);
  assert.equal(r.sampling.mode,'contact-initializer');assert.equal(r.sampling.contactBuffer,1);
  assert.equal(r.timestamps.length,r.samples);assert.equal(r.sampling.sourceIndices.length,r.samples);
  assert.equal(r.timestamps[0],.02);assert(Math.abs(r.timestamps.at(-1)-219.92)<1e-10);
  const intervals=r.timestamps.slice(1).map((time,i)=>time-r.timestamps[i]);
  assert(Math.abs(Math.min(...intervals)-.02)<1e-10);assert.equal(Math.max(...intervals),1);
  for(let k=0;k<r.samples;k++)assert(Math.abs(r.timestamps[k]-r.sampling.sourceIndices[k]*r.dt)<1e-10);
  if(r.imuAiding.enabled)for(let k=0;k<intervals.length;k++)assert(Math.abs(r.imuAiding.intervalSeconds[k]-intervals[k])<1e-10);
  const badTimes=structuredClone(r);badTimes.timestamps[1]=badTimes.timestamps[0];
  assert.throws(()=>validateRecording(badTimes,floats),/Invalid slice timestamps/);
  assert.equal(r.motionSource.sequenceId,'10871_humanoid_20260520_223451');
  assert.equal(r.motionSource.fullTrajectory,true);assert.equal(r.graph.equalities,29*r.samples);
  assert.equal(r.graph.equalityDimension,6*r.graph.equalities);
  assert.equal(r.graph.factorGroups.reduce((n,g)=>n+g.count,0),r.graph.costFactors);
  assert.equal(r.graph.factorGroups.length,5+(r.imuAiding.enabled?3:0)+(r.loopClosure.enabled?1:0));
  assert.equal(r.graph.contactSlipSigmaMeters,Number(name.split('-')[0])/1000);
  assert.equal(run.initializer,undefined);assert.equal(run.ikFrames,undefined);assert.equal(run.smoothing,undefined);
  assert(run.solver.iterations>0&&run.solver.iterations<=10);
  assert(Number.isFinite(run.solver.elapsedSeconds)&&run.solver.elapsedSeconds>0);
  assert.match(run.solver.timingScope,/excludes initialization/);
  assert(run.final.metrics.kinematic_equality_l2<1e-5);
  assert.equal(r.storage.states,'final.bin.gz');assert(floats.byteLength<3*1024*1024);
  assert.deepEqual(r.phases.flatMap(p=>Array.from({length:p.end-p.start+1},(_,i)=>p.start+i)),Array.from({length:r.samples},(_,i)=>i));
  for(const s of [run.final]){
   assert(s.factorErrors&&s.contactSlipOffset!==undefined);
   const sum=Object.values(s.factorErrors).reduce((n,v)=>{assert(Number.isFinite(v)&&v>=0);return n+v;},0);
   assert(Math.abs(sum-s.metrics.cost)<Math.max(1,sum)*1e-8);
   for(let k=0;k<r.samples*2;k++){
    const p=s.contactSlipOffset+k*7,d=floats[p+6];assert(d===-1||d>=0);
    if(d>=0)assert(Math.abs(Math.hypot(floats[p+3]-floats[p],floats[p+4]-floats[p+1],floats[p+5]-floats[p+2])-d)<1e-5);
   }
  }
  assert.throws(()=>historyFor(r,'contact','smoothing',0),/not retained/);
  assert.throws(()=>historyFor(r,'contact','initializer',0),/not been loaded/);
  const invalid=structuredClone(r);invalid.runs.contact.final.contactSlipOffset=floats.length;
  assert.throws(()=>validateRecording(invalid,floats),/Invalid contact slip/);
  if(r.imuAiding.enabled){assert.equal(r.imuAiding.intervals,r.samples-1);assert.deepEqual(r.imuAiding.biasPriorSigmas.slice(3),[.0001,.0001,.0001]);assert(run.final.bias.every(Number.isFinite));}
  assert(existsSync(new URL(`../public/variants/${name}/${r.storage.model}`,import.meta.url)));
  const plots=r.positionPlots;
  assert.equal(r.bodyNames[plots.pelvisBody],'robot/pelvis');
  for(const [label,state]of [['estimate',run.final],['reference',r.reference]]){
   assert.equal(plots.pelvisWorld[label].length,r.samples);
   for(let k=0;k<r.samples;k++){
    const offset=bodyOffset(r,state,k,plots.pelvisBody),position=new Vector3().fromArray(floats,offset);
    const rotation=new Quaternion().fromArray(floats,offset+3).normalize().invert();
    assert(new Vector3().fromArray(plots.pelvisWorld[label][k]).distanceTo(position)<5e-6);
    for(const side of ['left','right']){
     assert.equal(r.bodyNames[plots.footBodies[side]],`robot/${side}_ankle_roll_link`);
     const foot=new Vector3().fromArray(floats,bodyOffset(r,state,k,plots.footBodies[side])).sub(position).applyQuaternion(rotation);
     assert(new Vector3().fromArray(plots.feetBody[side][label][k]).distanceTo(foot)<1.5e-5);
    }
   }
  }

 });
}
test('tightening only stationary displacement weights reduces measured slip in all four modes',()=>{
 for(const imu of [false,true])for(const loop of [false,true]){
  const get=sigma=>variants.get(`${sigma}${imu?'-imu':''}${loop?'-loop':''}`).record;
  const records=[50,20,5].map(get),errors=records.map(r=>r.runs.contact.final.metrics.stance_step_slip_rms_m);
  assert(errors[2]<errors[1]&&errors[1]<errors[0]);
  for(const r of records){assert.deepEqual(r.phases,records[0].phases);assert.deepEqual(r.graph.factorGroups.map(g=>[g.id,g.count,g.dimension]),records[0].graph.factorGroups.map(g=>[g.id,g.count,g.dimension]));}
 }
});

test('playback uses irregular timestamps and retains uniform recording compatibility',()=>{
 const record={samples:4,dt:.02,timestamps:[.02,.04,1,2]};
 assert.equal(sliceTime(record,2),1);
 for(const [time,frame]of [[0,0],[.02,0],[.039,0],[.04,1],[.8,1],[1,2],[2,3],[3,3]])assert.equal(frameAtTime(record,time),frame);
 const uniform={samples:4,dt:.2};assert.equal(sliceTime(uniform,2),.4);assert.equal(frameAtTime(uniform,.45),2);
});

for(const [name,finalData] of variants){
 test(`initial estimate loads independently and matches the final problem: ${name}`,()=>{
  const {record:r,floats}=readRecording(new URL(`../public/variants/${name}/`,import.meta.url),'initial');
  const run=r.runs.contact,initial=run.initial,final=finalData.record.runs.contact.final;
  assert.deepEqual(r.availableStages,['initial']);assert.equal(run.final,undefined);assert.equal(run.smoothing,undefined);
  assert.deepEqual(run.solver,finalData.record.runs.contact.solver);
  assert.deepEqual(r.timestamps,finalData.record.timestamps);assert.deepEqual(r.graph,finalData.record.graph);
  assert.equal(r.storage.states,'initial.bin.gz');assert(floats.byteLength<3*1024*1024);
  assert.equal(initial.iteration,null);assert.equal(initial.frames,r.samples);
  assert(initial.metrics.cost>final.metrics.cost);assert(initial.metrics.kinematic_equality_l2<1e-5);
  assert.notDeepEqual(r.positionPlots.pelvisWorld.estimate,finalData.record.positionPlots.pelvisWorld.estimate);
  assert.deepEqual(r.positionPlots.pelvisWorld.reference,finalData.record.positionPlots.pelvisWorld.reference);
  assert(Math.abs(Object.values(initial.factorErrors).reduce((a,b)=>a+b,0)-initial.metrics.cost)<Math.max(1,initial.metrics.cost)*1e-8);
  if(r.imuAiding.enabled)assert.deepEqual(initial.bias,r.imuAiding.biasPriorMean);
  for(let k=0;k<r.samples;k++){
   const offset=bodyOffset(r,initial,k,r.positionPlots.pelvisBody);
   const position=new Vector3().fromArray(floats,offset),rotation=new Quaternion().fromArray(floats,offset+3).normalize().invert();
   assert(new Vector3().fromArray(r.positionPlots.pelvisWorld.estimate[k]).distanceTo(position)<5e-6);
   for(const side of ['left','right']){
    const foot=new Vector3().fromArray(floats,bodyOffset(r,initial,k,r.positionPlots.footBodies[side])).sub(position).applyQuaternion(rotation);
    assert(new Vector3().fromArray(r.positionPlots.feetBody[side].estimate[k]).distanceTo(foot)<1.5e-5);
   }
  }
  assert(existsSync(new URL(`../public/variants/${name}/${r.storage.model}`,import.meta.url)));
  assert.throws(()=>historyFor(r,'contact','smoothing',0),/not retained/);
 });
}
