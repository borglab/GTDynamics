import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync,existsSync} from 'node:fs';
import {readRecording} from '../../shared/kinematics/recordingTestUtils.mjs';
import {validateRecording,historyFor,bodyOffset,sliceTime,frameAtTime} from '../../shared/kinematics/recording.js';
import {Quaternion,Vector3} from 'three';
const variants=new Map();
for(const prior of ['none','upright','height','upright-height'])for(const imu of [false,true])for(const loop of [false,true]){
 const name=`${prior}${imu?'-imu':''}${loop?'-loop':''}`;
 variants.set(name,readRecording(new URL(`../public/variants/${name}/`,import.meta.url),'final'));
}
test('estimation tabs precede Motion Planning and the SCATE apps',()=>{
 const apps=JSON.parse(readFileSync(new URL('../../registry.json',import.meta.url)));
 const k=apps.findIndex(app=>app.id==='kinematics-inspector');
 assert.deepEqual(apps.slice(k-1).map(app=>app.id),['contact-vs-ik','kinematics-inspector','interpolation','dynamics','kinodynamics','motion-planning','scate/ur5','scate/ur10e']);
});
test('comparison includes every configuration and ranks actual saved errors',()=>{
 const summary=JSON.parse(readFileSync(new URL('../public/comparison.json',import.meta.url)));
 assert.equal(summary.variants.length,16);
 assert.deepEqual(new Set(summary.variants.map(row=>row.id)),new Set(variants.keys()));
 assert.equal(summary.bestVariant,summary.variants[0].id);
 for(const [index,row]of summary.variants.entries()){
  const r=variants.get(row.id).record,final=r.runs.contact.final;
  if(index)assert(row.alignedRmsMeters>=summary.variants[index-1].alignedRmsMeters);
  assert.equal(row.alignedRmsMeters,r.referenceAlignment.rmsAfterMeters);
  assert.equal(row.referenceRmsMeters,r.referenceAlignment.rmsBeforeMeters);
  assert.deepEqual(row.metrics,final.metrics);assert.deepEqual(row.factorErrors,final.factorErrors);
  assert.deepEqual(row.factorCounts,Object.fromEntries(r.graph.factorGroups.map(g=>[g.id,g.count])));
  assert.deepEqual(row.closurePositionErrorWorld,final.closurePositionErrorWorld);
  assert.equal(row.upright,r.graph.uprightPrior);assert.equal(row.height,r.graph.heightPrior);
  assert.equal(row.imu,r.imuAiding.enabled);assert.equal(row.loop,r.loopClosure.enabled);
  assert.equal(row.samples,r.samples);
  assert.equal(row.iterations,r.runs.contact.solver.iterations);
  assert.equal(row.elapsedSeconds,r.runs.contact.solver.elapsedSeconds);
 }
 assert.equal(summary.variants[0].alignedRmsMeters,Math.min(...[...variants.values()].map(({record:r})=>r.referenceAlignment.rmsAfterMeters)));
});
for(const [name,data]of variants){
 test(`complete contact-only graph, factor budget, and slip data: ${name}`,()=>{
  const {record:r,floats}=data,run=r.runs.contact;
  assert.deepEqual(Object.keys(r.runs),['contact']);assert.deepEqual(r.availableStages,['final']);
  assert(r.samples>2&&r.samples<r.contactPhaseInference.sourceSlices);assert.equal(r.dt,.02);assert.equal(r.maxIterations,50);
  assert.equal(r.sampling.mode,'double-support');assert.equal(r.sampling.interpolatedSolveStates,false);assert.equal(r.sampling.contactBuffer,1);
  assert.equal(r.contactInitialization.rotation_sigma_degrees,90);
  assert.equal(r.contactInitialization.translation_sigma_m,.01);
  assert(r.contactInitialization.initialize_upright);
  assert(r.contactInitialization.upright_prior&&r.contactInitialization.height_prior&&r.contactInitialization.motion_prior);
  assert.equal(r.contactInitialization.normal_prior,false);
  assert.equal(r.contactInitialization.height_sigma_m,.1);
  assert.equal(r.contactInitialization.ground_height_m,0);
  assert(Math.abs(r.contactInitialization.upright_sigma_degrees-15)<1e-12);
  assert.equal(r.contactInitialization.motion_rotation_sigma_degrees,180);
  assert.equal(r.contactInitialization.motion_translation_sigma_m,10);
  assert.equal(typeof r.contactInitialization.stalled,'boolean');
  if(r.contactInitialization.stalled)assert.equal(r.contactInitialization.converged,false);
  assert.equal(r.initializationFrame.anchorSourceIndex,r.sampling.sourceIndices[0]);
  assert(r.initializationFrame.worldTranslationMeters.every(Number.isFinite));
  for(let axis=0;axis<3;axis++)assert(Math.abs(r.motionSource.worldTranslationMeters[axis]-(r.initializationFrame.inputWorldTranslationMeters[axis]+r.initializationFrame.worldTranslationMeters[axis]))<1e-12);
  assert(Number.isFinite(r.contactInitialization.rotation_rms_degrees));
  assert.equal(r.timestamps.length,r.samples);assert.equal(r.sampling.sourceIndices.length,r.samples);
  assert.equal(r.timestamps[0],.02);assert(Math.abs(r.timestamps.at(-1)-219.92)<1e-10);
  const intervals=r.timestamps.slice(1).map((time,i)=>time-r.timestamps[i]);
  assert(Math.min(...intervals)>0);assert(new Set(intervals.map(t=>Math.round(t/r.dt))).size>1);
  assert.equal(r.sampling.maxSliceIntervalMs,200);
  const inferred=r.contactPhaseInference,phases=inferred.phases;
  assert.equal(inferred.method,'binary-foot max-product');assert(inferred.velocityAiding&&inferred.enforceWalkingSequence);
  assert.equal(phases[0].start,0);assert.equal(phases.at(-1).end,inferred.sourceSlices-1);
  const selected=[];
  for(let i=0;i<phases.length;i++){
   const p=phases[i];assert(p.contacts.length===1||p.contacts.length===2);
   if(i){assert.equal(p.start,phases[i-1].end+1);assert(p.contacts.length===2||phases[i-1].contacts.length===2);}
   const retained=r.sampling.sourceIndices.filter(k=>k>=p.start&&k<=p.end);
   if(p.contacts.length!==2){assert.equal(retained.length,0);continue;}
   const count=p.end-p.start+1;
   if(count*r.dt<=r.sampling.longPhaseSeconds+1e-12)assert.deepEqual(retained,[p.start+Math.floor((count-1)/2)]);
   else{
    assert.equal(retained[0],p.start+1);assert.equal(retained.at(-1),p.end-1);
    const gaps=retained.slice(1).map((k,j)=>k-retained[j]);
    assert(Math.max(...gaps)*r.dt<=r.sampling.maxSliceIntervalMs/1000+1e-12);
    assert(Math.max(...gaps)-Math.min(...gaps)<=1);
    assert.equal(gaps.length,Math.ceil((p.end-p.start-2)/Math.floor(r.sampling.maxSliceIntervalMs/1000/r.dt+1e-12)));
   }
   selected.push(...retained);
  }
  assert.deepEqual(r.sampling.sourceIndices,[...new Set(selected)]);
  assert.equal(r.graph.factorGroups.find(g=>g.id==='contact').count,2*r.samples);
  assert.equal(r.graph.stanceLandmarks,2+r.sampling.contactBreaks.length);
  for(const p of r.phases)assert.equal(p.contacts.length,2);
  for(let k=0;k<r.samples;k++)assert(Math.abs(r.timestamps[k]-r.sampling.sourceIndices[k]*r.dt)<1e-10);
  if(r.imuAiding.enabled)for(let k=0;k<intervals.length;k++)assert(Math.abs(r.imuAiding.intervalSeconds[k]-intervals[k])<1e-10);
  const badTimes=structuredClone(r);badTimes.timestamps[1]=badTimes.timestamps[0];
  assert.throws(()=>validateRecording(badTimes,floats),/Invalid slice timestamps/);
  assert.equal(r.motionSource.sequenceId,'10871_humanoid_20260520_223451');
  assert.equal(r.motionSource.fullTrajectory,true);assert.equal(r.graph.equalities,29*r.samples);
  assert.equal(r.graph.equalityDimension,6*r.graph.equalities);
  assert.equal(r.graph.factorGroups.reduce((n,g)=>n+g.count,0),r.graph.costFactors);
  assert.equal(r.graph.factorGroups.length,3+(r.graph.heightPrior?1:0)+(r.graph.uprightPrior?1:0)+(r.imuAiding.enabled?3:0)+(r.loopClosure.enabled?1:0));
  assert.equal(r.graph.contactTranslationSigmaMeters,.01);
  assert.equal(r.graph.contactRotationSigmaDegrees,90);
  assert.equal(r.graph.motionPrior,false);
  assert.equal(r.graph.heightPrior,name.includes('height'));
  assert.equal(r.graph.uprightPrior,name.includes('upright'));
  const ground=r.graph.factorGroups.find(group=>group.id==='height');
  if(r.graph.heightPrior){assert.equal(ground.dimension,1);assert.equal(ground.count,r.graph.stanceLandmarks);assert.equal(ground.residualDimension,ground.count);}
  else assert.equal(ground,undefined);
  const upright=r.graph.factorGroups.find(group=>group.id==='upright');
  if(r.graph.uprightPrior)assert.equal(upright.count,r.samples-1);
  else assert.equal(upright,undefined);
  assert(!r.graph.factorGroups.some(g=>g.id==='root'||g.factorType==='ContactPoseEqualityFactor'||g.factorType.includes('ContactNormalFactor')));
  assert.match(r.graph.factorGroups.find(group=>group.id==='contact').noise,/rotation 90°/);
  assert.equal(run.initializer,undefined);assert.equal(run.ikFrames,undefined);assert.equal(run.smoothing,undefined);
  assert(run.solver.iterations>0&&run.solver.iterations<=50);
  assert(Number.isFinite(run.solver.elapsedSeconds)&&run.solver.elapsedSeconds>0);
  assert.match(run.solver.timingScope,/excludes initialization/);
  const history=run.errorHistory;
  assert.equal(history.length,run.solver.iterations+1);
  assert.equal(history[0].iteration,0);
  const ids=r.graph.factorGroups.map(group=>group.id).sort();
  for(let i=0;i<history.length;i++){
   const row=history[i];assert.equal(row.iteration,i);
   assert.deepEqual(Object.keys(row.factorErrors).sort(),ids);
   assert(Object.values(row.factorErrors).every(value=>Number.isFinite(value)&&value>=0));
   assert(Math.abs(Object.values(row.factorErrors).reduce((a,b)=>a+b,0)-row.total)<Math.max(1,row.total)*1e-10);
   assert(row.elapsedSeconds>=0&&row.elapsedSeconds<=run.solver.elapsedSeconds);
   if(i){assert(row.elapsedSeconds>history[i-1].elapsedSeconds);assert(row.total<=history[i-1].total+Math.max(1,row.total)*1e-8);}
   assert.equal(row.poseOffset,undefined);
  }
  for(const id of ids)assert(Math.abs(history.at(-1).factorErrors[id]-run.final.factorErrors[id])<Math.max(1,run.final.factorErrors[id])*1e-9);
  assert(Math.abs(history.at(-1).total-run.final.metrics.cost)<Math.max(1,run.final.metrics.cost)*1e-9);
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
test('prior experiments share exactly the same initialization and sampling',()=>{
 for(const imu of [false,true])for(const loop of [false,true]){
  const records=['none','upright','height','upright-height'].map(prior=>readRecording(new URL(`../public/variants/${prior}${imu?'-imu':''}${loop?'-loop':''}/`,import.meta.url),'initial').record);
  for(const r of records){
   assert.deepEqual(r.phases,records[0].phases);
   assert.deepEqual(r.sampling,records[0].sampling);
   assert.deepEqual(r.positionPlots.pelvisWorld.estimate,records[0].positionPlots.pelvisWorld.estimate);
   assert.deepEqual(r.contactInitialization,records[0].contactInitialization);
   assert.equal(r.runs.contact.initial.factorErrors.contact,records[0].runs.contact.initial.factorErrors.contact);
  }
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
  assert.deepEqual(run.errorHistory,finalData.record.runs.contact.errorHistory);
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
