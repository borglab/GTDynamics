import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync,existsSync} from 'node:fs';
import {readRecording,poseBlock} from '../../shared/kinematics/recordingTestUtils.mjs';
const snippets=JSON.parse(readFileSync(new URL('../public/snippets.json',import.meta.url)));
test('exactly three ten-second snippets are offered',()=>{
 assert.deepEqual(snippets.map(s=>s.start),[40,100,204]);assert.equal(snippets.length,3);
});
for(const snippet of snippets)for(const imu of [false,true]){
 test(`complete recorded comparison and staged loading: ${snippet.id}, imu=${imu}`,()=>{
  const directory=new URL(`../public/snippets/${snippet.id}/${imu?'imu/':''}`,import.meta.url);
  const full=readRecording(directory),packed=readRecording(directory,true),r=full.record;
  assert.deepEqual(Object.keys(r.runs),['contact','ik']);assert(r.samples>2&&r.samples<500);assert.equal(r.dt,.02);
  assert.equal(r.sampling.mode,'double-support');assert.equal(r.sampling.maxSliceIntervalMs,200);
  assert.equal(r.sampling.interpolatedSolveStates,false);
  assert.equal(r.timestamps.length,r.samples);assert.equal(r.sampling.sourceIndices.length,r.samples);
  const all=Array.from({length:r.samples},(_,i)=>i);
  assert.deepEqual(r.runs.contact.sparseSamples,all);
  assert(r.phases.every(p=>p.contacts.length===2));
  for(const p of r.phases)for(let k=p.start+1;k<=p.end;k++)assert(r.timestamps[k]-r.timestamps[k-1]<=.2+1e-12);
  assert.equal(r.motionSource.sourceWindow.startSeconds,snippet.start);
  assert.equal(r.motionSource.sourceWindow.endExclusiveSeconds,snippet.start+10);
  assert.equal(r.contactPhaseInference.method,'binary-foot max-product');
  assert.equal(r.contactPhaseInference.sourceSlices,500);
  assert.equal(r.loopClosure.enabled,false);assert.equal(r.imuAiding.enabled,imu);
  assert.equal(r.motionSource.stationaryImuCalibration.sampleCount,5000);
  assert.equal(r.motionSource.licenseNotes[0].license,'CC-BY-4.0');
  assert(packed.floats.length<full.floats.length);
  assert.equal(r.graph.motionPrior,false);
  assert.equal(r.graph.contactTranslationSigmaMeters,.01);
  assert.equal(r.graph.contactRotationSigmaDegrees,90);
  assert(r.graph.stanceLandmarks>0);
  assert.equal(r.graph.factorGroups.find(g=>g.id==='contact').factorType,'ContactPoseFactor');
  assert(!r.graph.factorGroups.some(g=>g.factorType==='ContactPoseEqualityFactor'));
  const init=r.runs.contact.initialization;
  assert(init.initialize_upright&&init.upright_prior&&init.height_prior&&init.motion_prior);
  assert.equal(init.rotation_sigma_degrees,90);assert.equal(init.translation_sigma_m,.01);
  assert.equal(init.height_sigma_m,.1);assert.equal(init.ground_height_m,0);
  assert.equal(init.normal_prior,false);assert(Math.abs(init.upright_sigma_degrees-15)<1e-12);
  assert.equal(typeof init.stalled,'boolean');if(init.stalled)assert.equal(init.converged,false);
  assert.equal(r.initializationFrame.anchorSourceIndex,r.sampling.sourceIndices[0]);
  assert(r.initializationFrame.worldTranslationMeters.every(Number.isFinite));
  for(let axis=0;axis<3;axis++)assert(Math.abs(r.motionSource.worldTranslationMeters[axis]-(r.initializationFrame.inputWorldTranslationMeters[axis]+r.initializationFrame.worldTranslationMeters[axis]))<1e-12);
  for(const method of ['contact','ik']){
   const run=r.runs[method],small=packed.record.runs[method];
   assert.equal(small.smoothing.length,run.smoothing.length);assert.equal(small.initializer.length,0);assert.equal(small.ikFrames.length,0);
   assert(run.solver.iterations>0&&run.solver.iterations<=10);assert(run.smoothing.at(-1).metrics.kinematic_equality_l2<1e-5);
   for(let i=0;i<run.smoothing.length;i++){
    assert.deepEqual(poseBlock(packed,small.smoothing[i]),poseBlock(full,run.smoothing[i]));
    assert.deepEqual(small.smoothing[i].factorErrors,run.smoothing[i].factorErrors);
   }
   if(method==='contact'){assert.equal(run.initializer.length,run.initialization.iterations+1);}
   else{assert.equal(run.ikFrames.length,r.samples);assert(run.ikFrames.every(h=>h.length>0));}
  }
  assert(existsSync(new URL(r.storage.model,directory)));
 });
}
