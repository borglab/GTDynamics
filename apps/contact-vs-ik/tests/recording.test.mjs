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
  assert.deepEqual(Object.keys(r.runs),['contact','ik']);assert.equal(r.samples,500);assert.equal(r.dt,.02);
  assert.equal(r.motionSource.sourceWindow.startSeconds,snippet.start);
  assert.equal(r.motionSource.sourceWindow.endExclusiveSeconds,snippet.start+10);
  assert.equal(r.motionSource.detector.detectionRateHz,50);
  assert(r.motionSource.detector.unsupportedSamples<25);
  assert.equal(r.loopClosure.enabled,false);assert.equal(r.imuAiding.enabled,imu);
  assert.equal(r.motionSource.stationaryImuCalibration.sampleCount,5000);
  assert.equal(r.motionSource.licenseNotes[0].license,'CC-BY-4.0');
  assert(packed.floats.length<full.floats.length);
  for(const method of ['contact','ik']){
   const run=r.runs[method],small=packed.record.runs[method];
   assert.equal(small.smoothing.length,run.smoothing.length);assert.equal(small.initializer.length,0);assert.equal(small.ikFrames.length,0);
   assert(run.solver.iterations>0&&run.solver.iterations<=10);assert(run.smoothing.at(-1).metrics.kinematic_equality_l2<1e-5);
   for(let i=0;i<run.smoothing.length;i++){
    assert.deepEqual(poseBlock(packed,small.smoothing[i]),poseBlock(full,run.smoothing[i]));
    assert.deepEqual(small.smoothing[i].factorErrors,run.smoothing[i].factorErrors);
   }
   if(method==='contact'){assert.equal(run.initializer.length,run.initialization.iterations+1);}
   else{assert.equal(run.ikFrames.length,500);assert(run.ikFrames.every(h=>h.length>0));}
  }
  assert(existsSync(new URL(r.storage.model,directory)));
 });
}
