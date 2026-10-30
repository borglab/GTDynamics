import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFile} from 'node:fs/promises';
import test from 'node:test';
import {composePoses,inversePose,quaternionToRotation,transformPoint} from '../src/poseMath.js';

const publicRoot=new URL('../public/',import.meta.url);
const repoRoot=new URL('../../../',import.meta.url);
const registry=JSON.parse(await readFile(new URL('sequences.json',publicRoot),'utf8'));
const entries=await Promise.all(registry.sequences.map(async entry=>({
  ...entry,data:JSON.parse(await readFile(new URL(entry.data,publicRoot),'utf8')),
  recording:JSON.parse(await readFile(new URL(entry.recording,publicRoot),'utf8')),
})));
const sha256=bytes=>createHash('sha256').update(bytes).digest('hex');

test('sequence catalogue includes a genuine longer capture with explicit contact evidence',()=>{
  assert.equal(entries.length,2);
  const [policy,capture]=entries;
  assert.equal(policy.data.metadata.sampleCount,800);
  assert(capture.durationSeconds>policy.durationSeconds);
  assert.equal(capture.data.samples.length,1800);
  assert.equal(capture.contactEvidence,'kinematic');
  assert(capture.data.samples.every(s=>!('normalLoads' in s)),'No fabricated force measurements');
  assert.equal(capture.data.metadata.contactEvidence,'kinematic');
});

for(const entry of entries){
  test(`${entry.id}: recording identifies exact native source and input data`,async()=>{
    const recording=entry.recording;
    assert.equal(recording.solver,'C++ GTSAM Levenberg–Marquardt');
    assert.equal(recording.sequenceId,entry.id);
    assert.equal(recording.source.referenceExcludedFromNativeInput,true);
    const bytes=await readFile(new URL(registry.sequences.find(s=>s.id===entry.id).data,publicRoot));
    assert.equal(recording.source.datasetSha256,sha256(bytes));
    assert(Object.keys(recording.source.nativeSourceSha256).length>=5);
    for(const [path,hash] of Object.entries(recording.source.nativeSourceSha256))
      assert.equal(hash,sha256(await readFile(new URL(path,repoRoot))),`Native recording is stale for ${path}`);
    assert(recording.hardware.cpu.length>3);
  });
  for(const model of ['ground','patch','point']){
    test(`${entry.id}/${model}: native history, fixed anchors and exact contact objective agree`,()=>{
      const {graph,result,benchmark,environment}=entry.recording.models[model];
      assert.equal(result.backend,'native-cpp');
      assert.equal(environment.solver,'gtsam::LevenbergMarquardtOptimizer');
      assert.equal(environment.buildType,'Release');
      assert.equal(graph.frames.length,entry.data.frames.length);
      assert.equal(graph.landmarks.length,graph.contacts.length*(model==='point'?1:3));
      assert.equal(result.history.length,result.iterations+1);
      assert.equal(result.history[0].iteration,0);
      assert.equal(result.history.at(-1).iteration,result.iterations);
      for(let i=1;i<result.history.length;i++)assert(result.history[i].objective<=result.history[i-1].objective+1e-10);
      for(const anchor of graph.anchorFrameIndices){
        for(const snapshot of result.history)assert.deepEqual(snapshot.poses[anchor],graph.seedPoses[anchor]);
      }
      let objective=0,count=0;
      for(let i=0;i<graph.frames.length;i++)for(const o of graph.frames[i].observations){
        const predicted=transformPoint(result.poses[i],o.local),point=result.landmarks[o.landmarkIndex];
        objective+=predicted.reduce((sum,v,j)=>sum+(v-point[j])**2,0);count++;
      }
      assert(Math.abs(objective-result.objectiveFinal)<1e-9,'Reported native error has no hidden terms');
      assert(Math.abs(Math.sqrt(objective/count)-result.residualRms)<1e-10);
      if(model==='ground')for(const snapshot of result.history)
        for(const point of snapshot.landmarks)assert.equal(point[2],graph.groundPlane.height);
      if(model==='point')assert.equal(graph.observability.fullyConstrained,false);
      for(const phase of Object.values(benchmark.phases)){
        assert.equal(phase.samplesMs.length,benchmark.repetitions);
        const sorted=phase.samplesMs.slice().sort((a,b)=>a-b);
        assert.equal(phase.medianMs,sorted[Math.floor(sorted.length/2)]);
        assert.equal(phase.minMs,sorted[0]);
        assert.equal(phase.p95Ms,sorted[Math.ceil(sorted.length*.95)-1]);
        assert(sorted.every(t=>Number.isFinite(t)&&t>=0));
      }
      assert.equal(benchmark.medianMs,benchmark.phases.optimization.medianMs);
    });
  }
}

test('display transforms compose and invert without providing an optimizer',()=>{
  const p={rotation:quaternionToRotation([.8,.2,.3,.4]),translation:[1,-2,3]};
  const identity=composePoses(p,inversePose(p));
  assert(Math.hypot(...identity.translation)<1e-12);
  assert(Math.hypot(...transformPoint(identity,[1,2,3]).map((v,i)=>v-[1,2,3][i]))<1e-12);
});
