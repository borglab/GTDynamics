import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFile} from 'node:fs/promises';
import test from 'node:test';
import {composePoses,inversePose,quaternionToRotation,transformPoint} from '../src/poseMath.js';
import {factorVariants} from '../src/factorVariants.js';

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
  assert.equal(capture.data.metadata.stats.graphComponents,2);
  assert.deepEqual(capture.data.metadata.detector.handoverCorrections,[{sampleIndex:563,confirmedOnsetSample:564,foot:'right'}]);
  const unsupported=capture.data.samples.filter(s=>!s.support.left&&!s.support.right).map(s=>s.index);
  assert.deepEqual(unsupported,Array.from({length:11},(_,i)=>1357+i),'Preserve the short hop instead of inventing contact');
});

for(const entry of entries){
  test(`${entry.id}: contact boundaries are retained and frame gaps never exceed one second`,()=>{
    const {frames,samples,contacts,metadata}=entry.data;
    const boundaries=new Set(contacts.flatMap(c=>[c.start,c.end]));
    const selected=new Set(frames.map(f=>f.sampleIndex));
    for(const index of boundaries)assert(selected.has(index),`Missing contact boundary ${index}`);
    assert.equal(selected.size,frames.length);
    for(let i=0;i<frames.length;i++){
      const frame=frames[i];
      assert.equal(frame.t,samples[frame.sampleIndex].t,'Use the actual measured timestamp');
      assert.equal(frame.selection,boundaries.has(frame.sampleIndex)?'contact':'periodic');
      assert.deepEqual(frame.contacts,contacts.filter(c=>c.start<=frame.sampleIndex&&frame.sampleIndex<c.endExclusive).map(c=>c.id));
      if(i)assert(frame.t-frames[i-1].t<=1+1e-8,`Frame gap exceeds one second at ${frame.t}`);
    }
    assert.equal(metadata.frameSampling.maxIntervalSeconds,1);
    assert.equal(metadata.frameSampling.periodicFrameCount,frames.length-boundaries.size);
  });
  test(`${entry.id}: recording identifies exact native source and input data`,async()=>{
    const recording=entry.recording;
    assert.equal(recording.solver,'C++ GTSAM Levenberg–Marquardt');
    assert.equal(recording.sequenceId,entry.id);
    assert.equal(recording.source.referenceExcludedFromOptimization,true);
    const bytes=await readFile(new URL(registry.sequences.find(s=>s.id===entry.id).data,publicRoot));
    assert.equal(recording.source.datasetSha256,sha256(bytes));
    assert(Object.keys(recording.source.nativeSourceSha256).length>=5);
    for(const [path,hash] of Object.entries(recording.source.nativeSourceSha256))
      assert.equal(hash,sha256(await readFile(new URL(path,repoRoot))),`Native recording is stale for ${path}`);
    assert(recording.hardware.cpu.length>3);
  });
  for(const model of ['ground','patch','point'])for(const choice of factorVariants){
    const {id:variant,between,upright,ground}=choice;
    test(`${entry.id}/${model}/${variant}: native history, fixed anchors and exact contact objective agree`,()=>{
      const run=variant==='none'?entry.recording.models[model]:entry.recording.variants[model][variant];
      const {graph,result,benchmark,environment,evaluation}=run;
      assert.equal(graph.regularization.between,between);
      assert.equal(graph.regularization.upright,upright);
      assert.equal(graph.regularization.ground,ground);
      assert.equal(graph.betweenEdges.length,between?graph.frames.length-1:0);
      assert.equal(graph.uprightPoseIndices.length,upright?graph.frames.length:0);
      assert.equal(graph.groundLandmarkIndices.length,ground&&model!=='ground'?graph.landmarks.length:0);
      assert.equal(graph.dimensions.groundFactors,graph.groundLandmarkIndices.length);
      assert.equal(graph.regularization.groundAlreadyFixed,model==='ground');
      if(model!=='ground'){
        assert.equal(graph.groundPlane,null,'Selectable models have no hard floor');
        assert.equal(graph.dimensions.effectiveLandmarkVariables,graph.landmarks.length*3);
        if(!ground){
          const heights=result.landmarks.map(p=>p[2]);
          assert(Math.max(...heights)-Math.min(...heights)>0.01,'Landmark heights are allowed to vary');
          assert.equal(result.objectiveComponents.ground,0);
        }
      }
      assert.equal(graph.contactComponents.length,entry.data.metadata.stats.graphComponents);
      assert.equal(graph.components.length,between?1:graph.contactComponents.length);
      assert(evaluation.positionRmseMeters>=0&&Number.isFinite(evaluation.positionRmseMeters));
      assert(evaluation.orientationRmseDegrees>=0&&Number.isFinite(evaluation.orientationRmseDegrees));
      assert.equal(evaluation.alignedReferencePoses.length,graph.frames.length);
      assert.equal(result.backend,'native-cpp');
      assert.equal(environment.solver,'gtsam::LevenbergMarquardtOptimizer');
      assert.equal(environment.buildType,'Release');
      assert.equal(graph.frames.length,entry.data.frames.length);
      graph.frames.forEach((frame,i)=>{
        const source=entry.data.frames[i];
        assert.equal(frame.sampleIndex,source.sampleIndex);
        assert.equal(frame.time,source.t);
        assert.deepEqual([...new Set(frame.observations.map(o=>o.contactId))],source.contacts);
      });
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
      assert(Math.abs(objective-result.objectiveComponents.contact)<1e-9,'Contact error excludes the explicit prior penalties');
      assert(Math.abs(Object.values(result.objectiveComponents).reduce((a,b)=>a+b,0)-result.objectiveFinal)<1e-9);
      assert.equal(result.objectiveComponents.between>0,between);
      if(!upright)assert.equal(result.objectiveComponents.upright,0);
      const groundCost=ground&&model!=='ground'?result.landmarks.reduce((sum,p)=>sum+((p[2]-graph.regularization.groundHeight)/graph.regularization.groundSigma)**2,0):0;
      assert(Math.abs(groundCost-result.objectiveComponents.ground)<1e-9,'Ground cost is height-only and counted once per landmark');
      if(ground){
        assert.equal(graph.regularization.groundSigma,0.02);
        assert(Number.isFinite(graph.regularization.groundHeight));
      }
      assert(Math.abs(Math.sqrt(objective/count)-result.residualRms)<1e-10);
      if(model==='ground')for(const snapshot of result.history)
        for(const point of snapshot.landmarks)assert.equal(point[2],graph.groundPlane.height);
      if(model==='point'&&!between)assert.equal(graph.observability.fullyConstrained,false);
      for(const phase of Object.values(benchmark.phases)){
        assert.equal(phase.samplesMs.length,benchmark.repetitions);
        const sorted=phase.samplesMs.slice().sort((a,b)=>a-b);
        assert.equal(phase.medianMs,sorted[Math.floor(sorted.length/2)]);
        assert.equal(phase.minMs,sorted[0]);
        assert.equal(phase.p95Ms,sorted[Math.ceil(sorted.length*.95)-1]);
        assert(sorted.every(t=>Number.isFinite(t)&&t>=0));
      }
      assert.equal(benchmark.medianMs,benchmark.phases.optimization.medianMs);
      const truth=evaluation.alignedReferencePoses;
      for(const saved of result.history){
        let position=0,angle=0;
        saved.poses.forEach((pose,i)=>{
          position+=pose.translation.reduce((sum,value,j)=>sum+(value-truth[i].translation[j])**2,0);
          const trace=pose.rotation.reduce((sum,value,j)=>sum+value*truth[i].rotation[j],0);
          angle+=Math.acos(Math.max(-1,Math.min(1,(trace-1)/2)))**2;
        });
        assert(Math.abs(saved.referenceError.positionRmseMeters-Math.sqrt(position/truth.length))<1e-10);
        assert(Math.abs(saved.referenceError.orientationRmseDegrees-180/Math.PI*Math.sqrt(angle/truth.length))<1e-5);
      }
    });
  }
}

test('display transforms compose and invert without providing an optimizer',()=>{
  const p={rotation:quaternionToRotation([.8,.2,.3,.4]),translation:[1,-2,3]};
  const identity=composePoses(p,inversePose(p));
  assert(Math.hypot(...identity.translation)<1e-12);
  assert(Math.hypot(...transformPoint(identity,[1,2,3]).map((v,i)=>v-[1,2,3][i]))<1e-12);
});
