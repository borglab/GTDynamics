import test from 'node:test';
import assert from 'node:assert/strict';
import {alignedPositionPlots} from '../src/referenceAlignment.js';
import {readRecording} from '../../shared/kinematics/recordingTestUtils.mjs';

test('world alignment transforms pelvis curves and leaves body-relative feet unchanged',()=>{
 const plots={pelvisWorld:{estimate:[[5,6,7]],reference:[[1,2,3]]},feetBody:{left:{reference:[[.1,.2,-.8]]}}};
 const aligned=alignedPositionPlots(plots,{rotation:[[0,-1,0],[1,0,0],[0,0,1]],translation:[4,5,6]});
 assert.deepEqual(aligned.pelvisWorld.reference,[[2,6,9]]);
 assert.equal(aligned.feetBody,plots.feetBody);assert.equal(aligned.pelvisWorld.estimate,plots.pelvisWorld.estimate);
 assert.deepEqual(plots.pelvisWorld.reference,[[1,2,3]]);
 assert.equal(alignedPositionPlots(plots,undefined),plots);
});
for(const prior of ['none','upright','height','upright-height'])for(const imu of ['','-imu'])for(const loop of ['','-loop'])for(const stage of ['initial','final']){
 test(`recorded SE(3) alignment: ${prior}${imu}${loop}/${stage}`,()=>{
  const {record:r}=readRecording(new URL(`../public/variants/${prior}${imu}${loop}/`,import.meta.url),stage);
  const fit=r.referenceAlignment;
  assert.equal(fit.direction,'estimate_T_reference');assert.equal(fit.scale,1);assert.equal(fit.samples,r.samples);
  const rows=fit.rotation;
  for(let i=0;i<3;i++)for(let j=0;j<3;j++)assert(Math.abs(rows[i].reduce((sum,x,k)=>sum+x*rows[j][k],0)-(i===j?1:0))<1e-12);
  const det=rows[0][0]*(rows[1][1]*rows[2][2]-rows[1][2]*rows[2][1])-rows[0][1]*(rows[1][0]*rows[2][2]-rows[1][2]*rows[2][0])+rows[0][2]*(rows[1][0]*rows[2][1]-rows[1][1]*rows[2][0]);
  assert(Math.abs(det-1)<1e-12);
  const plots=alignedPositionPlots(r.positionPlots,fit);
  const rms=points=>Math.sqrt(points.reduce((sum,p,k)=>sum+p.reduce((s,x,j)=>s+(x-r.positionPlots.pelvisWorld.estimate[k][j])**2,0),0)/r.samples);
  assert(Math.abs(rms(plots.pelvisWorld.reference)-fit.rmsAfterMeters)<1e-10);
  assert(Math.abs(rms(r.positionPlots.pelvisWorld.reference)-fit.rmsBeforeMeters)<1e-10);
  assert(fit.rmsAfterMeters<=fit.rmsBeforeMeters+1e-10);
 });
}
