import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {Quaternion,Vector3} from 'three';
import {frameAtTime,jointGroups,validateRecording} from '../src/recording.js';

const read=name=>JSON.parse(gunzipSync(readFileSync(new URL(`../public/${name}`,import.meta.url))));
const recording=read('recording.json.gz'),model=read('model.json.gz');

test('dynamics replay contains finite fixed-kinematics states at original irregular times',()=>{
  validateRecording(recording,model);
  assert(recording.times[0]>=78&&recording.times.at(-1)<=98);
  assert(recording.times.at(-1)-recording.times[0]>18);
  assert(recording.times.length>20&&recording.times.length<300);
  const intervals=recording.times.slice(1).map((time,k)=>time-recording.times[k]);
  assert(new Set(intervals.map(time=>Math.round(time*1000))).size>1);
  for(const poses of [...recording.bodyPoses,...recording.contactPoses])for(const pose of poses){
    assert(Math.abs(Math.hypot(...pose.slice(3))-1)<1e-5,'Saved quaternion is XYZW and unit length');
  }
  const repeated=structuredClone(recording);repeated.times[1]=repeated.times[0];
  assert.throws(()=>validateRecording(repeated,model),/Timestamps must increase/);
  const corrupt=structuredClone(recording);corrupt.wrenches[0][0][0]=NaN;
  assert.throws(()=>validateRecording(corrupt,model),/Invalid wrenches/);
});

test('world forces and moments are rotations of torque-first sole-local wrenches',()=>{
  for(let k=0;k<recording.times.length;k++)for(let foot=0;foot<2;foot++){
    const quaternion=new Quaternion().fromArray(recording.contactPoses[k][foot],3);
    const wrench=recording.wrenches[k][foot];
    const force=new Vector3().fromArray(wrench,3).applyQuaternion(quaternion);
    const moment=new Vector3().fromArray(wrench).applyQuaternion(quaternion);
    assert(force.distanceTo(new Vector3().fromArray(recording.worldForces[k][foot]))<1e-4);
    assert(moment.distanceTo(new Vector3().fromArray(recording.worldMoments[k][foot]))<1e-4);
    if(!recording.contact[k][foot])assert(Math.hypot(...wrench)<1e-6,'Swing-foot wrench is zero');
  }
});

test('every saved contact wrench obeys compression, circular friction, CoP, and coupled yaw bounds',()=>{
  const settings=recording.settings,tolerance=1e-4;
  for(let k=0;k<recording.times.length;k++)for(let foot=0;foot<2;foot++){
    const [halfLength,halfWidth]=settings.supportRectangles[foot];
    const [mx,my,mz,fx,fy,fz]=recording.wrenches[k][foot];
    const mu=settings.friction/Math.sqrt(2);
    assert(fz>=-tolerance,`No tensile normal force at slice ${k}, foot ${foot}`);
    assert(Math.hypot(fx,fy)<=settings.friction*fz+tolerance,'Resultant force remains inside circular Coulomb cone');
    assert(Math.abs(fx)<=mu*fz+tolerance&&Math.abs(fy)<=mu*fz+tolerance,'Conservative pyramid bounds');
    assert(Math.abs(mx)<=halfWidth*fz+tolerance&&Math.abs(my)<=halfLength*fz+tolerance,'Resultant CoP stays in support rectangle');
    const lower=-mu*(halfLength+halfWidth)*fz+Math.abs(halfWidth*fx-mu*mx)+Math.abs(halfLength*fy-mu*my);
    const upper=mu*(halfLength+halfWidth)*fz-Math.abs(halfWidth*fx+mu*mx)-Math.abs(halfLength*fy+mu*my);
    assert(mz>=lower-tolerance&&mz<=upper+tolerance,'Yaw moment obeys the coupled surface-contact bounds');
  }
});

test('irregular-time playback uses the nearest retained slice without interpolation',()=>{
  const times=[78.02,78.18,78.64,79.10];
  assert.equal(frameAtTime(times,0),0);assert.equal(frameAtTime(times,100),3);
  assert.equal(frameAtTime(times,78.20),1);assert.equal(frameAtTime(times,78.55),2);
  for(let k=0;k<recording.times.length;k++)assert.equal(frameAtTime(recording.times,recording.times[k]),k);
});

test('optional torque plots include every actuator exactly once',()=>{
  const groups=jointGroups(recording.jointNames),indices=groups.flatMap(group=>group.indices);
  assert.equal(indices.length,29);assert.equal(new Set(indices).size,29);
  assert.deepEqual([...indices].sort((a,b)=>a-b),Array.from({length:29},(_,i)=>i));
  assert(groups.some(group=>group.title==='Left leg'));
  assert(groups.some(group=>group.title==='Right arm'));
});

test('replay publishes dynamics diagnostics and source attribution',()=>{
  assert(recording.source);assert(recording.provenance);assert(recording.settings);
  assert(recording.diagnostics?.solver);assert(recording.diagnostics.factors.length>0);
  assert(recording.diagnostics.factors.every(factor=>factor.type&&Number.isInteger(factor.count)&&factor.count>0));
  assert(recording.attribution.length>0);
  for(const name of ['MODEL_LICENSE.txt','SSIO_ATTRIBUTION.txt'])assert(readFileSync(new URL(`../public/${name}`,import.meta.url),'utf8').length>100);
});

test('optimized accelerations and their observation costs use the recorded units',()=>{
  assert.equal(recording.settings.accelerationsEstimated,true);
  assert.equal(recording.diagnostics.softDynamics,false);
  assert(recording.diagnostics.maxForceResidual<1e-8);
  assert(recording.diagnostics.maxMomentResidual<1e-8);
  const settings=recording.settings,radiansPerDegree=Math.PI/180;
  const angularSigma=settings.baseAngularAccelerationSigmaDegreesPerSecondSquared*radiansPerDegree;
  const linearSigma=settings.baseLinearAccelerationSigmaMetersPerSecondSquared;
  const jointSigma=settings.jointAccelerationSigmaDegreesPerSecondSquared*radiansPerDegree;
  let error=0;
  for(let k=0;k<recording.times.length;k++){
    for(let axis=0;axis<6;axis++){
      const residual=recording.baseTwistAccelerations[k][axis]-recording.baseAccelerationPriors[k][axis];
      error+=.5*(residual/(axis<3?angularSigma:linearSigma))**2;
    }
    for(let joint=0;joint<recording.jointNames.length;joint++){
      const residual=recording.jointAccelerations[k][joint]-recording.jointAccelerationPriors[k][joint];
      error+=.5*(residual/jointSigma)**2;
    }
  }
  const group=recording.diagnostics.factors.find(factor=>factor.type==='GP velocity-difference acceleration priors');
  assert(group);assert.equal(group.count,recording.times.length*30);
  assert(Math.abs(error-group.error)<1e-8);
  const total=recording.diagnostics.factors.reduce((sum,factor)=>sum+factor.error,0);
  assert(Math.abs(total-recording.diagnostics.objectiveFinal)<1e-8);
  const invalid=structuredClone(recording);invalid.jointAccelerations[0][0]=NaN;
  assert.throws(()=>validateRecording(invalid,model),/Invalid jointAccelerations/);
});
