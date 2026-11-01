<script setup>
import {computed,markRaw,nextTick,onBeforeUnmount,onMounted,ref,watch} from 'vue';
import {clearRecordingCache,loadRecording,referenceLabel} from '../../shared/kinematics/recording.js';
import {disposeRobotTemplates,ReplayScene} from '../../shared/kinematics/scene.js';
import Timeline from '../../shared/kinematics/Timeline.vue';
import FactorTable from '../../shared/kinematics/FactorTable.vue';
import BiasTable from '../../shared/kinematics/BiasTable.vue';
import Attribution from '../../shared/kinematics/Attribution.vue';
import PositionPlots from './PositionPlots.vue';
const data=ref(null),error=ref(''),frame=ref(0),useImu=ref(true),useLoop=ref(true),prior=ref(50);
const estimate=ref('final');
const showReference=ref(true),showSlip=ref(true),slipThreshold=ref(5),view=ref('trajectory');
const sceneElement=ref(null),timeline=ref(null),sceneError=ref('');
const directory=computed(()=>`variants/${prior.value}${useImu.value?'-imu':''}${useLoop.value?'-loop':''}/`);
const state=computed(()=>data.value?.record.runs.contact[estimate.value]);
const metrics=computed(()=>state.value?.metrics);
const contactSlip=computed(()=>{
  if(!data.value||state.value?.contactSlipOffset===undefined)return [];
  return Array.from({length:2},(_,foot)=>{
    const offset=state.value.contactSlipOffset+(frame.value*2+foot)*7;
    return data.value.floats[offset+6];
  });
});
let controller,scene;
const number=(value,digits=3)=>value==null?'—':Math.abs(value)<1e-3&&value!==0?value.toExponential(2):value.toFixed(digits);
function updateScene(){
  if(scene&&state.value)scene.update(state.value,frame.value,showReference.value,state.value,{showSlip:showSlip.value,slipThreshold:slipThreshold.value/1000});
}
function camera(value){view.value=value;if(value==='trajectory')scene?.fitPaths();else scene?.reset();}
async function load(){
  timeline.value?.pause();controller?.abort();
  const request=new AbortController();controller=request;data.value=null;error.value='';sceneError.value='';
  try{
    const loaded=await loadRecording(request.signal,directory.value,estimate.value);
    if(request.signal.aborted)return;
    if(Object.keys(loaded.record.runs).join(',')!=='contact')throw new Error('This viewer requires a contact-only trajectory recording');
    data.value=markRaw(loaded);
  }catch(e){if(!request.signal.aborted)error.value=e.message;}
}
watch(data,async loaded=>{
  scene?.dispose();scene=null;
  if(!loaded)return;
  await nextTick();
  if(data.value!==loaded||!sceneElement.value)return;
  try{
    scene=new ReplayScene(sceneElement.value,loaded.model,loaded.record,loaded.floats,'#087f71');
    updateScene();camera(view.value);
  }catch(e){sceneError.value=e.message;}
});
watch([state,frame,showReference,showSlip,slipThreshold],updateScene);
onMounted(load);
onBeforeUnmount(()=>{controller?.abort();scene?.dispose();scene=null;clearRecordingCache();disposeRobotTemplates();});
</script>
<template>
  <main class="kinematics-app" :data-frame="frame" :data-imu="useImu" :data-loop-closure="useLoop" :data-prior="prior" :data-estimate="estimate">
    <header class="page-header"><div><span class="eyebrow">GTDYNAMICS / G1</span><h1>Kinematics</h1><p>Full-trajectory contact-initialized smoothing · buffered contact slices.</p></div><span class="recorded-badge">OFFLINE RECORDING</span></header>
    <section class="toolbar"><div class="recording-controls">
      <label>Estimate <select v-model="estimate" aria-label="Estimate" @change="load"><option value="final">Final</option><option value="initial">Initial</option></select></label>
      <label>Stationary contact σ <select v-model.number="prior" aria-label="Stationary contact sigma" @change="load"><option :value="50">50 mm · loose</option><option :value="20">20 mm · medium</option><option :value="5">5 mm · tight</option></select></label>
      <label><input type="checkbox" v-model="useImu" @change="load"> IMU aiding</label><label><input type="checkbox" v-model="useLoop" @change="load"> Loop closure applied</label>
    </div><div class="settings"><label><input type="checkbox" v-model="showReference"> {{referenceLabel}}</label><label><input type="checkbox" v-model="showSlip"> Foot slip</label></div></section>
    <div v-if="error" role="alert" class="error-banner">{{error}}</div>
    <div v-else-if="!data" role="status" class="loading">Loading {{estimate}} trajectory…</div>
    <template v-else>
      <section class="optimization-summary" aria-label="Optimization statistics"><strong>CM-Opt: {{number(data.record.runs.contact.solver.elapsedSeconds,2)}} s · {{data.record.runs.contact.solver.iterations}} iterations</strong><span v-if="data.record.runs.contact.solver.iterations>=data.record.maxIterations">Iteration limit reached ({{data.record.maxIterations}}).</span><small :title="data.record.runs.contact.solver.timingScope">Recorded wall time for manifold setup and solve; initialization and replay export excluded.</small></section>
      <section class="trajectory-viewer">
        <header class="viewer-controls"><div class="button-group"><button :aria-pressed="view==='trajectory'" @click="camera('trajectory')">Fit trajectory</button><button :aria-pressed="view==='robot'" @click="camera('robot')">Follow robot</button></div><label v-if="showSlip">Slip threshold <input type="range" min="1" max="50" v-model.number="slipThreshold" aria-label="Slip threshold"> {{slipThreshold}} mm / interval</label></header>
        <div ref="sceneElement" class="scene large-scene"><p v-if="sceneError" role="alert" class="error-banner">{{sceneError}}</p></div>
        <div class="path-legend"><span class="estimated-path">Solid: estimated root path</span><span v-if="showReference" class="superodom-path">Dashed: {{referenceLabel}}</span><span v-if="showSlip" class="slip-legend">Yellow → red: continuing-contact displacement above {{slipThreshold}} mm per selected slice interval</span></div>
        <div v-if="showSlip" class="slip-readout"><span v-for="(distance,foot) in contactSlip" :key="foot">{{foot===0?'Left':'Right'}} foot: {{distance>=0?`${number(distance*1000,2)} mm`:'no continuing contact'}}</span><span>Arrows show displacement direction, enlarged for visibility.</span></div>
        <Timeline ref="timeline" :record="data.record" v-model:frame="frame" />
      </section>
      <p v-if="data.record.sampling?.mode==='contact-initializer'" class="model-note">{{data.record.samples.toLocaleString()}} slices selected by the contact initializer with a one-sample (20 ms) stance buffer on the 50 Hz source grid. Slice intervals range from {{Math.min(...data.record.sampling.intervalSeconds).toFixed(2)}} to {{Math.max(...data.record.sampling.intervalSeconds).toFixed(2)}} s. Initialization and smoothing use the same slices; IMU preintegration and playback use their actual timestamps.</p>
      <FactorTable :record="data.record" :state="state" :estimate="estimate" />
      <section class="physical-summary"><div><span>Stance displacement RMS / interval</span><strong>{{number(metrics.stance_step_slip_rms_m*1000)}} <small>mm</small></strong></div><div><span>Contact height RMS</span><strong>{{number(metrics.stance_height_rms_m*1000)}} <small>mm</small></strong></div><div><span>LiDAR-inertial reference position RMS</span><strong>{{number(metrics.reference_root_translation_rms_m)}} <small>m</small></strong></div><div><span>End COM − start anchor · XYZ</span><strong>{{state.closurePositionErrorWorld.map(v=>number(v)).join(', ')}} <small>m</small></strong></div></section>
      <p class="model-note">Stationary contacts are soft 3D displacement costs, with σ={{prior}} mm per axis per selected slice interval. Only this weight changes across the three contact settings. Foot-slip markers use the largest continuing sole-corner displacement per foot. Changing σ changes the objective weights; use physical residuals and the reference error when comparing settings.</p>
      <PositionPlots :plots="data.record.positionPlots" :frame="frame" :dt="data.record.dt" :timestamps="data.record.timestamps" @select="timeline?.pause();frame=$event" />
      <BiasTable :bias="state.bias" :aiding="data.record.imuAiding" :caption="estimate==='initial'?'IMU bias · initial prior mean':'Estimated IMU bias · final result'" />
      <Attribution :record="data.record" :directory="directory" />
    </template>
  </main>
</template>
