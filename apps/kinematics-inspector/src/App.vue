<script setup>
import KinematicsMath from '../../shared/paper/KinematicsMath.vue';
import AppAbstract from '../../shared/paper/AppAbstract.vue';
import AppPaper from '../../shared/paper/AppPaper.vue';
import {computed,markRaw,nextTick,onBeforeUnmount,onMounted,ref,watch} from 'vue';
import {clearRecordingCache,loadRecording,referenceLabel} from '../../shared/kinematics/recording.js';
import {disposeRobotTemplates,ReplayScene} from '../../shared/kinematics/scene.js';
import Timeline from '../../shared/kinematics/Timeline.vue';
import FactorTable from '../../shared/kinematics/FactorTable.vue';
import BiasTable from '../../shared/kinematics/BiasTable.vue';
import Attribution from '../../shared/kinematics/Attribution.vue';
import PositionPlots from './PositionPlots.vue';
import ErrorHistory from './ErrorHistory.vue';
import VariantComparison from './VariantComparison.vue';
import {alignedPositionPlots} from './referenceAlignment.js';
const data=ref(null),error=ref(''),frame=ref(0),useImu=ref(true),useLoop=ref(true),useUpright=ref(false),useHeight=ref(false);
const estimate=ref('final'),alignReference=ref(true),comparison=ref(null);
const alignment=computed(()=>alignReference.value?data.value?.record.referenceAlignment:undefined);
const referenceName=computed(()=>`${referenceLabel}${alignReference.value?' · SE(3) aligned':''}`);
const positionPlots=computed(()=>data.value?alignedPositionPlots(data.value.record.positionPlots,alignment.value):null);
const referenceRms=computed(()=>data.value?.record.referenceAlignment?.[alignReference.value?'rmsAfterMeters':'rmsBeforeMeters']);
const showReference=ref(true),showSlip=ref(true),slipThreshold=ref(5),view=ref('trajectory');
const sceneElement=ref(null),timeline=ref(null),sceneError=ref('');
const prior=computed(()=>[useUpright.value?'upright':'',useHeight.value?'height':''].filter(Boolean).join('-')||'none');
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
const comparisonController=new AbortController();
const number=(value,digits=3)=>value==null?'—':Math.abs(value)<1e-3&&value!==0?value.toExponential(2):value.toFixed(digits);
function updateScene(){
  if(scene&&state.value)scene.update(state.value,frame.value,showReference.value,state.value,{showSlip:showSlip.value,slipThreshold:slipThreshold.value/1000,referenceAlignment:alignment.value});
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
async function selectVariant(row){
  useUpright.value=row.upright;useHeight.value=row.height;useImu.value=row.imu;useLoop.value=row.loop;
  estimate.value='final';
  await load();
}
async function initialize(){
  try{
    const response=await fetch('comparison.json',{signal:comparisonController.signal});
    if(!response.ok)throw new Error(`Could not load comparison: HTTP ${response.status}`);
    comparison.value=markRaw(await response.json());
    await selectVariant(comparison.value.variants.find(row=>row.id===comparison.value.bestVariant));
  }catch(e){if(!comparisonController.signal.aborted)error.value=e.message;}
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
watch([state,frame,showReference,showSlip,slipThreshold,alignment],updateScene);
onMounted(initialize);
onBeforeUnmount(()=>{comparisonController.abort();controller?.abort();scene?.dispose();scene=null;clearRecordingCache();disposeRobotTemplates();});
</script>
<template>
  <main id="demo" class="kinematics-app" :data-frame="frame" :data-imu="useImu" :data-loop-closure="useLoop" :data-prior="prior" :data-estimate="estimate">
    <header class="page-header"><div><span class="eyebrow">GTDYNAMICS / G1</span><h1>Kinematics</h1><AppAbstract app="kinematics-inspector" /></div><span class="recorded-badge">OFFLINE RECORDING</span></header>
    <section class="toolbar"><div class="recording-controls">
      <label>Estimate <select v-model="estimate" aria-label="Estimate" @change="load"><option value="final">Final</option><option value="initial">Initial</option></select></label>
      <label><input type="checkbox" v-model="useUpright" @change="load"> Upright prior</label>
      <label><input type="checkbox" v-model="useHeight" @change="load"> Foot height prior</label>
      <label><input type="checkbox" v-model="useImu" @change="load"> IMU aiding</label><label><input type="checkbox" v-model="useLoop" @change="load"> Loop closure applied</label>
    </div><div class="settings"><label><input type="checkbox" v-model="showReference"> {{referenceLabel}}</label><label><input type="checkbox" v-model="alignReference"> Aligned reference (SE(3))</label><label><input type="checkbox" v-model="showSlip"> Foot slip</label></div></section>
    <div v-if="error" role="alert" class="error-banner">{{error}}</div>
    <div v-else-if="!data" role="status" class="loading">Loading {{estimate}} trajectory…</div>
    <template v-else>
      <section class="trajectory-viewer">
        <header class="viewer-controls"><div class="button-group"><button :aria-pressed="view==='trajectory'" @click="camera('trajectory')">Fit trajectory</button><button :aria-pressed="view==='robot'" @click="camera('robot')">Follow robot</button></div><label v-if="showSlip">Slip threshold <input type="range" min="1" max="50" v-model.number="slipThreshold" aria-label="Slip threshold"> {{slipThreshold}} mm / interval</label></header>
        <div ref="sceneElement" class="scene large-scene"><p v-if="sceneError" role="alert" class="error-banner">{{sceneError}}</p></div>
        <Timeline ref="timeline" :record="data.record" v-model:frame="frame" />
        <div class="path-legend"><span class="estimated-path">Solid: estimated root path</span><span v-if="showReference" class="superodom-path">Dashed: {{referenceName}}</span><span v-if="showSlip" class="slip-legend">Yellow → red: continuing-contact displacement above {{slipThreshold}} mm per selected slice interval</span></div>
        <div v-if="showSlip" class="slip-readout"><span v-for="(distance,foot) in contactSlip" :key="foot">{{foot===0?'Left':'Right'}} foot: {{distance>=0?`${number(distance*1000,2)} mm`:'no continuing contact'}}</span><span>Arrows show displacement direction, enlarged for visibility.</span></div>
      </section>
      <AppPaper app="kinematics-inspector" :frame="frame" :samples="data.record.samples" @select="timeline?.pause();frame=$event">
        <template #mathematics><KinematicsMath :record="data.record" /></template>
      <section class="optimization-summary" aria-label="Optimization statistics"><strong>CM-Opt: {{number(data.record.runs.contact.solver.elapsedSeconds,2)}} s · {{data.record.runs.contact.solver.iterations}} iterations</strong><span v-if="data.record.runs.contact.solver.iterations>=data.record.maxIterations">Iteration limit reached ({{data.record.maxIterations}}).</span><small :title="data.record.runs.contact.solver.timingScope">Recorded wall time includes manifold setup, solve, and factor-error recording; initialization and robot replay export excluded.</small></section>
      <p v-if="data.record.sampling?.mode==='double-support'" class="model-note">{{data.record.samples.toLocaleString()}} double-stance slices from the binary contact graph: one central slice per short yellow phase (≤ {{data.record.sampling.longPhaseSeconds*1000}} ms), otherwise evenly spaced slices between inward-buffered endpoints (maximum {{data.record.sampling.maxSliceIntervalMs}} ms within each phase). Initialization and smoothing use exactly these slices, without interpolated solve states. IMU pims use the actual intervals, from {{Math.min(...data.record.sampling.intervalSeconds).toFixed(2)}} to {{Math.max(...data.record.sampling.intervalSeconds).toFixed(2)}} s.</p>
      <p v-if="data.record.contactInitialization" class="initializer-settings">Contact initializer: rotation σ={{number(data.record.contactInitialization.rotation_sigma_degrees,0)}}° per axis; translation σ={{number(data.record.contactInitialization.translation_sigma_m*1000,0)}} mm per axis. Upright σ={{number(data.record.contactInitialization.upright_sigma_degrees,0)}}°; height σ={{number(data.record.contactInitialization.height_sigma_m*100,0)}} cm; motion σ={{number(data.record.contactInitialization.motion_translation_sigma_m,0)}} m / {{number(data.record.contactInitialization.motion_rotation_sigma_degrees,0)}}°. {{data.record.contactInitialization.iterations}} iterations · {{data.record.contactInitialization.stalled?'stalled: no accepted decrease':data.record.contactInitialization.converged?'stopping criterion reached':'iteration limit reached'}}.</p>
      <VariantComparison v-if="comparison" :summary="comparison" :selected="directory.split('/')[1]" @select="selectVariant" />
      <ErrorHistory :history="data.record.runs.contact.errorHistory" :groups="data.record.graph.factorGroups" />
      <FactorTable :record="data.record" :state="state" :estimate="estimate" />
      <section class="physical-summary"><div><span>Stance displacement RMS / interval</span><strong>{{number(metrics.stance_step_slip_rms_m*1000)}} <small>mm</small></strong></div><div><span>Contact height RMS</span><strong>{{number(metrics.stance_height_rms_m*1000)}} <small>mm</small></strong></div><div><span>Pelvis reference position RMS{{alignReference?' · SE(3) aligned':' · original frame'}}</span><strong>{{number(referenceRms)}} <small>m</small></strong></div><div><span>End COM − start anchor · XYZ</span><strong>{{state.closurePositionErrorWorld.map(v=>number(v)).join(', ')}} <small>m</small></strong></div></section>
      <p class="model-note">{{data.record.graph.stanceLandmarks}} world-frame stance pose landmarks, each shared by all observations during that stance. ContactPoseFactor uses translation σ=10 mm and rotation σ=90° per observation. Smoothing uses no weak motion prior or sole-normal prior. Optional root upright σ=15° and landmark height σ=10 cm priors are independent of the initializer's priors. Foot-slip markers remain a displacement diagnostic.</p>
      <p v-if="alignReference" class="model-note">Reference aligned by one SE(3) least-squares fit to the displayed estimate’s pelvis positions over all {{data.record.samples}} retained slices, with equal weight per slice and no scale change. This affects display and reference-position RMS only.</p>
      <PositionPlots :plots="positionPlots" :reference-label="referenceName" :frame="frame" :dt="data.record.dt" :timestamps="data.record.timestamps" @select="timeline?.pause();frame=$event" />
      <BiasTable :bias="state.bias" :aiding="data.record.imuAiding" :caption="estimate==='initial'?'IMU bias · initial prior mean':'Estimated IMU bias · final result'" />
      </AppPaper>
      <Attribution :record="data.record" :directory="directory" />
    </template>
  </main>
</template>
