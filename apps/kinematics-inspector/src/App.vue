<script setup>
import {computed,markRaw,nextTick,onBeforeUnmount,onMounted,ref,watch} from 'vue';
import {clearRecordingCache,historyFor,loadRecording,referenceLabel} from '../../shared/kinematics/recording.js';
import {disposeRobotTemplates,ReplayScene} from '../../shared/kinematics/scene.js';
import Timeline from '../../shared/kinematics/Timeline.vue';
import FactorTable from '../../shared/kinematics/FactorTable.vue';
import BiasTable from '../../shared/kinematics/BiasTable.vue';
import Attribution from '../../shared/kinematics/Attribution.vue';
import TracePlot from '../../shared/kinematics/TracePlot.vue';
const data=ref(null),error=ref(''),frame=ref(0),iteration=ref(0),useImu=ref(true),useLoop=ref(true),prior=ref(50);
const showReference=ref(true),showSlip=ref(true),slipThreshold=ref(5),view=ref('trajectory'),replaying=ref(false);
const sceneElement=ref(null),timeline=ref(null),sceneError=ref('');
const directory=computed(()=>`variants/${prior.value}${useImu.value?'-imu':''}${useLoop.value?'-loop':''}/`);
const history=computed(()=>data.value?historyFor(data.value.record,'contact','smoothing',frame.value):[]);
const state=computed(()=>history.value[Math.min(iteration.value,history.value.length-1)]);
const run=computed(()=>data.value?.record.runs.contact);
const finalIteration=computed(()=>history.value.at(-1)?.iteration??0);
const metrics=computed(()=>state.value?.metrics);
const contactSlip=computed(()=>{
  if(!data.value||state.value?.contactSlipOffset===undefined)return [];
  return Array.from({length:2},(_,foot)=>{
    const offset=state.value.contactSlipOffset+(frame.value*2+foot)*7;
    return data.value.floats[offset+6];
  });
});
let controller,scene,timer;
const number=(value,digits=3)=>value==null?'—':Math.abs(value)<1e-3&&value!==0?value.toExponential(2):value.toFixed(digits);
function stopIterations(){clearInterval(timer);timer=null;replaying.value=false;}
function selectIteration(value){stopIterations();iteration.value=Number(value);}
function replay(){
  if(replaying.value){stopIterations();return;}
  iteration.value=0;replaying.value=true;
  timer=setInterval(()=>{if(iteration.value>=history.value.length-1)stopIterations();else iteration.value++;},650);
}
function updateScene(){
  if(scene&&state.value)scene.update(state.value,frame.value,showReference.value,state.value,{showSlip:showSlip.value,slipThreshold:slipThreshold.value/1000});
}
function camera(value){view.value=value;if(value==='trajectory')scene?.fitPaths();else scene?.reset();}
async function load(){
  timeline.value?.pause();stopIterations();controller?.abort();
  const request=new AbortController();controller=request;data.value=null;error.value='';sceneError.value='';
  try{
    const loaded=await loadRecording(request.signal,directory.value,'smoothing');
    if(request.signal.aborted)return;
    if(Object.keys(loaded.record.runs).join(',')!=='contact')throw new Error('This viewer requires a contact-only trajectory recording');
    iteration.value=loaded.record.runs.contact.smoothing.length-2;
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
onBeforeUnmount(()=>{controller?.abort();stopIterations();scene?.dispose();scene=null;clearRecordingCache();disposeRobotTemplates();});
</script>
<template>
  <main class="kinematics-app" :data-frame="frame" :data-iteration="iteration" :data-imu="useImu" :data-loop-closure="useLoop" :data-prior="prior">
    <header class="page-header"><div><span class="eyebrow">GTDYNAMICS / G1</span><h1>Kinematics</h1><p>Full-trajectory contact-initialized smoothing · 219.8 s at 5 Hz.</p></div><span class="recorded-badge">OFFLINE RECORDING</span></header>
    <section class="toolbar"><div class="recording-controls">
      <label>Stationary contact σ <select v-model.number="prior" aria-label="Stationary contact sigma" @change="load"><option :value="50">50 mm · loose</option><option :value="20">20 mm · medium</option><option :value="5">5 mm · tight</option></select></label>
      <label><input type="checkbox" v-model="useImu" @change="load"> IMU aiding</label><label><input type="checkbox" v-model="useLoop" @change="load"> Loop closure applied</label>
    </div><div class="settings"><label><input type="checkbox" v-model="showReference"> {{referenceLabel}}</label><label><input type="checkbox" v-model="showSlip"> Foot slip</label></div></section>
    <div v-if="error" role="alert" class="error-banner">{{error}}</div>
    <div v-else-if="!data" role="status" class="loading">Loading trajectory and recorded iterations…</div>
    <template v-else>
      <section class="trajectory-viewer">
        <header class="viewer-controls"><div class="button-group"><button :aria-pressed="view==='trajectory'" @click="camera('trajectory')">Fit trajectory</button><button :aria-pressed="view==='robot'" @click="camera('robot')">Follow robot</button></div><label v-if="showSlip">Slip threshold <input type="range" min="1" max="50" v-model.number="slipThreshold" aria-label="Slip threshold"> {{slipThreshold}} mm / interval</label></header>
        <div ref="sceneElement" class="scene large-scene"><p v-if="sceneError" role="alert" class="error-banner">{{sceneError}}</p></div>
        <div class="path-legend"><span class="estimated-path">Solid: estimated root path</span><span v-if="showReference" class="superodom-path">Dashed: {{referenceLabel}}</span><span v-if="showSlip" class="slip-legend">Yellow → red: continuing-contact displacement above {{slipThreshold}} mm per {{data.record.dt}} s</span></div>
        <div v-if="showSlip" class="slip-readout"><span v-for="(distance,foot) in contactSlip" :key="foot">{{foot===0?'Left':'Right'}} foot: {{distance>=0?`${number(distance*1000,2)} mm`:'no continuing contact'}}</span><span>Arrows show displacement direction, enlarged for visibility.</span></div>
        <Timeline ref="timeline" :record="data.record" v-model:frame="frame" />
      </section>
      <section class="iteration-controls trajectory-iterations"><div class="row"><h2>CM-Opt iterations</h2><span class="mono">Iteration {{state.iteration}} / {{finalIteration}} · {{run.solver.iterations>=data.record.maxIterations?'iteration limit reached':'solver stopping criterion reached'}}</span></div>
        <input type="range" min="0" :max="history.length-1" :value="iteration" aria-label="Smoothing iteration" @input="selectIteration($event.target.value)">
        <div class="row"><div class="button-group"><button :disabled="iteration===0" @click="selectIteration(Math.max(0,iteration-1))" aria-label="Previous iteration">←</button><button @click="replay">{{replaying?'Pause iterations':'Replay iterations'}}</button><button :disabled="iteration===history.length-1" @click="selectIteration(Math.min(history.length-1,iteration+1))" aria-label="Next iteration">→</button></div><button @click="selectIteration(history.length-1)">Last iteration</button></div>
      </section>
      <FactorTable :record="data.record" :state="state" />
      <section class="physical-summary"><div><span>Stance displacement RMS / interval</span><strong>{{number(metrics.stance_step_slip_rms_m*1000)}} <small>mm</small></strong></div><div><span>Contact height RMS</span><strong>{{number(metrics.stance_height_rms_m*1000)}} <small>mm</small></strong></div><div><span>LiDAR-inertial reference position RMS</span><strong>{{number(metrics.reference_root_translation_rms_m)}} <small>m</small></strong></div><div><span>Final COM − start anchor · XYZ</span><strong>{{state.closurePositionErrorWorld.map(v=>number(v)).join(', ')}} <small>m</small></strong></div></section>
      <p class="model-note">Stationary contacts are soft 3D displacement costs, with σ={{prior}} mm per axis per {{data.record.dt}} s interval. Only this weight changes across the three contact settings. Foot-slip markers use the largest continuing sole-corner displacement per foot. Changing σ changes the objective weights; use physical residuals and the reference error when comparing settings.</p>
      <div class="diagnostics-grid"><section class="plot-block"><div class="row"><h3>Smoothing objective</h3><span class="mono">{{number(metrics.cost)}}</span></div><TracePlot :values="history.map(s=>s.metrics.cost)" :selected="iteration" interactive label="Smoothing objective by iteration" @select="selectIteration" /></section><BiasTable :bias="state.bias" :aiding="data.record.imuAiding" caption="Estimated IMU bias · selected iteration" /></div>
      <Attribution :record="data.record" :directory="directory" />
    </template>
  </main>
</template>
