<script setup>
import {computed,markRaw,onBeforeUnmount,onMounted,ref} from 'vue';
import {clearRecordingCache,loadRecording,referenceLabel} from '../../shared/kinematics/recording.js';
import {disposeRobotTemplates} from '../../shared/kinematics/scene.js';
import Timeline from '../../shared/kinematics/Timeline.vue';
import Attribution from '../../shared/kinematics/Attribution.vue';
import MethodPanel from './MethodPanel.vue';
const data=ref(null),error=ref(''),frame=ref(0),stage=ref('smoothing'),showReference=ref(true),joint=ref(0);
const selection=ref('turn'),useImu=ref(true),recordings=ref([]),timeline=ref(null);
const directory=computed(()=>`snippets/${selection.value}/${useImu.value?'imu/':''}`);
let controller;
async function changeRecording(value=selection.value,aided=useImu.value){
  timeline.value?.pause();controller?.abort();
  const request=new AbortController();controller=request;
  frame.value=value===selection.value?frame.value:0;
  selection.value=value;useImu.value=aided;data.value=null;error.value='';
  try{
    const loaded=await loadRecording(request.signal,directory.value,stage.value);
    if(request.signal.aborted)return;
    data.value=markRaw(loaded);joint.value=Math.max(0,loaded.record.jointNames.indexOf('left_knee_joint'));
  }catch(e){if(!request.signal.aborted)error.value=e.message;}
}
function changeStage(value){
  timeline.value?.pause();stage.value=value;
  if(!data.value||!data.value.record.availableStages.includes(value))changeRecording();
}
onMounted(async()=>{
  const request=new AbortController();controller=request;
  try{
    const response=await fetch(`${import.meta.env.BASE_URL}snippets.json`,{signal:request.signal});
    if(!response.ok)throw new Error('Could not load snippet index');
    recordings.value=await response.json();
    if(!request.signal.aborted)await changeRecording();
  }catch(e){if(!request.signal.aborted)error.value=e.message;}
});
onBeforeUnmount(()=>{controller?.abort();clearRecordingCache();disposeRobotTemplates();});
</script>
<template>
  <main :data-stage="stage" :data-frame="frame" :data-imu="useImu" :data-snippet="selection">
    <header class="page-header"><div><span class="eyebrow">GTDYNAMICS / G1</span><h1>Contact vs IK</h1><p>Kinematic smoothing from contact and inverse-kinematics initialization.</p></div><span class="recorded-badge">OFFLINE RECORDING</span></header>
    <section class="toolbar"><div class="recording-controls">
      <div class="stage-selector" role="group" aria-label="Inspection stage"><button :aria-pressed="stage==='smoothing'" @click="changeStage('smoothing')">CM-Opt smoothing</button><button :aria-pressed="stage==='initializer'" @click="changeStage('initializer')">Initialization</button></div>
      <select :value="selection" aria-label="Snippet" @change="changeRecording($event.target.value)"><option v-for="r in recordings" :key="r.id" :value="r.id">{{r.label}}</option></select>
      <label><input type="checkbox" :checked="useImu" @change="changeRecording(selection,$event.target.checked)"> IMU aiding</label>
    </div><div v-if="data" class="settings"><label><input type="checkbox" v-model="showReference"> {{referenceLabel}}</label><label>Joint <select v-model.number="joint" aria-label="Joint to inspect"><option v-for="(name,index) in data.record.jointNames" :key="name" :value="index">{{name}}</option></select></label></div></section>
    <div v-if="error" role="alert" class="error-banner">{{error}}</div>
    <div v-else-if="!data" role="status" class="loading">Loading {{stage==='smoothing'?'smoothing iterations':'initializer histories'}}…</div>
    <template v-else>
      <p class="snippet-note">{{data.record.samples}} slices at {{1/data.record.dt}} Hz. Both methods solve the same graph: {{data.record.graph.costFactors.toLocaleString()}} cost factors and {{data.record.graph.equalities.toLocaleString()}} kinematic equalities. CM-Opt limit: {{data.record.maxIterations}} iterations.</p>
      <Timeline ref="timeline" :record="data.record" v-model:frame="frame" />
      <div class="comparison"><MethodPanel v-for="method in ['contact','ik']" :key="method" :method="method" :stage="stage" :frame="frame" :show-reference="showReference" :data="data" :joint="joint" /></div>
      <Attribution :record="data.record" :directory="directory" />
    </template>
  </main>
</template>
