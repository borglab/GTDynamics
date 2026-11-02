<script setup>
import {computed,onBeforeUnmount,ref,watch} from 'vue';
import {sliceTime,frameAtTime} from './recording.js';
const props=defineProps({record:Object,frame:Number});
const emit=defineEmits(['update:frame']);
const playing=ref(false);
let animation=0,origin=0,first=0;
const sourcePhases=computed(()=>props.record.contactPhaseInference?.phases);
const phases=computed(()=>sourcePhases.value||props.record.phases);
const sourceIndex=computed(()=>sourcePhases.value?props.record.sampling.sourceIndices[props.frame]:props.frame);
const phase=computed(()=>phases.value.find(p=>p.start<=sourceIndex.value&&p.end>=sourceIndex.value));
function jump(phase){
  if(!sourcePhases.value){scrub(phase.start);return;}
  const indices=props.record.sampling.sourceIndices;
  const middle=(phase.start+phase.end)/2;
  let nearest=0;
  for(let k=1;k<indices.length;k++)if(Math.abs(indices[k]-middle)<Math.abs(indices[nearest]-middle))nearest=k;
  scrub(nearest);
}
function pause(){cancelAnimationFrame(animation);playing.value=false;}
function scrub(value){pause();emit('update:frame',Number(value));}
function tick(time){
  if(!playing.value)return;
  const elapsed=Math.max(0,time-origin);
  const next=frameAtTime(props.record,sliceTime(props.record,first)+elapsed/1000);
  emit('update:frame',next);
  if(next===props.record.samples-1)pause();else animation=requestAnimationFrame(tick);
}
function play(){
  if(playing.value){pause();return;}
  first=props.frame===props.record.samples-1?0:props.frame;
  origin=performance.now();playing.value=true;animation=requestAnimationFrame(tick);
}
watch(()=>props.record,pause);onBeforeUnmount(pause);defineExpose({pause});
</script>
<template>
  <section class="timeline">
    <div class="row"><div class="time-heading"><button class="play-button" @click="play">{{playing?'Pause walking':'Play walking'}}</button><strong>{{sliceTime(record,frame).toFixed(2)}} <small>/ {{sliceTime(record,record.samples-1).toFixed(2)}} s</small></strong></div><span class="mono">Slice {{frame}} · phase {{phases.indexOf(phase)+1}} / {{phases.length}} · {{phase?.contacts.join(' + ')||'no detected support'}}</span></div>
    <input type="range" :value="frame" min="0" :max="record.samples-1" aria-label="Walking time" @input="scrub($event.target.value)">
    <div class="phase-strip" :class="{dense:phases.length>80,inferred:!!sourcePhases}" aria-label="Contact phases"><button v-for="(p,index) in phases" :key="index" :class="{active:p===phase,double:p.contacts.length===2,left:p.contacts.length===1&&p.contacts[0]==='left',right:p.contacts.length===1&&p.contacts[0]==='right'}" :style="{flex:p.end-p.start+1}" :title="`Phase ${index+1}: ${p.contacts.join(' + ')} · source slices ${p.start}–${p.end}`" :aria-label="`Jump to phase ${index+1}`" @click="jump(p)"></button></div>
    <div class="plot-foot"><span>{{sourcePhases?'Inferred phases: red left · green right · yellow double · click for nearest estimated slice':'Contact-aligned phases · click to jump'}}</span><span>{{record.samples}} slices · {{record.timestamps?'contact-selected, irregular intervals':`${1/record.dt} Hz`}} · source {{record.motionSource.sourceWindow.startSeconds}}–{{record.motionSource.sourceWindow.endExclusiveSeconds.toFixed(1)}} s</span></div>
  </section>
</template>
<style scoped>
.phase-strip.inferred .left{background:#dc4545}
.phase-strip.inferred .right{background:#25a66c}
.phase-strip.inferred .double{background:#e7bf32}
.phase-strip.inferred .active{outline-color:#243833}
</style>
