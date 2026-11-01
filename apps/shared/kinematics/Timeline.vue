<script setup>
import {computed,onBeforeUnmount,ref,watch} from 'vue';
import {sliceTime,frameAtTime} from './recording.js';
const props=defineProps({record:Object,frame:Number});
const emit=defineEmits(['update:frame']);
const playing=ref(false);
let animation=0,origin=0,first=0;
const phase=computed(()=>props.record.phases.find(p=>p.start<=props.frame&&p.end>=props.frame));
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
    <div class="row"><div class="time-heading"><button class="play-button" @click="play">{{playing?'Pause walking':'Play walking'}}</button><strong>{{sliceTime(record,frame).toFixed(2)}} <small>/ {{sliceTime(record,record.samples-1).toFixed(2)}} s</small></strong></div><span class="mono">Frame {{frame}} · phase {{record.phases.indexOf(phase)+1}} / {{record.phases.length}} · {{phase?.contacts.join(' + ')||'no detected support'}}</span></div>
    <input type="range" :value="frame" min="0" :max="record.samples-1" aria-label="Walking time" @input="scrub($event.target.value)">
    <div class="phase-strip" :class="{dense:record.phases.length>80}" aria-label="Contact phases"><button v-for="(p,index) in record.phases" :key="index" :class="{active:p===phase,double:p.contacts.length===2}" :style="{flex:p.end-p.start+1}" :title="`Phase ${index+1}: frames ${p.start}–${p.end}`" :aria-label="`Jump to phase ${index+1}`" @click="scrub(p.start)"></button></div>
    <div class="plot-foot"><span>Contact-aligned phases · click to jump</span><span>{{record.samples}} samples · {{record.sampling?.mode==='contact-initializer'?'contact-selected, irregular intervals':`${1/record.dt} Hz`}} · source {{record.motionSource.sourceWindow.startSeconds}}–{{record.motionSource.sourceWindow.endExclusiveSeconds.toFixed(1)}} s</span></div>
  </section>
</template>
