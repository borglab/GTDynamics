<script setup>
import {computed,ref} from 'vue';
import PositionPlot from './PositionPlot.vue';
const props=defineProps({plots:Object,frame:Number,dt:Number,timestamps:Array,referenceLabel:{type:String,default:'LiDAR-inertial reference'}});
const emit=defineEmits(['select']);
const foot=ref('left'),timeRange=ref(null),resetVersion=ref(0);
const footLabel=computed(()=>foot.value==='left'?'Left foot':'Right foot');
const axis=(series,index)=>series.map(point=>point[index]);
const pelvis=computed(()=>({estimate:axis(props.plots.pelvisWorld.estimate,2),reference:axis(props.plots.pelvisWorld.reference,2)}));
const feet=computed(()=>[0,1,2].map(i=>({estimate:axis(props.plots.feetBody[foot.value].estimate,i),reference:axis(props.plots.feetBody[foot.value].reference,i)})));
function reset(){timeRange.value=null;resetVersion.value++;}
</script>
<template>
  <section class="position-section">
    <div class="row"><h2>Position comparison</h2><div class="position-tools"><button @click="reset">Reset plot zoom</button><label>Foot <select v-model="foot" aria-label="Foot to plot"><option value="left">Left</option><option value="right">Right</option></select></label></div></div>
    <p class="scope">The curves compare the <span class="position-estimate-key">estimated positions (solid)</span> with <span class="position-reference-key">{{referenceLabel}} and measured-joint positions (dashed)</span>. The plots share a time range. Dragging or scrolling zooms, the toolbar enables panning, and clicking a curve selects time.</p>
    <div class="position-groups">
      <div class="position-group"><h3>Pelvis · world frame</h3><p>The height is measured at the MuJoCo pelvis body origin, in metres.</p><PositionPlot label="Pelvis world z" v-bind="pelvis" :frame="frame" :dt="dt" :timestamps="timestamps" :time-range="timeRange" :reset-version="resetVersion" @range="timeRange=$event" @select="emit('select',$event)" /></div>
      <div class="position-group"><h3>{{footLabel}} · pelvis body frame</h3><p>The ankle-roll body origin is expressed in each trajectory's own pelvis frame.</p><PositionPlot v-for="(name,i) in ['x','y','z']" :key="name" :label="`${footLabel} body ${name}`" v-bind="feet[i]" :frame="frame" :dt="dt" :timestamps="timestamps" :time-range="timeRange" :reset-version="resetVersion" @range="timeRange=$event" @select="emit('select',$event)" /></div>
    </div>
  </section>
</template>
