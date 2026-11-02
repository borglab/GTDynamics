<script setup>
import {nextTick,onBeforeUnmount,onMounted,ref,watch} from 'vue';
import {frameAtTime} from './recording.js';
const props=defineProps({title:String,unit:String,times:Array,series:Array,frame:Number});
const emit=defineEmits(['select']);
const element=ref(null);
let Plotly,observer,alive=true;
async function draw(){
  if(!Plotly||!element.value)return;
  const traces=props.series.map(series=>({type:'scatter',mode:'lines+markers',x:props.times,y:series.values,name:series.name,line:{color:series.color,width:1.6,dash:series.dash||'solid'},marker:{size:3},hovertemplate:'%{x:.2f} s<br>%{y:.2f} '+props.unit+'<extra>%{fullData.name}</extra>'}));
  const time=props.times[props.frame];
  await Plotly.react(element.value,traces,{
    height:260,margin:{l:60,r:22,t:18,b:64},paper_bgcolor:'transparent',plot_bgcolor:'#fff',
    font:{family:'Inter,system-ui,sans-serif',color:'#42574e',size:11},
    xaxis:{title:{text:'Dataset time (s)'},gridcolor:'#edf0e9',zeroline:false},
    yaxis:{title:{text:props.unit},gridcolor:'#edf0e9',zerolinecolor:'#bac7bb'},
    legend:{orientation:'h',x:0,y:-.28,font:{size:10}},
    shapes:[{type:'line',xref:'x',x0:time,x1:time,yref:'paper',y0:0,y1:1,line:{color:'#86958b',width:1,dash:'dot'}}],
    hovermode:'x unified',dragmode:'zoom',uirevision:props.title,
  },{responsive:true,displaylogo:false,scrollZoom:true,modeBarButtonsToRemove:['select2d','lasso2d','toImage']});
  if(alive&&element.value)element.value.dataset.ready='true';
}
onMounted(async()=>{
  const module=await import('plotly.js-basic-dist-min');
  if(!alive)return;
  Plotly=module.default||module;
  await nextTick();await draw();
  if(!alive)return;
  element.value.on('plotly_click',event=>emit('select',frameAtTime(props.times,event.points[0].x)));
  observer=new ResizeObserver(()=>Plotly.Plots.resize(element.value));observer.observe(element.value);
});
watch(()=>[props.series,props.times],draw);
watch(()=>props.frame,()=>{
  if(Plotly&&element.value?.data){
    const time=props.times[props.frame];Plotly.relayout(element.value,{'shapes[0].x0':time,'shapes[0].x1':time});
  }
});
onBeforeUnmount(()=>{alive=false;observer?.disconnect();if(Plotly&&element.value)Plotly.purge(element.value);});
</script>
<template><article class="signal-panel"><h3>{{title}}</h3><div ref="element" class="signal-plot" :aria-label="title" /></article></template>
