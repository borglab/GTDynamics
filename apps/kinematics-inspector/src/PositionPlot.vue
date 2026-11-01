<script setup>
import {computed,onBeforeUnmount,onMounted,ref,watch} from 'vue';
const props=defineProps({estimate:Array,reference:Array,frame:Number,dt:Number,timestamps:Array,label:String,timeRange:Array,resetVersion:Number});
const emit=defineEmits(['select','range']);
const element=ref(null),error=ref('');
const time=computed(()=>props.timestamps||props.estimate.map((_,i)=>i*props.dt));
const fullRange=()=>[time.value[0],time.value.at(-1)];
const sameRange=(a,b)=>a&&b&&a.every((v,i)=>Math.abs(v-b[i])<1e-8);
const format=value=>value.toFixed(Math.abs(value)<1?3:2);
let Plotly,observer,ready=false,disposed=false,syncing=0;

function cursor(){
  const current=time.value[props.frame];
  return {type:'line',xref:'x',yref:'paper',x0:current,x1:current,y0:0,y1:1,line:{color:'#a5b0a6',width:1,dash:'dot'}};
}
function draw(){
  return Plotly.react(element.value,[
    {x:time.value,y:props.estimate,type:'scatter',mode:'lines',name:'Estimate',line:{color:'#087f71',width:1.7},hovertemplate:'%{y:.4f} m<extra>Estimate</extra>'},
    {x:time.value,y:props.reference,type:'scatter',mode:'lines',name:'Reference',line:{color:'#7b8fa7',width:1.7,dash:'dash'},hovertemplate:'%{y:.4f} m<extra>Reference</extra>'},
  ],{
    autosize:true,height:210,margin:{l:64,r:20,t:28,b:43},
    font:{family:'ui-sans-serif, system-ui, sans-serif',size:11,color:'#6b7971'},
    paper_bgcolor:'#fff',plot_bgcolor:'#fff',showlegend:false,dragmode:'zoom',hovermode:'x unified',
    uirevision:props.label,
    xaxis:{title:{text:'Time (s)',standoff:8},range:props.timeRange||fullRange(),autorange:false,gridcolor:'#e5eae3',zeroline:false},
    yaxis:{title:{text:'m',standoff:8},gridcolor:'#e5eae3',zeroline:false,autorange:true},
    shapes:[cursor()],
  },{scrollZoom:true,responsive:false,displayModeBar:true,displaylogo:false,modeBarButtonsToRemove:['select2d','lasso2d']});
}
function relayout(update){
  if(!ready||disposed)return;
  syncing++;
  return Plotly.relayout(element.value,update).finally(()=>syncing--);
}
function onRange(update){
  if(syncing||disposed)return;
  if(update['xaxis.autorange']){emit('range',null);return;}
  const range=update['xaxis.range']||[update['xaxis.range[0]'],update['xaxis.range[1]']];
  if(range.every(Number.isFinite)&&!sameRange(range,props.timeRange||fullRange()))emit('range',range);
}
onMounted(async()=>{
  const target=element.value;
  try{
    Plotly=(await import('plotly.js-basic-dist-min')).default;
    if(disposed)return;
    await draw();
    if(disposed){Plotly.purge(target);return;}
    ready=true;
    element.value.on('plotly_relayout',onRange);
    element.value.on('plotly_click',event=>{
      if(event.points?.length)emit('select',event.points[0].pointIndex);
    });
    observer=new ResizeObserver(()=>{if(ready&&!disposed)Plotly.Plots.resize(element.value);});
    observer.observe(element.value);
    element.value.dataset.ready='true';
  }catch(e){if(!disposed)error.value=`Position plot unavailable: ${e.message}`;}
});
watch(()=>props.frame,()=>relayout({'shapes[0].x0':time.value[props.frame],'shapes[0].x1':time.value[props.frame]}));
watch([()=>props.estimate,()=>props.reference,()=>props.label],()=>{if(ready&&!disposed)draw();});
watch([()=>props.timeRange,()=>props.resetVersion],([range,reset],[,previousReset])=>{
  const update={};
  if(reset!==previousReset){update['yaxis.autorange']=true;update['xaxis.range']=range||fullRange();update['xaxis.autorange']=false;}
  else if(ready&&!sameRange(element.value.layout.xaxis.range,range||fullRange())){update['xaxis.range']=range||fullRange();update['xaxis.autorange']=false;}
  if(Object.keys(update).length)relayout(update);
});
onBeforeUnmount(()=>{disposed=true;observer?.disconnect();if(Plotly&&element.value)Plotly.purge(element.value);ready=false;});
</script>
<template>
  <figure class="position-plot">
    <figcaption><strong>{{label}} <small>m</small></strong><span>Estimate {{format(estimate[frame])}} · reference {{format(reference[frame])}}</span></figcaption>
    <div v-if="error" role="alert">{{error}}</div>
    <div ref="element" class="position-chart" role="img" :aria-label="`${label} position: estimate and reference over time in seconds`"></div>
  </figure>
</template>
