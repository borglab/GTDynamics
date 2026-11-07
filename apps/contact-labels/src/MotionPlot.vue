<script setup>
import { themeColor, onThemeChange } from '../../shared/theme.js';
import {onMounted,onBeforeUnmount,ref,watch} from 'vue';
const props=defineProps({times:Array,series:Array,frame:Number,label:String,unit:String,thresholds:Array,range:Array,reset:Number});
const emit=defineEmits(['select','range']);const element=ref(null),error=ref('');let Plotly,observer,disposed=false,ready=false,syncing=0;
const full=()=>[props.times[0],props.times.at(-1)];
const shapes=()=>[
 {type:'line',xref:'x',yref:'paper',x0:props.times[props.frame],x1:props.times[props.frame],y0:0,y1:1,line:{color:'#7e8d83',width:1,dash:'dot'}},
 ...(props.thresholds||[]).map((value,i)=>({type:'line',xref:'paper',yref:'y',x0:0,x1:1,y0:value,y1:value,line:{color:i?'#c45c39':'#087f71',width:1,dash:'dash'}})),
];
async function draw(){await Plotly.react(element.value,props.series.map((series,i)=>({x:props.times,y:series.values,name:series.name,type:'scatter',mode:'lines',line:{color:i?'#735ca8':'#087f71',width:1.7},hovertemplate:`%{y:.3f} ${props.unit}<extra>${series.name}</extra>`})),{height:205,margin:{l:65,r:18,t:12,b:40},font:{family:'system-ui',size:11,color:themeColor('muted')},paper_bgcolor:themeColor('panel'),plot_bgcolor:themeColor('panel'),showlegend:true,legend:{orientation:'h',x:0,y:1.15},dragmode:'zoom',hovermode:'x unified',uirevision:props.label,xaxis:{title:{text:'Clip time (s)'},range:props.range||full(),gridcolor:themeColor('grid'),zeroline:false},yaxis:{title:{text:props.unit},gridcolor:themeColor('grid'),zeroline:false,rangemode:'tozero'},shapes:shapes()},{scrollZoom:true,displayModeBar:true,displaylogo:false,responsive:false,modeBarButtonsToRemove:['select2d','lasso2d']});}
function layout(update){if(!ready||disposed)return;syncing++;return Plotly.relayout(element.value,update).finally(()=>syncing--);}
const stopTheme = onThemeChange(() => { if (Plotly && ready && !disposed && element.value) draw(); });
onBeforeUnmount(stopTheme);
onMounted(async()=>{const target=element.value;try{Plotly=(await import('plotly.js-basic-dist-min')).default;if(disposed)return;await draw();if(disposed){Plotly.purge(target);return;}ready=true;element.value.dataset.ready='true';element.value.on('plotly_click',e=>{if(e.points?.length)emit('select',e.points[0].pointIndex);});element.value.on('plotly_relayout',e=>{if(syncing)return;if(e['xaxis.autorange']){emit('range',null);return;}const range=e['xaxis.range']||[e['xaxis.range[0]'],e['xaxis.range[1]']];if(range.every(Number.isFinite))emit('range',range);});observer=new ResizeObserver(()=>{if(!disposed)Plotly.Plots.resize(element.value);});observer.observe(element.value);}catch(e){if(!disposed)error.value=e.message;}});
watch(()=>props.frame,()=>layout({'shapes[0].x0':props.times[props.frame],'shapes[0].x1':props.times[props.frame]}));
watch([()=>props.series,()=>props.thresholds],()=>{if(ready&&!disposed)draw();});
watch([()=>props.range,()=>props.reset],([range,reset],[,old])=>{const next=range||full(),current=element.value?.layout?.xaxis.range;if(reset!==old)layout({'xaxis.range':next,'xaxis.autorange':false,'yaxis.autorange':true});else if(current&&next.some((v,i)=>Math.abs(v-current[i])>1e-8))layout({'xaxis.range':next,'xaxis.autorange':false});});
onBeforeUnmount(()=>{disposed=true;observer?.disconnect();if(Plotly&&element.value)Plotly.purge(element.value);ready=false;});
</script>
<template><figure class="motion-plot"><figcaption>{{label}}</figcaption><p v-if="error" role="alert">{{error}}</p><div ref="element" class="chart" role="img" :aria-label="`${label} over clip time`"></div></figure></template>
