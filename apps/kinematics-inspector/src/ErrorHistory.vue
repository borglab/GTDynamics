<script setup>
import { themeColor, onThemeChange } from '../../shared/theme.js';
import {onBeforeUnmount,onMounted,ref,watch} from 'vue';
const props=defineProps({history:Array,groups:Array});
const element=ref(null),axis=ref('iteration'),logScale=ref(true),error=ref('');
const colors={joint:'#2878b5',contact:'#087f71',anchor:'#89919b',height:'#d38a16',upright:'#ad4994',imu:'#ce493d',bias:'#7c65a9',velocity:'#8b6a42',closure:'#348e9a'};
let Plotly,observer,ready=false,disposed=false;
async function draw(){
  const history=props.history||[];
  const traces=[{id:'total',label:'Total'},...props.groups.map(group=>({id:group.id,label:group.label}))].map(({id,label})=>({
    x:history.map(row=>row[axis.value==='iteration'?'iteration':'elapsedSeconds']),
    y:history.map(row=>{const value=id==='total'?row.total:row.factorErrors[id];return logScale.value&&value===0?null:value;}),
    name:label,type:'scatter',mode:'lines+markers',marker:{size:4},
    line:{color:id==='total'?themeColor('text'):colors[id],width:id==='total'?3:1.6},
    hovertemplate:'%{y:.6g}<extra>%{fullData.name}</extra>',
  }));
  await Plotly.react(element.value,traces,{
    autosize:true,height:460,margin:{l:80,r:20,t:24,b:145},
    font:{family:'ui-sans-serif, system-ui, sans-serif',size:11,color:themeColor('muted')},
    paper_bgcolor:themeColor('panel'),plot_bgcolor:themeColor('panel'),dragmode:'zoom',hovermode:'x unified',
    legend:{orientation:'h',x:0,y:-.28},
    xaxis:{title:{text:axis.value==='iteration'?'Optimization iteration':'Elapsed solve time (s)'},gridcolor:themeColor('grid'),zeroline:false},
    yaxis:{title:{text:'Objective error'},type:logScale.value?'log':'linear',gridcolor:themeColor('grid'),zeroline:false},
  },{scrollZoom:true,responsive:false,displaylogo:false,modeBarButtonsToRemove:['select2d','lasso2d']});
}
const stopTheme = onThemeChange(() => { if (Plotly && ready && !disposed && element.value) draw(); });
onBeforeUnmount(stopTheme);
onMounted(async()=>{
  const target=element.value;
  try{
    Plotly=(await import('plotly.js-basic-dist-min')).default;
    if(disposed)return;
    await draw();
    if(disposed){Plotly.purge(target);return;}
    ready=true;target.dataset.ready='true';
    observer=new ResizeObserver(()=>{if(ready&&!disposed)Plotly.Plots.resize(target);});
    observer.observe(target);
  }catch(e){if(!disposed)error.value=`Error history unavailable: ${e.message}`;}
});
watch([axis,logScale,()=>props.history],()=>{if(ready&&!disposed)draw().catch(e=>{error.value=e.message;});});
onBeforeUnmount(()=>{disposed=true;ready=false;observer?.disconnect();if(Plotly&&element.value)Plotly.purge(element.value);});
</script>
<template>
  <section class="position-section error-history-section">
    <div class="row"><h2>Optimization error history</h2><div class="position-tools">
      <label>Horizontal axis <select v-model="axis" aria-label="Error history horizontal axis"><option value="iteration">Iteration</option><option value="time">Elapsed solve time</option></select></label>
      <label><input v-model="logScale" type="checkbox"> Logarithmic error scale</label>
    </div></div>
    <p class="scope">Each curve sums ½‖whitened residual‖² for one factor type over the entire trajectory. The total sums all types. Iteration 0 is the feasible starting estimate. Legend entries toggle curves, and dragging or scrolling zooms.<span v-if="logScale"> The logarithmic scale displays positive errors only.</span></p>
    <div v-if="error" role="alert">{{error}}</div>
    <div ref="element" class="error-history-chart" role="img" aria-label="Optimization objective error by factor type and total"></div>
  </section>
</template>
<style scoped>
.error-history-chart{width:100%;height:460px;min-width:0;overflow:hidden}
.error-history-section .row{flex-wrap:wrap;gap:12px}
</style>
