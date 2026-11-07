<script setup>
import AppAbstract from '../../shared/paper/AppAbstract.vue';
import AppPaper from '../../shared/paper/AppPaper.vue';
import SignalPlot from '../../shared/dynamics/SignalPlot.vue';
import {computed,markRaw,nextTick,onBeforeUnmount,onMounted,ref,watch} from 'vue';
import {frameAtTime,loadPresets,loadRecording,segmentAtFrame} from './recording.js';
import {CmrgScene} from './scene.js';

const presets=ref([]),presetId=ref(''),data=ref(null),error=ref(''),frame=ref(0),playing=ref(false);
const sceneElement=ref(null),view=ref('fit'),speed=ref(1);
const showPlanned=ref(true),showReplan=ref(true),showCable=ref(true);
const controller=new AbortController();
let scene,animation=0,origin=0,start=0;
const record=computed(()=>data.value?.record);
const times=computed(()=>record.value?.frames.time||[]);
const sampleTime=computed(()=>times.value[frame.value]||0);
const segments=computed(()=>record.value?.segments||[]);
const segmentIndex=computed(()=>segmentAtFrame(segments.value,frame.value));
const segment=computed(()=>segments.value[segmentIndex.value]);
const numbers=computed(()=>segments.value.map((_,i)=>i+1));
const number=(value,digits=2)=>typeof value==='number'?Math.abs(value)>0&&Math.abs(value)<.001?value.toExponential(2):value.toLocaleString('en-US',{maximumFractionDigits:digits}):'—';
const millimetres=value=>number(value*1000,1);
const plots=computed(()=>{
  if(!segments.value.length)return [];
  const colors=['#d44747','#219469','#4c6fb2'];
  const series=(names,get,palette=colors)=>names.map((name,i)=>({name,color:palette[i],values:segments.value.map(s=>get(s,i))}));
  return [
    {title:'Tool error after each maneuver',unit:'mm',series:series(['Arm 1 gripper','Arm 2 nailer'],(s,i)=>s.toolError[i]*1000)},
    {title:'Least sphere clearance',unit:'m',series:series(['Floor','Placed wood','Between arms'],(s,i)=>s.clearance[['floor','wood','cross_arm'][i]])},
    {title:'Solve time',unit:'s',series:series(['GPMP2 plan','STEAP execution'],(s,i)=>i===0?s.planSeconds:s.steapSeconds)},
  ];
});
function pause(){cancelAnimationFrame(animation);playing.value=false;}
function scrub(value){pause();frame.value=Number(value);}
function jump(index){scrub(segments.value[Math.min(Math.max(index,0),segments.value.length-1)].frameStart);}
function tick(now){
  if(!playing.value)return;
  const time=start+(now-origin)/1000*speed.value;
  frame.value=frameAtTime(times.value,time);
  if(time>=times.value.at(-1))pause();else animation=requestAnimationFrame(tick);
}
function play(){
  if(playing.value){pause();return;}
  if(frame.value===times.value.length-1)frame.value=0;
  start=sampleTime.value;origin=performance.now();playing.value=true;animation=requestAnimationFrame(tick);
}
function camera(mode){view.value=mode;if(mode==='follow')scene?.follow();else scene?.fit();}
function update(){scene?.update(frame.value,{showPlanned:showPlanned.value,showReplan:showReplan.value,showCable:showCable.value});}
watch([frame,showPlanned,showReplan,showCable],update);
async function open(id){
  pause();scene?.dispose();scene=null;data.value=null;frame.value=0;error.value='';
  try{
    const preset=presets.value.find(item=>item.id===id)||presets.value[0];
    presetId.value=preset.id;
    data.value=markRaw(await loadRecording(preset.file,controller.signal));
    if(controller.signal.aborted)return;
    await nextTick();
    scene=new CmrgScene(sceneElement.value,data.value.model,data.value.record);
    update();
  }catch(exception){if(!controller.signal.aborted)error.value=exception.message;}
}
onMounted(async()=>{
  try{presets.value=await loadPresets(controller.signal);await open(presets.value[0].id);}
  catch(exception){if(!controller.signal.aborted)error.value=exception.message;}
});
onBeforeUnmount(()=>{controller.abort();pause();scene?.dispose();});
</script>
<template>
  <main id="demo" class="steap-app" :data-frame="frame">
    <header class="page-header"><div><span class="eyebrow">GTDYNAMICS / BAR LAB CMRG</span><h1>CAST: A Collaborative Construction Application of STEAP</h1><AppAbstract app="steap-cmrg" /></div><span class="recorded-badge"><i />OFFLINE GPMP2 PLAN · SIMULATED STEAP EXECUTION</span></header>
    <div v-if="error" class="error-banner" role="alert">{{error}}</div>
    <div v-else-if="!data" class="loading" role="status">Loading assembly recording…</div>
    <template v-else>
      <section class="steap-viewer">
        <header class="viewer-tools"><div class="button-group"><button :aria-pressed="view==='fit'" @click="camera('fit')">Fit cell</button><button :aria-pressed="view==='follow'" @click="camera('follow')">Follow gripper</button><select v-if="presets.length>1" class="preset-select" :value="presetId" aria-label="Structure" @change="open($event.target.value)"><option v-for="preset in presets" :key="preset.id" :value="preset.id">{{preset.title}}</option></select></div><span class="path-legend"><span><i class="planned" />Nominal plan</span><span><i class="replan" />Replanned remainder</span><span><i class="executed" />Executed</span><span><i class="cable" />Dress cable (constraint spheres)</span></span></header>
        <div ref="sceneElement" class="steap-scene" />
        <section class="timeline">
          <div class="row"><div class="time-heading"><button class="play-button" @click="play">{{playing?'Pause build':'Play build'}}</button><strong>{{sampleTime.toFixed(1)}} <small>/ {{times.at(-1).toFixed(1)}} s</small></strong></div><span class="mono">Maneuver {{segmentIndex+1}} / {{segments.length}} · {{segment.label}}</span></div>
          <input type="range" :value="frame" min="0" :max="times.length-1" aria-label="Build time" @input="scrub($event.target.value)">
          <div class="display-options"><label>Speed <select v-model.number="speed" aria-label="Playback speed"><option :value=".1">0.1×</option><option :value=".25">0.25×</option><option :value=".5">0.5×</option><option :value="1">1×</option><option :value="2">2×</option><option :value="4">4×</option><option :value="8">8×</option><option :value="16">16×</option></select></label><label><input type="checkbox" v-model="showPlanned"> Nominal plan</label><label><input type="checkbox" v-model="showReplan"> Replanned remainder</label><label><input type="checkbox" v-model="showCable"> Dress cables</label><button @click="jump(segmentIndex-1)">◀ Previous</button><button @click="jump(segmentIndex+1)">Next ▶</button></div>
        </section>
      </section>
      <AppPaper app="steap-cmrg" :frame="segmentIndex" :samples="segments.length" @select="jump">
      <section class="maneuver-readout" aria-label="Current maneuver"><article><h3>{{segment.member}}</h3><strong>{{segment.label.split(': ')[1]}}</strong><span>Duration {{number(segment.duration,1)}} s · graph error {{number(segment.graphError,3)}}</span></article><article><h3>Tool error at end</h3><strong>{{millimetres(segment.toolError[0])}} <small>mm gripper</small></strong><span>Nailer {{millimetres(segment.toolError[1])}} mm · disturbance ≤ {{number(segment.disturbanceMax,4)}}</span></article><article><h3>Replanning shift</h3><strong>{{number(segment.replanShiftMax,4)}} <small>max command change</small></strong><span>m (gantry) or rad (joints) versus the nominal plan</span></article><article><h3>Least clearance</h3><strong>{{number(Math.min(segment.clearance.floor,segment.clearance.wood,segment.clearance.cross_arm),3)}} <small>m</small></strong><span>Floor {{number(segment.clearance.floor,2)}} · wood {{number(segment.clearance.wood,2)}} · arms {{number(segment.clearance.cross_arm,2)}}</span></article></section>
      <section class="signals"><div class="row"><h2>Maneuver results</h2><span class="scope">Select a row or plot point to move the demo to that maneuver. Distances are in metres; tool error is the gap between the nominal and the simulated measured tool position.</span></div><div class="plot-row"><SignalPlot v-for="plot in plots" :key="plot.title" :title="plot.title" :unit="plot.unit" :times="numbers" :series="plot.series" :frame="segmentIndex" x-title="Maneuver" @select="jump" /></div></section>
      <section class="solve-section"><h2>Maneuver table</h2><div class="table-scroll"><table class="factor-table maneuver-table"><thead><tr><th>#</th><th>Maneuver</th><th>Duration (s)</th><th>Graph error</th><th>Plan (s)</th><th>STEAP (s)</th><th>Gripper error (mm)</th><th>Nailer error (mm)</th><th>Least clearance (m)</th></tr></thead><tbody><tr v-for="(item,index) in segments" :key="item.key" :class="{active:index===segmentIndex}" @click="jump(index)"><th>{{index+1}}</th><th>{{item.label}}</th><td>{{number(item.duration,1)}}</td><td>{{number(item.graphError,3)}}</td><td>{{number(item.planSeconds,1)}}</td><td>{{number(item.steapSeconds,1)}}</td><td>{{millimetres(item.toolError[0])}}</td><td>{{millimetres(item.toolError[1])}}</td><td>{{number(Math.min(item.clearance.floor,item.clearance.wood,item.clearance.cross_arm),3)}}</td></tr></tbody></table></div>
      <details><summary>Planner, disturbance, and measurement settings</summary><pre>{{JSON.stringify({settings:record.settings,source:record.source},null,2)}}</pre></details></section>
      </AppPaper>
      <footer><p>Primary source: Shaji, Kim, D’Amato, Bruun, and Dellaert, “CAST: Collision-Aware Assembly with Construction Robots using Simultaneous Trajectory Estimation and Planning” (<a href="https://arxiv.org/abs/2609.24841">arXiv:2609.24841</a>).</p><p>Each maneuver is planned offline by GPMP2 from an RRT-Connect seed, with the CAST planner’s factors: Gaussian-process motion priors, joint and velocity limits, signed-distance factors against the floor, the tables and placed wood, cross-arm and bridge self-collision, the carried plank, and a learned neural-network dress cable for each arm. Each interval between support states is also checked at {{record.settings.planner.check_interpolated}} interpolated times ({{record.settings.planner.nail_check_interpolated}} for nailing).</p><p>STEAP execution is simulated. After each support state is commanded, the browser replays a measured configuration equal to the command plus a seeded AR(1) disturbance ({{record.settings.disturbance.gantry_sigma_m*1000}} mm on gantry axes, {{record.settings.disturbance.arm_sigma_deg}}° on arm joints). That measurement enters an ISAM2 Bayes tree as a position prior with the paper’s sigmas ({{record.settings.measurementSigma.gantryMeters*1000}} mm, {{record.settings.measurementSigma.armDegrees}}°), and the remaining states are re-estimated before the next command. No hardware was driven.</p><p>Arm 1 holds the gripper and arm 2 the nailer; nails are kinematic events and do not model wood or fastener mechanics. The setup presented in this visualization is a simplified version of the one in the original paper. Wood is 2×4 stock (3.5 in × 1.5 in). Sweeping maneuvers use {{record.settings.planner.state_count}} support states and nailing maneuvers {{record.settings.planner.nail_state_count}}.</p><p>Robot geometry: ABB IRB6620 arms and gantry CAD from the <a href="https://bruun-automation-research-lab.github.io/BAR-Documentation/index.html">Bruun Automation Research Lab</a>. STEAP method: Mukadam, Dong, Dellaert, and Boots, “STEAP: Simultaneous Trajectory Estimation and Planning” (<a href="https://arxiv.org/abs/1807.10425">arXiv:1807.10425</a>).</p><p><a href="./MODEL_NOTICE.txt">Model provenance and notice</a> · <a :href="presets.find(item=>item.id===presetId)?.file" download>Download assembly recording</a></p><details><summary>Source fingerprints</summary><pre>{{JSON.stringify(record.source,null,2)}}</pre></details></footer>
    </template>
  </main>
</template>
