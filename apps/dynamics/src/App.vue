<script setup>
import {computed,markRaw,nextTick,onBeforeUnmount,onMounted,ref,watch} from 'vue';
import {frameAtTime,jointGroups,loadRecording} from './recording.js';
import {DynamicsScene} from './scene.js';
import SignalPlot from './SignalPlot.vue';

const data=ref(null),error=ref(''),frame=ref(0),playing=ref(false);
const sceneElement=ref(null),showForces=ref(true),forceScale=ref(.002),showTorques=ref(false),view=ref('robot');
const controller=new AbortController();
let scene,animation=0,origin=0,start=0;
const record=computed(()=>data.value?.record);
const sampleTime=computed(()=>record.value?.times[frame.value]||0);
const factors=computed(()=>record.value?.diagnostics?.factors||[]);
const solver=computed(()=>record.value?.diagnostics?.solver||{});
const torques=computed(()=>record.value?jointGroups(record.value.jointNames).map(group=>({...group,series:group.indices.map(index=>({name:record.value.jointNames[index].replace(/_joint$/,''),values:record.value.jointTorques.map(sample=>sample[index])}))})):[]);
const number=(value,digits=2)=>typeof value==='number'?Math.abs(value)>0&&Math.abs(value)<.001?value.toExponential(2):value.toLocaleString('en-US',{maximumFractionDigits:digits}):'—';
const forcePlots=computed(()=>{
  if(!record.value)return [];
  return ['Magnitude','World x','World y','World z'].map((title,index)=>({title,series:[0,1].map(foot=>({
    name:foot===0?'Left foot':'Right foot',color:foot===0?'#d44747':'#219469',
    values:record.value.worldForces.map(sample=>index===0?Math.hypot(...sample[foot]):sample[foot][index-1]),
  }))}));
});
function pause(){cancelAnimationFrame(animation);playing.value=false;}
function scrub(value){pause();frame.value=Number(value);}
function tick(now){
  if(!playing.value)return;
  const time=start+(now-origin)/1000;
  frame.value=frameAtTime(record.value.times,time);
  if(time>=record.value.times.at(-1))pause();else animation=requestAnimationFrame(tick);
}
function play(){
  if(playing.value){pause();return;}
  if(frame.value===record.value.times.length-1)frame.value=0;
  start=sampleTime.value;origin=performance.now();playing.value=true;animation=requestAnimationFrame(tick);
}
function camera(mode){view.value=mode;if(mode==='robot')scene?.follow();else scene?.fit();}
function update(){scene?.update(frame.value,forceScale.value,showForces.value);}
watch([frame,forceScale,showForces],update);
onMounted(async()=>{
  try{
    data.value=markRaw(await loadRecording(controller.signal));
    if(controller.signal.aborted)return;
    await nextTick();
    scene=new DynamicsScene(sceneElement.value,data.value.model,data.value.record);
    update();
  }catch(exception){if(!controller.signal.aborted)error.value=exception.message;}
});
onBeforeUnmount(()=>{controller.abort();pause();scene?.dispose();});
</script>
<template>
  <main class="dynamics-app" :data-frame="frame">
    <header class="page-header"><div><span class="eyebrow">GTDYNAMICS / SSIO G1</span><h1>Dynamics</h1><p>Dynamics estimates accelerations, sole wrenches, and joint torques from GP-fitted G1 motion over 78–98 s.</p></div><span class="recorded-badge">OFFLINE CMC-OPT SOLVE</span></header>
    <div v-if="error" class="error-banner" role="alert">{{error}}</div>
    <div v-else-if="!data" class="loading" role="status">Loading dynamics recording…</div>
    <template v-else>
      <p v-if="record.diagnostics.status" class="solve-status">{{record.diagnostics.status}}</p>
      <section class="dynamics-viewer">
        <header class="viewer-tools"><div class="button-group"><button :aria-pressed="view==='trajectory'" @click="camera('trajectory')">Fit trajectory</button><button :aria-pressed="view==='robot'" @click="camera('robot')">Follow robot</button></div><span class="force-legend"><i class="left" />Left foot <i class="right" />Right foot</span></header>
        <div ref="sceneElement" class="dynamics-scene" />
        <section class="timeline">
          <div class="row"><div class="time-heading"><button class="play-button" @click="play">{{playing?'Pause dynamics':'Play dynamics'}}</button><strong>{{sampleTime.toFixed(2)}} <small>/ {{record.times.at(-1).toFixed(2)}} s</small></strong></div><span class="mono">Slice {{frame+1}} / {{record.times.length}} · {{record.contact[frame].map((contact,foot)=>contact?(foot===0?'left stance':'right stance'):(foot===0?'left swing':'right swing')).join(' · ')}}</span></div>
          <input type="range" :value="frame" min="0" :max="record.times.length-1" aria-label="Dynamics time" @input="scrub($event.target.value)">
          <div class="display-options"><label><input type="checkbox" v-model="showForces"> Ground reaction forces</label><label>Force scale <select v-model.number="forceScale" aria-label="Force scale"><option :value=".001">1 m / 1000 N</option><option :value=".002">1 m / 500 N</option><option :value=".004">1 m / 250 N</option></select></label><label><input type="checkbox" v-model="showTorques"> All joint torques</label></div>
        </section>
      </section>
      <p class="sampling-note">The solve uses {{record.times.length}} original double-support knots with poses and velocities fixed by the full-trajectory Gaussian-process fit. Acceleration priors use local velocity differences with a {{record.settings.accelerationDifferenceStepSeconds*1000}} ms step on either side of each knot. Intervening single-support intervals remain unsampled.</p>
      <section class="force-readout" aria-label="Current contact wrenches"><article v-for="(foot,index) in ['Left','Right']" :key="foot"><h3>{{foot}} foot · {{record.contact[frame][index]?'stance':'swing'}}</h3><strong>{{number(Math.hypot(...record.worldForces[frame][index]))}} <small>N</small></strong><span>World force XYZ: {{record.worldForces[frame][index].map(v=>number(v)).join(', ')}} N</span><span>Sole moment XYZ: {{record.wrenches[frame][index].slice(0,3).map(v=>number(v)).join(', ')}} Nm</span></article></section>
      <section class="signals"><div class="row"><h2>Ground reaction forces</h2><span class="scope">World frame · N · drag or scroll to zoom; click a curve to select a slice.</span></div><p class="scope">Markers are solved slices; lines connect samples for display.</p><div class="force-plots"><SignalPlot v-for="plot in forcePlots" :key="plot.title" :title="plot.title" unit="N" :times="record.times" :series="plot.series" :frame="frame" @select="scrub" /></div></section>
      <section v-if="showTorques" class="signals torque-section"><h2>Joint torques · all {{record.jointNames.length}} actuators</h2><p class="scope">Actuator torque in Nm, positive along the model's named joint axis. Click legend entries to isolate joints.</p><div class="torque-plots"><SignalPlot v-for="group in torques" :key="group.title" :title="group.title" unit="Nm" :times="record.times" :series="group.series" :frame="frame" @select="scrub" /></div><details v-if="record.jointEffortLimits"><summary>{{record.settings.torqueLimitsEnforced?'Actuator effort bounds':'Actuator effort limits · diagnostic only'}}</summary><table class="effort-table"><tbody><tr v-for="(name,index) in record.jointNames" :key="name"><th>{{name}}</th><td>± {{number(record.jointEffortLimits[index])}} Nm</td></tr></tbody></table></details></section>
      <section class="solve-section"><h2>Dynamics solve</h2><p class="solver-summary"><strong>{{typeof record.diagnostics.solver==='string'?record.diagnostics.solver:(solver.name||'CMC-Opt')}}</strong><span>{{number(solver.elapsedSeconds??record.diagnostics.elapsedSeconds)}} s · {{solver.iterations??record.diagnostics.iterations??'—'}} iterations</span><span>{{record.times.length}} fixed kinematic slices · {{record.jointNames.length}} actuators · one wrench per sole</span></p><p v-if="record.diagnostics.accelerationMethod" class="scope">Acceleration: {{record.diagnostics.accelerationMethod}}</p><p v-if="record.diagnostics.maxForceResidual!==undefined" class="scope">Maximum Newton–Euler residual: {{number(record.diagnostics.maxForceResidual)}} N / {{number(record.diagnostics.maxMomentResidual)}} Nm.</p><p v-if="record.diagnostics.accelerationCorrections" class="scope">Acceleration corrections relative to the GP-derived priors have per-component RMS {{number(record.diagnostics.accelerationCorrections.rootAngularRmsDegreesPerSecondSquared)}} °/s² at the root, {{number(record.diagnostics.accelerationCorrections.rootLinearRmsMetersPerSecondSquared)}} m/s² for root translation, and {{number(record.diagnostics.accelerationCorrections.jointRmsDegreesPerSecondSquared)}} °/s² for the joints.</p><div class="table-scroll"><table class="factor-table"><thead><tr><th>Factor / constraint</th><th>Count</th><th>Final error</th><th>Violation norm</th></tr></thead><tbody><tr v-for="factor in factors" :key="factor.type"><th>{{factor.type}}</th><td>{{number(factor.count,0)}}</td><td>{{number(factor.error,4)}}</td><td>{{number(factor.maxViolation,5)}}</td></tr></tbody></table></div><details><summary>Solver settings, feasibility, and timing scopes</summary><pre>{{JSON.stringify({settings:record.settings,diagnostics:record.diagnostics},null,2)}}</pre></details></section>
      <footer><p>The best recorded smoothing variant ({{record.source?.kinematicsVariant||record.source?.variant}}) supplies the fixed knot configurations. Gaussian-process interpolation fits velocities using dense encoders and the saved base-velocity observations. Native dynamics optimization estimates accelerations, sole-frame contact wrenches, and joint torques at the original irregular knot times. The browser replays the saved result.</p><p>Force arrows show ground-on-robot force in world coordinates, starting at the actual sole contact frame. Wrenches are ordered [moment x, y, z, force x, y, z] in the sole frame, in Nm and N. Sole axes are +x forward, +y left, +z up.</p><p>Contact phases use privileged SuperOdom LiDAR-inertial reference motion. Inferred forces and torques are model-based estimates, not measured force-plate or motor-torque ground truth.</p><p v-if="record.diagnostics.stanceFootSpeedRmsMetersPerSecond!==undefined">Labeled stance feet have {{number(record.diagnostics.stanceFootSpeedRmsMetersPerSecond)}} m/s RMS speed in the fixed GP velocity fit. This residual motion limits the rigid-contact interpretation of the estimated wrenches.</p><p v-for="credit in record.attribution||[]" :key="credit.url"><a :href="credit.url">{{credit.label}}</a><span v-if="credit.text"> · {{credit.text}}</span></p><p><a href="./SSIO_ATTRIBUTION.txt">SSIO attribution and modifications</a> · <a href="./MODEL_LICENSE.txt">G1 model license</a> · <a href="./recording.json.gz" download>Download dynamics recording</a></p><details><summary>Source fingerprints</summary><pre>{{JSON.stringify({source:record.source,provenance:record.provenance},null,2)}}</pre></details></footer>
    </template>
  </main>
</template>
