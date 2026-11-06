<script setup>
import {computed,markRaw,nextTick,onBeforeUnmount,onMounted,ref,watch} from 'vue';
import {loadRecording,frameAtTime,jointGroups} from './recording.js';
import {RecordedDynamicsScene} from '../../shared/RecordedDynamicsScene.js';
import SignalPlot from '../../shared/dynamics/SignalPlot.vue';
import AppAbstract from '../../shared/paper/AppAbstract.vue';
import AppPaper from '../../shared/paper/AppPaper.vue';
import KinodynamicsMath from '../../shared/paper/KinodynamicsMath.vue';
import SolverSettings from './SolverSettings.vue';

const data=ref(null),error=ref(''),frame=ref(0),playing=ref(false),stage=ref('final');
const sceneElement=ref(null),showForces=ref(true),showTorques=ref(false),showJointVelocities=ref(false);
const forceScale=ref(.002),errorScale=ref('total');
const abort=new AbortController();
let scene,animation=0,origin=0,start=0;
const record=computed(()=>data.value?.record);
const diagnostics=computed(()=>record.value?.diagnostics||{});
const sampling=computed(()=>record.value?.source.initializationSampling);
const comparison=computed(()=>record.value?.initializationComparison);
const initial=computed(()=>stage.value==='initial');
const initializationName=strategy=>({'contact-gp-dynamics':'Contact → GP → linear seed (default)',contact:'Contact poses',fk:'Fixed-pelvis FK'}[strategy]||strategy);
const visible=computed(()=>{
  if(!record.value)return null;
  if(!initial.value)return record.value;
  return {...record.value,...record.value.initial,
    worldForces:record.value.initial.worldForces??record.value.times.map(()=>[[0,0,0],[0,0,0]])};
});
const plotFrame=computed(()=>playing.value?5*Math.floor(frame.value/5):frame.value);
const time=computed(()=>record.value?.times[frame.value]||0);
const number=(value,digits=2)=>typeof value==='number'?(Math.abs(value)>0&&Math.abs(value)<.001?value.toExponential(2):value.toLocaleString('en-US',{maximumFractionDigits:digits})):'—';
const plots=computed(()=>record.value?['Magnitude','World x','World y','World z'].map((title,index)=>({title,series:[0,1].map(foot=>({name:foot?'Right foot':'Left foot',color:foot?'#219469':'#d44747',values:visible.value.worldForces.map(sample=>index===0?Math.hypot(...sample[foot]):sample[foot][index-1])}))})):[]);
const velocityPlots=computed(()=>record.value?['x','y','z'].map((axis,index)=>({title:`Pelvis world velocity ${axis}`,series:[{name:initial.value?'Initial':'Estimated',color:'#327e68',values:visible.value.worldVelocities.map(sample=>sample[index])}]})):[]);
function jointPlots(field){return record.value?jointGroups(record.value.jointNames).map(group=>({...group,series:group.indices.map(index=>({name:record.value.jointNames[index].replace(/_joint$/,''),values:visible.value[field].map(sample=>sample[index])}))})):[];}
const torques=computed(()=>jointPlots('jointTorques'));
const jointVelocities=computed(()=>jointPlots('jointVelocities'));
const errors=computed(()=>{
  const rows=diagnostics.value.factors||[];
  return rows.map(row=>({...row,value:row.error/(errorScale.value==='per'?row.count:1)}));
});
const maxError=computed(()=>Math.max(1,...errors.value.map(row=>row.value)));
const convergence=computed(()=>{
  const d=diagnostics.value;
  if(!d.errorHistory)return [];
  return [{name:'Total',color:'#163f32',values:d.errorHistory.map(row=>row.reduce((a,b)=>a+b,0))},...d.errorNames.map((name,index)=>({name,values:d.errorHistory.map(row=>row[index])}))];
});
function pause(){cancelAnimationFrame(animation);playing.value=false;}
function scrub(value){pause();frame.value=Number(value);}
function tick(now){if(!playing.value)return;const t=start+(now-origin)/1000;frame.value=frameAtTime(record.value.times,t);if(t>=record.value.times.at(-1))pause();else animation=requestAnimationFrame(tick);}
function play(){if(playing.value){pause();return;}if(frame.value===record.value.times.length-1)frame.value=0;start=time.value;origin=performance.now();playing.value=true;animation=requestAnimationFrame(tick);}
function update(){scene?.update(frame.value,forceScale.value,showForces.value);}
watch([frame,forceScale,showForces],update);
watch(stage,()=>{pause();scene?.setRecord(visible.value);update();});
onMounted(async()=>{try{data.value=markRaw(await loadRecording(abort.signal));if(abort.signal.aborted)return;await nextTick();scene=new RecordedDynamicsScene(sceneElement.value,data.value.model,visible.value);update();}catch(e){if(!abort.signal.aborted)error.value=e.message;}});
onBeforeUnmount(()=>{abort.abort();pause();scene?.dispose();});
</script>
<template>
  <main id="demo" class="dynamics-app kinodynamics-app" :data-frame="frame" :data-stage="stage">
    <header class="page-header"><div><span class="eyebrow">GTDYNAMICS / SSIO G1</span><h1>Kinodynamics</h1><AppAbstract app="kinodynamics" /></div><span class="recorded-badge">OFFLINE FULL-STATE SOLVE</span></header>
    <p v-if="error" class="error-banner" role="alert">{{error}}</p>
    <p v-else-if="!data" role="status">Loading kinodynamics recording…</p>
    <template v-else>
      <section class="dynamics-viewer">
        <header class="viewer-tools"><div class="button-group"><button @click="scene?.fit()">Fit trajectory</button><button @click="scene?.follow()">Follow robot</button></div><span class="force-legend"><i class="left" />Left foot <i class="right" />Right foot</span></header>
        <div ref="sceneElement" class="dynamics-scene kinodynamics-scene" />
        <section class="timeline"><div class="row"><div class="time-heading"><button class="play-button" @click="play">{{playing?'Pause':'Play'}}</button><strong>{{time.toFixed(2)}} <small>/ {{record.times.at(-1).toFixed(2)}} s</small></strong></div><span>Slice {{frame+1}} / {{record.times.length}} · {{record.contact[frame].map((on,foot)=>(foot?'Right':'Left')+(on?' stance':' swing')).join(' · ')}}</span></div>
          <input type="range" min="0" :max="record.times.length-1" :value="frame" aria-label="Kinodynamics time" @input="scrub($event.target.value)">
          <div class="display-options"><label>Estimate <select v-model="stage" aria-label="Estimate"><option value="final">Full solve</option><option value="initial">Raw initialization</option></select></label><label><input type="checkbox" v-model="showForces"> Ground reaction forces</label><label>Force scale <select v-model.number="forceScale"><option :value=".001">1 m / 1000 N</option><option :value=".002">1 m / 500 N</option><option :value=".004">1 m / 250 N</option></select></label><label><input type="checkbox" v-model="showTorques"> All joint torques</label><a v-if="showTorques" href="#joint-torque-results">View torque plots ↓</a><label><input type="checkbox" v-model="showJointVelocities"> All joint velocities</label><a v-if="showJointVelocities" href="#joint-velocity-results">View joint velocity plots ↓</a></div>
        </section>
      </section>
      <AppPaper app="kinodynamics" :frame="frame" :samples="record.times.length" @select="scrub">
        <template #mathematics><KinodynamicsMath /></template>
      <section class="timing-grid" aria-label="Solve timings">
        <article><strong>{{number(diagnostics.totalInitializationAndSolveSeconds??diagnostics.totalFromContactInitializationSeconds)}} s</strong><span>Initialization + coupled solve</span></article>
        <article><strong>{{number(diagnostics.configurationInitializationSeconds??diagnostics.contactInitializationSeconds,3)}} s</strong><span>Initialization pipeline</span></article>
        <article><strong>{{number(diagnostics.optimizationSeconds)}} s</strong><span>Coupled optimization · {{diagnostics.iterations}} iterations</span></article>
        <article><strong>{{record.times.length.toLocaleString()}}</strong><span>Optimized support slices · original timestamps</span></article>
      </section>
      <p class="solve-status"><template v-if="initial">Raw initialization: {{diagnostics.initialization}}. Equality violation norm: {{number(diagnostics.initialEqualityL2,3)}}.</template><template v-else>{{diagnostics.status}}. {{diagnostics.iterations<(diagnostics.maxIterations??5)?'The optimizer reached its stopping condition.':`The optimizer reached the ${diagnostics.maxIterations??5}-iteration limit.`}}</template></p>
      <p v-if="sampling" class="scope">Initialization and the coupled solve use the same {{sampling.selectedSlices}} support slices out of {{sampling.sourceSlices.toLocaleString()}} source samples: {{sampling.singleSupportSlices}} in single support and {{sampling.doubleSupportSlices}} in double support. Short phases contribute a central slice; long phases use buffered slices at most {{sampling.maxSliceIntervalMs}} ms apart within the phase, including points during swing. Each stance landmark continues while its foot stays down. <template v-if="diagnostics.initializationStrategy==='contact-gp-dynamics'">Contact poses and retained encoders define the GP fit, then one affine linear dynamics seed supplies accelerations, wrenches and torques.</template> IMU preintegration and collocation use the actual time between retained slices. The replay shows these optimized knots.</p>
      <section class="signals"><h2>Ground reaction forces</h2><div class="force-plots"><SignalPlot v-for="plot in plots" :key="plot.title" :title="plot.title" unit="N" :times="record.times" :series="plot.series" :frame="plotFrame" @select="scrub" /></div></section>
      <section class="signals"><h2>Pelvis velocity</h2><div class="force-plots"><SignalPlot v-for="plot in velocityPlots" :key="plot.title" :title="plot.title" unit="m/s" :times="record.times" :series="plot.series" :frame="plotFrame" @select="scrub" /></div></section>
      <section v-if="showTorques" id="joint-torque-results" class="signals torque-section"><h2>Joint torques · all {{record.jointNames.length}} actuators</h2><div class="torque-plots"><SignalPlot v-for="plot in torques" :key="plot.title" :title="plot.title" unit="Nm" :times="record.times" :series="plot.series" :frame="plotFrame" @select="scrub" /></div></section>
      <section v-if="showJointVelocities" id="joint-velocity-results" class="signals"><h2>Joint velocities</h2><div class="torque-plots"><SignalPlot v-for="plot in jointVelocities" :key="plot.title" :title="plot.title" unit="rad/s" :times="record.times" :series="plot.series" :frame="plotFrame" @select="scrub" /></div></section>
      <section class="solve-section"><h3>Optimization timing and feasibility</h3><p>{{diagnostics.initialization}}. Feasibility restoration is included in the coupled solve timing.</p>
        <p class="scope">Stance sole-origin speed: {{number(diagnostics.stanceFootSpeedRmsMetersPerSecond,3)}} m/s RMS. Maximum actuator effort: {{number(100*diagnostics.maxJointEffortLimitRatio,1)}}% of the model limit.</p><div class="table-scroll"><table class="factor-table"><tbody><tr v-if="diagnostics.gpInitializationSeconds"><th>Contact-pose initialization</th><td>{{number(diagnostics.contactInitializationSeconds,3)}} s</td></tr><tr v-if="diagnostics.gpInitializationSeconds"><th>GP assembly, fit and evaluation</th><td>{{number(diagnostics.gpInitializationSeconds,3)}} s</td></tr><tr v-if="diagnostics.dynamicsInitializationSeconds"><th>Linear dynamics seed</th><td>{{number(diagnostics.dynamicsInitializationSeconds,3)}} s</td></tr><tr v-if="diagnostics.dynamicsInitializationAssemblySeconds!==undefined"><th>Dynamics initializer: graph assembly</th><td>{{number(diagnostics.dynamicsInitializationAssemblySeconds,3)}} s</td></tr><tr v-if="diagnostics.dynamicsInitializationLinearizationSeconds!==undefined"><th>Dynamics initializer: equality-system assembly</th><td>{{number(diagnostics.dynamicsInitializationLinearizationSeconds,3)}} s</td></tr><tr v-if="diagnostics.dynamicsInitializationLinearSolveSeconds!==undefined"><th>Dynamics initializer: constrained Gaussian QR</th><td>{{number(diagnostics.dynamicsInitializationLinearSolveSeconds,3)}} s</td></tr><tr v-if="diagnostics.dynamicsInitializationValueAssemblySeconds!==undefined"><th>Dynamics initializer: variable reconstruction</th><td>{{number(diagnostics.dynamicsInitializationValueAssemblySeconds,3)}} s</td></tr><tr v-if="diagnostics.dynamicsInitializationValidationSeconds!==undefined"><th>Dynamics initializer: mechanics and wrench checks</th><td>{{number(diagnostics.dynamicsInitializationValidationSeconds,3)}} s</td></tr><tr><th>Measurement construction and IMU preintegration</th><td>{{number(diagnostics.measurementPreparationSeconds,3)}} s</td></tr><tr><th>Graph assembly</th><td>{{number(diagnostics.assemblySeconds,3)}} s</td></tr><tr><th>Initial feasibility and manifold setup</th><td>{{number(diagnostics.initializationSeconds,3)}} s</td></tr><tr><th>Coupled optimization</th><td>{{number(diagnostics.optimizationSeconds,3)}} s</td></tr><tr><th>Raw initial equality violation</th><td>{{number(diagnostics.initialEqualityL2,5)}}</td></tr><tr><th>Final equality / inequality violation</th><td>{{number(diagnostics.equalityL2,5)}} / {{number(diagnostics.inequalityL2,5)}}</td></tr><tr><th>Exact equality rows / wrench-cone inequality rows</th><td>{{number(diagnostics.equalityDimension,0)}} / {{number(diagnostics.inequalityDimension,0)}}</td></tr><tr><th>Ambient / reduced scalar coordinates</th><td>{{number(diagnostics.variableDimension,0)}} / {{number(diagnostics.reducedDimension,0)}}</td></tr></tbody></table></div>
        <p class="scope">Timings are wall-clock measurements on the local machine. Input/model loading, display serialization, and browser loading are excluded. Measurement construction and IMU preintegration are included in the total. Configuration and velocity collocation use trapezoidal factors between every adjacent slice. Articulated q/v/a kinematics, Newton–Euler balance, wrench transmission, and sole-wrench cones are exact constraints.</p>
        <section v-if="comparison" class="initialization-comparison" aria-label="Initialization comparison">
          <h3>Initialization strategies</h3>
          <p>Each strategy ran once on the same {{comparison.samples.toLocaleString()}} slices, with {{comparison.maxIterations}} coupled iterations and {{comparison.retractionThreads}} retraction workers. These measurements compare bounded runs; they do not establish convergence.</p>
          <div class="table-scroll"><table class="factor-table"><thead><tr><th>Strategy</th><th>Initialization (s)</th><th>Restoration (s)</th><th>Optimization (s)</th><th>Total (s)</th><th>Final objective</th><th>Feasible</th></tr></thead><tbody><tr v-for="run in comparison.runs" :key="run.initializationStrategy"><th>{{initializationName(run.initializationStrategy)}}</th><td>{{number(run.configurationInitializationSeconds,3)}}</td><td>{{number(run.initializationSeconds,3)}}</td><td>{{number(run.optimizationSeconds,2)}}</td><td>{{number(run.totalInitializationAndSolveSeconds,2)}}</td><td>{{number(run.objectiveFinal,2)}}</td><td>{{run.feasible?'Yes':'No'}}</td></tr></tbody></table></div>
          <p class="scope">{{comparison.timingScope}}. The default contact → GP → linear dynamics seed supplies the displayed replay; other runs contribute comparison statistics.</p>
        </section>
        <SignalPlot title="Factor error over optimization wall time" x-title="Optimization wall time (s)" unit="½ ‖whitened residual‖²" :times="diagnostics.wallSeconds" :series="convergence" :frame="diagnostics.wallSeconds.length-1" />
        <div class="row"><h3>Final error budget</h3><label>Error scale <select v-model="errorScale"><option value="total">Total</option><option value="per">Per factor</option></select></label></div>
        <div class="error-bars"><div class="error-row" v-for="row in errors" :key="row.type"><span>{{row.type}}</span><progress :value="row.value" :max="maxError" /><strong>{{number(row.value,3)}}</strong></div></div>
        <SolverSettings :record="record" />
        <details><summary>Factor counts, source fingerprints and stopping data</summary><pre>{{JSON.stringify({factors:diagnostics.factors,settings:record.settings,diagnostics},null,2)}}</pre></details>
      </section>
      </AppPaper>
      <footer><p>Contact phases use the saved binary-foot max-product labels from the Contact Labels app. These privileged labels use SuperOdom LiDAR-inertial reference motion and measured-joint velocities. Later reference poses provide evaluation data. The estimator uses encoders, contact-pose observations, and Gal3 IMU factors with one shared bias.</p><p>Body twists and their derivatives are angular-first in COM axes. Force arrows are ground-on-robot forces in world coordinates. Sole wrenches use [moment; force] in Nm and N at the canonical sole origin, with +x forward, +y left, +z up. The model includes rotor armature and rectangular support constraints with μ = 0.8; actuator torque limits are diagnostic rather than enforced. Contact rotation noise is 90°, translation noise is 1 cm, and stance-origin velocity noise is 0.1 m/s.</p><p>The browser replays stored solutions. The student-facing computation is in <code>examples/g1_mjlab/kinodynamic_walk.py</code>; run its main with <code>--output</code> to save the numerical arrays and timing report.</p><p v-for="credit in record.attribution" :key="credit.url"><a :href="credit.url">{{credit.label}}</a> · {{credit.text}}</p><p><a href="./SSIO_ATTRIBUTION.txt">SSIO attribution and modifications</a> · <a href="./MODEL_LICENSE.txt">Model license</a> · <a href="./recording.json.gz" download>Download replay</a></p></footer>
    </template>
  </main>
</template>
