<script setup>
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import { ReplayScene } from '../../shared/kinematics/scene.js';
import { historyFor, jointValue, referenceLabel, sliceTime } from '../../shared/kinematics/recording.js';
import TracePlot from '../../shared/kinematics/TracePlot.vue';
import BiasTable from '../../shared/kinematics/BiasTable.vue';
const props = defineProps({ method: String, stage: String, frame: Number, showReference: Boolean, data: Object, joint: Number });
const sceneElement = ref(null), selection = ref(0), replaying = ref(false), sceneError = ref('');
let scene, timer;
const title = computed(() => props.method === 'contact' ? 'Contact initialization' : 'Inverse kinematics');
const history = computed(() => historyFor(props.data.record, props.method, props.stage, props.frame));
const index = computed(() => Math.min(selection.value, history.value.length - 1));
const state = computed(() => history.value[index.value]);
const bias = computed(() => state.value.bias ?? props.data.record.runs[props.method].smoothing.at(-1).bias);
const dense = computed(() => state.value.frames === 1 ? props.data.record.runs[props.method].smoothing[0] : state.value);
const metrics = computed(() => state.value.metrics);
const errors = computed(() => history.value.map(s => props.stage === 'initializer' ? s.error : s.metrics.cost));
const jointAngles = computed(() => Array.from({ length: props.data.record.samples }, (_, k) => jointValue(props.data.record, props.data.floats, dense.value, k, props.joint)));
const measuredAngles = computed(() => Array.from({ length: props.data.record.samples }, (_, k) => jointValue(props.data.record, props.data.floats, props.data.record.reference, k, props.joint)));
const currentAngle = computed(() => jointValue(props.data.record, props.data.floats, state.value, props.frame, props.joint));
const number = (value, digits = 3) => value == null ? '—' : (Math.abs(value) < 1e-3 && value !== 0 ? value.toExponential(2) : value.toFixed(digits));
const frameMetrics = computed(() => dense.value.frameMetrics);
function updateScene() { if (scene) scene.update(state.value, props.frame, props.showReference, dense.value); }
function stop() { clearInterval(timer); timer = null; replaying.value = false; }
function select(value) { stop(); selection.value = Number(value); }
function replay() {
  if (replaying.value) { stop(); return; }
  selection.value = 0; replaying.value = true;
  timer = setInterval(() => { if (selection.value >= history.value.length - 1) stop(); else selection.value++; }, 650);
}
function final() { select(history.value.length - 1); }
watch(() => props.stage, () => { stop(); selection.value = history.value.length - 1; });
watch(() => props.frame, () => { if (props.stage === 'initializer' && props.method === 'ik') { stop(); selection.value = Math.min(selection.value, history.value.length - 1); } });
watch([state, () => props.frame, () => props.showReference], updateScene);
onMounted(() => {
  selection.value = history.value.length - 1;
  try {
    scene = new ReplayScene(sceneElement.value, props.data.model, props.data.record, props.data.floats, props.method === 'contact' ? '#087f71' : '#7955a5');
    updateScene();
  } catch (error) { sceneError.value = `3D view unavailable: ${error.message}`; }
});
onBeforeUnmount(() => { stop(); scene?.dispose(); });
</script>
<template>
  <article class="method-panel" :class="method" :data-method="method" :data-iteration="index">
    <header class="panel-heading"><div><span class="eyebrow">{{ method === 'contact' ? 'CONTACT' : 'IK' }}</span><h2>{{ title }}</h2></div><div><button class="quiet" @click="scene?.fitPaths()">Fit paths</button><button class="quiet" @click="scene?.reset()">Reset view</button></div></header>
    <div ref="sceneElement" class="scene"><p v-if="sceneError" role="alert" class="error-banner">{{ sceneError }}</p></div>
    <div class="scene-caption"><span>{{ state.label }}</span><span>{{ sliceTime(data.record,frame).toFixed(2) }} s · slice {{ frame }}</span></div>
    <section class="iteration-controls">
      <div class="row"><h3>{{ stage === 'smoothing' ? 'Whole-trajectory smoothing' : method === 'ik' ? `IK solve · frame ${frame}` : 'Sparse contact solve' }}</h3><span class="mono">Iteration {{ state.iteration }} / {{ history.at(-1).iteration }}</span></div>
      <input type="range" min="0" :max="history.length-1" :value="index" :aria-label="`${title} iteration`" @input="select($event.target.value)">
      <div class="row"><div class="button-group"><button @click="select(Math.max(0,index-1))" :disabled="index===0" aria-label="Previous iteration">←</button><button @click="replay">{{ replaying ? 'Pause iterations' : 'Replay iterations' }}</button><button @click="select(Math.min(history.length-1,index+1))" :disabled="index===history.length-1" aria-label="Next iteration">→</button></div><button class="quiet" @click="final">Last iteration</button></div>
    </section>
    <Teleport defer :to="`#${method}-results`">
    <article class="method-results" :class="method" :data-result-iteration="index">
    <header class="panel-heading"><h3>{{ title }} · numerical results</h3></header>
      <p class="scope" v-if="stage==='initializer'">{{ method==='ik' ? 'Each retained slice has its own LM history. The time selection determines which solve is displayed.' : `The contact graph jointly solves ${data.record.runs.contact.sparseSamples.length} retained slices. Interpolation is confined to visualization.` }}</p>
      <p class="scope" v-if="stage==='initializer' && method==='contact'">{{ data.record.runs.contact.initialization.stalled ? 'The initializer stalled after failing to accept a decrease.' : data.record.runs.contact.initialization.converged ? 'The initializer reached its stopping criterion.' : 'The initializer reached its iteration limit.' }}</p>
      <p class="scope" v-if="stage==='smoothing'">Iteration 0 is the feasible initial guess. CM-Opt is limited to {{ data.record.maxIterations }} iterations.</p>
    <div class="path-legend"><span class="estimated-path">Solid: estimated root path</span><span v-if="showReference" class="superodom-path">Dashed: {{referenceLabel}} path</span><span>XY ground projection</span></div>
    <BiasTable :bias="bias" :aiding="data.record.imuAiding" :caption="stage==='smoothing'?'Estimated IMU bias · selected iteration':'Estimated IMU bias · final smoothing iteration'" />
    <section class="plot-block">
      <div class="row"><h3>{{ stage==='smoothing' ? 'Smoothing objective' : 'Initializer objective' }}</h3><strong class="mono">{{ number(errors[index], 5) }}</strong></div>
      <TracePlot :values="errors" :selected="index" interactive label="Objective at every saved iteration" @select="select" />
      <div class="plot-foot"><span>Initial</span><span>½ weighted squared residuals · linear scale</span><span>Final</span></div>
    </section>
    <div v-if="metrics" class="metrics">
      <div><span>Equality L2</span><strong>{{ number(metrics.kinematic_equality_l2) }}</strong></div>
      <div><span>Contact height RMS</span><strong>{{ number(metrics.stance_height_rms_m*1000) }} <small>mm</small></strong></div>
      <div><span>Contact slip RMS / step</span><strong>{{ number(metrics.stance_step_slip_rms_m*1000) }} <small>mm</small></strong></div>
      <div><span>Joint measurement RMS</span><strong>{{ number(metrics.joint_measurement_rms_rad) }} <small>rad</small></strong></div>
      <div><span>Reference root position RMS</span><strong>{{ number(metrics.reference_root_translation_rms_m) }} <small>m</small></strong></div>
    </div>
    <div v-else class="metrics local-metrics"><div><span>Per-frame IK objective</span><strong>{{ number(state.error,5) }}</strong></div><div><span>Selected joint</span><strong>{{ number(currentAngle) }} <small>rad</small></strong></div></div>
    <div v-if="data.record.loopClosure?.enabled && state.closurePositionErrorWorld" class="metrics"><div><span>Final COM − start anchor · world [x, y, z] · m</span><strong>{{state.closurePositionErrorWorld.map(x=>number(x,3)).join(', ')}}</strong></div></div>
    <section class="plot-block">
      <div class="row"><h3>{{ data.record.jointNames[joint].replaceAll('_',' ') }}</h3><span class="mono">{{ number(currentAngle) }} rad</span></div>
      <TracePlot :values="jointAngles" :secondary="measuredAngles" :selected="frame" :timestamps="data.record.timestamps" label="Joint angle over walking time, with source reference" />
      <div class="plot-foot"><span>{{sliceTime(data.record,0).toFixed(2)}} s</span><span>Solid: {{state.frames===1 ? 'completed IK trajectory' : 'selected iteration'}} · dashed: source</span><span>{{sliceTime(data.record,data.record.samples-1).toFixed(2)}} s</span></div>
    </section>
    <section v-if="frameMetrics && state.frames!==1" class="plot-block">
      <div class="row"><h3>Contact height over time</h3><span class="mono">{{ number(frameMetrics.height[frame]*1000) }} mm</span></div>
      <TracePlot :values="frameMetrics.height.map(x=>x*1000)" :selected="frame" :timestamps="data.record.timestamps" label="Ground contact height RMS by frame in millimetres" />
      <p class="scope">The curve shows contact-height RMS at each frame; the summary above reports RMS over the trajectory. Frames without active contacts are displayed as zero.</p>
    </section>
    </article>
    </Teleport>
  </article>
</template>
