<script setup>
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import { ReplayScene } from './scene.js';
import { historyFor, jointValue } from './recording.js';
import TracePlot from './TracePlot.vue';
const props = defineProps({ method: String, stage: String, frame: Number, showReference: Boolean, data: Object, joint: Number });
const sceneElement = ref(null), selection = ref(0), replaying = ref(false), sceneError = ref('');
let scene, timer;
const title = computed(() => props.method === 'contact' ? 'Contact initialization' : 'Inverse kinematics');
const history = computed(() => historyFor(props.data.record, props.method, props.stage, props.frame));
const index = computed(() => Math.min(selection.value, history.value.length - 1));
const state = computed(() => history.value[index.value]);
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
    <header class="panel-heading"><div><span class="eyebrow">{{ method === 'contact' ? 'CONTACT' : 'IK' }}</span><h2>{{ title }}</h2></div><button class="quiet" @click="scene?.reset()">Reset view</button></header>
    <div ref="sceneElement" class="scene"><p v-if="sceneError" role="alert" class="error-banner">{{ sceneError }}</p></div>
    <div class="scene-caption"><span>{{ state.label }}</span><span>{{ (frame * data.record.dt).toFixed(2) }} s · frame {{ frame }}</span></div>
    <section class="iteration-controls">
      <div class="row"><h3>{{ stage === 'smoothing' ? 'Whole-trajectory smoothing' : method === 'ik' ? `IK solve · frame ${frame}` : 'Sparse contact solve' }}</h3><span class="mono">Iteration {{ state.iteration }} / {{ history.at(-1).iteration }}</span></div>
      <input type="range" min="0" :max="history.length-1" :value="index" :aria-label="`${title} iteration`" @input="select($event.target.value)">
      <div class="row"><div class="button-group"><button @click="select(Math.max(0,index-1))" :disabled="index===0" aria-label="Previous iteration">←</button><button @click="replay">{{ replaying ? 'Pause iterations' : 'Replay iterations' }}</button><button @click="select(Math.min(history.length-1,index+1))" :disabled="index===history.length-1" aria-label="Next iteration">→</button></div><button class="quiet" @click="final">Last iteration</button></div>
      <p class="scope" v-if="stage==='initializer'">{{ method==='ik' ? 'Each walking frame has its own LM history. Scrub time to inspect another solve.' : `${data.record.runs.contact.sparseSamples.length} supported samples solved jointly. Intermediate poses are interpolated offline.` }}</p>
      <p class="scope" v-if="stage==='initializer' && method==='contact'">{{ data.record.runs.contact.initialization.converged ? 'Converged.' : 'Stopped at the iteration limit; convergence criterion not met.' }}</p>
      <p class="scope" v-else>Iteration 0: feasible initial guess. CM-Opt limit: {{ data.record.maxIterations }} iterations.</p>
    </section>
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
    </div>
    <div v-else class="metrics local-metrics"><div><span>Per-frame IK objective</span><strong>{{ number(state.error,5) }}</strong></div><div><span>Selected joint</span><strong>{{ number(currentAngle) }} <small>rad</small></strong></div></div>
    <section class="plot-block">
      <div class="row"><h3>{{ data.record.jointNames[joint].replaceAll('_',' ') }}</h3><span class="mono">{{ number(currentAngle) }} rad</span></div>
      <TracePlot :values="jointAngles" :secondary="measuredAngles" :selected="frame" label="Joint angle over walking time, with source reference" />
      <div class="plot-foot"><span>0 s</span><span>Solid: {{state.frames===1 ? 'completed IK trajectory' : 'selected iteration'}} · dashed: source</span><span>{{ ((data.record.samples-1)*data.record.dt).toFixed(2) }} s</span></div>
    </section>
    <section v-if="frameMetrics && state.frames!==1" class="plot-block">
      <div class="row"><h3>Contact height over time</h3><span class="mono">{{ number(frameMetrics.height[frame]*1000) }} mm</span></div>
      <TracePlot :values="frameMetrics.height.map(x=>x*1000)" :selected="frame" label="Ground contact height RMS by frame in millimetres" />
      <p class="scope">Global RMS above; this curve shows each frame. No-contact frames show zero.</p>
    </section>
  </article>
</template>
