<script setup>
import { computed, markRaw, onBeforeUnmount, onMounted, ref } from 'vue';
import { loadRecording } from './recording.js';
import MethodPanel from './MethodPanel.vue';
const data = ref(null), error = ref(''), frame = ref(0), stage = ref('smoothing'), showReference = ref(false), joint = ref(0), playing = ref(false);
const controller = new AbortController();
let animation = 0, origin = 0, firstFrame = 0;
const phase = computed(() => data.value?.record.phases.find(p => p.start <= frame.value && p.end >= frame.value));
const motionCredit = computed(() => data.value?.record.motionSource?.licenseNotes[0]);
function pause() { cancelAnimationFrame(animation); playing.value = false; }
function scrub(value) { pause(); frame.value = Number(value); }
function tick(time) {
  if (!playing.value) return;
  const next = firstFrame + Math.floor((time-origin)/1000/data.value.record.dt);
  frame.value = Math.min(next, data.value.record.samples-1);
  if (frame.value === data.value.record.samples-1) pause(); else animation = requestAnimationFrame(tick);
}
function play() {
  if (playing.value) { pause(); return; }
  if (frame.value === data.value.record.samples-1) frame.value=0;
  firstFrame=frame.value; origin=performance.now(); playing.value=true; animation=requestAnimationFrame(tick);
}
function changeStage(value) { pause(); stage.value=value; }
onMounted(async () => { try { data.value=markRaw(await loadRecording(controller.signal)); joint.value=Math.max(0,data.value.record.jointNames.indexOf('left_knee_joint')); } catch(e) { if(!controller.signal.aborted) error.value=e.message; } });
onBeforeUnmount(() => { pause(); controller.abort(); });
</script>
<template>
  <main :data-stage="stage" :data-frame="frame">
    <header class="page-header"><div><span class="eyebrow">GTDYNAMICS / G1 WALKING</span><h1>Kinematics</h1><p>Full-trajectory kinematic optimization with contact and inverse-kinematics initialization.</p></div><span class="recorded-badge"><i></i> OFFLINE RECORDING</span></header>
    <div v-if="error" role="alert" class="error-banner"><h2>Recording could not be loaded</h2><p>{{error}}</p><p>Generate the saved data with <code>python apps/kinematics-inspector/scripts/export_recording.py</code> from the repository root.</p></div>
    <div v-else-if="!data" class="loading" role="status">Loading robot geometry and recorded iterations…</div>
    <template v-else>
      <section class="toolbar">
        <div class="stage-selector" role="group" aria-label="Inspection stage"><button :aria-pressed="stage==='smoothing'" @click="changeStage('smoothing')">CM-Opt smoothing</button><button :aria-pressed="stage==='initializer'" @click="changeStage('initializer')">Initialization</button></div>
        <div class="settings"><label><input type="checkbox" v-model="showReference"> Reference overlay</label><label>Joint <select v-model.number="joint" aria-label="Joint to inspect"><option v-for="(name,index) in data.record.jointNames" :key="name" :value="index">{{name}}</option></select></label></div>
      </section>
      <section class="timeline">
        <p v-if="data.record.motionSource" class="motion-title">{{data.record.motionSource.title}}</p>
        <div class="row"><div class="time-heading"><button class="play-button" @click="play">{{playing ? 'Pause walking' : 'Play walking'}}</button><strong>{{(frame*data.record.dt).toFixed(2)}} <small>/ {{((data.record.samples-1)*data.record.dt).toFixed(2)}} s</small></strong></div><span class="mono">Frame {{frame}} · phase {{data.record.phases.indexOf(phase)+1}} / {{data.record.phases.length}} · {{phase?.contacts.join(' + ') || 'flight'}}</span></div>
        <input type="range" :value="frame" min="0" :max="data.record.samples-1" aria-label="Walking time" @input="scrub($event.target.value)">
        <div class="phase-strip" aria-label="Contact phases"><button v-for="(p,index) in data.record.phases" :key="index" :class="{active:p===phase,double:p.contacts.length===2}" :style="{flex:p.end-p.start+1}" :title="`Phase ${index+1}: ${p.contacts.join(' + ') || 'flight'}, frames ${p.start}–${p.end}`" :aria-label="`Jump to phase ${index+1}`" @click="scrub(p.start)"></button></div>
        <div class="plot-foot"><span>Contact-aligned phases · click to jump</span><span>{{data.record.samples}} samples · {{data.record.graph.costFactors.toLocaleString()}} costs · {{data.record.graph.equalities.toLocaleString()}} equalities</span></div>
      </section>
      <div class="comparison"><MethodPanel v-for="method in ['contact','ik']" :key="method" :method="method" :stage="stage" :frame="frame" :show-reference="showReference" :data="data" :joint="joint" /></div>
      <footer><strong>About this recording</strong>
        <div v-if="motionCredit" class="motion-attribution">
          <p><a :href="data.record.motionSource.sourceUrl">{{data.record.motionSource.title}}</a> from <a :href="data.record.motionSource.datasetUrl">G1 Moves</a> by {{motionCredit.creator}}. Performer: {{motionCredit.performer}}. Licensed under <a :href="motionCredit.licenseUrl">CC BY 4.0</a>.</p>
          <p>Retargeted human motion captured with MOVIN TRACIN. We converted quaternion order, computed robot poses, inferred contact phases from sole height and speed, and calculated initialization and smoothing iterations. Contacts are kinematic estimates; the source contains no contact-force measurements.</p>
          <a href="./G1_MOVES_ATTRIBUTION.txt">Motion attribution and modifications</a>
        </div>
        <p>{{data.record.notes}}</p><p>Reference overlays are held-out poses computed with MuJoCo. Robot transforms and residuals were calculated offline from the example’s actual values; the viewer performs no optimization or forward kinematics.</p><details><summary>Source and input fingerprints</summary><pre>{{JSON.stringify(data.record.provenance,null,2)}}</pre></details><a href="./recording.json" download>Download recording metadata</a> · <a href="./MODEL_LICENSE.txt">Robot model license</a></footer>
    </template>
  </main>
</template>
