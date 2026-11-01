<script setup>
import { computed, markRaw, onBeforeUnmount, onMounted, ref } from 'vue';
import { clearRecordingCache, loadRecording, referenceLabel } from './recording.js';
import { disposeRobotTemplates } from './scene.js';
import MethodPanel from './MethodPanel.vue';
const data = ref(null), error = ref(''), frame = ref(0), stage = ref('smoothing'), showReference = ref(true), joint = ref(0), playing = ref(false);
const selection = ref('full');
const useImu = ref(true);
const useLoopClosure = ref(true);
const recordings = [{id:'turn', label:'Turn · 204–214 s · 50 Hz', directory:''}, {id:'full', label:'Entire trajectory · 5 Hz', directory:'full/'}];
const directory = computed(() => selection.value === 'full'
  ? `full${useImu.value ? '-imu' : ''}${useLoopClosure.value ? '-loop' : ''}/`
  : useImu.value ? 'imu/' : '');
let controller;
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
function changeStage(value) {
  pause(); stage.value=value;
  if (!data.value || !data.value.record.availableStages?.includes(value)) changeRecording(selection.value);
}
async function changeRecording(value, aided = useImu.value, closed = useLoopClosure.value) {
  pause(); controller?.abort();
  const request = new AbortController(); controller = request;
  const savedFrame = value === selection.value ? frame.value : 0;
  selection.value = value; useImu.value = aided; useLoopClosure.value = value === 'full' && closed;
  data.value = null; error.value = ''; frame.value = savedFrame;
  try {
    const loaded = await loadRecording(request.signal, directory.value, stage.value);
    if (request.signal.aborted) return;
    data.value = markRaw(loaded);
    joint.value = Math.max(0, loaded.record.jointNames.indexOf('left_knee_joint'));
  } catch(e) { if(!request.signal.aborted) error.value=e.message; }
}
onMounted(() => changeRecording(selection.value));
onBeforeUnmount(() => { pause(); controller?.abort(); clearRecordingCache(); disposeRobotTemplates(); });
</script>
<template>
  <main :data-stage="stage" :data-frame="frame" :data-imu="useImu" :data-loop-closure="useLoopClosure">
    <header class="page-header"><div><span class="eyebrow">GTDYNAMICS / G1 WALKING</span><h1>Kinematics</h1><p>Full-trajectory kinematic optimization with contact and inverse-kinematics initialization.</p></div><span class="recorded-badge"><i></i> OFFLINE RECORDING</span></header>
      <section class="toolbar">
        <div class="recording-controls">
        <div class="stage-selector" role="group" aria-label="Inspection stage"><button :aria-pressed="stage==='smoothing'" @click="changeStage('smoothing')">CM-Opt smoothing</button><button :aria-pressed="stage==='initializer'" @click="changeStage('initializer')">Initialization</button></div>
        <select :value="selection" aria-label="Recording" @change="changeRecording($event.target.value)"><option v-for="r in recordings" :key="r.id" :value="r.id">{{r.label}}</option></select>
        <label><input type="checkbox" :checked="useImu" @change="changeRecording(selection,$event.target.checked)"> IMU aiding</label>
        <label :title="selection === 'full' ? 'Soft return-to-start position observation; no orientation constraint' : 'Available for the entire trajectory; the turn excerpt does not return to its start'"><input type="checkbox" :checked="useLoopClosure" :disabled="selection !== 'full'" @change="changeRecording(selection,useImu,$event.target.checked)"> Loop closure applied <small v-if="selection !== 'full'">(entire trajectory)</small></label>
        </div>
        <div v-if="data" class="settings"><label><input type="checkbox" v-model="showReference"> {{referenceLabel}}</label><label>Joint <select v-model.number="joint" aria-label="Joint to inspect"><option v-for="(name,index) in data.record.jointNames" :key="name" :value="index">{{name}}</option></select></label></div>
      </section>
    <div v-if="error" role="alert" class="error-banner"><h2>Recording could not be loaded</h2><p>{{error}}</p><p>Generate the saved data with <code>python apps/kinematics-inspector/scripts/export_recording.py</code> from the repository root.</p></div>
    <div v-else-if="!data" class="loading" role="status">{{stage === 'smoothing' ? 'Loading smoothing iterations…' : 'Loading initializer histories…'}}</div>
    <template v-else>
      <section class="timeline">
        <p v-if="data.record.motionSource" class="motion-title">{{data.record.motionSource.title}}</p>
        <div class="row"><div class="time-heading"><button class="play-button" @click="play">{{playing ? 'Pause walking' : 'Play walking'}}</button><strong>{{(frame*data.record.dt).toFixed(2)}} <small>/ {{((data.record.samples-1)*data.record.dt).toFixed(2)}} s</small></strong></div><span class="mono">Frame {{frame}} · phase {{data.record.phases.indexOf(phase)+1}} / {{data.record.phases.length}} · {{phase?.contacts.join(' + ') || 'flight'}}</span></div>
        <input type="range" :value="frame" min="0" :max="data.record.samples-1" aria-label="Walking time" @input="scrub($event.target.value)">
        <div class="phase-strip" :class="{dense:data.record.phases.length>80}" aria-label="Contact phases"><button v-for="(p,index) in data.record.phases" :key="index" :class="{active:p===phase,double:p.contacts.length===2}" :style="{flex:p.end-p.start+1}" :title="`Phase ${index+1}: ${p.contacts.join(' + ') || 'flight'}, frames ${p.start}–${p.end}`" :aria-label="`Jump to phase ${index+1}`" @click="scrub(p.start)"></button></div>
        <div class="plot-foot"><span>Contact-aligned phases · click to jump</span><span>{{data.record.samples}} samples · {{data.record.graph.costFactors.toLocaleString()}} costs · {{data.record.graph.equalities.toLocaleString()}} equalities</span></div>
      </section>
      <div class="comparison"><MethodPanel v-for="method in ['contact','ik']" :key="method" :method="method" :stage="stage" :frame="frame" :show-reference="showReference" :data="data" :joint="joint" /></div>
      <footer><strong>About this recording</strong>
        <div v-if="motionCredit" class="motion-attribution">
          <p><a :href="data.record.motionSource.sourceUrl">{{data.record.motionSource.title}}</a> from <a :href="data.record.motionSource.datasetUrl">{{data.record.motionSource.datasetName || 'G1 Moves'}}</a> by {{motionCredit.creator}}.<template v-if="motionCredit.performer"> Performer: {{motionCredit.performer}}.</template> Licensed under <a :href="motionCredit.licenseUrl">CC BY 4.0</a>.</p>
          <p>{{motionCredit.changes}} Contacts are inferred from sole height and speed, not measured contact forces.</p>
          <a :href="`./${directory}${motionCredit.localNotice}`">Motion attribution and modifications</a>
        </div>
        <p v-if="data.record.motionSource?.imu">Physical pelvis IMU: {{data.record.motionSource.imu.samples.toLocaleString()}} accelerometer and gyroscope samples at nominally 200 Hz. Original timestamps are retained. {{useImu ? 'Galilean IMU factors connect the slices, with world-frame COM velocities and one shared sensor-frame bias.' : 'The kinematic smoother does not use IMU factors.'}} <a :href="`./${directory}${data.record.motionSource.imu.file}`" download>Download IMU measurements</a>.</p>
        <p v-if="useImu">Poses map COM coordinates into world coordinates (world_T_COM). The mounting transform maps IMU coordinates into the COM frame. GTSAM corrects sensor rotation and centripetal acceleration; angular-acceleration lever-arm terms are omitted. Initializers are unchanged. Aided and unaided objectives contain different factors; compare reference errors and physical residuals.</p>
        <p>Ground contact displacement is a soft cost: σ={{data.record.graph.contactSlipSigmaMeters * 1000}} mm per axis per slice interval ({{data.record.dt}} s). Stance feet may slip.</p>
        <p v-if="useLoopClosure">Loop closure adds a position prior on the final root COM at the anchored start position: σx=σy=30 cm, σz=5 cm. It adds no orientation constraint. This is a manually supplied observation for our smoother; it makes no assertion about loop closure in SuperOdom. Initializers and the LiDAR-inertial reference are unchanged.</p>
        <p v-if="data.record.motionSource?.fullTrajectory">Contact labels are detected at 50 Hz with a local floor estimate, then sampled at 5 Hz. {{data.record.motionSource.detector.unsupportedSamples}} samples have no detected support. Weak identity-motion priors connect {{data.record.motionSource.detector.contactComponents}} contact components for initialization (translation σ=10 m, rotation σ=π rad). The reference trajectory retains its original vertical drift.</p>
        <p>{{data.record.motionSource?.referenceDescription || 'Reference overlays are source root poses with measured-joint FK.'}} The first root pose anchors estimation; later reference poses are used for contact labeling and evaluation.</p>
        <details v-if="useImu"><summary>IMU frames, noise, and priors</summary><pre>{{JSON.stringify(data.record.imuAiding,null,2)}}</pre></details>
        <p>{{data.record.notes}}</p><p>Robot transforms and residuals were calculated offline from the example’s actual values; the viewer performs no optimization or forward kinematics.</p><details><summary>Source and input fingerprints</summary><pre>{{JSON.stringify(data.record.provenance,null,2)}}</pre></details><a :href="`./${directory}recording.json`" download>Download recording metadata</a> · <a href="./MODEL_LICENSE.txt">Robot model license</a></footer>
    </template>
  </main>
</template>
