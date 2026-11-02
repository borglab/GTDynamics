<script setup>
import { computed, markRaw, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import SignalPlot from './SignalPlot.vue';
import { frameAtTime, loadRecording } from './recording.js';
import { InterpolationScene } from './scene.js';

const data = ref(null), error = ref(''), sceneError = ref('');
const frame = ref(0), joint = ref(0), windowName = ref('middle');
const showKnots = ref(true), showVelocities = ref(false), playing = ref(false);
const sceneElement = ref(null);
const controller = new AbortController();
let scene, animation, startedAt, startedTime;
const record = computed(() => data.value?.record);
const time = computed(() => record.value?.times[frame.value] || 0);
const range = computed(() => windowName.value === 'middle' ? [78, 98]
  : [record.value.times[0], record.value.times.at(-1)]);
const firstFrame = computed(() => frameAtTime(record.value.times, range.value[0]));
const lastFrame = computed(() => frameAtTime(record.value.times, range.value[1]));
const jointName = computed(() => record.value?.jointNames[joint.value] || '');
const number = (value, digits = 3) => Number(value).toFixed(digits);
const degrees = value => value * 180 / Math.PI;
const column = (name, stride, index, convert = value => value) => Array.from(
  { length: record.value.times.length }, (_, i) => convert(data.value.arrays[name][i * stride + index]),
);
const jointAngleSeries = computed(() => {
  if (!data.value) return [];
  const count = record.value.jointNames.length;
  return [
    { name: 'Dense encoder', values: column('denseAngles', count, joint.value, degrees), color: '#98a19c' },
    { name: 'GP mean', values: column('jointAngles', count, joint.value, degrees), color: '#087f71' },
    { name: 'Fixed kinematics knots', times: record.value.knots.times,
      values: record.value.knots.jointAngles.map(row => degrees(row[joint.value])),
      color: '#ad5c28', mode: 'markers' },
  ];
});
const jointVelocitySeries = computed(() => data.value ? [{
  name: 'GP joint rate', color: '#087f71',
  values: column('jointVelocities', record.value.jointNames.length, joint.value, degrees),
}] : []);
const componentSeries = (offset, convert) => ['X', 'Y', 'Z'].map((name, i) => ({
  name, values: column('baseTwists', 6, offset + i, convert),
  color: ['#b2514e', '#087f71', '#536fbe'][i],
}));
const baseLinearSeries = computed(() => data.value ? componentSeries(3, value => value) : []);
const baseAngularSeries = computed(() => data.value ? componentSeries(0, degrees) : []);
const footSeries = computed(() => data.value ? ['Left', 'Right'].map((name, foot) => ({
  name: `${name} sole speed`, color: foot ? '#087f71' : '#bd4545',
  values: record.value.times.map((_, i) => {
    const offset = (i * 2 + foot) * 6 + 3;
    return Math.hypot(...data.value.arrays.footTwists.subarray(offset, offset + 3));
  }),
})) : []);

function updateScene() { scene?.update(frame.value, showKnots.value, showVelocities.value); }
function pause() { playing.value = false; cancelAnimationFrame(animation); }
function selectTime(value) {
  pause();
  frame.value = frameAtTime(record.value.times, Math.max(range.value[0], Math.min(value, range.value[1])));
}
function tick(now) {
  if (!playing.value) return;
  const elapsed = startedTime + (now - startedAt) / 1000;
  if (elapsed >= range.value[1]) { frame.value = lastFrame.value; pause(); return; }
  frame.value = frameAtTime(record.value.times, elapsed);
  animation = requestAnimationFrame(tick);
}
function play() {
  if (playing.value) { pause(); return; }
  if (frame.value >= lastFrame.value) frame.value = firstFrame.value;
  startedTime = time.value;
  startedAt = performance.now();
  playing.value = true;
  animation = requestAnimationFrame(tick);
}
watch([frame, showKnots, showVelocities], updateScene);
watch(windowName, () => selectTime(range.value[0]));
onMounted(async () => {
  try {
    data.value = markRaw(await loadRecording(controller.signal));
    frame.value = firstFrame.value;
    joint.value = Math.max(0, record.value.jointNames.indexOf('left_knee_joint'));
    await nextTick();
    if (controller.signal.aborted) return;
    try {
      scene = new InterpolationScene(sceneElement.value, data.value);
      updateScene();
      scene.follow(frame.value);
    } catch (e) { sceneError.value = e.message; }
  } catch (e) { if (!controller.signal.aborted) error.value = e.message; }
});
onBeforeUnmount(() => { pause(); controller.abort(); scene?.dispose(); });
</script>

<template>
  <main class="interpolation-app" :data-frame="frame">
    <header class="page-header">
      <div><span class="eyebrow">GTDYNAMICS / SSIO G1</span><h1>Interpolation</h1>
        <p>Floating-base and joint-velocity estimation from the Kinematics solution and dense encoders.</p>
      </div>
      <span class="recorded-badge">OFFLINE RECORDING</span>
    </header>
    <p v-if="error" role="alert" class="error-banner">{{ error }}</p>
    <p v-else-if="!data" role="status">Loading interpolation recording…</p>
    <template v-else>
      <section class="summary" aria-label="Interpolation statistics">
        <span>Source <strong>{{ record.source.variant }}</strong></span>
        <span><strong>{{ record.knots.times.length }}</strong> fixed knots</span>
        <span><strong>{{ record.times.length.toLocaleString() }}</strong> dense observations · {{ record.jointNames.length }} joints</span>
        <span>GP solve <strong>{{ number(record.diagnostics.solveSeconds, 2) }} s</strong></span>
        <span>Encoder RMS <strong>{{ number(record.diagnostics.jointMeasurementRmsDegrees) }}°</strong></span>
      </section>
      <section class="viewer-panel">
        <header class="viewer-tools">
          <div><button @click="scene?.fit(firstFrame, lastFrame)">Fit trajectory</button>
            <button @click="scene?.follow(frame)">Follow robot</button></div>
          <span>● Stance &nbsp; ○ Swing &nbsp; <span class="knot-legend">● Retained knot</span></span>
        </header>
        <p v-if="sceneError" role="alert" class="error-banner">{{ sceneError }}</p>
        <div ref="sceneElement" class="interpolation-scene" />
        <div class="timeline">
          <div class="time-row"><button @click="play">{{ playing ? 'Pause' : 'Play' }}</button>
            <strong>{{ number(time, 2) }} s</strong><span>Sample {{ frame + 1 }} / {{ record.times.length }}</span></div>
          <input type="range" :min="firstFrame" :max="lastFrame" :value="frame" aria-label="Playback time"
            @input="pause(); frame = Number($event.target.value)">
          <div class="controls">
            <label>Window <select v-model="windowName" aria-label="Playback window">
              <option value="middle">Mid-clip turns · 78–98 s</option><option value="full">Entire trajectory</option>
            </select></label>
            <label><input v-model="showKnots" type="checkbox"> Retained knots</label>
            <label><input v-model="showVelocities" type="checkbox"> Sole velocity vectors (0.3 m per m/s)</label>
          </div>
        </div>
      </section>
      <section class="plots-section">
        <div class="section-tools"><h2>Joint motion</h2>
          <label>Joint <select v-model.number="joint" aria-label="Joint">
            <option v-for="(name, index) in record.jointNames" :key="name" :value="index">{{ name }}</option>
          </select></label>
          <span>Encoder RMS {{ number(record.diagnostics.jointRmsDegrees[joint]) }}°</span>
        </div>
        <div class="plot-grid">
          <SignalPlot :title="`${jointName} · angle`" unit="degrees" :times="record.times"
            :series="jointAngleSeries" :time="time" :range="range" @select="selectTime" />
          <SignalPlot :title="`${jointName} · rate`" unit="degrees/s" :times="record.times"
            :series="jointVelocitySeries" :time="time" :range="range" @select="selectTime" />
          <SignalPlot title="Pelvis COM linear velocity · body axes" unit="m/s" :times="record.times"
            :series="baseLinearSeries" :time="time" :range="range" @select="selectTime" />
          <SignalPlot title="Pelvis angular velocity · body axes" unit="degrees/s" :times="record.times"
            :series="baseAngularSeries" :time="time" :range="range" @select="selectTime" />
        </div>
        <SignalPlot title="Sole world translation speed" unit="m/s" :times="record.times"
          :series="footSeries" :time="time" :range="range" @select="selectTime" />
      </section>
      <section class="diagnostics">
        <h2>Stance motion · entire trajectory</h2>
        <table><thead><tr><th>Sole</th><th>Stance samples</th><th>Translation RMS</th><th>Angular RMS</th></tr></thead>
          <tbody><tr v-for="(stats, side) in record.diagnostics.stance" :key="side">
            <th>{{ side }}</th><td>{{ stats.samples }}</td><td>{{ number(stats.linearRmsMetersPerSecond) }} m/s</td>
            <td>{{ number(stats.angularRmsDegreesPerSecond, 1) }}°/s</td>
          </tr></tbody></table>
      </section>
      <footer>
        <p>Knot configurations are fixed to the best saved Kinematics result, ranked by SE(3)-aligned pelvis error.
          Intermediate encoder measurements constrain one joint-vector WNOA GP. A separate Pose3 WNOA GP estimates
          pelvis twist, with saved world linear-velocity observations. No reference poses enter this fit.</p>
        <p>Contact labels are inherited from the privileged max-product support graph and used for diagnostics.
          This fit applies no stationary-foot constraints. Accelerations are reserved for the subsequent dynamics solve.
          Process PSDs describe white acceleration noise; conditional measurement covariances are augmented separately.</p>
        <p>Playback uses saved native FK transforms and actual timestamps. Plots support drag-to-zoom, scroll zoom,
          legend selection, and clicking a curve to move playback. Numerical data is stored in radians and SI units.</p>
        <p><a :href="record.source.motionSource.sourceUrl">SSIO / Open-IO G1 recording</a> ·
          <a href="./SSIO_ATTRIBUTION.txt">Dataset attribution</a> · <a href="./MODEL_LICENSE.txt">Model license</a> ·
          <a href="./recording.json.gz" download>Metadata</a> · <a href="./states.bin.gz" download>Trajectory data</a></p>
        <details><summary>Noise settings, frames, and source fingerprints</summary>
          <pre>{{ JSON.stringify({ settings: record.settings, frames: record.frames, source: record.source.checksums }, null, 2) }}</pre>
        </details>
      </footer>
    </template>
  </main>
</template>
