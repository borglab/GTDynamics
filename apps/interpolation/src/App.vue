<script setup>
import { computed, markRaw, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import SignalPlot from './SignalPlot.vue';
import FactorGraph from './FactorGraph.vue';
import FactorDefinitions from './FactorDefinitions.vue';
import apps from '../../registry.json';
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
const previousApp = apps[apps.findIndex(app => app.id === 'interpolation') - 1];
const previousHref = window.self === window.top
  ? new URL(`../${previousApp.id}/`, window.location.href).href
  : `${new URL('../', window.location.href).href}#${previousApp.id}`;
const knotIntervals = computed(() => record.value?.diagnostics.knotIntervalsSeconds || []);
const measurementFactors = computed(() => record.value.times.length - record.value.knots.times.length);
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
  <main id="demo" class="interpolation-app" :data-frame="frame">
    <header class="page-header">
      <div><span class="eyebrow">GTDYNAMICS / SSIO G1</span><h1>Interpolation</h1>
        <p class="abstract">The <a :href="previousHref" target="_parent">{{ previousApp.title }}</a> solution supplies sparse robot configurations;
          dynamics also needs velocities between them. This demo fits base and joint motion to those configurations
          and dense encoders using Gaussian-process factors, then inspects the resulting velocities and stance motion.
          For the full 220 s clip, recorded GP graph construction took 0.131 s and the solve took 0.038 s, excluding replay export.</p>
      </div>
      <span class="recorded-badge">OFFLINE RECORDING</span>
    </header>
    <p v-if="error" role="alert" class="error-banner">{{ error }}</p>
    <p v-else-if="!data" role="status">Loading interpolation recording…</p>
    <template v-else>
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
      <section class="paper-section" data-paper-section="motivation" aria-labelledby="motivation-title">
        <h2 id="motivation-title">From sparse configurations to velocities</h2>
        <p>The preceding solve estimates configurations at selected contact-phase slices. Finite differences
          depend strongly on slice spacing; independent curve fits can ignore the dense encoders and mix
          incompatible pose and velocity conventions. Here, a white-noise-on-acceleration (WNOA) model relates
          configuration and velocity at each retained slice, while intermediate encoders constrain joint motion.</p>
      </section>
      <section class="paper-section" data-paper-section="approach" aria-labelledby="approach-title">
        <h2 id="approach-title">Approach</h2>
        <p>At knot k, the unknowns are the pelvis COM body twist ν<sub>k</sub> ∈ ℝ⁶ and the
          joint-rate vector r<sub>k</sub> ∈ ℝ²⁹. The saved pose T<sub>k</sub> = world_T_COM
          and joint angles q<sub>k</sub> are fixed exactly, rather than held by finite-noise priors.
          A Pose3 GP and a single vector GP supply independent motion models. Saved Kinematics world-linear
          velocities provide additional base observations; dense root or LiDAR reference poses do not enter this fit.</p>
        <p>Substituting the fixed configurations leaves a Gaussian graph in velocities, solved once by sparse
          elimination. There is no nonlinear iteration. At query times, the GP conditional mean supplies
          configurations and their consistent first derivatives; native forward kinematics gives link and sole motion.</p>
        <FactorGraph :record="record" :time="time" @select="selectTime" />
      </section>
      <FactorDefinitions />
      <section class="paper-section results" data-paper-section="results" aria-labelledby="results-title">
        <h2 id="results-title">Recorded results</h2>
        <section class="summary" aria-label="Interpolation statistics">
          <span>Source <strong>{{ record.source.variant }}</strong></span>
          <span><strong>{{ record.knots.times.length }}</strong> fixed knots</span>
          <span><strong>{{ record.times.length.toLocaleString() }}</strong> dense observations · {{ record.jointNames.length }} joints</span>
          <span>GP solve <strong>{{ number(record.diagnostics.solveSeconds, 2) }} s</strong></span>
          <span>Encoder RMS <strong>{{ number(record.diagnostics.jointMeasurementRmsDegrees) }}°</strong></span>
        </section>
        <p>The source is the saved {{ record.source.variant }} variant, selected by the Kinematics comparison's
          minimum SE(3)-aligned pelvis error. All following values describe the entire exported trajectory;
          the playback window only changes the view. Click a plot to move playback; drag or scroll to zoom.</p>
        <div class="table-scroll"><table class="results-table">
          <thead><tr><th>Quantity</th><th>Recorded value</th><th>Scope</th></tr></thead>
          <tbody>
            <tr><th>Base / joint GP factors</th><td>{{ record.knots.times.length - 1 }} / {{ record.knots.times.length - 1 }}</td><td>One of each per adjacent knot pair</td></tr>
            <tr><th>Encoder factors</th><td>{{ measurementFactors.toLocaleString() }}</td><td>One 29-dimensional factor per interior measurement time; exact-knot observations add no velocity information</td></tr>
            <tr><th>Base velocity factors</th><td>{{ record.knots.times.length }}</td><td>Three-dimensional observations at every knot</td></tr>
            <tr><th>Knot interval</th><td>{{ number(Math.min(...knotIntervals)) }}–{{ number(Math.max(...knotIntervals)) }} s</td><td>Actual timestamps, including short contact phases</td></tr>
            <tr><th>GP process error</th><td>{{ number(record.diagnostics.processError, 2) }}</td><td>½ Σ eᵀ Q⁻¹ e for base and joint process factors; excludes measurement costs</td></tr>
            <tr><th>Graph assembly / solve / export</th><td>{{ number(record.diagnostics.graphConstructionSeconds, 3) }} / {{ number(record.diagnostics.solveSeconds, 3) }} / {{ number(record.diagnostics.exportSeconds, 2) }} s</td><td>Assembly accumulates successful add calls; solve covers elimination, result construction, and diagnostics. Export timer ends before writing metadata and geometry.</td></tr>
            <tr><th>MuJoCo FK / sole-twist discrepancy</th><td>{{ record.diagnostics.mujocoOracle.maxBodyPoseError.toExponential(2) }} / {{ record.diagnostics.mujocoOracle.maxSoleTwistError.toExponential(2) }}</td><td>Maximum component differences at 12 samples: body translation (m) / rotation matrix, and sole angular (rad/s) / linear velocity (m/s)</td></tr>
          </tbody>
        </table></div>
        <h3 class="subsection-title">Noise assumptions</h3>
        <p>These are chosen smoothing scales, not calibrated sensor specifications. Process scales are square
          roots of acceleration power spectral density (PSD), rather than per-sample acceleration standard deviations.
          Rotation scales below are specified in degrees and converted to radians before forming the PSD.</p>
        <div class="table-scroll"><table class="results-table">
          <thead><tr><th>Factor / component</th><th>Scale</th><th>Interpretation</th></tr></thead>
          <tbody>
            <tr><th>Base GP · rotation</th><td>{{ record.settings.baseAngularProcessSigmaDegreesPerSecondPowerThreeHalves }} °/s<sup>3/2</sup></td><td>Independent, identical scales on three angular axes</td></tr>
            <tr><th>Base GP · translation</th><td>{{ record.settings.baseLinearProcessSigmaMetersPerSecondPowerThreeHalves }} m/s<sup>3/2</sup></td><td>Independent, identical scales on three linear axes</td></tr>
            <tr><th>Joint-vector GP</th><td>{{ record.settings.jointProcessSigmaDegreesPerSecondPowerThreeHalves }} °/s<sup>3/2</sup></td><td>Same process scale for every joint</td></tr>
            <tr><th>Encoder measurement</th><td>{{ record.settings.jointMeasurementSigmaDegrees }}°</td><td>Standard deviation, before adding GP conditional covariance</td></tr>
            <tr><th>Base world-linear velocity</th><td>{{ record.settings.baseVelocitySigmaMetersPerSecond }} m/s</td><td>Isotropic standard deviation for saved Kinematics velocities</td></tr>
          </tbody>
        </table></div>
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
      </section>
      <section class="paper-section" data-paper-section="discussion" aria-labelledby="discussion-title">
        <h2 id="discussion-title">Discussion</h2>
        <p>The fit keeps the upstream solution intact and supplies velocities with explicit frames and units.
          Dense encoders constrain the interpolated joint motion, but fixed sparse knots can still cause overshoot
          and large local encoder errors. The plots and stance RMS values expose those disagreements rather
          than treating an interpolated curve as an additional measurement.</p>
        <p>Contact labels come from the privileged max-product support graph and are used only for coloring and
          diagnostics. No zero-foot-velocity or other stationary-foot factor is active. Consequently, the result
          can slip or rock during labeled stance. Encoder bridge correlations across measurement times are
          approximated as independent factors. The saved Kinematics velocities are derived observations, not
          independent sensor readings. Accelerations and dynamic consistency remain for a subsequent dynamics
          solve; this app estimates first derivatives only.</p>
        <p><a href="../#dynamics" target="_top">Next: Dynamics →</a></p>
      </section>
      <footer class="paper-section">
        <h2>Data, model, and downloads</h2>
        <p><a :href="record.source.motionSource.sourceUrl">SSIO / Open-IO G1 recording</a> ·
          <a href="./SSIO_ATTRIBUTION.txt">Dataset attribution</a> · <a href="./MODEL_LICENSE.txt">Model license</a> ·
          <a href="./recording.json.gz" download>Metadata</a> · <a href="./states.bin.gz" download>Trajectory data</a></p>
        <p>Motion: Yuheng Qiu and the SSIO authors / Open-IO, clip 10871_humanoid_20260520_223451,
          CC BY 4.0. The reference is SuperOdom LiDAR-inertial odometry. Geometry uses the pinned MuJoCo Menagerie
          G1 model through MJLab; model credit and license remain in the download above.
          Playback renders saved native body transforms with MuJoCo's Z-up convention, metre units, and explicit
          xyzw display quaternions. No native solver runs in the browser.</p>
        <details><summary>Frames, model versions, and source fingerprints</summary>
          <pre>{{ JSON.stringify({ frames: record.frames, modelVersions: record.modelVersions,
            modelFingerprint: record.modelFingerprint, source: record.source.checksums }, null, 2) }}</pre>
        </details>
      </footer>
    </template>
  </main>
</template>
