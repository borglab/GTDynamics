<script setup>
import ContactMath from '../../shared/paper/ContactMath.vue';
import AppAbstract from '../../shared/paper/AppAbstract.vue';
import AppPaper from '../../shared/paper/AppPaper.vue';
import {
  computed,
  markRaw,
  nextTick,
  onBeforeUnmount,
  onMounted,
  ref,
  watch,
} from "vue";
import { ContactScene } from "./scene.js";
import { playbackFrame } from "./playback.js";
import {
  loadJson,
  validateRecord,
  intervals,
  stateNames,
  stateColors,
  markerColors,
  supportNames,
  supportColors,
  footLabels,
  supportGraphResult,
  windowSupportStates,
} from "./recording.js";
import MotionPlot from "./MotionPlot.vue";
const snippets = ref([]),
  selection = ref("onset"),
  preset = ref("default"),
  method = ref("graph"),
  velocityAiding = ref(true),
  enforceWalkingSequence = ref(true),
  record = ref(null),
  frame = ref(0),
  error = ref(""),
  sceneElement = ref(null),
  walkingPrior = ref(true),
  showTrails = ref(true),
  playing = ref(false),
  view = ref("follow"),
  range = ref(null),
  reset = ref(0);
const sides = ["left", "right"];
let rootController = new AbortController(),
  loadController,
  model,
  scene,
  animation = 0,
  origin = 0,
  first = 0;
const thresholds = computed(() => record.value?.presets[preset.value]);
const selectedLabels = computed(() =>
  Object.fromEntries(
    sides.map((side) => [
      side,
      footLabels(
        record.value,
        side,
        preset.value,
        walkingPrior.value,
        method.value,
        velocityAiding.value,
        enforceWalkingSequence.value,
      ),
    ]),
  ),
);
const displayedLabels = (side) => selectedLabels.value[side];
const windowStates = computed(() =>
  windowSupportStates(record.value, preset.value, walkingPrior.value),
);
const activeGraph = computed(() =>
  supportGraphResult(record.value, velocityAiding.value, enforceWalkingSequence.value),
);
const comparisons = computed(() => [
  { name: "FG + velocity", states: supportGraphResult(record.value, true, enforceWalkingSequence.value).states },
  { name: "FG displacement", states: supportGraphResult(record.value, false, enforceWalkingSequence.value).states },
  { name: "Window model", states: windowStates.value },
]);
const agreement = computed(() => {
  const resolved = windowStates.value.filter((s) => s !== -1).length;
  const matches = windowStates.value.filter(
    (s, i) => s !== -1 && s === activeGraph.value.states[i],
  ).length;
  return { resolved, matches };
});
const velocitySeries = computed(() =>
  sides.map((side) => ({
    name: side === "left" ? "Left" : "Right",
    values: record.value.supportGraph.bodyVelocities[side].map((v) => v[0]),
  })),
);
const displacementSeries = computed(() =>
  sides.map((side) => ({
    name: side === "left" ? "Left" : "Right",
    values: record.value.supportGraph.displacements[side].map((d) => 1000 * d),
  })),
);
const statistics = computed(() =>
  Object.fromEntries(
    sides.map((side) => {
      const labels = displayedLabels(side);
      return [
        side,
        (method.value === "graph" ? [3, 4] : [1, 2, -1, 0]).map((label) => ({
          label,
          seconds: labels.filter((l) => l === label).length / 50,
        })),
      ];
    }),
  ),
);
const plotSeries = (field, scale = 1) =>
  sides.map((side) => ({
    name: side === "left" ? "Left" : "Right",
    values: (field === "score"
      ? record.value.feet[side].targets[preset.value].scores
      : record.value.feet[side].motion[field]
    ).map((v) => v * scale),
  }));
const series = computed(() => ({
  score: plotSeries("score"),
  speed: plotSeries("maxSpeed"),
  excursion: plotSeries("maxExcursion", 1000),
  angular: plotSeries("angularSpeed", 180 / Math.PI),
  angularExcursion: plotSeries("angularExcursion", 180 / Math.PI),
}));
const bounds = computed(() => ({
  speed: [thresholds.value.stationarySpeed, thresholds.value.movingSpeed],
  excursion: [
    thresholds.value.stationaryExcursion * 1000,
    thresholds.value.movingExcursion * 1000,
  ],
}));
const bands = computed(() =>
  Object.fromEntries(
    sides.map((side) => [side, intervals(displayedLabels(side))]),
  ),
);
function pause() {
  cancelAnimationFrame(animation);
  playing.value = false;
}
function selectFrame(value) {
  pause();
  frame.value = Number(value);
}
function tick(now) {
  if (!playing.value) return;
  frame.value = playbackFrame(first, now - origin, record.value.times.length);
  if (frame.value === record.value.times.length - 1) pause();
  else animation = requestAnimationFrame(tick);
}
function play() {
  if (playing.value) {
    pause();
    return;
  }
  first = frame.value === 999 ? 0 : frame.value;
  origin = performance.now();
  playing.value = true;
  animation = requestAnimationFrame(tick);
}
function camera(value) {
  view.value = value;
  if (value === "fit") scene?.fit();
  else scene?.follow();
}
function update() {
  if (scene && record.value)
    scene.update(
      frame.value,
      preset.value,
      showTrails.value,
      walkingPrior.value,
      method.value,
      velocityAiding.value,
      enforceWalkingSequence.value,
    );
}
async function changeSnippet() {
  pause();
  loadController?.abort();
  const request = new AbortController();
  loadController = request;
  record.value = null;
  error.value = "";
  frame.value = 0;
  range.value = null;
  try {
    const data = validateRecord(
      await loadJson(
        `snippets/${selection.value}.json.gz`,
        request.signal,
        true,
      ),
    );
    if (request.signal.aborted) return;
    record.value = markRaw(data);
  } catch (e) {
    if (!request.signal.aborted) error.value = e.message;
  }
}
watch(record, async (data) => {
  scene?.dispose();
  scene = null;
  if (!data) return;
  await nextTick();
  if (record.value !== data || !sceneElement.value) return;
  try {
    scene = new ContactScene(sceneElement.value, model, data);
    update();
    camera(view.value);
  } catch (e) {
    error.value = e.message;
  }
});
watch(
  [frame, preset, showTrails, walkingPrior, method, velocityAiding, enforceWalkingSequence],
  update,
);
onMounted(async () => {
  try {
    [snippets.value, model] = await Promise.all([
      loadJson("snippets.json", rootController.signal),
      loadJson("model.json.gz", rootController.signal, true),
    ]);
    if (!rootController.signal.aborted) await changeSnippet();
  } catch (e) {
    if (!rootController.signal.aborted) error.value = e.message;
  }
});
onBeforeUnmount(() => {
  pause();
  rootController.abort();
  loadController?.abort();
  scene?.dispose();
});
</script>
<template>
  <main id="demo"
    class="labels-app"
    :data-frame="frame"
    :data-snippet="selection"
    :data-preset="preset"
    :data-walking-prior="walkingPrior"
    :data-method="method"
    :data-velocity-aiding="velocityAiding"
    :data-walking-constraints="enforceWalkingSequence"
  >
    <header class="page-header">
      <div>
        <span class="eyebrow">GTDYNAMICS / SSIO G1</span>
        <h1>Contact Labels</h1>
        <AppAbstract app="contact-labels" />
      </div>
      <span class="recorded-badge">PRIVILEGED LABELS</span>
    </header>
    <section class="toolbar">
      <div class="recording-controls">
        <label
          >Viewer labels
          <select v-model="method" aria-label="Label model">
            <option value="graph">Discrete FG · max-product</option>
            <option value="window">Window model</option>
          </select></label
        >
        <label
          >Snippet
          <select
            v-model="selection"
            aria-label="Snippet"
            @change="changeSnippet"
          >
            <option
              v-for="snippet in snippets"
              :key="snippet.id"
              :value="snippet.id"
            >
              {{ snippet.label }}
            </option>
          </select></label
        ><label
          >Window thresholds
          <select v-model="preset" aria-label="Motion thresholds">
            <option value="strict">Strict</option>
            <option value="default">Default</option>
            <option value="tolerant">Tolerant</option>
          </select></label
        >
      </div>
      <label
        ><input
          type="checkbox"
          v-model="velocityAiding"
          :disabled="method !== 'graph'"
        />
        Joint-velocity factors</label
      >
      <label>
        <input type="checkbox" v-model="enforceWalkingSequence" :disabled="method !== 'graph'" />
        Require support and double-stance transitions
      </label>
      <label
        ><input type="checkbox" v-model="walkingPrior" /> Walking support
        prior</label
      ><label
        ><input type="checkbox" v-model="showTrails" /> Contact-frame
        trails</label
      >
    </section>
    <div v-if="error" role="alert" class="error-banner">{{ error }}</div>
    <div v-else-if="!record" role="status" class="loading">
      Loading reference poses and derived labels…
    </div>
    <template v-else>
      <section class="label-viewer">
        <header class="viewer-tools">
          <div class="button-group">
            <button @click="camera('fit')" :aria-pressed="view === 'fit'">
              Fit snippet</button
            ><button
              @click="camera('follow')"
              :aria-pressed="view === 'follow'"
            >
              Follow robot
            </button>
          </div>
          <div class="legend">
            <span
              v-for="label in method === 'graph'
                ? [3, 4]
                : walkingPrior
                  ? [1, 2, -1, 0]
                  : [1, -1, 0]"
              :key="label"
              :style="{ color: '#24382c' }"
              ><i
                class="contact-swatch"
                :style="{ background: markerColors[label] }"
              ></i>
              {{
                label === 3
                  ? "Stance"
                  : label === 4
                    ? "Swing"
                    : stateNames[label]
              }}</span
            >
          </div>
        </header>
        <div ref="sceneElement" class="label-scene"></div>

        <div class="time-controls">
          <div class="row">
            <button @click="play">{{ playing ? "Pause" : "Play" }}</button
            ><strong
              >{{ record.times[frame].toFixed(2) }} s
              <small
                >· {{ (record.times[frame] - record.times[0]).toFixed(2) }} / 20
                s</small
              ></strong
            ><span class="mono"
              >Sample {{ frame + 1 }} / {{ record.times.length }} · 50 Hz</span
            >
          </div>
          <input
            type="range"
            min="0"
            :max="record.times.length - 1"
            :value="frame"
            aria-label="Clip time"
            @input="selectFrame($event.target.value)"
          />
          <div v-for="side in sides" :key="side" class="label-band">
            <strong>{{ side === "left" ? "Left" : "Right" }}</strong>
            <div>
              <button
                v-for="(band, i) in bands[side]"
                :key="i"
                :style="{
                  flex: band.end - band.start,
                  background: markerColors[band.label],
                }"
                :title="`${stateNames[band.label]}: ${record.times[band.start].toFixed(2)}–${(record.times[band.end - 1] + 0.02).toFixed(2)} s`"
                :aria-label="`${side} ${stateNames[band.label]} interval ${i + 1}`"
                @click="selectFrame(band.start)"
              ></button
              ><i
                :style="{
                  left: `${(100 * frame) / (record.times.length - 1)}%`,
                }"
              ></i>
            </div>
          </div>
        </div>
      </section>
      <AppPaper app="contact-labels" :frame="frame" :samples="record.times.length" @select="selectFrame">
        <template #mathematics><ContactMath :graph="record.supportGraph" :velocity-aiding="velocityAiding" :walking-constraints="enforceWalkingSequence" /></template>
        <p class="scope viewer-caption">
          Dots and trails: black stance, white swing, gray unresolved. Axes:
          sole frames (+X forward, +Y left, +Z up). Dashed line: reference
          pelvis path. Geometry and motion are recorded; no browser
          optimization.
        </p>
      <section class="graph-comparison">
        <h2>Support sequence comparison</h2>
        <p class="scope">
          Joint MAP labels from GTSAM max-product. L: left support; R: right
          support; D: double stance. All three rows use the same 50 Hz poses.
          Window settings affect only the window model.
        </p>
        <div
          v-for="comparison in comparisons"
          :key="comparison.name"
          class="label-band comparison-band"
        >
          <strong>{{ comparison.name }}</strong>
          <div>
            <button
              v-for="(band, i) in intervals(comparison.states)"
              :key="i"
              :style="{
                flex: band.end - band.start,
                background: supportColors[band.label],
              }"
              :title="`${supportNames[band.label]}: ${record.times[band.start].toFixed(2)}–${(record.times[band.end - 1] + 0.02).toFixed(2)} s`"
              :aria-label="`${comparison.name} ${supportNames[band.label]} interval ${i + 1}`"
              @click="selectFrame(band.start)"
            ></button>
            <i
              :style="{ left: `${(100 * frame) / (record.times.length - 1)}%` }"
            ></i>
          </div>
          <span>{{ supportNames[comparison.states[frame]] }}</span>
        </div>
        <div class="legend">
          <span
            v-for="label in [0, 1, 2, -1]"
            :key="label"
            :style="{ color: supportColors[label] }"
            >● {{ supportNames[label] }}</span
          >
        </div>
        <p class="scope">
          The {{
            velocityAiding ? "FG + velocity" : "FG displacement"
          }} model agrees with the window model on {{ agreement.matches }} of
          {{ agreement.resolved }} resolved samples{{
            agreement.resolved
              ? ` (${((100 * agreement.matches) / agreement.resolved).toFixed(1)}%)`
              : ""
          }}. The comparison excludes {{ record.times.length - agreement.resolved }} unresolved samples.
        </p>
        <p class="scope">
          The discrete graph contains {{ record.supportGraph.variableCount }} binary
          variables on {{ record.supportGraph.sliceCount }} slices and
          {{ activeGraph.factorCount }} factors. Inference includes
          {{ record.supportGraph.contextSamplesPerEnd }} context samples at each
          end. The offline max-product solve took
          {{ (1000 * activeGraph.solveSeconds).toFixed(2) }} ms. Joint-velocity
          factors are {{ velocityAiding ? "enabled" : "disabled" }}, and walking constraints
          are {{ enforceWalkingSequence ? "enabled" : "disabled" }}.
        </p>
      </section>
      <div class="foot-cards">
        <section
          v-for="side in sides"
          :key="side"
          class="foot-card"
          :data-foot="side"
        >
          <div class="row">
            <h2>{{ side === "left" ? "Left" : "Right" }} foot</h2>
            <strong
              :style="{ color: stateColors[displayedLabels(side)[frame]] }"
              >{{ stateNames[displayedLabels(side)[frame]] }}</strong
            >
          </div>
          <p class="raw-label">
            Raw motion:
            {{ stateNames[record.feet[side].targets[preset].labels[frame]]
            }}<span v-if="displayedLabels(side)[frame] === 2">
              · Contact inferred from the other foot’s translation; rocking is
              allowed.</span
            >
          </p>
          <p class="foot-motion">
            {{ record.feet[side].motion.maxSpeed[frame].toFixed(3) }} m/s ·
            {{
              (record.feet[side].motion.maxExcursion[frame] * 1000).toFixed(1)
            }}
            mm ·
            {{
              (
                (record.feet[side].motion.angularSpeed[frame] * 180) /
                Math.PI
              ).toFixed(1)
            }}
            °/s
          </p>
          <div class="durations">
            <span
              v-for="stat in statistics[side]"
              :key="stat.label"
              :style="{ color: stateColors[stat.label] }"
              >{{ stateNames[stat.label] }}
              {{ stat.seconds.toFixed(2) }} s</span
            >
          </div>
          <p>
            Angular excursion:
            {{
              (
                (record.feet[side].motion.angularExcursion[frame] * 180) /
                Math.PI
              ).toFixed(2)
            }}°. Each foot uses one material contact frame.
          </p>
        </section>
      </div>
      <section class="plots">
        <div class="row">
          <h2>Motion and targets</h2>
          <button
            @click="
              range = null;
              reset++;
            "
          >
            Reset plot zoom
          </button>
        </div>
        <p class="scope">
          The plots show raw motion evidence for the left foot in teal and the
          right foot in purple. Dashed lines mark stationary and moving
          thresholds. Scores express margins relative to these thresholds and
          remain unchanged by the walking prior. The plots share a time range;
          dragging or scrolling zooms, and clicking a curve selects a sample.
        </p>
        <MotionPlot
          :times="record.times"
          :series="series.score"
          :frame="frame"
          label="Stationarity score"
          unit="score"
          :range="range"
          :reset="reset"
          @range="range = $event"
          @select="selectFrame"
        />
        <MotionPlot
          :times="record.times"
          :series="series.speed"
          :frame="frame"
          label="Contact-origin speed"
          unit="m/s"
          :thresholds="bounds.speed"
          :range="range"
          :reset="reset"
          @range="range = $event"
          @select="selectFrame"
        />
        <MotionPlot
          :times="record.times"
          :series="series.excursion"
          :frame="frame"
          label="Contact-origin excursion"
          unit="mm"
          :thresholds="bounds.excursion"
          :range="range"
          :reset="reset"
          @range="range = $event"
          @select="selectFrame"
        />
        <MotionPlot
          :times="record.times"
          :series="series.angular"
          :frame="frame"
          label="Contact-frame angular speed"
          unit="°/s"
          :thresholds="[
            (thresholds.stationaryAngularSpeed * 180) / Math.PI,
            (thresholds.movingAngularSpeed * 180) / Math.PI,
          ]"
          :range="range"
          :reset="reset"
          @range="range = $event"
          @select="selectFrame"
        />
        <MotionPlot
          :times="record.times"
          :series="series.angularExcursion"
          :frame="frame"
          label="Contact-frame angular excursion"
          unit="°"
          :thresholds="[
            (thresholds.stationaryAngularExcursion * 180) / Math.PI,
            (thresholds.movingAngularExcursion * 180) / Math.PI,
          ]"
          :range="range"
          :reset="reset"
          @range="range = $event"
          @select="selectFrame"
        />
        <MotionPlot
          :times="record.times"
          :series="displacementSeries"
          :frame="frame"
          label="Consecutive translation difference · FG measurement"
          unit="mm / 20 ms"
          :range="range"
          :reset="reset"
          @range="range = $event"
          @select="selectFrame"
        />
        <MotionPlot
          :times="record.times"
          :series="velocitySeries"
          :frame="frame"
          label="Foot forward velocity relative to pelvis · unary measurement"
          unit="m/s"
          :thresholds="[0]"
          :range="range"
          :reset="reset"
          @range="range = $event"
          @select="selectFrame"
        />
      </section>
      </AppPaper>
      <footer>
      <p class="scope target-definition">
        Targets use SuperOdom LiDAR-inertial poses and measured-joint FK. Window
        labels indicate stationarity, and graph labels infer support. Their
        motion-based evidence can also classify a foot held stationary in the air.
        Independent force measurements would be needed to validate physical
        contact. A future classifier would use onboard measurements only.
      </p>
      <p class="scope support-definition">
        The walking support prior favors contact for the less-translating foot
        when the other contact origin clearly translates. This support category
        includes rocking. Both feet translating, rotation alone, or incomplete
        windows leave the raw motion labels unchanged. Disabling the prior
        displays the raw labels throughout.
      </p>
      <section class="threshold-note">
        <strong>{{ record.windowSeconds * 1000 }} ms centered window</strong
        ><span
          >Stationary: contact origin ≤
          {{ thresholds.stationarySpeed.toFixed(2) }} m/s and ≤
          {{ (thresholds.stationaryExcursion * 1000).toFixed(0) }} mm
          excursion.</span
        ><span
          >Moving: contact origin ≥ {{ thresholds.movingSpeed.toFixed(2) }} m/s
          or ≥ {{ (thresholds.movingExcursion * 1000).toFixed(0) }} mm. Angular
          bounds also apply:
          {{ ((thresholds.stationaryAngularSpeed * 180) / Math.PI).toFixed(1) }}
          /
          {{ ((thresholds.movingAngularSpeed * 180) / Math.PI).toFixed(1) }} °/s
          and
          {{
            ((thresholds.stationaryAngularExcursion * 180) / Math.PI).toFixed(1)
          }}
          /
          {{ ((thresholds.movingAngularExcursion * 180) / Math.PI).toFixed(1) }}
          ° excursion. Between thresholds: uncertain.</span
        ><small
          >Speed uses window endpoints. Excursion checks every sample against
          the central pose, catching motion that returns to its starting point.
          Windows use the available source samples, including context outside
          each snippet. Gaps remain unfilled.</small
        >
      </section>
        <p class="scope">
          The reported agreement measures consistency between the two models.
          Graph measurements use consecutive translation differences and optional
          joint-derived forward velocities. Rotation and window statistics belong
          to the window model.
        </p>
        <p>
          <a :href="record.source.sourceUrl"
            >SSIO / Open-IO · {{ record.source.clip }}</a
          >
          by Yuheng Qiu and the SSIO authors / Open-IO. Motion licensed under
          <a href="https://creativecommons.org/licenses/by/4.0/">CC BY 4.0</a>.
          <a href="./SSIO_ATTRIBUTION.txt">Attribution and modifications</a> ·
          <a href="./MODEL_LICENSE.txt">Robot model license</a>.
        </p>
        <p>
          SuperOdom provides the LiDAR-inertial odometry reference. Its estimation
          error and finite-difference noise affect these
          derived targets. Uncertain samples should be masked when training.
          Target generation and future classifier evaluation require separate
          recordings or held-out intervals.
        </p>
        <details>
          <summary>Target provenance and input boundary</summary>
          <pre>{{
            JSON.stringify(
              {
                walkingPrior: record.walkingPrior,
                supportGraph: {
                  ...record.supportGraph,
                  states: undefined,
                  withoutWalkingConstraints: undefined,
                  displacements: undefined,
                },
                source: record.source,
                provenance: record.provenance,
              },
              null,
              2,
            )
          }}</pre>
        </details>
        <a :href="`./snippets/${selection}.json.gz`" download
          >Download labels, motion diagnostics, and display poses</a
        >
      </footer>
    </template>
  </main>
</template>
