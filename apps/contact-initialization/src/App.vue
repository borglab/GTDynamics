<script setup>
import ContactPoseMath from '../../shared/paper/ContactPoseMath.vue';
import AppAbstract from '../../shared/paper/AppAbstract.vue';
import AppPaper from '../../shared/paper/AppPaper.vue';
import {
  computed,
  nextTick,
  onBeforeUnmount,
  onMounted,
  ref,
  shallowRef,
  watch,
} from "vue";
import PoseContactGraph from "./PoseContactGraph.vue";
import { ContactScene } from "./scene.js";
import { composePoses, inversePose } from "./poseMath.js";
import { factorVariants } from "./factorVariants.js";
import { loadJson } from "../../shared/loadJson.js";

const sequences = ref([]),
  sequenceId = ref(""),
  data = shallowRef(null),
  recording = shallowRef(null),
  error = ref(""),
  loading = ref(false);
const variant = ref("ground"),
  selected = ref(0),
  iteration = ref(0),
  stage = ref("estimate");
const showReference = ref(true),
  showGhosts = ref(true),
  playing = ref(false),
  replaying = ref(false),
  sceneHost = ref(null);
let scene,
  animation,
  lastEvent = 0,
  lastIteration = 0,
  generation = 0;
const abort = new AbortController();
const ssioAttribution = `${import.meta.env.BASE_URL}../contact-labels/SSIO_ATTRIBUTION.txt`;
const sequence = computed(() =>
  sequences.value.find((s) => s.id === sequenceId.value),
);
const run = computed(() => recording.value?.runs[variant.value]);
const graph = computed(() => run.value?.graph);
const history = computed(() => run.value?.result.history || []);
const snapshot = computed(() => history.value[iteration.value]);
const frames = computed(() => graph.value?.frames || []);
const rows = computed(() =>
  factorVariants.filter((v) => recording.value?.runs[v.id]),
);
const current = computed(() => frames.value[selected.value]);
const metrics = computed(() => snapshot.value?.referenceError);
const lossPoints = computed(() => {
  const values = history.value.map((s) =>
    Math.log10(Math.max(1e-12, s.objective)),
  );
  const lo = Math.min(...values),
    span = Math.max(1e-6, Math.max(...values) - lo);
  return values
    .map(
      (v, i) =>
        `${10 + (580 * i) / Math.max(1, values.length - 1)},${70 - (60 * (v - lo)) / span}`,
    )
    .join(" ");
});
const geometry = computed(() => {
  if (!graph.value || !snapshot.value || !data.value) return null;
  const g = graph.value,
    s = snapshot.value,
    reference = run.value.evaluation.alignedReferencePoses;
  const comTbody = inversePose(g.rootBodyTcom);
  const lift = (p) => ({
    ...p,
    translation: p.translation.map((v, i) =>
      i === 2 ? v - g.groundHeight : v,
    ),
  });
  const roots = stage.value === "reference" ? reference : s.poses;
  return {
    model: data.value.model,
    samples: data.value.samples,
    frames: g.frames,
    poses: roots.map((p) => lift(composePoses(p, comTbody))),
    referencePoses: reference.map((p) => lift(composePoses(p, comTbody))),
    contacts: g.contacts.map((c) => {
      let pose = s.contactPoses[c.id];
      if (stage.value === "reference") {
        const i = g.frames.findIndex((f) =>
          f.observations.some((o) => o.contactId === c.id),
        );
        pose = composePoses(
          reference[i],
          g.frames[i].observations.find((o) => o.contactId === c.id).localPose,
        );
      }
      return { ...c, pose: lift(pose) };
    }),
    components: g.components,
    floorConstrained: g.ground,
    mode: stage.value,
  };
});
function updateScene() {
  if (scene && geometry.value)
    scene.update({
      ...geometry.value,
      selectedIndex: selected.value,
      showReference: showReference.value && stage.value !== "reference",
      showGhosts: showGhosts.value,
    });
}
function selectFrame(i) {
  selected.value = Math.max(0, Math.min(frames.value.length - 1, Number(i)));
}
function selectVariant(id) {
  if (!recording.value?.runs[id]) return;
  variant.value = id;
  iteration.value = history.value.length - 1;
  selected.value = Math.min(selected.value, frames.value.length - 1);
  replaying.value = false;
  nextTick(updateScene);
}
function togglePrior(name, enabled) {
  const flags = {
    between: graph.value.between,
    ground: graph.value.ground,
    upright: !!graph.value.upright,
    [name]: enabled,
  };
  const match = factorVariants.find((v) =>
    ["between", "ground", "upright"].every((k) => v[k] === flags[k]),
  );
  if (match) selectVariant(match.id);
}
async function loadSequence() {
  const token = ++generation;
  loading.value = true;
  error.value = "";
  playing.value = false;
  replaying.value = false;
  try {
    const entry = sequence.value;
    const [dataset, saved] = await Promise.all([
      loadJson(entry.data, abort.signal, entry.data.endsWith(".gz")),
      loadJson(entry.recording, abort.signal, entry.recording.endsWith(".gz")),
    ]);
    if (token !== generation) return;
    if (
      saved.schemaVersion !== 3 ||
      saved.solver !== "Initializer.InitializeSolutionContactPoses"
    )
      throw new Error(
        "Expected a native pose-contact recording. Regenerate app assets.",
      );
    data.value = dataset;
    recording.value = saved;
    variant.value = saved.defaultVariant;
    iteration.value = history.value.length - 1;
    selected.value = 0;
    await nextTick();
    if (!scene) {
      scene = new ContactScene(sceneHost.value, { onSelect: selectFrame });
      await scene.init();
    }
    if (token !== generation) return;
    updateScene();
  } catch (e) {
    if (!abort.signal.aborted) error.value = e.message;
  } finally {
    if (token === generation) loading.value = false;
  }
}
function exportInitialization() {
  const payload = {
    schemaVersion: 3,
    solver: recording.value.solver,
    sequenceId: sequenceId.value,
    variant: variant.value,
    source: recording.value.source,
    graph: graph.value,
    result: history.value.at(-1),
    benchmark: run.value.benchmark,
    frames:
      "Root poses are world_T_pelvisCOM; contact poses are world_T_sole. Rotations are row-major; translations are metres.",
  };
  const url = URL.createObjectURL(
    new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" }),
  );
  const a = document.createElement("a");
  a.href = url;
  a.download = `${sequenceId.value}-pose-initialization.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
function animate(now) {
  if (playing.value && now - lastEvent > 420) {
    selectFrame((selected.value + 1) % frames.value.length);
    lastEvent = now;
  }
  if (replaying.value && now - lastIteration > 280) {
    if (iteration.value < history.value.length - 1) iteration.value++;
    else replaying.value = false;
    lastIteration = now;
  }
  animation = requestAnimationFrame(animate);
}
watch([geometry, selected, showReference, showGhosts], () =>
  nextTick(updateScene),
);
watch(sequenceId, loadSequence);
onMounted(async () => {
  try {
    const response = await fetch(`${import.meta.env.BASE_URL}sequences.json`, {
      signal: abort.signal,
    });
    sequences.value = (await response.json()).sequences;
    sequenceId.value = sequences.value[0].id;
    animation = requestAnimationFrame(animate);
  } catch (e) {
    if (!abort.signal.aborted) error.value = e.message;
  }
});
onBeforeUnmount(() => {
  generation++;
  abort.abort();
  cancelAnimationFrame(animation);
  scene?.dispose();
});
</script>

<template>
  <main id="demo"
    class="lab"
    data-model="pose"
    :data-sequence="sequenceId"
    :data-variant="variant"
    :data-event="selected"
  >
    <header>
      <h1>Contact Initialization</h1>
      <AppAbstract app="contact-initialization" />
    </header>
    <div class="toolbar">
      <label
        >Sequence
        <select
          v-model="sequenceId"
          aria-label="Motion sequence"
          :disabled="loading"
        >
          <option v-for="s in sequences" :key="s.id" :value="s.id">
            {{ s.title }}
          </option>
        </select></label
      ><span v-if="loading">Loading…</span
      ><button v-if="run" @click="exportInitialization">
        Export initialization JSON
      </button>
    </div>
    <p v-if="error" class="error-banner" role="alert">{{ error }}</p>
    <template v-if="run && snapshot">
      <div class="visualization-grid">
        <section class="scene-card">
          <div class="toolbar">
            <button @click="scene?.setView('perspective')">3D</button
            ><button @click="scene?.setView('side')">Side</button
            ><button @click="scene?.setView('top')">Top</button
            ><button aria-label="Fit trajectory" @click="scene?.fit()">
              Fit
            </button>
          </div>
          <div class="scene-wrap">
            <div
              ref="sceneHost"
              class="scene"
              role="img"
              aria-label="G1 trajectory and sole contact frames"
            ></div>
            <div class="scene-readout">
              Slice {{ current.sampleIndex }} ·
              {{
                (
                  current.time +
                  (recording.source.sourceIndices?.[0] ?? 0) / 50
                ).toFixed(3)
              }}
              s ·
              {{
                current.observations
                  .map((o) => graph.contacts[o.contactId].foot)
                  .join(" + ")
              }}
            </div>
          </div>
          <div class="toolbar playback">
            <button
              :aria-label="
                playing ? 'Pause event playback' : 'Play event playback'
              "
              @click="playing = !playing"
            >
              {{ playing ? "Pause" : "Play" }}</button
            ><button
              aria-label="Previous event"
              @click="selectFrame(selected - 1)"
            >
              ‹</button
            ><input
              type="range"
              aria-label="Contact event"
              min="0"
              :max="frames.length - 1"
              :value="selected"
              @input="selectFrame($event.target.value)"
            /><button
              aria-label="Next event"
              @click="selectFrame(selected + 1)"
            >
              ›</button
            ><span>{{ selected + 1 }} / {{ frames.length }}</span
            ><label
              ><input v-model="showReference" type="checkbox" />Reference</label
            ><label
              ><input v-model="showGhosts" type="checkbox" />Snapshots</label
            >
          </div>
          <div
            class="toolbar prior-controls"
            aria-label="Trajectory and prior controls"
          >
            <label
              >Display
              <select v-model="stage" aria-label="Display trajectory">
                <option value="estimate">Estimated trajectory</option>
                <option value="reference">Reference trajectory</option>
              </select></label
            >
            <label
              ><input
                type="checkbox"
                :checked="graph.between"
                :disabled="!!recording.unavailable.none"
                aria-label="Weak motion prior"
                @change="togglePrior('between', $event.target.checked)"
              />Weak motion</label
            >
            <label
              ><input
                type="checkbox"
                :checked="graph.ground"
                aria-label="Sole ground priors"
                @change="togglePrior('ground', $event.target.checked)"
              />{{
                graph.normal ? "Sole height and normal" : "Contact height"
              }}</label
            >
            <label v-if="rows.some((row) => row.upright)"
              ><input
                type="checkbox"
                :checked="graph.upright"
                aria-label="Pelvis upright prior"
                @change="togglePrior('upright', $event.target.checked)"
              />Pelvis upright</label
            >
          </div>
        </section>

      </div>
      <AppPaper app="contact-initialization" :frame="selected" :samples="frames.length" @select="selectFrame">
        <template #mathematics><ContactPoseMath :graph="graph" /></template>
        <template #graph>
        <PoseContactGraph
          :graph="graph"
          :selected="selected"
          @select="selectFrame"
        />
        </template>
      <div class="headline-metrics">
        <div>
          <span>Native LM median</span
          ><strong
            >{{
              run.benchmark.phases.optimization.medianMs.toFixed(2)
            }}
            ms</strong
          ><small
            >{{ run.result.iterations }} iterations ·
            {{
              run.result.stalled
                ? "stalled: no accepted decrease"
                : run.result.converged
                  ? "stopping criterion reached"
                  : "iteration cap reached"
            }}</small
          >
        </div>
        <div>
          <span>Full initializer call median</span
          ><strong
            >{{ run.benchmark.phases.call.medianMs.toFixed(2) }} ms</strong
          ><small>{{
            sequence.contactEvidence === "privileged-max-product"
              ? "Measured FK and sparse solve; no dense expansion"
              : "Includes frozen FK and dense expansion"
          }}</small>
        </div>
        <div>
          <span>Contact residual RMS</span
          ><strong
            >{{ (snapshot.rms * 1000).toFixed(2) }} mm /
            {{ ((snapshot.rotationRms * 180) / Math.PI).toFixed(2) }}°</strong
          ><small>Translation / rotation, before whitening</small>
        </div>
        <div>
          <span>Root reference RMSE</span
          ><strong
            >{{ metrics.positionRmseMeters.toFixed(3) }} m /
            {{ metrics.orientationRmseDegrees.toFixed(2) }}°</strong
          ><small>First pose aligned; LiDAR-inertial reference</small>
        </div>
      </div>
      <p v-if="recording.unavailable.none" class="notice">
        {{ recording.unavailable.none }} There is one global gauge; the hop is
        joined by a prior, not contact evidence.
      </p>
      <section class="solve-card">
        <h2>Recorded native iterations</h2>
        <div class="toolbar">
          <button
            :aria-label="
              replaying ? 'Pause native replay' : 'Replay GTSAM solve'
            "
            @click="
              replaying = !replaying;
              playing = false;
              if (replaying) iteration = 0;
            "
          >
            {{
              replaying ? "Pause native replay" : "Replay GTSAM solve"
            }}</button
          ><input
            type="range"
            aria-label="Solver iteration"
            min="0"
            :max="history.length - 1"
            v-model.number="iteration"
          /><span>Iteration {{ snapshot.iteration }}</span>
        </div>
        <svg
          viewBox="0 0 600 80"
          class="loss-plot"
          role="img"
          aria-label="Log weighted objective over native iterations"
        >
          <polyline
            :points="lossPoints"
            fill="none"
            stroke="#13836e"
            stroke-width="2"
          />
        </svg>
        <table class="factor-table">
          <caption>
            Error budget at iteration
            {{
              snapshot.iteration
            }}
            · GTSAM error = ½ weighted squared residual
          </caption>
          <thead>
            <tr>
              <th>Factors</th>
              <th>Count</th>
              <th>Residual dimensions</th>
              <th>Error</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="g in graph.factorGroups" :key="g.id">
              <th>{{ g.id }}</th>
              <td>{{ g.count }}</td>
              <td>{{ g.residualDimension }}</td>
              <td>{{ snapshot.factorErrors[g.id].toFixed(3) }}</td>
            </tr>
          </tbody>
        </table>
      </section>
      <section v-if="recording.source.labelStates" class="graph-card">
        <h2>Max-product support labels</h2>
        <svg
          viewBox="0 0 1000 28"
          width="100%"
          height="28"
          role="img"
          aria-label="Left red, right green, double stance yellow; selected slices marked black"
        >
          <rect
            v-for="(state, k) in recording.source.labelStates"
            :key="k"
            :x="k"
            y="0"
            width="1.1"
            height="20"
            :fill="['#c93838', '#16884b', '#e4bd31'][state]"
          />
          <line
            v-for="frame in frames"
            :key="frame.sampleIndex"
            :x1="frame.sampleIndex"
            :x2="frame.sampleIndex"
            y1="20"
            y2="28"
            stroke="black"
          />
        </svg>
        <p>
          {{ recording.source.sampling }} Black ticks mark solved slices. Red:
          left; green: right; yellow: double stance.
        </p>
        <p>
          Labels are the exact saved Contact Labels max-product result, with
          joint-velocity factors, duration priors, and walking constraints
          enabled. LiDAR-inertial poses contribute to those labels and the first
          root gauge; subsequent reference poses are used for evaluation, not as
          pose factors.
        </p>
        <a :href="ssioAttribution">SSIO dataset attribution and license</a>
      </section>
      <section class="comparison-panel">
        <h2>Saved prior configurations</h2>
        <p>
          Standard deviations: contact translation
          {{ graph.noise.contactTranslation * 100 }} cm, contact rotation
          {{ graph.noise.contactRotationDegrees.toFixed(1) }}°; weak zero-motion
          {{ graph.noise.betweenTranslation }} m /
          {{ graph.noise.betweenRotationDegrees.toFixed(0) }}°; height
          {{ graph.noise.height * 100 }} cm.
          <template v-if="rows.some((row) => row.upright)"
            >Pelvis upright {{ graph.noise.uprightDegrees.toFixed(1) }}°;
            contact height centered at {{ graph.groundHeight.toFixed(3) }} m in
            the solve frame; sole-normal alignment off.</template
          >
          <template v-else
            >Sole-normal alignment
            {{ graph.noise.normalDegrees.toFixed(2) }}°.</template
          >
          Ground height and upright leave horizontal position and yaw free.
        </p>
        <p v-if="graph.initializeUpright">
          All configurations use upright starting values. Contact height uses
          the first selected stance's ground level. “Stalled” means LM returned
          no decrease with a nonzero gradient.
        </p>
        <div class="table-scroll">
          <table class="configuration-table">
            <thead>
              <tr>
                <th>Configuration</th>
                <th>Iterations</th>
                <th>LM median (ms)</th>
                <th>Call median (ms)</th>
                <th>Contact RMS (mm / °)</th>
                <th>Root RMSE (m / °)</th>
              </tr>
            </thead>
            <tbody>
              <tr
                v-for="row in rows"
                :key="row.id"
                :class="{ active: row.id === variant }"
              >
                <th>
                  <button
                    @click="selectVariant(row.id)"
                    :aria-pressed="row.id === variant"
                  >
                    {{ row.title }}
                  </button>
                </th>
                <td>
                  {{ recording.runs[row.id].result.iterations
                  }}{{
                    recording.runs[row.id].result.stalled
                      ? " (stalled)"
                      : recording.runs[row.id].result.converged
                        ? ""
                        : " (limit)"
                  }}
                </td>
                <td>
                  {{
                    recording.runs[
                      row.id
                    ].benchmark.phases.optimization.medianMs.toFixed(2)
                  }}
                </td>
                <td>
                  {{
                    recording.runs[
                      row.id
                    ].benchmark.phases.call.medianMs.toFixed(2)
                  }}
                </td>
                <td>
                  {{
                    (
                      recording.runs[row.id].result.history.at(-1).rms * 1000
                    ).toFixed(2)
                  }}
                  /
                  {{
                    (
                      (recording.runs[row.id].result.history.at(-1)
                        .rotationRms *
                        180) /
                      Math.PI
                    ).toFixed(2)
                  }}
                </td>
                <td>
                  {{
                    recording.runs[row.id].result.history
                      .at(-1)
                      .referenceError.positionRmseMeters.toFixed(3)
                  }}
                  /
                  {{
                    recording.runs[row.id].result.history
                      .at(-1)
                      .referenceError.orientationRmseDegrees.toFixed(2)
                  }}
                </td>
              </tr>
            </tbody>
          </table>
        </div>
        <p class="timing-context">
          {{ run.benchmark.runs }} runs / {{ run.benchmark.warmup }} warmups.
          {{ run.benchmark.scope }}
        </p>
        <details>
          <summary>Timing samples and provenance</summary>
          <pre>{{
            JSON.stringify(
              { benchmark: run.benchmark, source: recording.source },
              null,
              2,
            )
          }}</pre>
        </details>
      </section>
      </AppPaper>
      <footer>
        <p>
          All solves use
          <code>Initializer.InitializeSolutionContactPoses</code> from
          GTDynamics through its Python bindings. The browser only selects saved
          results. No IMU or reference trajectory factors enter these solves.
        </p>
        <p>
          {{ sequence.description }}. Contact schedules remain input
          assumptions; a low residual does not establish no-slip contact or
          trajectory accuracy.
        </p>
        <a :href="sequence.sourceUrl">Motion source</a> ·
        <a href="G1_MOVES_ATTRIBUTION.txt">Capture attribution</a> ·
        <a href="THIRD_PARTY_LICENSES.txt">Recording licenses</a> ·
        <a href="assets/unitree_g1/LICENSE">Canonical model license</a>
      </footer>
    </template>
  </main>
</template>
