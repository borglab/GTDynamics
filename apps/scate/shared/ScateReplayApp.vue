<script setup>
/** The SCATE replay page, shared by the UR5 and UR10e apps. */
import { computed, onMounted, onBeforeUnmount, reactive, ref, shallowRef } from 'vue';
import AppAbstract from '../../shared/paper/AppAbstract.vue';
import AppPaper from '../../shared/paper/AppPaper.vue';
import MathTypeset from '../../shared/paper/MathTypeset.vue';
import SignalPlot from '../../shared/dynamics/SignalPlot.vue';
import RunningPlot from './RunningPlot.vue';
import equations from './scate.math.html?math';
import { createReplayViewer, isComparison, runColors, UR_MESH_FOR_LINK } from './viewer.js';

const props = defineProps({
  /** App id in apps/registry.json, e.g. "scate/ur5". */
  app: { type: String, required: true },
  /** Robot name shown on the page, e.g. "UR5". */
  robot: { type: String, required: true },
  /** Page heading; "SCATE <robot>" if empty. */
  title: { type: String, default: '' },
  /** Mesh name in meshes.json for each link name in the replay. */
  meshForLink: { type: Object, default: () => UR_MESH_FOR_LINK },
});

const root = ref(null), replay = shallowRef(null), time = ref(0), joint = ref(0);
let viewer, download;
const downloadUrl = ref('');
const frame = computed(() => replay.value ? Math.min(replay.value.num_poses - 1, Math.floor(time.value / replay.value.dt)) : 0);
const runs = computed(() => replay.value?.runs || []);
const format = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const times = computed(() => runs.value[0]?.frames.map(f => f.t) || []);
const displayFrame = computed(() => Math.max(0, times.value.findLastIndex(t => t <= time.value)));
const stepTimes = computed(() => runs.value[0]?.steps.map(s => s.k * replay.value.dt) || []);
const controlTimes = computed(() => stepTimes.value.slice(0, -1));
// Run colors as literals: the online and blind theme colors, then the comparison colors.
const THEME_COLORS = { 'var(--online)': '#3987e5', 'var(--blind)': '#eb6834' };
const colors = computed(() => Object.fromEntries(Object.entries(runColors(runs.value)).map(([name, c]) => [name, THEME_COLORS[c] || c])));
const color = run => colors.value[run.name];
// A comparison of base-disturbance costs (or any runs beyond online and blind) has per-run toggles.
const comparison = computed(() => isComparison(runs.value));
const hidden = reactive(new Set());
const shownRuns = computed(() => runs.value.filter(run => !hidden.has(run.name)));
function toggleRun(name, visible) {
  if (visible) hidden.delete(name); else hidden.add(name);
  viewer?.setRunVisible(name, visible);
}
const stateSeries = field => shownRuns.value.map(run => ({ name: run.label, color: color(run), values: run.frames.map(f => f[field][joint.value]) }));
const controls = computed(() => shownRuns.value.map(run => ({name: run.label, color: color(run), values: run.u.map(u => u[joint.value])})));
const stepSeries = field => shownRuns.value.map(run => ({name: run.label, color: color(run), values: run.steps.map(step => step[field])}));

// Base reaction wrench [torque; force] per frame, when the replay records it.
const hasBase = computed(() => !!runs.value[0]?.frames[0]?.base);
const torque = frame => Math.hypot(frame.base[0], frame.base[1], frame.base[2]);
const force = frame => Math.hypot(frame.base[3], frame.base[4], frame.base[5]);
const baseSeries = measure => shownRuns.value.map(run => ({ name: run.label, color: color(run), values: run.frames.map(measure) }));
const clearanceSeries = computed(() => shownRuns.value.map(run => ({
  name: run.label, color: color(run), values: run.frames.map(f => f.clear === null ? null : 100 * f.clear),
})));
const clearanceReferences = computed(() => replay.value ? [
  { value: 0, label: 'contact', color: 'var(--critical)' },
  { value: 100 * replay.value.obstacle_standoff, label: 'standoff', dashed: true },
] : []);
// A value of a run at the playback time, linear between frames.
function valueAt(run, measure) {
  const frames = run.frames;
  let i = 0;
  while (i + 1 < frames.length && frames[i + 1].t <= time.value) i++;
  const a = frames[i], b = frames[Math.min(i + 1, frames.length - 1)];
  const va = measure(a), vb = measure(b);
  if (va === null || vb === null || b.t === a.t) return va;
  return va + (vb - va) * Math.max(0, Math.min(1, (time.value - a.t) / (b.t - a.t)));
}
const clearanceState = clear => clear === null ? 'idle' : clear < 0 ? 'bad' : clear < replay.value.obstacle_standoff ? 'warn' : 'ok';
const clearanceText = { idle: '–', bad: 'collision', warn: 'inside standoff', ok: 'clear' };
// Whole-run base-disturbance measures: RMS, peak and integral of the torque magnitude.
function baseSummary(run) {
  const frames = run.frames, values = frames.map(torque);
  let integral = 0, square = 0;
  for (let k = 1; k < frames.length; k++) {
    const dt = frames[k].t - frames[k - 1].t;
    integral += 0.5 * (values[k] + values[k - 1]) * dt;
    square += 0.5 * (values[k] ** 2 + values[k - 1] ** 2) * dt;
  }
  const duration = frames.at(-1).t - frames[0].t;
  return { rms: Math.sqrt(square / duration), peak: Math.max(...values), integral };
}
function selectTime(t) { viewer?.selectTime(t); }
function receiveReplay(data) {
  replay.value = data;
  joint.value = 0;
  hidden.clear();
  if (download) URL.revokeObjectURL(download);
  download = URL.createObjectURL(new Blob([JSON.stringify(data)], {type: 'application/json'}));
  downloadUrl.value = download;
}
onMounted(() => {
  viewer = createReplayViewer(root.value, {
    robotName: props.robot, meshForLink: props.meshForLink,
    onReplay: receiveReplay, onTime: t => {time.value = t;},
  });
});
onBeforeUnmount(() => { viewer?.dispose(); if (download) URL.revokeObjectURL(download); });
</script>

<template>
  <main ref="root" class="scate-app" :data-frame="frame">
    <header class="page-header">
      <div><h1>{{ title || `SCATE ${robot}` }}</h1><AppAbstract :app="app" /></div>
      <div class="source"><span class="tag">Recorded native solve</span>
        <span class="tag" id="srcTag">Loading</span>
        <label class="filebtn" for="file">Open replay.json<input id="file" type="file" accept=".json,application/json"></label>
      </div>
    </header>
    <section id="demo" aria-label="Interactive SCATE replay">
  <div class="main">
    <div class="stage" id="stage">

      <span class="clock" id="clock">t = 0.00 s</span>
      <span class="hint">Drag to orbit · scroll to zoom · right-drag to pan</span>
      <div class="drop" id="drop" hidden>Drop replay.json to load it</div>
      <div class="err" id="err" hidden></div>
    </div>

    <aside class="demo-readouts">
      <section v-if="comparison" class="panel wide" aria-labelledby="h-runs" data-run-toggles>
        <h2 id="h-runs">Runs at this instant</h2>
        <table class="run-table">
          <thead><tr><th scope="col">Show</th><th scope="col">Clearance</th><th v-if="hasBase" scope="col">Base ‖τ‖</th></tr></thead>
          <tbody>
            <tr v-for="run in runs" :key="run.name" class="run-toggle" :data-run="run.name">
              <th scope="row"><label class="runname">
                <input type="checkbox" :checked="!hidden.has(run.name)" @change="toggleRun(run.name, $event.target.checked)">
                <span class="swatch" :style="{ background: color(run) }"></span>{{ run.label }}
              </label></th>
              <td :class="`gap ${clearanceState(valueAt(run, f => f.clear))}`" :title="clearanceText[clearanceState(valueAt(run, f => f.clear))]">{{ format(100 * valueAt(run, f => f.clear), 1) }} cm</td>
              <td v-if="hasBase">{{ format(valueAt(run, torque)) }} N·m</td>
            </tr>
          </tbody>
        </table>
      </section>

      <section v-if="hasBase && replay" class="panel wide" aria-labelledby="h-running" data-running-plots>
        <h2 id="h-running">So far</h2>
        <div class="running-plots">
          <RunningPlot title="Base torque ‖τ‖" unit="N·m" :times="times" :series="baseSeries(torque)" :time="time" :digits="1" @select="selectTime" />
          <RunningPlot title="Obstacle clearance" unit="cm" :times="times" :series="clearanceSeries" :time="time" :references="clearanceReferences" :max="15" :digits="0" @select="selectTime" />
        </div>
      </section>

      <section v-show="!comparison" class="panel" aria-labelledby="h-state">
        <h2 id="h-state">At this instant</h2>
        <div class="runrow">
          <span class="runname"><span class="swatch" style="background:var(--online)"></span><span id="nmOnline">SCATE online</span></span>
          <span class="chip idle" id="chipOnline">clear</span>
        </div>
        <dl class="read">
          <dt>Obstacle clearance</dt><dd id="rOnClear">–</dd>
          <dt>Self clearance</dt><dd id="rOnSelf">–</dd>
        </dl>
        <div class="runrow" id="blindRow">
          <span class="runname"><span class="swatch" style="background:var(--blind)"></span><span id="nmBlind">Blind</span></span>
          <span class="chip idle" id="chipBlind">clear</span>
        </div>
        <dl class="read" id="blindRead">
          <dt>Obstacle clearance</dt><dd id="rBlClear">–</dd>
        </dl>
      </section>

      <section v-show="!comparison" class="panel" aria-labelledby="h-est">
        <h2 id="h-est">Obstacle estimate (online)</h2>
        <dl class="read">
          <dt>Status</dt><dd id="rSeen">–</dd>
          <dt>Error vs truth</dt><dd id="rErr">–</dd>
          <dt>3σ, largest axis</dt><dd id="rSig">–</dd>
          <dt>Plan step k</dt><dd id="rK">–</dd>
          <dt>Solve time</dt><dd id="rMs">–</dd>
        </dl>
      </section>

      <section class="panel" aria-labelledby="h-view">
        <h2 id="h-view">Show</h2>
        <div v-show="!comparison" class="seg" role="radiogroup" aria-label="Runs">
          <label><input type="radio" name="runs" value="both" id="runsBoth" checked>Both</label>
          <label><input type="radio" name="runs" value="online" id="runsOnline">Online</label>
          <label><input type="radio" name="runs" value="blind" id="runsBlind">Blind</label>
        </div>
        <div class="layers">
          <label><input type="checkbox" id="lyMesh" checked>Arm meshes</label>
          <label><input type="checkbox" id="lySph" checked>Collision spheres</label>
          <label><input type="checkbox" id="lyPlan" checked>Current plan</label>
          <label><input type="checkbox" id="lyOff">Offline plan</label>
          <label><input type="checkbox" id="lyEst" checked>Estimate + 3σ</label>
          <label><input type="checkbox" id="lyRange">Sensor range</label>
          <label><input type="checkbox" id="lyStand" checked>Standoff shell</label>
          <label><input type="checkbox" id="lyTrail" checked>Tool trail</label>
        </div>

      </section>
    </aside>
  </div>

  <section class="transport" aria-label="Playback">
    <div class="controls">
      <button id="play" type="button" aria-label="Play">▶ Play</button>
      <button id="restart" type="button">Restart</button>
      <select id="speed" aria-label="Playback speed">
        <option value="0.5">0.5×</option>
        <option value="1">1×</option>
        <option value="2" selected>2×</option>
        <option value="4">4×</option>
      </select>
      <input id="scrub" type="range" min="0" max="1000" step="1" value="0" aria-label="Time">
      <span class="time" id="timeTxt">0.00 / 0.00 s</span>
    </div>
  </section>

    </section>
    <AppPaper :app="app" :frame="frame" :samples="replay?.num_poses || 1" @select="selectTime($event * replay.dt)">
      <template #mathematics>
        <MathTypeset :html="equations" />
        <div v-if="replay" class="math-parameters" data-math-parameters>
          <table><caption>Settings stored in this replay</caption><tbody>
            <tr><th>Horizon</th><td>{{ replay.num_poses }} states · Δt = {{ replay.dt }} s</td></tr>
            <tr><th>Obstacle radius / standoff</th><td>{{ format(100 * replay.planner_obstacle_radius) }} / {{ format(100 * replay.obstacle_standoff) }} cm</td></tr>
            <tr><th>Self-collision standoff</th><td>{{ format(100 * replay.self_standoff) }} cm</td></tr>
            <tr><th>Online sensor range</th><td>{{ format(runs.find(r => r.name === 'online')?.sensor_range) }} m</td></tr>
            <tr><th>Acceleration limits</th><td>{{ replay.u_limit.join(', ') }} rad/s²</td></tr>
          </tbody></table>
          <p>The recording does not export all factor noise scales, velocity limits, or interpolation-check settings; those are defined in the native example. The displayed values above come from the selected file.</p>
        </div>
      </template>
      <p>Signed clearance uses the true obstacle and robot collision spheres. Negative clearance means penetration. The uncertainty ellipsoid uses the landmark’s marginal covariance; it is not a collision-probability bound.</p>
        <div class="legend" aria-label="Sphere colors">
          <div><i style="background:linear-gradient(90deg, var(--online) 50%, var(--blind) 50%)"></i>Sphere clear of the standoff (run color)</div>
          <div><i style="background:var(--serious)"></i>Inside the standoff (or self standoff)</div>
          <div><i style="background:var(--critical)"></i>Penetrating the obstacle or another sphere</div>
        </div>
    <div class="chart" id="chart">
      <svg id="chartSvg" role="img" aria-label="Obstacle clearance over time for each run"></svg>
      <div class="tip" id="tip" hidden></div>
    </div>
    <p class="note" id="note">Clearance is the gap between the closest collision sphere and the true obstacle surface. Click or drag on the chart to scrub.</p>

      <div v-if="replay" class="table-scroll run-summary">
        <table class="result-table"><caption>Recorded execution and solve summaries</caption>
          <thead><tr><th>Run</th><th>Minimum obstacle gap</th><th>Minimum self gap</th><th>First detection</th><th>Final joint error</th><th>Peak solve time</th><template v-if="hasBase"><th>RMS base torque</th><th>Peak base torque</th><th>∫‖τ‖ dt</th></template></tr></thead>
          <tbody><tr v-for="run in runs" :key="run.name"><th>{{ run.label }}</th>
            <td>{{ format(100 * run.summary.min_obstacle_clearance) }} cm</td>
            <td>{{ format(100 * run.summary.min_self_clearance) }} cm</td>
            <td>{{ run.summary.first_detection < 0 ? 'Never' : `${run.summary.first_detection * replay.dt} s` }}</td>
            <td>{{ format(run.summary.goal_error, 4) }} rad</td><td>{{ format(run.summary.max_solve_ms) }} ms</td>
            <template v-if="hasBase"><td>{{ format(baseSummary(run).rms) }} N·m</td><td>{{ format(baseSummary(run).peak) }} N·m</td><td>{{ format(baseSummary(run).integral) }} N·m·s</td></template>
          </tr></tbody>
        </table>
        <p>Final joint error is the maximum absolute joint-angle difference between the executed final configuration and the goal. Per-step solve time sums LM optimization calls, excluding graph rebuilding, covariance extraction, replay generation, and browser rendering. These are measurements from the recorded run.</p>
      </div>
      <template v-if="replay">
        <label>Joint <select v-model.number="joint" aria-label="Joint for result plots"><option v-for="(_, i) in replay.q_start" :value="i">q{{ i + 1 }}</option></select></label>
        <div class="results-grid">
          <SignalPlot title="Executed joint position" unit="rad" :times="times" :series="stateSeries('q')" :frame="displayFrame" x-title="Time (s)" @select="selectTime(times[$event])" />
          <SignalPlot title="Executed joint velocity" unit="rad/s" :times="times" :series="stateSeries('v')" :frame="displayFrame" x-title="Time (s)" @select="selectTime(times[$event])" />
          <SignalPlot title="Executed acceleration control" unit="rad/s²" :times="controlTimes" :series="controls" :frame="Math.min(frame, controlTimes.length - 1)" x-title="Interval start (s)" @select="selectTime(controlTimes[$event])" />
          <SignalPlot title="Native optimization time per step" unit="ms" :times="stepTimes" :series="stepSeries('solve_ms')" :frame="frame" x-title="Time (s)" @select="selectTime(stepTimes[$event])" />
          <SignalPlot title="Weighted graph error after each solve" unit="½‖r‖²" :times="stepTimes" :series="stepSeries('graph_error')" :frame="frame" x-title="Time (s)" @select="selectTime(stepTimes[$event])" />
          <SignalPlot title="Active factors per step" unit="factors" :times="stepTimes" :series="stepSeries('factors')" :frame="frame" x-title="Time (s)" @select="selectTime(stepTimes[$event])" />
          <template v-if="hasBase">
            <SignalPlot title="Base reaction torque magnitude" unit="N·m" :times="times" :series="baseSeries(torque)" :frame="displayFrame" x-title="Time (s)" @select="selectTime(times[$event])" />
            <SignalPlot title="Base reaction force magnitude" unit="N" :times="times" :series="baseSeries(force)" :frame="displayFrame" x-title="Time (s)" @select="selectTime(times[$event])" />
          </template>
        </div>
        <p v-if="hasBase">The base reaction is the gravity-free wrench the moving arm applies to its mount, in the base frame about its origin: the motion-induced disturbance an attitude controller would cancel. It is computed from each frame's executed joint positions, velocities and acceleration controls. RMS, peak and integral use these frames.</p>
        <p>The graph changes with observations and executed intervals, so its error is not a fixed-objective convergence curve. Controls are constant over each interval; points mark their start times. All plots select playback time.</p>
        <details><summary>Recording provenance</summary><p class="note">{{ replay.source }}</p></details>
        <p><a :href="downloadUrl" download="replay.json">Download the displayed replay</a></p>
      </template>
      <template #discussion>
        <p>This example is informed by Matthew King-Smith, Panagiotis Tsiotras, and Frank Dellaert’s
          <a href="https://arxiv.org/abs/2204.13251" target="_blank" rel="noopener">Simultaneous Control and Trajectory Estimation for Collision Avoidance of Autonomous Robotic Spacecraft Systems</a> (2022).
          That work jointly estimates states, controls, and obstacle locations, and studies reactive and predictive navigation for spacecraft. The {{ robot }} assumptions above describe this implementation’s adaptation.</p>
        <p>Link poses come from the {{ robot }} model in GTDynamics; the collision spheres are the planner's geometry. Link positions and xyzw quaternions are saved in the robot world frame, with metres and a Z-up convention. The recorded runs and their original diagnostics remain downloadable.</p>
      </template>
    </AppPaper>
  </main>
</template>
