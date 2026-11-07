<script setup>
import { computed, onMounted, onBeforeUnmount, ref, shallowRef } from 'vue';
import AppAbstract from '../../../shared/paper/AppAbstract.vue';
import AppPaper from '../../../shared/paper/AppPaper.vue';
import MathTypeset from '../../../shared/paper/MathTypeset.vue';
import SignalPlot from '../../../shared/dynamics/SignalPlot.vue';
import equations from './scate.math.html?math';
import { createReplayViewer } from './viewer.js';

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
const color = run => run.name === 'online' ? '#3987e5' : '#eb6834';
const stateSeries = field => runs.value.map(run => ({ name: run.label, color: color(run), values: run.frames.map(f => f[field][joint.value]) }));
const controls = computed(() => runs.value.map(run => ({name: run.label, color: color(run), values: run.u.map(u => u[joint.value])})));
const stepSeries = field => runs.value.map(run => ({name: run.label, color: color(run), values: run.steps.map(step => step[field])}));
function selectTime(t) { viewer?.selectTime(t); }
function receiveReplay(data) {
  replay.value = data;
  joint.value = 0;
  if (download) URL.revokeObjectURL(download);
  download = URL.createObjectURL(new Blob([JSON.stringify(data)], {type: 'application/json'}));
  downloadUrl.value = download;
}
onMounted(() => {
  viewer = createReplayViewer(root.value, {onReplay: receiveReplay, onTime: t => {time.value = t;}});
});
onBeforeUnmount(() => { viewer?.dispose(); if (download) URL.revokeObjectURL(download); });
</script>

<template>
  <main ref="root" class="scate-app" :data-frame="frame">
    <header class="page-header">
      <div><h1>SCATE UR5</h1><AppAbstract app="scate/ur5" /></div>
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
      <section class="panel" aria-labelledby="h-state">
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

      <section class="panel" aria-labelledby="h-est">
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
        <div class="seg" role="radiogroup" aria-label="Runs">
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
    <AppPaper app="scate/ur5" :frame="frame" :samples="replay?.num_poses || 1" @select="selectTime($event * replay.dt)">
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
          <thead><tr><th>Run</th><th>Minimum obstacle gap</th><th>Minimum self gap</th><th>First detection</th><th>Final joint error</th><th>Peak solve time</th></tr></thead>
          <tbody><tr v-for="run in runs" :key="run.name"><th>{{ run.label }}</th>
            <td>{{ format(100 * run.summary.min_obstacle_clearance) }} cm</td>
            <td>{{ format(100 * run.summary.min_self_clearance) }} cm</td>
            <td>{{ run.summary.first_detection < 0 ? 'Never' : `${run.summary.first_detection * replay.dt} s` }}</td>
            <td>{{ format(run.summary.goal_error, 4) }} rad</td><td>{{ format(run.summary.max_solve_ms) }} ms</td>
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
        </div>
        <p>The graph changes with observations and executed intervals, so its error is not a fixed-objective convergence curve. Controls are constant over each interval; points mark their start times. All plots select playback time.</p>
        <details><summary>Recording provenance</summary><p class="note">{{ replay.source }}</p></details>
        <p><a :href="downloadUrl" download="replay.json">Download the displayed replay</a></p>
      </template>
      <template #discussion>
        <p>This example is informed by Matthew King-Smith, Panagiotis Tsiotras, and Frank Dellaert’s
          <a href="https://arxiv.org/abs/2204.13251" target="_blank" rel="noopener">Simultaneous Control and Trajectory Estimation for Collision Avoidance of Autonomous Robotic Spacecraft Systems</a> (2022).
          That work jointly estimates states, controls, and obstacle locations, and studies reactive and predictive navigation for spacecraft. The UR5 assumptions above describe this implementation’s adaptation.</p>
        <p>Robot geometry comes from the UR5 collision meshes in GTDynamics. Link positions and xyzw quaternions are saved in the robot world frame, with metres and a Z-up convention. The online and blind replays and their original diagnostics remain downloadable.</p>
      </template>
    </AppPaper>
  </main>
</template>
