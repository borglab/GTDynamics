<script setup>
/**
 * Hardware experiment of the SCATE UR10e Base app. The page talks to the
 * executor (examples/example_scate_ur10e/hardware) through the dev server
 * (liveServer.mjs, api/scate-hw/*): on a laptop the dev server starts it with
 * a simulated arm; in the lab it is the ROS 2 node on ur_bridge.
 *
 * The flow: solve the setup first (the live solve: its runs are the ghost
 * runs), pick the run to execute, and begin. The executor simulates the same
 * run again, refuses unless it is safe, and needs the arm at the start pose
 * (it can move it there). During the run it re-plans at every state while
 * the arm moves; this panel shows its progress and draws the measured arm,
 * and afterwards loads the hardware run beside its ghost run.
 */
import { computed, onBeforeUnmount, ref, watch } from 'vue';
import { linkTransforms, transformPoint } from './kinematics.js';

const props = defineProps({
  viewer: { type: Object, default: null },
  /** The robot description (scate_live describe). */
  description: { type: Object, default: null },
  /** The page's current request body, without hardware_sigma. */
  request: { type: Object, default: null },
  /** The last live solve: {replay, key} (key = JSON of the request it solved). */
  ghost: { type: Object, default: null },
  /** Show the measured arm: monitor({q, q_goal, obstacles, path}) or monitor(null). */
  monitor: { type: Function, default: () => {} },
});
const emit = defineEmits(['busy']);

const api = path => new URL(`api/scate-hw/${path}`, document.baseURI);
const status = ref(null);
const error = ref('');
const note = ref('');
const chosen = ref('');
const acting = ref(false);
let timer = null, preparedKey = null;
let awaitingResult = false;  // a run started from this page: load its replay when it ends

const BUSY = ['preparing', 'moving', 'running', 'finishing'];
const busy = computed(() => acting.value || BUSY.includes(status.value?.state));
watch(busy, value => emit('busy', value), { immediate: true });

/** Base-cost sigma of a replay run, from its name (scate_reactive_scenario.h baseWrenchRunName). */
function runSigma(name) {
  if (name === 'cost_off') return 0;
  if (name === 'online') return 3;
  const match = /^sigma_(.+)$/.exec(name);
  return match ? Number(match[1]) : null;
}
const requestKey = computed(() => JSON.stringify(props.request));
const fresh = computed(() => !!props.ghost && props.ghost.key === requestKey.value);
const runs = computed(() => (fresh.value ? props.ghost.replay.runs.filter(run => runSigma(run.name) !== null) : []));
watch(runs, list => {
  if (!list.some(run => run.name === chosen.value)) {
    chosen.value = (list.find(run => run.summary?.safety?.safe) || list[0])?.name || '';
  }
}, { immediate: true });
const run = computed(() => runs.value.find(r => r.name === chosen.value) || null);
const safety = computed(() => run.value?.summary?.safety || null);
const cm = metres => (metres === null || metres === undefined ? '–' : `${(100 * metres).toFixed(1)} cm`);
const percent = ratio => `${Math.round(100 * ratio)}%`;

const executorLabel = computed(() => {
  const s = status.value;
  if (!s) return 'Looking for the executor…';
  if (!s.connected) return s.hint;
  const arm = s.robot.kind === 'sim' ? 'simulated arm' : s.robot.kind === 'ros' ? 'UR10e through ur_bridge' : s.robot.kind;
  return `Executor: ${arm} · arm ${s.robot.status === 'ready' ? 'ready' : s.robot.status}`;
});

const blocked = computed(() => {
  const s = status.value;
  if (!s?.connected) return 'No executor.';
  if (!props.ghost) return 'Solve and run first: the hardware run begins only after its ghost run.';
  if (!fresh.value) return 'The setup changed since the last solve: solve and run again to see its ghost run.';
  if (!run.value) return 'Choose the run to execute.';
  if (!safety.value?.safe) return 'This ghost run is not safe, so it cannot run on hardware.';
  if (s.robot.status !== 'ready') return `The arm is not ready: ${s.robot.status}`;
  if (busy.value) return '';
  return '';
});
const canBegin = computed(() => !blocked.value && !busy.value);
const needsMove = computed(() => status.value?.state === 'ready' && status.value?.ghost && !status.value?.at_start);
const running = computed(() => ['running', 'moving'].includes(status.value?.state));

async function post(path, body = {}) {
  const response = await fetch(api(path), {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok || data.ok === false) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

async function refresh() {
  try {
    status.value = await fetch(api('status')).then(response => response.json());
  } catch {
    status.value = { connected: false, hint: 'Hardware runs need the dev server (npm run dev in apps/).' };
  }
  const s = status.value;
  if (s.connected && ['running', 'moving'].includes(s.state) && s.robot.q) showArm(s);
  if (awaitingResult && s.connected && ['done', 'aborted', 'error'].includes(s.state)) {
    awaitingResult = false;
    if (s.replay_ready) await loadResult();
    else props.monitor(null);
  }
}
function schedule() {
  clearTimeout(timer);
  const fast = running.value || busy.value;
  timer = setTimeout(async () => { await refresh(); schedule(); }, fast ? 150 : 800);
}

/** Draw the measured arm, the obstacles now and the current plan's tool path. */
function showArm(s) {
  const robot = props.description;
  if (!robot) return;
  const tool = robot.spheres[robot.tool_sphere];
  const toolAt = q => transformPoint(linkTransforms(robot, q)[tool.link], tool.center);
  const obstacles = (props.request?.obstacles || []).map((o, j) => {
    const now = s.run?.obstacles?.[j] || o.start;
    const end = o.start.map((x, i) => x + o.velocity[i] * robot.duration);
    return { start: now, end, radius: o.radius };
  });
  props.monitor({
    q: s.robot.q, q_goal: props.request.q_goal, obstacles,
    path: s.state === 'running' && s.run?.plan_q ? s.run.plan_q.map(toolAt) : null,
  });
}

async function loadResult() {
  try {
    const data = await fetch(api('replay')).then(response => {
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return response.json();
    });
    props.monitor(null);
    await props.viewer?.loadReplay(data, status.value.state === 'done' ? 'Hardware run' : 'Hardware run (stopped)');
  } catch (err) {
    error.value = `Could not load the hardware run: ${err.message}`;
  }
}

async function waitWhile(states, seconds) {
  const end = Date.now() + 1000 * seconds;
  while (Date.now() < end) {
    await refresh();
    if (!states.includes(status.value?.state)) return status.value;
    await new Promise(resolve => setTimeout(resolve, 200));
  }
  throw new Error('the executor did not answer in time');
}

/** Begin: load the request into the executor (its own ghost check), then start if the arm is at the start. */
async function begin() {
  error.value = ''; note.value = '';
  acting.value = true;
  try {
    const sigma = runSigma(run.value.name);
    const body = { ...props.request, hardware_sigma: sigma };
    const key = JSON.stringify(body);
    // The executor keeps a prepared request until it runs; prepare again if it changed.
    if (status.value.state !== 'ready' || preparedKey !== key) {
      note.value = 'The executor is checking the ghost run…';
      preparedKey = null;
      await post('prepare', body);
      const s = await waitWhile(['preparing'], 180);
      if (s.state !== 'ready') throw new Error(s.message);
      preparedKey = key;
    }
    if (!status.value.at_start) {
      note.value = `The arm is ${status.value.start_distance.toFixed(3)} rad from the start pose. Move it there, then begin.`;
      return;
    }
    note.value = '';
    await post('begin');
    awaitingResult = true;
    await refresh();
  } catch (err) {
    error.value = err.message;
  } finally {
    acting.value = false;
    schedule();
  }
}

async function moveToStart() {
  error.value = ''; note.value = 'Moving the arm to the start pose…';
  try {
    await post('move_to_start');
    schedule();
    const s = await waitWhile(['moving'], 300);
    note.value = s.at_start ? 'At the start pose. Press Begin hardware experiment to run.' : s.message;
    props.monitor(null);
  } catch (err) {
    error.value = err.message;
  }
}

async function stop() {
  try { await post('abort', { reason: 'stopped from the app' }); } catch (err) { error.value = err.message; }
}

const progress = computed(() => {
  const r = status.value?.run;
  return r?.duration ? Math.min(1, r.t / r.duration) : 0;
});

refresh().then(schedule);
onBeforeUnmount(() => { clearTimeout(timer); props.monitor(null); });
</script>

<template>
  <section class="hardware" data-hardware-panel :data-state="status?.state || 'none'">
    <h3>Hardware experiment</h3>
    <p class="live-hint" data-executor>{{ executorLabel }}</p>
    <template v-if="status?.connected">
      <div v-if="runs.length" class="hw-row">
        <label for="hw-run">Run on hardware</label>
        <select id="hw-run" v-model="chosen" :disabled="busy">
          <option v-for="r in runs" :key="r.name" :value="r.name">{{ r.label }}{{ r.summary?.safety?.safe ? '' : ' (not safe)' }}</option>
        </select>
      </div>
      <p v-if="safety" :class="safety.safe ? 'hw-safe' : 'live-warning'" data-ghost-safety>
        <template v-if="safety.safe">Ghost run safe: walls and floor {{ cm(run.summary.min_environment_clearance) }},
          obstacles {{ cm(run.summary.min_obstacle_clearance) }}, speed {{ percent(safety.max_velocity_ratio) }} and
          acceleration {{ percent(safety.max_acceleration_ratio) }} of the limits.</template>
        <template v-else>Ghost run not safe: {{ safety.reasons.join('; ') }}.</template>
      </p>
      <p v-if="blocked && !busy" class="live-hint" data-hw-blocked>{{ blocked }}</p>
      <div class="live-buttons">
        <button type="button" class="primary" :disabled="!canBegin" data-begin @click="begin">Begin hardware experiment</button>
        <button v-if="needsMove" type="button" :disabled="busy" data-move @click="moveToStart">Move arm to start</button>
        <button v-if="running" type="button" class="danger" data-stop @click="stop">Stop arm</button>
      </div>
      <div v-if="status.state === 'running'" class="live-progress" role="status" data-hw-progress>
        <div class="bar"><div :style="{ width: `${100 * progress}%` }"></div></div>
        <span>t = {{ status.run.t?.toFixed(1) }} of {{ status.run.duration }} s · state {{ status.run.k }} ·
          tracking {{ (status.run.tracking_error ?? 0).toFixed(3) }} rad ·
          re-plan {{ status.run.last_solve_ms?.toFixed(0) ?? '–' }} ms</span>
      </div>
      <p v-if="note" class="live-hint" role="status">{{ note }}</p>
      <p v-if="status.state === 'preparing' || status.state === 'moving' || status.state === 'finishing'" class="live-hint">{{ status.message }}</p>
      <p v-if="['done', 'aborted', 'error', 'unsafe'].includes(status.state)" :class="status.state === 'done' ? 'hw-safe' : 'live-warning'" data-hw-result>
        {{ status.message }}
        <template v-if="status.result?.summary"> Goal error {{ status.result.summary.goal_error.toFixed(3) }} rad,
          walls and floor {{ cm(status.result.summary.min_environment_clearance) }}, obstacles {{ cm(status.result.summary.min_obstacle_clearance) }}.</template>
      </p>
    </template>
    <p v-if="error" class="live-warning" role="alert">{{ error }}</p>
  </section>
</template>

<style scoped>
.hardware { border-top: 1px solid var(--grid); margin-top: 10px; padding-top: 8px; }
.hardware h3 { font-size: 12px; letter-spacing: .06em; text-transform: uppercase; margin: 0 0 4px; }
.hw-row { display: flex; gap: 8px; align-items: center; font-size: 12.5px; margin: 4px 0; }
.hw-row select { font: inherit; font-size: 12.5px; }
.hw-safe { font-size: 12px; color: var(--online); margin: 4px 0; }
.live-hint { font-size: 12px; color: var(--muted); margin: 6px 0; }
.live-warning { font-size: 12px; color: var(--critical); margin: 4px 0; }
.live-buttons { display: flex; flex-wrap: wrap; gap: 6px; margin: 6px 0; }
.live-buttons button { font: inherit; font-size: 12.5px; padding: 5px 10px; border: 1px solid var(--grid); border-radius: 6px; background: var(--surface); color: var(--ink); cursor: pointer; }
button.primary { background: var(--online); border-color: var(--online); color: #fff; font-weight: 600; }
button.danger { background: var(--critical); border-color: var(--critical); color: #fff; font-weight: 600; }
button:disabled { opacity: .5; cursor: not-allowed; }
.live-progress { font-size: 12px; color: var(--ink-2); margin-top: 6px; }
.live-progress .bar { height: 4px; background: var(--grid); border-radius: 2px; overflow: hidden; margin-bottom: 4px; }
.live-progress .bar div { height: 100%; background: var(--online); transition: width .2s; }
</style>
