<script setup>
/**
 * Live simulation controls of the SCATE UR10e Base app: pick the robot, drag
 * a start and goal pose and the moving obstacles (editor.js), then solve with
 * the native scate_live through the dev server (liveServer.mjs) and play the
 * result. Without edits, a solve uses the recorded experiment's setup.
 */
import { computed, onBeforeUnmount, ref, shallowRef, watch } from 'vue';
import { createSetupEditor, EDITOR_COLORS } from './editor.js';
import { armClearance, clampJoints, linkTransforms, transformPoint } from './kinematics.js';

const props = defineProps({
  /** The page's replay viewer (createReplayViewer). */
  viewer: { type: Object, default: null },
  /** The robot the page opens with: "ur10e" or "ur5". */
  defaultRobot: { type: String, default: 'ur10e' },
});
const emit = defineEmits(['editing', 'solved']);

const ROBOTS = [{ key: 'ur10e', name: 'UR10e' }, { key: 'ur5', name: 'UR5' }];
const RUNS = [{ sigma: 0, label: 'Base cost off' }, { sigma: 3, label: 'σ = 3' }, { sigma: 0.3, label: 'σ = 0.3' }, { sigma: 0.1, label: 'σ = 0.1' }];
const api = path => new URL(`api/scate-live/${path}`, document.baseURI);

const robotKey = ref(props.defaultRobot);
const description = shallowRef(null);
const setup = shallowRef(null);         // edited in place by the editor; revision re-renders
const revision = ref(0);
const mode = ref(null);                 // null, 'pose' or 'obstacles'
const activePose = ref('start');
const showSpheres = ref(false);
const edited = ref({ poses: false, obstacles: false });
const selectedSigmas = ref(RUNS.map(run => run.sigma));
const server = ref({ checked: false, available: false, hint: '' });
const job = ref(null);                  // {state, done, total, label, error, elapsed}
const error = ref('');
const descriptions = {};
let editor = null, polling = null;

const solving = computed(() => job.value?.state === 'running');
const degrees = radians => radians * 180 / Math.PI;

/** The recorded experiment of a robot as an editable setup. */
function defaultSetup(robot) {
  const d = robot.defaults;
  return {
    q_start: d.q_start.slice(), q_goal: d.q_goal.slice(),
    obstacles: d.obstacles.map(o => ({ start: o.start.slice(), end: o.end.slice(), radius: o.radius })),
  };
}

async function loadRobot(key) {
  error.value = '';
  try {
    descriptions[key] ??= await fetch(new URL(`robots/${key}.json`, document.baseURI)).then(response => {
      if (!response.ok) throw new Error(`robots/${key}.json: HTTP ${response.status}`);
      return response.json();
    });
    description.value = descriptions[key];
    setup.value = defaultSetup(description.value);
    edited.value = { poses: false, obstacles: false };
    if (editor && props.viewer) {
      const meshes = await props.viewer.editorContext().meshes(key);
      editor.setRobot(description.value, meshes);
      editor.setSetup(setup.value);
    }
    revision.value++;
  } catch (err) {
    error.value = `Could not load the ${key.toUpperCase()} model: ${err.message}`;
  }
}

async function ensureEditor() {
  if (editor || !props.viewer) return;
  editor = createSetupEditor(props.viewer.editorContext(), {
    onChange: () => {
      if (mode.value === 'pose') edited.value.poses = true;
      if (mode.value === 'obstacles') edited.value.obstacles = true;
      revision.value++;
    },
  });
  if (description.value) {
    editor.setRobot(description.value, await props.viewer.editorContext().meshes(robotKey.value));
    editor.setSetup(setup.value);
  }
}

async function setMode(next) {
  await ensureEditor();
  mode.value = mode.value === next ? null : next;
  props.viewer?.setEditing(!!mode.value);
  editor?.setShowSpheres(showSpheres.value);
  editor?.setActivePose(activePose.value);
  editor?.setMode(mode.value);
  emit('editing', !!mode.value);
}
function done() { if (mode.value) setMode(mode.value); }

async function chooseRobot(key) {
  if (key === robotKey.value || solving.value) return;
  robotKey.value = key;
  await loadRobot(key);
  if (!mode.value) await setMode('pose');
}

function choosePose(key) { activePose.value = key; editor?.setActivePose(key); }
watch(showSpheres, show => editor?.setShowSpheres(show));

// Joint sliders of the active pose, in degrees.
const poseKey = computed(() => activePose.value === 'start' ? 'q_start' : 'q_goal');
const jointRows = computed(() => {
  revision.value;
  if (!description.value || !setup.value) return [];
  return description.value.joints.map((joint, j) => ({
    name: joint.name.replace(/_joint$/, '').replaceAll('_', ' '),
    lower: degrees(joint.lower), upper: degrees(joint.upper), value: degrees(setup.value[poseKey.value][j]),
  }));
});
function setJoint(j, value) {
  const q = setup.value[poseKey.value].slice();
  q[j] = Number(value) * Math.PI / 180;
  setup.value[poseKey.value] = clampJoints(description.value, q);
  edited.value.poses = true;
  editor?.refresh();
  revision.value++;
}
function resetPoses() {
  const d = description.value.defaults;
  setup.value.q_start = d.q_start.slice(); setup.value.q_goal = d.q_goal.slice();
  edited.value.poses = false;
  editor?.refresh(); revision.value++;
}

// Obstacles: radius sliders, speeds, add and remove.
const obstacleRows = computed(() => {
  revision.value;
  if (!setup.value) return [];
  const duration = description.value.duration;
  return setup.value.obstacles.map(o => ({
    radius: 100 * o.radius,
    speed: 100 * Math.hypot(...o.end.map((v, i) => v - o.start[i])) / duration,
    start: o.start.map(v => v.toFixed(2)).join(', '),
  }));
});
function setRadius(index, centimetres) {
  setup.value.obstacles[index].radius = Number(centimetres) / 100;
  edited.value.obstacles = true;
  editor?.refresh(); revision.value++;
}
function addObstacle() {
  // Above the goal tool point, standing still: drag it into place.
  const robot = description.value, tool = robot.spheres[robot.tool_sphere];
  const point = transformPoint(linkTransforms(robot, setup.value.q_goal)[tool.link], tool.center);
  const start = [point[0], point[1], Math.min(robot.workspace.max[2] - 0.1, point[2] + 0.3)];
  setup.value.obstacles.push({ start, end: start.slice(), radius: 0.08 });
  edited.value.obstacles = true;
  editor?.setSetup(setup.value); editor?.setMode(mode.value); revision.value++;
}
function removeObstacle(index) {
  if (setup.value.obstacles.length <= 1) return;
  setup.value.obstacles.splice(index, 1);
  edited.value.obstacles = true;
  editor?.setSetup(setup.value); editor?.setMode(mode.value); revision.value++;
}
function resetObstacles() {
  setup.value.obstacles = defaultSetup(description.value).obstacles;
  edited.value.obstacles = false;
  editor?.setSetup(setup.value); editor?.setMode(mode.value); revision.value++;
}

// Warnings: the arm already touching an obstacle where it starts or ends.
const warnings = computed(() => {
  revision.value;
  if (!setup.value) return [];
  const robot = description.value, out = [];
  const starts = setup.value.obstacles.map(o => ({ center: o.start, radius: o.radius }));
  const ends = setup.value.obstacles.map(o => ({ center: o.end, radius: o.radius }));
  if (armClearance(robot, setup.value.q_start, starts).gap < 0) out.push('The start pose touches an obstacle at t = 0.');
  if (armClearance(robot, setup.value.q_goal, ends).gap < 0) out.push('The goal pose touches an obstacle where it ends.');
  return out;
});
const summary = computed(() => {
  const parts = [edited.value.poses ? 'your start and goal' : 'the experiment’s start and goal',
    edited.value.obstacles ? 'your obstacles' : 'the experiment’s obstacles'];
  return `Solving uses ${parts.join(' and ')} on the ${description.value?.name || ''}.`;
});

function requestBody() {
  const duration = description.value.duration;
  return {
    robot: robotKey.value, q_start: setup.value.q_start, q_goal: setup.value.q_goal,
    obstacles: setup.value.obstacles.map(o => ({
      start: o.start, velocity: o.end.map((v, i) => (v - o.start[i]) / duration), radius: o.radius,
    })),
    sigmas: RUNS.map(run => run.sigma).filter(sigma => selectedSigmas.value.includes(sigma)),
    seed: 0,
  };
}

async function solve() {
  error.value = '';
  if (!selectedSigmas.value.length) { error.value = 'Choose at least one run.'; return; }
  try {
    const response = await fetch(api('solve'), {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(requestBody()),
    });
    const body = await response.json();
    if (!response.ok) throw new Error(body.error || `HTTP ${response.status}`);
    job.value = { id: body.id, state: 'running', done: 0, total: requestBody().sigmas.length, label: 'starting', elapsed: 0 };
    poll(body.id);
  } catch (err) {
    error.value = `Could not start the solve: ${err.message}`;
  }
}

function poll(id) {
  clearTimeout(polling);
  polling = setTimeout(async () => {
    try {
      const status = await fetch(api(`jobs/${id}`)).then(response => response.json());
      if (job.value?.id !== id) return;
      job.value = { id, ...status };
      if (status.state === 'running') return poll(id);
      if (status.state === 'failed') { error.value = `The solve failed: ${status.error}`; return; }
      const data = await fetch(api(`jobs/${id}/replay`)).then(response => response.json());
      if (mode.value) await setMode(mode.value);  // leave editing to show the result
      await props.viewer.loadReplay(data, 'Live solve');
      emit('solved', data);
    } catch (err) {
      error.value = `Lost the solve: ${err.message}`;
      job.value = null;
    }
  }, 250);
}

async function checkServer() {
  try {
    const response = await fetch(api('status'));
    const type = response.headers.get('content-type') || '';
    if (!response.ok || !type.includes('json')) throw new Error('no endpoint');
    const status = await response.json();
    server.value = { checked: true, available: status.available, hint: status.hint };
  } catch {
    server.value = { checked: true, available: false,
      hint: 'Live solving needs the dev server: run npm run dev in apps/ with scate_live built.' };
  }
}

watch(() => props.viewer, viewer => { if (viewer) loadRobot(robotKey.value); }, { immediate: true });
checkServer();
onBeforeUnmount(() => { clearTimeout(polling); editor?.dispose(); });

const progress = computed(() => job.value ? Math.min(1, job.value.done / Math.max(1, job.value.total)) : 0);
</script>

<template>
  <section class="panel wide live-panel" aria-labelledby="h-live" data-live-panel :data-mode="mode || 'none'">
    <h2 id="h-live">Live simulation</h2>
    <div class="seg live-robot" role="radiogroup" aria-label="Robot">
      <label v-for="r in ROBOTS" :key="r.key">
        <input type="radio" name="live-robot" :value="r.key" :checked="robotKey === r.key" :disabled="solving"
               @change="chooseRobot(r.key)"><span>{{ r.name }}</span>
      </label>
    </div>
    <div class="live-buttons">
      <button type="button" :aria-pressed="mode === 'pose'" :disabled="!description || solving" @click="setMode('pose')">Set start and goal position</button>
      <button type="button" :aria-pressed="mode === 'obstacles'" :disabled="!description || solving" @click="setMode('obstacles')">Set obstacles</button>
    </div>

    <div v-if="mode === 'pose'" class="live-edit" data-pose-editor>
      <div class="seg" role="radiogroup" aria-label="Pose to edit">
        <label><input type="radio" name="live-pose" :checked="activePose === 'start'" @change="choosePose('start')">
          <span><i class="dot" :style="{ background: EDITOR_COLORS.start }"></i>Start</span></label>
        <label><input type="radio" name="live-pose" :checked="activePose === 'goal'" @change="choosePose('goal')">
          <span><i class="dot" :style="{ background: EDITOR_COLORS.goal }"></i>Goal</span></label>
      </div>
      <p class="live-hint">Drag any link of the {{ activePose }} arm to pull it there; the sliders set each joint.</p>
      <div v-for="(row, j) in jointRows" :key="row.name" class="joint-row">
        <label :for="`live-joint-${j}`">{{ row.name }}</label>
        <input :id="`live-joint-${j}`" type="range" :min="row.lower" :max="row.upper" step="0.5" :value="row.value"
               @input="setJoint(j, $event.target.value)">
        <output>{{ row.value.toFixed(0) }}°</output>
      </div>
      <label class="live-check"><input v-model="showSpheres" type="checkbox"> Show collision spheres</label>
      <div class="live-buttons"><button type="button" @click="resetPoses">Reset to experiment</button><button type="button" class="primary" @click="done">Done</button></div>
    </div>

    <div v-if="mode === 'obstacles'" class="live-edit" data-obstacle-editor>
      <p class="live-hint">Drag a red sphere to where it starts and its orange marker to where it is at t = {{ description.duration }} s. Same place = standing still.</p>
      <div v-for="(row, index) in obstacleRows" :key="index" class="obstacle-row" :data-obstacle="index">
        <div class="obstacle-head"><strong>Obstacle {{ index + 1 }}</strong>
          <span>{{ row.speed.toFixed(1) }} cm/s</span>
          <button type="button" class="link" :disabled="obstacleRows.length <= 1" @click="removeObstacle(index)">Remove</button></div>
        <div class="joint-row">
          <label :for="`live-radius-${index}`">radius</label>
          <input :id="`live-radius-${index}`" type="range" min="2" max="25" step="0.5" :value="row.radius" @input="setRadius(index, $event.target.value)">
          <output>{{ row.radius.toFixed(1) }} cm</output>
        </div>
      </div>
      <div class="live-buttons">
        <button type="button" :disabled="obstacleRows.length >= 4" @click="addObstacle">Add obstacle</button>
        <button type="button" @click="resetObstacles">Reset to experiment</button>
        <button type="button" class="primary" @click="done">Done</button>
      </div>
    </div>

    <p v-for="warning in warnings" :key="warning" class="live-warning">{{ warning }}</p>
    <fieldset class="live-runs">
      <legend>Runs to solve</legend>
      <label v-for="run in RUNS" :key="run.sigma"><input v-model="selectedSigmas" type="checkbox" :value="run.sigma" :disabled="solving"> {{ run.label }}</label>
    </fieldset>
    <p class="live-hint">{{ summary }}</p>
    <button type="button" class="primary solve" :disabled="!server.available || solving || !description" data-solve @click="solve">
      {{ solving ? 'Solving…' : 'Solve and run' }}
    </button>
    <div v-if="solving" class="live-progress" role="status">
      <div class="bar"><div :style="{ width: `${100 * progress}%` }"></div></div>
      <span>{{ job.label }} · run {{ Math.min(job.done + 1, job.total) }} of {{ job.total }} · {{ job.elapsed.toFixed(1) }} s</span>
    </div>
    <p v-else-if="job?.state === 'done'" class="live-hint" role="status">Solved in {{ job.elapsed.toFixed(1) }} s; playing the result.</p>
    <p v-if="server.checked && !server.available" class="live-hint" data-live-unavailable>{{ server.hint }}</p>
    <p v-if="error" class="live-warning" role="alert">{{ error }}</p>
  </section>
</template>

<style scoped>
.live-panel h2 { margin-bottom: 8px; }
.live-robot { margin-bottom: 8px; }
.live-buttons { display: flex; flex-wrap: wrap; gap: 6px; margin: 6px 0; }
.live-buttons button, .solve { font: inherit; font-size: 12.5px; padding: 5px 10px; border: 1px solid var(--grid); border-radius: 6px; background: var(--surface); color: var(--ink); cursor: pointer; }
.live-buttons button[aria-pressed="true"] { background: var(--ink); color: var(--surface); border-color: var(--ink); }
button.primary { background: var(--online); border-color: var(--online); color: #fff; }
button:disabled { opacity: .5; cursor: not-allowed; }
button.link { border: 0; background: none; color: var(--online); padding: 0; font-size: 12px; cursor: pointer; }
.solve { width: 100%; margin-top: 4px; font-weight: 600; }
.live-edit { border-top: 1px solid var(--grid); margin-top: 6px; padding-top: 6px; }
.live-hint { font-size: 12px; color: var(--muted); margin: 6px 0; }
.live-warning { font-size: 12px; color: var(--critical); margin: 4px 0; }
.joint-row { display: grid; grid-template-columns: 82px 1fr 52px; align-items: center; gap: 6px; font-size: 12px; }
.joint-row output { font-family: var(--font-data); text-align: right; }
.obstacle-row { padding: 4px 0; border-bottom: 1px solid var(--grid); }
.obstacle-head { display: flex; gap: 8px; align-items: baseline; font-size: 12.5px; }
.obstacle-head span { color: var(--muted); font-family: var(--font-data); margin-left: auto; }
.live-check { display: block; font-size: 12.5px; margin: 4px 0; }
.live-runs { border: 0; padding: 0; margin: 8px 0 0; display: flex; flex-wrap: wrap; gap: 4px 12px; font-size: 12.5px; }
.live-runs legend { font-size: 11.5px; color: var(--muted); padding: 0; margin-bottom: 2px; }
.dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; margin-right: 5px; vertical-align: -1px; }
.live-progress { font-size: 12px; color: var(--ink-2); margin-top: 6px; }
.live-progress .bar { height: 4px; background: var(--grid); border-radius: 2px; overflow: hidden; margin-bottom: 4px; }
.live-progress .bar div { height: 100%; background: var(--online); transition: width .2s; }
</style>
