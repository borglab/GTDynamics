<template>
  <div class="demo-root">
    <div id="mujoco-container" class="mujoco-container"></div>

    <div class="panel">
      <h1>GT Dynamics Clip Playback</h1>

      <label>
        Robot
        <select :value="selectedRobot" @change="onRobotChange">
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">
            {{ robot.label }}
          </option>
        </select>
      </label>

      <label>
        Motion
        <select :value="selectedMotion" @change="onMotionChange" :disabled="motions.length === 0 || loadingMotion">
          <option v-for="motion in motions" :key="motion.name" :value="motion.name">
            {{ motion.name }}
          </option>
        </select>
      </label>

      <div class="row">
        <button type="button" @click="togglePlay" :disabled="!ready">
          {{ isPlaying ? 'Pause' : 'Play' }}
        </button>
        <label class="loop-toggle">
          <input type="checkbox" v-model="loopEnabled" @change="onLoopChange" />
          Loop
        </label>
      </div>

      <label>
        Time
        <input
          type="range"
          min="0"
          :max="duration"
          step="0.001"
          :value="currentTime"
          @input="onScrub"
          :disabled="duration <= 0"
        />
      </label>
      <div class="meta">{{ currentTime.toFixed(2) }}s / {{ duration.toFixed(2) }}s</div>

      <div class="meta">Frame: {{ currentFrame }}</div>
      <div class="meta">Sim: {{ simHz.toFixed(1) }} Hz</div>
      <div v-if="statusMessage" class="status">{{ statusMessage }}</div>
      <div v-if="errorMessage" class="error">{{ errorMessage }}</div>
    </div>

    <div v-if="loading" class="overlay">Loading MuJoCo WebAssembly and local assets…</div>
  </div>
</template>

<script setup>
import { onBeforeUnmount, onMounted, ref } from 'vue';
import loadMujoco from 'mujoco-js';
import { MuJoCoClipDemo } from '@/simulation/main.js';

const robots = [
  {
    id: 'g1',
    label: 'G1',
    motionIndex: './examples/motions/g1/motions.json'
  },
  {
    id: 'a1',
    label: 'A1',
    motionIndex: './examples/motions/a1/motions.json'
  },
  {
    id: 'spot',
    label: 'Spot',
    motionIndex: './examples/motions/spot/motions.json'
  }
];

const loading = ref(true);
const loadingMotion = ref(false);
const ready = ref(false);
const selectedRobot = ref(robots[0].id);
const motions = ref([]);
const selectedMotion = ref('');
const isPlaying = ref(false);
const loopEnabled = ref(true);
const duration = ref(0);
const currentTime = ref(0);
const currentFrame = ref(0);
const simHz = ref(0);
const statusMessage = ref('');
const errorMessage = ref('');

let demo = null;
let uiTimer = null;

function parseMotionEntries(indexPayload, indexUrl) {
  if (Array.isArray(indexPayload)) {
    const baseUrl = new URL('.', indexUrl);
    return indexPayload.map((entry) => {
      const file = typeof entry === 'string' ? entry : entry?.file;
      const name = typeof entry === 'string' ? entry.replace(/\.json$/i, '') : (entry?.name ?? 'motion');
      return {
        name,
        clipName: entry?.clip ?? 'default',
        url: new URL(file, baseUrl).toString()
      };
    });
  }

  if (indexPayload && typeof indexPayload === 'object') {
    if (Array.isArray(indexPayload.motions)) {
      const basePath = typeof indexPayload.base_path === 'string' ? indexPayload.base_path : '.';
      const baseUrl = new URL(basePath.endsWith('/') ? basePath : `${basePath}/`, indexUrl);
      return indexPayload.motions.map((entry) => {
        const file = typeof entry === 'string' ? entry : (entry.file ?? entry.path);
        const name = typeof entry === 'string' ? entry.replace(/\.json$/i, '') : (entry.name ?? 'motion');
        const clipName = typeof entry === 'object' && entry !== null ? (entry.clip ?? 'default') : 'default';
        return {
          name,
          clipName,
          url: new URL(file, baseUrl).toString()
        };
      });
    }

    const pairs = Object.entries(indexPayload);
    if (pairs.length > 0 && typeof pairs[0][1] === 'string') {
      const baseUrl = new URL('.', indexUrl);
      return pairs.map(([name, file]) => ({
        name,
        clipName: 'default',
        url: new URL(file, baseUrl).toString()
      }));
    }
  }

  throw new Error('Unsupported motions index format');
}

async function loadMotionIndex(robotId) {
  const robot = robots.find((entry) => entry.id === robotId);
  if (!robot) {
    throw new Error(`Robot '${robotId}' is not configured in the UI`);
  }

  const motionIndexUrl = new URL(robot.motionIndex, window.location.href).toString();
  const response = await fetch(motionIndexUrl);
  if (!response.ok) {
    throw new Error(`Failed to load motions index '${robot.motionIndex}': ${response.status}`);
  }

  const payload = await response.json();
  return parseMotionEntries(payload, motionIndexUrl);
}

function startUiPolling() {
  uiTimer = window.setInterval(() => {
    if (!demo) {
      return;
    }
    duration.value = demo.getDuration();
    currentTime.value = demo.getCurrentTime();
    currentFrame.value = demo.getCurrentFrame();
    simHz.value = demo.getSimStepHz();
  }, 100);
}

async function loadMotionByName(motionName) {
  if (!demo) {
    return;
  }

  const entry = motions.value.find((motion) => motion.name === motionName);
  if (!entry) {
    return;
  }

  loadingMotion.value = true;
  errorMessage.value = '';
  try {
    await demo.loadClipFromUrl(entry.url, entry.clipName ?? 'default');
    demo.setLoop(loopEnabled.value);
    demo.setTime(0, true);
    demo.setPlaying(true);
    isPlaying.value = true;
    statusMessage.value = `Loaded clip '${entry.name}'`;
  } catch (error) {
    errorMessage.value = String(error);
    statusMessage.value = '';
    demo.setPlaying(false);
    isPlaying.value = false;
  } finally {
    loadingMotion.value = false;
  }
}

async function loadRobot(robotId) {
  if (!demo) {
    return;
  }

  loading.value = true;
  errorMessage.value = '';
  statusMessage.value = '';
  try {
    await demo.loadRobot(robotId);
    motions.value = await loadMotionIndex(robotId);
    selectedMotion.value = motions.value.length > 0 ? motions.value[0].name : '';
    if (selectedMotion.value) {
      await loadMotionByName(selectedMotion.value);
    }
    ready.value = true;
  } catch (error) {
    errorMessage.value = String(error);
    ready.value = false;
  } finally {
    loading.value = false;
  }
}

async function onRobotChange(event) {
  const robotId = event.target.value;
  selectedRobot.value = robotId;
  await loadRobot(robotId);
}

async function onMotionChange(event) {
  const motionName = event.target.value;
  selectedMotion.value = motionName;
  await loadMotionByName(motionName);
}

function togglePlay() {
  if (!demo) {
    return;
  }
  isPlaying.value = !isPlaying.value;
  demo.setPlaying(isPlaying.value);
}

function onLoopChange() {
  if (!demo) {
    return;
  }
  demo.setLoop(loopEnabled.value);
}

function onScrub(event) {
  if (!demo) {
    return;
  }
  const value = Number(event.target.value);
  currentTime.value = Number.isFinite(value) ? value : 0;
  demo.setTime(currentTime.value, true);
}

onMounted(async () => {
  try {
    const mujoco = await loadMujoco();
    demo = new MuJoCoClipDemo(mujoco);
    await demo.init(selectedRobot.value);
    startUiPolling();
    motions.value = await loadMotionIndex(selectedRobot.value);
    selectedMotion.value = motions.value.length > 0 ? motions.value[0].name : '';
    if (selectedMotion.value) {
      await loadMotionByName(selectedMotion.value);
    }
    ready.value = true;
  } catch (error) {
    errorMessage.value = String(error);
  } finally {
    loading.value = false;
  }
});

onBeforeUnmount(() => {
  if (uiTimer) {
    window.clearInterval(uiTimer);
    uiTimer = null;
  }
  if (demo) {
    demo.destroy();
    demo = null;
  }
});
</script>

<style scoped>
.demo-root {
  position: relative;
  width: 100vw;
  height: 100vh;
  overflow: hidden;
  background: #0e1116;
  color: #e8eef5;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
}

.mujoco-container {
  width: 100%;
  height: 100%;
}

.panel {
  position: absolute;
  top: 16px;
  left: 16px;
  width: min(360px, calc(100vw - 32px));
  display: flex;
  flex-direction: column;
  gap: 10px;
  padding: 14px;
  background: rgba(10, 14, 18, 0.8);
  border: 1px solid rgba(255, 255, 255, 0.15);
  border-radius: 10px;
  backdrop-filter: blur(6px);
}

h1 {
  margin: 0;
  font-size: 16px;
  font-weight: 600;
}

label {
  display: flex;
  flex-direction: column;
  gap: 6px;
  font-size: 12px;
}

select,
button,
input[type='range'] {
  width: 100%;
}

select,
button {
  border-radius: 6px;
  border: 1px solid rgba(255, 255, 255, 0.2);
  background: #151b22;
  color: #f2f6fb;
  padding: 7px 8px;
}

button {
  cursor: pointer;
}

.row {
  display: flex;
  gap: 8px;
}

.row button {
  flex: 1;
}

.loop-toggle {
  flex: 1;
  flex-direction: row;
  align-items: center;
  justify-content: center;
  border: 1px solid rgba(255, 255, 255, 0.2);
  border-radius: 6px;
  padding: 7px 8px;
  background: #151b22;
}

.meta {
  font-size: 12px;
  color: #b8c3d0;
}

.status {
  font-size: 12px;
  color: #9ee493;
}

.error {
  font-size: 12px;
  color: #ffb4b4;
}

.overlay {
  position: absolute;
  inset: 0;
  display: grid;
  place-items: center;
  background: rgba(0, 0, 0, 0.6);
  font-size: 16px;
}

@media (max-width: 760px) {
  .panel {
    top: auto;
    left: 8px;
    right: 8px;
    bottom: 8px;
    width: auto;
  }
}
</style>
