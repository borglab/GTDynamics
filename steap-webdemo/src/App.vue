<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref, shallowRef, watch } from 'vue'
import { Euler, Quaternion } from 'three'
import { SteapViewer } from './viewer.js'

const canvas = ref(null)
const viewerReady = ref(false)
const connected = ref(false)
const connecting = ref(true)
const modelError = ref('')
const error = ref('')
const notice = ref('Connecting to the C++ optimizer…')
const description = shallowRef(null)
const start = ref(null)
const goal = ref(null)
const selection = ref('start')
const transformMode = ref('translate')
const intermediateStates = ref(10)
const duration = ref(4)
const graphDirty = ref(false)
const graphReady = ref(false)
const building = ref(false)
const solving = ref(false)
const playing = ref(false)
const time = ref(0)
const speed = ref(1)
const ghosts = ref(true)
const liveReplan = ref(true)
const trackingError = ref(null)
const tcpPositionError = ref(null)
const endpointStatus = ref('moving')
const endpointName = ref('goal')
const endpointPositionError = ref(null)
const endpointRotationError = ref(null)
const latest = shallowRef(null)
const history = shallowRef([])
const historyIndex = ref(0)
const followLatest = ref(true)
const replayingIterations = ref(false)
const maximumIterations = ref(40)
const targetPending = ref(false)
let viewer, socket, reconnectTimer, targetTimer, replayTimer
let disposed = false
let initializingViewer = false

const selectedPose = computed(() => selection.value === 'start' ? start.value : goal.value)
const selectedEuler = computed(() => {
  if (!selectedPose.value) return [0, 0, 0]
  const euler = new Euler().setFromQuaternion(new Quaternion(...selectedPose.value.quaternion), 'XYZ')
  return [euler.x, euler.y, euler.z].map(value => value * 180 / Math.PI)
})
// Keep typed values independent of playback's render cadence. Commit on blur
// or Enter, while drag/selection changes refresh the displayed coordinates.
const positionInputs = ref(['', '', ''])
const rotationInputs = ref(['', '', ''])
watch(selectedPose, pose => {
  positionInputs.value = pose ? pose.position.map(value => value.toFixed(3)) : ['', '', '']
  rotationInputs.value = pose ? selectedEuler.value.map(value => value.toFixed(1)) : ['', '', '']
}, { immediate: true })
const shown = computed(() => history.value[historyIndex.value] || latest.value)
const graph = computed(() => latest.value?.graph || {})
const stateCount = computed(() => graph.value.states || latest.value?.states?.length || Number(intermediateStates.value) + 2)
const timestep = computed(() => Number(duration.value) / (Number(intermediateStates.value) + 1))
const actualDuration = computed(() => latest.value?.duration || Number(duration.value))
const canSolve = computed(() => connected.value && graphReady.value && !graphDirty.value && !building.value)
const canPlay = computed(() => viewerReady.value && graphReady.value && !graphDirty.value && !!latest.value?.states?.length)
const graphSlices = computed(() => {
  const count = stateCount.value
  if (count <= 7) return Array.from({ length: count }, (_, index) => index)
  return [0, 1, 2, 'ellipsis', count - 3, count - 2, count - 1]
})
const costPoints = computed(() => {
  const costs = history.value.map(item => Math.log10(Math.max(Number(item.error) || 1e-12, 1e-12)))
  if (costs.length < 2) return ''
  const min = Math.min(...costs), max = Math.max(...costs)
  return costs.map((cost, index) => `${index * 290 / (costs.length - 1) + 5},${43 - (cost - min) / (max - min || 1) * 35}`).join(' ')
})
const costMarker = computed(() => {
  if (history.value.length < 2) return null
  const points = costPoints.value.split(' ')
  const [x, y] = (points[historyIndex.value] || points[points.length - 1]).split(',').map(Number)
  return { x, y }
})
const residual = computed(() => shown.value?.endpoint_error)
const residualLarge = computed(() => {
  const value = residual.value
  const tolerance = description.value?.endpoint_tolerance ?? { position: 1e-4, rotation: 1e-3 }
  return value && (value.start_position >= tolerance.position || value.goal_position >= tolerance.position || value.start_rotation >= tolerance.rotation || value.goal_rotation >= tolerance.rotation)
})
const connectionLabel = computed(() => connected.value ? 'C++ connected' : connecting.value ? 'Connecting' : 'C++ offline')
const stateLabel = computed(() => {
  if (graphDirty.value) return 'Rebuild required'
  if (building.value) return 'Building graph'
  if (solving.value) return 'Optimizing'
  if (latest.value?.converged) return 'Converged'
  if (latest.value?.stop_reason === 'iteration_limit') return 'Iteration limit reached'
  if (latest.value?.stop_reason === 'stalled') return 'Endpoint residual remains'
  return graphReady.value ? 'Graph ready' : 'Set your endpoints'
})

function clonePose(pose) {
  return { position: [...pose.position], quaternion: [...pose.quaternion] }
}

function formatNumber(value, digits = 2) {
  if (value == null || !Number.isFinite(Number(value))) return '—'
  const number = Number(value)
  return Math.abs(number) >= 10000 || (Math.abs(number) < 0.01 && number !== 0)
    ? number.toExponential(2) : number.toLocaleString(undefined, { maximumFractionDigits: digits })
}

function transmit(message) {
  if (socket?.readyState !== WebSocket.OPEN) {
    error.value = 'The C++ optimizer is disconnected. Start the demo server to reconnect.'
    return false
  }
  socket.send(JSON.stringify(message))
  return true
}

function setSelection(value) {
  selection.value = value
  viewer?.setSelection(value)
}

function setTransformMode(value) {
  transformMode.value = value
  viewer?.setTransformMode(value)
}

function stopIterationReplay() {
  clearInterval(replayTimer)
  replayingIterations.value = false
}

function showHistory(index) {
  historyIndex.value = Math.min(Math.max(Number(index), 0), Math.max(history.value.length - 1, 0))
  const snapshot = history.value[historyIndex.value]
  if (snapshot && viewerReady.value) viewer.setTrajectory(snapshot)
}

function reviewIteration(event) {
  stopIterationReplay()
  followLatest.value = false
  showHistory(event.target.value)
}

function followSolution() {
  stopIterationReplay()
  followLatest.value = true
  showHistory(history.value.length - 1)
}

function toggleIterationReplay() {
  if (replayingIterations.value) return stopIterationReplay()
  if (history.value.length < 2) return
  followLatest.value = false
  if (historyIndex.value >= history.value.length - 1) showHistory(0)
  replayingIterations.value = true
  replayTimer = setInterval(() => {
    if (historyIndex.value >= history.value.length - 1) return stopIterationReplay()
    showHistory(historyIndex.value + 1)
  }, 240)
}

function captureSnapshot(snapshot) {
  latest.value = snapshot
  graphReady.value = true
  if (snapshot.type === 'built' || snapshot.type === 'targets') {
    stopIterationReplay()
    history.value = [snapshot]
    historyIndex.value = 0
    followLatest.value = true
  } else {
    history.value = [...history.value, snapshot].slice(-201)
  }
  if (followLatest.value) showHistory(history.value.length - 1)
}

function queueTargets() {
  if (!graphReady.value || graphDirty.value) return
  targetPending.value = true
  if (!targetTimer) targetTimer = setTimeout(flushTargets, 150)
}

function flushTargets() {
  clearTimeout(targetTimer)
  targetTimer = null
  if (!targetPending.value || !start.value || !goal.value) return
  targetPending.value = false
  error.value = ''
  transmit({ type: 'targets', start: clonePose(start.value), goal: clonePose(goal.value), solve: liveReplan.value && playing.value })
}

function targetChanged(targets) {
  start.value = clonePose(targets.start)
  goal.value = clonePose(targets.goal)
  queueTargets()
}

function updatePosition(index, event) {
  const value = Number(event.target.value)
  if (!Number.isFinite(value) || !selectedPose.value) return
  const pose = clonePose(selectedPose.value)
  pose.position[index] = value
  if (selection.value === 'start') start.value = pose
  else goal.value = pose
  viewer?.setTargets(start.value, goal.value)
  queueTargets()
}

function updateRotation(index, event) {
  const value = Number(event.target.value)
  if (!Number.isFinite(value) || !selectedPose.value) return
  const angles = [...selectedEuler.value]
  angles[index] = value
  const quaternion = new Quaternion().setFromEuler(new Euler(...angles.map(angle => angle * Math.PI / 180), 'XYZ'))
  const pose = { position: [...selectedPose.value.position], quaternion: quaternion.toArray() }
  if (selection.value === 'start') start.value = pose
  else goal.value = pose
  viewer?.setTargets(start.value, goal.value)
  queueTargets()
}

function restoreTargets() {
  if (!description.value) return
  start.value = clonePose(description.value.start)
  goal.value = clonePose(description.value.goal)
  viewer?.setTargets(start.value, goal.value)
  queueTargets()
}

function markGraphDirty() {
  graphDirty.value = graphReady.value
  if (playing.value) togglePlayback()
  if (solving.value) transmit({ type: 'stop' })
  clearTimeout(targetTimer)
  targetTimer = null
  targetPending.value = false
}

function buildGraph() {
  if (!start.value || !goal.value) return
  const count = Number(intermediateStates.value), seconds = Number(duration.value)
  if (!Number.isInteger(count) || count < 1 || count > 40 || !Number.isFinite(seconds) || seconds < 0.5 || seconds > 20) {
    error.value = 'Choose 1–40 intermediate states and a duration from 0.5–20 seconds.'
    return
  }
  clearTimeout(targetTimer)
  targetTimer = null
  targetPending.value = false
  stopIterationReplay()
  viewer?.setPlaying(false)
  playing.value = false
  error.value = ''
  building.value = true
  if (!transmit({ type: 'build', intermediate_states: count, duration: seconds, start: clonePose(start.value), goal: clonePose(goal.value) })) building.value = false
}

function solveGraph() {
  if (solving.value) {
    // Send the latest drag sample first, so a delayed target update cannot
    // silently restart optimization after the user has pressed Stop.
    flushTargets()
    transmit({ type: 'stop' })
    return
  }
  flushTargets()
  followSolution()
  error.value = ''
  const iterations = Math.round(Number(maximumIterations.value))
  if (!Number.isFinite(iterations) || iterations < 1 || iterations > 80) {
    error.value = 'Choose between 1 and 80 optimizer iterations.'
    return
  }
  if (transmit({ type: 'solve', max_iterations: iterations, iteration_delay_ms: 80 })) solving.value = true
}

function togglePlayback() {
  playing.value = !playing.value
  viewer?.setPlaying(playing.value)
  if (playing.value && targetPending.value) flushTargets()
}

function changeTime(event) {
  time.value = Number(event.target.value)
  viewer?.setTime(time.value)
}

function changeSpeed() {
  viewer?.setSpeed(Number(speed.value))
}

function changeGhosts() {
  viewer?.setGhosts(ghosts.value)
}

async function setupViewer(data) {
  if (initializingViewer || viewerReady.value || disposed) return
  initializingViewer = true
  modelError.value = ''
  try {
    await nextTick()
    viewer = new SteapViewer(canvas.value, {
      onTargetChange: targetChanged,
      onSelectionChange: value => { selection.value = value },
      onTimeChange: state => {
        time.value = state.time
        playing.value = state.playing
        trackingError.value = state.trackingError ?? null
        tcpPositionError.value = state.tcpPositionError ?? null
        endpointStatus.value = state.endpointStatus ?? 'moving'
        endpointName.value = state.endpointName ?? 'goal'
        endpointPositionError.value = state.endpointPositionError ?? null
        endpointRotationError.value = state.endpointRotationError ?? null
      },
    })
    await viewer.init(data)
    if (disposed) { viewer.dispose(); return }
    // Endpoint controls remain usable while meshes load; use the latest edits.
    viewer.setTargets(start.value, goal.value)
    viewerReady.value = true
    viewer.setSelection(selection.value)
    viewer.setTransformMode(transformMode.value)
    viewer.setGhosts(ghosts.value)
    if (latest.value) viewer.setTrajectory(shown.value)
  } catch (exception) {
    modelError.value = `MuJoCo could not load: ${exception.message}`
    viewer?.dispose()
    viewer = null
  } finally {
    initializingViewer = false
  }
}

function onMessage(event) {
  let data
  try { data = JSON.parse(event.data) } catch { error.value = 'The optimizer sent an invalid response.'; return }
  if (data.type === 'description') {
    description.value = data
    if (!start.value || !goal.value) {
      start.value = clonePose(data.start)
      goal.value = clonePose(data.goal)
    }
    notice.value = 'Set the frames, then build your factor graph.'
    setupViewer({ ...data, start: start.value, goal: goal.value })
  } else if (['built', 'targets', 'iteration'].includes(data.type)) {
    if (data.type === 'built') {
      building.value = false
      graphDirty.value = false
      notice.value = 'Graph built. Solve to see the trajectory evolve.'
    }
    captureSnapshot(data)
  } else if (data.type === 'status') {
    solving.value = Boolean(data.solving)
    if (data.message) notice.value = data.message
  } else if (data.type === 'error') {
    error.value = data.message || 'The optimizer encountered an error.'
    building.value = false
    solving.value = false
  }
}

function connect() {
  if (disposed) return
  connecting.value = true
  socket = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/socket`)
  socket.onopen = () => {
    connected.value = true
    connecting.value = false
    error.value = ''
  }
  socket.onmessage = onMessage
  socket.onerror = () => { notice.value = 'Waiting for the C++ bridge on port 8766.' }
  socket.onclose = () => {
    connected.value = false
    connecting.value = false
    solving.value = false
    building.value = false
    if (graphReady.value) graphDirty.value = true
    playing.value = false
    viewer?.setPlaying(false)
    if (!disposed) reconnectTimer = setTimeout(connect, 2000)
  }
}

onMounted(connect)
defineExpose({ getDiagnostics: () => viewer?.getDiagnostics?.() })
onBeforeUnmount(() => {
  disposed = true
  clearTimeout(reconnectTimer)
  clearTimeout(targetTimer)
  stopIterationReplay()
  socket?.close()
  viewer?.dispose()
})
</script>

<template>
  <div class="lab-shell">
    <header class="masthead">
      <a class="brand" href="./" aria-label="STEAP Lab home"><span class="brand-mark" aria-hidden="true">S<span>↗</span></span><span>GTDynamics<span class="brand-divider"> / </span><strong>STEAP Lab</strong></span></a>
      <div class="masthead-right"><span class="lab-caption">INTERACTIVE ROBOTICS LAB</span><span class="connection-pill" :class="{ online: connected }"><span class="status-dot"></span>{{ connectionLabel }}</span></div>
    </header>

    <main>
      <section class="intro">
        <div><p class="eyebrow">01 / TRAJECTORY OPTIMIZATION</p><h1>A motion plan, <em>taking shape.</em></h1><p class="intro-copy">Move the endpoints. Connect the states. Watch a Franka arm find its way.</p></div>
        <div class="technology-caption"><span>GTDynamics + GTSAM</span><span>MuJoCo in your browser</span></div>
      </section>

      <div v-if="error" class="error-banner" role="alert"><span>{{ error }}</span><button class="text-button" @click="error = ''" aria-label="Dismiss error">Dismiss ×</button></div>

      <div class="workspace">
        <section class="viewport-panel" aria-label="Interactive Franka workspace">
          <div class="viewport-toolbar"><div class="viewport-label"><span class="live-dot"></span><strong>Franka Panda</strong><span>7 DOF</span></div><label class="toggle-label"><input v-model="ghosts" type="checkbox" @change="changeGhosts" /> Show trajectory poses</label></div>
          <div class="canvas-wrap"><div ref="canvas" class="robot-canvas"></div>
            <div v-if="!viewerReady" class="loading-overlay"><div class="loading-symbol" aria-hidden="true">↗</div><strong>{{ modelError ? 'Scene unavailable' : description ? 'Loading the Franka scene' : 'Waiting for the optimizer' }}</strong><p>{{ modelError || (description ? 'Initializing MuJoCo WebAssembly and the robot meshes…' : 'The local C++ server provides the robot model and the factor graph.') }}</p><button v-if="modelError" class="button secondary small" @click="setupViewer(description)">Retry scene</button></div>
            <div v-if="viewerReady" class="scene-legend"><span><i class="legend-dot start"></i> Start frame</span><span><i class="legend-dot goal"></i> Goal frame</span><span><i class="legend-line"></i> Current estimate</span></div>
            <div v-if="viewerReady" class="scene-instruction">Drag the {{ selection }} {{ transformMode === 'rotate' ? 'rings to rotate' : 'arrows to translate' }}<span>Orbit: drag background · Zoom: scroll</span></div>
            <div class="view-badge" :class="{ busy: solving }"><span class="status-dot"></span>{{ stateLabel }}</div>
          </div>

          <div class="playback-panel">
            <div class="playback-top"><div><span class="section-kicker">ARM PLAYBACK</span><p>Replay the current trajectory estimate</p></div><label class="speed-control"><span>Speed</span><select v-model="speed" @change="changeSpeed" aria-label="Playback speed"><option :value="0.25">0.25×</option><option :value="0.5">0.5×</option><option :value="1">1×</option><option :value="2">2×</option></select></label></div>
            <div class="playback-controls"><button class="play-button" :disabled="!canPlay" @click="togglePlayback" :aria-label="playing ? 'Pause arm playback' : 'Play arm trajectory'"><span aria-hidden="true">{{ playing ? 'Ⅱ' : '▶' }}</span>{{ playing ? 'Pause' : 'Play' }}</button><input class="time-slider" type="range" :value="time" min="0" :max="actualDuration" step="0.01" :disabled="!canPlay" @input="changeTime" aria-label="Arm trajectory time" /><span class="time-readout">{{ time.toFixed(2) }} <span>/ {{ actualDuration.toFixed(1) }} s</span></span></div>
            <div class="playback-footer"><label class="toggle-label"><input v-model="liveReplan" type="checkbox" /> Replan when frames move during playback</label><span v-if="trackingError != null" class="tracking-readout">Tracking {{ formatNumber(trackingError, 3) }} rad<span v-if="tcpPositionError != null"> · {{ formatNumber(tcpPositionError * 1000, 1) }} mm TCP</span></span><span v-else class="muted">Position-controlled MuJoCo replay</span></div>
            <p v-if="endpointStatus === 'settling'" class="endpoint-feedback" role="status">Settling at the {{ endpointName }} frame before reversing.</p>
            <p v-else-if="endpointStatus === 'unreached'" class="endpoint-feedback warning" role="status">Playback paused: the {{ endpointName }} frame was not reached ({{ formatNumber(endpointPositionError * 1000, 2) }} mm, {{ formatNumber(endpointRotationError * 180 / Math.PI, 2) }}°). Check the endpoint residuals, then solve again or adjust the frame.</p>
          </div>
        </section>

        <aside class="control-panel">
          <section class="control-section target-section"><div class="section-heading"><div><span class="step-number">01</span><h2>Place the endpoints</h2></div><button class="text-button" :disabled="!description" @click="restoreTargets">Reset</button></div>
            <div class="frame-selector" role="group" aria-label="Selected endpoint"><button :class="{ selected: selection === 'start' }" @click="setSelection('start')"><i class="legend-dot start"></i> Start frame<span>T₀</span></button><button :class="{ selected: selection === 'goal' }" @click="setSelection('goal')"><i class="legend-dot goal"></i> Goal frame<span>Tₙ</span></button></div>
            <div class="mode-selector" role="group" aria-label="Frame manipulation mode"><button :class="{ selected: transformMode === 'translate' }" @click="setTransformMode('translate')"><span aria-hidden="true">↔</span> Translate</button><button :class="{ selected: transformMode === 'rotate' }" @click="setTransformMode('rotate')"><span aria-hidden="true">⟳</span> Rotate</button></div>
            <div class="coordinate-heading"><span>Position</span><span>METERS · WORLD FRAME</span></div><div class="coordinate-inputs"><label v-for="(axis, index) in ['X', 'Y', 'Z']" :key="axis"><span :class="`axis-${axis.toLowerCase()}`">{{ axis }}</span><input type="number" step="0.01" v-model="positionInputs[index]" :disabled="!selectedPose" @change="updatePosition(index, $event)" :aria-label="`${selection} position ${axis} in meters`" /></label></div>
            <div class="coordinate-heading"><span>Orientation</span><span>DEGREES · XYZ EULER</span></div><div class="coordinate-inputs"><label v-for="(axis, index) in ['Rx', 'Ry', 'Rz']" :key="axis"><span>{{ axis }}</span><input type="number" step="5" v-model="rotationInputs[index]" :disabled="!selectedPose" @change="updateRotation(index, $event)" :aria-label="`${selection} rotation ${axis} in degrees`" /></label></div>
            <p class="field-hint">The colored frames define full 6-DOF pose priors on the hand.</p>
          </section>

          <section class="control-section"><div class="section-heading"><div><span class="step-number">02</span><h2>Build the factor graph</h2></div></div><div class="form-grid"><label class="field-label">Intermediate states<input v-model.number="intermediateStates" type="number" min="1" max="40" step="1" :disabled="building" @input="markGraphDirty" /></label><label class="field-label">Duration <span class="unit">seconds</span><input v-model.number="duration" type="number" min="0.5" max="20" step="0.5" :disabled="building" @input="markGraphDirty" /></label></div><div class="configuration-summary"><span>{{ Number(intermediateStates) + 2 }} slices, including endpoints</span><span>Δt {{ formatNumber(timestep, 3) }} s</span></div><button class="button secondary full" :disabled="!connected || !start || building" @click="buildGraph"><span aria-hidden="true">⊞</span>{{ building ? 'Building…' : graphReady ? 'Rebuild graph' : 'Build graph' }}</button><p v-if="graphDirty" class="field-hint warning">The discretization changed. Rebuild to use these settings.</p></section>

          <section class="control-section solve-section"><div class="section-heading"><div><span class="step-number">03</span><h2>Optimize the motion</h2></div><label class="iteration-cap"><span>Max. iter.</span><input v-model.number="maximumIterations" aria-label="Maximum solver iterations" type="number" min="1" max="80" /></label></div><button class="button primary full" :disabled="!canSolve" @click="solveGraph"><span aria-hidden="true">{{ solving ? '■' : '↗' }}</span>{{ solving ? 'Stop optimization' : 'Solve trajectory' }}</button><p class="field-hint">Each Levenberg–Marquardt iteration updates the path and every robot state.</p><div class="solver-status" role="status" aria-live="polite"><span class="status-dot" :class="{ busy: solving }"></span>{{ notice }}</div></section>
        </aside>
      </div>

      <section class="graph-panel"><div class="graph-header"><div><span class="section-kicker">THE FACTOR GRAPH</span><h2>One slice per instant. One connected motion.</h2></div><div class="graph-stats"><div><strong>{{ graphReady ? stateCount : '—' }}</strong><span>STATES</span></div><div><strong>{{ graph.variables ?? '—' }}</strong><span>VARIABLES</span></div><div><strong>{{ graph.factors ?? '—' }}</strong><span>FACTORS</span></div></div></div><div class="graph-strip" :class="{ inactive: !graphReady }"><template v-for="(slice, index) in graphSlices" :key="slice"><div v-if="slice === 'ellipsis'" class="slice-ellipsis">···</div><div v-else class="state-slice" :class="{ first: slice === 0, last: slice === stateCount - 1 }"><span class="slice-title">{{ slice === 0 ? 'START' : slice === stateCount - 1 ? 'GOAL' : `t${slice}` }}</span><div class="slice-variables"><span>q</span><span>q̇</span><span>T</span><span>V</span></div><span class="slice-caption">{{ slice === 0 || slice === stateCount - 1 ? 'pose + rest priors' : 'kinematic constraints' }}</span></div><div v-if="index < graphSlices.length - 1" class="integration-edge"><span>∫</span><i></i><small>Δt</small></div></template></div><div class="graph-explanation"><p><strong>Inside each slice</strong> Joint angles <em>q</em>, joint velocities <em>q̇</em>, link poses <em>T</em>, and link twists <em>V</em> obey the robot’s kinematics.</p><p><strong>Between slices</strong> Trapezoidal integration connects joint angles to velocities and link poses to twists. Velocity smoothness regularizes the motion.</p></div></section>

      <section class="iteration-panel"><div class="iteration-intro"><span class="section-kicker">OPTIMIZER PLAYBACK</span><h2>Watch the solution evolve.</h2><p>Scrub the actual GTSAM iterations. Arm playback follows whichever trajectory estimate you select.</p><div class="history-actions"><button class="button secondary small" :disabled="history.length < 2" @click="toggleIterationReplay"><span aria-hidden="true">{{ replayingIterations ? 'Ⅱ' : '▶' }}</span>{{ replayingIterations ? 'Pause iterations' : 'Replay iterations' }}</button><button class="text-button live-follow" :class="{ active: followLatest }" :disabled="!history.length" @click="followSolution"><span class="status-dot"></span>Follow latest</button></div></div><div class="iteration-details"><div class="iteration-metrics"><div><span>ITERATION</span><strong>{{ shown?.iteration ?? '—' }}<small v-if="history.length"> / {{ history.at(-1)?.iteration ?? 0 }}</small></strong></div><div><span>GRAPH ERROR</span><strong>{{ formatNumber(shown?.error, 3) }}</strong></div><div class="cost-chart"><span>ERROR HISTORY · LOG SCALE</span><svg viewBox="0 0 300 50" aria-label="Graph error across solver iterations" role="img"><line x1="5" y1="44" x2="295" y2="44" stroke="currentColor" opacity="0.2" /><polyline v-if="costPoints" :points="costPoints" fill="none" stroke="currentColor" stroke-width="2" /><circle v-if="costMarker" :cx="costMarker.x" :cy="costMarker.y" r="3.5" fill="currentColor" /></svg></div></div><input class="iteration-slider" type="range" min="0" :max="Math.max(history.length - 1, 0)" :value="historyIndex" :disabled="history.length < 2" @input="reviewIteration" aria-label="Recorded optimizer iteration" /><div class="slider-labels"><span>Initial estimate</span><span>{{ followLatest ? 'Following live optimization' : 'Reviewing a captured estimate' }}</span><span>Latest estimate</span></div><div v-if="residual" class="residuals" :class="{ 'residual-warning': residualLarge }"><span>Endpoint residuals</span><span><i class="legend-dot start"></i>{{ formatNumber(residual.start_position * 1000, 1) }} mm · {{ formatNumber(residual.start_rotation * 180 / Math.PI, 1) }}°</span><span><i class="legend-dot goal"></i>{{ formatNumber(residual.goal_position * 1000, 1) }} mm · {{ formatNumber(residual.goal_rotation * 180 / Math.PI, 1) }}°</span></div><p v-if="residualLarge" class="residual-message">The current estimate misses an endpoint. Try more iterations or move the target closer to the arm’s reachable workspace.</p></div></section>

      <footer><span><strong>STEAP-inspired planning</strong> · Live kinematic factor graph optimization</span><a class="architecture-link" href="./architecture.html" target="_blank" rel="noopener">Architecture &amp; equations ↗</a><span>Teaching demo · No obstacle avoidance or measurement estimation</span></footer>
    </main>
  </div>
</template>
