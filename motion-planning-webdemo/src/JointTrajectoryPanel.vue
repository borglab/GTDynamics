<script setup>
import { computed, ref } from 'vue'
import { sampleTrajectoryState } from './viewer.js'

const props = defineProps({
  snapshot: Object,
  history: { type: Array, default: () => [] },
  jointNames: { type: Array, default: () => [] },
  playbackTime: { type: Number, default: 0 },
})
const colors = ['#8abfff', '#f2ba72', '#c5a0f5', '#80d5b0', '#f18f9e', '#73d6df', '#d6d77d']
const hidden = ref([])
const inspectedTime = ref(null)
const left = 58, right = 600, top = 18, bottom = 218
const duration = computed(() => props.snapshot?.duration || 1)
const cursorTime = computed(() => Math.max(0, Math.min(duration.value, inspectedTime.value ?? props.playbackTime)))
const cursor = computed(() => sampleTrajectoryState(props.snapshot, cursorTime.value))
const joints = computed(() => (props.snapshot?.states?.[0]?.q || []).map((_, index) => ({
  index, name: props.jointNames[index] || `Joint ${index + 1}`, color: colors[index % colors.length],
})))

// Snapshots are immutable. Reuse their sampled curves while scrubbing history;
// weak keys release old plots when rebuilding or replacing the trajectory.
const cache = new WeakMap()
function samples(snapshot) {
  if (!snapshot?.states?.length) return []
  if (!cache.has(snapshot)) {
    const count = Math.max(1, (snapshot.states.length - 1) * 12)
    cache.set(snapshot, Array.from({ length: count + 1 }, (_, i) => {
      const t = snapshot.duration * i / count
      return { t, ...sampleTrajectoryState(snapshot, t) }
    }))
  }
  return cache.get(snapshot)
}
const curves = computed(() => samples(props.snapshot))
// Use the whole recorded solve so the axes stay fixed when replaying iterations.
const ranges = computed(() => {
  const bounds = { q: [Infinity, -Infinity], v: [Infinity, -Infinity] }
  const snapshots = props.history.length ? props.history : [props.snapshot]
  for (const snapshot of snapshots) {
    for (const sample of samples(snapshot)) {
      for (const key of ['q', 'v']) {
        for (const value of sample[key]) {
          bounds[key][0] = Math.min(bounds[key][0], value)
          bounds[key][1] = Math.max(bounds[key][1], value)
        }
      }
    }
  }
  return Object.fromEntries(Object.entries(bounds).map(([key, [min, max]]) => {
    if (!Number.isFinite(min)) return [key, [-1, 1]]
    const padding = Math.max(max - min, 0.1) * 0.1
    return [key, [min - padding, max + padding]]
  }))
})
function x(t) { return left + t / duration.value * (right - left) }
function y(value, range) { return bottom - (value - range[0]) / (range[1] - range[0]) * (bottom - top) }
const plots = computed(() => [
  { key: 'q', title: 'Joint angles', unit: 'rad' },
  { key: 'v', title: 'Joint velocities', unit: 'rad/s' },
].map(plot => {
  const range = ranges.value[plot.key]
  return {
    ...plot, range,
    ticks: Array.from({ length: 5 }, (_, i) => range[0] + (range[1] - range[0]) * i / 4),
    series: joints.value.map(joint => ({
      ...joint,
      points: curves.value.map(sample => `${x(sample.t)},${y(sample[plot.key][joint.index], range)}`).join(' '),
      knots: props.snapshot.states.map((state, i, states) => ({
        x: x(i * duration.value / Math.max(1, states.length - 1)),
        y: y(state[plot.key][joint.index], range),
      })),
    })),
  }
}))
function toggleJoint(index) {
  hidden.value = hidden.value.includes(index) ? hidden.value.filter(i => i !== index) : [...hidden.value, index]
}
function inspect(event) {
  const svg = event.currentTarget
  const matrix = svg.getScreenCTM()
  if (!matrix) return
  const point = svg.createSVGPoint()
  point.x = event.clientX; point.y = event.clientY
  const local = point.matrixTransform(matrix.inverse())
  inspectedTime.value = Math.max(0, Math.min(duration.value, (local.x - left) / (right - left) * duration.value))
}
function format(value) { return Number.isFinite(value) ? value.toFixed(3) : '—' }
function tick(value) { return Math.abs(value) < 0.00001 ? '0' : Number(value.toPrecision(3)).toString() }
</script>

<template>
  <section class="joint-panel" aria-labelledby="joint-panel-title" :data-iteration="snapshot?.iteration ?? ''">
    <div class="joint-header">
      <div><span class="section-kicker">JOINT TRAJECTORIES</span><h2 id="joint-panel-title">See how each joint moves.</h2>
        <p>Trajectory time on the horizontal axis. Curves follow the selected optimizer iteration.</p></div>
      <span class="joint-iteration">ITERATION <strong>{{ snapshot?.iteration ?? '—' }}</strong></span>
    </div>
    <p v-if="!curves.length" class="joint-empty">Build the factor graph to inspect joint angles and velocities. Solve to watch the time series evolve.</p>
    <template v-else>
      <div class="joint-legend" role="group" aria-label="Visible joints">
        <button v-for="joint in joints" :key="joint.index" :aria-pressed="!hidden.includes(joint.index)"
          :style="{ '--joint-color': joint.color }" @click="toggleJoint(joint.index)">
          <span aria-hidden="true"></span>{{ joint.name }}
        </button>
        <span class="joint-legend-hint">Click a joint to show or hide it</span>
      </div>
      <div class="joint-plots">
        <figure v-for="plot in plots" :key="plot.key" :data-quantity="plot.key">
          <figcaption>{{ plot.title }} <span>{{ plot.unit }}</span></figcaption>
          <svg viewBox="0 0 620 258" role="img" :aria-label="`${plot.title} in ${plot.unit} versus trajectory time in seconds, iteration ${snapshot.iteration}`"
            @pointermove="inspect" @pointerdown="inspect">
            <g v-for="value in plot.ticks" :key="value" class="joint-axis">
              <line :x1="left" :x2="right" :y1="y(value, plot.range)" :y2="y(value, plot.range)" />
              <text :x="left - 8" :y="y(value, plot.range) + 4" text-anchor="end">{{ tick(value) }}</text>
            </g>
            <g v-for="i in 5" :key="i" class="joint-axis">
              <line :x1="x((i - 1) * duration / 4)" :x2="x((i - 1) * duration / 4)" :y1="top" :y2="bottom" />
              <text :x="x((i - 1) * duration / 4)" :y="bottom + 18" text-anchor="middle">{{ tick((i - 1) * duration / 4) }}</text>
            </g>
            <text class="joint-time-label" x="329" y="253" text-anchor="middle">Trajectory time (s)</text>
            <g v-for="series in plot.series" :key="series.index" v-show="!hidden.includes(series.index)" :stroke="series.color" :fill="series.color">
              <polyline class="joint-curve" :data-joint="series.index" :points="series.points" fill="none" stroke-width="1.8" />
              <circle v-for="(knot, i) in series.knots" :key="i" :cx="knot.x" :cy="knot.y" r="2.5" stroke="none" />
            </g>
            <line class="joint-cursor" :x1="x(cursorTime)" :x2="x(cursorTime)" :y1="top" :y2="bottom" />
            <template v-for="joint in joints" :key="joint.index">
              <circle v-if="!hidden.includes(joint.index)" :cx="x(cursorTime)" :cy="y(cursor[plot.key][joint.index], plot.range)" r="3.5" :fill="joint.color" stroke="#141e27" stroke-width="1.5" />
            </template>
          </svg>
        </figure>
      </div>
      <div class="joint-inspector">
        <label for="joint-inspect-time">Inspect <strong>{{ cursorTime.toFixed(2) }} s</strong></label>
        <input id="joint-inspect-time" type="range" min="0" :max="duration" step="0.001" :value="cursorTime"
          aria-label="Joint plot inspection time" @input="inspectedTime = Number($event.target.value)" />
        <button class="text-button" :class="{ active: inspectedTime === null }" :aria-pressed="inspectedTime === null" @click="inspectedTime = null">Follow arm time</button>
      </div>
      <div class="joint-values-wrap">
        <table class="joint-values"><caption class="sr-only">Joint values at the inspection time for the selected optimizer iteration</caption>
          <thead><tr><th scope="col">At {{ cursorTime.toFixed(2) }} s</th><th v-for="joint in joints" :key="joint.index" scope="col" :style="{ color: joint.color }">{{ joint.name }}</th></tr></thead>
          <tbody><tr v-for="plot in plots" :key="plot.key"><th scope="row">{{ plot.title }} <span>({{ plot.unit }})</span></th>
            <td v-for="joint in joints" :key="joint.index">{{ format(cursor[plot.key][joint.index]) }}</td></tr></tbody>
        </table>
      </div>
      <p class="joint-note">Dots are optimized states; curves use the same Hermite interpolation as arm playback. Values describe the selected plan, in trajectory seconds. Hover over a plot or use the time slider to inspect.</p>
    </template>
  </section>
</template>

<style scoped>
.joint-panel { margin-top: 20px; padding: 25px; border: 1px solid var(--border); border-radius: 11px; background: var(--panel); }
.joint-header { display: flex; justify-content: space-between; align-items: start; gap: 20px; }
.joint-header h2 { font-size: 20px; font-weight: 500; margin-top: 7px; }
.joint-header p, .joint-empty, .joint-note { color: var(--muted); font-size: 11px; line-height: 1.8; margin-top: 10px; }
.joint-iteration { white-space: nowrap; font: 10px var(--mono); color: var(--muted); padding-top: 6px; }
.joint-iteration strong { margin-left: 8px; font-size: 20px; color: var(--accent); font-weight: 400; }
.joint-empty { padding: 35px 0; text-align: center; }
.joint-legend { display: flex; flex-wrap: wrap; align-items: center; gap: 8px; margin: 22px 0 18px; }
.joint-legend button { display: inline-flex; align-items: center; gap: 7px; padding: 6px 9px; border: 1px solid var(--border); border-radius: 5px; background: #101820; font: 11px var(--mono); cursor: pointer; }
.joint-legend button[aria-pressed='false'] { opacity: .45; text-decoration: line-through; }
.joint-legend button span { width: 16px; height: 3px; background: var(--joint-color); }
.joint-legend-hint { font-size: 10px; color: var(--muted); margin-left: 5px; }
.joint-plots { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 22px; }
figure { margin: 0; min-width: 0; }
figcaption { font-size: 13px; font-weight: 500; margin-bottom: 8px; }
figcaption span { color: var(--muted); font: 10px var(--mono); margin-left: 7px; }
svg { display: block; width: 100%; overflow: visible; cursor: crosshair; }
.joint-axis line { stroke: var(--border); stroke-width: .7; }
.joint-axis text, .joint-time-label { fill: var(--muted); font: 10px var(--mono); }
.joint-cursor { stroke: #dce5eb; stroke-width: 1; stroke-dasharray: 4 4; opacity: .7; }
.joint-inspector { display: flex; align-items: center; gap: 18px; margin: 22px 0 18px; font-size: 11px; }
.joint-inspector label { color: var(--muted); white-space: nowrap; }
.joint-inspector strong { display: inline-block; min-width: 58px; margin-left: 6px; font-family: var(--mono); color: var(--text); }
.joint-inspector input { flex: 1; min-width: 50px; }
.joint-inspector button { font-size: 11px; white-space: nowrap; }
.joint-inspector button.active { color: var(--accent); }
.joint-values-wrap { overflow-x: auto; }
.joint-values { width: 100%; border-collapse: collapse; font: 11px var(--mono); text-align: right; white-space: nowrap; }
.joint-values th, .joint-values td { padding: 10px 8px; border-bottom: 1px solid var(--border); }
.joint-values th { font-weight: 400; }.joint-values th:first-child { text-align: left; color: var(--muted); }
.joint-values th span { font-size: 10px; }.joint-note { margin-top: 14px; }
.sr-only { position: absolute; width: 1px; height: 1px; overflow: hidden; clip-path: inset(50%); }
@media (max-width: 850px) { .joint-plots { grid-template-columns: 1fr; gap: 22px; } }
@media (max-width: 560px) {
  .joint-panel { padding: 21px 17px; }.joint-header { flex-wrap: wrap; gap: 10px; }
  .joint-header h2 { font-size: 19px; }.joint-inspector { flex-wrap: wrap; gap: 12px; }
  .joint-legend-hint { flex-basis: 100%; margin-top: 4px; }.joint-values { font-size: 10px; }
}
</style>
