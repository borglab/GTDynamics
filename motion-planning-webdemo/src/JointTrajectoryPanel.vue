<script setup>
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'
import { sampleTrajectoryState } from './viewer.js'

const props = defineProps({
  snapshot: Object,
  history: { type: Array, default: () => [] },
  jointNames: { type: Array, default: () => [] },
  playbackTime: { type: Number, default: 0 },
})
const quantities = [
  { key: 'q', title: 'Angle', unit: 'rad', color: '#8abfff' },
  { key: 'v', title: 'Velocity', unit: 'rad/s', color: '#f2ba72' },
]
const panel = ref(null)
const plotWidth = ref(900)
let resizeObserver
onMounted(() => {
  resizeObserver = new ResizeObserver(entries => {
    plotWidth.value = Math.max(240, entries[0].contentRect.width)
  })
  resizeObserver.observe(panel.value)
})
onBeforeUnmount(() => resizeObserver?.disconnect())
const inspectedTime = ref(null)
const left = 64, top = 12, bottom = 92
const right = computed(() => plotWidth.value - 64)
const duration = computed(() => props.snapshot?.duration || 1)
const cursorTime = computed(() => Math.max(0, Math.min(duration.value, inspectedTime.value ?? props.playbackTime)))
const cursor = computed(() => sampleTrajectoryState(props.snapshot, cursorTime.value))
const joints = computed(() => (props.snapshot?.states?.[0]?.q || []).map((_, index) => ({
  index, name: props.jointNames[index] || `Joint ${index + 1}`,
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
  const bounds = joints.value.map(() => ({ q: [Infinity, -Infinity], v: [Infinity, -Infinity] }))
  const snapshots = props.history.length ? props.history : [props.snapshot]
  for (const snapshot of snapshots) {
    for (const sample of samples(snapshot)) {
      for (const joint of joints.value) {
        for (const { key } of quantities) {
          const range = bounds[joint.index][key], value = sample[key][joint.index]
          range[0] = Math.min(range[0], value)
          range[1] = Math.max(range[1], value)
        }
      }
    }
  }
  return bounds.map(joint => Object.fromEntries(Object.entries(joint).map(([key, [min, max]]) => {
    if (!Number.isFinite(min)) return [key, [-1, 1]]
    const padding = Math.max(max - min, 0.1) * 0.1
    return [key, [min - padding, max + padding]]
  })))
})
function x(t) { return left + t / duration.value * (right.value - left) }
function y(value, range) { return bottom - (value - range[0]) / (range[1] - range[0]) * (bottom - top) }
const plots = computed(() => joints.value.map(joint => ({
  ...joint,
  series: quantities.map(quantity => {
    const range = ranges.value[joint.index][quantity.key]
    return {
      ...quantity, range,
      ticks: Array.from({ length: 3 }, (_, i) => range[0] + (range[1] - range[0]) * i / 2),
      points: curves.value.map(sample => `${x(sample.t)},${y(sample[quantity.key][joint.index], range)}`).join(' '),
      knots: props.snapshot.states.map((state, i, states) => ({
        x: x(i * duration.value / Math.max(1, states.length - 1)),
        y: y(state[quantity.key][joint.index], range),
      })),
    }
  }),
})))
function inspect(event) {
  const svg = event.currentTarget
  const matrix = svg.getScreenCTM()
  if (!matrix) return
  const point = svg.createSVGPoint()
  point.x = event.clientX; point.y = event.clientY
  const local = point.matrixTransform(matrix.inverse())
  inspectedTime.value = Math.max(0, Math.min(duration.value, (local.x - left) / (right.value - left) * duration.value))
}
function format(value) { return Number.isFinite(value) ? value.toFixed(3) : '—' }
function tick(value) { return Math.abs(value) < 0.00001 ? '0' : Number(value.toPrecision(3)).toString() }
</script>

<template>
  <section ref="panel" class="joint-panel" aria-labelledby="joint-panel-title" :data-iteration="snapshot?.iteration ?? ''">
    <div class="joint-header">
      <div><span class="section-kicker">JOINT TRAJECTORIES</span><h2 id="joint-panel-title">See how each joint moves.</h2>
        <p>One plot per joint, synchronized in trajectory time and optimizer iteration.</p></div>
      <span class="joint-iteration">ITERATION <strong>{{ snapshot?.iteration ?? '—' }}</strong></span>
    </div>
    <p v-if="!curves.length" class="joint-empty">Build the factor graph to inspect joint angles and velocities. Solve to watch the time series evolve.</p>
    <template v-else>
      <div class="joint-legend">
        <span v-for="quantity in quantities" :key="quantity.key" :style="{ color: quantity.color }">
          <i :class="{ dashed: quantity.key === 'v' }" aria-hidden="true"></i>
          {{ quantity.title }} · {{ quantity.unit }} · {{ quantity.key === 'q' ? 'left axis' : 'right axis' }}
        </span>
      </div>
      <div class="joint-inspector">
        <label for="joint-inspect-time">Inspect <strong>{{ cursorTime.toFixed(2) }} s</strong></label>
        <input id="joint-inspect-time" type="range" min="0" :max="duration" step="0.001" :value="cursorTime"
          aria-label="Joint plot inspection time" @input="inspectedTime = Number($event.target.value)" />
        <button class="text-button" :class="{ active: inspectedTime === null }" :aria-pressed="inspectedTime === null" @click="inspectedTime = null">Follow arm time</button>
      </div>
      <div class="joint-plots">
        <figure v-for="plot in plots" :key="plot.index" :data-joint="plot.index">
          <figcaption><strong>{{ plot.name }}</strong><span v-for="series in plot.series" :key="series.key" :style="{ color: series.color }">
            {{ series.title }} <output class="joint-value" :data-quantity="series.key">{{ format(cursor[series.key][plot.index]) }}</output> {{ series.unit }}
          </span></figcaption>
          <svg :viewBox="`0 0 ${plotWidth} 120`" role="img"
            :aria-label="`${plot.name}: angle in radians (left axis) and velocity in radians per second (right axis) versus trajectory time, iteration ${snapshot.iteration}`"
            @pointermove="inspect" @pointerdown="inspect">
            <line v-for="i in 3" :key="`grid-${i}`" class="joint-grid" :x1="left" :x2="right"
              :y1="top + (bottom - top) * (i - 1) / 2" :y2="top + (bottom - top) * (i - 1) / 2" />
            <g v-for="series in plot.series" :key="`axis-${series.key}`" class="joint-axis" :fill="series.color">
              <text v-for="value in series.ticks" :key="value" :x="series.key === 'q' ? left - 8 : right + 8"
                :y="y(value, series.range) + 4" :text-anchor="series.key === 'q' ? 'end' : 'start'">{{ tick(value) }}</text>
            </g>
            <g v-for="i in 5" :key="i" class="joint-axis joint-time-axis">
              <line class="joint-grid" :x1="x((i - 1) * duration / 4)" :x2="x((i - 1) * duration / 4)" :y1="top" :y2="bottom" />
              <text :x="x((i - 1) * duration / 4)" :y="bottom + 18" text-anchor="middle">{{ tick((i - 1) * duration / 4) }}</text>
            </g>
            <g v-for="series in plot.series" :key="series.key" :stroke="series.color" :fill="series.color">
              <polyline class="joint-curve" :data-quantity="series.key" :data-joint="plot.index" :points="series.points"
                fill="none" stroke-width="1.8" :stroke-dasharray="series.key === 'v' ? '5 3' : undefined" />
              <circle v-for="(knot, i) in series.knots" :key="i" :cx="knot.x" :cy="knot.y" r="2" stroke="none" />
              <circle :cx="x(cursorTime)" :cy="y(cursor[series.key][plot.index], series.range)" r="3.5" stroke="#141e27" stroke-width="1.5" />
            </g>
            <line class="joint-cursor" :x1="x(cursorTime)" :x2="x(cursorTime)" :y1="top" :y2="bottom" />
          </svg>
        </figure>
      </div>
      <p class="joint-time-label">Trajectory time (s) · shared across all seven joints</p>
      <p class="joint-note">Each joint has its own angle and velocity scales, held fixed across recorded iterations. Dots are optimized states; curves use the same Hermite interpolation as arm playback. Hover over any plot or use the time slider to inspect all joints together.</p>
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
.joint-legend { display: flex; flex-wrap: wrap; gap: 12px 25px; margin-top: 20px; font-size: 11px; }
.joint-legend span { display: inline-flex; align-items: center; gap: 8px; }
.joint-legend i { display: inline-block; width: 22px; border-top: 2px solid currentColor; }
.joint-legend i.dashed { border-top-style: dashed; }
.joint-plots { display: grid; grid-template-columns: minmax(0, 1fr); gap: 12px; }
figure { margin: 0; min-width: 0; border-top: 1px solid var(--border); padding-top: 12px; }
figcaption { display: flex; flex-wrap: wrap; align-items: baseline; gap: 8px 24px; font: 11px var(--mono); margin-bottom: 4px; }
figcaption strong { color: var(--text); font-weight: 500; margin-right: auto; }
figcaption span { white-space: nowrap; }
.joint-value { display: inline-block; min-width: 48px; text-align: right; }
svg { display: block; width: 100%; cursor: crosshair; }
.joint-grid { stroke: var(--border); stroke-width: .7; }
.joint-axis text { font: 10px var(--mono); }
.joint-time-axis { fill: var(--muted); }
.joint-time-label { color: var(--muted); font: 11px var(--mono); text-align: center; margin-top: 6px; }
.joint-cursor { stroke: #dce5eb; stroke-width: 1; stroke-dasharray: 4 4; opacity: .7; }
.joint-inspector { display: flex; align-items: center; gap: 18px; margin: 20px 0; font-size: 11px; }
.joint-inspector label { color: var(--muted); white-space: nowrap; }
.joint-inspector strong { display: inline-block; min-width: 58px; margin-left: 6px; font-family: var(--mono); color: var(--text); }
.joint-inspector input { flex: 1; min-width: 50px; }
.joint-inspector button { font-size: 11px; white-space: nowrap; }
.joint-inspector button.active { color: var(--accent); }
.joint-note { margin-top: 14px; }
@media (max-width: 560px) {
  .joint-panel { padding: 21px 17px; }.joint-header { flex-wrap: wrap; gap: 10px; }
  .joint-header h2 { font-size: 19px; }.joint-inspector { flex-wrap: wrap; gap: 12px; }
  figcaption { gap: 6px 12px; font-size: 10px; }figcaption strong { flex-basis: 100%; }
}
</style>
