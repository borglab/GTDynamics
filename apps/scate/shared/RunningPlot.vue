<script setup>
/**
 * A small time plot that draws each series up to the playback time, on axes
 * fixed by the whole recording, with a dot at each current value. Click or
 * drag to choose a time.
 */
import { computed, ref } from 'vue';

const props = defineProps({
  /** Heading shown above the plot. */
  title: { type: String, required: true },
  /** Unit shown on the value axis. */
  unit: { type: String, default: '' },
  /** Sample times in seconds, shared by every series. */
  times: { type: Array, required: true },
  /** Series to draw: [{name, color, values}], values aligned with times (null = no value). */
  series: { type: Array, required: true },
  /** Playback time in seconds. */
  time: { type: Number, default: 0 },
  /** Horizontal reference lines: [{value, label, color, dashed}]. */
  references: { type: Array, default: () => [] },
  /** Digits of the axis labels. */
  digits: { type: Number, default: 1 },
  /** Upper end of the value axis, if the data should not set it; larger values are clipped. */
  max: { type: Number, default: null },
});
const emit = defineEmits(['select']);

const width = 300, height = 118;
const margin = { left: 30, right: 6, top: 8, bottom: 18 };
const svg = ref(null);
const clipId = `running-plot-${Math.random().toString(36).slice(2)}`;
const tEnd = computed(() => props.times.at(-1) || 1);
const x = t => margin.left + (width - margin.left - margin.right) * t / tEnd.value;

function niceStep(span) {
  const raw = span / 3, power = Math.pow(10, Math.floor(Math.log10(raw || 1)));
  for (const m of [1, 2, 2.5, 5, 10]) if (raw <= m * power) return m * power;
  return 10 * power;
}

// Value axis over the whole recording of the shown series and the references.
const range = computed(() => {
  let lo = Infinity, hi = -Infinity;
  for (const s of props.series) for (const v of s.values) if (Number.isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  for (const r of props.references) { lo = Math.min(lo, r.value); hi = Math.max(hi, r.value); }
  if (!Number.isFinite(lo)) { lo = 0; hi = 1; }
  if (props.max !== null) hi = Math.min(hi, props.max);
  if (hi - lo < 1e-9) hi = lo + 1;
  const step = niceStep(hi - lo);
  return { lo: Math.floor(lo / step) * step, hi: Math.ceil(hi / step) * step, step };
});
const y = v => margin.top + (height - margin.top - margin.bottom) * (range.value.hi - v) / (range.value.hi - range.value.lo);
const ticks = computed(() => {
  const { lo, hi, step } = range.value, out = [];
  for (let v = lo; v <= hi + step * 1e-6; v += step) out.push(v);
  return out;
});
const timeTicks = computed(() => {
  const step = niceStep(tEnd.value), out = [];
  for (let t = 0; t <= tEnd.value + 1e-9; t += step) out.push(t);
  return out;
});

// Index of the last sample at or before the playback time, and the fraction to the next one.
const position = computed(() => {
  const t = Math.max(0, Math.min(tEnd.value, props.time));
  let i = 0;
  while (i + 1 < props.times.length && props.times[i + 1] <= t) i++;
  const next = Math.min(i + 1, props.times.length - 1);
  const span = props.times[next] - props.times[i];
  return { i, next, s: span > 0 ? (t - props.times[i]) / span : 0, t };
});
const valueNow = values => {
  const { i, next, s } = position.value;
  const a = values[i], b = values[next];
  if (!Number.isFinite(a)) return null;
  return Number.isFinite(b) ? a + (b - a) * s : a;
};
const lines = computed(() => props.series.map(series => {
  const { i, t } = position.value;
  let d = '';
  for (let k = 0; k <= i; k++) {
    const v = series.values[k];
    if (!Number.isFinite(v)) continue;
    d += `${d ? 'L' : 'M'}${x(props.times[k]).toFixed(1)} ${y(v).toFixed(1)}`;
  }
  const now = valueNow(series.values);
  if (now !== null) d += `${d ? 'L' : 'M'}${x(t).toFixed(1)} ${y(now).toFixed(1)}`;
  return { ...series, d, now };
}));

function timeAt(event) {
  const box = svg.value.getBoundingClientRect();
  const px = (event.clientX - box.left) * width / box.width;
  return Math.max(0, Math.min(tEnd.value, (px - margin.left) / (width - margin.left - margin.right) * tEnd.value));
}
let dragging = false;
function down(event) { dragging = true; svg.value.setPointerCapture(event.pointerId); emit('select', timeAt(event)); }
function move(event) { if (dragging) emit('select', timeAt(event)); }
function up() { dragging = false; }
const label = v => (Math.abs(v) < 1e-9 ? 0 : v).toFixed(props.digits);
</script>

<template>
  <figure class="running-plot" :data-series="series.length">
    <figcaption>{{ title }}<span v-if="unit" class="unit"> ({{ unit }})</span></figcaption>
    <svg ref="svg" :viewBox="`0 0 ${width} ${height}`" role="img" :aria-label="`${title} up to the playback time`"
         @pointerdown="down" @pointermove="move" @pointerup="up" @pointercancel="up">
      <defs><clipPath :id="clipId"><rect :x="margin.left" :y="margin.top" :width="width - margin.left - margin.right" :height="height - margin.top - margin.bottom" /></clipPath></defs>
      <g class="grid">
        <g v-for="v in ticks" :key="`y${v}`">
          <line :x1="margin.left" :x2="width - margin.right" :y1="y(v)" :y2="y(v)" />
          <text :x="margin.left - 5" :y="y(v) + 3.5" text-anchor="end">{{ label(v) }}</text>
        </g>
        <text v-for="t in timeTicks" :key="`t${t}`" :x="x(t)" :y="height - 5" text-anchor="middle">{{ t }} s</text>
      </g>
      <g v-for="r in references" :key="r.label" class="reference">
        <line :x1="margin.left" :x2="width - margin.right" :y1="y(r.value)" :y2="y(r.value)"
              :stroke="r.color || 'var(--muted)'" :stroke-dasharray="r.dashed ? '4 3' : null" />
        <text :x="width - margin.right - 2" :y="y(r.value) - 3" text-anchor="end">{{ r.label }}</text>
      </g>
      <g v-for="s in lines" :key="s.name" :clip-path="`url(#${clipId})`">
        <path :d="s.d" :stroke="s.color" fill="none" stroke-width="1.8" stroke-linejoin="round" stroke-linecap="round" />
        <circle v-if="s.now !== null" :cx="x(position.t)" :cy="y(s.now)" r="3" :fill="s.color" />
      </g>
      <line class="cursor" :x1="x(position.t)" :x2="x(position.t)" :y1="margin.top" :y2="height - margin.bottom" />
    </svg>
  </figure>
</template>

<style scoped>
.running-plot { margin: 0; min-width: 0; }
.running-plot figcaption { font-size: 11.5px; font-weight: 600; color: var(--ink-2); margin-bottom: 2px; }
.running-plot .unit { font-weight: 400; color: var(--muted); }
.running-plot svg { display: block; width: 100%; height: auto; touch-action: none; cursor: crosshair; }
.grid line { stroke: var(--grid); stroke-width: 1; }
.grid text, .reference text { font-size: 9px; fill: var(--muted); font-family: "IBM Plex Mono", monospace; }
.reference line { stroke-width: 1; }
.cursor { stroke: var(--ink); stroke-width: 1; opacity: .6; }
</style>
