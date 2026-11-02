<script setup>
import { computed } from 'vue';
const props = defineProps({ record: Object, time: Number });
const emit = defineEmits(['select']);
const interval = computed(() => {
  const times = props.record.knots.times;
  let low = 0, high = times.length - 1;
  while (low + 1 < high) {
    const middle = (low + high) >>> 1;
    if (times[middle] <= props.time) low = middle;
    else high = middle;
  }
  return Math.min(low, times.length - 2);
});
const start = computed(() => Math.max(0, Math.min(interval.value - 1, props.record.knots.times.length - 3)));
const slices = computed(() => Array.from({ length: 3 }, (_, index) => ({
  index: start.value + index, x: 180 + index * 330,
  time: props.record.knots.times[start.value + index],
})));
const edges = computed(() => slices.value.slice(0, -1).map((slice, index) => ({
  index: slice.index, x: slice.x + 165, left: slice.x, right: slices.value[index + 1].x,
  measurements: props.record.knots.frames[slice.index + 1] - props.record.knots.frames[slice.index] - 1,
})));
const cursorX = computed(() => {
  const i = interval.value, times = props.record.knots.times;
  return 180 + (i - start.value + (props.time - times[i]) / (times[i + 1] - times[i])) * 330;
});
</script>
<template>
  <figure class="factor-graph">
    <div class="graph-title"><strong>Velocity graph · {{ time.toFixed(2) }} s</strong>
      <span>Click a slice to move playback</span></div>
    <div class="graph-scroll">
      <svg viewBox="0 0 1020 365" role="group" aria-label="Reduced velocity factor graph, synchronized with playback"
        :data-interval="interval">
        <title>Two independent velocity chains with fixed configurations substituted</title>
        <rect v-for="edge in edges" :key="`highlight-${edge.index}`" :x="edge.left" y="17"
          :width="edge.right - edge.left" height="325" class="interval-highlight"
          :opacity="edge.index === interval ? 1 : 0" />
        <text x="18" y="111" class="graph-label">Base twist</text>
        <text x="18" y="244" class="graph-label">Joint rates</text>
        <g v-for="edge in edges" :key="edge.index">
          <line :x1="edge.left" y1="108" :x2="edge.right" y2="108" />
          <rect :x="edge.x - 17" y="91" width="34" height="34" class="process-factor" />
          <text :x="edge.x" y="113" class="factor-symbol">G<tspan baseline-shift="sub" font-size="9">B</tspan></text>
          <line :x1="edge.left" y1="241" :x2="edge.right" y2="241" />
          <rect :x="edge.x - 17" y="224" width="34" height="34" class="process-factor" />
          <text :x="edge.x" y="246" class="factor-symbol">G<tspan baseline-shift="sub" font-size="9">J</tspan></text>
          <template v-if="edge.measurements > 0">
            <path :d="`M ${edge.left} 241 L ${edge.x} 302 L ${edge.right} 241`" />
            <rect :x="edge.x - 14" y="288" width="28" height="28" class="measurement-factor" />
            <text :x="edge.x" y="307" class="factor-symbol">M</text>
            <text :x="edge.x" y="333" class="graph-note">{{ edge.measurements }} separate encoder factors</text>
          </template>
        </g>
        <g v-for="slice in slices" :key="slice.index">
          <g role="button" tabindex="0" :aria-label="`Go to knot ${slice.index} at ${slice.time.toFixed(2)} seconds`"
            class="graph-slice" @click="emit('select', slice.time)" @keydown.enter.prevent="emit('select', slice.time)"
            @keydown.space.prevent="emit('select', slice.time)">
            <title>Move playback to this fixed knot</title>
            <rect :x="slice.x - 65" y="17" width="130" height="30" rx="4" class="slice-title" />
            <text :x="slice.x" y="37" class="graph-note">k = {{ slice.index }} · {{ slice.time.toFixed(2) }} s</text>
          </g>
          <line :x1="slice.x" y1="69" :x2="slice.x" y2="108" />
          <rect :x="slice.x - 12" y="57" width="24" height="24" class="measurement-factor" />
          <text :x="slice.x" y="74" class="factor-symbol">V</text>
          <circle :cx="slice.x" cy="108" r="20" class="velocity-variable" />
          <text :x="slice.x" y="113" class="variable-symbol">ν<tspan baseline-shift="sub" font-size="10">{{ slice.index }}</tspan></text>
          <text :x="slice.x" y="151" class="graph-note">T<tspan baseline-shift="sub">{{ slice.index }}</tspan> fixed · world_T_COM</text>
          <text :x="slice.x" y="195" class="graph-note">q<tspan baseline-shift="sub">{{ slice.index }}</tspan> fixed · 29 angles</text>
          <circle :cx="slice.x" cy="241" r="20" class="velocity-variable" />
          <text :x="slice.x" y="246" class="variable-symbol">r<tspan baseline-shift="sub" font-size="10">{{ slice.index }}</tspan></text>
        </g>
        <line :x1="cursorX" y1="50" :x2="cursorX" y2="342" class="playback-cursor" />
      </svg>
    </div>
    <figcaption>Circles: velocity variables. Squares: soft factors. Fixed T and q are substituted inputs,
      not unknown nodes or finite-noise priors. G<sub>B</sub> and G<sub>J</sub> are base and joint WNOA factors;
      V observes base world-linear velocity. Each M symbol groups the stated number of separate encoder factors
      with the same scope. The highlighted interval and dashed cursor follow playback. This is a three-knot
      excerpt of the actual graph; no contact or acceleration variables are hidden in the diagram.</figcaption>
  </figure>
</template>
