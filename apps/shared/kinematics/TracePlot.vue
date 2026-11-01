<script setup>
import { computed } from 'vue';
const props = defineProps({ values: Array, secondary: Array, selected: Number, label: String, interactive: Boolean });
const emit = defineEmits(['select']);
const width = 440, height = 105, pad = 12;
const bounds = computed(() => {
  const all = [...props.values, ...(props.secondary || [])].filter(Number.isFinite);
  const lo = Math.min(...all), hi = Math.max(...all);
  return { lo, span: Math.max(hi - lo, Math.abs(hi) * .01, 1e-9) };
});
const x = i => pad + i * (width - 2 * pad) / Math.max(1, props.values.length - 1);
const y = v => height - pad - (v - bounds.value.lo) * (height - 2 * pad) / bounds.value.span;
const points = values => values.map((v, i) => `${x(i)},${y(v)}`).join(' ');
function select(event) {
  if (!props.interactive) return;
  const box = event.currentTarget.getBoundingClientRect();
  const position = (event.clientX - box.left) / box.width * width;
  emit('select', Math.max(0, Math.min(props.values.length - 1, Math.round((position - pad) / (width - 2 * pad) * (props.values.length - 1)))));
}
</script>
<template>
  <svg class="trace" :viewBox="`0 0 ${width} ${height}`" role="img" :aria-label="label" @click="select">
    <line :x1="pad" :x2="width-pad" :y1="height-pad" :y2="height-pad" class="baseline" />
    <polyline v-if="secondary?.length" :points="points(secondary)" class="reference-line" />
    <polyline :points="points(values)" class="value-line" />
    <line :x1="x(selected)" :x2="x(selected)" y1="4" :y2="height-pad" class="cursor-line" />
    <circle :cx="x(selected)" :cy="y(values[selected])" r="4" class="cursor-point" />
  </svg>
</template>
