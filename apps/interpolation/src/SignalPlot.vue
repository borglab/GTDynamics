<script setup>
import { nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue';

const props = defineProps({ title: String, unit: String, times: Array, series: Array, time: Number, range: Array });
const emit = defineEmits(['select']);
const element = ref(null), error = ref('');
let Plotly, observer, disposed = false, ready = false;

async function draw() {
  if (!Plotly || disposed) return;
  await Plotly.react(element.value, props.series.map(series => ({
    type: 'scatter', mode: series.mode || 'lines', name: series.name,
    x: series.times || props.times, y: series.values,
    line: { color: series.color, width: 1.6, dash: series.dash },
    marker: { size: 5, color: series.color },
  })), {
    height: 260, margin: { l: 62, r: 15, t: 8, b: 52 },
    paper_bgcolor: '#fff', plot_bgcolor: '#fff',
    font: { family: 'system-ui, sans-serif', size: 11, color: '#52625b' },
    xaxis: { title: { text: 'Time (s)' }, range: props.range, gridcolor: '#e5eae3' },
    yaxis: { title: { text: props.unit }, gridcolor: '#e5eae3', automargin: true },
    legend: { orientation: 'h', y: -0.25 }, hovermode: 'x unified',
    dragmode: 'zoom', uirevision: `${props.title}-${props.unit}`,
    shapes: [{ type: 'line', xref: 'x', yref: 'paper', x0: props.time, x1: props.time,
      y0: 0, y1: 1, line: { color: '#82938a', width: 1, dash: 'dot' } }],
  }, { scrollZoom: true, displaylogo: false, responsive: false });
  if (!disposed) element.value.dataset.ready = 'true';
}

onMounted(async () => {
  try {
    Plotly = (await import('plotly.js-basic-dist-min')).default;
    if (disposed) return;
    await draw();
    if (disposed) return;
    ready = true;
    element.value.on('plotly_click', event => {
      if (event.points?.length) emit('select', event.points[0].x);
    });
    observer = new ResizeObserver(() => { if (!disposed) Plotly.Plots.resize(element.value); });
    observer.observe(element.value);
  } catch (e) { if (!disposed) error.value = e.message; }
});
watch(() => props.series, async () => { await nextTick(); if (ready) draw(); });
watch(() => props.time, time => {
  if (ready && !disposed) Plotly.relayout(element.value, { 'shapes[0].x0': time, 'shapes[0].x1': time });
});
watch(() => props.range, range => {
  if (ready && !disposed) Plotly.relayout(element.value, { 'xaxis.range': range, 'yaxis.autorange': true });
});
onBeforeUnmount(() => {
  disposed = true;
  observer?.disconnect();
  if (Plotly && element.value) Plotly.purge(element.value);
});
</script>

<template>
  <figure class="signal-plot">
    <figcaption>{{ title }}</figcaption>
    <p v-if="error" role="alert">{{ error }}</p>
    <div ref="element" class="plotly-chart" :aria-label="title" />
  </figure>
</template>
