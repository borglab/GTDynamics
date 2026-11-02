<script setup>
import { computed } from 'vue';
import MathTypeset from './MathTypeset.vue';
import definitions from './equations/contact-labels.math.html?math';
const props = defineProps({
  graph: { type: Object, required: true },
  velocityAiding: { type: Boolean, default: true },
  walkingConstraints: { type: Boolean, default: true },
});
const number = value => Number(Number(value).toPrecision(6)).toString();
const distances = computed(() => [0, 3, 10, 20].map(mm => {
  const density = sigma => Math.sqrt(2 / Math.PI) / sigma * Math.exp(-0.5 * (mm / 1000 / sigma) ** 2);
  const s = density(props.graph.supportSigma), w = density(props.graph.swingSigma);
  return { mm, s, w, ratio: s / w };
}));
const velocities = computed(() => [-0.5, -0.3, -0.1, 0, 0.1, 0.3, 0.5].map(v => {
  const stance = Math.max(props.graph.minimumProbability, Math.min(1 - props.graph.minimumProbability, 1 / (1 + Math.exp(v / props.graph.velocityScale))));
  return { v, stance, swing: 1 - stance };
}));
const factorCount = computed(() => (props.velocityAiding ? 8 : 6) * props.graph.sliceCount - 6 + (props.walkingConstraints ? 2 * props.graph.sliceCount - 1 : 0));
</script>

<template>
  <section class="contact-mathematics" aria-label="Contact factor definitions">
    <MathTypeset :html="definitions" />
    <div class="math-parameters" data-math-parameters="contact">
      <table><caption>Parameters from the selected recording</caption><thead><tr><th>Symbol / quantity</th><th>Value</th></tr></thead><tbody>
        <tr><td>σs · stationary displacement scale</td><td>{{graph.supportSigma}} m</td></tr>
        <tr><td>σw · swing displacement scale</td><td>{{graph.swingSigma}} m</td></tr>
        <tr><td>sv · velocity scale</td><td>{{graph.velocityScale}} m/s</td></tr>
        <tr><td>ε · velocity weight floor</td><td>{{graph.minimumProbability}}</td></tr>
        <tr><td>η · simultaneous-swing weight</td><td>{{graph.noSupportWeight}} (stance odds multiplier {{1/graph.noSupportWeight}})</td></tr>
        <tr><td>ρ · isolated-slice weight</td><td>{{graph.singleSliceWeight}} (extra cost {{number(-Math.log(graph.singleSliceWeight))}})</td></tr>
        <tr><td>Sampling interval</td><td>{{1000*graph.sampleIntervalSeconds}} ms</td></tr>
        <tr><td>Inference extent</td><td>{{graph.sliceCount}} slices · {{graph.variableCount}} variables · {{graph.contextSamplesPerEnd}} context slices per end</td></tr>
        <tr><td>Selected graph</td><td>{{factorCount}} factors · velocity {{velocityAiding?'on':'off'}} · hard walking constraints {{walkingConstraints?'on':'off'}}</td></tr>
      </tbody></table>
      <table><caption>Transition potential A · rows and columns WW, WS, SW, SS</caption><thead><tr><th>From / to</th><th v-for="state in ['WW','WS','SW','SS']" :key="state">{{state}}</th></tr></thead><tbody>
        <tr v-for="(row,i) in graph.transition" :key="i"><th>{{['WW','WS','SW','SS'][i]}}</th><td v-for="(value,j) in row" :key="j">{{value}}</td></tr>
      </tbody></table>
      <table><caption>Displacement density examples · not recorded samples</caption><thead><tr><th>d (mm)</th><th>S(d) (m⁻¹)</th><th>W(d) (m⁻¹)</th><th>S/W</th></tr></thead><tbody>
        <tr v-for="row in distances" :key="row.mm"><td>{{row.mm}}</td><td>{{number(row.s)}}</td><td>{{number(row.w)}}</td><td>{{number(row.ratio)}}</td></tr>
      </tbody></table>
      <table><caption>Velocity compatibility examples · not contact posteriors</caption><thead><tr><th>v (m/s)</th><th>Stance weight</th><th>Swing weight</th></tr></thead><tbody>
        <tr v-for="row in velocities" :key="row.v"><td>{{row.v}}</td><td>{{row.stance.toFixed(6)}}</td><td>{{row.swing.toFixed(6)}}</td></tr>
      </tbody></table>
    </div>
  </section>
</template>
