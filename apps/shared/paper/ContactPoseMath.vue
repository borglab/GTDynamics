<script setup>
import MathTypeset from './MathTypeset.vue';
import definitions from './equations/contact-pose.math.html?math';
defineProps({ graph: { type: Object, default: null } });
const number = value => Number(value).toLocaleString(undefined, { maximumFractionDigits: 5 });
</script>

<template>
  <section aria-label="Pose contact factor definitions">
    <MathTypeset :html="definitions" />
    <div v-if="graph?.noise" class="math-parameters" data-math-parameters="initialization">
      <table><caption>Selected initializer · standard deviations and active priors</caption><thead><tr><th>Residual</th><th>Standard deviation per component</th><th>Active</th></tr></thead><tbody>
        <tr><td>Contact pose</td><td>{{number(graph.noise.contactRotationDegrees)}}° / {{number(graph.noise.contactTranslation)}} m</td><td>Yes</td></tr>
        <tr><td>Root upright</td><td>{{number(graph.noise.uprightDegrees)}}° (direction scale)</td><td>{{graph.upright?'Yes':'No'}}</td></tr>
        <tr><td>Landmark height</td><td>{{number(graph.noise.height)}} m · target {{graph.groundHeight}} m</td><td>{{graph.ground?'Yes':'No'}}</td></tr>
        <tr><td>Identity root motion</td><td>{{number(graph.noise.betweenRotationDegrees)}}° / {{number(graph.noise.betweenTranslation)}} m</td><td>{{graph.between?'Yes':'No'}}</td></tr>
        <tr><td>Sole normal</td><td>{{number(graph.noise.normalDegrees)}}° (direction scale)</td><td>{{graph.normal?'Yes':'No'}}</td></tr>
      </tbody></table>
    </div>
  </section>
</template>
