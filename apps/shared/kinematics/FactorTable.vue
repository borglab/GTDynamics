<script setup>
import {computed} from 'vue';
const props=defineProps({record:Object,state:Object});
const total=computed(()=>Object.values(props.state.factorErrors).reduce((sum,value)=>sum+value,0));
const number=value=>value===0?'0':Math.abs(value)<.001?value.toExponential(2):value.toLocaleString(undefined,{maximumFractionDigits:3});
</script>
<template>
  <section class="factor-section">
    <div class="row"><h2>Factor graph and error budget</h2><span class="mono">Iteration {{state.iteration}} · objective {{number(total)}}</span></div>
    <p class="scope">Each cost is ½‖whitened residual‖², evaluated by the native factor graph at the selected iteration. Counts and types describe the actual graph.</p>
    <div class="table-scroll"><table class="factor-table"><thead><tr><th scope="col">Objective / factor type</th><th scope="col">Factors</th><th scope="col">Residual dimensions</th><th scope="col">σ / covariance</th><th scope="col">Error</th><th scope="col">Share</th></tr></thead><tbody>
      <tr v-for="group in record.graph.factorGroups" :key="group.id" :data-factor="group.id"><th scope="row">{{group.label}}<small>{{group.factorType}}</small></th><td>{{group.count.toLocaleString()}}</td><td>{{(group.count*group.dimension).toLocaleString()}}</td><td>{{group.noise}}</td><td class="factor-error">{{number(state.factorErrors[group.id])}}</td><td>{{total>0?(100*state.factorErrors[group.id]/total).toFixed(1):'0'}}%</td></tr>
    </tbody><tfoot><tr><th scope="row">Total soft cost</th><td>{{record.graph.costFactors.toLocaleString()}}</td><td>{{record.graph.factorGroups.reduce((sum,g)=>sum+g.count*g.dimension,0).toLocaleString()}}</td><td></td><td>{{number(total)}}</td><td>100%</td></tr></tfoot></table></div>
    <div class="equality-summary"><strong>Kinematic equality constraints</strong><span>ExpressionEqualityConstraint&lt;Vector6&gt;</span><span>{{record.graph.equalities.toLocaleString()}} factors · {{record.graph.equalityDimension.toLocaleString()}} residual dimensions</span><span>Violation L2: {{number(state.metrics.kinematic_equality_l2)}}</span><span>Enforced by CM-Opt; excluded from the soft-cost total. No inequalities.</span></div>
  </section>
</template>
