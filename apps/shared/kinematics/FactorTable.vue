<script setup>
import {computed} from 'vue';
const props=defineProps({record:Object,state:Object,estimate:{type:String,default:'final'}});
const total=computed(()=>Object.values(props.state.factorErrors).reduce((sum,value)=>sum+value,0));
const maxError=computed(()=>Math.max(...Object.values(props.state.factorErrors),1e-20));
const perFactor=group=>group.count?props.state.factorErrors[group.id]/group.count:0;
const maxPerFactor=computed(()=>Math.max(...props.record.graph.factorGroups.map(perFactor),1e-20));
const number=value=>value===0?'0':Math.abs(value)<.001?value.toExponential(2):value.toLocaleString(undefined,{maximumFractionDigits:3});
</script>
<template>
  <section class="factor-section">
    <div class="row"><h2>Factor graph and error budget</h2><span class="mono">{{estimate==='initial'?'Initial':'Final'}} objective {{number(total)}}</span></div>
    <p class="scope">Error is ½‖whitened residual‖² at the {{estimate}} estimate. Error per factor is the group total divided by its factor count, not its residual dimension. Bars in each column share a linear scale from zero to that column's largest value.</p>
    <div class="table-scroll"><table class="factor-table"><thead><tr><th scope="col">Objective / factor type</th><th scope="col">Factors</th><th scope="col">Residual dimensions</th><th scope="col">σ / covariance</th><th scope="col">Total error</th><th scope="col">Error per factor</th><th scope="col">Share</th></tr></thead><tbody>
      <tr v-for="group in record.graph.factorGroups" :key="group.id" :data-factor="group.id"><th scope="row">{{group.label}}<small>{{group.factorType}}</small></th><td>{{group.count.toLocaleString()}}</td><td>{{(group.count*group.dimension).toLocaleString()}}</td><td>{{group.noise}}</td><td class="factor-error"><span>{{number(state.factorErrors[group.id])}}</span><div class="error-bar total-error" role="img" :aria-label="`${group.label}: total error ${state.factorErrors[group.id]}`"><i :style="{width:`${100*state.factorErrors[group.id]/maxError}%`}"></i></div></td><td class="factor-average"><span>{{number(perFactor(group))}}</span><div class="error-bar average-error" role="img" :aria-label="`${group.label}: error per factor ${perFactor(group)}`"><i :style="{width:`${100*perFactor(group)/maxPerFactor}%`}"></i></div></td><td>{{total>0?(100*state.factorErrors[group.id]/total).toFixed(1):'0'}}%</td></tr>
    </tbody><tfoot><tr><th scope="row">Total soft cost</th><td>{{record.graph.costFactors.toLocaleString()}}</td><td>{{record.graph.factorGroups.reduce((sum,g)=>sum+g.count*g.dimension,0).toLocaleString()}}</td><td></td><td>{{number(total)}}</td><td>{{number(total/record.graph.costFactors)}}</td><td>100%</td></tr></tfoot></table></div>
    <div class="equality-summary"><strong>Kinematic equality constraints</strong><span>ExpressionEqualityConstraint&lt;Vector6&gt;</span><span>{{record.graph.equalities.toLocaleString()}} factors · {{record.graph.equalityDimension.toLocaleString()}} residual dimensions</span><span>Violation L2: {{number(state.metrics.kinematic_equality_l2)}}</span><span>Enforced by CM-Opt; excluded from the soft-cost total. No inequalities.</span></div>
  </section>
</template>
