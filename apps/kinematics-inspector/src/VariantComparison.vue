<script setup>
defineProps({summary:Object,selected:String});
defineEmits(['select']);
const number=(value,digits=3)=>value==null?'—':value!==0&&Math.abs(value)<.001?value.toExponential(2):value.toLocaleString(undefined,{minimumFractionDigits:digits,maximumFractionDigits:digits});
</script>
<template>
  <section class="variant-comparison" aria-labelledby="comparison-heading">
    <h2 id="comparison-heading">All checkbox combinations · final results</h2>
    <p class="scope">The table ranks results by SE(3)-aligned pelvis-position RMS on the same {{summary.variants[0].samples}} slices. Factor costs are ½‖whitened residual‖²; each cell shows the total and mean per factor. The View button displays the corresponding result.</p>
    <div class="comparison-scroll"><table class="comparison-table">
      <thead><tr><th scope="col">Rank / view</th><th scope="col">Upright</th><th scope="col">Height</th><th scope="col">IMU</th><th scope="col">Loop</th><th scope="col">Aligned pelvis RMS<small>m · ranking metric</small></th><th scope="col">Original pelvis RMS<small>m</small></th><th scope="col">Slip RMS<small>mm / interval</small></th><th scope="col">Height RMS<small>mm</small></th><th scope="col">Joint RMS<small>degrees</small></th><th scope="col">COM reference RMS<small>m · original frame</small></th><th scope="col">Equality violation<small>L2</small></th><th scope="col">End − start<small>XYZ · m</small></th><th scope="col">Total objective</th><th v-for="group in summary.factorGroups" :key="group.id" scope="col" :title="group.factorType">{{group.label}}<small>total / per factor</small></th><th scope="col">Iterations</th><th scope="col">Solve time<small>s</small></th></tr></thead>
      <tbody><tr v-for="(row,index) in summary.variants" :key="row.id" :data-variant="row.id" :class="{selected:row.id===selected,best:row.id===summary.bestVariant}" :aria-current="row.id===selected?'true':undefined">
        <th scope="row">{{index+1}} <span v-if="row.id===summary.bestVariant">Best</span><button :aria-label="`View ${row.id}`" @click="$emit('select',row)">View</button></th>
        <td v-for="key in ['upright','height','imu','loop']" :key="key" :aria-label="row[key]?'On':'Off'">{{row[key]?'✓':'—'}}</td>
        <td class="ranking-error">{{number(row.alignedRmsMeters)}}</td><td>{{number(row.referenceRmsMeters)}}</td><td>{{number(row.metrics.stance_step_slip_rms_m*1000)}}</td><td>{{number(row.metrics.stance_height_rms_m*1000)}}</td><td>{{number(row.metrics.joint_measurement_rms_rad*180/Math.PI)}}</td><td>{{number(row.metrics.reference_root_translation_rms_m)}}</td><td>{{number(row.metrics.kinematic_equality_l2)}}</td><td>{{row.closurePositionErrorWorld.map(value=>number(value)).join(', ')}}</td><td>{{number(row.metrics.cost)}}</td>
        <td v-for="group in summary.factorGroups" :key="group.id"><template v-if="row.factorErrors[group.id]!=null">{{number(row.factorErrors[group.id])}}<small>{{number(row.factorErrors[group.id]/row.factorCounts[group.id])}}</small></template><template v-else>—</template></td>
        <td>{{row.iterations}}</td><td>{{number(row.elapsedSeconds,2)}}</td>
      </tr></tbody>
    </table></div>
  </section>
</template>
<style scoped>
.variant-comparison{background:var(--dark-panel, white);border:1px solid var(--dark-border, #dbe2d7);border-radius:10px;padding:22px;margin-bottom:20px;min-width:0}.variant-comparison h2{font-size:18px}.comparison-scroll{overflow:auto;max-height:630px;margin-top:15px}.comparison-table{border-collapse:separate;border-spacing:0;font-size:12px;font-variant-numeric:tabular-nums;width:100%}.comparison-table th,.comparison-table td{padding:9px 12px;text-align:right;border-bottom:1px solid var(--dark-border, #e7ece3);white-space:nowrap}.comparison-table thead th{position:sticky;top:0;background:var(--dark-page, #f2f5ef);z-index:1}.comparison-table th:first-child{text-align:left;position:sticky;left:0;background:var(--dark-panel, white)}.comparison-table thead th:first-child{z-index:2;background:var(--dark-page, #f2f5ef)}.comparison-table small{display:block;font-size:10px;font-weight:400;color:var(--dark-secondary, #6b7971);margin-top:4px}.comparison-table button{margin-left:10px;font-size:11px;padding:4px 7px}.comparison-table .selected td,.comparison-table .selected th{background:var(--dark-surface, #eaf3ec)}.comparison-table .best .ranking-error{font-weight:700;color:#087f71}.comparison-table th span{font-size:10px;color:#087f71;margin-left:5px}@media(max-width:600px){.variant-comparison{padding:14px}.comparison-scroll{max-height:500px}}
</style>
