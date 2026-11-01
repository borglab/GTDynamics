<script setup>
import {computed} from 'vue';
const props=defineProps({record:Object,directory:String});
const source=computed(()=>props.record.motionSource);
const credit=computed(()=>source.value?.licenseNotes?.[0]);
</script>
<template>
  <footer>
    <div v-if="credit" class="motion-attribution"><p><a :href="source.sourceUrl">G1 · {{source.sequenceId}}</a> from <a :href="source.datasetUrl">{{source.datasetName}}</a> by {{credit.creator}}. Licensed under <a :href="credit.licenseUrl">CC BY 4.0</a>.</p><p>{{credit.changes}}</p><a :href="`./${directory}${credit.localNotice}`">Motion attribution and modifications</a></div>
    <p>SuperOdom supplies the LiDAR-inertial reference. It is an odometry estimate; loop-closure settings in the source dataset are unspecified. Contacts are inferred from kinematics.</p>
    <p>All optimization, robot transforms, factor errors, and slip measurements are computed offline. The browser replays saved results.</p>
    <details><summary>Frames, priors, and source fingerprints</summary><pre>{{JSON.stringify({imu:record.imuAiding,closure:record.loopClosure,provenance:record.provenance},null,2)}}</pre></details>
    <a :href="`./${directory}${record.availableStages?.includes('initial')?'initial.json.gz':'recording.json'}`" download>Download recording metadata</a> · <a href="./MODEL_LICENSE.txt">Robot model license</a>
  </footer>
</template>
