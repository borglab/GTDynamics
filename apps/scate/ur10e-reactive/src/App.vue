<script setup>
/** Reactive SCATE on the UR10e, comparing base-disturbance costs; the page is shared with the other SCATE apps. */
import ScateReplayApp from '../../shared/ScateReplayApp.vue';
</script>

<template>
  <ScateReplayApp app="scate/ur10e-reactive" robot="UR10e" title="SCATE UR10e: base disturbance" />
</template>
