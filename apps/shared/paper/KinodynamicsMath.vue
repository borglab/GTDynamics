<script setup>
import MathTypeset from './MathTypeset.vue';
import definitions from './equations/kinodynamics.math.html?math';
</script>

<template>
  <section aria-label="Coupled kinodynamics factor definitions">
    <MathTypeset :html="definitions" />
  </section>
</template>
