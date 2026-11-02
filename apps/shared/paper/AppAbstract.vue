<script setup>
import { papers } from './content.js';
defineProps({ app: { type: String, required: true } });
</script>

<template>
  <p class="app-abstract">{{ papers[app].abstract }}</p>
</template>

<style scoped>
.app-abstract {
  max-width: 110ch;
  font-size: 13px;
  line-height: 1.6;
  margin: 6px 0 0;
}
</style>
