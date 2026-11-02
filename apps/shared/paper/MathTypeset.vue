<script setup>
defineProps({ html: { type: String, required: true } });
</script>

<template>
  <!-- HTML is trusted repository content, typeset by MathJax during the build. -->
  <div class="math-definitions" v-html="html" />
</template>

<style>
.math-definitions { font-size: 13px; line-height: 1.75; }
.math-definitions h4 { font-size: 15px; margin: 28px 0 12px; }
.math-definitions p { max-width: 110ch; margin: 12px 0; }
.math-definitions mjx-container { max-width: 100%; }
.math-definitions mjx-container[display="true"] { display: block; overflow-x: auto; overflow-y: hidden; padding: 8px 0; text-align: left; }
.math-definitions svg { max-width: none; }
.math-parameters { overflow-x: auto; margin: 20px 0; font-size: 12px; line-height: 1.6; }
.math-parameters table { border-collapse: collapse; min-width: min(100%, 450px); font-variant-numeric: tabular-nums; }
.math-parameters caption { text-align: left; font-weight: 600; margin-bottom: 10px; }
.math-parameters th, .math-parameters td { padding: 7px 14px 7px 0; border-bottom: 1px solid var(--border, #dbe2d7); text-align: left; }
@media print { .math-definitions mjx-container, .math-parameters table { break-inside: avoid; } }
</style>
