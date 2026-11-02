<script setup>
import { computed } from 'vue';
import { papers } from './content.js';
import PaperGraph from './PaperGraph.vue';

const props = defineProps({
  app: { type: String, required: true },
  frame: { type: Number, default: 0 },
  samples: { type: Number, default: 3 },
});
defineEmits(['select']);
const paper = computed(() => papers[props.app]);
</script>

<template>
  <article class="app-paper" :aria-label="`${app} method and results`">
    <section class="paper-section" data-paper-section="motivation">
      <h2>Motivation</h2>
      <p>{{ paper.motivation }}</p>
    </section>
    <section class="paper-section" data-paper-section="approach">
      <h2>Factor graph formulation</h2>
      <p>{{ paper.approach }}</p>
      <slot name="graph">
        <PaperGraph :kind="paper.graph" :frame="frame" :samples="samples" @select="$emit('select', $event)" />
      </slot>
      <h3>Factors and constraints</h3>
      <dl class="factor-definitions">
        <template v-for="factor in paper.factors" :key="factor.id">
          <dt>{{ factor.name }}</dt>
          <dd>{{ factor.description }}</dd>
        </template>
      </dl>
    </section>
    <section class="paper-section paper-results" data-paper-section="results">
      <h2>Results</h2>
      <slot />
    </section>
    <section class="paper-section" data-paper-section="discussion">
      <h2>Discussion</h2>
      <p>{{ paper.discussion }}</p>
      <slot name="discussion" />
      <p v-if="paper.next" class="next-app">
        <a :href="`../#${paper.next}`" target="_top">Next: {{ paper.nextTitle }} →</a>
      </p>
    </section>
  </article>
</template>

<style scoped>
.app-paper { margin-top: 32px; }
.app-paper > .paper-section {
  padding: 24px 0;
  margin: 0;
  border: 0;
  border-top: 1px solid var(--border, #dbe2d7);
  border-radius: 0;
  background: transparent;
}
.paper-section > h2 { font-size: 20px; margin: 0 0 14px; }
.paper-section > h3 { font-size: 16px; margin: 24px 0 16px; }
.paper-section > p, .factor-definitions {
  max-width: 110ch;
  font-size: 13px;
  line-height: 1.75;
}
.paper-section > p { margin: 12px 0; }
.factor-definitions { margin: 0; }
.factor-definitions dt { margin-top: 18px; font-weight: 600; }
.factor-definitions dd { margin: 5px 0 0; }
.next-app a { color: inherit; text-underline-offset: 4px; }
</style>
