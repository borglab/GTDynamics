<script setup>
import { computed } from 'vue';
import { graphSlices, paperGraph } from './graph.js';

const props = defineProps({ kind: String, frame: Number, samples: Number });
const emit = defineEmits(['select']);
const graph = computed(() => paperGraph(props.kind, graphSlices(props.frame, props.samples)));
const nodes = computed(() => new Map(graph.value.nodes.map(node => [node.id, node])));
</script>

<template>
  <figure class="paper-graph">
    <div class="paper-graph-scroll">
      <svg :viewBox="`0 0 900 ${graph.height}`" role="group" :aria-label="`${kind} factor graph schematic`">
        <g v-for="factor in graph.factors" :key="factor.id" class="graph-factor">
          <title>{{ factor.title }}</title>
          <line v-for="key in factor.keys" :key="key" :x1="factor.x" :y1="factor.y" :x2="nodes.get(key).x" :y2="nodes.get(key).y" />
          <rect :x="factor.x - 6" :y="factor.y - 6" width="12" height="12" />
          <text :x="factor.x + 11" :y="factor.y - 9">{{ factor.code }}</text>
        </g>
        <g v-for="node in graph.nodes" :key="node.id" class="graph-variable"
          :class="{ selected: node.slice === frame, interactive: node.slice !== undefined }"
          :role="node.slice !== undefined ? 'button' : undefined"
          :tabindex="node.slice !== undefined ? 0 : undefined"
          :aria-label="node.slice !== undefined ? `Show slice ${node.slice} in demo (${node.label})` : node.label"
          @click="node.slice !== undefined && emit('select', node.slice)"
          @keydown.enter="node.slice !== undefined && emit('select', node.slice)"
          @keydown.space.prevent="node.slice !== undefined && emit('select', node.slice)">
          <circle :cx="node.x" :cy="node.y" r="23" />
          <text :x="node.x" :y="node.y + 5">{{ node.label }}</text>
        </g>
      </svg>
    </div>
    <figcaption>
      Circles: variables. Squares: factors. {{ graph.caption }}
      <template v-if="graph.nodes.some(node => node.slice !== undefined)">
        This is a structural schematic; optional factors are shown for explanation and may be disabled in the selected run.
        Select a numbered variable to update the demo’s time slice.
      </template>
      Hover a square for its definition.
      <a href="#demo">Return to demo ↑</a>
    </figcaption>
  </figure>
</template>

<style scoped>
.paper-graph { margin: 24px 0; }
.paper-graph-scroll { overflow-x: auto; border-block: 1px solid var(--border, #dbe2d7); }
svg { display: block; width: 100%; max-height: 420px; min-width: 600px; color: inherit; }
line { stroke: currentColor; stroke-width: 1.1; opacity: .35; }
rect { fill: currentColor; }
text { fill: currentColor; font: 12px ui-monospace, monospace; }
.graph-factor text { font-size: 10px; }
circle { fill: var(--panel, #fff); stroke: currentColor; stroke-width: 1.5; }
.graph-variable text { text-anchor: middle; pointer-events: none; }
.interactive { cursor: pointer; }
.selected circle, .interactive:focus circle { stroke-width: 4; }
figcaption { font-size: 12px; line-height: 1.65; margin-top: 12px; opacity: .8; }
</style>
