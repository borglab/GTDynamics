<script setup>
import { computed, nextTick, ref, watch } from "vue";

const props = defineProps({ graph: Object, selected: Number });
const emit = defineEmits(["select"]);
const scroller = ref(null);
const frames = computed(() => props.graph.frames);
const width = computed(() => Math.max(560, frames.value.length * 58 + 30));
const frameX = (i) => 32 + 58 * i;
const contactY = (c) => (c.foot === "left" ? 175 : 265);
const positions = computed(() => {
  const indices = new Map();
  frames.value.forEach((f, i) =>
    f.observations.forEach((o) => {
      const span = indices.get(o.contactId) ?? [i, i];
      span[1] = i;
      indices.set(o.contactId, span);
    }),
  );
  return new Map([...indices].map(([id, [a, b]]) => [id, frameX((a + b) / 2)]));
});
const active = computed(
  () =>
    new Set(frames.value[props.selected]?.observations.map((o) => o.contactId)),
);
const count = (id) =>
  props.graph.factorGroups.find((g) => g.id === id)?.count ?? 0;
watch(
  () => props.selected,
  async () => {
    await nextTick();
    const host = scroller.value,
      x = frameX(props.selected);
    if (
      host &&
      (x < host.scrollLeft + 30 || x > host.scrollLeft + host.clientWidth - 30)
    )
      host.scrollLeft = Math.max(0, x - host.clientWidth / 2);
  },
);
</script>

<template>
  <section class="graph-card pose-graph">
    <h2>Pose/contact graph</h2>
    <p class="graph-stats">
      {{ frames.length }} root poses + {{ graph.contacts.length }} contact poses
      · {{ count("contact") }} observations
    </p>
    <div ref="scroller" class="graph-scroll">
      <svg
        :width="width"
        :viewBox="`0 0 ${width} 320`"
        height="320"
        role="img"
        aria-label="Bipartite graph of root poses and stance contact poses"
      >
        <text x="16" y="18" class="row-label">Root poses · time →</text>
        <g v-for="(f, i) in frames" :key="f.sampleIndex">
          <g v-if="graph.between && i" class="motion-factor">
            <title>Weak motion {{ i - 1 }} → {{ i }}</title>
            <line :x1="frameX(i - 1)" y1="64" :x2="frameX(i)" y2="64" />
            <rect :x="frameX(i) - 32" y="61" width="6" height="6" />
          </g>
          <g v-if="graph.upright && i" class="upright-factor">
            <title>Pelvis upright {{ i }}</title>
            <line :x1="frameX(i)" y1="48" :x2="frameX(i)" y2="31" />
            <rect :x="frameX(i) - 3" y="25" width="6" height="6" />
          </g>
          <g
            v-for="o in f.observations"
            :key="o.contactId"
            :class="[
              'observation-factor',
              graph.contacts[o.contactId].foot,
              { highlight: i === selected },
            ]"
          >
            <line
              :x1="frameX(i)"
              y1="80"
              :x2="positions.get(o.contactId)"
              :y2="contactY(graph.contacts[o.contactId]) - 13"
            />
            <rect
              :x="(frameX(i) + positions.get(o.contactId)) / 2 - 3"
              :y="(80 + contactY(graph.contacts[o.contactId]) - 13) / 2 - 3"
              width="6"
              height="6"
            />
          </g>
        </g>
        <g
          v-for="(f, i) in frames"
          :key="f.sampleIndex"
          class="pose-node"
          :class="{ selected: i === selected }"
          role="button"
          tabindex="0"
          :aria-label="`Select event ${i}`"
          @click="emit('select', i)"
          @keydown.enter="emit('select', i)"
          @keydown.space.prevent="emit('select', i)"
        >
          <circle :cx="frameX(i)" cy="64" r="16" />
          <text :x="frameX(i)" y="68">
            X
            <tspan baseline-shift="sub" font-size="8">{{ i }}</tspan>
          </text>
          <text :x="frameX(i)" y="100" class="node-time">
            {{ f.time.toFixed(2) }}s
          </text>
          <path
            v-if="graph.anchorFrameIndices.includes(i)"
            :d="`M${frameX(i) - 7} 39h14m-7 0v-10`"
            class="anchor-mark"
          />
        </g>
        <g
          v-for="c in graph.contacts"
          :key="c.id"
          :class="['contact-node', c.foot, { active: active.has(c.id) }]"
        >
          <circle :cx="positions.get(c.id)" :cy="contactY(c)" r="15" />
          <text :x="positions.get(c.id)" :y="contactY(c) + 4">
            {{ c.foot[0].toUpperCase() }}{{ c.id }}
          </text>
          <g v-if="graph.ground" class="height-factor">
            <title>Contact height {{ c.id }}</title>
            <line
              :x1="positions.get(c.id)"
              :y1="contactY(c) + 15"
              :x2="positions.get(c.id)"
              :y2="contactY(c) + 32"
            />
            <rect
              :x="positions.get(c.id) - 3"
              :y="contactY(c) + 32"
              width="6"
              height="6"
            />
          </g>
          <g v-if="graph.normal" class="normal-factor">
            <title>Sole normal {{ c.id }}</title>
            <line
              :x1="positions.get(c.id) + 15"
              :y1="contactY(c)"
              :x2="positions.get(c.id) + 25"
              :y2="contactY(c)"
            />
            <rect
              :x="positions.get(c.id) + 25"
              :y="contactY(c) - 3"
              width="6"
              height="6"
            />
          </g>
        </g>
      </svg>
    </div>
    <div class="graph-key">
      <span>○ Pose3 variable</span><span>▪ Factor</span><span>X₀ fixed</span>
      <span class="left">Left sole</span><span class="right">Right sole</span>
      <span v-if="graph.between" class="motion-factor"
        >{{ count("motion") }} motion</span
      >
      <span v-if="graph.upright" class="upright-factor"
        >{{ count("upright") }} upright</span
      >
      <span v-if="graph.ground" class="height-factor"
        >{{ count("height") }} height</span
      >
      <span v-if="graph.normal" class="normal-factor"
        >{{ count("normal") }} normal</span
      >
    </div>
  </section>
</template>

<style scoped>
.graph-stats {
  font-size: 12px;
  color: #61716a;
}
.graph-scroll {
  border-block: 1px solid #e0e7e2;
  background: #fafbf9;
}
svg {
  display: block;
}
svg text {
  font:
    11px ui-monospace,
    monospace;
  text-anchor: middle;
  fill: #243438;
}
svg .row-label {
  text-anchor: start;
  fill: #61716a;
  font-size: 10px;
}
svg .node-time {
  font-size: 9px;
  fill: #61716a;
}
line,
path {
  stroke: currentColor;
  fill: none;
  stroke-width: 1.5;
}
rect {
  fill: currentColor;
}
.left {
  color: #c93838;
}
.right {
  color: #16884b;
}
.observation-factor {
  opacity: 0.4;
}
.observation-factor.highlight {
  opacity: 1;
}
.pose-node circle {
  fill: #fff;
  stroke: #243438;
  stroke-width: 1.5;
}
.pose-node.selected circle {
  fill: #243438;
  stroke-width: 3;
}
.pose-node.selected > text:first-of-type {
  fill: white;
}
.contact-node circle {
  fill: white;
  stroke: currentColor;
  stroke-width: 1.5;
}
.contact-node.active circle {
  fill: #edf2e8;
  stroke-width: 3;
}
.motion-factor {
  color: #a4761a;
}
.upright-factor {
  color: #7650a5;
}
.height-factor {
  color: #287eaa;
}
.normal-factor {
  color: #b16a36;
}
.graph-key {
  display: flex;
  gap: 12px;
  flex-wrap: wrap;
  font-size: 12px;
  padding-top: 16px;
}
</style>
