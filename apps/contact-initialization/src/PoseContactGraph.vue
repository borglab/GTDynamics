<script setup>
import { renderFactorGraph } from "fgz/browser";
import { onMounted, ref, watch } from "vue";
import { edgeEndpoint, frameX, poseContactLayout } from "./poseContactLayout.js";

const props = defineProps({ graph: Object, selected: Number });
const emit = defineEmits(["select"]);
const scroller = ref(null);
let view;

const count = (id) =>
  props.graph.factorGroups.find((group) => group.id === id)?.count ?? 0;

function appendSvg(parent, tag, attributes, text) {
  const element = document.createElementNS(parent.namespaceURI, tag);
  for (const [name, value] of Object.entries(attributes)) {
    element.setAttribute(name, value);
  }
  if (text !== undefined) element.textContent = text;
  parent.appendChild(element);
}

function syncSelection() {
  if (!view) return;
  const i = props.selected;
  view.setSelection(props.graph.frames.length ? `X${i}` : null);
  const host = scroller.value;
  const x = frameX(i);
  if (x < host.scrollLeft + 30 || x > host.scrollLeft + host.clientWidth - 30) {
    host.scrollLeft = Math.max(0, x - host.clientWidth / 2);
  }
}

function mountGraph() {
  const graph = props.graph;
  const layout = poseContactLayout(graph);
  const variables = new Map(layout.variables.map(v => [v.id, v]));
  const factors = new Map(layout.factors.map(f => [f.id, f]));
  view = renderFactorGraph(scroller.value, layout, {
    viewport: { width: Math.max(560, graph.frames.length * 58 + 30), height: 320 },
    edgeEndpoint,
    isSelectable: v => v.frame !== undefined,
    isHighlightable: f => f.kind === "observation",
    onSelect: id => emit("select", variables.get(id).frame),
  });

  const { svg } = view;
  svg.setAttribute("aria-label", "Bipartite graph of root poses and stance contact poses");
  appendSvg(svg, "text", { x: 16, y: 18, class: "row-label" }, "Root poses · time →");

  for (const node of svg.querySelectorAll("[data-variable]")) {
    const { frame, foot } = variables.get(node.dataset.variable);
    const label = node.querySelector("text");
    label.setAttribute("y", 4);
    if (foot) {
      node.classList.add("contact-node", foot);
      continue;
    }
    // The timeline leaves a monospace space between X and its subscript.
    label.firstChild.appendData(" ");
    node.classList.add("pose-node");
    node.setAttribute("aria-label", `Select event ${frame}`);
    appendSvg(node, "text", { y: 36, class: "node-time" }, `${graph.frames[frame].time.toFixed(2)}s`);
    if (graph.anchorFrameIndices.includes(frame)) {
      appendSvg(node, "path", { d: "M-7 -25h14m-7 0v-10", class: "anchor-mark" });
    }
  }
  for (const element of svg.querySelectorAll("[data-factor], [data-edge]")) {
    const factor = factors.get(element.dataset.factor ?? element.dataset.factorId);
    element.classList.add(`${factor.kind}-factor`);
    if (factor.foot) element.classList.add(factor.foot);
    if (element.hasAttribute("data-factor")) {
      element.querySelector("title").textContent = factor.title;
    } else if (factor.kind !== "observation") {
      appendSvg(element, "title", {}, factor.title);
    }
  }
  syncSelection();
}

onMounted(mountGraph);
watch(() => props.graph, mountGraph, { flush: "post" });
watch(() => props.selected, syncSelection, { flush: "post" });
</script>

<template>
  <section class="graph-card pose-graph">
    <h2>Pose/contact graph</h2>
    <p class="graph-stats">
      {{ graph.frames.length }} root poses + {{ graph.contacts.length }} contact poses
      · {{ count("contact") }} observations
    </p>
    <div ref="scroller" class="graph-scroll"></div>
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
  color: var(--dark-secondary, #61716a);
}
.graph-scroll {
  border-block: 1px solid var(--dark-border, #e0e7e2);
  background: var(--dark-page, #fafbf9);
}
.graph-scroll :deep(svg) {
  display: block;
}
.graph-scroll :deep(text) {
  font: 11px ui-monospace, monospace;
  text-anchor: middle;
  dominant-baseline: auto;
  fill: var(--dark-text, #243438);
}
.graph-scroll :deep(tspan) {
  font-size: 8px;
}
.graph-scroll :deep(.row-label) {
  text-anchor: start;
  fill: var(--dark-muted, #61716a);
  font-size: 10px;
}
.graph-scroll :deep(.node-time) {
  font-size: 9px;
  fill: var(--dark-muted, #61716a);
}
.graph-scroll :deep(line),
.graph-scroll :deep(path) {
  stroke: currentColor;
  fill: none;
  stroke-width: 1.5;
}
.graph-scroll :deep([data-factor] rect) {
  x: -3px;
  y: -3px;
  width: 6px;
  height: 6px;
  fill: currentColor;
}
.pose-graph :deep(.left) {
  color: #c93838;
}
.pose-graph :deep(.right) {
  color: #16884b;
}
.graph-scroll :deep(.observation-factor) {
  opacity: 0.4;
}
.graph-scroll :deep(.observation-factor[data-active="true"]) {
  opacity: 1;
}
.graph-scroll :deep(.pose-node circle) {
  r: 16px;
  fill: var(--dark-panel, #fff);
  stroke: var(--dark-muted, #243438);
}
.graph-scroll :deep(.pose-node[aria-pressed="true"] circle) {
  fill: var(--dark-text, #243438);
}
.graph-scroll :deep(.pose-node[aria-pressed="true"] > text:first-of-type) {
  fill: var(--dark-panel, white);
}
.graph-scroll :deep(.contact-node circle) {
  r: 15px;
  fill: var(--dark-panel, white);
  stroke: currentColor;
}
.graph-scroll :deep(.contact-node[data-active="true"] circle) {
  fill: var(--dark-panel, #edf2e8);
}
.pose-graph :deep(.motion-factor) {
  color: #a4761a;
}
.pose-graph :deep(.upright-factor) {
  color: #7650a5;
}
.pose-graph :deep(.height-factor) {
  color: #287eaa;
}
.pose-graph :deep(.normal-factor) {
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
