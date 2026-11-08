<script setup>
import { onBeforeUnmount, onMounted, ref } from 'vue';
import AppAbstract from '../../shared/paper/AppAbstract.vue';
import AppPaper from '../../shared/paper/AppPaper.vue';
import MathTypeset from '../../shared/paper/MathTypeset.vue';
import PaperGraph from '../../shared/paper/PaperGraph.vue';
import license from '../../../models/urdfs/ur10e/LICENSE?url';
import { createExplorer } from './explorer.js';
import equations from './ik.math.html?math';

const mode = ref('loading');
const offlineReason = ref('');
const poseGoal = ref(true);
const graphSolver = ref(true);
let explorer;

onMounted(() => {
  explorer = createExplorer({
    backend: import.meta.env.DEV,
    onLive: () => { mode.value = 'live'; },
    onOffline: (message) => { mode.value = 'offline'; offlineReason.value = message; },
    onMethod: (scheme, orientation) => {
      graphSolver.value = scheme === 'gtd';
      poseGoal.value = orientation;
    },
  });
});
onBeforeUnmount(() => explorer?.dispose());
</script>

<template>
  <main class="urdf-app" :data-mode="mode">
    <header class="page-header">
      <div>
        <h1>URDF IK Explorer</h1>
        <AppAbstract app="urdf" />
      </div>
      <span class="mode-label">{{ mode === 'live' ? 'Live native solve' : mode === 'offline' ? 'Local backend required' : 'Connecting…' }}</span>
    </header>

    <section id="demo" aria-label="Interactive inverse kinematics">
      <div id="viewport">
        <div id="help">
          <b>Drag</b> the orange target to move it · <b>double-click</b> to edit its pose
          (<kbd>W</kbd> move, <kbd>E</kbd> rotate, <kbd>Esc</kbd> done) · drag the background to orbit
        </div>
        <div id="view-tools">
          <label class="check"><input type="checkbox" id="show-skeleton" checked> Skeleton</label>
          <label class="check"><input type="checkbox" id="show-meshes" checked> Meshes</label>
          <button id="fit" class="small secondary">Fit view</button>
        </div>
        <div id="toast" hidden></div>

        <div v-if="mode === 'offline'" class="offline" role="region" aria-labelledby="offline-title">
          <div class="offline-card">
            <h2 id="offline-title">Local GTDynamics server required</h2>
            <p>Live solving runs in the GTDynamics Python bindings ({{ offlineReason }}). Install the bindings, then start the site from <code>apps/</code>.</p>
            <pre><code>python -c "import gtdynamics"
npm run dev</code></pre>
            <p><a href="https://github.com/borglab/GTDynamics/tree/master/apps/urdf#readme" target="_blank" rel="noopener">URDF IK setup ↗</a></p>
          </div>
        </div>
      </div>

      <aside id="panel" aria-label="Inverse kinematics controls" :inert="mode === 'offline' || undefined">
        <div class="selectors">
          <label>Model <select id="model"></select></label>
          <label>End-effector frame <select id="ee" disabled></select></label>
          <label>IK scheme <select id="scheme"></select></label>
          <label class="check"><input type="checkbox" id="orientation" checked> Constrain orientation</label>
        </div>

        <div class="configuration">
          <section id="pose-section">
            <h2>Tool pose <span class="edit-flag">Editing</span></h2>
            <table class="pose">
              <thead><tr><th scope="col">Axis</th><th scope="col">Current</th><th scope="col">Target</th></tr></thead>
              <tbody id="pose-rows"></tbody>
            </table>
            <div class="buttons">
              <button id="solve">Solve to target</button>
              <button id="reset-target" class="secondary">Target = current</button>
            </div>
          </section>

          <section aria-labelledby="joint-heading">
            <div class="joint-heading">
              <h2 id="joint-heading">Joints</h2>
              <button id="home" class="small secondary">Home</button>
            </div>
            <div id="joints" role="region" aria-label="Joint coordinates" tabindex="0"></div>
          </section>
        </div>

        <div id="status" class="status idle">
          <div class="status-head" role="status">
            <span id="status-badge">Loading…</span>
            <span id="status-meta"></span>
          </div>
          <a href="#solve-details">Diagnostics and solver settings ↓</a>
        </div>
      </aside>
    </section>

    <AppPaper app="urdf">
      <template #graph>
        <p v-if="!graphSolver" class="hint">The selected solver works in joint space. This diagram shows the GTDynamics graph for the current goal type.</p>
        <PaperGraph :kind="poseGoal ? 'urdf-pose' : 'urdf-point'" :frame="0" :samples="1" />
      </template>
      <template #mathematics><MathTypeset :html="equations" /></template>

      <div id="solve-details" class="solve-details">
        <section>
          <h3>Latest solve</h3>
          <p>The demo reports world-frame position error in millimetres and, for pose goals, rotation error in degrees. Time measures the native solve in milliseconds. The iteration count appears when the solver reports it.</p>
          <p id="result-summary" class="result-summary"></p>
          <ul id="reasons"></ul>
          <p id="quat" class="hint mono"></p>
          <div id="notes" hidden></div>
        </section>
        <section>
          <h3>Solver settings</h3>
          <p id="scheme-desc"></p>
          <div class="grid3" :inert="mode === 'offline' || undefined">
            <label>Pos tol (mm) <input type="number" id="pos-tol" value="1" min="0.001" step="0.1"></label>
            <label>Rot tol (°) <input type="number" id="rot-tol" value="0.5" min="0.001" step="0.1"></label>
            <label>Max iters <input type="number" id="max-iters" min="1" step="1" placeholder="auto"></label>
          </div>
          <p>The tolerances determine whether the app reports a solve as successful. The iteration limit sets the solver’s effort budget.</p>
          <a href="#demo">Return to demo ↑</a>
        </section>
      </div>

      <template #discussion>
        <p>The UR10e model and meshes come from Universal Robots’ ROS 2 Description 4.4.0 (81f7c9fe). <a :href="license" download="UR10e-LICENSE">UR10e model license</a>.</p>
      </template>
    </AppPaper>
  </main>
</template>
