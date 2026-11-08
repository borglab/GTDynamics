<script setup>
import { onBeforeUnmount, onMounted, ref } from 'vue';
import { createExplorer } from './explorer.js';

// loading → live (backend answered) or offline (no backend, e.g. GitHub Pages).
const mode = ref('loading');
const offlineReason = ref('');
let explorer;
onMounted(() => {
  explorer = createExplorer({
    // Only the Vite dev server proxies api/ to scripts/server.py; a static build never probes it.
    backend: import.meta.env.DEV,
    onLive: () => { mode.value = 'live'; },
    onOffline: (message) => { mode.value = 'offline'; offlineReason.value = message; },
  });
});
onBeforeUnmount(() => explorer?.dispose());
</script>

<template>
  <div class="urdf-app" :data-mode="mode">
    <aside id="panel">
      <header>
        <h1>URDF IK Explorer</h1>
        <span class="sub">GTDynamics</span>
      </header>

      <section class="intro" aria-label="About this demo">
        <p>Load the URDF models in <code>models/urdfs</code> and test whether GTDynamics parses them
          into a kinematic model that inverse kinematics can solve.</p>
        <ul>
          <li>Choose a <b>Model</b> and an <b>End-effector frame</b>.</li>
          <li><b>Drag</b> the orange handle to move the end-effector. IK re-solves as you drag.</li>
          <li><b>Double-click</b> the handle to rotate the end-effector with the gizmo
            (<kbd>W</kbd> move, <kbd>E</kbd> rotate). Double-click again or press <kbd>Esc</kbd> to finish.</li>
          <li>Choose an <b>IK scheme</b> to compare solvers on the same target.</li>
          <li>Use the <b>Joints</b> sliders to set the configuration directly, or <b>Home</b> to reset it.</li>
        </ul>
      </section>

      <section class="controls">
        <label>Model
          <select id="model"></select>
        </label>
        <label>End-effector frame
          <select id="ee" disabled></select>
        </label>
        <label>IK scheme
          <select id="scheme"></select>
        </label>
        <p id="scheme-desc" class="hint"></p>
        <label class="check">
          <input type="checkbox" id="orientation" checked>
          Constrain orientation (full 6-DoF pose)
        </label>
        <div class="grid3">
          <label>Pos tol (mm)
            <input type="number" id="pos-tol" value="1" min="0.001" step="0.1">
          </label>
          <label>Rot tol (°)
            <input type="number" id="rot-tol" value="0.5" min="0.001" step="0.1">
          </label>
          <label>Max iters
            <input type="number" id="max-iters" min="1" step="1" placeholder="auto">
          </label>
        </div>
      </section>

      <section id="status" class="status idle">
        <div class="status-head">
          <span id="status-badge">Loading…</span>
          <span id="status-meta"></span>
        </div>
        <ul id="reasons"></ul>
      </section>

      <section id="pose-section">
        <h2>End-effector pose <span class="edit-flag">editing</span></h2>
        <table class="pose">
          <thead>
            <tr><th></th><th>Current</th><th>Target</th></tr>
          </thead>
          <tbody id="pose-rows"></tbody>
        </table>
        <p class="hint mono" id="quat"></p>
        <div class="buttons">
          <button id="solve">Solve to target</button>
          <button id="reset-target" class="secondary">Target = current</button>
        </div>
      </section>

      <section>
        <h2>Joints <button id="home" class="small secondary">Home</button></h2>
        <div id="joints"></div>
      </section>

      <section id="notes" hidden></section>
    </aside>

    <main id="viewport">
      <div id="help">
        <b>Drag</b> the orange handle to move the end-effector ·
        <b>double-click</b> it to edit its pose
        (<kbd>W</kbd> move, <kbd>E</kbd> rotate, <kbd>Esc</kbd> done) ·
        drag elsewhere to orbit, scroll to zoom
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
          <p>Pick a robot from <code>models/urdfs</code> and an end-effector frame, then drag the
            end-effector around. Each move runs an IK scheme on GTDynamics' kinematic model: the
            GTDynamics factor graph (LM), damped least squares, the Jacobian pseudo-inverse or
            transpose, or cyclic coordinate descent. When a target can't be reached, the app
            explains why.</p>
          <p>Every solve runs in the GTDynamics Python bindings, so this page needs its local
            backend, and none is available here ({{ offlineReason }}). To run the app on your machine:</p>
          <p>1. Build and install the GTDynamics Python bindings (see <code>python/README.md</code>), then check that they import:</p>
          <pre><code>python -c "import gtdynamics"</code></pre>
          <p>2. From <code>apps/</code>, start the apps site. It starts this app's backend when it finds the bindings:</p>
          <pre><code>npm run dev</code></pre>
          <p>Set <code>GTDYNAMICS_PYTHON</code> to choose the interpreter. Details:
            <a href="https://github.com/borglab/GTDynamics/tree/master/apps/urdf#readme" target="_blank" rel="noopener">apps/urdf/README.md ↗</a></p>
        </div>
      </div>
    </main>
  </div>
</template>
