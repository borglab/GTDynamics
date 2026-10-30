/**
 * Real MuJoCo regression test. Requires the compiled native optimizer, a running
 * Vite dev server (npm run dev), Chrome, and an external Playwright installation.
 * This keeps browser-test dependencies out of the app's runtime dependencies.
 *
 * Example from apps/motion-planning/:
 * PLAYWRIGHT_MODULE=/path/to/node_modules/playwright/index.mjs \
 *   node tests/playback.browser.mjs
 *
 * Optional: MOTION_PLANNING_URL, MOTION_PLANNING_OPTIMIZER, CHROME_EXECUTABLE. Without an explicit
 * module path, import the locally installed `playwright` package.
 */
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { existsSync } from 'node:fs';
import { createInterface } from 'node:readline';
import { fileURLToPath } from 'node:url';

const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const binary = process.env.MOTION_PLANNING_OPTIMIZER || fileURLToPath(
  new URL('../../../build/apps/motion-planning/backend/motion_planning_optimizer', import.meta.url));
const url = process.env.MOTION_PLANNING_URL || 'http://127.0.0.1:3001/';
const optimizer = spawn(binary, [], { stdio: ['pipe', 'pipe', 'ignore'] });
const lines = createInterface({ input: optimizer.stdout })[Symbol.asyncIterator]();
async function send(command, fields = {}) {
  optimizer.stdin.write(`${JSON.stringify({ command, ...fields })}\n`);
  const line = await lines.next();
  assert(!line.done, 'Native optimizer closed its output unexpectedly');
  const response = JSON.parse(line.value);
  assert.notEqual(response.type, 'error', response.message);
  return response;
}
let browser;
try {
  const description = await send('describe');
  let snapshot = await send('build', { intermediate_states: 10, duration: 4,
    start: description.start, goal: description.goal });
  for (let i = 0; i < 80 && !snapshot.done && !snapshot.converged; i += 1) snapshot = await send('iterate');
  assert(snapshot.converged, 'Default native trajectory should converge before checking playback');
  const chrome = process.env.CHROME_EXECUTABLE
    || (existsSync('/opt/google/chrome/chrome') ? '/opt/google/chrome/chrome' : undefined);
  browser = await chromium.launch({ executablePath: chrome, headless: true,
    args: ['--no-sandbox', '--enable-unsafe-swiftshader'] });
  const page = await browser.newPage();
  const errors = [];
  page.on('pageerror', error => errors.push(String(error)));
  // An isolated canvas shares the real dev server's modules and assets without
  // touching the running classroom demo's graph or WebSocket connection.
  await page.route(url, route => route.fulfill({ contentType: 'text/html',
    body: '<div id="viewer" style="width:900px;height:700px"></div>' }));
  await page.goto(url);
  const result = await page.evaluate(async ({ description, snapshot }) => {
    const { MotionPlanningViewer, sampleTrajectory } = await import(new URL('./src/viewer.js', location.href).href);
    const viewer = new MotionPlanningViewer(document.querySelector('#viewer'));
    await viewer.init(description);
    viewer.setTrajectory(snapshot);
    const scenarios = [];
    for (const speed of [1, 2, 3]) {
      viewer.setPlaying(false);
      viewer.setTime(0);
      viewer.setSpeed(speed);
      viewer.setPlaying(true);
      const arrivals = [];
      let maximumTrackingError = 0;
      for (let i = 0; i < 10000 && arrivals.length < 2 && viewer.playing; i += 1) {
        const beforeDirection = viewer.direction;
        viewer.step();
        if (i % 10 === 0) maximumTrackingError = Math.max(maximumTrackingError,
          viewer.getDiagnostics().tcpPositionError);
        if (beforeDirection !== viewer.direction) arrivals.push(viewer.getDiagnostics().lastEndpointArrival);
      }
      scenarios.push({ speed, arrivals, maximumTrackingError, playing: viewer.playing });
    }

    viewer.setSpeed(1);
    for (let i = 0; i < 250; i += 1) viewer.step();
    const beforeQ = Array.from(viewer.data.qpos), beforeV = Array.from(viewer.data.qvel);
    const replacement = structuredClone(snapshot);
    replacement.states.forEach(state => { state.q[0] += 0.05; });
    viewer.setTrajectory(replacement);
    const replacementPreservesState = JSON.stringify(beforeQ) === JSON.stringify(Array.from(viewer.data.qpos))
      && JSON.stringify(beforeV) === JSON.stringify(Array.from(viewer.data.qvel));
    let maxStep = 0, previous = Array.from(viewer.data.qpos);
    for (let i = 0; i < 100; i += 1) {
      viewer.step();
      maxStep = Math.max(maxStep, ...Array.from(viewer.data.qpos, (q, j) => Math.abs(q - previous[j])));
      previous = Array.from(viewer.data.qpos);
    }
    viewer.setTrajectory(snapshot);
    viewer.setPlaying(false);
    viewer.setTime(snapshot.duration * 0.4);
    const scrubError = Math.max(...sampleTrajectory(snapshot, viewer.time)
      .map((q, j) => Math.abs(q - viewer.data.qpos[viewer.qposAddresses[j]])));

    // Deliberately make the interactive goal disagree with the solved endpoint:
    // an unsatisfied target must pause instead of claiming arrival forever.
    viewer.setTime(snapshot.duration);
    const unreachable = structuredClone(description.goal);
    unreachable.position[0] += 0.2;
    viewer.setTargets(description.start, unreachable);
    viewer.setPlaying(true);
    for (let i = 0; i < 1500 && viewer.playing; i += 1) viewer.step();
    const timeout = viewer.getDiagnostics();
    viewer.setTargets(description.start, description.goal);
    viewer.setPlaying(true);
    for (let i = 0; i < 150; i += 1) viewer.step();
    const recovered = viewer.getDiagnostics();
    viewer.dispose();
    return { scenarios, replacementPreservesState, maxStep, scrubError,
      timeout: { playing: timeout.playing, status: timeout.endpointStatus,
        name: timeout.endpointName, holdTime: timeout.endpointHoldTime,
        positionError: timeout.endpointPositionError },
      recovered: { playing: recovered.playing, status: recovered.endpointStatus,
        arrival: recovered.lastEndpointArrival } };
  }, { description, snapshot });
  for (const scenario of result.scenarios) {
    assert(scenario.playing, `Playback unexpectedly paused at ${scenario.speed}×`);
    assert.equal(scenario.arrivals.length, 2);
    assert(scenario.maximumTrackingError < 0.003, `${scenario.speed}× tracking exceeded 3 mm`);
    for (const arrival of scenario.arrivals) {
      assert(arrival.positionError <= 1e-4, `${arrival.name} missed the 0.1 mm arrival tolerance`);
      assert(arrival.rotationError <= 1e-3, `${arrival.name} missed the orientation tolerance`);
      assert(arrival.elapsed >= 0.1 && arrival.elapsed <= 2);
    }
  }
  assert(result.replacementPreservesState, 'Replacing the trajectory must not teleport qpos or reset qvel');
  assert(result.maxStep < 0.01, 'Live replacement created a discontinuous physical motion');
  assert(result.scrubError < 1e-12, 'Explicit scrubbing must apply the selected estimate');
  assert.equal(result.timeout.playing, false);
  assert.equal(result.timeout.status, 'unreached');
  assert.equal(result.timeout.name, 'goal');
  assert(result.timeout.holdTime >= 2 && result.timeout.holdTime <= 2.01);
  assert(result.timeout.positionError > 0.19);
  assert(result.recovered.playing && result.recovered.status === 'moving');
  assert.deepEqual(errors, []);
  console.log(JSON.stringify(result, null, 2));
  console.log('PASS: real MuJoCo tracking, endpoint settling in both directions at 1×/2×/3×, live continuity, scrub, timeout and recovery.');
} finally {
  await browser?.close();
  optimizer.stdin.end();
  optimizer.kill();
}
