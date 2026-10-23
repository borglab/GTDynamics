/** Run against npm run dev with PLAYWRIGHT_MODULE=/path/to/playwright/index.mjs. */
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { sampleTrajectoryState } from '../src/viewer.js';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const browser = await chromium.launch({
  executablePath: process.env.CHROME_EXECUTABLE || (existsSync('/opt/google/chrome/chrome') ? '/opt/google/chrome/chrome' : undefined),
  headless: true, args: ['--no-sandbox', '--enable-unsafe-swiftshader'],
});
try {
  const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } });
  const errors = [], snapshots = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('websocket', socket => socket.on('framereceived', ({ payload }) => {
    const message = JSON.parse(payload.toString());
    if (['built', 'targets', 'iteration'].includes(message.type)) snapshots.push(message);
  }));
  await page.goto(process.env.MOTION_PLANNING_URL || 'http://127.0.0.1:3001/');
  await page.locator('.joint-empty').waitFor();
  await page.getByRole('button', { name: 'Build graph', exact: true }).click();
  await page.locator('.joint-panel[data-iteration="0"]').waitFor();
  assert.equal(await page.locator('.joint-curve').count(), 14);
  async function checkStack() {
    const rows = await page.locator('.joint-plots figure').evaluateAll(nodes => nodes.map(n => {
      const box = n.getBoundingClientRect(), svg = n.querySelector('svg').getBoundingClientRect();
      return { top: box.top, bottom: box.bottom, left: svg.left, width: svg.width, curves: n.querySelectorAll('.joint-curve').length };
    }));
    assert.equal(rows.length, 7, 'One plot for each joint');
    rows.forEach((row, i) => {
      assert.equal(row.curves, 2, 'Each joint shows its angle and velocity');
      assert.equal(row.left, rows[0].left, 'Time axes align');
      assert.equal(row.width, rows[0].width, 'Time scales align');
      if (i) assert(row.top >= rows[i - 1].bottom, 'Joint plots stack vertically');
    });
  }
  await checkStack();
  assert.equal(await page.locator('.iteration-panel + .joint-panel').count(), 1);
  await page.getByRole('button', { name: 'Play arm trajectory', exact: true }).waitFor();
  await page.evaluate(() => {
    window.plotIterations = [];
    new MutationObserver(() => {
      const panel = document.querySelector('.joint-panel');
      window.plotIterations.push({ iteration: Number(panel.dataset.iteration), points: panel.querySelector('.joint-curve').getAttribute('points') });
    }).observe(document.querySelector('.joint-panel'), { subtree: true, attributes: true, attributeFilter: ['data-iteration'] });
  });
  const before = await page.locator('.joint-curve').first().getAttribute('points');
  await page.getByRole('button', { name: 'Solve trajectory' }).click();
  await page.waitForFunction(() => document.querySelector('.solver-status')?.textContent.includes('Converged'));
  const observed = await page.evaluate(() => window.plotIterations);
  assert(new Set(observed.map(item => item.iteration)).size > 2, 'Plots update during the native solve');
  assert(observed.some(item => item.points !== before), 'Joint curves evolve during optimization');
  const last = snapshots.at(-1);
  assert.equal(Number(await page.locator('.joint-panel').getAttribute('data-iteration')), last.iteration);
  const paths = () => page.locator('.joint-curve').evaluateAll(nodes => nodes.map(n => n.getAttribute('points')));
  const finalPaths = await paths();
  const finalTicks = await page.locator('.joint-axis text').allTextContents();
  async function setRange(label, value) {
    await page.getByRole('slider', { name: label, exact: true }).evaluate((node, v) => {
      node.value = v; node.dispatchEvent(new Event('input', { bubbles: true }));
    }, String(value));
  }
  async function checkValues(snapshot, t) {
    await setRange('Joint plot inspection time', t);
    const actual = [...await page.locator('.joint-value[data-quantity="q"]').allTextContents(),
      ...await page.locator('.joint-value[data-quantity="v"]').allTextContents()];
    const expected = sampleTrajectoryState(snapshot, t);
    assert.deepEqual(actual, [...expected.q, ...expected.v].map(value => value.toFixed(3)), 'Readout matches the selected native snapshot');
  }
  await checkValues(last, 1.237);
  await setRange('Recorded optimizer iteration', 0);
  assert.equal(await page.locator('.joint-panel').getAttribute('data-iteration'), '0');
  await checkValues(snapshots[0], 1.237);
  assert.notDeepEqual(await paths(), finalPaths);
  assert.deepEqual(await page.locator('.joint-axis text').allTextContents(), finalTicks, 'Scrubbing keeps axes fixed');
  await setRange('Recorded optimizer iteration', snapshots.length - 1);
  assert.deepEqual(await paths(), finalPaths);
  await page.getByRole('button', { name: 'Replay iterations', exact: true }).click();
  await page.waitForFunction(() => Number(document.querySelector('.joint-panel').dataset.iteration) > 0);
  await page.getByRole('button', { name: 'Follow latest', exact: true }).click();
  assert.equal(Number(await page.locator('.joint-panel').getAttribute('data-iteration')), last.iteration);
  const plot = page.locator('.joint-plots svg').nth(3);
  const width = await plot.evaluate(svg => svg.getBoundingClientRect().width);
  await plot.hover({ position: { x: 64 + (width - 128) * 0.65, y: 50 } });
  const hoveredTime = Number(await page.getByRole('slider', { name: 'Joint plot inspection time' }).inputValue());
  assert(Math.abs(hoveredTime - last.duration * 0.65) < 0.01, 'Hover uses trajectory time');
  const cursors = await page.locator('.joint-cursor').evaluateAll(nodes => nodes.map(n => Number(n.getAttribute('x1'))));
  assert.equal(cursors.length, 7);
  assert(cursors.every(value => value === cursors[0]), 'Hover synchronizes every cursor');
  await page.getByRole('button', { name: 'Follow arm time', exact: true }).click();
  await page.waitForFunction(() => !document.querySelector('.time-slider').disabled);
  await setRange('Arm trajectory time', 2.5);
  assert.equal(await page.getByRole('slider', { name: 'Joint plot inspection time' }).inputValue(), '2.5');
  if (process.env.JOINT_PLOT_SCREENSHOT) await page.locator('.joint-panel').screenshot({ path: process.env.JOINT_PLOT_SCREENSHOT });
  await page.setViewportSize({ width: 390, height: 844 });
  assert(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth), 'Panel fits a narrow viewport');
  await checkStack();
  await page.getByLabel('Duration seconds').fill('6');
  await page.getByRole('button', { name: 'Rebuild graph', exact: true }).click();
  await page.locator('.joint-panel[data-iteration="0"]').waitFor();
  assert.equal(await page.getByRole('slider', { name: 'Joint plot inspection time' }).getAttribute('max'), '6');
  await checkValues(snapshots.at(-1), 4.5);
  assert.deepEqual(errors, []);
  console.log('PASS: live solver plots, exact history data, stable axes, replay, seven aligned joint rows, synchronized hover, arm-time cursor, responsive layout, and graph rebuild.');
} finally { await browser.close(); }
