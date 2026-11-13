/**
 * Live simulation end to end on the app's dev server: drag the start arm and
 * an obstacle, solve with the native scate_live, play the result, then the
 * same on the UR5. Needs scate_live built (or SCATE_LIVE_BIN) and Chrome.
 *
 *   PLAYWRIGHT_MODULE=/path/to/playwright/index.mjs node scate/ur10e-reactive/tests/live.browser.mjs
 */
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { createServer } from 'vite';
import { findScateLive } from '../../shared/liveServer.mjs';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');

if (!findScateLive()) {
  console.log('SKIP: scate_live is not built (cmake --build build --target scate_live)');
  process.exit(0);
}
const server = await createServer({ root: fileURLToPath(new URL('..', import.meta.url)), server: { host: '127.0.0.1', port: 0 } });
await server.listen();
const browser = await chromium.launch({ executablePath: process.env.CHROME_EXECUTABLE || (existsSync('/usr/bin/google-chrome') ? '/usr/bin/google-chrome' : undefined), headless: true, args: ['--no-sandbox', '--enable-unsafe-swiftshader'] });
try {
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } }), errors = [];
  page.setDefaultTimeout(60000);
  page.on('pageerror', error => errors.push(error.message));
  await page.goto(`http://127.0.0.1:${server.httpServer.address().port}/`);
  await page.locator('#stage[data-ready="true"]').waitFor();
  const solveButton = page.locator('[data-solve]');
  await page.locator('[data-solve]:not([disabled])').waitFor();
  const canvas = await page.locator('#stage canvas').boundingBox();

  // Drag somewhere on the canvas until the edited value changes (a miss orbits the camera a little).
  async function dragUntil(read) {
    const before = await read();
    for (const fy of [0.45, 0.5, 0.4, 0.55, 0.35, 0.6]) {
      for (const fx of [0.4, 0.45, 0.35, 0.5, 0.55, 0.3, 0.6]) {
        const x = canvas.x + canvas.width * fx, y = canvas.y + canvas.height * fy;
        await page.mouse.move(x, y); await page.mouse.down();
        await page.mouse.move(x + 50, y - 30, { steps: 6 }); await page.mouse.up();
        if (await read() !== before) return true;
      }
    }
    return false;
  }

  await page.getByRole('button', { name: 'Set start and goal position' }).click();
  assert.equal(await page.locator('[data-run-toggles]').count(), 0, 'replay panels hide while editing');
  assert(await dragUntil(() => page.locator('[data-pose-editor]').innerText()), 'dragging the start arm moves its joints');
  await page.getByRole('button', { name: 'Reset to experiment' }).click();
  await page.getByRole('button', { name: 'Set obstacles' }).click();
  assert(await dragUntil(() => page.locator('[data-obstacle-editor]').innerText()), 'dragging moves an obstacle');
  await page.locator('#live-radius-0').evaluate(input => { input.value = '6'; input.dispatchEvent(new Event('input', { bubbles: true })); });
  await page.getByRole('button', { name: 'Done' }).click();
  assert.match(await page.locator('[data-live-panel]').innerText(), /your obstacles/);

  await solveButton.click();
  await page.getByText('Live solve', { exact: true }).waitFor({ timeout: 120000 });
  assert.equal(await page.locator('[data-run-toggles] .run-toggle').count(), 4);
  assert.match(await page.locator('h1').innerText(), /UR10e/);

  await page.getByText('UR5', { exact: true }).click();
  await page.locator('[data-pose-editor]').waitFor();
  await page.getByRole('button', { name: 'Done' }).click();
  for (const label of ['Base cost off', 'σ = 0.3', 'σ = 0.1']) await page.locator('.live-runs label', { hasText: label }).locator('input').uncheck();
  await solveButton.click();
  await page.waitForFunction(() => document.querySelector('h1')?.innerText.includes('UR5'), null, { timeout: 120000 });
  assert.match(await page.locator('[data-live-panel]').innerText(), /Solved in/);
  assert.deepEqual(errors, []);
  console.log('PASS: drag start arm and obstacle, live UR10e solve with four runs, live UR5 solve');
} finally {
  await browser.close();
  await server.close();
}
