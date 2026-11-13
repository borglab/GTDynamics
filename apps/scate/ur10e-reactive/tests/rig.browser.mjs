// Browser check of the MK 111 rig option in the live simulation: the checkbox
// draws the wall and floor, a start pose through the floor is flagged, and a
// solve with the rig returns a replay that draws both planes. Needs Chrome,
// Playwright and a built scate_live, like live.browser.mjs.
//   node tests/rig.browser.mjs [screenshot-dir]
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
const shots = process.argv[2];
const server = await createServer({ root: fileURLToPath(new URL('..', import.meta.url)), server: { host: '127.0.0.1', port: 0 } });
await server.listen();
const browser = await chromium.launch({ executablePath: process.env.CHROME_EXECUTABLE || (existsSync('/usr/bin/google-chrome') ? '/usr/bin/google-chrome' : undefined), headless: true, args: ['--no-sandbox', '--enable-unsafe-swiftshader'] });
try {
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } }), errors = [];
  page.setDefaultTimeout(60000);
  page.on('pageerror', error => errors.push(error.message));
  await page.goto(`http://127.0.0.1:${server.httpServer.address().port}/`);
  await page.locator('#stage[data-ready="true"]').waitFor();
  await page.locator('[data-solve]:not([disabled])').waitFor();
  const panel = page.locator('[data-live-panel]');
  const setJoint = (j, degrees) => page.locator(`#live-joint-${j}`).evaluate((input, value) => {
    input.value = String(value); input.dispatchEvent(new Event('input', { bubbles: true }));
  }, degrees);

  const rig = page.locator('[data-rig] input');
  assert.equal(await rig.isChecked(), false, 'the rig is off by default');
  await rig.check();
  assert.match(await panel.innerText(), /keeping clear of the MK 111 wall and floor/);

  await page.getByRole('button', { name: 'Set start and goal position' }).click();
  if (shots) await page.screenshot({ path: `${shots}/rig_editor.png` });
  // Stretch the start arm out over base +y, where the floor is.
  await setJoint(0, 90); await setJoint(1, -10); await setJoint(2, 0);
  assert.match(await panel.innerText(), /start pose goes through the floor/);
  if (shots) await page.screenshot({ path: `${shots}/rig_through_floor.png` });
  await rig.uncheck();
  assert.doesNotMatch(await panel.innerText(), /through the floor/, 'no rig, no warning');
  await rig.check();
  await page.getByRole('button', { name: 'Reset to experiment' }).click();
  assert.doesNotMatch(await panel.innerText(), /through the floor/);
  await page.getByRole('button', { name: 'Done' }).click();

  for (const label of ['Base cost off', 'σ = 0.3', 'σ = 0.1']) await page.locator('.live-runs label', { hasText: label }).locator('input').uncheck();
  await page.locator('[data-solve]').click();
  await page.getByText('Live solve', { exact: true }).waitFor({ timeout: 120000 });
  await page.waitForTimeout(500);
  if (shots) await page.screenshot({ path: `${shots}/rig_solved.png` });
  assert.deepEqual(errors, []);
  console.log('PASS: rig checkbox, through-the-floor warning, live UR10e solve with the rig');
} finally {
  await browser.close();
  await server.close();
}
