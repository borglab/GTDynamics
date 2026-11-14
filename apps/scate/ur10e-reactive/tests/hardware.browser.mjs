// Browser check of the hardware experiment flow with the simulated arm: the
// dev server starts the executor itself (python3 -m scate_hw), the page plans
// for hardware, solves the ghost run, begins, watches the run and loads the
// hardware run beside its ghost run. Needs Chrome, Playwright, and built
// scate_live and scate_hardware. Takes about a minute (the run is real time).
//   node tests/hardware.browser.mjs [screenshot-dir]
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { createServer } from 'vite';
import { findScateHardware, findScateLive } from '../../shared/liveServer.mjs';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');

if (!findScateLive() || !findScateHardware()) {
  console.log('SKIP: scate_live and scate_hardware must be built');
  process.exit(0);
}
delete process.env.SCATE_HW_URL;  // let the dev server start the simulated arm
process.env.SCATE_HW_AUTOSTART = '1';
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

  await page.locator('[data-hardware-mode] input').check();
  assert.equal(await page.locator('[data-rig] input').isChecked(), true, 'hardware mode includes the rig');
  assert.match(await panel.innerText(), /rig experiment’s start and goal/);
  await page.locator('[data-hardware-panel]').waitFor();
  await page.getByText(/Executor: simulated arm/).waitFor({ timeout: 60000 });
  assert.match(await page.locator('[data-hw-blocked]').innerText(), /Solve and run first/);
  assert.equal(await page.locator('[data-begin]').isDisabled(), true);

  await page.getByRole('button', { name: 'Set start and goal position' }).click();
  if (shots) await page.screenshot({ path: `${shots}/hw_editor_lab_up.png` });
  await page.getByRole('button', { name: 'Done' }).click();

  for (const label of ['Base cost off', 'σ = 0.3', 'σ = 0.1']) await page.locator('.live-runs label', { hasText: label }).locator('input').uncheck();
  await page.locator('[data-solve]').click();
  await page.getByText('Live solve', { exact: true }).waitFor({ timeout: 120000 });
  await page.locator('[data-ghost-safety]').waitFor();
  assert.match(await page.locator('[data-ghost-safety]').innerText(), /Ghost run safe/);
  await page.locator('[data-begin]:not([disabled])').waitFor();
  if (shots) await page.screenshot({ path: `${shots}/hw_ghost.png` });

  // Changing the setup makes the ghost run stale.
  await page.locator('[data-limits] input').first().fill('0.4');
  await page.locator('[data-limits] input').first().dispatchEvent('input');
  assert.match(await page.locator('[data-hw-blocked]').innerText(), /setup changed/);
  await page.locator('[data-limits] input').first().fill('0.5');
  await page.locator('[data-limits] input').first().dispatchEvent('input');
  await page.locator('[data-begin]:not([disabled])').waitFor();

  await page.locator('[data-begin]').click();
  await page.locator('[data-hardware-panel][data-state="running"]').waitFor({ timeout: 180000 });
  await page.locator('[data-hw-progress]').waitFor();
  assert.equal(await page.locator('[data-solve]').isDisabled(), true, 'no solving during a run');
  await page.waitForTimeout(8000);
  if (shots) await page.screenshot({ path: `${shots}/hw_running.png` });
  await page.locator('[data-hardware-panel][data-state="done"]').waitFor({ timeout: 90000 });
  await page.locator('#srcTag', { hasText: 'Hardware run' }).waitFor({ timeout: 30000 });
  await page.waitForTimeout(500);
  const runs = await page.locator('[data-run-toggles] .run-toggle').evaluateAll(rows => rows.map(r => r.dataset.run));
  if (!runs.length) console.log('DEBUG', errors, await panel.innerText(), await page.locator('#srcTag').innerText());
  assert.deepEqual(runs, ['online', 'ghost']);
  assert.match(await page.locator('[data-hw-result]').innerText(), /Hardware run complete/);
  await page.waitForTimeout(500);
  if (shots) await page.screenshot({ path: `${shots}/hw_result.png` });
  assert.deepEqual(errors, []);
  console.log('PASS: hardware mode, ghost solve, begin, real-time run on the simulated arm, hardware replay loaded');
} finally {
  await browser.close();
  await server.close();
}
