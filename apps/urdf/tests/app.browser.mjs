/** Live browser IK from static assets and native IK when GTDynamics is available. */
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { createServer, preview } from 'vite';
import { startBackend, waitForBackend } from '../scripts/backend.mjs';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = fileURLToPath(new URL('../', import.meta.url));
const browser = await chromium.launch({
  executablePath: process.env.CHROME_EXECUTABLE || (existsSync('/usr/bin/google-chrome') ? '/usr/bin/google-chrome' : undefined),
  headless: true, args: ['--no-sandbox', '--enable-unsafe-swiftshader'],
});

async function check(url, run) {
  const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
  page.setDefaultTimeout(60_000);
  const errors = [];
  page.on('pageerror', (error) => errors.push(error.message));
  try {
    await page.goto(url);
    await run(page);
    assert.deepEqual(errors, []);
  } finally {
    await page.close();
  }
}

let backend;
try {
  // The static site must load and solve with only its built assets.
  for (const base of ['/', '/GTDynamics/urdf/']) {
    const server = await preview({ root, configFile: false, base, preview: { host: '127.0.0.1', port: 0, strictPort: true } });
    try {
      await check(`http://127.0.0.1:${server.httpServer.address().port}${base}`, async (page) => {
        const origin = new URL(page.url()).origin;
        const unexpected = [], failures = [];
        page.on('response', response => {
          if (response.status() >= 400 && !response.url().endsWith('/favicon.ico')) failures.push(response.url());
        });
        await page.route('**/*', route => {
          const url = new URL(route.request().url());
          if (url.origin !== origin || /\/(api|files|pkg)\//.test(url.pathname)) {
            unexpected.push(url.href);
            return route.abort();
          }
          return route.continue();
        });
        await page.reload();
        await page.locator('.urdf-app[data-mode="browser"]').waitFor();
        const status = page.locator('#status-badge');
        const model = page.locator('#model');
        await status.filter({ hasText: /^Ready$/ }).waitFor();
        assert.deepEqual(await model.locator('option').evaluateAll(options => options.map(option => option.value)),
          ['ur10e/ur10e.urdf', 'ur5/ur5.urdf']);
        assert.deepEqual(await page.locator('#scheme option').allTextContents(), ['Damped least squares']);
        const coordinate = async axis => Number(await page.locator(`#cur-${axis}`).innerText());
        for (const [name, home] of [
          ['ur10e/ur10e.urdf', [1.18425, 0.2907, 0.06085]],
          ['ur5/ur5.urdf', [0.81725, 0.19145, -0.005491]],
        ]) {
          if (await model.inputValue() !== name) await model.selectOption(name);
          await status.filter({ hasText: /^Ready$/ }).waitFor();
          assert.equal(await page.locator('#ee').inputValue(), 'tool0');
          assert.equal(await page.locator('#joints input[type=range]').count(), 6);
          assert(await page.locator('#orientation').isChecked());
          const targetX = (await coordinate(0) - 0.035).toFixed(4);
          await page.locator('#tgt-0').fill(targetX);
          await page.locator('#solve').click();
          await status.filter({ hasText: /^Solved$/ }).waitFor();
          assert(Math.abs(await coordinate(0) - Number(targetX)) <= 0.0011);
          assert.match(await page.locator('#result-summary').innerText(), /mm.*°/);
          await page.locator('#home').click();
          await status.filter({ hasText: 'Forward kinematics' }).waitFor();
          for (const [axis, expected] of home.entries()) {
            assert(Math.abs(await coordinate(axis) - expected) < 0.0001, `${name} home axis ${axis}`);
          }
          await page.locator('#tgt-0').fill('5');
          await page.locator('#solve').click();
          await status.filter({ hasText: /^Not solved$/ }).waitFor();
          assert.match(await page.locator('#reasons').innerText(), /maximum reach/);
        }
        await page.waitForFunction(() =>
          new Set(performance.getEntriesByType('resource').map(entry => entry.name)
            .filter(url => url.endsWith('.dae'))).size === 14);
        assert.equal(await page.locator('#viewport canvas').count(), 1);
        assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
        assert.deepEqual(unexpected, []);
        assert.deepEqual(failures, []);
      });
      console.log(`PASS browser IK ${base}: both models, FK, pose solve and reach diagnostics without a backend`);
    } finally {
      server.httpServer.closeAllConnections();
      await new Promise((resolve) => server.httpServer.close(resolve));
    }
  }

  backend = startBackend();
  if (!backend) {
    console.log('SKIP native IK: no GTDynamics Python');
  } else {
    await waitForBackend();
    // The dev server proxies api/, files/ and pkg/ to the backend, also under a base path.
    process.env.NODE_ENV = 'development';
    const server = await createServer({ root, base: '/urdf/', server: { host: '127.0.0.1', port: 0 } });
    await server.listen();
    try {
      await check(`http://127.0.0.1:${server.httpServer.address().port}/urdf/`, async (page) => {
        await page.locator('.urdf-app[data-mode="native"]').waitFor();
        await page.locator('#status-badge', { hasText: 'Ready' }).waitFor();
        assert.match(await page.locator('#model').inputValue(), /^ur10e\//);
        assert.equal(await page.locator('#joints input[type=range]').count(), 6);
        assert.equal(await page.locator('#scheme').inputValue(), 'gtd');
        for (const size of [{ width: 1440, height: 775 }, { width: 1280, height: 675 }]) {
          // The shell consumes about 125 pixels above the app at these widths.
          await page.setViewportSize(size);
          assert(await page.locator('#demo').evaluate(demo => demo.getBoundingClientRect().bottom <= innerHeight));
          assert(await page.locator('#joints').evaluate(joints => joints.scrollHeight <= joints.clientHeight));
        }
        await page.locator('#tgt-0').fill('0.6');
        await page.locator('#solve').click();
        await page.locator('#status-badge', { hasText: /^(Solved|Not solved)$/ }).waitFor();
        assert.match(await page.locator('#result-summary').innerText(), /mm/);
        await page.locator('#home').click();
        await page.locator('#status-badge', { hasText: 'Forward kinematics' }).waitFor();
        await page.locator('#orientation').uncheck();
        await page.getByRole('group', { name: 'urdf-point factor graph schematic' }).waitFor();
        await page.locator('#model').selectOption('ur5/ur5.urdf');
        await page.locator('#status-badge', { hasText: 'Ready' }).waitFor();
        await page.locator('#show-skeleton').uncheck();
        await page.locator('#show-meshes').uncheck();
        await page.locator('#show-meshes').check();
        await page.locator('#fit').click();
        assert.equal(await page.locator('#viewport canvas').count(), 1);
        await page.setViewportSize({ width: 390, height: 730 });
        assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
      });
      console.log('PASS native factor graph /urdf/');
    } finally {
      await server.close();
    }
  }
} finally {
  backend?.kill('SIGTERM');
  await browser.close();
}
