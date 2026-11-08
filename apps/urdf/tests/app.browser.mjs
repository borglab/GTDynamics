/** URDF IK: offline notice on the static build, and (with GTDynamics Python) a live solve. */
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
  // npm run build first: the static site has no backend, so the page explains how to run one.
  for (const base of ['/', '/GTDynamics/urdf/']) {
    const server = await preview({ root, configFile: false, base, preview: { host: '127.0.0.1', port: 0, strictPort: true } });
    try {
      await check(`http://127.0.0.1:${server.httpServer.address().port}${base}`, async (page) => {
        await page.locator('.urdf-app[data-mode="offline"]').waitFor();
        await page.getByRole('heading', { name: 'Local GTDynamics server required' }).waitFor();
        assert.equal(await page.locator('#status-badge').innerText(), 'Backend not running');
        assert.equal(await page.locator('#viewport canvas').count(), 1);
        assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
      });
      console.log(`PASS offline ${base}`);
    } finally {
      server.httpServer.closeAllConnections();
      await new Promise((resolve) => server.httpServer.close(resolve));
    }
  }

  backend = startBackend();
  if (!backend) {
    console.log('SKIP live: no GTDynamics Python');
  } else {
    await waitForBackend();
    // The dev server proxies api/, files/ and pkg/ to the backend, also under a base path.
    process.env.NODE_ENV = 'development';
    const server = await createServer({ root, base: '/urdf/', server: { host: '127.0.0.1', port: 0 } });
    await server.listen();
    try {
      await check(`http://127.0.0.1:${server.httpServer.address().port}/urdf/`, async (page) => {
        await page.locator('.urdf-app[data-mode="live"]').waitFor();
        await page.locator('#status-badge', { hasText: 'Ready' }).waitFor();
        assert.match(await page.locator('#model').inputValue(), /^ur10e\//);
        assert.equal(await page.locator('#joints input[type=range]').count(), 6);
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
      console.log('PASS live /urdf/');
    } finally {
      await server.close();
    }
  }
} finally {
  backend?.kill('SIGTERM');
  await browser.close();
}
