/** Built-site smoke test. Supply PLAYWRIGHT_MODULE from an external installation. */
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { mkdir } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { preview } from 'vite';
import apps from '../registry.json' with { type: 'json' };

const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = fileURLToPath(new URL('../', import.meta.url));
const browser = await chromium.launch({
  executablePath: process.env.CHROME_EXECUTABLE
    || (existsSync('/usr/bin/google-chrome') ? '/usr/bin/google-chrome' : undefined),
  headless: true,
  args: ['--no-sandbox', '--enable-unsafe-swiftshader'],
});
try {
  for (const base of ['/', '/GTDynamics/']) {
    const server = await preview({
      root, configFile: false, base,
      preview: { host: '127.0.0.1', port: 0, strictPort: true },
    });
    const context = await browser.newContext({ viewport: { width: 1440, height: 1100 } });
    try {
      const url = `http://127.0.0.1:${server.httpServer.address().port}${base}`;
      const page = await context.newPage();
      page.setDefaultTimeout(60_000);
      const errors = [], failedRequests = [], sockets = [];
      page.on('pageerror', error => errors.push(error.message));
      page.on('response', response => {
        if (response.status() >= 400 && !response.url().endsWith('/favicon.ico')) {
          failedRequests.push(`${response.status()} ${response.url()}`);
        }
      });
      page.on('websocket', socket => sockets.push(socket.url()));
      await page.goto(url);
      console.log(`Checking ${base}: overview and navigation`);
      assert.equal(await page.getByRole('tab', { name: 'Overview' }).getAttribute('aria-selected'), 'true');
      assert.equal(await page.locator('iframe').count(), 0, 'Overview does not load a simulation');
      assert.equal(await page.locator('.app-link').count(), apps.length);
      if (process.env.SCREENSHOT_DIR && base === '/') {
        await mkdir(process.env.SCREENSHOT_DIR, { recursive: true });
        await page.screenshot({ path: `${process.env.SCREENSHOT_DIR}/overview.png`, fullPage: true });
      }

      // Keyboard activation follows the tab order.
      await page.getByRole('tab', { name: 'Overview' }).focus();
      await page.keyboard.press('ArrowRight');
      await page.getByRole('tab', { name: 'Motion Planning', selected: true }).waitFor();
      assert.equal(await page.getByRole('tab', { name: 'Motion Snippets' }).count(), 0);
      console.log(`Checking ${base}: Motion Planning assets and recorded solve`);
      const planning = page.frameLocator('iframe');
      await planning.getByText('Recorded native solve.', { exact: true }).waitFor();
      await planning.locator('.loading-overlay').waitFor({ state: 'hidden' });
      assert.equal(await planning.locator('.error-banner').count(), 0);
      assert.equal(await planning.locator('.joint-curve').count(), 14, 'Seven position and velocity plots');
      assert.equal(await planning.getByRole('button', { name: 'Build graph', exact: true }).count(), 0);
      await planning.getByRole('button', { name: 'Play arm trajectory' }).click();
      await page.waitForTimeout(500);
      assert(Number(await planning.getByRole('slider', { name: 'Arm trajectory time' }).inputValue()) > 0);
      await planning.getByRole('button', { name: 'Pause arm playback' }).click();
      await planning.getByRole('slider', { name: 'Recorded optimizer iteration' }).evaluate(input => {
        input.value = '0';
        input.dispatchEvent(new Event('input', { bubbles: true }));
      });
      await planning.locator('.joint-panel[data-iteration="0"]').waitFor();
      assert.equal(await planning.locator('.joint-panel').getAttribute('data-iteration'), '0');
      await planning.getByRole('button', { name: 'Follow latest' }).click();
      assert(Number(await planning.locator('.joint-panel').getAttribute('data-iteration')) > 0);
      await planning.getByRole('button', { name: 'Replay iterations' }).click();
      await planning.getByRole('button', { name: 'Pause iterations' }).waitFor();
      if (process.env.SCREENSHOT_DIR && base === '/') {
        await page.screenshot({ path: `${process.env.SCREENSHOT_DIR}/planning.png`, fullPage: true });
      }
      // A direct nested app URL also works without the shell or a server backend.
      const direct = await context.newPage();
      await direct.goto(`${url}motion-planning/`);
      await direct.getByText('Recorded native solve.', { exact: true }).waitFor();
      await direct.close();

      await page.getByRole('tab', { name: 'Overview' }).click();
      await page.getByRole('tab', { name: 'Overview', selected: true }).waitFor();
      assert.equal(await page.locator('iframe').count(), 0, 'Leaving an app unloads its simulation');
      await page.goBack();
      await page.getByRole('tab', { name: 'Motion Planning', selected: true }).waitFor();
      assert.equal(await page.getByRole('tab', { name: 'Motion Planning' }).getAttribute('aria-selected'), 'true');
      await page.goForward();
      await page.setViewportSize({ width: 390, height: 844 });
      assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), 'Overview fits mobile width');
      if (process.env.SCREENSHOT_DIR && base === '/') {
        await page.screenshot({ path: `${process.env.SCREENSHOT_DIR}/mobile.png`, fullPage: true });
      }
      assert.deepEqual(errors, [], 'No browser script errors');
      assert.deepEqual(failedRequests, [], 'All static assets load, including under a project prefix');
      assert.deepEqual(sockets, [], 'The static site never attempts a backend connection');
      console.log(`PASS ${base}: default overview, keyboard tabs, physics playback, recorded iterations, mobile, history, static assets`);
    } finally {
      await context.close();
      server.httpServer.closeAllConnections();
      await new Promise(resolve => server.httpServer.close(resolve));
    }
  }
} finally {
  await browser.close();
}
