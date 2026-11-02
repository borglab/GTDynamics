/** Run after building; supply PLAYWRIGHT_MODULE from an external installation. */
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { preview } from 'vite';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');

const root = fileURLToPath(new URL('../../', import.meta.url));
const browser = await chromium.launch({
  executablePath: process.env.CHROME_BIN || '/usr/bin/google-chrome',
  headless: true, args: ['--no-sandbox', '--disable-dev-shm-usage', '--enable-unsafe-swiftshader'],
});
try {
  for (const path of ['/', '/GTDynamics/']) {
    const server = await preview({ root, configFile: false, base: path,
      preview: { host: '127.0.0.1', port: 0, strictPort: true } });
    const url = `http://127.0.0.1:${server.httpServer.address().port}`;
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
    page.setDefaultTimeout(60000);
    const errors = [], requests = [];
    page.on('pageerror', error => errors.push(error.message));
    page.on('request', request => requests.push(request.url()));
    await page.goto(`${url}${path}interpolation/`);
    await page.waitForSelector('.interpolation-scene canvas');
    await page.waitForFunction(() => document.querySelectorAll('.plotly-chart[data-ready="true"]').length === 5);
    assert.equal(await page.getByRole('heading', { name: 'Interpolation', exact: true }).count(), 1);
    assert.equal(await page.getByRole('combobox', { name: 'Joint', exact: true }).locator('option').count(), 29);
    const requestCount = requests.length;
    await page.getByRole('combobox', { name: 'Joint', exact: true }).selectOption('20');
    await page.getByRole('checkbox', { name: 'Sole velocity vectors', exact: false }).check();
    await page.getByRole('checkbox', { name: 'Retained knots', exact: true }).uncheck();
    await page.getByRole('button', { name: 'Follow robot', exact: true }).click();
    await page.getByRole('button', { name: 'Play', exact: true }).click();
    const initial = await page.locator('main').getAttribute('data-frame');
    await page.waitForFunction(frame => document.querySelector('main').dataset.frame !== frame, initial);
    await page.getByRole('button', { name: 'Pause', exact: true }).click();
    await page.getByRole('combobox', { name: 'Playback window' }).selectOption('full');
    await page.getByRole('button', { name: 'Fit trajectory', exact: true }).click();
    assert.equal(requests.length, requestCount, 'Controls reuse one loaded trajectory');
    assert(!requests.some(request => /\.wasm|variants\//.test(request)));
    assert.deepEqual(errors, []);
    if (path === '/') {
      await page.getByRole('combobox', { name: 'Playback window' }).selectOption('middle');
      await page.getByRole('combobox', { name: 'Joint', exact: true }).selectOption({ label: 'left_knee_joint' });
      await page.getByRole('checkbox', { name: 'Retained knots', exact: true }).check();
      await page.getByRole('checkbox', { name: 'Sole velocity vectors', exact: false }).uncheck();
      await page.getByRole('button', { name: 'Fit trajectory', exact: true }).click();
      await page.getByRole('button', { name: 'Follow robot', exact: true }).click();
      await page.screenshot({ path: '/tmp/gtd-interpolation.png', fullPage: true });
    }
    await page.goto(`${url}${path}#interpolation`);
    await page.frameLocator('iframe').locator('.interpolation-scene canvas').waitFor();
    const tabs = await page.getByRole('tab').allTextContents();
    assert.equal(tabs.at(-1).trim(), 'Interpolation');
    assert.equal(tabs.at(-2).trim(), 'Kinematics');
    await page.getByRole('tab', { name: 'Kinematics', exact: true }).click();
    await page.frameLocator('iframe').locator('.large-scene canvas').waitFor();
    await page.getByRole('tab', { name: 'Interpolation', exact: true }).click();
    await page.frameLocator('iframe').locator('.interpolation-scene canvas').waitFor();
    assert.deepEqual(errors, [], 'Leaving and reopening the app must be clean');
    await page.close();
    await new Promise(resolve => server.httpServer.close(resolve));
  }
  console.log('Interpolation browser checks passed at / and /GTDynamics/.');
} finally { await browser.close(); }
