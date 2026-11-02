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
    const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
    page.setDefaultTimeout(60000);
    const errors = [], requests = [];
    page.on('pageerror', error => errors.push(error.message));
    page.on('request', request => requests.push(request.url()));
    await page.goto(`${url}${path}interpolation/`);
    await page.waitForSelector('.interpolation-scene canvas');
    await page.waitForFunction(() => document.querySelectorAll('.plotly-chart[data-ready="true"]').length === 5);
    assert.equal(await page.getByRole('heading', { name: 'Interpolation', exact: true }).count(), 1);
    assert.equal(await page.getByRole('combobox', { name: 'Joint', exact: true }).locator('option').count(), 29);
    const previousLink = page.locator('.abstract a');
    assert.equal(await previousLink.getAttribute('href'), `${url}${path}kinematics-inspector/`);
    const oldInterval = await page.locator('.factor-graph svg').getAttribute('data-interval');
    await page.locator('.graph-slice').last().click();
    assert.notEqual(await page.locator('.factor-graph svg').getAttribute('data-interval'), oldInterval,
      'Graph selection moves playback into the selected knot interval');
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
    const embedded = page.frameLocator('iframe');
    assert.equal(await embedded.locator('.abstract a').getAttribute('href'), `${url}${path}#kinematics-inspector`);
    const viewerBottom = await embedded.locator('.viewer-panel').evaluate(element => element.getBoundingClientRect().bottom);
    const frameHeight = await page.locator('iframe').evaluate(element => element.clientHeight);
    assert(viewerBottom <= frameHeight, 'Complete demo and controls fit below shared navigation at 1440×900');
    const sections = await embedded.locator('main > section, main > footer').evaluateAll(elements =>
      elements.map(element => element.querySelector('h2')?.textContent || 'Demo'));
    assert.deepEqual(sections, ['Demo', 'From sparse configurations to velocities', 'Approach',
      'Factors and fixed inputs', 'Recorded results', 'Discussion', 'Data, model, and downloads']);
    const tabs = await page.getByRole('tab').allTextContents();
    assert.equal(tabs.at(-1).trim(), 'Interpolation');
    assert.equal(tabs.at(-2).trim(), 'Kinematics');
    await page.getByRole('tab', { name: 'Kinematics', exact: true }).click();
    await page.frameLocator('iframe').locator('.large-scene canvas').waitFor();
    await page.getByRole('tab', { name: 'Interpolation', exact: true }).click();
    await page.frameLocator('iframe').locator('.interpolation-scene canvas').waitFor();
    assert.deepEqual(errors, [], 'Leaving and reopening the app must be clean');
    if (path === '/') {
      await page.screenshot({ path: '/tmp/gtd-interpolation-paper.png' });
      await embedded.locator('.factor-graph').screenshot({ path: '/tmp/gtd-interpolation-graph.png' });
      await page.setViewportSize({ width: 1366, height: 768 });
      await page.waitForFunction(() => {
        const frame = document.querySelector('iframe');
        return frame.contentDocument.querySelector('.viewer-panel').getBoundingClientRect().bottom <= frame.clientHeight;
      });
      await page.setViewportSize({ width: 390, height: 844 });
      await page.waitForFunction(() => {
        const frame = document.querySelector('iframe');
        return frame.contentDocument.body.scrollWidth <= frame.contentWindow.innerWidth;
      });
      assert(await embedded.locator('body').evaluate(element => element.scrollWidth <= window.innerWidth),
        'Tables and graph scroll inside their panels on mobile');
      await embedded.getByRole('checkbox', { name: 'Retained knots', exact: true }).uncheck();
      await page.screenshot({ path: '/tmp/gtd-interpolation-mobile.png' });
    }
    await page.close();
    await new Promise(resolve => server.httpServer.close(resolve));
  }
  console.log('Interpolation browser checks passed at / and /GTDynamics/.');
} finally { await browser.close(); }
