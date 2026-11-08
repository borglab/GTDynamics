/** Cross-app preference, live recoloring, and independent-document synchronization. */
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { mkdir } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { preview } from 'vite';
import apps from '../registry.json' with { type: 'json' };

const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = fileURLToPath(new URL('../', import.meta.url));
const browser = await chromium.launch({
  executablePath: process.env.CHROME_EXECUTABLE || (existsSync('/usr/bin/google-chrome') ? '/usr/bin/google-chrome' : undefined),
  headless: true, args: ['--no-sandbox', '--enable-unsafe-swiftshader'],
});
const ready = {
  'contact-labels': '.label-scene canvas', 'contact-initialization': '.scene canvas',
  'contact-vs-ik': '.comparison .scene canvas', 'kinematics-inspector': '.scene canvas',
  interpolation: '.interpolation-scene canvas', dynamics: '.dynamics-scene[data-ready="true"] canvas',
  kinodynamics: '.kinodynamics-scene[data-ready="true"] canvas', 'motion-planning': '.robot-canvas canvas',
  'scate/ur5': '#stage[data-ready="true"] canvas',
  urdf: '.urdf-app[data-mode="browser"] #status-badge:has-text("Ready")',
};
try {
  for (const base of ['/', '/GTDynamics/']) {
    const server = await preview({root, configFile: false, base, preview: {host: '127.0.0.1', port: 0, strictPort: true}});
    const context = await browser.newContext({viewport: {width: 1440, height: 900}});
    try {
      const url = `http://127.0.0.1:${server.httpServer.address().port}${base}`;
      const page = await context.newPage(), errors = [], failures = [];
      page.setDefaultTimeout(60_000);
      page.on('pageerror', error => errors.push(error.message));
      page.on('response', response => { if (response.status() >= 400) failures.push(response.url()); });
      await page.goto(`${url}#scate/ur5`);
      const frame = page.frameLocator('iframe');
      await frame.locator(ready['scate/ur5']).waitFor();
      assert.equal(await frame.locator('html').getAttribute('data-theme'), 'light');
      await frame.locator('#stage canvas').evaluate(canvas => { canvas.dataset.identity = 'same-viewer'; });
      const time = await frame.locator('#timeTxt').innerText();
      await page.getByRole('button', {name: 'Dark theme', exact: true}).click();
      await frame.locator('html[data-theme="dark"]').waitFor();
      assert.equal(await frame.locator('#stage canvas').getAttribute('data-identity'), 'same-viewer');
      assert.equal(await frame.locator('#timeTxt').innerText(), time, 'Theme changes preserve replay time');
      await page.reload();
      await page.frameLocator('iframe').locator(ready['scate/ur5']).waitFor();
      assert.equal(await page.getByRole('button', {name: 'Dark theme', exact: true}).getAttribute('aria-pressed'), 'true');
      const direct = await context.newPage();
      await direct.goto(`${url}scate/ur5/`);
      await direct.locator('#stage[data-ready="true"]').waitFor();
      assert.equal(await direct.locator('html').getAttribute('data-theme'), 'dark');
      await page.getByRole('button', {name: 'Dark theme', exact: true}).click();
      await direct.locator('html[data-theme="light"]').waitFor();
      await direct.close();
      await page.getByRole('button', {name: 'Dark theme', exact: true}).click();
      for (const app of apps) {
        await page.getByRole('tab', {name: app.title, exact: true}).click();
        const appFrame = page.frameLocator('iframe');
        await appFrame.locator(ready[app.id]).first().waitFor();
        if (app.id === 'motion-planning') await appFrame.locator('.loading-overlay').waitFor({state: 'hidden'});
        assert.equal(await appFrame.locator('html').getAttribute('data-theme'), 'dark');
        const background = await appFrame.locator('html').evaluate(node => getComputedStyle(node).backgroundColor);
        const channels = background.match(/[\d.]+/g).slice(0, 3).map(Number);
        assert(channels.every(channel => channel < 60), `${app.id}: dark page background (${background})`);
        if (process.env.SCREENSHOT_DIR && base === '/') {
          await mkdir(process.env.SCREENSHOT_DIR, {recursive: true});
          await page.screenshot({path: `${process.env.SCREENSHOT_DIR}/${app.id.replaceAll('/', '-')}-dark.png`});
        }
        await page.getByRole('button', {name: 'Dark theme', exact: true}).click();
        await appFrame.locator('html[data-theme="light"]').waitFor();
        await page.getByRole('button', {name: 'Dark theme', exact: true}).click();
        console.log(`PASS ${base}${app.id}: loads dark, recolors in place`);
      }
      // Select SCATE explicitly: the loop above ends on whichever app is registered last.
      await page.getByRole('tab', {name: 'SCATE UR5', exact: true}).click();
      const scate = page.frameLocator('iframe');
      await scate.locator(ready['scate/ur5']).waitFor();
      await scate.locator('.signal-plot[data-ready="true"]').first().waitFor();
      await scate.locator('html[data-theme="dark"]').waitFor();
      const plotColor = await scate.locator('.js-plotly-plot').first().evaluate(async plot => {
        for (let i = 0; i < 60 && plot._fullLayout.plot_bgcolor !== '#141d25'; i++) await new Promise(resolve => setTimeout(resolve, 50));
        return plot._fullLayout.plot_bgcolor;
      });
      assert.equal(plotColor, '#141d25');
      await page.setViewportSize({width: 390, height: 844});
      assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), 'Global toggle fits mobile navigation');
      assert.deepEqual(errors, []);
      assert.deepEqual(failures, []);
      console.log(`PASS ${base}: persistent preference, direct apps, storage sync, plot palette, mobile header`);
    } finally {
      await context.close(); server.httpServer.closeAllConnections();
      await new Promise(resolve => server.httpServer.close(resolve));
    }
  }
} finally { await browser.close(); }
