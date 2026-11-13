/** Content order, demo fit, and diagram/replay links on the built static site. */
import assert from 'node:assert/strict';
import { mkdir } from 'node:fs/promises';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { preview } from 'vite';
import registry from '../registry.json' with { type: 'json' };

const apps = registry.filter(app => !app.parentTab);

const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = fileURLToPath(new URL('../', import.meta.url));
const ready = {
  urdf: '.urdf-app[data-mode="browser"] #status-badge:has-text("Ready")',
  'contact-labels': '.label-scene canvas',
  'contact-initialization': '.scene canvas',
  'contact-vs-ik': '.comparison .scene canvas',
  'kinematics-inspector': '.scene canvas',
  interpolation: '.interpolation-scene canvas',
  dynamics: '.dynamics-scene[data-ready="true"]',
  kinodynamics: '.kinodynamics-scene[data-ready="true"]',
  'motion-planning': '.robot-canvas canvas',
  'scate/ur10e-reactive': '#stage[data-ready="true"] canvas',
  'steap-cmrg': '.steap-scene canvas',
};
const controls = {
  urdf: '#panel',
  'contact-labels': '.time-controls',
  'contact-initialization': '.prior-controls',
  'contact-vs-ik': '.iteration-controls',
  'kinematics-inspector': '.trajectory-viewer .timeline',
  interpolation: '.viewer-panel .timeline',
  dynamics: '.display-options',
  kinodynamics: '.display-options',
  'motion-planning': '.playback-controls',
  'scate/ur10e-reactive': '.transport',
};
async function fitsWidth(frame) {
  // Plotly resizes asynchronously after the viewport and its containers change.
  return frame.locator('#demo').evaluate(async () => {
    for (let attempt = 0; attempt < 40; attempt++) {
      if (document.documentElement.scrollWidth <= innerWidth + 1) return true;
      await new Promise(resolve => setTimeout(resolve, 50));
    }
    return false;
  });
}
const browser = await chromium.launch({
  executablePath: process.env.CHROME_EXECUTABLE
    || (existsSync('/usr/bin/google-chrome') ? '/usr/bin/google-chrome' : undefined),
  headless: true, args: ['--no-sandbox', '--enable-unsafe-swiftshader'],
});
try {
  for (const base of ['/', '/GTDynamics/']) {
    const server = await preview({ root, configFile: false, base,
      preview: { host: '127.0.0.1', port: 0, strictPort: true } });
    const context = await browser.newContext({ viewport: { width: 1440, height: 900 } });
    try {
      const page = await context.newPage();
      page.setDefaultTimeout(60_000);
      const errors = [], failures = [];
      page.on('pageerror', error => errors.push(error.message));
      page.on('response', response => {
        if (response.status() >= 400 && !response.url().endsWith('/favicon.ico')) failures.push(response.url());
      });
      const url = `http://127.0.0.1:${server.httpServer.address().port}${base}`;
      for (const app of apps) {
        await page.setViewportSize({ width: 1440, height: 900 });
        await page.goto(`${url}#${app.id}`);
        const frame = page.frameLocator('iframe');
        await frame.locator(ready[app.id]).first().waitFor();
        const nativePaper = app.id === 'interpolation';
        const paper = frame.locator(nativePaper ? 'main' : '.app-paper');
        const abstract = frame.locator(nativePaper ? '.abstract' : '.app-abstract');
        await paper.waitFor();
        if (app.id === 'motion-planning') await frame.locator('.loading-overlay').waitFor({ state: 'hidden' });
        assert.equal(await abstract.count(), 1);
        assert.deepEqual(await paper.locator(':scope > [data-paper-section]').evaluateAll(
          sections => sections.map(section => section.dataset.paperSection)),
        ['motivation', 'approach', 'results', 'discussion']);
        assert(await frame.locator('[data-paper-section="approach"] svg').count() > 0, `${app.id}: approach has a graph`);
        assert(await frame.locator(nativePaper ? '.factor-definitions article' : '.factor-definitions dt').count() >= (nativePaper ? 5 : 6), `${app.id}: factor inventory`);
        assert(await abstract.evaluate(node => Boolean(
          node.compareDocumentPosition(document.querySelector('canvas')) & Node.DOCUMENT_POSITION_FOLLOWING)),
        `${app.id}: abstract precedes demo`);

        if (['contact-labels','contact-initialization','contact-vs-ik','kinematics-inspector','kinodynamics'].includes(app.id)) {
          assert(await paper.locator('mjx-container[jax="SVG"]').count() >= 4, `${app.id}: in-app mathematical definitions`);
          assert.equal(await paper.locator('[data-mml-node="merror"]').count(), 0);
          assert.equal(await paper.locator('[data-math-parameters]').count(), 1);
          assert.equal(await frame.locator('a[href*="contact-factors.html"]').count(), 0);
        }

        if (base === '/') {
          for (const size of [{ width: 1440, height: 900 }, { width: 1280, height: 800 }]) {
            await page.setViewportSize(size);
            await frame.locator('#demo').evaluate(() => window.scrollTo(0, 0));
            const box = await page.locator('iframe').boundingBox();
            if (controls[app.id]) {
              const bottom = Math.max(...await frame.locator(controls[app.id]).evaluateAll(
                nodes => nodes.map(node => node.getBoundingClientRect().bottom)));
              assert(bottom <= box.height, `${app.id}: controls end at ${bottom.toFixed(0)}, iframe ${box.height.toFixed(0)} (${size.width}×${size.height})`);
            }
            assert(await fitsWidth(frame), `${app.id}: no horizontal overflow`);
          }
          if (process.env.SCREENSHOT_DIR) {
            await mkdir(process.env.SCREENSHOT_DIR, { recursive: true });
            await page.screenshot({ path: `${process.env.SCREENSHOT_DIR}/${app.id.replaceAll('/', '-')}.png` });
          }
        }
        if (app.id === 'contact-vs-ik') {
          assert.equal(await frame.locator('.comparison-results .method-results').count(), 2);
          await frame.getByRole('slider', { name: 'Contact initialization iteration', exact: true }).fill('0');
          await frame.locator('.method-results.contact[data-result-iteration="0"]').waitFor();
        }
        if (app.id === 'urdf') {
          assert.equal(await paper.locator('mjx-container[jax="SVG"]').count(), 3);
          await frame.getByText('Live browser solve', { exact: true }).waitFor();
        } else if (app.id === 'contact-initialization') {
          await frame.getByRole('button', { name: 'Select event 1', exact: true }).click();
          assert.equal(await frame.locator('main').getAttribute('data-event'), '1');
        } else if (nativePaper) {
          const before = await frame.locator('main').getAttribute('data-frame');
          await frame.getByRole('button', { name: /^Go to knot / }).last().click();
          assert.notEqual(await frame.locator('main').getAttribute('data-frame'), before);
        } else if (app.id === 'steap-cmrg') {
          const before = await frame.locator('main').getAttribute('data-frame');
          await frame.getByRole('button', { name: 'Next ▶', exact: true }).click();
          assert.notEqual(await frame.locator('main').getAttribute('data-frame'), before);
        } else {
          if (app.id === 'scate/ur10e-reactive') await frame.getByRole('slider', {name: 'Time', exact: true}).fill('0');
          await frame.getByRole('button', { name: /^Show slice 1 in demo/ }).first().click();
          if (app.id !== 'motion-planning') assert.equal(await frame.locator('main').getAttribute('data-frame'), '1');
        }
        if (['dynamics', 'kinodynamics'].includes(app.id)) {
          await frame.getByRole('checkbox', { name: 'All joint torques', exact: true }).check();
          await frame.getByRole('link', { name: 'View torque plots' }).click();
          await frame.locator('.torque-section .signal-plot[data-ready="true"]').first().waitFor();
          const box = await frame.locator('.torque-section').boundingBox();
          assert(box.y < 900, 'Torque link reveals the results');
        }
        if (base === '/') {
          await page.setViewportSize({ width: 390, height: 844 });
          assert(await fitsWidth(frame), `${app.id}: fits mobile width`);
        }
        if (app.id !== apps.at(-1).id) {
          const next = apps[apps.indexOf(app) + 1];
          await frame.getByRole('link', { name: `Next: ${next.title} →`, exact: true }).click();
          await page.getByRole('tab', { name: next.title, selected: true }).waitFor();
          assert.equal(new URL(page.url()).pathname, base, 'Next-app link preserves deployment prefix');
        }
        console.log(`PASS ${base}${app.id}: paper structure, demo fit, graph/replay links, results`);
      }
      assert.deepEqual(errors, []);
      assert.deepEqual(failures, []);
    } finally {
      await context.close();
      server.httpServer.closeAllConnections();
      await new Promise(resolve => server.httpServer.close(resolve));
    }
  }
} finally {
  await browser.close();
}
