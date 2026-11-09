/** Base-disturbance comparison: run toggles, running plots, results and fit in the hub. */
import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {preview} from 'vite';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=fileURLToPath(new URL('../../../',import.meta.url));
const browser=await chromium.launch({executablePath:process.env.CHROME_EXECUTABLE||(existsSync('/usr/bin/google-chrome')?'/usr/bin/google-chrome':undefined),headless:true,args:['--no-sandbox','--enable-unsafe-swiftshader']});
try {
  for(const base of ['/','/GTDynamics/']) {
    const server=await preview({root,configFile:false,base,preview:{host:'127.0.0.1',port:0,strictPort:true}});
    const context=await browser.newContext({viewport:{width:1440,height:900}});
    try {
      const page=await context.newPage(),errors=[],external=[];
      page.setDefaultTimeout(60000);
      page.on('pageerror',e=>errors.push(e.message));
      page.on('request',request=>{if(!request.url().startsWith('http://127.0.0.1:'))external.push(request.url());});
      await page.goto(`http://127.0.0.1:${server.httpServer.address().port}${base}#scate/ur10e-reactive`);
      const frame=page.frameLocator('iframe');
      await frame.locator('#stage[data-ready="true"]').waitFor();
      assert.equal(await frame.locator('.factor-definitions dt').count(),8);
      // One toggle per run; the online/blind radio and panel are hidden.
      const toggles=frame.locator('[data-run-toggles] .run-toggle');
      assert.deepEqual(await toggles.evaluateAll(nodes=>nodes.map(n=>n.dataset.run)),['online','cost_off','sigma_0.3','sigma_0.1']);
      assert.equal(await frame.locator('.seg').isVisible(),false);
      // Running plots draw every shown run, then one fewer when a run is hidden.
      const plots=frame.locator('[data-running-plots] .running-plot');
      assert.equal(await plots.count(),2);
      assert.equal(await plots.first().getAttribute('data-series'),'4');
      await toggles.filter({hasText:'Base cost off'}).locator('input').uncheck();
      assert.equal(await plots.first().getAttribute('data-series'),'3');
      assert.equal(await frame.locator('#chartSvg path').count(),3);
      await toggles.filter({hasText:'Base cost off'}).locator('input').check();
      assert.equal(await frame.locator('#chartSvg path').count(),4);
      // Scrubbing moves the running plots' cursor and the readouts.
      const readout=()=>toggles.first().locator('td').nth(1).innerText();
      await frame.getByRole('slider',{name:'Time',exact:true}).fill('100');
      const before=await readout();
      await frame.getByRole('slider',{name:'Time',exact:true}).fill('400');
      assert.notEqual(await readout(),before);
      await frame.locator('.result-table th',{hasText:'RMS base torque'}).waitFor();
      await frame.locator('.signal-plot[data-ready="true"]').first().waitFor();
      const box=await page.locator('iframe').boundingBox();
      const bottom=await frame.locator('.transport').evaluate(node=>node.getBoundingClientRect().bottom);
      assert(bottom<=box.height,`Transport fits: ${bottom}/${box.height}`);
      await page.setViewportSize({width:390,height:844});
      assert(await frame.locator('html').evaluate(async node=>{
        for(let attempt=0;attempt<40;attempt++){if(node.scrollWidth<=innerWidth+1)return true;await new Promise(r=>setTimeout(r,50));}
        return false;
      }),'Fits mobile');
      assert.deepEqual(errors,[]);assert.deepEqual(external,[],'No CDN dependencies');
      console.log(`PASS ${base}: run toggles, running plots, comparison chart and results, fit`);
    } finally {await context.close();server.httpServer.closeAllConnections();await new Promise(r=>server.httpServer.close(r));}
  }
} finally {await browser.close();}
