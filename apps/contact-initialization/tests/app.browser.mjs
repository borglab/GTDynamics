/** Run after apps/npm run build. Uses an externally supplied Playwright. */
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { mkdir, readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { preview } from 'vite';

const {chromium}=await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root=fileURLToPath(new URL('../../',import.meta.url));
const browser=await chromium.launch({
  executablePath:process.env.CHROME_EXECUTABLE || (existsSync('/usr/bin/google-chrome')?'/usr/bin/google-chrome':undefined),
  headless:true,args:['--no-sandbox','--enable-unsafe-swiftshader'],
});
try {
  for(const base of ['/', '/GTDynamics/']) {
    const server=await preview({root,configFile:false,base,preview:{host:'127.0.0.1',port:0,strictPort:true}});
    const context=await browser.newContext({viewport:{width:1500,height:1100},acceptDownloads:true});
    try {
      const page=await context.newPage();
      page.setDefaultTimeout(30_000);
      const errors=[],failed=[],sockets=[];
      page.on('pageerror',e=>errors.push(e.message));
      page.on('response',r=>{if(r.status()>=400&&!r.url().endsWith('/favicon.ico'))failed.push(`${r.status()} ${r.url()}`);});
      page.on('websocket',s=>sockets.push(s.url()));
      const url=`http://127.0.0.1:${server.httpServer.address().port}${base}`;
      await page.goto(`${url}#contact-initialization`);
      await page.getByRole('tab',{name:'Contact Initialization',selected:true}).waitFor();
      const app=page.frameLocator('iframe');
      const lab=app.locator('.lab');
      await app.locator('canvas').waitFor();
      assert.equal(await lab.getAttribute('data-model'),'ground');
      assert.equal(await lab.getAttribute('data-sequence'),'g1-policy-8s');
      assert.equal(await app.locator('.error-banner').count(),0);
      assert.match(await app.locator('.native-timings').innerText(),/GTSAM solves it/);
      assert.match(await app.locator('.native-badge').innerText(),/Release · 11 runs · 2 warmups/);
      assert.match(await app.locator('.graph-stats').innerText(),/75[\s\S]*38[\s\S]*672/);
      const startingEvent=Number(await lab.getAttribute('data-event'));
      await app.getByRole('button',{name:'Next event',exact:true}).click();
      assert.equal(Number(await lab.getAttribute('data-event')),startingEvent+1);
      await app.getByRole('button',{name:'Previous event',exact:true}).click();
      assert.equal(Number(await lab.getAttribute('data-event')),startingEvent);
      await app.getByRole('slider',{name:'Contact event',exact:true}).fill('50');
      assert.equal(await lab.getAttribute('data-event'),'50');
      assert.match(await app.locator('.scene-readout').innerText(),/5.60/);
      await app.getByRole('button',{name:'Play event playback'}).click();
      await page.waitForTimeout(600);
      await app.getByRole('button',{name:'Pause event playback'}).click();
      assert(Number(await lab.getAttribute('data-event'))>50);
      await app.getByRole('button',{name:'Freeze the robot'}).click();
      assert.match(await app.locator('.scene-card .panel-top').innerText(),/RIGID SNAPSHOT/);
      await app.getByRole('button',{name:'Find the events'}).click();
      assert.match(await app.locator('.scene-card .panel-top').innerText(),/MEASURED WALK/);
      await app.getByRole('button',{name:'Connect the contacts'}).click();
      for(const [mode,scalars] of [['patch',786],['point',558],['ground',672]]) {
        await app.getByRole('combobox',{name:'Contact geometry'}).selectOption(mode);
        await app.getByRole('button',{name:'Replay GTSAM solve'}).waitFor({state:'visible'});
        assert.equal(await lab.getAttribute('data-model'),mode);
        assert.match(await app.locator('.graph-stats').innerText(),new RegExp(String(scalars)));
        const metrics=await app.locator('.evaluation-card .solve-metrics strong').allTextContents();
        assert(metrics.every(m=>Number.isFinite(parseFloat(m))));
        if(mode==='point')assert.match(await app.locator('.insight-card h2').innerText(),/not a full pose/);
      }
      await app.getByRole('button',{name:'Replay GTSAM solve'}).click();
      await page.waitForTimeout(650);
      await app.getByRole('button',{name:'Pause native replay'}).click();
      const iteration=app.getByRole('slider',{name:'Solver iteration'});
      assert(Number(await iteration.inputValue())>0);
      await iteration.fill('0');
      const initialRms=parseFloat(await app.locator('.solve-card .solve-metrics strong').first().innerText());
      await iteration.fill(await iteration.getAttribute('max'));
      const finalRms=parseFloat(await app.locator('.solve-card .solve-metrics strong').first().innerText());
      assert(finalRms<initialRms,'Recorded native optimization decreases contact error');
      const downloadPromise=page.waitForEvent('download');
      await app.getByRole('button',{name:'Export initialization JSON'}).click();
      const download=await downloadPromise;
      const exported=JSON.parse(await readFile(await download.path(),'utf8'));
      assert.equal(exported.eventPoses.length,75);
      assert.equal(exported.contacts.length,38);
      assert.equal(exported.model,'ground');
      assert.equal(exported.solver,'C++ GTSAM Levenberg–Marquardt');
      assert.equal(exported.benchmark.repetitions,11);
      assert.equal(exported.assumptions.groundPlane.enabled,true);
      assert(!('reference' in exported));
      await app.getByRole('button',{name:'Top',exact:true}).click();
      await app.getByRole('button',{name:'Reset camera'}).click();
      await app.getByRole('checkbox',{name:'Reference',exact:true}).uncheck();
      await app.getByRole('checkbox',{name:'Snapshots',exact:true}).uncheck();
      await app.getByRole('checkbox',{name:'Reference',exact:true}).check();
      await app.getByRole('checkbox',{name:'Snapshots',exact:true}).check();
      await app.getByRole('combobox',{name:'Motion sequence'}).selectOption('g1-steptouch-30s');
      await app.locator('.recording-loading').waitFor({state:'hidden'});
      assert.equal(await lab.getAttribute('data-sequence'),'g1-steptouch-30s');
      assert.match(await app.locator('.graph-stats').innerText(),/35[\s\S]*19/);
      assert.match(await app.locator('.component-notice').innerText(),/3 disconnected support components/);
      assert.match(await app.locator('.evaluation-card').innerText(),/component-relative/);
      assert.match(await app.locator('.timeline-caption').innerText(),/height\/speed heuristic/);
      for(const mode of ['point','patch','ground']){
        await app.getByRole('combobox',{name:'Contact geometry'}).selectOption(mode);
        await page.waitForTimeout(50);
        assert.equal(await lab.getAttribute('data-model'),mode);
        assert.equal(await app.locator('.error-banner').count(),0);
      }
      await app.getByRole('combobox',{name:'Motion sequence'}).selectOption('g1-policy-8s');
      await app.locator('.recording-loading').waitFor({state:'hidden'});
      assert.match(await app.locator('.graph-stats').innerText(),/75[\s\S]*38[\s\S]*672/);
      await app.getByRole('slider',{name:'Contact event',exact:true}).fill('24');
      await app.locator('body').evaluate(()=>window.scrollTo(0,0));
      await page.waitForTimeout(350);
      if(process.env.SCREENSHOT_DIR&&base==='/') {
        await mkdir(process.env.SCREENSHOT_DIR,{recursive:true});
        await page.screenshot({path:`${process.env.SCREENSHOT_DIR}/contact-initialization.png`});
      }
      await page.setViewportSize({width:390,height:844});
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'Shared site fits mobile');
      assert(await app.locator('body').evaluate(el=>el.scrollWidth<=innerWidth),'Contact app fits mobile');
      await app.getByRole('button',{name:'Next event',exact:true}).click();
      if(process.env.SCREENSHOT_DIR&&base==='/')await page.screenshot({path:`${process.env.SCREENSHOT_DIR}/contact-mobile.png`});
      const direct=await context.newPage();
      await direct.goto(`${url}contact-initialization/`);
      await direct.locator('canvas').waitFor();
      assert.equal(await direct.locator('.error-banner').count(),0);
      await direct.close();
      await page.getByRole('tab',{name:'Overview'}).click();
      await page.locator('iframe').waitFor({state:'detached'});
      assert.equal(await page.locator('iframe').count(),0);
      assert.deepEqual(errors,[],'No browser errors');
      assert.deepEqual(failed,[],'Assets work under the project-site prefix');
      assert.deepEqual(sockets,[],'No server backend required');
      console.log(`PASS ${base}: two sequences, native timings, all three models, event playback, stages, GTSAM replay, JSON export, camera, mobile, direct URL`);
    } finally {
      await context.close();server.httpServer.closeAllConnections();await new Promise(resolve=>server.httpServer.close(resolve));
    }
  }
} finally {await browser.close();}
