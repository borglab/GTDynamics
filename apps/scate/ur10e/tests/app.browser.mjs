/** SCATE replay, explanatory graph, file loading and theme behavior in the hub. */
import assert from 'node:assert/strict';
import {readFileSync, existsSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {preview} from 'vite';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=fileURLToPath(new URL('../../../',import.meta.url));
const replay=readFileSync(new URL('../public/replay.json',import.meta.url));
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
      await page.goto(`http://127.0.0.1:${server.httpServer.address().port}${base}#scate/ur10e`);
      const frame=page.frameLocator('iframe');
      await frame.locator('#stage[data-ready="true"]').waitFor();
      assert.equal(await frame.locator('#drop').isVisible(),false);
      assert.equal(await frame.locator('.app-abstract').count(),1);
      assert.deepEqual(await frame.locator('[data-paper-section]').evaluateAll(nodes=>nodes.map(n=>n.dataset.paperSection)),['motivation','approach','results','discussion']);
      assert.equal(await frame.locator('.factor-definitions dt').count(),9);
      assert(await frame.locator('mjx-container').count()>=3);
      const before=parseFloat(await frame.locator('#timeTxt').innerText());
      await frame.getByRole('button',{name:'Play',exact:true}).click();
      await page.waitForTimeout(400);
      await frame.getByRole('button',{name:'Pause',exact:true}).click();
      assert(parseFloat(await frame.locator('#timeTxt').innerText())>before);
      await frame.getByRole('slider',{name:'Time',exact:true}).fill('50');
      assert.equal(await frame.locator('main').getAttribute('data-frame'),'1');
      await frame.getByRole('button',{name:/Show slice 2 in demo/}).first().click();
      assert.equal(await frame.locator('main').getAttribute('data-frame'),'2');
      await frame.locator('#lyMesh').uncheck(); await frame.locator('#lyMesh').check();
      await frame.locator('.seg label').filter({hasText:/^Online$/}).click();
      assert(await frame.locator('#runsOnline').isChecked());
      await frame.locator('.seg label').filter({hasText:/^Both$/}).click();
      await frame.locator('.signal-plot[data-ready="true"]').first().waitFor();
      await frame.getByRole('combobox',{name:'Joint for result plots'}).selectOption('2');
      await frame.locator('#file').setInputFiles({name:'replay.json',mimeType:'application/json',buffer:replay});
      await frame.getByRole('combobox',{name:'Joint for result plots'}).evaluate(async select=>{
        for(let i=0;i<40&&select.value!=='0';i++)await new Promise(r=>setTimeout(r,50));
      });
      assert.equal(await frame.getByRole('combobox',{name:'Joint for result plots'}).inputValue(),'0');
      for(const size of [{width:1440,height:900},{width:1280,height:800}]) {
        await page.setViewportSize(size);
        await frame.locator('main').evaluate(()=>window.scrollTo(0,0));
        const box=await page.locator('iframe').boundingBox();
        const bottom=await frame.locator('.transport').evaluate(node=>node.getBoundingClientRect().bottom);
        assert(bottom<=box.height,`Transport fits ${size.width}×${size.height}: ${bottom}/${box.height}`);
      }
      await page.setViewportSize({width:390,height:844});
      assert(await frame.locator('html').evaluate(async node => {
        // Plotly responds to container resize asynchronously.
        for (let attempt = 0; attempt < 40; attempt++) {
          if (node.scrollWidth <= innerWidth + 1) return true;
          await new Promise(resolve => setTimeout(resolve, 50));
        }
        return false;
      }), 'SCATE fits mobile');
      await page.getByRole('tab',{name:'Overview',exact:true}).click();
      await page.locator('iframe').waitFor({state: 'detached'});
      await page.getByRole('tab',{name:'SCATE UR10e',exact:true}).click();
      await page.frameLocator('iframe').locator('#stage[data-ready="true"]').waitFor();
      assert.deepEqual(errors,[]);assert.deepEqual(external,[],'No CDN dependencies');
      console.log(`PASS ${base}: SCATE paper, playback, graph, overlays, file reload, fit, unload/reopen`);
    } finally {await context.close();server.httpServer.closeAllConnections();await new Promise(r=>server.httpServer.close(r));}
  }
} finally {await browser.close();}
