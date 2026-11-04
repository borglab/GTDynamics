/** Static replay smoke tests at root and GitHub Pages deployment paths. */
import assert from 'node:assert/strict';
import {existsSync,readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {fileURLToPath} from 'node:url';
import {preview} from 'vite';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=fileURLToPath(new URL('../',import.meta.url));
const record=JSON.parse(gunzipSync(readFileSync(new URL('../public/recording.json.gz',import.meta.url))));
const browser=await chromium.launch({executablePath:process.env.CHROME_EXECUTABLE||(existsSync('/usr/bin/google-chrome')?'/usr/bin/google-chrome':undefined),headless:true,args:['--no-sandbox','--enable-unsafe-swiftshader']});
try{
  for(const base of ['/','/GTDynamics/dynamics/']){
    const server=await preview({root,configFile:false,base,preview:{host:'127.0.0.1',port:0,strictPort:true}});
    const context=await browser.newContext({viewport:{width:1500,height:1100}});
    try{
      const page=await context.newPage(),errors=[],failures=[],requests=[];
      page.setDefaultTimeout(60000);
      page.on('pageerror',error=>errors.push(error.message));
      page.on('request',request=>requests.push(request.url()));
      page.on('response',response=>{if(response.status()>=400&&!response.url().endsWith('favicon.ico'))failures.push(response.url());});
      if(base==='/')for(const name of ['recording.json.gz','model.json.gz']){
        // Exercise raw gzip as served by Pages; the second path uses Vite's
        // Content-Encoding:gzip and therefore fetch's automatic decompression.
        await page.route(`**/${name}`,route=>route.fulfill({status:200,contentType:'application/gzip',body:readFileSync(new URL(`../public/${name}`,import.meta.url))}));
      }
      await page.goto(`http://127.0.0.1:${server.httpServer.address().port}${base}`);
      await page.locator('.dynamics-scene[data-ready="true"]').waitFor();
      assert.equal(await page.getByRole('heading',{name:'Dynamics',exact:true}).count(),1);
      assert.equal(await page.locator('.solve-status').innerText(),record.diagnostics.status);
      assert.match(await page.locator('.sampling-note').innerText(),new RegExp(`${record.times.length} original double-support knots`));
      assert.match(await page.locator('.sampling-note').innerText(),/full-trajectory Gaussian-process fit/);
      assert.match(await page.locator('.sampling-note').innerText(),new RegExp(`${record.settings.accelerationDifferenceStepSeconds*1000} ms step`));
      assert.equal(await page.locator('canvas').count(),1);
      await page.waitForFunction(()=>document.querySelectorAll('.force-plots .signal-plot[data-ready="true"]').length===4);
      assert.equal(await page.locator('.torque-section').count(),0);
      assert.equal(requests.filter(url=>url.endsWith('/recording.json.gz')).length,1);
      assert.equal(requests.filter(url=>url.endsWith('/model.json.gz')).length,1);
      assert(!requests.some(url=>/\.wasm|mujoco|variants\//.test(url)),'Browser loads saved transforms, with no dynamics backend or other trajectories');
      const slider=page.getByRole('slider',{name:'Dynamics time'});
      await slider.fill('10');
      assert.equal(await page.locator('main').getAttribute('data-frame'),'10');
      assert.equal(await page.locator('.dynamics-scene').getAttribute('data-frame'),'10');
      const forces=page.getByRole('checkbox',{name:'Ground reaction forces',exact:true});
      await forces.uncheck();assert.equal(await page.locator('.dynamics-scene').getAttribute('data-force-arrows'),'0');
      await forces.check();assert(Number(await page.locator('.dynamics-scene').getAttribute('data-force-arrows'))>0);
      await page.getByRole('combobox',{name:'Force scale'}).selectOption('0.004');
      await page.getByRole('button',{name:'Fit trajectory',exact:true}).click();
      await page.getByRole('button',{name:'Follow robot',exact:true}).click();
      await page.getByRole('button',{name:'Play dynamics',exact:true}).click();await page.waitForTimeout(500);
      await page.getByRole('button',{name:'Pause dynamics',exact:true}).click();assert(Number(await slider.inputValue())>10);
      await page.getByRole('checkbox',{name:'All joint torques',exact:true}).check();
      await page.waitForFunction(()=>document.querySelectorAll('.torque-section .signal-plot[data-ready="true"]').length===5);
      assert.equal(await page.locator('.torque-section .signal-plot').evaluateAll(plots=>plots.reduce((count,plot)=>count+plot.data.length,0)),29);
      if(base==='/')await page.locator('.torque-section').screenshot({path:'/tmp/gtd-dynamics-torques.png'});
      await page.getByRole('checkbox',{name:'All joint torques',exact:true}).uncheck();
      assert.equal(await page.locator('.torque-section').count(),0);
      assert(await page.locator('.factor-table tbody tr').count()>0);
      if(base==='/'){
        await page.screenshot({path:'/tmp/gtd-dynamics.png',fullPage:true});
        await page.locator('.dynamics-viewer').screenshot({path:'/tmp/gtd-dynamics-viewer.png'});
      }
      await page.setViewportSize({width:390,height:844});
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'App fits portrait width');
      assert.deepEqual(errors,[]);assert.deepEqual(failures,[]);
      console.log(`PASS ${base}: forces, torques, plots, irregular-time playback, static assets, portrait`);
    }finally{await context.close();server.httpServer.closeAllConnections();await new Promise(resolve=>server.httpServer.close(resolve));}
  }
}finally{await browser.close();}
