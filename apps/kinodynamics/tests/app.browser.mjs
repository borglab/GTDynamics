/** Validate offline replay, controls, plots and subpath deployment. */
import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {preview} from 'vite';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=fileURLToPath(new URL('../',import.meta.url));
const browser=await chromium.launch({executablePath:process.env.CHROME_EXECUTABLE||(existsSync('/usr/bin/google-chrome')?'/usr/bin/google-chrome':undefined),headless:true,args:['--no-sandbox','--enable-unsafe-swiftshader']});
try{
  for(const base of ['/','/GTDynamics/kinodynamics/']){
    const server=await preview({root,configFile:false,base,preview:{host:'127.0.0.1',port:0,strictPort:true}});
    const context=await browser.newContext({viewport:{width:1460,height:1100}});
    try{
      const page=await context.newPage();page.setDefaultTimeout(60000);
      const errors=[],failures=[],requests=[];
      page.on('pageerror',e=>errors.push(e.message));
      page.on('request',r=>requests.push(r.url()));
      page.on('response',r=>{if(r.status()>=400&&!r.url().endsWith('favicon.ico'))failures.push(r.url());});
      await page.goto(`http://127.0.0.1:${server.httpServer.address().port}${base}`);
      await page.locator('.kinodynamics-scene[data-ready="true"]').waitFor();
      assert.equal(await page.getByRole('heading',{name:'Kinodynamics',exact:true}).count(),1);
      assert.match(await page.locator('.timing-grid').innerText(),/1,000/);
      assert.equal(await page.locator('canvas').count(),1);
      const slider=page.getByRole('slider',{name:'Kinodynamics time'});
      await slider.fill('500');assert.equal(await page.locator('main').getAttribute('data-frame'),'500');
      await page.getByRole('button',{name:'Play',exact:true}).click();await page.waitForTimeout(250);await page.getByRole('button',{name:'Pause',exact:true}).click();
      assert(Number(await page.locator('main').getAttribute('data-frame'))>500);
      const estimate=page.getByRole('combobox',{name:'Estimate',exact:true});
      await estimate.selectOption('initial');
      assert.equal(await page.locator('.kinodynamics-scene').getAttribute('data-force-arrows'),'0');
      await estimate.selectOption('final');
      assert(Number(await page.locator('.kinodynamics-scene').getAttribute('data-force-arrows'))>0);
      await page.getByRole('checkbox',{name:'All joint torques',exact:true}).check();
      await page.locator('.torque-section .signal-plot[data-ready="true"]').first().waitFor();
      assert.equal(await page.locator('.torque-section .signal-plot').count(),5);
      await page.getByRole('checkbox',{name:'All joint velocities',exact:true}).check();
      await page.getByRole('button',{name:'Fit trajectory',exact:true}).click();
      await page.getByRole('button',{name:'Follow robot',exact:true}).click();
      await page.setViewportSize({width:390,height:844});
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth+1));
      assert(!requests.some(url=>/mujoco.*\.(wasm|js)(?:\?|$)/.test(url)));
      assert.deepEqual(errors,[]);assert.deepEqual(failures,[]);
    }finally{await context.close();await new Promise(resolve=>server.httpServer.close(resolve));}
  }
}finally{await browser.close();}
console.log('Kinodynamics browser checks passed at root and /GTDynamics/');
