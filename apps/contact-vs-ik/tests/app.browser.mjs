import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {preview} from 'vite';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=fileURLToPath(new URL('../',import.meta.url));
const browser=await chromium.launch({executablePath:process.env.CHROME_EXECUTABLE||(existsSync('/usr/bin/google-chrome')?'/usr/bin/google-chrome':undefined),headless:true,args:['--no-sandbox','--enable-unsafe-swiftshader']});
try{
 for(const base of ['/','/GTDynamics/contact-vs-ik/']){
  const server=await preview({root,configFile:false,base,preview:{host:'127.0.0.1',port:0,strictPort:true}});
  const context=await browser.newContext({viewport:{width:1460,height:1100}});
  try{
   const page=await context.newPage();page.setDefaultTimeout(60000);
   const errors=[],failures=[],requests=[];page.on('pageerror',e=>errors.push(e.message));page.on('request',r=>requests.push(r.url()));page.on('response',r=>{if(r.status()>=400&&!r.url().endsWith('favicon.ico'))failures.push(r.url());});
   const ready=()=>page.locator('.method-panel.ik .scene[data-ready="true"]').waitFor();
   await page.goto(`http://127.0.0.1:${server.httpServer.address().port}${base}`);await ready();
   assert.equal(await page.getByRole('heading',{name:'Contact vs IK',exact:true}).count(),1);assert.equal(await page.locator('canvas').count(),2);
   const snippets=page.getByRole('combobox',{name:'Snippet',exact:true});assert.equal(await snippets.locator('option').count(),3);assert.equal(await snippets.inputValue(),'turn');
   assert.equal(await page.getByRole('slider',{name:'Walking time',exact:true}).getAttribute('max'),'499');
   assert(!requests.some(url=>url.endsWith('/states.bin.gz')));
   const aiding=page.getByRole('checkbox',{name:'IMU aiding',exact:true}),reference=page.getByRole('checkbox',{name:'LiDAR-inertial reference',exact:true});
   assert(await aiding.isChecked());await reference.uncheck();assert.equal(await page.locator('.superodom-path').count(),0);await reference.check();
   for(const id of ['walk','corner','turn']){
    await snippets.selectOption(id);await ready();assert.equal(await page.locator('main').getAttribute('data-frame'),'0');
    await page.getByRole('slider',{name:'Walking time',exact:true}).fill('250');
    await page.getByRole('button',{name:'Initialization',exact:true}).click();await ready();
    assert.match(await page.locator('.method-panel.ik .iteration-controls').innerText(),/IK solve · frame 250/);
    assert.match(await page.locator('.method-panel.contact .iteration-controls').innerText(),/Sparse contact solve/);
    await page.getByRole('button',{name:'CM-Opt smoothing',exact:true}).click();
    assert.match(await page.locator('.method-panel.contact .bias-values').innerText(),/Gyroscope prior/);
   }
   await aiding.uncheck();await ready();assert.match(await page.locator('.method-panel.contact .bias-values').innerText(),/not estimated/);
   await aiding.check();await ready();
   const iteration=page.getByRole('slider',{name:'Contact initialization iteration',exact:true});await iteration.fill('0');assert.equal(await page.locator('.method-panel.contact').getAttribute('data-iteration'),'0');
   await page.locator('.method-panel.contact').getByRole('button',{name:'Last iteration',exact:true}).click();
   await page.getByRole('slider',{name:'Walking time',exact:true}).fill('0');
   await page.getByRole('button',{name:'Play walking',exact:true}).click();
   await page.waitForFunction(()=>Number(document.querySelector('main').dataset.frame)>0,null,{timeout:15000});
   // Pause atomically: a slow browser may already have reached the end while
   // Playwright waits to click, when the same button becomes Play again.
   const paused=await page.evaluate(()=>{
    const button=document.querySelector('.play-button');
    if(button.textContent.includes('Pause'))button.click();
    return Number(document.querySelector('main').dataset.frame);
   });assert(paused>0);
   await page.waitForTimeout(250);assert.equal(Number(await page.locator('main').getAttribute('data-frame')),paused);
   assert.equal(await page.getByRole('button',{name:'Play walking',exact:true}).count(),1);
   if(base==='/')await page.screenshot({path:'/tmp/gtd-contact-vs-ik.png',fullPage:true});
   await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));
   assert.deepEqual(errors,[]);assert.deepEqual(failures,[]);console.log(`Passed Contact vs IK ${base}`);
  }finally{await context.close();server.httpServer.closeAllConnections();await new Promise(resolve=>server.httpServer.close(resolve));}
 }
}finally{await browser.close();}
