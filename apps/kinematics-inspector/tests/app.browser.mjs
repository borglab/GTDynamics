/** Full-trajectory viewer: recorded parameter experiments and foot-slip overlays. */
import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {preview} from 'vite';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=fileURLToPath(new URL('../',import.meta.url));
const browser=await chromium.launch({executablePath:process.env.CHROME_EXECUTABLE||(existsSync('/usr/bin/google-chrome')?'/usr/bin/google-chrome':undefined),headless:true,args:['--no-sandbox','--enable-unsafe-swiftshader']});
try{
 for(const base of ['/','/GTDynamics/kinematics-inspector/']){
  const server=await preview({root,configFile:false,base,preview:{host:'127.0.0.1',port:0,strictPort:true}});
  const context=await browser.newContext({viewport:{width:1460,height:1100}});
  try{
   const page=await context.newPage();page.setDefaultTimeout(60000);
   const errors=[],failures=[];page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400&&!r.url().endsWith('favicon.ico'))failures.push(r.url());});
   const ready=()=>page.locator('.large-scene[data-ready="true"]').waitFor();
   await page.goto(`http://127.0.0.1:${server.httpServer.address().port}${base}`);await ready();
   assert.equal(await page.locator('canvas').count(),1);assert.equal(await page.getByRole('heading',{name:'Kinematics',exact:true}).count(),1);
   assert(!/\bIK\b/.test(await page.locator('main').innerText()));assert.equal(await page.getByRole('button',{name:'Initialization',exact:true}).count(),0);
   const imu=page.getByRole('checkbox',{name:'IMU aiding',exact:true}),closure=page.getByRole('checkbox',{name:'Loop closure applied',exact:true});
   assert(await imu.isChecked());assert(await closure.isChecked());
   assert.equal(await page.getByRole('combobox',{name:'Stationary contact sigma'}).inputValue(),'50');
   assert.equal(await page.locator('.factor-table tbody tr').count(),9);
   assert.match(await page.locator('.equality-summary').innerText(),/31,900/);
   const slip=page.getByRole('checkbox',{name:'Foot slip',exact:true}),scene=page.locator('.large-scene');
   const initialCount=Number(await scene.getAttribute('data-slip-count'));assert(initialCount>0);
   await slip.uncheck();assert.equal(await scene.getAttribute('data-slip-count'),'0');await slip.check();
   assert.equal(Number(await scene.getAttribute('data-slip-count')),initialCount);
   await page.getByRole('slider',{name:'Slip threshold',exact:true}).fill('50');
   assert(Number(await scene.getAttribute('data-slip-count'))<initialCount);
   await page.getByRole('slider',{name:'Slip threshold',exact:true}).fill('5');
   const loose=Number((await page.locator('.physical-summary strong').first().innerText()).split(' ')[0]);
   await page.getByRole('slider',{name:'Walking time',exact:true}).fill('600');
   await page.getByRole('combobox',{name:'Stationary contact sigma'}).selectOption('5');await ready();
   assert.equal(await page.locator('main').getAttribute('data-frame'),'600');
   const tight=Number((await page.locator('.physical-summary strong').first().innerText()).split(' ')[0]);assert(tight<loose);
   assert.match(await page.locator('[data-factor="stationary"]').innerText(),/0.005 m/);
   const finalCost=await page.locator('.factor-table tfoot').innerText();
   await page.getByRole('slider',{name:'Smoothing iteration',exact:true}).fill('0');assert.notEqual(await page.locator('.factor-table tfoot').innerText(),finalCost);
   await page.getByRole('button',{name:'Last iteration',exact:true}).click();
   await imu.uncheck();await ready();assert.equal(await page.locator('.factor-table tbody tr').count(),6);assert.match(await page.locator('.bias-values').innerText(),/not estimated/);
   await imu.check();await ready();assert.equal(await page.locator('.factor-table tbody tr').count(),9);
   await closure.uncheck();await ready();assert.equal(await page.locator('[data-factor="closure"]').count(),0);
   await closure.check();await ready();
   await page.getByRole('combobox',{name:'Stationary contact sigma'}).selectOption('20');await ready();
   assert.match(await page.locator('[data-factor="stationary"]').innerText(),/0.02 m/);
   await page.getByRole('button',{name:'Follow robot',exact:true}).click();assert.equal(await page.getByRole('button',{name:'Follow robot',exact:true}).getAttribute('aria-pressed'),'true');
   await page.getByRole('button',{name:'Play walking',exact:true}).click();await page.waitForTimeout(800);await page.getByRole('button',{name:'Pause walking',exact:true}).click();assert(Number(await page.locator('main').getAttribute('data-frame'))>600);
   await page.getByRole('button',{name:'Fit trajectory',exact:true}).click();
   if(base==='/')await page.screenshot({path:'/tmp/gtd-kinematics-split.png',fullPage:true});
   await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));
   if(base==='/')await page.screenshot({path:'/tmp/gtd-kinematics-split-mobile.png',fullPage:true});
   assert.deepEqual(errors,[]);assert.deepEqual(failures,[]);console.log(`Passed Kinematics ${base}`);
  }finally{await context.close();server.httpServer.closeAllConnections();await new Promise(resolve=>server.httpServer.close(resolve));}
 }
 const hub=await preview({root:fileURLToPath(new URL('../../',import.meta.url)),preview:{host:'127.0.0.1',port:0,strictPort:true}});
 const page=await browser.newPage({viewport:{width:1460,height:1100}});
 try{
  await page.goto(`http://127.0.0.1:${hub.httpServer.address().port}/`);
  const tabs=await page.getByRole('tab').allTextContents();assert.equal(tabs.at(-1).trim(),'Kinematics');assert.equal(tabs.at(-2).trim(),'Contact vs IK');
  await page.getByRole('tab',{name:'Contact vs IK',exact:true}).click();
  await page.frameLocator('iframe').getByRole('heading',{name:'Contact vs IK',exact:true}).waitFor();
  await page.frameLocator('iframe').locator('.method-panel.ik .scene[data-ready="true"]').waitFor({timeout:60000});
  await page.getByRole('tab',{name:'Kinematics',exact:true}).click();
  await page.frameLocator('iframe').locator('.large-scene[data-ready="true"]').waitFor({timeout:60000});
  await page.getByRole('tab',{name:'Overview',exact:true}).click();await page.locator('iframe').waitFor({state:'detached'});assert.equal(await page.locator('iframe').count(),0);
  console.log('Passed shared hub order, both apps, and unload');
 }finally{await page.close();hub.httpServer.closeAllConnections();await new Promise(resolve=>hub.httpServer.close(resolve));}
}finally{await browser.close();}
