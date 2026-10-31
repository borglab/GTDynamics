/** Test the built viewer at both root and project-site prefixes. */
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { preview } from 'vite';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = fileURLToPath(new URL('../', import.meta.url));
const browser = await chromium.launch({executablePath: process.env.CHROME_EXECUTABLE || (existsSync('/usr/bin/google-chrome') ? '/usr/bin/google-chrome' : undefined), headless:true, args:['--no-sandbox','--enable-unsafe-swiftshader']});
try {
  for(const base of ['/', '/GTDynamics/kinematics-inspector/']) {
    const server = await preview({root,configFile:false,base,preview:{host:'127.0.0.1',port:0,strictPort:true}});
    const context = await browser.newContext({viewport:{width:1460,height:1100}});
    try {
      const page = await context.newPage(); page.setDefaultTimeout(60000);
      const errors=[], failures=[], sockets=[];
      page.on('pageerror', e=>errors.push(e.message));
      page.on('response', r=>{if(r.status()>=400&&!r.url().endsWith('favicon.ico'))failures.push(r.url());});
      page.on('websocket', s=>sockets.push(s.url()));
      await page.goto(`http://127.0.0.1:${server.httpServer.address().port}${base}`);
      await page.locator('.method-panel.ik .scene[data-ready="true"]').waitFor();
      assert.equal(await page.locator('canvas').count(),2);
      const contact=page.locator('.method-panel.contact'), ik=page.locator('.method-panel.ik');
      const contactIteration=contact.getByRole('slider',{name:'Contact initialization iteration'});
      assert.equal(await contactIteration.getAttribute('max'),'10');
      assert.equal(await ik.getByRole('slider',{name:'Inverse kinematics iteration'}).getAttribute('max'),'10');
      assert.match(await contact.locator('.iteration-controls').innerText(),/Iteration 10 \/ 10/);
      assert(!/\bstates\b/.test(await page.locator('main').innerText()));
      const credit = page.locator('.motion-attribution');
      assert.match(await credit.innerText(), /10871_humanoid_20260520_223451.*Open-IO.*Yuheng Qiu/s);
      assert.equal(await credit.getByRole('link', {name:'CC BY 4.0', exact:true}).getAttribute('href'), 'https://creativecommons.org/licenses/by/4.0/');
      const notice = await page.request.get(new URL('./OPENIO_ATTRIBUTION.txt', page.url()).href);
      assert.equal(notice.status(), 200);
      assert.match(await notice.text(), /10871_humanoid_20260520_223451/);
      const imu = await page.request.get(new URL('./imu.json', page.url()).href);
      assert.equal(imu.status(), 200);
      assert.equal((await imu.json()).timeSeconds.length, 2000);
      assert.match(await page.locator('footer').innerText(), /does not use IMU factors/);
      await contactIteration.fill('0');
      assert.equal(await contact.getAttribute('data-iteration'),'0');
      await contact.getByRole('button',{name:'Next iteration',exact:true}).click();
      assert.equal(await contact.getAttribute('data-iteration'),'1');
      await contact.getByRole('button',{name:'Last iteration',exact:true}).click();
      assert(Number(await contact.getAttribute('data-iteration'))>1);
      await page.getByRole('slider',{name:'Walking time'}).fill('400');
      await page.getByRole('checkbox',{name:'SuperOdom reference'}).check();
      await page.getByRole('combobox',{name:'Joint to inspect'}).selectOption({label:'right_knee_joint'});
      assert.match(await ik.locator('.plot-block').nth(1).innerText(),/right knee joint/);
      if(base==='/') await page.screenshot({path:'/tmp/gtd-kinematics-inspector.png',fullPage:true});
      await page.getByRole('button',{name:'Initialization',exact:true}).click();
      assert.equal(await page.locator('main').getAttribute('data-stage'),'initializer');
      assert.match(await ik.locator('.iteration-controls').innerText(),/IK solve · frame 400/);
      await ik.getByRole('slider',{name:'Inverse kinematics iteration'}).fill('0');
      assert.equal(await ik.getAttribute('data-iteration'),'0');
      const walkingTime = page.getByRole('slider',{name:'Walking time'});
      await walkingTime.fill(await walkingTime.getAttribute('max'));
      await ik.getByRole('button',{name:'Last iteration',exact:true}).click();
      await contact.getByRole('slider',{name:'Contact initialization iteration'}).fill('1');
      await contact.getByRole('button',{name:'Replay iterations',exact:true}).click();
      await page.waitForTimeout(850);
      await contact.getByRole('button',{name:'Pause iterations',exact:true}).click();
      assert(Number(await contact.getAttribute('data-iteration'))>0);
      await page.getByRole('button',{name:'Jump to phase 2',exact:true}).click();
      const before=Number(await page.locator('main').getAttribute('data-frame'));
      await page.getByRole('button',{name:'Play walking',exact:true}).click();
      await page.waitForTimeout(500);
      await page.getByRole('button',{name:'Pause walking',exact:true}).click();
      assert(Number(await page.locator('main').getAttribute('data-frame'))>before);
      // Switch with Initialization selected: old scenes/iterates must be disposed.
      const recording = page.getByRole('combobox', {name:'Recording',exact:true});
      await recording.selectOption('full');
      await page.locator('.method-panel.ik .scene[data-ready="true"]').waitFor();
      assert.equal(await page.locator('main').getAttribute('data-frame'), '0');
      assert.equal(await page.getByRole('slider',{name:'Walking time'}).getAttribute('max'), '1099');
      assert.match(await page.locator('footer').innerText(), /43,991/);
      assert.equal(await page.getByRole('link',{name:'Download IMU measurements'}).getAttribute('href'), './full/imu.json');
      await page.getByRole('slider',{name:'Walking time'}).fill('1045');
      await page.getByRole('button',{name:'CM-Opt smoothing',exact:true}).click();
      assert.match(await contact.locator('.iteration-controls').innerText(), /Iteration 10 \/ 10/);
      if(base==='/') await page.screenshot({path:'/tmp/gtd-kinematics-full.png',fullPage:true});
      await page.setViewportSize({width:390,height:844});
      await page.getByRole('button',{name:'CM-Opt smoothing',exact:true}).click();
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth+1));
      if(base==='/') await page.screenshot({path:'/tmp/gtd-kinematics-inspector-mobile.png',fullPage:true});
      await recording.selectOption('turn');
      await page.locator('.method-panel.ik .scene[data-ready="true"]').waitFor();
      assert.equal(await page.getByRole('slider',{name:'Walking time'}).getAttribute('max'), '499');
      assert.equal(await page.locator('main').getAttribute('data-frame'), '0');
      assert.deepEqual(errors,[]); assert.deepEqual(failures,[]); assert.deepEqual(sockets,[]);
      console.log(`Passed: ${base}`);
    } finally {await context.close(); await server.httpServer.close();}
  }
  // Use the real hub config: preview must not inherit development proxies.
  const hub = await preview({root:fileURLToPath(new URL('../../',import.meta.url)),
    preview:{host:'127.0.0.1',port:0,strictPort:true}});
  const page = await browser.newPage({viewport:{width:1460,height:1100}});
  try {
    await page.goto(`http://127.0.0.1:${hub.httpServer.address().port}/#kinematics-inspector`);
    await page.getByRole('tab',{name:'Kinematics',selected:true}).waitFor();
    await page.frameLocator('iframe').locator('.method-panel.ik .scene[data-ready="true"]').waitFor({timeout:60000});
    console.log('Passed: shared hub with production preview config');
  } finally {await page.close(); await hub.httpServer.close();}
} finally {await browser.close();}
