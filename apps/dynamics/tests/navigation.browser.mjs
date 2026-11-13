/** Check hub tab ordering, lazy loading, and direct project-prefix embedding. */
import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {preview} from 'vite';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=fileURLToPath(new URL('../../',import.meta.url));
const browser=await chromium.launch({executablePath:process.env.CHROME_EXECUTABLE||(existsSync('/usr/bin/google-chrome')?'/usr/bin/google-chrome':undefined),headless:true,args:['--no-sandbox','--enable-unsafe-swiftshader']});
try{
  for(const base of ['/','/GTDynamics/']){
    const server=await preview({root,configFile:false,base,preview:{host:'127.0.0.1',port:0,strictPort:true}});
    const context=await browser.newContext({viewport:{width:1500,height:1100}});
    try{
      const page=await context.newPage(),errors=[];page.setDefaultTimeout(60000);
      page.on('pageerror',error=>errors.push(error.message));
      await page.goto(`http://127.0.0.1:${server.httpServer.address().port}${base}`);
      assert.deepEqual((await page.getByRole('tab').allTextContents()).slice(-2),['SCATE','CAST']);
      assert.equal(await page.locator('iframe').count(),0);
      for(const app of ['Dynamics','Kinodynamics']) {
        await page.getByRole('tab',{name:app,exact:true}).click();
        await page.getByRole('tab',{name:app,exact:true,selected:true}).waitFor();
        await page.frameLocator('iframe').getByRole('heading',{name:app,exact:true}).waitFor();
        await page.frameLocator('iframe').locator('.dynamics-scene[data-ready="true"]').waitFor();
        assert.equal(await page.getByRole('tab',{name:app,exact:true}).getAttribute('aria-selected'),'true');
        await page.getByRole('tab',{name:'Overview',exact:true}).click();
        await page.locator('iframe').waitFor({state:'detached'});
        assert.equal(await page.locator('iframe').count(),0);
        await page.getByRole('tab',{name:app,exact:true}).click();
        await page.getByRole('tab',{name:app,exact:true,selected:true}).waitFor();
        await page.frameLocator('iframe').getByRole('heading',{name:app,exact:true}).waitFor();
        await page.frameLocator('iframe').locator('.dynamics-scene[data-ready="true"]').waitFor();
      }
      assert.deepEqual(errors,[]);
      console.log(`PASS ${base}: Dynamics and Kinodynamics lazy replay, unload/reopen`);
    }finally{await context.close();server.httpServer.closeAllConnections();await new Promise(resolve=>server.httpServer.close(resolve));}
  }
}finally{await browser.close();}
