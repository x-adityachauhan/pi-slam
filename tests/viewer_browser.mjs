// Optional browser regression. Install Playwright separately; the viewer itself
// has no npm dependency. Set PLAYWRIGHT_MODULE and PLAYWRIGHT_CHROMIUM if needed.
import assert from 'node:assert/strict';
import path from 'node:path';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const output = process.env.VIEWER_TEST_OUTPUT || '/tmp';
const browser = await chromium.launch({headless:true,
  executablePath:process.env.PLAYWRIGHT_CHROMIUM,
  args:['--no-sandbox','--enable-unsafe-swiftshader']});
try {
  const page = await browser.newPage({viewport:{width:1600,height:1000}});
  const errors=[]; let requests=0;
  page.on('pageerror', e=>errors.push(e.message));
  page.on('request', r=>{if(new URL(r.url()).pathname==='/api')requests++;});
  await page.goto(process.env.VIEWER_URL || 'http://127.0.0.1:8090');
  await page.waitForFunction(()=>document.querySelector('#stats').textContent.includes('surface points'));
  await page.waitForFunction(()=>document.querySelector('#camera').complete && document.querySelector('#camera').naturalWidth>0);
  assert.equal(await page.locator('#error').isVisible(),false);
  const stats=await page.evaluate(()=>{
    draw();
    const pixels=new Uint8Array(canvas.width*canvas.height*4);
    gl.readPixels(0,0,canvas.width,canvas.height,gl.RGBA,gl.UNSIGNED_BYTE,pixels);
    let visible=0;
    for(let i=0;i<pixels.length;i+=4)if(pixels[i]>45||pixels[i+1]>45||pixels[i+2]>45)visible++;
    return {points:state.points.length,faces:state.faces.length,visible,glError:gl.getError()};
  });
  assert(stats.points>100 && stats.faces>100 && stats.visible>100);
  assert.equal(stats.glError,0);
  await page.screenshot({path:path.join(output,'viewer-v2-camera.png'),fullPage:true});
  const before=requests;
  await page.waitForTimeout(1200);
  assert.equal(requests,before,'Saved maps should not poll their large JSON payload');
  await page.locator('#reset').click();
  const box=await page.locator('#map').boundingBox();
  await page.mouse.move(box.x+box.width/2,box.y+box.height/2);
  await page.mouse.down();await page.mouse.move(box.x+box.width/2+80,box.y+box.height/2+30,{steps:5});await page.mouse.up();
  await page.screenshot({path:path.join(output,'viewer-v2-orbit.png'),fullPage:true});
  await page.locator('#surfaces').uncheck();
  await page.locator('#surfaces').check();
  await page.locator('#camera-view').click();
  const choices=await page.locator('#maps option').evaluateAll(options=>options.map(o=>o.value));
  if(choices.length>1){
    const current=await page.locator('#maps').inputValue();
    const other=choices.find(x=>x!==current);
    await page.locator('#maps').selectOption(other);
    await page.waitForFunction(id=>document.querySelector('#status').textContent.includes(id),other);
    await page.waitForFunction(()=>document.querySelector('#camera').complete && document.querySelector('#camera').naturalWidth>0);
    await page.screenshot({path:path.join(output,'viewer-v2-other-map.png'),fullPage:true});
    const downloading=page.waitForEvent('download');
    await page.locator('#download').click();
    assert.equal((await downloading).suggestedFilename(),other+'.ply');
  }
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({passed:true,...stats,mapOptions:choices,apiRequests:requests}));
} finally { await browser.close(); }
