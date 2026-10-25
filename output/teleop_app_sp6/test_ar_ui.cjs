// Isolated browser test: all URLs are served from local files, no robot socket.
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');

(async () => {
  const browser = await chromium.launch({ headless: true,
    ...(process.env.CHROME_EXECUTABLE ? { executablePath: process.env.CHROME_EXECUTABLE } : {}) });
  try {
    const page = await browser.newPage({ viewport: { width: 390, height: 844 } });
    const errors = [];
    page.on('pageerror', e => errors.push(e.message));
    await page.route('**/*', async route => {
      const filename = new URL(route.request().url()).pathname.slice(1) || 'index.html';
      if (!['index.html', 'style.css', 'app.js', 'ar_control_modes.js'].includes(filename)) {
        return route.abort();
      }
      const type = filename.endsWith('.js') ? 'application/javascript' : filename.endsWith('.css') ? 'text/css' : 'text/html';
      await route.fulfill({ body: fs.readFileSync(path.join(__dirname, filename)), contentType: type });
    });
    await page.goto('http://ar-test.invalid/');
    await page.evaluate(() => {
      window.sent = [];
      ws = { readyState: 1, send: text => window.sent.push(JSON.parse(text)) };
      state = STATE.CONNECTED; welcomed = true; estopActive = false;
      xr.session = {}; xr.pose = { p: [1, 2, 3], q: [0, 0, 0, 1] }; xr.t = performance.now();
      document.getElementById('arOverlay').hidden = false;
      document.getElementById('arOverlay').classList.add('opaque');
      updateButtons();
    });
    const press = async () => {
      await page.evaluate(() => { xr.t = performance.now(); });
      await page.locator('#arClutch').dispatchEvent('pointerdown', { pointerId: 17 });
    };
    const sample = async () => page.evaluate(() => {
      // Avoid speed-guard intervention so this test isolates mode routing.
      xr.pose = { p: [1.02, 2.01, 3.03], q: [.15, 0, 0, Math.sqrt(1 - .15 ** 2)] };
      xr.t = performance.now();
      arHold.last = { p: xr.pose.p, q: xr.pose.q, t: xr.t };
      arTick();
      return window.sent.filter(m => m.type === 'pose_update').at(-1).payload;
    });
    await press();
    let payload = await sample();
    assert.deepEqual(payload.q_rel, [0, 0, 0, 1]);
    assert.deepEqual(payload.p_rel_m, [.02, .01, .03]);
    await page.locator('[data-ar-control="wrist"]').click();
    assert.equal(await page.evaluate(() => hold === null && arHold === null), true);
    assert.equal(await page.evaluate(() => window.sent.at(-1).type), 'pose_stop');
    await page.evaluate(() => { xr.pose = { p: [1, 2, 3], q: [0, 0, 0, 1] }; });
    await press();
    payload = await sample();
    assert.deepEqual(payload.p_rel_m, [0, 0, 0]);
    assert.notDeepEqual(payload.q_rel, [0, 0, 0, 1]);
    assert.equal(await page.locator('.ar-grip-btn').first().isDisabled(), true);
    await page.locator('[data-ar-control="translation"]').click();
    assert.equal(await page.evaluate(() => hold === null), true);
    await page.evaluate(() => { xr.pose = { p: [1, 2, 3], q: [0, 0, 0, 1] }; });
    await press();
    payload = await sample();
    assert.deepEqual(payload.q_rel, [0, 0, 0, 1]);
    const modes = await page.evaluate(() => window.sent.filter(m => m.type === 'pose_start').map(m => m.payload.control_mode));
    assert.deepEqual(modes, ['translation', 'wrist', 'translation']);
    await page.locator('#arClutch').dispatchEvent('pointerup', { pointerId: 17 });
    await page.screenshot({ path: path.join(__dirname, 'explicit_modes_preview.png') });
    assert.deepEqual(errors, []);
    console.log('AR UI PASS: translation -> stop -> wrist -> stop -> translation; fresh origins, filtered payloads, no JS errors');
  } finally { await browser.close(); }
})().catch(e => { console.error(e); process.exitCode = 1; });
