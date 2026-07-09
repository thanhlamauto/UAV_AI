#!/usr/bin/env node
/* Verify the realistic Level-3 demo renders nonblank WebGL frames. */

const fs = require('fs');
const path = require('path');
const { execFileSync } = require('child_process');

function loadChromium() {
  try {
    return require('playwright-core').chromium;
  } catch (coreError) {
    try {
      return require('playwright').chromium;
    } catch (_) {
      throw coreError;
    }
  }
}

function findChromePath() {
  if (process.env.CHROME_PATH) return process.env.CHROME_PATH;
  const candidates = [
    '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
    '/Applications/Chromium.app/Contents/MacOS/Chromium',
    '/usr/bin/google-chrome',
    '/usr/bin/chromium',
    '/usr/bin/chromium-browser',
  ];
  for (const candidate of candidates) {
    if (fs.existsSync(candidate)) return candidate;
  }
  try {
    const found = execFileSync('bash', ['-lc', 'command -v google-chrome || command -v chromium || command -v chromium-browser || true'], {
      encoding: 'utf8',
    }).trim();
    if (found) return found.split('\n')[0];
  } catch (_) {
    return null;
  }
  return null;
}

const chromium = loadChromium();
const url = process.argv[2] || 'http://localhost:8765/level3_realistic_indoor.html';
const outDir = process.argv[3] || 'outputs/figures/level3_video_preview';
const chromePath = findChromePath();

const viewports = [
  { name: 'desktop', width: 1600, height: 900, progress: 0.55 },
  { name: 'mobile', width: 420, height: 860, progress: 0.55 },
];

async function canvasStats(page) {
  return await page.evaluate(() => {
    const canvas = document.querySelector('canvas#scene');
    if (!canvas) throw new Error('canvas#scene not found');
    const gl = canvas.getContext('webgl2') || canvas.getContext('webgl');
    if (!gl) throw new Error('WebGL context not available');
    const width = Math.min(canvas.width, 420);
    const height = Math.min(canvas.height, 260);
    const x0 = Math.max(0, Math.floor((canvas.width - width) / 2));
    const y0 = Math.max(0, Math.floor((canvas.height - height) / 2));
    const pixels = new Uint8Array(width * height * 4);
    gl.readPixels(x0, y0, width, height, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
    let sum = 0;
    let sumSq = 0;
    let colorful = 0;
    let dark = 0;
    for (let i = 0; i < pixels.length; i += 4) {
      const r = pixels[i];
      const g = pixels[i + 1];
      const b = pixels[i + 2];
      const lum = (r + g + b) / 3;
      sum += lum;
      sumSq += lum * lum;
      if (Math.max(r, g, b) - Math.min(r, g, b) > 22) colorful += 1;
      if (lum < 80) dark += 1;
    }
    const count = pixels.length / 4;
    const mean = sum / count;
    const variance = Math.max(0, sumSq / count - mean * mean);
    return {
      canvasWidth: canvas.width,
      canvasHeight: canvas.height,
      sampleWidth: width,
      sampleHeight: height,
      sampleX: x0,
      sampleY: y0,
      luminanceMean: Number(mean.toFixed(3)),
      luminanceStd: Number(Math.sqrt(variance).toFixed(3)),
      colorfulRatio: Number((colorful / count).toFixed(4)),
      darkRatio: Number((dark / count).toFixed(4)),
      frameStats: typeof window.__level3RealisticFrameStats === 'function' ? window.__level3RealisticFrameStats() : null,
    };
  });
}

(async () => {
  fs.mkdirSync(outDir, { recursive: true });
  const launchOptions = {
    headless: true,
    args: ['--no-sandbox', '--disable-gpu-sandbox', '--ignore-gpu-blocklist', '--enable-webgl'],
  };
  if (chromePath) launchOptions.executablePath = chromePath;
  const browser = await chromium.launch(launchOptions);
  const results = [];

  try {
    for (const viewport of viewports) {
      const page = await browser.newPage({ viewport });
      await page.goto(url, { waitUntil: 'networkidle', timeout: 60000 });
      await page.waitForFunction(() => window.__level3RealisticReady === true, null, { timeout: 30000 });
      await page.evaluate((progress) => window.__level3RealisticSetProgress(progress), viewport.progress);
      await page.waitForTimeout(500);
      const screenshot = path.join(outDir, `level3_realistic_${viewport.name}.png`);
      await page.screenshot({ path: screenshot, fullPage: false });
      const stats = await canvasStats(page);
      const ok =
        stats.canvasWidth > 0 &&
        stats.canvasHeight > 0 &&
        stats.luminanceStd > 5.0 &&
        stats.colorfulRatio > 0.02 &&
        stats.darkRatio >= 0 &&
        stats.frameStats &&
        stats.frameStats.obstacles >= 5;
      results.push({ viewport, screenshot, ok, ...stats });
      await page.close();
    }
  } finally {
    await browser.close();
  }

  const jsonPath = path.join(outDir, 'level3_realistic_render_check.json');
  fs.writeFileSync(jsonPath, `${JSON.stringify(results, null, 2)}\n`);
  for (const result of results) {
    console.log(
      `${result.ok ? 'PASS' : 'FAIL'} ${result.viewport.name}: ${result.screenshot} ` +
        `std=${result.luminanceStd} colorful=${result.colorfulRatio} dark=${result.darkRatio}`
    );
  }
  console.log(`Wrote ${jsonPath}`);
  if (results.some((result) => !result.ok)) process.exit(1);
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
