#!/usr/bin/env node
/* Verify the browser-based 3D simulation renders a nonblank WebGL scene.
 *
 * Requires `playwright-core` on NODE_PATH or in the current node resolution
 * tree, and a local Chrome/Chromium executable.
 */

const fs = require('fs');
const path = require('path');

const { chromium } = require('playwright-core');

const url = process.argv[2] || 'http://localhost:8765/';
const outDir = process.argv[3] || 'outputs/figures';
const chromePath =
  process.env.CHROME_PATH ||
  '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';

const viewports = [
  { name: 'desktop', width: 1440, height: 900 },
  { name: 'mobile', width: 390, height: 844 },
];

async function canvasStats(page) {
  return await page.evaluate(() => {
    const canvas = document.querySelector('canvas#scene');
    if (!canvas) {
      throw new Error('canvas#scene not found');
    }
    const gl = canvas.getContext('webgl2') || canvas.getContext('webgl');
    if (!gl) {
      throw new Error('WebGL context not available');
    }
    const width = Math.min(canvas.width, 360);
    const height = Math.min(canvas.height, 240);
    const pixels = new Uint8Array(width * height * 4);
    gl.readPixels(0, 0, width, height, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
    let sum = 0;
    let sumSq = 0;
    let nonBg = 0;
    const bg = [232, 237, 243];
    for (let i = 0; i < pixels.length; i += 4) {
      const r = pixels[i];
      const g = pixels[i + 1];
      const b = pixels[i + 2];
      const lum = (r + g + b) / 3;
      sum += lum;
      sumSq += lum * lum;
      if (Math.abs(r - bg[0]) + Math.abs(g - bg[1]) + Math.abs(b - bg[2]) > 30) {
        nonBg += 1;
      }
    }
    const count = pixels.length / 4;
    const mean = sum / count;
    const variance = Math.max(0, sumSq / count - mean * mean);
    return {
      canvasWidth: canvas.width,
      canvasHeight: canvas.height,
      sampleWidth: width,
      sampleHeight: height,
      luminanceMean: Number(mean.toFixed(3)),
      luminanceStd: Number(Math.sqrt(variance).toFixed(3)),
      nonBackgroundRatio: Number((nonBg / count).toFixed(4)),
    };
  });
}

(async () => {
  fs.mkdirSync(outDir, { recursive: true });
  const browser = await chromium.launch({
    executablePath: chromePath,
    headless: true,
    args: ['--no-sandbox', '--disable-gpu-sandbox'],
  });

  const results = [];
  try {
    for (const viewport of viewports) {
      const page = await browser.newPage({ viewport });
      await page.goto(url, { waitUntil: 'networkidle', timeout: 45000 });
      await page.waitForFunction(() => window.__uavSimReady === true, null, { timeout: 20000 });
      await page.waitForTimeout(1500);
      const shotPath = path.join(outDir, `uav_3d_sim_${viewport.name}.png`);
      await page.screenshot({ path: shotPath, fullPage: false });
      const stats = await canvasStats(page);
      const ok =
        stats.canvasWidth > 0 &&
        stats.canvasHeight > 0 &&
        stats.luminanceStd > 2.0 &&
        stats.nonBackgroundRatio > 0.015;
      results.push({ viewport, screenshot: shotPath, ok, ...stats });
      await page.close();
    }
  } finally {
    await browser.close();
  }

  const jsonPath = path.join(outDir, 'uav_3d_sim_render_check.json');
  fs.writeFileSync(jsonPath, `${JSON.stringify(results, null, 2)}\n`);
  for (const result of results) {
    console.log(
      `${result.ok ? 'PASS' : 'FAIL'} ${result.viewport.name}: ${result.screenshot} ` +
        `std=${result.luminanceStd} nonBg=${result.nonBackgroundRatio}`
    );
  }
  console.log(`Wrote ${jsonPath}`);
  if (results.some((result) => !result.ok)) {
    process.exit(1);
  }
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
