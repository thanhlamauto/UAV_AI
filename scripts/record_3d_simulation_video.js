#!/usr/bin/env node
/* Record the browser 3D UAV simulation into an MP4.
 *
 * The script expects the simulation to be served over HTTP, for example:
 *
 *   scripts/serve_3d_simulation.sh 8765
 *
 * It uses Playwright Core with a local Chrome/Chromium executable, captures
 * deterministic frames by setting the simulation progress from JS, and encodes
 * them with ffmpeg.
 */

const fs = require('fs');
const os = require('os');
const path = require('path');
const { spawnSync } = require('child_process');

const { chromium } = require('playwright-core');

function argValue(flag, fallback) {
  const idx = process.argv.indexOf(flag);
  if (idx >= 0 && idx + 1 < process.argv.length) return process.argv[idx + 1];
  return fallback;
}

const url = argValue('--url', 'http://localhost:8765/');
const output = argValue('--output', 'outputs/videos/uav_3d_simulation_astar.mp4');
const planner = argValue('--planner', 'astar');
const cameraMode = argValue('--camera', 'orbit');
const fps = Number(argValue('--fps', '24'));
const duration = Number(argValue('--duration-s', '10'));
const width = Number(argValue('--width', '1280'));
const height = Number(argValue('--height', '720'));
const chromePath =
  process.env.CHROME_PATH ||
  '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';

if (!Number.isFinite(fps) || fps <= 0) throw new Error('Invalid --fps');
if (!Number.isFinite(duration) || duration <= 0) throw new Error('Invalid --duration-s');
if (!Number.isFinite(width) || width <= 0) throw new Error('Invalid --width');
if (!Number.isFinite(height) || height <= 0) throw new Error('Invalid --height');

function runFfmpeg(frameDir, outputPath) {
  fs.mkdirSync(path.dirname(outputPath), { recursive: true });
  const result = spawnSync(
    'ffmpeg',
    [
      '-y',
      '-v',
      'error',
      '-framerate',
      String(fps),
      '-i',
      path.join(frameDir, 'frame_%05d.png'),
      '-c:v',
      'libx264',
      '-pix_fmt',
      'yuv420p',
      '-movflags',
      '+faststart',
      outputPath,
    ],
    { encoding: 'utf8' }
  );
  if (result.status !== 0) {
    throw new Error(`ffmpeg failed: ${result.stderr || result.stdout}`);
  }
}

(async () => {
  const frameCount = Math.max(2, Math.round(duration * fps));
  const frameDir = fs.mkdtempSync(path.join(os.tmpdir(), 'uav-3d-sim-frames-'));
  const browser = await chromium.launch({
    executablePath: chromePath,
    headless: true,
    args: ['--no-sandbox', '--disable-gpu-sandbox'],
  });

  try {
    const page = await browser.newPage({ viewport: { width, height } });
    await page.goto(url, { waitUntil: 'networkidle', timeout: 45000 });
    await page.waitForFunction(() => window.__uavSimReady === true, null, { timeout: 20000 });
    await page.evaluate((mode) => {
      if (typeof window.__uavSimSetCameraMode === 'function') {
        window.__uavSimSetCameraMode(mode);
      }
    }, cameraMode);

    for (let idx = 0; idx < frameCount; idx += 1) {
      const alpha = frameCount <= 1 ? 1 : idx / (frameCount - 1);
      await page.evaluate(
        ({ progress, plannerName }) => window.__uavSimSetProgress(progress, plannerName),
        { progress: alpha, plannerName: planner }
      );
      await page.waitForTimeout(20);
      await page.screenshot({
        path: path.join(frameDir, `frame_${String(idx + 1).padStart(5, '0')}.png`),
        fullPage: false,
      });
    }
  } finally {
    await browser.close();
  }

  runFfmpeg(frameDir, output);
  fs.rmSync(frameDir, { recursive: true, force: true });
  console.log(`Wrote ${output}`);
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
