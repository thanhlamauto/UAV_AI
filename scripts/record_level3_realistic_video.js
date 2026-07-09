#!/usr/bin/env node
/* Record the realistic Level-3 indoor UAV WebGL demo into an MP4. */

const fs = require('fs');
const os = require('os');
const path = require('path');
const { execFileSync, spawnSync } = require('child_process');

function loadChromium() {
  try {
    return require('playwright-core').chromium;
  } catch (coreError) {
    try {
      return require('playwright').chromium;
    } catch (fullError) {
      throw coreError;
    }
  }
}

const chromium = loadChromium();

function argValue(flag, fallback) {
  const idx = process.argv.indexOf(flag);
  if (idx >= 0 && idx + 1 < process.argv.length) return process.argv[idx + 1];
  return fallback;
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

const url = argValue('--url', 'http://localhost:8765/level3_realistic_indoor.html');
const output = argValue('--output', 'outputs/videos/level3_realistic_indoor_pov_esdf_mppi.mp4');
const preview = argValue('--preview', 'outputs/figures/level3_video_preview/level3_realistic_midframe.png');
const previewLate = argValue('--preview-late', 'outputs/figures/level3_video_preview/level3_realistic_lateframe.png');
const cameraMode = argValue('--camera', 'pov');
const fps = Number(argValue('--fps', '24'));
const duration = Number(argValue('--duration-s', '15'));
const width = Number(argValue('--width', '1920'));
const height = Number(argValue('--height', '1080'));
const chromePath = findChromePath();

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
      '-preset',
      'medium',
      '-crf',
      '18',
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
  const frameDir = fs.mkdtempSync(path.join(os.tmpdir(), 'level3-realistic-frames-'));
  const launchOptions = {
    headless: true,
    args: [
      '--no-sandbox',
      '--disable-gpu-sandbox',
      '--ignore-gpu-blocklist',
      '--enable-webgl',
      '--enable-accelerated-2d-canvas',
      '--window-size=' + width + ',' + height,
    ],
  };
  if (chromePath) launchOptions.executablePath = chromePath;
  const browser = await chromium.launch(launchOptions);

  try {
    const page = await browser.newPage({ viewport: { width, height }, deviceScaleFactor: 1 });
    await page.goto(url, { waitUntil: 'networkidle', timeout: 60000 });
    await page.waitForFunction(() => window.__level3RealisticReady === true, null, { timeout: 30000 });
    await page.evaluate((mode) => {
      if (typeof window.__level3RealisticSetCameraMode === 'function') {
        window.__level3RealisticSetCameraMode(mode);
      }
    }, cameraMode);

    fs.mkdirSync(path.dirname(preview), { recursive: true });
    for (let idx = 0; idx < frameCount; idx += 1) {
      const alpha = frameCount <= 1 ? 1 : idx / (frameCount - 1);
      await page.evaluate((progress) => window.__level3RealisticSetProgress(progress), alpha);
      await page.waitForTimeout(12);
      const framePath = path.join(frameDir, `frame_${String(idx + 1).padStart(5, '0')}.png`);
      await page.screenshot({ path: framePath, fullPage: false });
      if (idx === Math.floor(frameCount * 0.50)) fs.copyFileSync(framePath, preview);
      if (idx === Math.floor(frameCount * 0.78)) fs.copyFileSync(framePath, previewLate);
    }
  } finally {
    await browser.close();
  }

  runFfmpeg(frameDir, output);
  fs.rmSync(frameDir, { recursive: true, force: true });
  console.log(`Wrote ${output}`);
  console.log(`Wrote ${preview}`);
  console.log(`Wrote ${previewLate}`);
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
