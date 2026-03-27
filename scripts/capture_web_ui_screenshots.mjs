#!/usr/bin/env node
/**
 * Capture screenshots of all web_ui tabs for bug documentation.
 * Usage: node scripts/capture_web_ui_screenshots.mjs [--prefix pre|post] [--url http://...]
 */

import pkg from '/tmp/node_modules/playwright-core/index.js';
const { chromium } = pkg;
import { mkdir } from 'fs/promises';
import path from 'path';

const args = process.argv.slice(2);
const prefix = args.includes('--prefix') ? args[args.indexOf('--prefix') + 1] : 'screenshot';
const urlArg = args.includes('--url') ? args[args.indexOf('--url') + 1] : null;
const BASE_URL = urlArg || 'http://client.ros2.lan:8080';

const OUT_DIR = path.join(import.meta.dirname, '..', 'docs', 'web_ui_bugs');

const TABS = [
  { label: 'Gripper Cam', name: 'camera' },
  { label: 'IMU', name: 'sensor_graph' },
  { label: 'Arm Servos', name: 'effector_graph' },
  { label: 'Local Map', name: 'nav_local' },
  { label: 'GPS Map', name: 'nav_gps' },
  { label: '3D Scene', name: 'scene3d' },
  { label: 'Robot Status', name: 'robot_status' },
];

async function run() {
  await mkdir(OUT_DIR, { recursive: true });

  const executablePath = process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH ||
    '/Users/szymonri/Library/Caches/ms-playwright/chromium_headless_shell-1208/chrome-headless-shell-mac-arm64/chrome-headless-shell';
  const browser = await chromium.launch({ headless: true, executablePath });
  const page = await browser.newPage();
  await page.setViewportSize({ width: 1280, height: 900 });

  console.log(`Opening ${BASE_URL} ...`);
  try {
    await page.goto(BASE_URL, { waitUntil: 'networkidle', timeout: 15000 });
  } catch (e) {
    console.error(`Failed to load ${BASE_URL}: ${e.message}`);
    await browser.close();
    process.exit(1);
  }

  // Wait for React to mount
  await page.waitForTimeout(2000);

  for (const tab of TABS) {
    // Click the tab button by exact text
    try {
      await page.locator(`.tab-btn:text-is("${tab.label}")`).first().click({ timeout: 5000 });
    } catch {
      try {
        await page.getByRole('button', { name: tab.label, exact: true }).click({ timeout: 5000 });
      } catch (e2) {
        console.warn(`Could not click tab "${tab.label}": ${e2.message}`);
      }
    }
    // Wait for render
    await page.waitForTimeout(3000);

    const file = path.join(OUT_DIR, `${prefix}_${tab.name}.png`);
    await page.screenshot({ path: file, fullPage: false });
    console.log(`Saved: ${file}`);
  }

  await browser.close();
  console.log('Done.');
}

run().catch((e) => { console.error(e); process.exit(1); });
