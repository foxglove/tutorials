import { mkdir } from "node:fs/promises";
import path from "node:path";

import puppeteer from "puppeteer-core";

const late = process.argv.includes("--late");
const args = process.argv.slice(2).filter((arg) => arg !== "--late");
const url = args[0] ?? "http://127.0.0.1:5173/";
const mediaDir = path.resolve(args[1] ?? "../../media");

const browser = await puppeteer.launch({
  executablePath: "/usr/local/bin/google-chrome",
  headless: true,
  args: [
    "--no-sandbox",
    "--disable-dev-shm-usage",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--enable-webgl",
    "--ignore-gpu-blocklist",
    "--window-size=1400,900",
  ],
  defaultViewport: { width: 1400, height: 900, deviceScaleFactor: 1 },
});

try {
  const page = await browser.newPage();
  page.on("console", (message) => {
    console.log(`PAGE ${message.type()}: ${message.text()}`);
  });
  page.on("pageerror", (error) => {
    console.log(`PAGEERROR ${error.message}`);
  });
  await page.goto(url, { waitUntil: "networkidle0", timeout: 60000 });
  await page.waitForSelector("canvas", { timeout: 20000 });
  await page.waitForFunction(
    () => {
      const root = document.querySelector("[data-pose-count]");
      if (!root) {
        return false;
      }
      return (
        root.getAttribute("data-loading") === "false" &&
        Number(root.getAttribute("data-pose-count")) > 50
      );
    },
    { timeout: 20000 },
  );
  await new Promise((resolve) => setTimeout(resolve, 400));
  await dollyTowardTruck(page);

  const panel = await page.$("#panel");
  if (!panel) {
    throw new Error("Panel element missing");
  }
  await mkdir(mediaDir, { recursive: true });

  const header = await page.$("#header");
  const headerBox = await header?.boundingBox();
  if (headerBox) {
    await page.mouse.move(headerBox.x + 20, headerBox.y + 10);
  }
  await new Promise((resolve) => setTimeout(resolve, 250));
  const currentPath = path.join(mediaDir, "current-truck.png");
  await panel.screenshot({ path: currentPath });
  console.log("wrote", currentPath);

  const timeline = await page.$("#timeline");
  const box = await timeline?.boundingBox();
  if (!box) {
    throw new Error("Timeline element missing");
  }
  await page.mouse.move(box.x + box.width * 0.42, box.y + box.height / 2);
  await new Promise((resolve) => setTimeout(resolve, 500));
  const ghostPath = path.join(mediaDir, "ghost-preview.png");
  await panel.screenshot({ path: ghostPath });
  console.log("wrote", ghostPath);

  if (late) {
    const lateUrl = new URL(url);
    lateUrl.searchParams.set("t", "34");
    await page.goto(lateUrl.toString(), { waitUntil: "networkidle0", timeout: 60000 });
    await page.waitForFunction(
      () => {
        const root = document.querySelector("[data-pose-count]");
        if (!root) {
          return false;
        }
        return (
          root.getAttribute("data-loading") === "false" &&
          Number(root.getAttribute("data-pose-count")) > 50
        );
      },
      { timeout: 20000 },
    );
    await new Promise((resolve) => setTimeout(resolve, 400));
    await dollyTowardTruck(page);
    const latePanel = await page.$("#panel");
    const lateTimeline = await page.$("#timeline");
    const lateBox = await lateTimeline?.boundingBox();
    if (!latePanel || !lateBox) {
      throw new Error("Late-seek capture missing panel or timeline");
    }
    await page.mouse.move(lateBox.x + lateBox.width * (24 / 40), lateBox.y + lateBox.height / 2);
    await new Promise((resolve) => setTimeout(resolve, 500));
    const latePath = path.join(mediaDir, "late-ghost.png");
    await latePanel.screenshot({ path: latePath });
    console.log("wrote", latePath);
  }
} finally {
  await browser.close();
}

async function dollyTowardTruck(page) {
  const canvas = await page.$("#panel canvas");
  const box = await canvas?.boundingBox();
  if (!box) {
    return;
  }
  await page.mouse.move(box.x + box.width * 0.5, box.y + 28);
  for (let step = 0; step < 16; step += 1) {
    await page.mouse.wheel({ deltaY: -100 });
    await new Promise((resolve) => setTimeout(resolve, 20));
  }
  await new Promise((resolve) => setTimeout(resolve, 300));
}
