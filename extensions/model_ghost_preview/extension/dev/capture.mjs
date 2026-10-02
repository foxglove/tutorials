import { mkdir } from "node:fs/promises";
import path from "node:path";

import puppeteer from "puppeteer-core";

const url = process.argv[2] ?? "http://127.0.0.1:5173/";
const mediaDir = path.resolve(process.argv[3] ?? "../../media");

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
} finally {
  await browser.close();
}
