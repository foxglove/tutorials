import path from "node:path";
import { fileURLToPath } from "node:url";

import { defineConfig } from "vite";

const root = path.dirname(fileURLToPath(import.meta.url));

export default defineConfig({
  root,
  server: {
    host: "127.0.0.1",
    port: 5173,
    strictPort: true,
  },
  esbuild: {
    jsx: "automatic",
  },
});
