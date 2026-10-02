import type { ExtensionContext } from "@foxglove/extension";

import { initGhostPreviewPanel } from "./GhostPreviewPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Model Ghost Preview",
    initPanel: initGhostPreviewPanel,
  });
}
