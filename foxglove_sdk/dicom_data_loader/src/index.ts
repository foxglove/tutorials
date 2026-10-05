import type { Experimental } from "@foxglove/extension";

import wasmUrl from "../rust/target/wasm32-unknown-unknown/release/foxglove_dicom_data_loader.wasm";

export function activate(extensionContext: Experimental.ExtensionContext): void {
  extensionContext.registerDataLoader({
    type: "file",
    wasmUrl,
    supportedFileType: ".dcm",
    supportsMultiFile: true,
  });
}
