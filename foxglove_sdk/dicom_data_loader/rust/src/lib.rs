//! DICOM CT series played on a Foxglove timeline.

mod messages;
mod parse;
mod render;
mod study;

pub use study::{EmptyStudy, Study, load_study};

#[cfg(target_arch = "wasm32")]
mod loader;

#[cfg(target_arch = "wasm32")]
pub use loader::DicomLoader;

#[cfg(target_arch = "wasm32")]
foxglove_data_loader::export!(DicomLoader);
