//! DICOM CT data loader for Foxglove.
//!
//! [`load_study`] turns `.dcm` readers into timeline messages. The WASM adapter
//! is the only piece that talks to the Foxglove host.

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
