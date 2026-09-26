---
title: "Play DICOM CT series in Foxglove"
short_description: "A Rust data loader that opens uncompressed DICOM files and plays slices or breathing phases on the timeline."
---

# Play DICOM CT series in Foxglove

Foxglove does not open DICOM. This tutorial is a data loader extension, written in Rust and compiled to WebAssembly, that does. Drag a folder of `.dcm` files into Foxglove and the timeline becomes the navigation control: a sweep through axial slices, or a short animation of a patient breathing. No MCAP conversion and no custom panel.

The loader publishes lung-window images, a 3D point cloud of bone and lung, a plot of lung area or volume, and a metadata message.

## Prerequisites

- Rust stable, with the `wasm32-unknown-unknown` target (`rustup target add wasm32-unknown-unknown`)
- Node.js 18 or newer
- Python 3, only for downloading the sample series

## Download sample data

Both series are public, CC BY 3.0, and hosted by the NCI Imaging Data Commons. The script uses the `idc-index` package. Nothing is committed to the repository.

```bash
pip install -r scripts/requirements.txt
python scripts/download_sample_data.py --dataset chest      # one 133-slice chest CT
python scripts/download_sample_data.py --dataset breathing  # ten respiratory phases
python scripts/download_sample_data.py --dataset all
```

Files are written under `data/chest` and `data/breathing`.

## Build and install

From this directory:

```bash
npm install
npm run package
```

`npm run package` builds the release WebAssembly module and writes `foxglove.foxglove-dicom-data-loader-0.1.0.foxe`. In Foxglove, open Settings → Extensions and install that file. For the desktop app you can instead run `npm run local-install`.

## Open the data

Open one dataset at a time. Select or drag every `.dcm` file from `data/chest` or from `data/breathing` together. The loader only sees the files you hand it, so a partial selection is a partial volume.

Import `foxglove_layouts/dicom_layout.json` for a layout with axial, coronal, and sagittal images, a 3D view, the stats plot, and the metadata panel. The coronal and sagittal panels subscribe to `/dicom/coronal/annotations` and `/dicom/sagittal/annotations`. Those topics exist in sweep mode and draw a line at the current axial slice. 4D mode does not publish them.

Press play.

- **Chest CT** is sweep mode. Each frame is the next axial slice, inferior to superior, at 100 ms per slice (about 13 seconds). Coronal and sagittal views are the centre multi-planar reconstruction, published once. The line on those views tracks the slice. The 3D panel shows bone, lung, and the current slice as a point cloud.
- **4D lung** is breathing mode. The loader finds ten series that share a frame of reference and the same geometry, ordered by the phase percentage in `SeriesDescription`. Each frame is one phase at 400 ms, and the cycle repeats three times so playback looks continuous. The centre coronal image and the lung cloud update every phase, so the diaphragm and lung volume move. Bone, the bounding box, and the metadata message stay at the start of the timeline; Foxglove keeps them visible by backfill when you seek.

## How it works

`rust/src/lib.rs` exports a [`DataLoader`](https://docs.rs/foxglove_data_loader). Foxglove calls `initialize` with the paths you opened. The loader reads each file through the host `reader` interface, keeps uncompressed little-endian monochrome slices, and groups them by `SeriesInstanceUID`.

Slices in a series are ordered along the slice normal (`ImagePositionPatient` dotted with the cross product of the image orientation). The first frame is the inferior end.

Mode detection:

- **4D** when at least two series share `FrameOfReferenceUID` and have the same rows, columns, and slice count. Phases are ordered by a percentage parsed from the series description, such as `Gated, 40.0%`, and otherwise by series number.
- **Sweep** otherwise, using the series with the most slices. Extra series produce a warning.

`StudyDate` and `StudyTime` become the timeline origin, interpreted as UTC. If they do not parse, the origin is zero. Messages are not materialised up front. `create_iter` yields one frame at a time, and `get_backfill` returns the latest message at or before the seek time so panels are not empty when playback starts in the middle.

Images are windowed with a CT lung window (level -600 HU, width 1500) and encoded as `mono8`. Coronal and sagittal views resample along the stack so the spacing matches the in-plane pixels, with superior at the top. Positions are DICOM LPS millimetres converted to metres in a `patient` frame whose origin is the centre of the volume, so the anatomy sits at the origin of Foxglove's z-up view. Every 4D phase uses the first phase's origin. Recentering each phase would hide the breathing motion.

The lung mask is a demo heuristic. A flood fill from the image border marks air connected to the outside of the body (voxels below -400 HU). Voxels that are not outside air and sit between -1000 and -400 HU count as lung. Body is everything that is not outside air. Bone is HU of 250 or higher. Point clouds are stride-subsampled so each cloud stays at or under 300,000 points.

## Limitations

Only uncompressed little-endian transfer syntaxes are read (Implicit VR and Explicit VR). JPEG and other encapsulated pixel data are skipped, with a warning. Multi-frame files are skipped. The lung and body masks are not a segmentation. The loader does not publish patient name, ID, or other identifying fields.

## Data

- Chest CT: LIDC-IDRI, series `1.3.6.1.4.1.14519.5.2.1.6279.6001.179049373636438705059720603192`. Armato SG 3rd, McLennan G, Bidaut L, McNitt-Gray MF, Meyer CR, Reeves AP, Zhao B, Aberle DR, Henschke CI, Hoffman EA, Kazerooni EA, MacMahon H, Van Beek EJR, Yankelevitz D, et al.: The Lung Image Database Consortium (LIDC) and Image Database Resource Initiative (IDRI): A completed reference database of lung nodules on CT scans. Medical Physics, 38: 915--931, 2011. [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/). Data from [The Cancer Imaging Archive](https://www.cancerimagingarchive.net/).
- Breathing CT: 4D-Lung, patient `100_HM10395`, study S100. Hugo GD, Weiss E, Sleeman WC, Balik S, Keall PJ, Lu J, Williamson JF. (2016). Data from 4D Lung Imaging of NSCLC Patients. The Cancer Imaging Archive. [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/).
