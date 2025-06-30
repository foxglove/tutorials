---
title: "SubPipe Dataset to MCAP"
video_url: "https://youtu.be/jJej6aT1jKg"
visualize_url: "https://app.foxglove.dev/~/view?ds=foxglove-sample-stream&ds.recordingId=vqKKQcot421Kwg84&ds.overrideLayoutId=b7513959-1d46-4a89-bc24-1584d9677ca1&ds.start=2023-09-01T13:19:45.047438263Z&ds.end=2023-09-01T13:20:15.047438263Z"
short_description: "The underwater dataset, containing SLAM, object detection and image segmentation"
---

# Subpipe dataset to MCAP

![Subpipe](subpipe.gif)

## First steps

1. Download the dataset from [here](https://zenodo.org/records/12666132).
2. Copy the folder `DATA` into this directory:

* `subpipe_mcap`
    * `DATA`
    * `foxglove_layout`
    * `schemas`
    * `convert_csv2mcap.py`
    * ...

3. Run the script `convert_csv2mcap.py`. The MCAP file will be generated in an `output` folder.
4. Open Foxglove and load the layout `foxglove_layout/Subpipe.json`


