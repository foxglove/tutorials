# Visualizing Open X-Embodiment datasets in Foxglove

These are the tutorial files that show how you can visualize Open-X Embodiment dataset in Foxglove.

To execute the script, you will need to install the following Python dependencies:

```
tensorflow
gcsfs
foxglove-sdk
tensorflow_datasets
```

We include a layout `foxglove_tutorials/open_x_embodiment/layouts/berkeley_autloab_ur5_layout.json` that you can use to view all the data that we included in the tutorial.

## Running the script

1. Run the Python script: `python3 visualize_berkeley_autolab_ur5.py`
2. In Foxglove, open a remote connection to `ws://localhost:8765`
3. Arrange panels to visualize the data, or use the layout included in this tutorial: `open_x_embodiment/layouts/berkeley_autloab_ur5_layout.json`
