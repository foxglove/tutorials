---
title: "Using Foxglove to Visualize Ethernet/IP data"
blog_post_url: "TODO"
short_description: "Using Foxglove data, it's easier than ever to stream time series data. In this project, we show you how."
---

# Use Foxglove for Real-Time Industrial PLC Data Visualization and Playback

In this tutorial, we implement a simple script to stream values of Ethernet/IP tags to Foxglove. The example includes a simulated plant to demonstrate this proof of concept.

## Dependencies
Run `pip install -r requirements.txt` to install dependencies for this tutorial. We depend on `cpppo` for simulating an Ethernet/IP device, and `pylogix` to run the simulated plant and read the tag values for our Foxglove bridge.

## Running the example
Make sure you’ve cloned the repository and installed the dependencies from the previous section.

Navigate to the server directory (`foxglove_sdk/ethernet_ip_integration/server`) and execute these two scripts in separate terminal windows:
```
python3 eip_server.py
python3 run_plant.py
```
After the plant is started, run the foxglove_eip_bridge.py (located in `foxglove_sdk/ethernet_ip_integration/`):
```
python3 foxglove_eip_bridge.py
```
If successful, it should start Foxglove server and show the following output:
```
2025-07-04 18:18:52,547 [INFO] Started server on 127.0.0.1:8765
2025-07-04 18:18:52,548 [INFO] Starting EtherNet/IP to Foxglove bridge
2025-07-04 18:18:52,548 [INFO] Connecting to PLC at 0.0.0.0
2025-07-04 18:18:52,548 [INFO] Streaming 35 tags to Foxglove
```

## Viewing the data
To view the data, start Foxglove and open a new connection to `ws://localhost:8765`. If everything is well, you will see several topics derived from EIP tags. Now, you should be able to load different panels and view the data. You can use a layout in `foxglove_sdk/ethernet_ip_integration/layouts/eip_layout.json`.
