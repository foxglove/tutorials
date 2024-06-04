# ROS Client Librart for Rust: rclrs

This package contains the code for the diagnostics tutorial found on [Foxglove's blog]("https://foxglove.dev/blog).

It contains the package `string_length_node`, which as one node. This node uses the `rclrs` to subscribe to a `String` message and publishes in another topic of type `UInt8` the length of the last message received.

