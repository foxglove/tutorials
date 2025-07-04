---
title: "ROS 2 Rust Tutorial (rclrs)"
short_description: "Tutorial for using ROS Client Library for Rust"
blog_post_url: "https://foxglove.dev/blog/first-steps-using-rust-with-ros2"
---

# ROS Client Librart for Rust: rclrs

This package contains the code for the ROS 2 Rust tutorial found on [Foxglove's blog](https://foxglove.dev/blog/first-steps-using-rust-with-ros2).

It contains the package `string_length_node`, which has one node. This node uses the `rclrs` to subscribe to a `String` message and publishes in another topic of type `UInt8` the length of the last message received.

To use this package, it has to be used in a workspace with the `rclrs` sourced. Please follow instructions [here](https://github.com/ros2-rust/ros2_rust/tree/main) to install and compile `rclrs`.
