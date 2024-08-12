# Diagnostics basic use

This folder contains two packages that publish a minimal example of `DiagnosticArray`, as seen in the example tutorial in [Foxglove's blog]("https://foxglove.dev/blog).

These packages only serve as as basic example to understand the fundamentals of diagnostic messages. For more advanced diagnostics, refer to the package [`diagnostic_updater`](http://wiki.ros.org/diagnostic_updater).

The packages `diagnostics_publisher_cpp` and `diagnostics_publisher_py` have the same functionality:

A `diagnostic_publisher` node that publishes a `DiagnosticArray` message with two `DiagnosticStatus` elements:
1. Element 1 contains diagnostic information about a supervised topic of type Int16 
2. Element 2 contains diagnostic information about a counter inside the node

