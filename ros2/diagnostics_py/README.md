# Diagnostics tutorial (Python)

This package contains the code for the diagnostics tutorial found on [Foxglove's blog]("https://foxglove.dev/blog).

It contains the `diagnostic_publisher` node that publishes a `DiagnosticArray` message with two `DiagnosticStatus` elements:
1. Element 1 contains diagnostic information about a supervised topic of type Int16 
2. Element 2 contains diagnostic information about a counter inside the node
