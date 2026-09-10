# SCAN-Planner Nav2 Controller

This package adapts the local planning core of SCAN-Planner to the
`nav2_core::Controller` interface. It consumes the Nav2 global path, extracts a
rolling local reference, repairs colliding path sections with costmap A*,
optimizes a cubic uniform B-spline using rebound-style smoothness, collision,
dynamic-feasibility and fitness costs, and returns differential-drive velocity
commands to Nav2's controller server.

Unlike the original standalone SCAN-Planner stack, this plugin deliberately
uses Nav2's local costmap, TF, odometry and command pipeline. Do not launch the
standalone `scan_planner_node` or its `closed_loop_controller` at the same time.

The optimized local trajectory is published on
`/controller_server/scan_planner_trajectory` for RViz inspection.
