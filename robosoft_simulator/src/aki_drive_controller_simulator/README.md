# Lower-level drive-controller example

`aki_drive_controller_simulator_node` demonstrates a lower-level manual drive
controller that operates outside the autonomous RoboSoft command path. It
consumes the processed simulator remote topics and sends a guarded
`geometry_msgs/TwistStamped` command to the vehicle model.

Enable latching, axis deadzones, speed and curvature limits and input
watchdogs reproduce the behaviour expected by the supplied AKI scenario. This
node is application-specific; other platforms can replace it while retaining
the generic vehicle and sensor simulators.
