# Tractor-state simulator

`tractor_state_simulator_node` publishes engine, key-switch, guidance, hitch,
PTO and auxiliary-valve measurements to the ROS2ISOBUS T-ECU server input
topics. Values and publication rate are configurable.

Stopping this node exercises T-ECU input timeouts: the server returns affected
CAN signals to their ISO 11783 `not available` encodings.
