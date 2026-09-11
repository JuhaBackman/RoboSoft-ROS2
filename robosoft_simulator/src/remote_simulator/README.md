# Remote simulator

`remote_simulator_node` maps `sensor_msgs/Joy` or runtime parameters to a
processed remote-control state and proprietary CAN frames. Joystick axis,
button and sign mappings are parameters so different controllers can be used.

CAN frames are published as `ros2_isobus/msg/IsobusFrame` on
`ISOBUS/bus_tx_frames`. Requests received from the ROS2ISOBUS `can_bridge_node`
on `ISOBUS/bus_rx_frames` trigger the simulated remote's address claim. The node
does not open a SocketCAN interface.

The node publishes the processed state locally for other simulator components
and applies a receive timeout that releases start/deadman and reports an unsafe
state when joystick data disappears.

The default PS4 mapping uses Share (button 8) and Options (button 9) as
momentary emergency-stop inputs. Holding either button sets the remote safety
circuit to unsafe; releasing both returns it to the normal safe state. The
button indices are configurable with `button_emergency_stop_share` and
`button_emergency_stop_options`.
