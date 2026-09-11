# Physical remote controller

`remote_controller_node` decodes the AKI physical radio remote from
proprietary CAN frames received through ROS2ISOBUS.

## CAN protocol

| PGN | Content |
| --- | --- |
| `0xFF00` | Joystick axes |
| `0xFF01` | Mode, safety, start, deadman and switches |
| `0xFF02` | Speed selector |

Frames are accepted only from the configured source address. Missing switch
traffic triggers the receive watchdog and publishes a disconnected, safe
status.

## Interfaces

| Direction | Interface | Type |
| --- | --- | --- |
| Subscribe | ROS2ISOBUS `bus_rx` | `ros2_isobus/IsobusFrame` |
| Publish | `robosoft/remote/status` | `RemoteControlStatus` |
| Publish | `joy` | `sensor_msgs/Joy` |

`joy` is a generic observation of the physical controls. Manual motion on the
real AKI goes directly from the remote to the lower-level drive controller;
RoboSoft does not close that manual control loop.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `layout` | `aki` | Remote button mapping selected by the deployment |
| `source_address` | `128` (`0x80`) | Expected remote CAN source address |
| `watchdog_timeout_ms` | `300` | Maximum allowed gap in required remote traffic |
