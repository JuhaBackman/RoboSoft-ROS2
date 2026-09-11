# UVC and LED controller

`uvc_led_node` converts logical RoboSoft implement modes into the proprietary
AKI UVC/camera-light CAN protocol and decodes hardware feedback.

## CAN protocol

| PGN | Direction | Purpose |
| --- | --- | --- |
| `0xFF30` | RoboSoft to hardware | UVC, camera trigger and light command |
| `0xFF31` | Hardware to RoboSoft | Sequence state and completion feedback |

The node obtains its transmitting source address from the ROS2ISOBUS address
manager. It retries commands until matching feedback is received. Missing
feedback makes `ImplementStatus.connected` false.

## Interfaces

- Subscribes `robosoft/implement/command`, ROS2ISOBUS `bus_rx` and address
  manager status.
- Publishes ROS2ISOBUS `bus_tx` and the transient-local
  `robosoft/implement/status`.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `camera_light_time_ds` | `40` | Camera-light time in deciseconds |
| `camera_trigger_time_ds` | `30` | Camera-trigger time in deciseconds |
| `camera_light_power` | `500` | Camera-light power command |
| `uvc_light_time_ds` | `200` | UVC treatment time in deciseconds |
| `uvc_light_distance_mm` | `150` | UVC light distance in millimetres |
| `status_source_address` | `-1` | Expected feedback source; `-1` accepts any source |
| `feedback_timeout_ms` | `1000` | Hardware feedback watchdog |
