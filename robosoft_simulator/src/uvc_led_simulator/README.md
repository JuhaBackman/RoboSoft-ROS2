# UVC and LED simulator

`uvc_led_simulator_node` emulates the proprietary treatment and camera-light
CAN interface used by the supplied AKI scenario. It decodes commands, publishes
hardware feedback and completes treatment sequences after a configurable
duration.

The node subscribes to `ISOBUS/bus_rx_frames` and publishes status and address
claims to `ISOBUS/bus_tx_frames` using `ros2_isobus/msg/IsobusFrame`. The
ROS2ISOBUS `can_bridge_node` is the only component that accesses SocketCAN.

The component can be disabled when testing an application without this
implement protocol.
