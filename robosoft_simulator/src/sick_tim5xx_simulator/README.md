# SICK TiM5xx Ethernet simulator

`sick_tim5xx_simulator_node` is a reusable ROS-to-Ethernet adapter. It consumes
the same `sensor_msgs/PointCloud2` interface normally published by a lidar
driver (`cloud` by default) and presents that cloud to an unmodified
`sick_scan_xd` TiM5xx driver over TCP CoLa B.

The node has no vehicle, map, TASK or odometry dependency. Any simulator or
algorithm capable of publishing a lidar-frame point cloud can provide its
input. RoboSoft's TASK-based example source is documented in
[`task_lidar_cloud_generator`](../task_lidar_cloud_generator/README.md).

## Conversion

Finite `x` and `y` coordinates are assigned to the nearest ray of the fixed
TiM5xx profile: 811 rays from -135 to +135 degrees at approximately 0.3333
degree resolution. When multiple input points occupy one ray, the nearest
range is used. Missing rays use SICK's zero no-return code. Ranges above
65.535 m cannot be represented by the 16-bit millimetre channel and are
ignored. Input fields other than `x` and `y` are allowed and ignored.

The Ethernet stream runs at 15 Hz by default using the latest complete cloud.
Transmission pauses if the cloud becomes stale. RSSI is synthetic: 100 for a
return and zero otherwise.

## Parameters and interfaces

| Parameter | Default | Meaning |
| --- | --- | --- |
| `cloud_topic` | `cloud` | Input `sensor_msgs/PointCloud2` topic |
| `cloud_timeout_s` | `1.0` | Stop scan transmission when input is stale |
| `scan_rate_hz` | `15.0` | Ethernet scan transmission rate |
| `bind_address` | `0.0.0.0` | IPv4 listen address |
| `port` | `2112` | TCP CoLa B port |

The server handles one scanner connection at a time, accepts reconnection and
exits on SIGINT. Nonblocking sockets preserve partial TCP frames and writes and
limit buffering for stalled clients.

Supported commands cover the TiM5xx driver startup and scan session:
`SCdevicestate`, `SetAccessMode`, `EIHstCola`, `FirmwareVersion`, `DeviceIdent`,
`SerialNumber`, `LocationName`, `ODoprh`, `ODpwrc`, `LMPoutputRange`,
`LMPscancfg`, `LMDscandatacfg`, `LMCstartmeas`, `LMCstopmeas`, `Run`,
`FREchoFilter`, and polled/event `LMDscandata`. The profile is fixed and CoLa A
is not supported.

Protocol layout was checked against the upstream
[SICK TiM5xx configuration](https://github.com/SICKAG/sick_scan_xd/blob/master/launch/sick_tim_5xx.launch)
and [binary scan parser](https://github.com/SICKAG/sick_scan_xd/blob/master/driver/src/sick_lmd_scandata_parser.cpp).

## Standalone use

Start a point-cloud source in the simulator's ROS domain and then run:

```bash
ros2 run robosoft_simulator sick_tim5xx_simulator_node
```

On the controller computer, point the real driver to the simulator computer:

```bash
ros2 run sick_scan_xd sick_generic_caller \
  "$(ros2 pkg prefix --share sick_scan_xd)/launch/sick_tim_5xx.launch" \
  hostname:=192.168.1.100
```

When both nodes run on one machine, use separate ROS domains or remap the
simulator input. Otherwise the driver's output `cloud` would share a name with
the simulator input. The supplied AKI scenario already isolates the simulator
in domain 42.

## Verification

The integration test launches this adapter, the TASK cloud generator and the
installed, unmodified `sick_scan_xd` driver:

```bash
python3 src/robosoft_simulator/test/lidar_driver_smoke.py
```

It verifies geometry, rate, movement response and driver reconnection over TCP.
