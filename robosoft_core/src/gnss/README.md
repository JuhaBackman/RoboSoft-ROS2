# GNSS

Generic serial, NMEA 0183, NTRIP and geographic-coordinate nodes. The nodes
are protocol adapters and contain no platform-specific state logic.

## Components

| Node | Responsibility |
| --- | --- |
| `serial_node` | Owns one serial device, publishes complete input lines and writes binary output |
| `nmea0183_parser_node` | Converts supported NMEA 0183 sentences to standard ROS measurements |
| `ntrip_client_node` | Receives RTCM corrections from an NTRIP caster |
| `gps_to_cartesian_node` | Owns the WGS84 map origin and converts GNSS data to local odometry |

The normal data flow is:

```text
GNSS receiver -> serial_node -> nmea0183_parser_node -> gps_to_cartesian_node
                      ^                  |
                      |                  +-> ntrip_client_node (GGA)
                      +------ RTCM corrections -----+
```

Launch remapping connects `serial/line` to `nmea0183/sentence` and
`ntrip/corrections` to `serial/write`. This keeps the serial transport usable
with protocols other than NMEA 0183.

## Interfaces

| Interface | Type | Producer or consumer |
| --- | --- | --- |
| `serial/line` | `std_msgs/String` | Serial input published by `serial_node` |
| `serial/write` | `std_msgs/UInt8MultiArray` | Binary serial output consumed by `serial_node` |
| `nmea0183/sentence` | `std_msgs/String` | NMEA input consumed by parser and NTRIP client |
| `ntrip/corrections` | `std_msgs/UInt8MultiArray` | RTCM output from `ntrip_client_node` |
| `gnss/fix` | `sensor_msgs/NavSatFix` | Position and covariance |
| `gnss/heading` | `geometry_msgs/TwistStamped` | Heading in ROS angular convention |
| `gnss/attitude` | `geometry_msgs/Vector3Stamped` | Roll, pitch and yaw |
| `gnss/imu` | `sensor_msgs/Imu` | Standard attitude representation |
| `gnss/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | Ground velocity |
| `/map_origin` | `sensor_msgs/NavSatFix` | Transient-local WGS84 origin |
| `odometry` | `nav_msgs/Odometry` | Local Cartesian pose and velocity |
| `odom -> base_link` | TF2 | Optional local vehicle transform |

The parser supports GGA, GST, VTG and KindHelm PKHM sentences. Invalid
checksums and unsupported sentences are not published as measurements.

## Parameters

### `serial_node`

| Parameter | Default | Description |
| --- | --- | --- |
| `device` | empty | Serial device path |
| `baud_rate` | `115200` | Port speed |
| `max_write_queue_bytes` | `65536` | Maximum queued binary output |

### `nmea0183_parser_node`

| Parameter | Default | Description |
| --- | --- | --- |
| `message_timeout_ms` | `1500` | Time after which received navigation data is considered stale |

### `ntrip_client_node`

| Parameter | Default | Description |
| --- | --- | --- |
| `enabled` | `false` | Enables caster connection |
| `server` | empty | Caster host name |
| `port` | `2101` | Caster TCP port |
| `mountpoint` | empty | NTRIP mountpoint |
| `protocol_version` | `auto` | NTRIP version selection |
| `username`, `password` | empty | Caster credentials |
| `require_gga` | `true` | Wait for receiver GGA before streaming |
| `gga_interval_s` | `5.0` | GGA forwarding interval |
| `reconnect_delay_s` | `10.0` | Delay after a failed connection |
| `connect_timeout_s` | `10.0` | TCP connection timeout |

### `gps_to_cartesian_node`

| Parameter | Default | Description |
| --- | --- | --- |
| `reference_latitude` | `60.0` | WGS84 map-origin latitude |
| `reference_longitude` | `25.0` | WGS84 map-origin longitude |
| `odom_frame` | `odom` | Local Cartesian frame |
| `base_frame` | `base_link` | Vehicle frame |
| `publish_tf` | `true` | Publish `odom -> base_link` |

Machine-specific devices and NTRIP credentials belong in deployment
configuration. Never commit live caster credentials.
