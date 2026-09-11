# robosoft_simulator

Reusable hardware-interface and vehicle simulators for testing applications
built with RoboSoft. Simulator components model the boundary outside the
controller under test: vehicle motion, sensor transport, operator controls,
ISOBUS tractor services and implement hardware.

The supplied launch file is an AKI integration example. Components that use
standard ROS interfaces or ISO 11783 are reusable in other application
scenarios; proprietary remote and UVC/LED simulators are enabled only when the
target application needs those protocols. No ROS 2/DDS connection is required
between a simulator computer and an embedded controller computer.

## Components

| Component | Responsibility |
| --- | --- |
| [`vehicle_model_node`](src/vehicle_model/README.md) | Delayed, rate-limited kinematic vehicle response |
| [`tractor_state_simulator_node`](src/tractor_state_simulator/README.md) | Measurements consumed by the ROS2ISOBUS T-ECU server |
| [`gnss_serial_simulator_node`](src/gnss_serial_simulator/README.md) | Delayed NMEA 0183 output over a serial endpoint |
| [`sick_tim5xx_simulator_node`](src/sick_tim5xx_simulator/README.md) | Generic PointCloud2 to TiM5xx Ethernet CoLa B adapter |
| [`task_lidar_cloud_generator_node`](src/task_lidar_cloud_generator/README.md) | TASK obstacle ray casting to PointCloud2 |
| [`remote_simulator_node`](src/remote_simulator/README.md) | Joystick/parameter input to proprietary remote CAN frames |
| [`aki_drive_controller_simulator_node`](src/aki_drive_controller_simulator/README.md) | Example lower-level manual drive controller |
| [`uvc_led_simulator_node`](src/uvc_led_simulator/README.md) | Proprietary treatment and lighting hardware emulator |

[ROS2ISOBUS][ros2isobus] supplies `can_bridge_node`, `address_manager_node` and the
configurable T-ECU server used by the launch scenario. All simulated CAN devices
exchange `IsobusFrame` messages with that single CAN bridge; simulator nodes do
not open the SocketCAN interface themselves.

## Supplied AKI scenario

The repository-level `start_aki_simulator.sh` script is the recommended entry
point for the complete scenario. It prepares the selected physical or virtual
CAN and GNSS transport before invoking `aki_can_simulator.launch.py`. Launch
the ROS file directly when those interfaces are already managed externally.
The script header documents its arguments, environment overrides and resource
cleanup; the operational examples below cover the common test arrangements.

The CAN simulation is divided into independent nodes:

- `vehicle_model_node` provides the local kinematic vehicle model
- `tractor_state_simulator_node` publishes the physical tractor measurements
  consumed by the T-ECU server
- `gnss_serial_simulator_node` converts VehicleModel odometry to NMEA 0183
  GGA/GST/VTG/PKHM sentences on a serial endpoint
- ROS2ISOBUS `tecu_server_node` provides the ISO 11783 T-ECU server
- `remote_simulator_node` emulates physical AKI remote PGNs `0xFF00–0xFF02`
- `aki_drive_controller_simulator_node` emulates the lower-level controller
  from RemoteSimulator's processed local status and joystick topics
- `uvc_led_simulator_node` emulates UVC/LED PGNs `0xFF30–0xFF31`

The TECU receives speed and curvature commands from the embedded AKI over CAN
and sends them locally to VehicleModel as `geometry_msgs/TwistStamped`.
VehicleModel publishes standard `nav_msgs/Odometry`, which TECU converts back
to CAN feedback. The UVC sequence becomes active when its CAN command arrives
and completes after the configured duration.

VehicleModel reproduces the identified AKI response: speed is limited to
`-1.2...2.3 m/s`, curvature to `-0.6...0.6 1/m` (`-600...600 1/km`), and their
rates to `-5.2...5.6 m/s²` and `-0.28...0.28 1/m/s`, respectively. Curvature
uses a 1.4 s first-order time constant; speed uses 4.5 s while accelerating
and 1.35 s while decelerating. A separate 100 ms command delay precedes these
dynamics. With the virtual interfaces, `gnss_serial_simulator_node` delays
vehicle odometry by 100 ms before forming NMEA sentences, keeping command and
position-measurement delays separate. Physical mode sets both configured
transport delays to zero because real CAN and RS-232 already add them. Vehicle
dynamics and limits remain identical in both modes. All values are ROS
parameters.

CAN, GNSS serial and lidar TCP cross between computers. ROS parameters and
the vehicle odometry topics remain local to the simulator computer. The TASK
cloud generator and generic TiM5xx adapter communicate locally through the
standard `sensor_msgs/PointCloud2` `cloud` topic. See the
[lidar Ethernet guide](src/sick_tim5xx_simulator/README.md) and
[TASK cloud generator guide](src/task_lidar_cloud_generator/README.md).

### PS4 controller as the AKI remote

ROS 2's standard `joy/joy_node` reads the Linux joystick device and publishes
`sensor_msgs/Joy`. The simulator launch can start it and route `/joy` to
`remote_simulator_node`, which converts the controls to the same CAN PGNs as
the physical AKI remote. Joystick support is enabled by default; connect the
PS4 controller before starting:

```bash
./start_aki_simulator.sh virtual
```

The default Linux joystick index is zero (`/dev/input/js0`). Select another
controller when needed:

```bash
JOYSTICK_DEVICE_ID=1 \
  ./start_aki_simulator.sh virtual
```

The default PS4 mapping is:

- left and right sticks: remote joystick axes 1 and 2
- R2: remote speed-selector field (not required by manual drive simulation)
- L1: left manual-drive enable
- R1: right manual-drive enable and AUTO start deadman
- Cross (button 0): start
- Triangle (button 2), Circle (button 1) and Square (button 3): modes 1,
  2 and 3
- Share (button 8) or Options (button 9), while held: emergency stop

Button and axis indices are parameters under `remote_simulator_node` in
`config/aki_can_simulator.yaml`. Controller mappings can differ between USB
and Bluetooth. Inspect the actual indices with:

```bash
ros2 run joy joy_enumerate_devices
ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
ros2 topic echo /joy
```

The processed remote convention is positive X to the right and positive Y
up/forward. The four `axis_*_sign` parameters adapt SDL driver directions to
that convention before both CAN and simulator-local messages are published.

When joystick messages stop for more than `joy_timeout_s`, the simulated
remote sets safety to false and releases start and deadman. While a joystick is
connected, safety is true to represent a released emergency stop; it is not
mapped to a momentary controller button. Set `ENABLE_JOYSTICK=false` when the
static remote parameters should be used without a joystick.

`remote_simulator_node` is the only component that maps raw `/joy` axes and
buttons. It sends the resulting state over CAN to RoboSoft and publishes the
same snapshot locally as `simulator/remote/status` and
`simulator/remote/joy`. In MANUAL mode, `aki_drive_controller_simulator_node`
uses those processed topics. Select MANUAL with Triangle. Hold either L1 or R1
when beginning joystick motion; the enable button may then be released while
the joystick remains displaced. The left stick Y axis controls speed and the
right stick X axis controls curvature. Returning both control axes to center
stops and disarms manual drive, so the next start again requires L1 or R1.
RoboSoft is not part of this manual command path. In AUTO modes, VehicleModel
instead uses the command received through the T-ECU server.

The lower-level simulation has its own safety timeout. Missing processed status
or joystick messages disable manual drive and publish a zero command. Maximum
speed, curvature, deadzone and steering direction are configurable under
`aki_drive_controller_simulator_node`; all device mappings
remain under `remote_simulator_node`.

### Simulator computer isolation

The launch starts the complete simulator-side stack:

- ROS2ISOBUS `can_bridge_node`
- ROS2ISOBUS `address_manager_node`
- ROS2ISOBUS `tecu_server_node`
- RoboSoft vehicle, remote-control and UVC/LED simulator nodes
- RoboSoft tractor-state measurement simulator

All processes started by the launch use simulator-only `ROS_DOMAIN_ID=42` and
`ROS_LOCALHOST_ONLY=1`. The robot computer can therefore use another ROS
domain and no DDS traffic is exchanged between the computers. The physical
CAN bus is their only connection.

Change the local simulator domain when necessary with `domain_id`; do not use
the robot computer's domain. The setting applies only to processes started by
the launch and does not modify the parent shell.

### T-ECU protocol scope

ROS2ISOBUS `tecu_server_node` uses the primary Tractor ECU NAME (function 134,
function instance 0), claims its address and resolves address conflicts. It
implements these ISO 11783-7/9 messages:

- Request and Address Claimed (`0xEA00`, `0xEE00`)
- Required Tractor Facilities and Tractor Facilities Response (`0xFE08`,
  `0xFE09`), including the mandatory response at startup
- Wheel- and ground-based speed/distance (`0xFE48`, `0xFE49`)
- Agricultural Guidance Machine Info and Guidance System Command (`0xAC00`,
  `0xAD00`)
- Class 3 cruise, rear hitch, rear PTO and auxiliary-valve status/commands

The server is selected as Class 1, Class 2 or Class 3 with `tecu_class`.
Class 3 features such as guidance, cruise, hitch, PTO and the number of
auxiliary valves are configured independently in
`config/aki_can_simulator.yaml`.

Guidance and speed commands require a 100 ms heartbeat and expire after about
300 ms without a new command. ISO curvature is converted
from km^-1, positive right, to the ROS convention 1/m, positive left. The
facilities response advertises Class 3 support.

VehicleModel supplies measured motion through the remapped `vehicle/twist`
topic. `tractor_state_simulator_node` publishes engine speed, remaining power
time, key-switch state, guidance readiness, mechanical lockout, rear hitch,
rear PTO and AUX-valve states to `ISOBUS/tecu/server/inputs/*`. Its default
10 Hz rate is faster than the T-ECU server's one-second input timeout.

The simulated values are configured under `tractor_state_simulator_node` in
`config/aki_can_simulator.yaml`. If this node is stopped, the corresponding
CAN signals automatically return to the ISO 11783 `not available` values
after `input_timeout_ms`.

Address claiming and raw CAN access are provided by the ROS2ISOBUS
`address_manager_node` and `can_bridge_node`; the T-ECU server itself uses only
the package's bus and address-manager ROS interfaces.

### Local test with virtual CAN

Create `vcan0` once:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Build and start the simulator:

```bash
colcon build --packages-up-to robosoft_simulator
source install/setup.bash
ros2 launch robosoft_simulator aki_can_simulator.launch.py \
  can_interface:=vcan0 domain_id:=42
```

Start the simulator and AKI controller in separate terminals. The first
argument selects the interface implementation explicitly:

```bash
./start_aki_simulator.sh virtual
./start_aki_gui.sh virtual
```

On a one-computer vcan test, `start_aki_simulator.sh` creates and owns the
virtual serial pair automatically with `socat`:

```text
/tmp/aki_gnss_sim <-> /tmp/aki_gnss_robot
```

The simulator writes to `/tmp/aki_gnss_sim`, while AKI's generic `serial_node`
opens `/tmp/aki_gnss_robot`. The pair is removed when the simulator stops, so
stale pseudo-terminal links cannot be reused on the next run. If AKI is started
first, its serial node reconnects when the simulator creates the pair. Install
`socat` once when necessary:

```bash
sudo apt install socat
```

Run the AKI stack against the same virtual bus:

```bash
ros2 launch robosoft_aki aki_nodes.launch.py \
  can_interface:=vcan0 \
  tecu_server_name:=01006007008600A0 \
  tecu_server_address:=240 \
  serial_device:=/tmp/aki_gnss_robot
```

For physical interfaces, `physical`, `can0` and the real serial devices are
the defaults. Start the simulator computer with:

```bash
./start_aki_simulator.sh
# equivalent to:
./start_aki_simulator.sh physical can0 /dev/ttyUSB0
```

On the AKI controller computer:

```bash
./start_aki_gui.sh
# equivalent to:
./start_aki_gui.sh physical can0
```

The simulator script's third argument is its local GNSS serial device. AKI
uses `/dev/ttymxc2` from `aki_params.yaml` by default; override it with, for
example, `./start_aki_gui.sh physical can0 serial_device:=/dev/ttyUSB1`.
Connect TX/RX/GND between the two physical serial adapters. Device names may
be different on the computers.

The same scripts work on one or two computers. Two independent virtual CAN
interfaces do not communicate across computers without a separately
configured CAN-over-IP tunnel, so normal two-computer operation uses
`physical`.

Run AKI on the robot computer in its own ROS domain. Connect CAN-H, CAN-L and
ground with correct termination, and configure both CAN interfaces for the
same bitrate, for example:

```bash
sudo ip link set can0 up type can bitrate 250000
```

To control simulator parameters from another terminal on the simulator
computer, use the same local domain and localhost setting:

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  ros2 param set /remote_simulator_node mode 3
```

### Remote operation

The default is safe manual mode (`mode=1`). Parameters can be changed while
the simulator runs:

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ros2 param set /remote_simulator_node mode 3
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ros2 param set /remote_simulator_node start true
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ros2 param set /remote_simulator_node start false
```

Modes use the physical AKI remote numbering `0..6`. Device-loss tests can be
performed by disabling an emulator:

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ros2 param set /remote_simulator_node enabled false
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ros2 param set /uvc_led_simulator_node enabled false
```

## Build and test

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to robosoft_simulator
source install/setup.bash
colcon test --packages-select robosoft_simulator
colcon test-result --verbose
```

Simulator tests cover CAN codec handling, lidar map/ray geometry, the real SICK
driver integration and the configurable vehicle dynamics.
Application-level validation should additionally exercise startup dependencies,
message timeouts, emergency stops, task execution and recovery after simulated
device loss.

## Extending the simulator

Add a separate component when a new sensor or hardware protocol must cross the
controller boundary. Prefer standard ROS messages between simulator components
and keep protocol encoding in the component that owns the emulated transport.
Application-specific emulators should be optional launch actions so generic
vehicle, GNSS and tractor-service simulation remains reusable.

## Package structure

```text
robosoft_simulator/
├── include/robosoft_simulator/       # Shared protocol utilities
├── src/
│   ├── aki_drive_controller_simulator/  # Node, implementation and README
│   ├── gnss_serial_simulator/           # Node, implementation and README
│   ├── remote_simulator/                # Node, implementation and README
│   ├── sick_tim5xx_simulator/            # Generic PointCloud2-to-Ethernet adapter
│   ├── task_lidar_cloud_generator/      # TASK-map PointCloud2 source
│   ├── tractor_state_simulator/         # Node, implementation and README
│   ├── uvc_led_simulator/               # Node, implementation and README
│   └── vehicle_model/                   # Node, implementation and README
├── config/
├── launch/
└── test/
```

Scenario control and visualization can be added
as separate simulator components in later simulation stages. The repository
does not contain empty placeholder components.

## Safety

Simulation demonstrates software behaviour but does not validate physical
stopping distance, actuator polarity, bus wiring, emergency-stop circuits or
machine hazards. Repeat safety tests with restrained hardware and the target
platform's approved procedure before enabling motion.

## License

GPL-3.0-only. See the repository [LICENSE](../LICENSE). ROS2ISOBUS retains its
own license.

[ros2isobus]: https://github.com/AGRIForward/ROS2ISOBUS
