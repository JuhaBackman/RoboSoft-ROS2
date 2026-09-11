RoboSoft – ROS 2 control software
=================================

RoboSoft is a reusable ROS 2 control framework for mobile robots and
agricultural machinery. It provides ISO 11783 TASK-based route management,
localization, path tracking, safety coordination, GNSS connectivity and an
optional Qt 6 operator interface. Robot applications select the shared
components they need and add machine-specific state guards and hardware
adapters.

RoboSoft uses ISO 11783 for both machine communication and robot task
descriptions. Communication is provided by the separate
[ROS2ISOBUS](https://github.com/AGRIForward/ROS2ISOBUS) package. RoboSoft
implements the loading, processing and saving of ISO 11783-10 TASK data and
uses that structure to describe the routes and operations assigned to a robot.

The repository includes AKI as an example application and a hardware-interface
simulator for development and integration testing.

Background
----------

RoboSoft ROS 2 is a port and refactor of the legacy RoboSoft robot-control
system. In the earlier implementation, distributed software components were
connected using Qt's signal/slot mechanism. The current implementation uses
ROS 2 nodes, topics, services and standard message types to provide explicit
interfaces between reusable control components and robot-specific software.
The ROS 2 adaptation and refactor were assisted by OpenAI Codex/ChatGPT.
RoboSoft ROS 2 does not include every function available in the legacy
RoboSoft system. In particular, the legacy system supported the ISO 5231
Extended farm management information systems data interface (EFDI) for
communication between a robot and a back-office system. EFDI support in
RoboSoft ROS 2 remains under development for a future release.

The related [ROS2ISOBUS](https://github.com/AGRIForward/ROS2ISOBUS) project is
based on legacy ISOBUS research code used in earlier projects. Its ROS 2
adaptation and refactor were also assisted by OpenAI Codex/ChatGPT. The ISOBUS
components and the legacy RoboSoft control system were used, for example, in
the robot described in
[this IFAC publication](https://doi.org/10.1016/j.ifacol.2022.11.106).

**Research and testing only:** this software is not a certified machine-safety
system. Validate every safety function and command path for the target machine
before enabling physical motion or an implement.

- Authors: Juha Backman et al. (Luonnonvarakeskus / Natural Resources Institute Finland)
- Contact: juha.backman@luke.fi
- License: GPL-3.0-only (see [LICENSE](LICENSE))
- Change history: [CHANGELOG.md](CHANGELOG.md)

Overview
--------

- **RoboSoft interfaces:** shared robot-state, TASK, route, implement, safety
  and service contracts. Docs:
  [robosoft_interfaces/README.md](robosoft_interfaces/README.md)
- **RoboSoft Core:** reusable TASK, navigation, localization, GNSS, safety,
  state-machine and Qt/QML GUI components. Docs:
  [robosoft_core/README.md](robosoft_core/README.md)
- **AKI example:** field-robot state machine, physical remote control and
  UVC/LED integration. Docs: [robosoft_aki/README.md](robosoft_aki/README.md)
- **Simulator:** vehicle, CAN-device, GNSS and SICK TiM5xx simulation for
  hardware-boundary testing. Docs:
  [robosoft_simulator/README.md](robosoft_simulator/README.md)

Research platforms
------------------

RoboSoft has been used with agricultural research robots at the Natural
Resources Institute Finland's
[Roboverstas](https://www.luke.fi/en/services/roboverstas-autonomous-machinery-and-data-collection-systems-in-food-production-environments).
The research environment includes a robotized ISOBUS-compatible tractor,
field-robot platforms and an electric robot for tunnels and greenhouses.

<img src="robosoft_aki/docs/images/AKI.jpg"
     alt="AKI field robot operating in a strawberry tunnel" width="600">

*AKI field robot in a strawberry production environment.*

Dependencies
------------

- ROS 2 Jazzy and a C++17 toolchain
- [ROS2ISOBUS](https://github.com/AGRIForward/ROS2ISOBUS) for SocketCAN,
  ISO 11783 address management, T-ECU and NMEA 2000 communication
- [VIATOC](https://sourceforge.net/p/viatoc/wiki/Home/) as a development-time
  dependency for generating the NMPC path-tracking solver; the checked-in
  generated solver does not require VIATOC during normal builds or at runtime
- Qt 6 Quick/QML for the optional operator interface
- `sick_scan_xd` for the AKI SICK TiM5xx lidar integration
- Linux SocketCAN for physical or virtual CAN communication

ROS2ISOBUS is a separate ROS 2 package. Place it in the same workspace before
building packages that use ISOBUS interfaces. Optional and platform-specific
dependencies are described in the package documentation.

Building
--------

Create an empty ROS 2 workspace and clone RoboSoft as its `src` directory.
Then clone ROS2ISOBUS inside that directory as a separate Git repository:

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws

git clone \
  https://github.com/JuhaBackman/RoboSoft-ROS2.git src
git clone \
  https://github.com/AGRIForward/ROS2ISOBUS.git src/Ros2ISOBUS
```

The directory layout before the first build should be:

```text
~/ros2_ws/
└── src/                         RoboSoft repository
    ├── .git/
    ├── README.md
    ├── robosoft_interfaces/
    ├── robosoft_core/
    ├── robosoft_aki/
    ├── robosoft_simulator/
    ├── start_aki_gui.sh
    ├── start_aki_simulator.sh
    └── Ros2ISOBUS/              separate ROS2ISOBUS repository
        ├── .git/
        ├── package.xml
        └── src/
```

The nested ROS2ISOBUS working tree is excluded by RoboSoft's `.gitignore`, so
the repositories retain independent histories and remotes. Do not add
ROS2ISOBUS files to the RoboSoft repository.

Install dependencies and build from the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

After the build, `build/`, `install/` and `log/` are created next to `src/`
under `~/ros2_ws`. The startup scripts locate this workspace through their
position in `~/ros2_ws/src`.

Qt can be disabled for headless builds with
`-DBUILD_ROBOSOFT_GUI=OFF`. Package-specific configuration, launch and
hardware notes are provided in each package README.

Running the examples
--------------------

AKI and its simulator can be started in separate terminals. Both scripts use
physical interfaces by default. Run them from the RoboSoft repository:

```bash
cd ~/ros2_ws/src
./start_aki_simulator.sh physical
./start_aki_gui.sh physical
```

For a one-computer setup using virtual CAN and a pseudo-terminal GNSS link:

```bash
cd ~/ros2_ws/src
./start_aki_simulator.sh virtual
./start_aki_gui.sh virtual
```

The AKI application has also been deployed on an
[Epec 6807 Display Unit](https://epec.fi/epec-oy-products/displays/display-unit-6807/),
with the RoboSoft control stack and Qt operator interface running inside a
Docker container on the embedded display. See the AKI documentation for the
runtime layout and startup procedure. Questions about reproducing this
deployment can be sent to juha.backman@luke.fi.

See the [AKI documentation](robosoft_aki/README.md) for application-specific
startup requirements. Development TASK files and the ISO 11783 TASKDATA
example are located in `robosoft_core/tasks`. Deployments should set
`task_directory` to a writable machine-specific location.

Navigation interfaces
---------------------

RoboSoft route control retains ISO 11783 TASK semantics while exposing common
ROS navigation types. Active routes use `nav_msgs/Path`, localization uses
`nav_msgs/Odometry` and the `odom -> base_link` transform, and motion commands
use `geometry_msgs/TwistStamped`.

The supplied controllers are optional at the ROS boundary. A user may connect
another controller, including Nav2, by consuming the route and localization
interfaces and publishing the expected motion command. Nav2 is not a RoboSoft
runtime dependency and requires a user-provided `FollowPath` adapter and Nav2
configuration. The detailed contract is documented in
[robosoft_core/src/navigation/README.md](robosoft_core/src/navigation/README.md).

Testing
-------

```bash
colcon test --packages-select \
  robosoft_interfaces robosoft_core robosoft_aki robosoft_simulator
colcon test-result --verbose
```

The automated tests cover TASK conversion, NMEA parsing, path control,
localization adapters, safety logic and application-specific CAN protocols.
Physical machine, steering, lidar and implement integrations require separate
machine-specific validation.

Safety
------

Start integration with simulated CAN. Before physical operation, verify at
minimum emergency stops, watchdogs, source addresses, actuator directions,
speed and curvature limits, GNSS frames, lidar health handling and implement
safe states. Initial powered tests should prevent unintended propulsion by
mechanical or electrical means appropriate to the machine.

Contributing
------------

Contributions are welcome. Keep reusable functionality in `robosoft_core`, use
standard ROS interfaces where they preserve the required semantics, and keep
machine-specific protocols in their application packages. Submit changes with
clear build and test notes.

License
-------

RoboSoft is licensed under GPL-3.0-only. Dependencies, generated solver code
and external drivers retain their own licenses; see their source notices and
project documentation.
