# Changelog

All notable public-facing changes to this project are documented in this file.

## Unreleased

- Split localization into independently replaceable object-detection,
  landmark-association and Extended Kalman Filter ROS 2 nodes.
- Added sensor-independent landmark detection and association interfaces.
- Moved lidar localization and safety geometry to TF2 vehicle-frame
  transforms, with AKI safety boundaries expressed from `base_link`.
- Disabled the SICK driver's duplicate dynamic TF and suppressed stale
  pre-TASK landmark-estimate feedback.

## 2026-09-11 – Initial public release

- Added the reusable RoboSoft ROS 2 control framework with ISO 11783-10 TASK
  loading, saving, route recording and task execution.
- Added shared GNSS, localization, safety, route-control, path-tracking and
  Qt/QML operator-interface components.
- Added the AKI field-robot application with physical remote, UVC/LED, lidar,
  ROS2ISOBUS T-ECU and NMEA 2000 integration.
- Added hardware-boundary simulation for vehicle dynamics, CAN devices, GNSS
  serial communication and SICK TiM5xx lidar.
- Added lightweight and VIATOC-generated NMPC path-tracking controllers.
