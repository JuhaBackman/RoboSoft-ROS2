# Task management

ISO 11783-10 TASKDATA loading, persistence and runtime task ownership.

## Components

| Node or library | Responsibility |
| --- | --- |
| `task_manager_node` | Owns the current task, services and recorded-route storage |
| `task_xml` | Converts between TASK XML and `robosoft_interfaces/TaskData` |
| `task_path` | Resolves task names safely under the configured directory |

## Interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `task_manager/task_loaded` | `TaskData` | Reliable transient-local current-task snapshot |
| `task_manager/select_task` | `SelectTask` | Load an XML task by filename |
| `task_manager/save_task` | `SaveTask` | Save the current task under a filename |
| `task_manager/modify_status` | `ModifyTaskStatus` | Change ISO task status and active guidance index |
| `task_manager/update_lidar_clusters` | `UpdateLidarClusters` | Store mapped lidar landmarks |
| `route_control/recorded_route` | `GuidanceLine` | Completed route appended to the current task |

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `task_directory` | `/opt/robosoft/tasks` | Base directory for relative task filenames |
| `autosave_recorded_routes` | `true` | Persist recorded routes automatically when possible |
| `task_extension_format` | `xml_elements` | Encoding for fields without an ISO 11783 representation |

Supported extension formats are `xml_elements` and `designator_json`.
Standard ISO 11783 fields are always used when they can represent the value.
`designator_json` serializes only additional RoboSoft data into designator
strings. The obsolete pre-standard RoboSoft XML layout is not accepted.

## Supported TASKDATA subset

- task, customer, farm and partfield metadata
- standard `TSK.G` task status
- products, devices, operation techniques and cultural practices
- guidance allocations, shifts, line strings and route points
- route speed, direction and implement metadata
- lidar landmarks required by localization

Manual recording and automatic execution both use the standard
`TSK.G=RUNNING` value. `RobotState.substate` identifies whether the robot is
recording or executing the task.

## Task files

Development tasks and the format example are in the package-level
[`tasks`](../../tasks) directory and are installed to
`share/robosoft_core/tasks`:

- `EMPTY_TASK.XML` is the template for a new manual recording.
- `TASKDATA.XML` documents the supported ISO 11783 structure.
- Other XML files are development tasks created through the GUI.

The robot configurations set `task_directory` to the writable runtime path
`/opt/ros2_ws/tasks`. Deployment scripts may seed that directory from the example
files installed under the `robosoft_core` package, but must not overwrite
tasks recorded or edited on the robot.
