# Task management

ISO 11783-10 TASKDATA-subset loading, persistence and runtime task ownership.
RoboSoft reads and writes the subset needed by its mission workflow; this is
not a complete ISOXML parser or a promise of lossless round-trip preservation
for unsupported elements.

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

Supported extension formats are `xml_elements` and `designator_json`. Both are
RoboSoft-specific encodings and are not generally interoperable ISO 11783
exchange formats:

- `xml_elements` writes `RoboSoftExtension` children that are outside the
  ISO 11783-10 schema.
- `designator_json` stores RoboSoft metadata in standard designator attributes;
  the result can exceed the 32-character `PNT.B` or `LSG.B` limit and changes
  the standard field's intended meaning. The writer warns whenever that limit
  is exceeded.

Files containing either RoboSoft extension representation are not claimed to
be schema-conformant. `TASKDATA.XML` contains no RoboSoft extensions and
documents the standard-only subset exercised by the focused tests. The
obsolete bracket-encoded RoboSoft layout is not accepted.

## Supported TASKDATA subset

- task, customer, farm and partfield metadata used by missions
- standard `TSK.G` task status
- products, operation techniques and cultural practices
- guidance allocations with required allocation stamps, guidance groups,
  line strings and route points
- route speed, direction and implement metadata
- lidar landmarks required by localization

The standard mappings relevant to mission serialization include:

- `PNT.E` is a signed integer altitude in millimetres on disk and metres in
  `RoutePoint.altitude`. An absent optional `PNT.E` remains absent when a
  directly loaded `Task` is saved.
- `PFD.A/B/C/D/E/F` represent ID, optional code, required designator, required
  area in square metres, optional customer reference and optional farm
  reference.
- `FRM.H` contains the optional farm country; `FRM.D` is a PO-box field and is
  not interpreted as country.
- Every written `GAN` includes its required `ASP`. Loaded allocation stamps
  are preserved by the internal model. Newly generated allocations use the
  `TaskData.timestamp` converted to UTC, with the Unix epoch representing an
  unset zero timestamp, and effective type `4` for MICS output.

`TaskData` carries `task_id`, `farm_id` and `partfield_id` separately from
their human-readable names. Valid loaded IDs are retained; missing IDs are
generated deterministically as `TSK1`, `FRM1` and `PFD1`.

`TaskData` does not yet expose a separate allocation stamp for every guidance
group. A direct `Task` load/save preserves those per-group values, while a
TASK routed through the ROS message reconstructs allocation starts from the
single task timestamp and writes MICS type `4`. A future interface revision is
needed if per-group allocation history must survive that message boundary.

Complete Device/DeviceElement serialization is not currently supported. A
parsed, unreferenced DVC is omitted on output rather than being written without
mandatory `ClientNAME`, structure/localization labels and a `DET` hierarchy.
Device-referencing TASK content is outside the supported subset and must not be
saved through RoboSoft until complete DVC/DET support is implemented.

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
