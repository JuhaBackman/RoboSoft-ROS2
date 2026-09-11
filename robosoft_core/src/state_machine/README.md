# Robot main state machine

`RobotMainNodeBase` is the shared C++ foundation for robot-level RoboSoft
state machines. It is a library rather than a separately launched ROS node.
An application derives its main coordination node from the class, so that
process owns the platform-specific coordination and safety inputs.

## Shared responsibilities

- `INIT`, `MAIN` and `EXIT` state identifiers
- `INIT_WAIT_NODES`, `INIT_WAIT_DATA`, `MAIN_WAIT`, `MAIN_MANUAL`,
  `MAIN_MANUAL_RECORD`, `MAIN_AUTO` and `MAIN_STOP` substates
- transient-local `task_manager/task_loaded` subscription
- validated `task_manager/modify_status` requests
- required-node discovery during the first INIT stage
- state-transition logging and `robot_state/status` publication

The base class validates the active guidance index before a status request. An
absent or stale index is sent as `-1`, allowing manual recording while
preventing an invalid route selection.

## Application responsibilities

Derived nodes implement state-entry and state-exit actions and retain their
own periodic transition guards. This keeps operator controls, vehicle buses,
localization, implements and emergency-stop semantics in the application
where their complete context and fail-safe behaviour can be validated.

New applications should derive their main node from
`robosoft_core::RobotMainNodeBase`, use the shared task and transition methods,
and document every hardware condition that permits automatic operation.
