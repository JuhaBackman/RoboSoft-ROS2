# Simple path tracking

`path_tracking_simple_node` is the lightweight RoboSoft tracking controller.
It combines current and lookahead route curvature as feed-forward and applies
P-type lateral- and heading-error feedback. The implementation is stateless,
small and suitable as a baseline controller or for platforms where predictive
optimization is unnecessary.

The node uses the shared PathTracking subscriptions, watchdogs, lidar speed
limit and `navigation/path_tracking_command` output described in the parent
[navigation documentation](../README.md). Its reusable control law is exported
as `robosoft_core/navigation/path_tracking_simple_controller.hpp`.

The executable is `path_tracking_simple_node`. A robot launch can assign it the
stable runtime name `path_tracking_node` so state machines and downstream
command coordinators remain independent of the selected algorithm.
