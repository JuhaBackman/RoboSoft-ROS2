# Nonlinear model predictive path tracking

`path_tracking_nmpc_node` uses a nonlinear model-predictive controller to
optimize speed- and curvature-command rates along the active TASK route. It
shares its ROS interfaces and safety gating with the simple controller, so an
application can select either controller without changing downstream command
wiring.

The checked-in solver has a 4.0-second prediction horizon divided into 40
intervals, resulting in a 0.1-second integration step. The node publishes its
100 ms callback duration, scheduling margin and deadline-miss count on
`/performance/path_tracking` for runtime monitoring.

## VIATOC-generated solver

[VIATOC](https://sourceforge.net/p/viatoc/wiki/Home/) is a toolkit for defining
nonlinear optimal-control problems in C++ and generating C or C++ controller
source code. RoboSoft uses VIATOC at development time to generate the NMPC
solver. A normal RoboSoft build compiles the generated sources already stored
under `generated/`; installing VIATOC on the target controller is therefore not
required.

VIATOC and its gradient-projection and code-generation approach are described
in the following peer-reviewed publication:

> Jouko Kalmari, Juha Backman and Arto Visala, “A toolkit for nonlinear model
> predictive control using gradient projection and code generation,” *Control
> Engineering Practice*, vol. 39, pp. 56–66, 2015, ISSN 0967-0661.
> [DOI: 10.1016/j.conengprac.2015.01.002](https://doi.org/10.1016/j.conengprac.2015.01.002)
> ([ScienceDirect](https://www.sciencedirect.com/science/article/pii/S096706611500012X)).

The generated controller solves a nonlinear finite-horizon optimization at
each control cycle and applies the first optimized control values. VIATOC uses
a gradient-projection method with Barzilai–Borwein-type step-length selection
and produces self-contained code suitable for controllers with constrained
runtime resources.

The optimal-control problem is defined in
[`generate_path_tracking_nmpc.cpp`](https://github.com/JuhaBackman/RoboSoft-ROS2/blob/main/robosoft_core/tools/generate_path_tracking_nmpc.cpp).
That program declares the prediction states, controls, differential equations,
cost dimensions, constraints, horizon and RK4 integration method. The
RoboSoft runtime wrapper supplies machine-specific numeric parameters and route
references to the generated solver.

## Vehicle and actuator model

The prediction uses a planar curvature-controlled kinematic vehicle model with
first-order actuator response and command-delay approximations. Its state
vector is:

```text
x = [X, Y, psi, v, kappa, v_delayed, kappa_delayed, v_command, kappa_command]
```

where `X` and `Y` are the local-map coordinates of the vehicle reference point,
`psi` is yaw, `v` is signed longitudinal speed and `kappa` is path curvature.
The delayed states approximate command transport, while the command states are
the values requested from the lower-level controller.

The optimized control vector contains the requested command rates:

```text
u = [d(v_command)/dt, d(kappa_command)/dt]
```

The continuous prediction equations are:

```text
dX/dt                 = v cos(psi)
dY/dt                 = v sin(psi)
dpsi/dt               = v kappa
dv/dt                 = a_limit tanh((v_delayed - v) /
                                      (tau_v a_limit))
dkappa/dt             = kappa_rate_limit
                        tanh((kappa_delayed - kappa) /
                             (tau_kappa kappa_rate_limit))
dv_delayed/dt         = (v_command - v_delayed) / tau_delay
dkappa_delayed/dt     = (kappa_command - kappa_delayed) / tau_delay
dv_command/dt         = u_v
dkappa_command/dt     = u_kappa
```

The hyperbolic tangent gives a smooth first-order response whose derivative is
bounded by the configured acceleration or curvature-rate capability. The
runtime wrapper selects the acceleration or deceleration response constant and
rate limit from the direction of the current speed transition. Curvature-rate
constraints can be asymmetric even though the response equation uses the
larger configured magnitude for its smooth saturation.

This model makes the following assumptions:

- Motion is planar and the odometry reference point is the rear-axle centre.
- Longitudinal and lateral tyre slip, roll, pitch and wheel dynamics are not
  modelled.
- Steering is represented directly by curvature, so wheelbase and steering
  angle do not appear in the generated model.
- Command transport is approximated by a first-order state, not an exact
  discrete pure delay.
- Signed speed supports both forward and reverse TASK route segments.

At the beginning of an optimization, `X`, `Y`, `psi` and `v` come from local
odometry. Measured curvature is calculated as yaw rate divided by speed. At
very low speed, where that ratio is not reliable, the previous curvature
command is used. Delayed and requested command states are initialized from the
previous controller output, and the previous control sequence is shifted to
warm-start the next solution.

## Route reference and objective

The node receives the complete active `GuidanceLine` from
`route_control/current_route`, converts its WGS84 points with `/map_origin`, and
uses local `odometry` as the prediction's initial state. The
`route_control/status.segment_index` is the first candidate segment; it already
identifies the segment whose start is the previously passed route point. For
each predicted `(x,y)` position, the controller projects onto a bounded set of
non-earlier route segments and interpolates an `(x,y,yaw,v,k)` reference. The
search advances monotonically, is limited by route arc distance, and rejects
segments whose heading conflicts with the predicted yaw. This prevents a
self-crossing route from selecting a geometrically nearby but later branch.
After optimization these projections are rebuilt from the new prediction. This
outer reference update is repeated `reference_update_iterations` times.

For each prediction interval, the solver minimizes weighted errors in
`(X, Y, psi, v, kappa)` relative to the projected route. The delayed and
requested command states are also driven toward the route speed and curvature.
The reference command rates are zero, so the `speed_rate_weight` and
`curvature_rate_weight` terms penalize abrupt changes. A separately weighted
terminal state error encourages the end of the horizon to remain aligned with
the route. In compact form, the objective is:

```text
J = sum((x - x_ref)' Q (x - x_ref) + u' R u)
    + (x_terminal - x_ref_terminal)' P
      (x_terminal - x_ref_terminal)
```

`position_weight` is applied independently to map-frame X and Y errors;
`heading_weight`, `speed_weight` and `curvature_weight` control the other
physical-state errors. `command_state_weight` applies to all four delayed and
requested command states. `P` uses the state weights multiplied by
`terminal_weight_multiplier`.

The generated solver uses RK4 integration and is warm-started from the previous
solution. Only the first optimized command is applied before the problem is
solved again at the next 100 ms callback. The output
`geometry_msgs/TwistStamped.angular.z` is formed as `speed * curvature`.
Invalid solver output, stale inputs, an incompatible robot state or invalid
enabled-lidar status produces a zero command.

Each control callback uses the model as follows:

1. Read the latest odometry, route, route segment index, robot state and safety
   status.
2. Initialize the measured model states and shift the previous optimized
   control sequence forward by one interval.
3. Simulate the model over the horizon with the current control sequence.
4. Project every predicted pose onto a bounded, monotonically advancing part of
   the route and build the corresponding state-reference sequence.
5. Optimize the command-rate sequence and repeat the model rollout and route
   projection `reference_update_iterations` times.
6. Integrate the first optimized command rates for one interval, publish the
   resulting speed and curvature command, and retain the solution as the next
   callback's warm start.

## Runtime parameters

No parameter below requires VIATOC regeneration:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `speed_acceleration_response_s` | `4.5` | Identified acceleration time constant |
| `speed_deceleration_response_s` | `1.35` | Identified deceleration time constant |
| `curvature_response_s` | `1.4` | Identified curvature time constant |
| `command_delay_s` | `0.1` | Command transport-delay approximation |
| `minimum_speed_m_s`, `maximum_speed_m_s` | `-1.2`, `2.3` | Speed state and command limits |
| `minimum_tracking_speed_m_s` | `0.0` | Minimum signed command magnitude while the active TASK speed is nonzero; safety gating can still command zero |
| `minimum_acceleration_m_s2`, `maximum_acceleration_m_s2` | `-5.2`, `5.6` | Lower/upper speed-response and optimized command-rate limits |
| `minimum_curvature_m_inv`, `maximum_curvature_m_inv` | `-0.6`, `0.6` | Curvature state and command limits |
| `minimum_curvature_rate_m_inv_s`, `maximum_curvature_rate_m_inv_s` | `-0.28`, `0.28` | Lower/upper optimized curvature command-rate limits and response saturation |
| `position_weight` | `8.0` | Per-axis map-position objective weight |
| `heading_weight` | `25.0` | Yaw objective weight |
| `speed_weight` | `4.0` | Speed-state objective weight |
| `curvature_weight` | `12.0` | Curvature-state objective weight |
| `command_state_weight` | `0.2` | Delayed and requested command-state weight |
| `speed_rate_weight` | `4.0` | Optimized `dV` objective weight |
| `curvature_rate_weight` | `12.0` | Optimized `dk` objective weight |
| `terminal_weight_multiplier` | `3.0` | Terminal/state weight ratio |
| `initial_reference_search_distance_m` | `3.0` | Forward search from RouteControl's current segment |
| `reference_step_search_distance_m` | `2.0` | Forward search between consecutive prediction references |
| `reference_heading_tolerance_rad` | `1.5708` | Maximum yaw difference accepted during route association |
| `reference_update_iterations` | `3` | Route reprojections per control update |
| `solver_iterations` | `8` | Optimization iterations per route reprojection |

The common state, topic, lidar and timeout parameters are listed in the parent
[navigation documentation](../README.md).

### Runtime values versus generated values

The generator contains numeric values for model parameters, bounds and
objective weights because the exported solver must be usable as complete C++
code. These are fallback values, not a second robot configuration. On startup,
`path_tracking_nmpc_node` reads the robot's ROS parameters and
`PathTrackingNmpcController` writes them into the generated solver before its
first optimization:

| Generated storage | Runtime source | Values replaced at runtime |
| --- | --- | --- |
| `p[]` | Robot YAML/node parameters | Response time constants, command delay and response-rate parameters |
| `xMin[]`, `xMax[]` | Robot YAML/node parameters | Speed and curvature limits for physical, delayed and requested states |
| `uMin[]`, `uMax[]` | Robot YAML/node parameters | `dV` acceleration and `dk` curvature-rate limits |
| `Q`, `R`, `P` | Robot YAML/node parameters | State, control-rate and terminal objective weights |

Consequently, vehicle identification values, existing minimum/maximum limits,
objective weights, `solver_iterations` and `reference_update_iterations` must
be changed in the robot's YAML configuration. They do not require rebuilding
the generated solver. For example, AKI supplies its values in
`robosoft_aki/config/aki_params.yaml`; another robot should supply its own
parameter file instead of editing `robosoft_core`.

The generated fallback values should still be kept consistent with the public
C++ and node defaults so that direct library use remains predictable. Editing
only a fallback and regenerating does not override a value supplied by YAML.

### Changes requiring solver regeneration

Regenerate the VIATOC solver when changing any of the following:

- prediction states or their ordering;
- optimized controls or their ordering;
- differential equations or the model structure;
- number or ordering of VIATOC `Parameter` expressions;
- objective formulation or matrix dimensions;
- which states or controls have generated constraints;
- prediction horizon length or discretization interval count;
- integrator type, currently RK4.

The current generated horizon is 4.0 seconds with 40 intervals, giving a
0.1-second integration step. This interval count is unrelated to the runtime
`solver_iterations` parameter. Adding or removing a constraint requires
regeneration, but changing the numeric limit of an existing speed, curvature,
`dV` or `dk` constraint only requires a YAML change because the wrapper updates
the corresponding generated array.

## Generated code

Normal RoboSoft builds compile the checked-in files under `generated/` and do
not require VIATOC. The generator definition is the linked
[`generate_path_tracking_nmpc.cpp`](https://github.com/JuhaBackman/RoboSoft-ROS2/blob/main/robosoft_core/tools/generate_path_tracking_nmpc.cpp).
To intentionally change the horizon, state structure or equations, obtain and
build [VIATOC](https://sourceforge.net/p/viatoc/wiki/Home/), compile that
generator against `libVIATOC`, and run it with `generated/` as its output
directory.

VIATOC model parameters are represented with its `Parameter` expressions and
are exposed through the generated solver's runtime `p` array. The ROS wrapper
sets this array, constraints and tunable weighting matrices from parameters at
startup. The copied VIATOC runtime retains its LGPL-3.0-or-later notice and
license in `generated/COPYING.LESSER`.

The VIATOC snapshot used for this solver has an argument-order typo in its C++
header exporter. Apply the included compatibility patch before regeneration:

```bash
patch -d "${VIATOC_ROOT}" -p1 \
  < robosoft_core/tools/viatoc_cpp_export_signature.patch
cmake -S "${VIATOC_ROOT}" -B /tmp/viatoc-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/viatoc-build --target VIATOC
g++ -std=c++17 -I"${VIATOC_ROOT}/include" \
  robosoft_core/tools/generate_path_tracking_nmpc.cpp \
  /tmp/viatoc-build/libVIATOC.a -o /tmp/generate_path_tracking_nmpc
/tmp/generate_path_tracking_nmpc \
  robosoft_core/src/navigation/path_tracking_nmpc/generated
```

After generation, copy `nmpc`, `solver` and `projection` `.cpp`/`.h` files and
`COPYING.LESSER` from VIATOC's `src/solver_cpp` and root directory into the
generated directory. Do not hand-edit the generated solver sources.
