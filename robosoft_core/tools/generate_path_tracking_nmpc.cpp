// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

// This program is intentionally not part of the normal ROS 2 build. It uses
// VIATOC to regenerate the solver checked into path_tracking_nmpc/generated.

#include <mpcexport.h>

#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
  if (argc != 2) {
    std::cerr << "Usage: generate_path_tracking_nmpc OUTPUT_DIRECTORY\n";
    return 1;
  }

  // Vehicle pose and motion in the local map frame.
  DifferentialState position_x;
  DifferentialState position_y;
  DifferentialState yaw;
  DifferentialState speed;
  DifferentialState curvature;

  // First-order command-delay states approximate the identified transport
  // delay while keeping all physical values runtime configurable.
  DifferentialState delayed_speed_command;
  DifferentialState delayed_curvature_command;
  DifferentialState speed_command;
  DifferentialState curvature_command;

  // Optimized command rates. Integrating these produces the speed and
  // curvature commands sent to the lower-level controller.
  Control speed_command_rate;
  Control curvature_command_rate;

  // Runtime parameters, in this stable index order:
  // 0 speed response, 1 curvature response, 2 command delay,
  // 3 speed response rate and 4 curvature response rate.
  //
  // The numeric values below are generated fallback values only. During a
  // normal ROS 2 run PathTrackingNmpcController replaces the generated p[]
  // values with node parameters, normally loaded from the robot's YAML file.
  // Tune those runtime values in YAML without regenerating; editing the
  // fallbacks here only affects a subsequently generated standalone solver.
  // Adding, removing or reordering Parameter objects does require regeneration
  // and a matching update to ParameterIndex in the runtime controller.
  Parameter speed_response_s(4.5);
  Parameter curvature_response_s(1.4);
  Parameter command_delay_s(0.1);
  Parameter speed_rate_m_s2(5.6);
  Parameter curvature_rate_m_inv_s(0.28);

  // Changing the state set, control set or any equation below changes the
  // generated solver structure and therefore requires regeneration.
  DifferentialEquation dynamics;
  dynamics << dot(position_x) == speed * cos(yaw);
  dynamics << dot(position_y) == speed * sin(yaw);
  dynamics << dot(yaw) == speed * curvature;
  dynamics << dot(speed) == speed_rate_m_s2 * tanh(
    (delayed_speed_command - speed) /
    (speed_response_s * speed_rate_m_s2));
  dynamics << dot(curvature) == curvature_rate_m_inv_s * tanh(
    (delayed_curvature_command - curvature) /
    (curvature_response_s * curvature_rate_m_inv_s));
  dynamics << dot(delayed_speed_command) ==
    (speed_command - delayed_speed_command) / command_delay_s;
  dynamics << dot(delayed_curvature_command) ==
    (curvature_command - delayed_curvature_command) / command_delay_s;
  dynamics << dot(speed_command) == speed_command_rate;
  dynamics << dot(curvature_command) == curvature_command_rate;

  // Generated fallback objective weights. The runtime controller overwrites
  // Q, R and P from ROS parameters before optimization, so weight tuning in a
  // robot YAML file does not require regeneration. Changing the objective
  // dimensions or formulation does require regeneration.
  Matrix q = eye(9);
  q(0, 0) = 8.0;
  q(1, 1) = 8.0;
  q(2, 2) = 25.0;
  q(3, 3) = 4.0;
  q(4, 4) = 12.0;
  q(5, 5) = 0.2;
  q(6, 6) = 0.2;
  q(7, 7) = 0.2;
  q(8, 8) = 0.2;
  Matrix r = eye(2);
  r(0, 0) = 4.0;
  r(1, 1) = 12.0;
  Matrix terminal = q * 3.0;

  // Horizon length and discretization define numSteps and dt in generated
  // code. Changing either value, or the integrator type below, requires solver
  // regeneration. Runtime solver_iterations and reference_update_iterations
  // are wrapper parameters and are not these discretization intervals.
  constexpr double horizon_s = 4.0;
  constexpr int intervals = 40;
  MPCexport nmpc(0.0, horizon_s, intervals);
  nmpc.minimizeLSQ(q, r);
  nmpc.minimizeLSQEndTerm(terminal);
  nmpc.subjectTo(dynamics);
  // These bounds create the constraint structure and provide generated
  // fallbacks. PathTrackingNmpcController overwrites the numeric xMin/xMax and
  // uMin/uMax values from ROS parameters. Tuning existing bounds therefore
  // does not require regeneration; adding or removing a constrained state or
  // control does.
  nmpc.subjectTo(-1.2 <= speed <= 2.3);
  nmpc.subjectTo(-0.6 <= curvature <= 0.6);
  nmpc.subjectTo(-1.2 <= delayed_speed_command <= 2.3);
  nmpc.subjectTo(-0.6 <= delayed_curvature_command <= 0.6);
  nmpc.subjectTo(-1.2 <= speed_command <= 2.3);
  nmpc.subjectTo(-0.6 <= curvature_command <= 0.6);
  nmpc.subjectTo(-5.2 <= speed_command_rate <= 5.6);
  nmpc.subjectTo(-0.28 <= curvature_command_rate <= 0.28);
  nmpc.set(INTEGRATOR_TYPE, INT_RK4);
  nmpc.set(TUNABLE_PARAMETERS, YES);
  nmpc.set(EXPORT_CPP, YES);
  nmpc.set(CLASS_NAME, "PathTrackingNmpcSolver");
  nmpc.exportCode(std::string(argv[1]));
  return 0;
}
