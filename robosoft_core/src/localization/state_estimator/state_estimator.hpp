// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "VehicleEKF.h"
#include "lidar_cluster.hpp"

namespace robosoft_core
{

/// Hand-written adapter around the unchanged VIATOC-generated VehicleEKF.
class StateEstimator
{
public:
  StateEstimator();
  void setPositionMeasurement(double x, double y, double yaw, bool valid);
  void setControls(double speed_m_s, double curvature_m_inv);
  bool update(const std::vector<ClusterLandmark *> & clusters);
  bool initialized() const {return initialized_;}
  bool reliable() const {return reliable_;}
  double covarianceX() const {return ekf_.P[0];}
  double covarianceY() const
  {
    return ekf_.P[1 * ekf_.numEKFStates + 1];
  }
  double covarianceYaw() const
  {
    return ekf_.P[2 * ekf_.numEKFStates + 2];
  }
  double x() const {return ekf_.x[0];}
  double y() const {return ekf_.x[1];}
  double yaw() const {return ekf_.x[2];}

private:
  VehicleEKF ekf_;
  bool initialized_{false};
  bool reliable_{false};
};

}  // namespace robosoft_core
