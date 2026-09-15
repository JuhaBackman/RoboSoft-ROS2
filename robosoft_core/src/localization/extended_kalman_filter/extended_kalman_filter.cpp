// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "extended_kalman_filter.hpp"

namespace robosoft_core
{

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
  ekf_.initializeCovariances();
}

void ExtendedKalmanFilter::setPositionMeasurement(
  double x, double y, double yaw, bool valid)
{
  if (!initialized_ && valid) {
    ekf_.x[0] = x;
    ekf_.x[1] = y;
    ekf_.x[2] = yaw;
    initialized_ = true;
  }
  if (!initialized_) {
    return;
  }

  if (valid) {
    while (yaw - ekf_.x[2] > M_PI) {yaw -= 2.0 * M_PI;}
    while (yaw - ekf_.x[2] < -M_PI) {yaw += 2.0 * M_PI;}
    ekf_.y[0] = x;
    ekf_.y[1] = y;
    ekf_.y[2] = yaw;
    ekf_.R[0] = 2e-5;
    ekf_.R[1 * ekf_.numEKFMeasurements + 1] = 2e-5;
    ekf_.R[2 * ekf_.numEKFMeasurements + 2] = 1e-6;
  } else {
    for (int index = 0; index < 3; ++index) {
      ekf_.y[index] = ekf_.x[index];
      ekf_.R[index * ekf_.numEKFMeasurements + index] =
        std::numeric_limits<REAL_TYPE>::infinity();
    }
  }
}

void ExtendedKalmanFilter::setControls(double speed_m_s, double curvature_m_inv)
{
  ekf_.u[0] = speed_m_s;
  ekf_.u[1] = curvature_m_inv;
}

bool ExtendedKalmanFilter::update(std::vector<Landmark *> const & landmarks)
{
  if (!initialized_) {
    return false;
  }
  constexpr int maximum_clusters = 6;
  const int count = std::min<int>(landmarks.size(), maximum_clusters);

  for (int index = 0; index < maximum_clusters * 2; ++index) {
    const int measurement = 3 + index;
    ekf_.R[measurement * ekf_.numEKFMeasurements + measurement] =
      std::numeric_limits<REAL_TYPE>::infinity();
  }
  for (int row = 3; row < ekf_.numEKFStates; ++row) {
    for (int column = 0; column < ekf_.numEKFStates; ++column) {
      ekf_.P[row * ekf_.numEKFStates + column] = 0.0;
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 3; column < ekf_.numEKFStates; ++column) {
      ekf_.P[row * ekf_.numEKFStates + column] = 0.0;
    }
  }

  for (int index = 0; index < count; ++index) {
    const int state = 3 + index * 2;
    auto & landmark = *landmarks[index];
    ekf_.R[state * ekf_.numEKFMeasurements + state] =
      landmark.measurement_covariance[0];
    ekf_.R[(state + 1) * ekf_.numEKFMeasurements + state + 1] =
      landmark.measurement_covariance[3];
    ekf_.P[state * ekf_.numEKFStates + state] = landmark.covariance[0];
    ekf_.P[state * ekf_.numEKFStates + state + 1] = landmark.covariance[1];
    ekf_.P[(state + 1) * ekf_.numEKFStates + state] = landmark.covariance[2];
    ekf_.P[(state + 1) * ekf_.numEKFStates + state + 1] =
      landmark.covariance[3];
    ekf_.y[state] = landmark.measurement.x;
    ekf_.y[state + 1] = landmark.measurement.y;
    ekf_.x[state] = landmark.center.x;
    ekf_.x[state + 1] = landmark.center.y;
  }

  ekf_.estimateStates();
  for (int index = 0; index < count; ++index) {
    const int state = 3 + index * 2;
    auto & landmark = *landmarks[index];
    landmark.center = {ekf_.x[state], ekf_.x[state + 1]};
    landmark.covariance = {
      ekf_.P[state * ekf_.numEKFStates + state],
      ekf_.P[state * ekf_.numEKFStates + state + 1],
      ekf_.P[(state + 1) * ekf_.numEKFStates + state],
      ekf_.P[(state + 1) * ekf_.numEKFStates + state + 1]};
  }

  reliable_ =
    ekf_.P[0] <= 1e-4 &&
    ekf_.P[1 * ekf_.numEKFStates + 1] <= 1e-4 &&
    ekf_.P[2 * ekf_.numEKFStates + 2] <= 1e-5;
  return true;
}

}  // namespace robosoft_core
