# Copyright 2026 Juha Backman / Natural Resources Institute Finland
# SPDX-License-Identifier: GPL-3.0-only

int64 timestamp
uint8 MODE_RAW_GNSS=0
uint8 MODE_GNSS_LIDAR_EKF=1
uint8 MODE_LANDMARK_DEAD_RECKONING=2

bool initialized
bool reliable
uint8 mode
uint32 active_lidar_clusters
float64 covariance_x
float64 covariance_y
float64 covariance_yaw
