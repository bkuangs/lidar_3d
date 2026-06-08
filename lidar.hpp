#pragma once
#include "geometry.hpp"

struct Lidar {
    Eigen::Isometry3d pose;   // 3d affine transform

    double minRange;          // min/max distance thresholds
    double maxRange;

    double minAzimuth;         // radians, [0, 2pi]
    double maxAzimuth;         
    int azimuthSamples;        // 720

    double minElevation;       // radians, [-15 deg, 15 deg]
    double maxElevation;       
    int elevationSamples;      // 16
};