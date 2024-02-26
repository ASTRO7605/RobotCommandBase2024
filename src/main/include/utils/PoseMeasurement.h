#pragma once

#include "Constants.h"
#include <frc/geometry/Pose3d.h>
/// @brief A AprilTag measurement result.
struct PoseMeasurement {
    frc::Pose3d pose;

    units::millisecond_t timestamp;

    units::meter_t distance;

    double ambiguity;
};