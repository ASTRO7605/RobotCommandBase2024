// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Pose3d.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonUtils.h>

#include "Constants.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <span>
#include <string>
#include <vector>

/// @brief A AprilTag measurement result.
struct PoseMeasurement {
    frc::Pose3d pose;

    units::millisecond_t latency;

    units::meter_t distance;
};

/// @brief A wrapper for the Limelight NetworkTables API.
class Vision : public frc2::SubsystemBase {
  public:
    /// @param table_name the NetworkTables table to use.
    Vision(std::string_view table_name);

    void Periodic() override;
    //void SetPipeline(int pipeline);

    /// @brief Check whether the Limelight sees any valid target.
    /// @return true if >= 1 target is in view, false otherwise.
    bool SeesValidTarget();

    /// @brief Check horizontal error.
    /// @return Horizontal offset from crosshair to target (deg), 0 if not in view.
    double XError();

    /// @brief Check vertical error.
    /// @return Vertical offset from crosshair to target (deg), 0 if not in view.
    double YError();

    /// @brief Get ID of AprilTag in view
    /// @return ID of the primary in-view AprilTag (0 if none).
    int ViewTagID();

    /// @brief Update the base's pose estimation using vision data.
    /// @return Pose estimate of the robot.
    PoseMeasurement GetRobotPoseEstimate();

    /// @brief Get distance between camera and AprilTag in meters
    /// @return Distance in meters.
    double GetAprilTagDistanceMeters();

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    //std::shared_ptr<nt::NetworkTable> m_Vision;

    photon::PhotonCamera m_Vision;
};
