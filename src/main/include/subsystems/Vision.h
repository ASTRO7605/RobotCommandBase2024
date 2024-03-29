// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utils/PoseMeasurement.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/Filesystem.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose3d.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>

#include "Constants.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/fs.h>

#include <optional>
#include <span>
#include <string>
#include <vector>

/// @brief A wrapper for the Photonvision API
class Vision : public frc2::SubsystemBase {
  public:
    /// @param table_name the NetworkTables table to use.
    Vision(std::string_view table_name, frc::Transform3d cameraPose);

    void Periodic() override;
    // void SetPipeline(int pipeline);

    /// @brief Check whether the Limelight sees any valid target.
    /// @return true if >= 1 target is in view, false otherwise.
    bool SeesValidTarget();

    /// @brief Check horizontal error.
    /// @return Horizontal offset from crosshair to target (deg), 0 if not in view.
    // double XError();

    /// @brief Check vertical error.
    /// @return Vertical offset from crosshair to target (deg), 0 if not in view.
    // double YError();

    /// @brief Get ID of AprilTag in view
    /// @return ID of the primary in-view AprilTag (0 if none).
    // int ViewTagID();

    /// @brief Update the base's pose estimation using vision data.
    /// @return Pose estimate of the robot.
    // PoseMeasurement GetRobotPoseEstimate();

    std::optional<PoseMeasurement> GetRobotPoseEstimate();

    double GetAprilTagDistanceMeters(double XDistance, double YDistance, double ZDistance);

    int GetAprilTagIDInView();

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    // std::shared_ptr<nt::NetworkTable> m_Vision;
    std::shared_ptr<photon::PhotonCamera> camera;
    frc::Transform3d robotToCam;
    photon::PhotonPoseEstimator m_PhotonPoseEstimator;
    bool isLatestSingleResultValid;
    bool isLatestMultiResultValid;
    photon::MultiTargetPNPResult latestMultiResult;
    photon::PhotonTrackedTarget latestSingleResult;
    fs::path deployDirectory{frc::filesystem::GetDeployDirectory()};
    double target_distance{};
    double target_ambiguity{};

    int lastAprilTagSeen{};
};
