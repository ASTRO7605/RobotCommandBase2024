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

#include <cmath>
#include <optional>
#include <span>
#include <string>

/// @brief A wrapper for the Photonvision API
class Vision : public frc2::SubsystemBase {
  public:
    /// @param table_name the NetworkTables table to use.
    /// @param cameraPose centre of robot to camera distance.
    Vision(std::string_view table_name, frc::Transform3d cameraPose);

    void Periodic() override;

    /// @brief Get robot position estimate based on vision data.
    /// @return Pose estimate of the robot.
    std::optional<PoseMeasurement> GetRobotPoseEstimate();

    /// @brief Get ID of last AprilTag seen
    /// @return ID of the primary in-view AprilTag (0 if none was ever seen).
    int GetLastAprilTagIdSeen() { return lastAprilTagSeen; };

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    // std::shared_ptr<nt::NetworkTable> m_Vision;

    double PythagorasXYZ(double XDistance, double YDistance, double ZDistance);

    std::shared_ptr<photon::PhotonCamera> camera;
    frc::Transform3d robotToCam;
    photon::PhotonPoseEstimator m_PhotonPoseEstimator;
    fs::path deployDirectory{frc::filesystem::GetDeployDirectory()};
    double target_distance{};
    double target_ambiguity{};

    int lastAprilTagSeen{};
    std::optional<PoseMeasurement> latestMeasurement{};
};
