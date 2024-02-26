// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utils/PoseMeasurement.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

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

class Limelight : public frc2::SubsystemBase {
  public:
    /// @param table_name the NetworkTables table to use.
    Limelight(std::string table_name);

    void Periodic() override;

    std::optional<photon::PhotonTrackedTarget> GetLatestTarget();
  private:
    photon::PhotonCamera camera;
};
