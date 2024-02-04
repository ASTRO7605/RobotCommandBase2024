// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include <wpi/deprecated.h>

#include <rev/CANSparkBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <rev/SparkPIDController.h>

#include <frc/DigitalInput.h>
#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

WPI_IGNORE_DEPRECATED
#include <ctre/Phoenix.h>
WPI_UNIGNORE_DEPRECATED

class Barre : public frc2::SubsystemBase {
  public:
    Barre();

    void Periodic() override;

  private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_MoteurPremierJoint;
    ctre::phoenix::motorcontrol::can::TalonSRX m_MoteurDeuxiemeJoint;
};