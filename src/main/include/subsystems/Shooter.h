// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <rev/SparkPIDController.h>

#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class Shooter : public frc2::SubsystemBase {
public:
  Shooter();

  void Periodic() override;

private:
  rev::CANSparkMax m_leftMotor;
  rev::CANSparkMax m_rightMotor;

  rev::SparkRelativeEncoder m_leftMotorEncoder;
  rev::SparkRelativeEncoder m_rightMotorEncoder;

  rev::SparkPIDController m_leftMotorPIDController;
  rev::SparkPIDController m_rightMotorPIDController;
};