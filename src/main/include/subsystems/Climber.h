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

class Climber : public frc2::SubsystemBase {
  public:
    Climber();

    void Periodic() override;

    /// @brief Set climber to a specific position
    /// @param position Position of the climber (1/10 inch)
    void SetClimberPosition(double position);

    /// @brief Function that returns the current climber position
    /// @return The angle of the shooter (1/10th inch)
    double GetClimberPosition();

    /// @brief Set climber angle to specific manual speed
    /// @param speed speed in (1/10 inch) / s
    void ManualClimber(double speed);

    /// @brief checks to see if climber is at the target position
    /// @param target targeted position (1/10 inch)
    /// @return true or false
    bool IsClimberAtTargetPosition(double target);

  private:
    rev::CANSparkMax m_LeftHookMotor;
    rev::CANSparkMax m_RightHookMotor;

    rev::SparkRelativeEncoder m_LeftHookMotorEncoder;
    rev::SparkRelativeEncoder m_RightHookMotorEncoder;

    rev::SparkPIDController m_LeftHookMotorPIDController;
    rev::SparkPIDController m_RightHookMotorPIDController;
};