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

class LeftHook : public frc2::SubsystemBase {
  public:
    LeftHook();

    void Periodic() override;

    /// @brief Set left hook to a specific position
    /// @param position Position of the left hook (1/10 inch)
    void SetLeftHookPosition(double position);

    /// @brief Set left hook to specific motor output
    /// @param percent output of motor
    void ManualLeftHook(double percent);

    /// @brief checks to see if left hook is at the target position
    /// @param target targeted position (1/10 inch)
    /// @return true or false
    bool IsLeftHookAtTargetPosition(double target);

    void KeepLeftHookPosition();

  private:
    rev::CANSparkMax m_LeftHookMotor;

    rev::SparkRelativeEncoder m_LeftHookMotorEncoder;

    rev::SparkPIDController m_LeftHookMotorPIDController;
};