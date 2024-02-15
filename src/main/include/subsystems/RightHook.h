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
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class RightHook : public frc2::SubsystemBase {
  public:
    RightHook();

    void Periodic() override;

    /// @brief Set right hook to a specific position
    /// @param position Position of the right hook (1/10 inch)
    void SetRightHookPosition(double position);

    /// @brief Set right hook to specific motor output
    /// @param percent output of motor
    void ManualRightHook(double percent);

    /// @brief checks to see if right hook is at the target position
    /// @param target targeted position (1/10 inch)
    /// @return true or false
    bool IsRightHookAtTargetPosition(double target);

    void KeepRightHookPosition();

  private:
    rev::CANSparkMax m_RightHookMotor;

    rev::SparkRelativeEncoder m_RightHookMotorEncoder;

    rev::SparkPIDController m_RightHookMotorPIDController;
};