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

    /// @brief Set left hook to a specific position
    /// @param position Position of the left hook (1/10 inch)
    void SetRightHookPosition(double position);

    /// @brief Set left hook to specific motor output
    /// @param percent output of motor
    void ManualRightHook(double percent);

    /// @brief checks to see if left hook is at the target position
    /// @param target targeted position (1/10 inch)
    /// @return true or false
    bool IsRightHookAtTargetPosition(double target);

    void KeepRightHookPosition();

    bool IsRightHookStopped();

    /// @brief Sets left hook encoder to a position
    /// @param newPosition Position to set the encoder to (1/10 inch)
    void SetRightHookEncoderPosition(double newPosition);

    frc::TrapezoidProfile<units::meters>::State GetRightHookState();

    bool IsInitDone();

    void SetInitDone();

    bool IsInitScheduled();

    void SetInitScheduled();

  private:
    rev::CANSparkMax m_RightHookMotor;

    rev::SparkRelativeEncoder m_RightHookMotorEncoder;

    rev::SparkPIDController m_RightHookMotorPIDController;

    bool isInitDone;
    bool isInitScheduled;
};