// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

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

class Intake : public frc2::SubsystemBase {
  public:
    Intake();

    void Periodic() override;

    bool IsObjectInIntake();

    /// @brief Set intake motors on or off
    /// @param on true for on, false for off
    /// @param reversed true to eject note
    void SetIntake(bool on, bool reversed);

  private:
    rev::CANSparkMax m_TopMotor;
    rev::CANSparkMax m_BottomMotor;

    rev::SparkRelativeEncoder m_TopMotorEncoder;
    rev::SparkRelativeEncoder m_BottomMotorEncoder;

    rev::SparkPIDController m_TopMotorPIDController;
    rev::SparkPIDController m_BottomMotorPIDController;

    std::shared_ptr<frc::DigitalInput> m_capteurInterieurIntake;
};