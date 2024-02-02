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

#include <ctre/Phoenix.h>

class ShooterWheels : public frc2::SubsystemBase {
  public:
    ShooterWheels();

    void Periodic() override;
    /// @brief Set shooter wheels to a specific speed
    /// @param speeds Speed of wheels (RPM)
    void SetWheelSpeeds(double speeds);

    bool IsObjectInShooter();

  private:
    rev::CANSparkMax m_LeftFlywheelMotor;
    rev::CANSparkMax m_RightFlywheelMotor;

    rev::SparkRelativeEncoder m_LeftMotorEncoder;
    rev::SparkRelativeEncoder m_RightMotorEncoder;

    rev::SparkPIDController m_LeftFlywheelMotorPIDController;
    rev::SparkPIDController m_RightFlywheelMotorPIDController;

    std::shared_ptr<frc::DigitalInput> m_capteurInterieurShooter;
};