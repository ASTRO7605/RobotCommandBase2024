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

class Shooter : public frc2::SubsystemBase {
  public:
    Shooter();

    void Periodic() override;
    /// @brief Set shooter wheels to a specific speed
    /// @param speeds Speed of wheels (RPM)
    void SetWheelSpeeds(double speeds);

    bool IsObjectInShooter();

  private:
    rev::CANSparkMax m_LeftMotor;
    rev::CANSparkMax m_RightMotor;

    rev::SparkRelativeEncoder m_LeftMotorEncoder;
    rev::SparkRelativeEncoder m_RightMotorEncoder;

    rev::SparkPIDController m_LeftMotorPIDController;
    rev::SparkPIDController m_RightMotorPIDController;

    ctre::phoenix::motorcontrol::can::TalonSRX m_MoteurAngle;

    std::shared_ptr<frc::DigitalInput> m_capteurInterieurShooter;
};