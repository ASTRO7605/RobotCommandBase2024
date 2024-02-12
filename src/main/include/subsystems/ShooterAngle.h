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

class ShooterAngle : public frc2::SubsystemBase {
  public:
    ShooterAngle();

    void Periodic() override;

    /// @brief Set shooter to a specific angle
    /// @param angle Angle of the shooter (1/10 degree)
    void SetShooterAngle(double angle);

    /// @brief Function that returns the current shooter angle
    /// @return The angle of the shooter (1/10th degree)
    double GetShooterAngle();

    /// @brief Set shooter angle to specific manual percentage
    /// @param percent percent of motor output
    void ManualShooterAngle(double percent);

    /// @brief checks to see if shooter is at the target angle
    /// @param target targeted angle, 1/10th of degree
    /// @return true or false
    bool IsShooterAtTargetAngle(double target);

  private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_MoteurAngle;
};