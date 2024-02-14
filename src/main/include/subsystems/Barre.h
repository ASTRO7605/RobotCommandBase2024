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

    /// @brief Sets 1er joint to a manual percentage output
    /// @param percent percent of motor output [-1,1]
    void Manual1erJoint(double percent);

    /// @brief Sets 2e joint to a manual percentage output
    /// @param percent percent of motor output [-1,1]
    void Manual2eJoint(double percent);

    /// @brief Set 1er joint to a specific angle
    /// @param angle Angle of 1er joint (1/10 degree)
    void Set1erJointAngle(double angle);

    /// @brief Set 2e joint to a specific angle
    /// @param angle Angle of 2e joint (1/10 degree)
    void Set2eJointAngle(double angle);

    /// @brief checks to see if 1er joint is at the target angle
    /// @param target targeted angle, 1/10th of degree
    /// @return true or false
    bool Is1erJointAtTargetAngle(double target);

    /// @brief checks to see if 2e joint is at the target angle
    /// @param target targeted angle, 1/10th of degree
    /// @return true or false
    bool Is2eJointAtTargetAngle(double target);

    void SeedEncoder1erJoint();

    void SeedEncoder2eJoint();

    void Periodic() override;

  private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_MoteurPremierJoint;
    ctre::phoenix::motorcontrol::can::TalonSRX m_MoteurDeuxiemeJoint;
};