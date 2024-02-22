// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <rev/CANSparkBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <rev/SparkPIDController.h>
#include <wpi/interpolating_map.h>

#include <frc/DigitalInput.h>
#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class ShooterWheels : public frc2::SubsystemBase {
  public:
    ShooterWheels();

    void Periodic() override;
    /// @brief Set shooter wheels to a specific speed
    /// @param speeds Speed of wheels (RPM)
    void SetWheelSpeeds(double speeds, bool spin);

    bool IsObjectInShooter();

    void StopWheels();

    /// @brief function used to start or stop wheels manually. If wheels are not running, start
    /// them, if they are, stop them.
    /// @param speeds Speed of wheels (RPM)
    void ManualToggleStartWheels(double speeds);

    /// @brief Function to check if wheel speeds are within a certain threshold of a target
    /// @param target Target to check (RPM)
    /// @return True if within threshold, false if not
    bool AreWheelsDoneAccelerating(double target, bool spin);

    double GetInterpolatedWheelSpeeds(double distanceMeters);

  private:
    rev::CANSparkMax m_LeftFlywheelMotor;
    rev::CANSparkMax m_RightFlywheelMotor;

    rev::SparkRelativeEncoder m_LeftFlywheelMotorEncoder;
    rev::SparkRelativeEncoder m_RightFlywheelMotorEncoder;

    rev::SparkPIDController m_LeftFlywheelMotorPIDController;
    rev::SparkPIDController m_RightFlywheelMotorPIDController;

    std::shared_ptr<frc::DigitalInput> m_capteurInterieurShooter;

    bool areWheelsRunning;

    wpi::interpolating_map<double, double> interpolatingMapShooterWheels;

    bool m_MotorsInitialized;
};