#include "subsystems/ShooterWheels.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

ShooterWheels::ShooterWheels()
    : m_LeftFlywheelMotor{ShooterConstant::leftMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_RightFlywheelMotor{ShooterConstant::rightMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_LeftFlywheelMotorEncoder{
          m_LeftFlywheelMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_RightFlywheelMotorEncoder{
          m_RightFlywheelMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_LeftFlywheelMotorPIDController{m_LeftFlywheelMotor.GetPIDController()},
      m_RightFlywheelMotorPIDController{m_RightFlywheelMotor.GetPIDController()} {

    m_capteurInterieurShooter.reset(new frc::DigitalInput(ShooterConstant::capteurID));

    m_LeftFlywheelMotor.RestoreFactoryDefaults();
    m_RightFlywheelMotor.RestoreFactoryDefaults();

    // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces for docs
    m_LeftFlywheelMotor.SetCANTimeout(50);
    m_RightFlywheelMotor.SetCANTimeout(50);

    m_LeftFlywheelMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 50);
    m_RightFlywheelMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 50);

    m_LeftFlywheelMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 50);
    m_RightFlywheelMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 50);

    m_LeftFlywheelMotor.SetInverted(false);
    m_RightFlywheelMotor.SetInverted(true);

    m_LeftFlywheelMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_RightFlywheelMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_LeftFlywheelMotorEncoder.SetPositionConversionFactor(
        ShooterConstant::FConversionFactorWheels);             // 42 counts per revolution
    m_LeftFlywheelMotorEncoder.SetVelocityConversionFactor(1); // already in RPM

    m_RightFlywheelMotorEncoder.SetPositionConversionFactor(
        ShooterConstant::FConversionFactorWheels);              // 42 counts per revolution
    m_RightFlywheelMotorEncoder.SetVelocityConversionFactor(1); // already in RPM

    m_LeftFlywheelMotorPIDController.SetP(ShooterConstant::kPLeftFlywheel);
    m_LeftFlywheelMotorPIDController.SetI(ShooterConstant::kILeftFlywheel);
    m_LeftFlywheelMotorPIDController.SetD(ShooterConstant::kDLeftFlywheel);
    m_LeftFlywheelMotorPIDController.SetFF(ShooterConstant::kFFLeftFlywheel);

    m_RightFlywheelMotorPIDController.SetP(ShooterConstant::kPRightFlywheel);
    m_RightFlywheelMotorPIDController.SetI(ShooterConstant::kIRightFlywheel);
    m_RightFlywheelMotorPIDController.SetD(ShooterConstant::kDRightFlywheel);
    m_RightFlywheelMotorPIDController.SetFF(ShooterConstant::kFFRightFlywheel);

    m_LeftFlywheelMotor.EnableVoltageCompensation(ShooterConstant::kVoltageCompensation);
    m_RightFlywheelMotor.EnableVoltageCompensation(ShooterConstant::kVoltageCompensation);

    m_LeftFlywheelMotor.SetSmartCurrentLimit(ShooterConstant::currentLimitFlywheels);
    m_RightFlywheelMotor.SetSmartCurrentLimit(ShooterConstant::currentLimitFlywheels);

    areWheelsRunning = false;

    for (auto couple : ShooterConstant::wheelSpeedsAccordingToDistance) {
        interpolatingMapShooterWheels.insert(couple.first.value(), couple.second);
    }
}

void ShooterWheels::Periodic() {
    frc::SmartDashboard::PutNumber("leftShooterMotorVelocity",
                                   m_LeftFlywheelMotorEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("rightShooterMotorVelocity",
                                   m_RightFlywheelMotorEncoder.GetVelocity());
    frc::SmartDashboard::PutBoolean("IsObjectInShooter", IsObjectInShooter());
}

void ShooterWheels::SetWheelSpeeds(double speeds, bool spin) {
    double rightSpeeds{speeds};
    double leftSpeeds{speeds};
    if (spin) {
        leftSpeeds = rightSpeeds - ShooterConstant::kRPMDifferenceSpin;
    }
    m_LeftFlywheelMotorPIDController.SetReference(leftSpeeds,
                                                  rev::CANSparkMax::ControlType::kVelocity);
    m_RightFlywheelMotorPIDController.SetReference(rightSpeeds,
                                                   rev::CANSparkMax::ControlType::kVelocity);
}

bool ShooterWheels::IsObjectInShooter() {
    return !(m_capteurInterieurShooter->Get());
} // si vrai, pas d'objet

void ShooterWheels::StopWheels() {
    m_LeftFlywheelMotor.StopMotor();
    m_RightFlywheelMotor.StopMotor();
    areWheelsRunning = false;
}

void ShooterWheels::ManualToggleStartWheels(double speeds) {
    if (areWheelsRunning) {
        StopWheels();
    } else {
        SetWheelSpeeds(speeds, false);
        areWheelsRunning = true;
    }
}

bool ShooterWheels::AreWheelsDoneAccelerating(double target, bool spin) {
    double rightTarget{target};
    double leftTarget{target};
    if (spin) {
        leftTarget = rightTarget - ShooterConstant::kRPMDifferenceSpin;
    }
    if ((fabs(m_LeftFlywheelMotorEncoder.GetVelocity() - leftTarget) <=
         ShooterConstant::speedThreshold) &&
        fabs((m_RightFlywheelMotorEncoder.GetVelocity() - rightTarget) <=
             ShooterConstant::speedThreshold)) {
        return true;
    }
    return false;
}

double ShooterWheels::GetInterpolatedWheelSpeeds(double distanceMeters) {
    return interpolatingMapShooterWheels[distanceMeters];
}