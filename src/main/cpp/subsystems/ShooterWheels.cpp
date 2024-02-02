#include "subsystems/ShooterWheels.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

ShooterWheels::ShooterWheels()
    : m_LeftFlywheelMotor{ShooterConstant::leftMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_RightFlywheelMotor{ShooterConstant::rightMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_LeftMotorEncoder{
          m_LeftFlywheelMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_RightMotorEncoder{
          m_RightFlywheelMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_LeftFlywheelMotorPIDController{m_LeftFlywheelMotor.GetPIDController()},
      m_RightFlywheelMotorPIDController{m_RightFlywheelMotor.GetPIDController()} {

    m_capteurInterieurShooter.reset(new frc::DigitalInput(ShooterConstant::capteurID));

    m_LeftFlywheelMotor.RestoreFactoryDefaults();
    m_RightFlywheelMotor.RestoreFactoryDefaults();

    m_LeftFlywheelMotor.SetInverted(false);
    m_RightFlywheelMotor.SetInverted(false);

    m_LeftFlywheelMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_RightFlywheelMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_LeftMotorEncoder.SetPositionConversionFactor(
        ShooterConstant::FConversionFactorWheels); // 42 counts per revolution
    m_LeftMotorEncoder.SetVelocityConversionFactor(ShooterConstant::FConversionFactorWheels);

    m_RightMotorEncoder.SetPositionConversionFactor(
        ShooterConstant::FConversionFactorWheels); // 42 counts per revolution
    m_RightMotorEncoder.SetVelocityConversionFactor(ShooterConstant::FConversionFactorWheels);

    m_LeftFlywheelMotorPIDController.SetP(frc::Preferences::GetDouble("kPFlywheel"));
    m_LeftFlywheelMotorPIDController.SetI(frc::Preferences::GetDouble("kIFlywheel"));
    m_LeftFlywheelMotorPIDController.SetD(frc::Preferences::GetDouble("kDFlywheel"));
    m_LeftFlywheelMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFFlywheel"));

    m_RightFlywheelMotorPIDController.SetP(frc::Preferences::GetDouble("kPFlywheel"));
    m_RightFlywheelMotorPIDController.SetI(frc::Preferences::GetDouble("kIFlywheel"));
    m_RightFlywheelMotorPIDController.SetD(frc::Preferences::GetDouble("kDFlywheel"));
    m_RightFlywheelMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFFlywheel"));

    m_LeftFlywheelMotor.EnableVoltageCompensation(ShooterConstant::kVoltageCompensation);
    m_RightFlywheelMotor.EnableVoltageCompensation(ShooterConstant::kVoltageCompensation);

    m_LeftFlywheelMotor.SetSmartCurrentLimit(ShooterConstant::currentLimitFlywheels);
    m_RightFlywheelMotor.SetSmartCurrentLimit(ShooterConstant::currentLimitFlywheels);
}

void ShooterWheels::Periodic() {
    frc::SmartDashboard::PutNumber("leftShooterMotorVelocity", m_LeftMotorEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("rightShooterMotorVelocity", m_RightMotorEncoder.GetVelocity());
    frc::SmartDashboard::PutBoolean("IsObjectInShooter", IsObjectInShooter());
}

void ShooterWheels::SetWheelSpeeds(double speeds) {
    m_LeftFlywheelMotorPIDController.SetReference(speeds, rev::CANSparkMax::ControlType::kVelocity);
    m_RightFlywheelMotorPIDController.SetReference(speeds,
                                                   rev::CANSparkMax::ControlType::kVelocity);
}

bool ShooterWheels::IsObjectInShooter() {
    return !(m_capteurInterieurShooter->Get());
} // si vrai, pas d'objet