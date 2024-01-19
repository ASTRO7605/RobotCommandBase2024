#include "subsystems/Shooter.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

Shooter::Shooter()
    : m_leftMotor{ShooterConstant::leftMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_rightMotor{ShooterConstant::rightMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_leftMotorEncoder{m_leftMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_rightMotorEncoder{m_rightMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_leftMotorPIDController{m_leftMotor.GetPIDController()},
      m_rightMotorPIDController{m_rightMotor.GetPIDController()} {

    m_leftMotor.RestoreFactoryDefaults();
    m_rightMotor.RestoreFactoryDefaults();

    m_leftMotor.SetInverted(false);
    m_rightMotor.SetInverted(false);

    m_leftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_leftMotorEncoder.SetPositionConversionFactor((1 / 42)); // 42 counts per revolution
    m_leftMotorEncoder.SetVelocityConversionFactor((1 / 42) / 60);

    m_rightMotorEncoder.SetPositionConversionFactor((1 / 42)); // 42 counts per revolution
    m_rightMotorEncoder.SetVelocityConversionFactor((1 / 42) / 60);

    m_leftMotorPIDController.SetP(frc::Preferences::GetDouble("kPFlywheel"));
    m_leftMotorPIDController.SetI(frc::Preferences::GetDouble("kIFlywheel"));
    m_leftMotorPIDController.SetD(frc::Preferences::GetDouble("kDFlywheel"));

    m_rightMotorPIDController.SetP(frc::Preferences::GetDouble("kPFlywheel"));
    m_rightMotorPIDController.SetI(frc::Preferences::GetDouble("kIFlywheel"));
    m_rightMotorPIDController.SetD(frc::Preferences::GetDouble("kDFlywheel"));
}

void Shooter::Periodic() {
    frc::SmartDashboard::PutNumber("leftMotorRotations", m_leftMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("leftMotorVelocity", m_leftMotorEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("rightMotorRotations", m_rightMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("rightMotorVelocity", m_rightMotorEncoder.GetVelocity());
}