#include "subsystems/Shooter.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

Shooter::Shooter()
    : m_LeftMotor{ShooterConstant::leftMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_RightMotor{ShooterConstant::rightMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_LeftMotorEncoder{m_LeftMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_RightMotorEncoder{m_RightMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_LeftMotorPIDController{m_LeftMotor.GetPIDController()},
      m_RightMotorPIDController{m_RightMotor.GetPIDController()},
      m_MoteurAngle{ShooterConstant::angleMotorID} {

    m_capteurInterieurShooter.reset(new frc::DigitalInput(ShooterConstant::capteurID));

    m_LeftMotor.RestoreFactoryDefaults();
    m_RightMotor.RestoreFactoryDefaults();
    m_MoteurAngle.ConfigFactoryDefault();

    m_LeftMotor.SetInverted(false);
    m_RightMotor.SetInverted(false);
    m_MoteurAngle.SetInverted(false);

    m_LeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_RightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_MoteurAngle.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_MoteurAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, ShooterConstant::kTimeoutMs);
    m_MoteurAngle.SetSensorPhase(false);

    m_MoteurAngle.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, ShooterConstant::kTimeoutMs);

    m_MoteurAngle.ConfigPeakOutputForward(ShooterConstant::kPeakOutputForward, ShooterConstant::kTimeoutMs);
    m_MoteurAngle.ConfigPeakOutputReverse(ShooterConstant::kPeakOutputReverse, ShooterConstant::kTimeoutMs);
    m_MoteurAngle.ConfigNominalOutputForward(ShooterConstant::kNominalOutputForward, ShooterConstant::kTimeoutMs);
    m_MoteurAngle.ConfigNominalOutputReverse(ShooterConstant::kNominalOutputReverse, ShooterConstant::kTimeoutMs);

    m_MoteurAngle.SelectProfileSlot(0, 0); //position
    m_MoteurAngle.Config_kP(0, frc::Preferences::GetDouble("kPPositionAngleLanceur"), ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kI(0, frc::Preferences::GetDouble("kIPositionAngleLanceur"), ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kD(0, frc::Preferences::GetDouble("kDPositionAngleLanceur"), ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kF(0, frc::Preferences::GetDouble("kFPositionAngleLanceur"), ShooterConstant::kTimeoutMs);

    m_MoteurAngle.SelectProfileSlot(1, 0); //position
    m_MoteurAngle.Config_kP(1, frc::Preferences::GetDouble("kPVitesseAngleLanceur"), ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kI(1, frc::Preferences::GetDouble("kIVitesseAngleLanceur"), ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kD(1, frc::Preferences::GetDouble("kDVitesseAngleLanceur"), ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kF(1, frc::Preferences::GetDouble("kFVitesseAngleLanceur"), ShooterConstant::kTimeoutMs);

    m_LeftMotorEncoder.SetPositionConversionFactor(
        ShooterConstant::FConversionFactorWheels); // 42 counts per revolution
    m_LeftMotorEncoder.SetVelocityConversionFactor(ShooterConstant::FConversionFactorWheels);

    m_RightMotorEncoder.SetPositionConversionFactor(
        ShooterConstant::FConversionFactorWheels); // 42 counts per revolution
    m_RightMotorEncoder.SetVelocityConversionFactor(ShooterConstant::FConversionFactorWheels);

    m_LeftMotorPIDController.SetP(frc::Preferences::GetDouble("kPFlywheel"));
    m_LeftMotorPIDController.SetI(frc::Preferences::GetDouble("kIFlywheel"));
    m_LeftMotorPIDController.SetD(frc::Preferences::GetDouble("kDFlywheel"));
    m_LeftMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFFlywheel"));

    m_RightMotorPIDController.SetP(frc::Preferences::GetDouble("kPFlywheel"));
    m_RightMotorPIDController.SetI(frc::Preferences::GetDouble("kIFlywheel"));
    m_RightMotorPIDController.SetD(frc::Preferences::GetDouble("kDFlywheel"));
    m_RightMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFFlywheel"));

    m_LeftMotor.SetSmartCurrentLimit(ShooterConstant::currentLimitFlywheels);
    m_RightMotor.SetSmartCurrentLimit(ShooterConstant::currentLimitFlywheels);
}

void Shooter::Periodic() {
    frc::SmartDashboard::PutNumber("leftMotorRotations", m_LeftMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("leftMotorVelocity", m_LeftMotorEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("rightMotorRotations", m_RightMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("rightMotorVelocity", m_RightMotorEncoder.GetVelocity());
}

void Shooter::SetWheelSpeeds(double speeds) {
    m_LeftMotorPIDController.SetReference(speeds, rev::CANSparkMax::ControlType::kVelocity);
    m_RightMotorPIDController.SetReference(speeds, rev::CANSparkMax::ControlType::kVelocity);
}

bool Shooter::IsObjectInShooter() {
    return !(m_capteurInterieurShooter->Get());
} // si vrai, pas d'objet