#include "subsystems/Shooter.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
double computekAF(double angle) {
    double val{ShooterConstant::kMaxAF *
               cos(ShooterConstant::FDegToRad * (angle - ShooterConstant::kCdMOffset))};
    frc::SmartDashboard::PutNumber("CurrentkAfValueShooter", val);
    return val;
}

Shooter::Shooter()
    : m_LeftFlywheelMotor{ShooterConstant::leftMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_RightFlywheelMotor{ShooterConstant::rightMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_LeftMotorEncoder{
          m_LeftFlywheelMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_RightMotorEncoder{
          m_RightFlywheelMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_LeftFlywheelMotorPIDController{m_LeftFlywheelMotor.GetPIDController()},
      m_RightFlywheelMotorPIDController{m_RightFlywheelMotor.GetPIDController()},
      m_MoteurAngle{ShooterConstant::angleMotorID} {

    m_capteurInterieurShooter.reset(new frc::DigitalInput(ShooterConstant::capteurID));

    m_LeftFlywheelMotor.RestoreFactoryDefaults();
    m_RightFlywheelMotor.RestoreFactoryDefaults();
    m_MoteurAngle.ConfigFactoryDefault();

    m_LeftFlywheelMotor.SetInverted(false);
    m_RightFlywheelMotor.SetInverted(false);
    m_MoteurAngle.SetInverted(false);

    m_LeftFlywheelMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_RightFlywheelMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_MoteurAngle.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_MoteurAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0,
                                               ShooterConstant::kTimeoutMs);
    m_MoteurAngle.SetSensorPhase(false);

    m_MoteurAngle.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10,
                                       ShooterConstant::kTimeoutMs);

    m_MoteurAngle.ConfigPeakOutputForward(ShooterConstant::kPeakOutputForward,
                                          ShooterConstant::kTimeoutMs);
    m_MoteurAngle.ConfigPeakOutputReverse(ShooterConstant::kPeakOutputReverse,
                                          ShooterConstant::kTimeoutMs);
    m_MoteurAngle.ConfigNominalOutputForward(ShooterConstant::kNominalOutputForward,
                                             ShooterConstant::kTimeoutMs);
    m_MoteurAngle.ConfigNominalOutputReverse(ShooterConstant::kNominalOutputReverse,
                                             ShooterConstant::kTimeoutMs);

    m_MoteurAngle.SelectProfileSlot(0, 0); // position
    m_MoteurAngle.Config_kP(0, frc::Preferences::GetDouble("kPPositionAngleLanceur"),
                            ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kI(0, frc::Preferences::GetDouble("kIPositionAngleLanceur"),
                            ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kD(0, frc::Preferences::GetDouble("kDPositionAngleLanceur"),
                            ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kF(0, frc::Preferences::GetDouble("kFPositionAngleLanceur"),
                            ShooterConstant::kTimeoutMs);

    m_MoteurAngle.SelectProfileSlot(1, 0); // vitesse
    m_MoteurAngle.Config_kP(1, frc::Preferences::GetDouble("kPVitesseAngleLanceur"),
                            ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kI(1, frc::Preferences::GetDouble("kIVitesseAngleLanceur"),
                            ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kD(1, frc::Preferences::GetDouble("kDVitesseAngleLanceur"),
                            ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kF(1, frc::Preferences::GetDouble("kFVitesseAngleLanceur"),
                            ShooterConstant::kTimeoutMs);

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

    m_MoteurAngle.ConfigVoltageCompSaturation(ShooterConstant::kVoltageCompensation);
    m_MoteurAngle.EnableVoltageCompensation(true);

    m_LeftFlywheelMotor.EnableVoltageCompensation(ShooterConstant::kVoltageCompensation);
    m_RightFlywheelMotor.EnableVoltageCompensation(ShooterConstant::kVoltageCompensation);

    // init de l'encodeur du moteur angle?
    // motion magic?
    // current limit?

    m_LeftFlywheelMotor.SetSmartCurrentLimit(ShooterConstant::currentLimitFlywheels);
    m_RightFlywheelMotor.SetSmartCurrentLimit(ShooterConstant::currentLimitFlywheels);
}

void Shooter::Periodic() {
    frc::SmartDashboard::PutNumber("leftShooterMotorVelocity", m_LeftMotorEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("rightShooterMotorVelocity", m_RightMotorEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("angleShooterPosition",
                                   m_MoteurAngle.GetSelectedSensorPosition() *
                                       ShooterConstant::FConversionFactorPositionAngle);
    frc::SmartDashboard::PutNumber("angleShooterVelocity",
                                   m_MoteurAngle.GetSelectedSensorVelocity() *
                                       ShooterConstant::FConversionFactorVelocityAngle);
    frc::SmartDashboard::PutBoolean("IsObjectInShooter", IsObjectInShooter());
}

void Shooter::SetWheelSpeeds(double speeds) {
    m_LeftFlywheelMotorPIDController.SetReference(speeds, rev::CANSparkMax::ControlType::kVelocity);
    m_RightFlywheelMotorPIDController.SetReference(speeds,
                                                   rev::CANSparkMax::ControlType::kVelocity);
}

void Shooter::SetShooterAngle(double angle) {
    m_MoteurAngle.SelectProfileSlot(0, 0);
    m_MoteurAngle.Set(ControlMode::Position,
                      angle * ShooterConstant::FConversionFactorPositionAngle,
                      DemandType::DemandType_ArbitraryFeedForward,
                      computekAF(m_MoteurAngle.GetSelectedSensorPosition() *
                                 ShooterConstant::FConversionFactorPositionAngle));
}

void Shooter::ManualShooterAngle(double speed) {
    m_MoteurAngle.SelectProfileSlot(1, 0);
    m_MoteurAngle.Set(ControlMode::Velocity,
                      speed * ShooterConstant::FConversionFactorVelocityAngle,
                      DemandType::DemandType_ArbitraryFeedForward,
                      computekAF(m_MoteurAngle.GetSelectedSensorPosition() *
                                 ShooterConstant::FConversionFactorPositionAngle));
}

bool Shooter::IsObjectInShooter() {
    return !(m_capteurInterieurShooter->Get());
} // si vrai, pas d'objet