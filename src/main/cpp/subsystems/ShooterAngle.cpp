#include "subsystems/ShooterAngle.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
double computekAF(double angle) {
    double val{ShooterConstant::kMaxAF *
               cos(ShooterConstant::FDegToRad * (angle - ShooterConstant::kCdMOffset))};
    frc::SmartDashboard::PutNumber("CurrentkAfValueShooter", val);
    return val;
}

ShooterAngle::ShooterAngle() : m_MoteurAngle{ShooterConstant::angleMotorID} {

    m_MoteurAngle.ConfigFactoryDefault();

    m_MoteurAngle.SetInverted(false);

    m_MoteurAngle.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_MoteurAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0,
                                               ShooterConstant::kTimeoutMs);

    double absoluteEncoderPosition {m_MoteurAngle.GetSensorCollection().GetPulseWidthPosition()};

    m_MoteurAngle.SetSelectedSensorPosition(
        absoluteEncoderPosition + (ShooterConstant::absoluteEncoderOffset / ShooterConstant::FConversionFactorPositionAngle));

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

    m_MoteurAngle.ConfigVoltageCompSaturation(ShooterConstant::kVoltageCompensation);
    m_MoteurAngle.EnableVoltageCompensation(true);

    m_MoteurAngle.ConfigPeakCurrentLimit(ShooterConstant::kPeakCurrentLimit);
    m_MoteurAngle.ConfigPeakCurrentLimit(ShooterConstant::kPeakCurrentDuration);
    m_MoteurAngle.ConfigContinuousCurrentLimit(ShooterConstant::kContinuousCurrent);
    m_MoteurAngle.EnableCurrentLimit(true);
}

void ShooterAngle::Periodic() {
    frc::SmartDashboard::PutNumber("angleShooterPosition",
                                   m_MoteurAngle.GetSelectedSensorPosition() *
                                       ShooterConstant::FConversionFactorPositionAngle);
    frc::SmartDashboard::PutNumber("angleShooterVelocity",
                                   m_MoteurAngle.GetSelectedSensorVelocity() *
                                       ShooterConstant::FConversionFactorVelocityAngle);
}

void ShooterAngle::SetShooterAngle(double angle) {
    m_MoteurAngle.SelectProfileSlot(0, 0);
    m_MoteurAngle.Set(ControlMode::Position,
                      angle / ShooterConstant::FConversionFactorPositionAngle,
                      DemandType::DemandType_ArbitraryFeedForward,
                      computekAF(m_MoteurAngle.GetSelectedSensorPosition() *
                                 ShooterConstant::FConversionFactorPositionAngle));
}

void ShooterAngle::ManualShooterAngle(double speed) {
    m_MoteurAngle.SelectProfileSlot(1, 0);
    m_MoteurAngle.Set(ControlMode::Velocity,
                      speed / ShooterConstant::FConversionFactorVelocityAngle,
                      DemandType::DemandType_ArbitraryFeedForward,
                      computekAF(m_MoteurAngle.GetSelectedSensorPosition() *
                                 ShooterConstant::FConversionFactorPositionAngle));
}

bool ShooterAngle::IsShooterAtTargetAngle(double target) {
    if ((fabs((m_MoteurAngle.GetSelectedSensorPosition() *
               ShooterConstant::FConversionFactorPositionAngle) -
              target) <= ShooterConstant::angleThreshold)) {
        return true;
    }
    return false;
}