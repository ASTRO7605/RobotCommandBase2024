#include "subsystems/ShooterAngle.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
double computekAF(double angle) {
    double val{ShooterConstant::kMaxAF * cos(ShooterConstant::FDegToRad * (angle / 10))};
    frc::SmartDashboard::PutNumber("CurrentkAfValueShooter", val);
    return val;
}

ShooterAngle::ShooterAngle() : m_MoteurAngle{ShooterConstant::angleMotorID} {

    m_MoteurAngle.SetInverted(false);

    m_MoteurAngle.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_MoteurAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0,
                                               ShooterConstant::kTimeoutMs);

    m_MoteurAngle.SetSensorPhase(false);

    m_MoteurAngle.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 101,
                                       ShooterConstant::kTimeoutMs);
    m_MoteurAngle.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 101,
                                       ShooterConstant::kTimeoutMs);

    SeedEncoder();

    m_MoteurAngle.ConfigPeakOutputForward(ShooterConstant::kPeakOutputForward,
                                          ShooterConstant::kTimeoutMs);
    m_MoteurAngle.ConfigPeakOutputReverse(ShooterConstant::kPeakOutputReverse,
                                          ShooterConstant::kTimeoutMs);
    m_MoteurAngle.ConfigNominalOutputForward(ShooterConstant::kNominalOutputForward,
                                             ShooterConstant::kTimeoutMs);
    m_MoteurAngle.ConfigNominalOutputReverse(ShooterConstant::kNominalOutputReverse,
                                             ShooterConstant::kTimeoutMs);

    m_MoteurAngle.ConfigForwardSoftLimitThreshold(ShooterConstant::kForwardSoftLimit /
                                                  ShooterConstant::FConversionFactorPositionAngle);
    m_MoteurAngle.ConfigReverseSoftLimitThreshold(ShooterConstant::kReverseSoftLimit /
                                                  ShooterConstant::FConversionFactorPositionAngle);
    m_MoteurAngle.ConfigForwardSoftLimitEnable(true);
    m_MoteurAngle.ConfigReverseSoftLimitEnable(true);

    m_MoteurAngle.SelectProfileSlot(0, 0); // position
    m_MoteurAngle.Config_kP(0, frc::Preferences::GetDouble("kPPositionAngleLanceur"),
                            ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kI(0, frc::Preferences::GetDouble("kIPositionAngleLanceur"),
                            ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kD(0, frc::Preferences::GetDouble("kDPositionAngleLanceur"),
                            ShooterConstant::kTimeoutMs);
    m_MoteurAngle.Config_kF(0, frc::Preferences::GetDouble("kFPositionAngleLanceur"),
                            ShooterConstant::kTimeoutMs);

    m_MoteurAngle.ConfigMotionAcceleration(frc::Preferences::GetDouble("kAccelerationAngle") /
                                           ShooterConstant::FConversionFactorAccelerationAngle);
    m_MoteurAngle.ConfigMotionCruiseVelocity(frc::Preferences::GetDouble("kVitesseAngle") /
                                             ShooterConstant::FConversionFactorVelocityAngle);

    m_MoteurAngle.ConfigVoltageCompSaturation(ShooterConstant::kVoltageCompensation);
    m_MoteurAngle.EnableVoltageCompensation(true);

    m_MoteurAngle.ConfigPeakCurrentLimit(ShooterConstant::kPeakCurrentLimit);
    m_MoteurAngle.ConfigPeakCurrentLimit(ShooterConstant::kPeakCurrentDuration);
    m_MoteurAngle.ConfigContinuousCurrentLimit(ShooterConstant::kContinuousCurrent);
    m_MoteurAngle.EnableCurrentLimit(true);

    isShooterAtInitPose = false;

    for (auto couple : ShooterConstant::shooterAngleAccordingToDistance) {
        interpolatingMapShooterAngle.insert(couple.first.value(), couple.second);
    }

    m_MoteurAngle.NeutralOutput();
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
    m_MoteurAngle.Set(ControlMode::MotionMagic,
                      angle / ShooterConstant::FConversionFactorPositionAngle,
                      DemandType::DemandType_ArbitraryFeedForward,
                      computekAF(m_MoteurAngle.GetSelectedSensorPosition() *
                                 ShooterConstant::FConversionFactorPositionAngle));
}

void ShooterAngle::ManualShooterAngle(double percent) {
    m_MoteurAngle.Set(ControlMode::PercentOutput, percent,
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

void ShooterAngle::KeepCurrentAngle() {
    m_MoteurAngle.Set(ControlMode::MotionMagic, m_MoteurAngle.GetSelectedSensorPosition(),
                      DemandType::DemandType_ArbitraryFeedForward,
                      computekAF(m_MoteurAngle.GetSelectedSensorPosition() *
                                 ShooterConstant::FConversionFactorPositionAngle));
}

void ShooterAngle::SeedEncoder() {
    int absoluteEncoderPosition{m_MoteurAngle.GetSensorCollection().GetPulseWidthPosition()};
    absoluteEncoderPosition = absoluteEncoderPosition % 4096;

    if (absoluteEncoderPosition < 0) {
        absoluteEncoderPosition = absoluteEncoderPosition + 4096;
    }

    double relativeEncoderPosition{absoluteEncoderPosition +
                                   (ShooterConstant::absoluteEncoderOffset /
                                    ShooterConstant ::FConversionFactorPositionAngle)};
    if (relativeEncoderPosition < 0) {
        relativeEncoderPosition = relativeEncoderPosition + 4096;
    }
    relativeEncoderPosition = static_cast<int>(relativeEncoderPosition) % 4096;
    m_MoteurAngle.SetSelectedSensorPosition(relativeEncoderPosition);
}

bool ShooterAngle::IsShooterAngleAtInitPose() { return isShooterAtInitPose; }

void ShooterAngle::SetShooterAngleAtInitPoseFlag() { isShooterAtInitPose = true; }

double ShooterAngle::GetInterpolatedShooterAngle(double distanceMeters) {
    return interpolatingMapShooterAngle[distanceMeters];
}

void ShooterAngle::SetMotorNeutral() { m_MoteurAngle.NeutralOutput(); }