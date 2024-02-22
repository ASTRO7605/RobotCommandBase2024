#include "subsystems/Barre.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
double computekAF1erJoint(double angle) {
    double val{BarreConstant::kMaxAF1erJoint * cos(BarreConstant::FDegToRad * ((angle) / 10))};
    frc::SmartDashboard::PutNumber("CurrentkAfValue1erJoint", val);
    return val;
}

Barre::Barre()
    : m_MoteurPremierJoint{BarreConstant::moteurPremierJointID},
      m_MoteurDeuxiemeJoint{BarreConstant::moteurDeuxiemeJointID}, m_MotorsInitialized{false} {}

void Barre::Periodic() {
    frc::SmartDashboard::PutNumber("1erJointPosition",
                                   m_MoteurPremierJoint.GetSelectedSensorPosition() *
                                       BarreConstant::FConversionFactorPosition1erJoint);
    frc::SmartDashboard::PutNumber("1erJointVelocity",
                                   m_MoteurPremierJoint.GetSelectedSensorVelocity() *
                                       BarreConstant::FConversionFactorVelocity1erJoint);
    frc::SmartDashboard::PutNumber("2eJointPosition",
                                   m_MoteurDeuxiemeJoint.GetSelectedSensorPosition() *
                                       BarreConstant::FConversionFactorPosition2eJoint);
    frc::SmartDashboard::PutNumber("2eJointVelocity",
                                   m_MoteurDeuxiemeJoint.GetSelectedSensorVelocity() *
                                       BarreConstant::FConversionFactorVelocity2eJoint);

    // Non-zero or false indicates success
    if (m_MotorsInitialized || m_MoteurPremierJoint.ConfigFactoryDefault() ||
            m_MoteurDeuxiemeJoint.ConfigFactoryDefault() ||

            m_MoteurPremierJoint.ConfigSelectedFeedbackSensor(
                FeedbackDevice::CTRE_MagEncoder_Relative, 0, BarreConstant::kTimeoutMs) ||
            m_MoteurDeuxiemeJoint.ConfigSelectedFeedbackSensor(
                FeedbackDevice::CTRE_MagEncoder_Relative, 0, BarreConstant::kTimeoutMs) ||

            m_MoteurPremierJoint.ConfigFeedbackNotContinuous(true) ||
            m_MoteurDeuxiemeJoint.ConfigFeedbackNotContinuous(true) ||

            SeedEncoder1erJoint() || SeedEncoder2eJoint() ||

            m_MoteurPremierJoint.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10,
                                                      BarreConstant::kTimeoutMs) ||
            m_MoteurPremierJoint.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic,
                                                      10, BarreConstant::kTimeoutMs) ||
            m_MoteurDeuxiemeJoint.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0,
                                                       10, BarreConstant::kTimeoutMs) ||
            m_MoteurDeuxiemeJoint.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic,
                                                       10, BarreConstant::kTimeoutMs) ||

            m_MoteurPremierJoint.ConfigPeakOutputForward(BarreConstant::kPeakOutputForward,
                                                         BarreConstant::kTimeoutMs) ||
            m_MoteurPremierJoint.ConfigPeakOutputReverse(BarreConstant::kPeakOutputReverse,
                                                         BarreConstant::kTimeoutMs) ||
            m_MoteurPremierJoint.ConfigNominalOutputForward(BarreConstant::kNominalOutputForward,
                                                            BarreConstant::kTimeoutMs) ||
            m_MoteurPremierJoint.ConfigNominalOutputReverse(BarreConstant::kNominalOutputReverse,
                                                            BarreConstant::kTimeoutMs) ||

            m_MoteurDeuxiemeJoint.ConfigPeakOutputForward(BarreConstant::kPeakOutputForward,
                                                          BarreConstant::kTimeoutMs) ||
            m_MoteurDeuxiemeJoint.ConfigPeakOutputReverse(BarreConstant::kPeakOutputReverse,
                                                          BarreConstant::kTimeoutMs) ||
            m_MoteurDeuxiemeJoint.ConfigNominalOutputForward(BarreConstant::kNominalOutputForward,
                                                             BarreConstant::kTimeoutMs) ||
            m_MoteurDeuxiemeJoint.ConfigNominalOutputReverse(BarreConstant::kNominalOutputReverse,
                                                             BarreConstant::kTimeoutMs) ||

            m_MoteurPremierJoint.SelectProfileSlot(0, 0) || // Motion
            m_MoteurPremierJoint.Config_kP(0, BarreConstant::kPMotion1erJoint,
                                           BarreConstant::kTimeoutMs) ||
            m_MoteurPremierJoint.Config_kI(0, BarreConstant::kIMotion1erJoint,
                                           BarreConstant::kTimeoutMs) ||
            m_MoteurPremierJoint.Config_kD(0, BarreConstant::kDMotion1erJoint,
                                           BarreConstant::kTimeoutMs) ||
            m_MoteurPremierJoint.Config_kF(0, BarreConstant::kFMotion1erJoint,
                                           BarreConstant::kTimeoutMs) ||

            m_MoteurDeuxiemeJoint.SelectProfileSlot(0, 0) || // Motion
            m_MoteurDeuxiemeJoint.Config_kP(0, BarreConstant::kPMotion2eJoint,
                                            BarreConstant::kTimeoutMs) ||
            m_MoteurDeuxiemeJoint.Config_kI(0, BarreConstant::kIMotion2eJoint,
                                            BarreConstant::kTimeoutMs) ||
            m_MoteurDeuxiemeJoint.Config_kD(0, BarreConstant::kDMotion2eJoint,
                                            BarreConstant::kTimeoutMs) ||
            m_MoteurDeuxiemeJoint.Config_kF(0, BarreConstant::kFMotion2eJoint,
                                            BarreConstant::kTimeoutMs) ||

            m_MoteurPremierJoint.ConfigMotionCruiseVelocity(
                frc::Preferences::GetDouble("kVitesse1erJoint") /
                BarreConstant::FConversionFactorVelocity1erJoint) ||
            m_MoteurPremierJoint.ConfigMotionAcceleration(
                frc::Preferences::GetDouble("kAcceleration1erJoint") /
                BarreConstant::FConversionFactorAcceleration1erJoint) ||

            m_MoteurDeuxiemeJoint.ConfigMotionCruiseVelocity(
                frc::Preferences::GetDouble("kVitesse2eJoint") /
                BarreConstant::FConversionFactorVelocity2eJoint);
        m_MoteurDeuxiemeJoint.ConfigMotionAcceleration(
            frc::Preferences::GetDouble("kAcceleration2eJoint") /
            BarreConstant::FConversionFactorAcceleration2eJoint) ||

        m_MoteurPremierJoint.ConfigVoltageCompSaturation(BarreConstant::kVoltageCompensation) ||

        m_MoteurDeuxiemeJoint.ConfigVoltageCompSaturation(BarreConstant::kVoltageCompensation) ||

        m_MoteurPremierJoint.ConfigPeakCurrentLimit(BarreConstant::kPeakCurrentLimit) ||
        m_MoteurPremierJoint.ConfigPeakCurrentLimit(BarreConstant::kPeakCurrentDuration) ||
        m_MoteurPremierJoint.ConfigContinuousCurrentLimit(BarreConstant::kContinuousCurrent) ||

        m_MoteurDeuxiemeJoint.ConfigPeakCurrentLimit(BarreConstant::kPeakCurrentLimit) ||
        m_MoteurDeuxiemeJoint.ConfigPeakCurrentLimit(BarreConstant::kPeakCurrentDuration) ||
        m_MoteurDeuxiemeJoint.ConfigContinuousCurrentLimit(BarreConstant::kContinuousCurrent) ||

        m_MoteurPremierJoint.ConfigForwardSoftLimitThreshold(
            BarreConstant::kForwardSoftLimit1erJoint /
            BarreConstant::FConversionFactorPosition1erJoint) ||
        m_MoteurPremierJoint.ConfigReverseSoftLimitThreshold(
            BarreConstant::kReverseSoftLimit1erJoint /
            BarreConstant::FConversionFactorPosition1erJoint) ||
        m_MoteurPremierJoint.ConfigForwardSoftLimitEnable(true) ||
        m_MoteurPremierJoint.ConfigReverseSoftLimitEnable(true) ||

        m_MoteurDeuxiemeJoint.ConfigForwardSoftLimitThreshold(
            BarreConstant::kForwardSoftLimit2eJoint /
            BarreConstant::FConversionFactorPosition2eJoint) ||
        m_MoteurDeuxiemeJoint.ConfigReverseSoftLimitThreshold(
            BarreConstant::kReverseSoftLimit2eJoint /
            BarreConstant::FConversionFactorPosition2eJoint) ||
        m_MoteurDeuxiemeJoint.ConfigForwardSoftLimitEnable(true) ||
        m_MoteurDeuxiemeJoint.ConfigReverseSoftLimitEnable(true)) {
    } else {
        // Stuff that doesn't return a status code
        m_MoteurPremierJoint.EnableVoltageCompensation(true);
        m_MoteurDeuxiemeJoint.EnableVoltageCompensation(true);

        m_MoteurPremierJoint.EnableCurrentLimit(true);
        m_MoteurDeuxiemeJoint.EnableCurrentLimit(true);

        m_MoteurPremierJoint.SetSensorPhase(true);
        m_MoteurDeuxiemeJoint.SetSensorPhase(false);

        m_MoteurPremierJoint.SetInverted(true);
        m_MoteurDeuxiemeJoint.SetInverted(true);

        m_MoteurPremierJoint.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_MoteurDeuxiemeJoint.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

        m_MotorsInitialized = true;
    }
}

void Barre::Manual1erJoint(double percent) {
    m_MoteurPremierJoint.Set(ControlMode::PercentOutput, percent,
                             DemandType::DemandType_ArbitraryFeedForward,
                             computekAF1erJoint(m_MoteurPremierJoint.GetSelectedSensorPosition() *
                                                BarreConstant::FConversionFactorPosition1erJoint));
    // m_MoteurPremierJoint.Set(ControlMode::Velocity,
    //                          percent / BarreConstant::FConversionFactorVelocity1erJoint,
    //                          DemandType::DemandType_ArbitraryFeedForward,
    //                          computekAF1erJoint(m_MoteurPremierJoint.GetSelectedSensorPosition()
    //                          *
    //                                             BarreConstant::FConversionFactorPosition1erJoint));
}

void Barre::Manual2eJoint(double percent) {
    m_MoteurDeuxiemeJoint.Set(ControlMode::PercentOutput, percent);
    // m_MoteurDeuxiemeJoint.Set(ControlMode::Velocity,
    //                          percent / BarreConstant::FConversionFactorVelocity2eJoint);
}

void Barre::Set1erJointAngle(double angle) {
    m_MoteurPremierJoint.Set(ControlMode::MotionMagic,
                             angle / BarreConstant::FConversionFactorPosition1erJoint,
                             DemandType::DemandType_ArbitraryFeedForward,
                             computekAF1erJoint(m_MoteurPremierJoint.GetSelectedSensorPosition() *
                                                BarreConstant::FConversionFactorPosition1erJoint));
}

void Barre::Set2eJointAngle(double angle) {
    m_MoteurDeuxiemeJoint.Set(ControlMode::MotionMagic,
                              angle / BarreConstant::FConversionFactorPosition2eJoint);
}

bool Barre::Is1erJointAtTargetAngle(double target) {
    if ((fabs((m_MoteurPremierJoint.GetSelectedSensorPosition() *
               BarreConstant::FConversionFactorPosition1erJoint) -
              target) <= BarreConstant::angleThreshold)) {
        return true;
    }
    return false;
}

bool Barre::Is2eJointAtTargetAngle(double target) {
    if ((fabs((m_MoteurDeuxiemeJoint.GetSelectedSensorPosition() *
               BarreConstant::FConversionFactorPosition2eJoint) -
              target) <= BarreConstant::angleThreshold)) {
        return true;
    }
    return false;
}

ctre::phoenix::ErrorCode Barre::SeedEncoder1erJoint() {

    int absoluteEncoderPositionPremier{
        m_MoteurPremierJoint.GetSensorCollection().GetPulseWidthPosition()};
    absoluteEncoderPositionPremier = absoluteEncoderPositionPremier % 4096;

    if (absoluteEncoderPositionPremier < 0) {
        absoluteEncoderPositionPremier = absoluteEncoderPositionPremier + 4096;
    }

    double relativeEncoderPositionPremier{absoluteEncoderPositionPremier +
                                          (BarreConstant::absoluteEncoderOffset1erJoint /
                                           BarreConstant ::FConversionFactorPosition1erJoint)};
    if (relativeEncoderPositionPremier < 0) {
        relativeEncoderPositionPremier = relativeEncoderPositionPremier + 4096;
    }
    relativeEncoderPositionPremier = static_cast<int>(relativeEncoderPositionPremier) % 4096;
    return m_MoteurPremierJoint.SetSelectedSensorPosition(relativeEncoderPositionPremier);
}

ctre::phoenix::ErrorCode Barre::SeedEncoder2eJoint() {
    int absoluteEncoderPositionDeuxieme{
        -m_MoteurDeuxiemeJoint.GetSensorCollection().GetPulseWidthPosition()};
    absoluteEncoderPositionDeuxieme = absoluteEncoderPositionDeuxieme % 4096;

    if (absoluteEncoderPositionDeuxieme < 0) {
        absoluteEncoderPositionDeuxieme = absoluteEncoderPositionDeuxieme + 4096;
    }

    double relativeEncoderPositionDeuxieme{absoluteEncoderPositionDeuxieme +
                                           (BarreConstant::absoluteEncoderOffset2eJoint /
                                            BarreConstant ::FConversionFactorPosition2eJoint)};
    if (relativeEncoderPositionDeuxieme < 0) {
        relativeEncoderPositionDeuxieme = relativeEncoderPositionDeuxieme + 4096;
    }
    relativeEncoderPositionDeuxieme = static_cast<int>(relativeEncoderPositionDeuxieme) % 4096;
    return m_MoteurDeuxiemeJoint.SetSelectedSensorPosition(relativeEncoderPositionDeuxieme);
}

void Barre::KeepCurrentAngle1erJoint() {
    m_MoteurPremierJoint.Set(ControlMode::MotionMagic,
                             m_MoteurPremierJoint.GetSelectedSensorPosition(),
                             DemandType::DemandType_ArbitraryFeedForward,
                             computekAF1erJoint(m_MoteurPremierJoint.GetSelectedSensorPosition() *
                                                BarreConstant::FConversionFactorPosition1erJoint));
}

void Barre::KeepCurrentAngle2eJoint() {
    m_MoteurDeuxiemeJoint.Set(ControlMode::MotionMagic,
                              m_MoteurDeuxiemeJoint.GetSelectedSensorPosition());
}
