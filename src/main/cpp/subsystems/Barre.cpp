#include "subsystems/Barre.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
double computekAF1erJoint(double angle) {
    double val{BarreConstant::kMaxAF1erJoint *
               cos(BarreConstant::FDegToRad * (angle - BarreConstant::kCdMOffset1erJoint))};
    frc::SmartDashboard::PutNumber("CurrentkAfValue1erJoint", val);
    return val;
}

double computekAF2eJoint(double angle) {
    double val{BarreConstant::kMaxAF2eJoint *
               cos(BarreConstant::FDegToRad * (angle - BarreConstant::kCdMOffset2eJoint))};
    frc::SmartDashboard::PutNumber("CurrentkAfValue2eJoint", val);
    return val;
}

Barre::Barre()
    : m_MoteurPremierJoint{BarreConstant::moteurPremierJointID},
      m_MoteurDeuxiemeJoint{BarreConstant::moteurDeuxiemeJointID} {

    m_MoteurPremierJoint.ConfigFactoryDefault();
    m_MoteurDeuxiemeJoint.ConfigFactoryDefault();

    m_MoteurPremierJoint.SetInverted(false);
    m_MoteurDeuxiemeJoint.SetInverted(false);

    m_MoteurPremierJoint.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_MoteurDeuxiemeJoint.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_MoteurPremierJoint.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0,
                                                      BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0,
                                                       BarreConstant::kTimeoutMs);

    double absoluteEncoderPositionPremier{m_MoteurPremierJoint.GetSelectedSensorPosition()};
    double absoluteEncoderPositionDeuxieme{m_MoteurDeuxiemeJoint.GetSelectedSensorPosition()};

    m_MoteurPremierJoint.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0,
                                                      BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0,
                                                       BarreConstant::kTimeoutMs);

    m_MoteurPremierJoint.SetSelectedSensorPosition(absoluteEncoderPositionPremier);
    m_MoteurDeuxiemeJoint.SetSelectedSensorPosition(absoluteEncoderPositionDeuxieme);

    m_MoteurPremierJoint.SetSensorPhase(false);
    m_MoteurDeuxiemeJoint.SetSensorPhase(false);

    m_MoteurPremierJoint.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10,
                                              BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10,
                                               BarreConstant::kTimeoutMs);

    m_MoteurPremierJoint.ConfigPeakOutputForward(BarreConstant::kPeakOutputForward,
                                                 BarreConstant::kTimeoutMs);
    m_MoteurPremierJoint.ConfigPeakOutputReverse(BarreConstant::kPeakOutputReverse,
                                                 BarreConstant::kTimeoutMs);
    m_MoteurPremierJoint.ConfigNominalOutputForward(BarreConstant::kNominalOutputForward,
                                                    BarreConstant::kTimeoutMs);
    m_MoteurPremierJoint.ConfigNominalOutputReverse(BarreConstant::kNominalOutputReverse,
                                                    BarreConstant::kTimeoutMs);

    m_MoteurDeuxiemeJoint.ConfigPeakOutputForward(BarreConstant::kPeakOutputForward,
                                                  BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.ConfigPeakOutputReverse(BarreConstant::kPeakOutputReverse,
                                                  BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.ConfigNominalOutputForward(BarreConstant::kNominalOutputForward,
                                                     BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.ConfigNominalOutputReverse(BarreConstant::kNominalOutputReverse,
                                                     BarreConstant::kTimeoutMs);

    m_MoteurPremierJoint.SelectProfileSlot(0, 0); // position
    m_MoteurPremierJoint.Config_kP(0, frc::Preferences::GetDouble("kPPosition1erJoint"),
                                   BarreConstant::kTimeoutMs);
    m_MoteurPremierJoint.Config_kI(0, frc::Preferences::GetDouble("kIPosition1erJoint"),
                                   BarreConstant::kTimeoutMs);
    m_MoteurPremierJoint.Config_kD(0, frc::Preferences::GetDouble("kDPosition1erJoint"),
                                   BarreConstant::kTimeoutMs);
    m_MoteurPremierJoint.Config_kF(0, frc::Preferences::GetDouble("kFPosition1erJoint"),
                                   BarreConstant::kTimeoutMs);

    m_MoteurPremierJoint.SelectProfileSlot(1, 0); // vitesse
    m_MoteurPremierJoint.Config_kP(1, frc::Preferences::GetDouble("kPVitesse1erJoint"),
                                   BarreConstant::kTimeoutMs);
    m_MoteurPremierJoint.Config_kI(1, frc::Preferences::GetDouble("kIVitesse1erJoint"),
                                   BarreConstant::kTimeoutMs);
    m_MoteurPremierJoint.Config_kD(1, frc::Preferences::GetDouble("kDVitesse1erJoint"),
                                   BarreConstant::kTimeoutMs);
    m_MoteurPremierJoint.Config_kF(1, frc::Preferences::GetDouble("kFVitesse1erJoint"),
                                   BarreConstant::kTimeoutMs);

    m_MoteurDeuxiemeJoint.SelectProfileSlot(0, 0); // position
    m_MoteurDeuxiemeJoint.Config_kP(0, frc::Preferences::GetDouble("kPPosition2eJoint"),
                                    BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.Config_kI(0, frc::Preferences::GetDouble("kIPosition2eJoint"),
                                    BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.Config_kD(0, frc::Preferences::GetDouble("kDPosition2eJoint"),
                                    BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.Config_kF(0, frc::Preferences::GetDouble("kFPosition2eJoint"),
                                    BarreConstant::kTimeoutMs);

    m_MoteurDeuxiemeJoint.SelectProfileSlot(1, 0); // vitesse
    m_MoteurDeuxiemeJoint.Config_kP(1, frc::Preferences::GetDouble("kPVitesse2eJoint"),
                                    BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.Config_kI(1, frc::Preferences::GetDouble("kIVitesse2eJoint"),
                                    BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.Config_kD(1, frc::Preferences::GetDouble("kDVitesse2eJoint"),
                                    BarreConstant::kTimeoutMs);
    m_MoteurDeuxiemeJoint.Config_kF(1, frc::Preferences::GetDouble("kFVitesse2eJoint"),
                                    BarreConstant::kTimeoutMs);

    m_MoteurPremierJoint.ConfigVoltageCompSaturation(BarreConstant::kVoltageCompensation);
    m_MoteurPremierJoint.EnableVoltageCompensation(true);

    m_MoteurDeuxiemeJoint.ConfigVoltageCompSaturation(BarreConstant::kVoltageCompensation);
    m_MoteurDeuxiemeJoint.EnableVoltageCompensation(true);

    m_MoteurPremierJoint.ConfigPeakCurrentLimit(BarreConstant::kPeakCurrentLimit);
    m_MoteurPremierJoint.ConfigPeakCurrentLimit(BarreConstant::kPeakCurrentDuration);
    m_MoteurPremierJoint.ConfigContinuousCurrentLimit(BarreConstant::kContinuousCurrent);
    m_MoteurPremierJoint.EnableCurrentLimit(true);

    m_MoteurDeuxiemeJoint.ConfigPeakCurrentLimit(BarreConstant::kPeakCurrentLimit);
    m_MoteurDeuxiemeJoint.ConfigPeakCurrentLimit(BarreConstant::kPeakCurrentDuration);
    m_MoteurDeuxiemeJoint.ConfigContinuousCurrentLimit(BarreConstant::kContinuousCurrent);
    m_MoteurDeuxiemeJoint.EnableCurrentLimit(true);
}

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
}