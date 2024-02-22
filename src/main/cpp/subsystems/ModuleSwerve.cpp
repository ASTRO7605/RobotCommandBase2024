// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ModuleSwerve.h"

ModuleSwerve::ModuleSwerve(int TurningMotorID, int DrivingMotorID, int CANcoderID)
    : m_TurningMotor{TurningMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_DrivingMotor{DrivingMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_TurningCANcoder{CANcoderID}, m_TurningSparkMaxEncoder{m_TurningMotor.GetEncoder(
                                         rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_DrivingEncoder{m_DrivingMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_TurningPIDController{m_TurningMotor.GetPIDController()},
      m_DrivingPIDController{m_DrivingMotor.GetPIDController()}, m_MotorsInitialized{false},
      hasEncoderBeenSeeded{false} /*,
m_DesiredState{units::meters_per_second_t{0.0}, frc::Rotation2d()}*/
{}

void ModuleSwerve::Periodic() {
    // CTRE: Ok is 0
    if (m_MotorsInitialized || m_TurningCANcoder.GetPosition().SetUpdateFrequency(20_Hz) ||
            m_TurningCANcoder.OptimizeBusUtilization() ||

            m_TurningMotor.RestoreFactoryDefaults() != rev::REVLibError::kOk ||
            m_DrivingMotor.RestoreFactoryDefaults() != rev::REVLibError::kOk ||

            m_TurningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake) !=
                rev::REVLibError::kOk ||
            m_DrivingMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake) !=
                rev::REVLibError::kOk ||

            m_DrivingEncoder.SetPositionConversionFactor(DriveConstant::kWheelCirconfM /
                                                         DriveConstant::kDrivingGearRatio) !=
                rev::REVLibError::kOk || // from motor rotations to meters
            m_DrivingEncoder.SetVelocityConversionFactor(
                (DriveConstant::kWheelCirconfM / DriveConstant::kDrivingGearRatio) / 60) !=
                rev::REVLibError::kOk || // m/s
            m_DrivingEncoder.SetPosition(0) != rev::REVLibError::kOk ||
            m_TurningSparkMaxEncoder.SetPosition(
                units::radian_t{m_TurningCANcoder.GetAbsolutePosition().GetValue()}.value()) !=
                rev::REVLibError::kOk ||

            m_TurningSparkMaxEncoder.SetPositionConversionFactor(
                (2 * std::numbers::pi) /
                DriveConstant::kTurningGearRatio); // from motor rotations to radians
        m_TurningSparkMaxEncoder.SetVelocityConversionFactor(
            ((2 * std::numbers::pi) / DriveConstant::kTurningGearRatio) / 60) !=
            rev::REVLibError::kOk || // radians/s

        m_TurningPIDController.SetPositionPIDWrappingEnabled(
            true); // PID controller can go through 0 to get to setpoint
        m_TurningPIDController.SetPositionPIDWrappingMinInput(0) != rev::REVLibError::kOk ||
        m_TurningPIDController.SetPositionPIDWrappingMaxInput(std::numbers::pi * 2) !=
            rev::REVLibError::kOk ||

        m_DrivingPIDController.SetP(DriveConstant::kPDriving) != rev::REVLibError::kOk ||
        m_DrivingPIDController.SetI(DriveConstant::kIDriving) != rev::REVLibError::kOk ||
        m_DrivingPIDController.SetD(DriveConstant::kDDriving) != rev::REVLibError::kOk ||
        m_DrivingPIDController.SetFF(DriveConstant::kFFDriving) != rev::REVLibError::kOk ||
        m_DrivingPIDController.SetOutputRange(DriveConstant::kDrivingMinInput,
                                              DriveConstant::kDrivingMaxInput) !=
            rev::REVLibError::kOk ||

        m_TurningPIDController.SetP(DriveConstant::kPTurning) != rev::REVLibError::kOk ||
        m_TurningPIDController.SetI(DriveConstant::kITurning) != rev::REVLibError::kOk ||
        m_TurningPIDController.SetD(DriveConstant::kDTurning) != rev::REVLibError::kOk ||
        m_TurningPIDController.SetFF(DriveConstant::kFFTurning) != rev::REVLibError::kOk ||
        m_TurningPIDController.SetOutputRange(DriveConstant::kTurningMinInput,
                                              DriveConstant::kTurningMaxInput) !=
            rev::REVLibError::kOk ||

        m_DrivingMotor.SetSmartCurrentLimit(DriveConstant::currentLimit) != rev::REVLibError::kOk ||
        m_TurningMotor.SetSmartCurrentLimit(DriveConstant::currentLimit) != rev::REVLibError::kOk ||

        m_DrivingEncoder.SetPosition(0) != rev::REVLibError::kOk) {
    } else {
        // Stuff that doesn't allow error checking
        m_TurningMotor.SetInverted(true);
        m_DrivingMotor.SetInverted(true);
        m_MotorsInitialized = true;
    }
    // m_DesiredState.angle = frc::Rotation2d(
    //     units::radian_t{m_TurningCANcoder.GetAbsolutePosition()
    //                         .GetValue()}); // on dit aux swerves de garder leur position
    //                         initiale)
    if (m_MotorsInitialized && !hasEncoderBeenSeeded) {
        auto absoluteEncoderPose = m_TurningCANcoder.GetPosition().WaitForUpdate(1_s);
        if (absoluteEncoderPose.GetStatus().IsOK()) {
            if (m_TurningSparkMaxEncoder.SetPosition(
                    units::radian_t{absoluteEncoderPose.GetValue()}.value()) ==
                rev::REVLibError::kOk) {
                hasEncoderBeenSeeded = true;
            } else {
                frc::SmartDashboard::PutString("debug", "sparkFailed");
            }
        } else {
        }
    }
}

// void ModuleSwerve::SeedSparkMaxEncoder() {
//     m_TurningSparkMaxEncoder.SetPosition(
//         units::radian_t{m_TurningCANcoder.GetAbsolutePosition().GetValue()}
//             .value()); // seed les spark max avec la valeur du CANcoder
// }

frc::SwerveModuleState ModuleSwerve::GetState() {
    return {units::meters_per_second_t{m_DrivingEncoder.GetVelocity()},
            units::radian_t{m_TurningSparkMaxEncoder.GetPosition()}};
}

frc::SwerveModulePosition ModuleSwerve::GetPosition() {
    return {units::meter_t{m_DrivingEncoder.GetPosition()},
            units::radian_t{m_TurningSparkMaxEncoder.GetPosition()}};
}

void ModuleSwerve::SetDesiredState(frc::SwerveModuleState desiredState) {

    frc::SwerveModuleState optimizedDesiredState{
        frc::SwerveModuleState::Optimize( // permet aux roues de ne jamais aller
                                          // plus de 90 degrés à la fois
            desiredState,
            frc::Rotation2d{units::radian_t{m_TurningSparkMaxEncoder.GetPosition()}})};

    m_DrivingPIDController.SetReference(optimizedDesiredState.speed.value(),
                                        rev::CANSparkMax::ControlType::kVelocity);

    m_TurningPIDController.SetReference(optimizedDesiredState.angle.Radians().value(),
                                        rev::CANSparkMax::ControlType::kPosition);

    /*m_DesiredState = desiredState;*/
}

void ModuleSwerve::ResetEncoders() { m_DrivingEncoder.SetPosition(0); }

double ModuleSwerve::GetCANcoderAbsolutePosition() {
    return units::radian_t{m_TurningCANcoder.GetAbsolutePosition().GetValue()}.value();
}

double ModuleSwerve::GetCANcoderPosition() {
    return units::radian_t{m_TurningCANcoder.GetPosition().GetValue()}.value();
}

double ModuleSwerve::GetTurningSparkMaxPosition() { return m_TurningSparkMaxEncoder.GetPosition(); }

double ModuleSwerve::GetDrivingPosition() { return m_DrivingEncoder.GetPosition(); }

double ModuleSwerve::GetDrivingVelocity() { return m_DrivingEncoder.GetVelocity(); }

void ModuleSwerve::SetIdleMode(DriveConstant::IdleMode idleMode) {
    if (idleMode == DriveConstant::IdleMode::Brake) {
        m_DrivingMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
        m_TurningMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    } else if (idleMode == DriveConstant::IdleMode::Coast) {
        m_DrivingMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
        m_TurningMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    }
}