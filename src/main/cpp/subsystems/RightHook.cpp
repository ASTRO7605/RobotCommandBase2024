#include "subsystems/RightHook.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
RightHook::RightHook()
    : m_RightHookMotor{ClimberConstant::rightHookMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_RightHookMotorEncoder{
          m_RightHookMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_RightHookMotorPIDController{m_RightHookMotor.GetPIDController()}, isInitDone{false},
      isInitScheduled{false}, m_MotorInitialized{false} {}

void RightHook::Periodic() {
    frc::SmartDashboard::PutNumber("rightHookPosition", m_RightHookMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("rightHookVelocity", m_RightHookMotorEncoder.GetVelocity());

    if (m_MotorInitialized || m_RightHookMotor.RestoreFactoryDefaults() != rev::REVLibError::kOk ||

        m_RightHookMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake) !=
            rev::REVLibError::kOk ||

        m_RightHookMotorEncoder.SetPositionConversionFactor(
            ClimberConstant::FConversionFactorPosition) != rev::REVLibError::kOk ||

        m_RightHookMotorEncoder.SetVelocityConversionFactor(
            ClimberConstant::FConversionFactorVelocity) != rev::REVLibError::kOk ||

        // position
        m_RightHookMotorPIDController.SetP(frc::Preferences::GetDouble("kPHooksPosition"),
                                           ClimberConstant::positionPIDSlotID) !=
            rev::REVLibError::kOk ||
        m_RightHookMotorPIDController.SetI(frc::Preferences::GetDouble("kIHooksPosition"),
                                           ClimberConstant::positionPIDSlotID) !=
            rev::REVLibError::kOk ||
        m_RightHookMotorPIDController.SetD(frc::Preferences::GetDouble("kDHooksPosition"),
                                           ClimberConstant::positionPIDSlotID) !=
            rev::REVLibError::kOk ||
        m_RightHookMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFHooksPosition"),
                                            ClimberConstant::positionPIDSlotID) !=
            rev::REVLibError::kOk ||

        m_RightHookMotor.EnableVoltageCompensation(ClimberConstant::kVoltageCompensation) !=
            rev::REVLibError::kOk ||

        m_RightHookMotor.SetSmartCurrentLimit(ClimberConstant::currentLimit) !=
            rev::REVLibError::kOk ||

        m_RightHookMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward,
                                      ClimberConstant::kForwardSoftLimit /
                                          ClimberConstant::FConversionFactorPosition) !=
            rev::REVLibError::kOk) {
    } else {
        // Stuff that doesn't return an error code
        m_RightHookMotor.SetInverted(false);
        m_MotorInitialized = true;
    }
}

void RightHook::SetRightHookPosition(double position) {
    m_RightHookMotorPIDController.SetReference(position, rev::CANSparkBase::ControlType::kPosition,
                                               ClimberConstant::positionPIDSlotID,
                                               ClimberConstant::kAFHooks);
}

void RightHook::ManualRightHook(double percent) {
    m_RightHookMotorPIDController.SetReference(percent, rev::CANSparkBase::ControlType::kDutyCycle,
                                               ClimberConstant::positionPIDSlotID,
                                               ClimberConstant::kAFHooks);
}

bool RightHook::IsRightHookAtTargetPosition(double target) {
    if (fabs(m_RightHookMotorEncoder.GetPosition() - target) <=
        ClimberConstant::positionThreshold) {
        return true;
    }
    return false;
}

bool RightHook::IsRightHookStopped() {
    if (fabs(m_RightHookMotorEncoder.GetVelocity()) <= ClimberConstant::kThresholdMotorStopped) {
        return true;
    }
    return false;
}

void RightHook::SetRightHookEncoderPosition(double newPosition) {
    m_RightHookMotorEncoder.SetPosition(newPosition);
}

frc::TrapezoidProfile<units::meters>::State RightHook::GetRightHookState() {
    return (frc::TrapezoidProfile<units::meters>::State{
        units::meter_t{m_RightHookMotorEncoder.GetPosition() *
                       ClimberConstant::FConversionTenthInchToMeter},
        units::meters_per_second_t{
            m_RightHookMotorEncoder.GetVelocity() *
            ClimberConstant::FConversionTenthInchPerSecondToMeterPerSecond}});
}

void RightHook::KeepRightHookPosition() {
    m_RightHookMotorPIDController.SetReference(
        m_RightHookMotorEncoder.GetPosition(), rev::CANSparkBase::ControlType::kPosition,
        ClimberConstant::positionPIDSlotID, ClimberConstant::kAFHooks);
}

bool RightHook::IsInitDone() { return isInitDone; }

void RightHook::SetInitDone() { isInitDone = true; }

bool RightHook::IsInitScheduled() { return isInitScheduled; }

void RightHook::SetInitScheduled() { isInitScheduled = true; }