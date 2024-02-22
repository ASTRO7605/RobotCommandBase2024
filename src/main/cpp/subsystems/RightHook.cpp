#include "subsystems/RightHook.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
RightHook::RightHook()
    : m_RightHookMotor{ClimberConstant::rightHookMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_RightHookMotorEncoder{
          m_RightHookMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_RightHookMotorPIDController{m_RightHookMotor.GetPIDController()} {
    m_RightHookMotor.RestoreFactoryDefaults();
    m_RightHookMotor.SetCANTimeout(50);

    // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces for docs
    // Prefer prime numbers

    m_RightHookMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 59);

    m_RightHookMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 23);

    m_RightHookMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 23);

    m_RightHookMotor.SetInverted(true);

    m_RightHookMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    m_RightHookMotorEncoder.SetPositionConversionFactor(ClimberConstant::FConversionFactorPosition);

    m_RightHookMotorEncoder.SetVelocityConversionFactor(ClimberConstant::FConversionFactorVelocity);

    // position
    m_RightHookMotorPIDController.SetP(frc::Preferences::GetDouble("kPHooksPosition"),
                                       ClimberConstant::positionPIDSlotID);
    m_RightHookMotorPIDController.SetI(frc::Preferences::GetDouble("kIHooksPosition"),
                                       ClimberConstant::positionPIDSlotID);
    m_RightHookMotorPIDController.SetD(frc::Preferences::GetDouble("kDHooksPosition"),
                                       ClimberConstant::positionPIDSlotID);
    m_RightHookMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFHooksPosition"),
                                        ClimberConstant::positionPIDSlotID);

    m_RightHookMotor.EnableVoltageCompensation(ClimberConstant::kVoltageCompensation);

    m_RightHookMotor.SetSmartCurrentLimit(ClimberConstant::currentLimit);

    m_RightHookMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward,
                                  ClimberConstant::kForwardSoftLimit);
    m_RightHookMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, true);

    isInitDone = false;
    isInitScheduled = false;
}

void RightHook::Periodic() {
    frc::SmartDashboard::PutNumber("rightHookPosition", m_RightHookMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("rightHookVelocity", m_RightHookMotorEncoder.GetVelocity());
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