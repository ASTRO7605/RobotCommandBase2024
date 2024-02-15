#include "subsystems/LeftHook.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
LeftHook::LeftHook()
    : m_LeftHookMotor{ClimberConstant::leftHookMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_LeftHookMotorEncoder{
          m_LeftHookMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_LeftHookMotorPIDController{m_LeftHookMotor.GetPIDController()} {
    m_LeftHookMotor.RestoreFactoryDefaults();

    m_LeftHookMotor.SetInverted(false);

    m_LeftHookMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    m_LeftHookMotorEncoder.SetPositionConversionFactor(ClimberConstant::FConversionFactorPosition);

    m_LeftHookMotorEncoder.SetVelocityConversionFactor(ClimberConstant::FConversionFactorVelocity);

    // position
    m_LeftHookMotorPIDController.SetP(frc::Preferences::GetDouble("kPHooksPosition"),
                                      ClimberConstant::positionPIDSlotID);
    m_LeftHookMotorPIDController.SetI(frc::Preferences::GetDouble("kIHooksPosition"),
                                      ClimberConstant::positionPIDSlotID);
    m_LeftHookMotorPIDController.SetD(frc::Preferences::GetDouble("kDHooksPosition"),
                                      ClimberConstant::positionPIDSlotID);
    m_LeftHookMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFHooksPosition"),
                                       ClimberConstant::positionPIDSlotID);

    m_LeftHookMotor.EnableVoltageCompensation(ClimberConstant::kVoltageCompensation);

    m_LeftHookMotor.SetSmartCurrentLimit(ClimberConstant::currentLimit);

    m_LeftHookMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward,
                                 ClimberConstant::kForwardSoftLimit *
                                     ClimberConstant::FConversionFactorPosition);

    isInitDone = false;
    isInitScheduled = false;
}

void LeftHook::Periodic() {
    frc::SmartDashboard::PutNumber("leftHookPosition", m_LeftHookMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("leftHookVelocity", m_LeftHookMotorEncoder.GetVelocity());
}

void LeftHook::SetLeftHookPosition(double position) {
    m_LeftHookMotorPIDController.SetReference(position, rev::CANSparkBase::ControlType::kPosition,
                                              ClimberConstant::positionPIDSlotID,
                                              ClimberConstant::kAFHooks);
}

void LeftHook::ManualLeftHook(double percent) {
    m_LeftHookMotorPIDController.SetReference(percent, rev::CANSparkBase::ControlType::kDutyCycle,
                                              ClimberConstant::positionPIDSlotID,
                                              ClimberConstant::kAFHooks);
}

bool LeftHook::IsLeftHookAtTargetPosition(double target) {
    if (fabs(m_LeftHookMotorEncoder.GetPosition() - target) <= ClimberConstant::positionThreshold) {
        return true;
    }
    return false;
}

bool LeftHook::IsLeftHookStopped() {
    if (fabs(m_LeftHookMotorEncoder.GetVelocity()) <= ClimberConstant::kThresholdMotorStopped) {
        return true;
    }
    return false;
}

void LeftHook::SetLeftHookEncoderPosition(double newPosition) {
    m_LeftHookMotorEncoder.SetPosition(newPosition);
}

frc::TrapezoidProfile<units::meters>::State LeftHook::GetLeftHookState() {
    return (frc::TrapezoidProfile<units::meters>::State{
        units::meter_t{m_LeftHookMotorEncoder.GetPosition() *
                       ClimberConstant::FConversionTenthInchToMeter},
        units::meters_per_second_t{
            m_LeftHookMotorEncoder.GetVelocity() *
            ClimberConstant::FConversionTenthInchPerSecondToMeterPerSecond}});
}

void LeftHook::KeepLeftHookPosition() {
    m_LeftHookMotorEncoder.SetPosition(m_LeftHookMotorEncoder.GetPosition());
}

bool LeftHook::IsInitDone() { return isInitDone; }

void LeftHook::SetInitDone() { isInitDone = true; }

bool LeftHook::IsInitScheduled() { return isInitScheduled; }

void LeftHook::SetInitScheduled() { isInitScheduled = true; }