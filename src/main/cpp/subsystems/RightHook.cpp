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

    m_RightHookMotor.SetInverted(false);

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

void RightHook::ManualRightHook(double percent){
    m_RightHookMotorPIDController.SetReference(percent, rev::CANSparkBase::ControlType::kDutyCycle,
                                               ClimberConstant::positionPIDSlotID,
                                               ClimberConstant::kAFHooks);
}

bool RightHook::IsRightHookAtTargetPosition(double target){
    if (fabs(m_RightHookMotorEncoder.GetPosition() - target) <=
         ClimberConstant::positionThreshold) {
            return true;
    }
    return false;
}
