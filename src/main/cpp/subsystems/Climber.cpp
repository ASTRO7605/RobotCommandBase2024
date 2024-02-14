#include "subsystems/Climber.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
Climber::Climber()
    : m_LeftHookMotor{ClimberConstant::leftHookMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_RightHookMotor{ClimberConstant::rightHookMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_LeftHookMotorEncoder{
          m_LeftHookMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_RightHookMotorEncoder{
          m_RightHookMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_LeftHookMotorPIDController{m_LeftHookMotor.GetPIDController()},
      m_RightHookMotorPIDController{m_RightHookMotor.GetPIDController()} {
    m_LeftHookMotor.RestoreFactoryDefaults();
    m_RightHookMotor.RestoreFactoryDefaults();

    m_LeftHookMotor.SetInverted(false);
    m_RightHookMotor.SetInverted(false);

    m_LeftHookMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_RightHookMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    m_LeftHookMotorEncoder.SetPositionConversionFactor(ClimberConstant::FConversionFactorPosition);
    m_RightHookMotorEncoder.SetPositionConversionFactor(ClimberConstant::FConversionFactorPosition);

    m_LeftHookMotorEncoder.SetVelocityConversionFactor(ClimberConstant::FConversionFactorVelocity);
    m_RightHookMotorEncoder.SetVelocityConversionFactor(ClimberConstant::FConversionFactorVelocity);

    // position
    m_LeftHookMotorPIDController.SetP(frc::Preferences::GetDouble("kPHooksPosition"),
                                      ClimberConstant::positionPIDSlotID);
    m_LeftHookMotorPIDController.SetI(frc::Preferences::GetDouble("kIHooksPosition"),
                                      ClimberConstant::positionPIDSlotID);
    m_LeftHookMotorPIDController.SetD(frc::Preferences::GetDouble("kDHooksPosition"),
                                      ClimberConstant::positionPIDSlotID);
    m_LeftHookMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFHooksPosition"),
                                       ClimberConstant::positionPIDSlotID);

    // velocity
    m_LeftHookMotorPIDController.SetP(frc::Preferences::GetDouble("kPHooksVelocity"),
                                      ClimberConstant::velocityPIDSlotID);
    m_LeftHookMotorPIDController.SetI(frc::Preferences::GetDouble("kIHooksVelocity"),
                                      ClimberConstant::velocityPIDSlotID);
    m_LeftHookMotorPIDController.SetD(frc::Preferences::GetDouble("kDHooksVelocity"),
                                      ClimberConstant::velocityPIDSlotID);
    m_LeftHookMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFHooksVelocity"),
                                       ClimberConstant::velocityPIDSlotID);

    // position
    m_RightHookMotorPIDController.SetP(frc::Preferences::GetDouble("kPHooksPosition"),
                                       ClimberConstant::positionPIDSlotID);
    m_RightHookMotorPIDController.SetI(frc::Preferences::GetDouble("kIHooksPosition"),
                                       ClimberConstant::positionPIDSlotID);
    m_RightHookMotorPIDController.SetD(frc::Preferences::GetDouble("kDHooksPosition"),
                                       ClimberConstant::positionPIDSlotID);
    m_RightHookMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFHooksPosition"),
                                        ClimberConstant::positionPIDSlotID);

    // velocity
    m_RightHookMotorPIDController.SetP(frc::Preferences::GetDouble("kPHooksVelocity"),
                                       ClimberConstant::velocityPIDSlotID);
    m_RightHookMotorPIDController.SetI(frc::Preferences::GetDouble("kIHooksVelocity"),
                                       ClimberConstant::velocityPIDSlotID);
    m_RightHookMotorPIDController.SetD(frc::Preferences::GetDouble("kDHooksVelocity"),
                                       ClimberConstant::velocityPIDSlotID);
    m_RightHookMotorPIDController.SetFF(frc::Preferences::GetDouble("kFFHooksVelocity"),
                                        ClimberConstant::velocityPIDSlotID);

    m_LeftHookMotor.EnableVoltageCompensation(ClimberConstant::kVoltageCompensation);
    m_RightHookMotor.EnableVoltageCompensation(ClimberConstant::kVoltageCompensation);

    m_LeftHookMotor.SetSmartCurrentLimit(ClimberConstant::currentLimit);
    m_RightHookMotor.SetSmartCurrentLimit(ClimberConstant::currentLimit);
}

void Climber::Periodic() {
    frc::SmartDashboard::PutNumber("leftHookPosition", m_LeftHookMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("rightHookPosition", m_RightHookMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("leftHookVelocity", m_LeftHookMotorEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("rightHookVelocity", m_RightHookMotorEncoder.GetVelocity());
}

void Climber::SetLeftHookPosition(double position) {
    m_LeftHookMotorPIDController.SetReference(position, rev::CANSparkBase::ControlType::kPosition,
                                              ClimberConstant::positionPIDSlotID,
                                              ClimberConstant::kAFHooks);
}

void Climber::SetRightHookPosition(double position) {
    m_RightHookMotorPIDController.SetReference(position, rev::CANSparkBase::ControlType::kPosition,
                                               ClimberConstant::positionPIDSlotID,
                                               ClimberConstant::kAFHooks);
}

void Climber::ManualLeftHook(double percent) {
    m_LeftHookMotorPIDController.SetReference(percent, rev::CANSparkBase::ControlType::kDutyCycle,
                                              ClimberConstant::positionPIDSlotID,
                                              ClimberConstant::kAFHooks);
}

void Climber::ManualRightHook(double percent){
    m_RightHookMotorPIDController.SetReference(percent, rev::CANSparkBase::ControlType::kDutyCycle,
                                               ClimberConstant::positionPIDSlotID,
                                               ClimberConstant::kAFHooks);
}

bool Climber::IsLeftHookAtTargetPosition(double target) {
    if (fabs(m_LeftHookMotorEncoder.GetPosition() - target) <=
         ClimberConstant::positionThreshold) {
        return true;
    }
    return false;
}

bool Climber::IsRightHookAtTargetPosition(double target){
    if (fabs(m_RightHookMotorEncoder.GetPosition() - target) <=
         ClimberConstant::positionThreshold) {
            return true;
    }
    return false;
}