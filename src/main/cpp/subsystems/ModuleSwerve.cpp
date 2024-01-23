// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ModuleSwerve.h"

ModuleSwerve::ModuleSwerve(int TurningMotorID, int DrivingMotorID,
                           int CANcoderID)
    : m_TurningMotor{TurningMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_DrivingMotor{DrivingMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_TurningCANcoder{CANcoderID},
      m_TurningSparkMaxEncoder{m_TurningMotor.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_DrivingEncoder{m_DrivingMotor.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_TurningPIDController{m_TurningMotor.GetPIDController()},
      m_DrivingPIDController{m_DrivingMotor.GetPIDController()},
      m_DesiredState{units::meters_per_second_t{0.0}, frc::Rotation2d()} {
  m_TurningMotor.RestoreFactoryDefaults();
  m_DrivingMotor.RestoreFactoryDefaults();

  m_TurningMotor.SetInverted(true);
  m_DrivingMotor.SetInverted(false);

  m_TurningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_DrivingMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_DrivingEncoder.SetPositionConversionFactor(
      DriveConstant::kWheelCirconfM /
      DriveConstant::kDrivingGearRatio); // from motor rotations to meters
  m_DrivingEncoder.SetVelocityConversionFactor(
      (DriveConstant::kWheelCirconfM / DriveConstant::kDrivingGearRatio) /
      60); // m/s
  m_DrivingEncoder.SetPosition(0);

  m_TurningSparkMaxEncoder.SetPositionConversionFactor(
      (2 * std::numbers::pi) /
      DriveConstant::kTurningGearRatio); // from motor rotations to radians
  m_TurningSparkMaxEncoder.SetVelocityConversionFactor(
      ((2 * std::numbers::pi) / DriveConstant::kTurningGearRatio) /
      60); // radians/s
  m_TurningSparkMaxEncoder.SetPosition(
      units::radian_t{m_TurningCANcoder.GetAbsolutePosition().GetValue()}
          .value()); // seed les spark max avec la valeur du CANcoder

  m_TurningPIDController.SetPositionPIDWrappingEnabled(
      true); // PID controller can go through 0 to get to setpoint
  m_TurningPIDController.SetPositionPIDWrappingMinInput(0);
  m_TurningPIDController.SetPositionPIDWrappingMaxInput(std::numbers::pi * 2);

  m_DrivingPIDController.SetP(DriveConstant::kPDriving);
  m_DrivingPIDController.SetI(DriveConstant::kIDriving);
  m_DrivingPIDController.SetD(DriveConstant::kDDriving);
  m_DrivingPIDController.SetFF(DriveConstant::kFFDriving);
  m_DrivingPIDController.SetOutputRange(DriveConstant::kDrivingMinInput,
                                        DriveConstant::kDrivingMaxInput);

  m_TurningPIDController.SetP(DriveConstant::kPTurning);
  m_TurningPIDController.SetI(DriveConstant::kITurning);
  m_TurningPIDController.SetD(DriveConstant::kDTurning);
  m_TurningPIDController.SetFF(DriveConstant::kFFTurning);
  m_TurningPIDController.SetOutputRange(DriveConstant::kTurningMinInput,
                                        DriveConstant::kTurningMaxInput);

  m_DrivingMotor.SetSmartCurrentLimit(DriveConstant::currentLimit);
  m_TurningMotor.SetSmartCurrentLimit(DriveConstant::currentLimit);

  m_DrivingMotor.BurnFlash();
  m_TurningMotor.BurnFlash();

  m_DesiredState.angle = frc::Rotation2d(units::radian_t{
      m_TurningCANcoder.GetAbsolutePosition()
          .GetValue()}); // on dit aux swerves de garder leur position initiale
}

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
          desiredState, frc::Rotation2d{units::radian_t{
                            m_TurningSparkMaxEncoder.GetPosition()}})};

  m_DrivingPIDController.SetReference(optimizedDesiredState.speed.value(),
                                      rev::CANSparkMax::ControlType::kVelocity);

  m_TurningPIDController.SetReference(
      optimizedDesiredState.angle.Radians().value(),
      rev::CANSparkMax::ControlType::kPosition);

  m_DesiredState = desiredState;
}

void ModuleSwerve::ResetEncoders() { m_DrivingEncoder.SetPosition(0); }

double ModuleSwerve::GetCANcoderAbsolutePosition() {
  return units::radian_t{m_TurningCANcoder.GetAbsolutePosition().GetValue()}
      .value();
}

double ModuleSwerve::GetCANcoderPosition() {
  return units::radian_t{m_TurningCANcoder.GetPosition().GetValue()}.value();
}

double ModuleSwerve::GetTurningSparkMaxPosition() {
  return m_TurningSparkMaxEncoder.GetPosition();
}

double ModuleSwerve::GetDrivingPosition() {
  return m_DrivingEncoder.GetPosition();
}

double ModuleSwerve::GetDrivingVelocity() {
  return m_DrivingEncoder.GetVelocity();
}