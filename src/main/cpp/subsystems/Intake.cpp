#include "subsystems/Intake.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
Intake::Intake()
    : m_TopMotor{IntakeConstant::topMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_BottomMotor{IntakeConstant::bottomMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_TopMotorEncoder{m_TopMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_BottomMotorEncoder{m_BottomMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
      m_TopMotorPIDController{m_TopMotor.GetPIDController()},
      m_BottomMotorPIDController{m_BottomMotor.GetPIDController()}, m_MotorsInitialized{false} {

    m_capteurInterieurIntake.reset(new frc::DigitalInput(IntakeConstant::capteurID));
}

void Intake::Periodic() {
    frc::SmartDashboard::PutBoolean("IsObjectInIntake", IsObjectInIntake());

    if (m_MotorsInitialized || m_TopMotor.RestoreFactoryDefaults() != rev::REVLibError::kOk ||
        m_BottomMotor.RestoreFactoryDefaults() != rev::REVLibError::kOk ||

        m_TopMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake) != rev::REVLibError::kOk ||
        m_BottomMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake) != rev::REVLibError::kOk ||

        m_TopMotor.EnableVoltageCompensation(IntakeConstant::kVoltageCompensation) !=
            rev::REVLibError::kOk ||
        m_BottomMotor.EnableVoltageCompensation(IntakeConstant::kVoltageCompensation) !=
            rev::REVLibError::kOk ||

        m_TopMotor.SetSmartCurrentLimit(IntakeConstant::kCurrentLimit) != rev::REVLibError::kOk ||
        m_BottomMotor.SetSmartCurrentLimit(IntakeConstant::kCurrentLimit) !=
            rev::REVLibError::kOk) {
    } else {
        // Stuff that doesn't return error codes
        m_TopMotor.SetInverted(true);
        m_BottomMotor.SetInverted(true);
        m_MotorsInitialized = true;
    }
}

bool Intake::IsObjectInIntake() {
    return !(m_capteurInterieurIntake->Get());
} // si vrai, pas d'objet

void Intake::SetIntake(bool on, bool reversed, bool forShot) {
    double voltage{};
    if (forShot || reversed) {
        voltage = IntakeConstant::kVoltageIntakeShot;
    } else {
        voltage = IntakeConstant::kVoltageIntakeCommand;
    }
    if (reversed) {
        voltage *= -1;
    }
    if (!on) {
        voltage = 0;
    }
    m_TopMotor.SetVoltage(units::voltage::volt_t{voltage});
    m_BottomMotor.SetVoltage(units::voltage::volt_t{voltage});
}