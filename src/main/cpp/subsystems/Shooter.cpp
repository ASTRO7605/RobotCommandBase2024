#include "subsystems/Shooter.h"

Shooter::Shooter() : 
m_motor1{ShooterConstant::Motor1ID, rev::CANSparkMax::MotorType::kBrushless},
m_motor2{ShooterConstant::Motor2ID, rev::CANSparkMax::MotorType::kBrushless},
m_motor1PIDController{m_motor1.GetPIDController()},
m_motor2PIDController{m_motor2.GetPIDController()},
m_motor1Encoder{m_motor1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
m_motor2Encoder{m_motor2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)}
{
m_motor1.RestoreFactoryDefaults();
m_motor2.RestoreFactoryDefaults();

m_motor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
m_motor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

m_motor2.SetInverted(true);
m_motor1PIDController.SetP(ShooterConstant::kPMotors);
m_motor1PIDController.SetI(ShooterConstant::kIMotors);
m_motor1PIDController.SetD(ShooterConstant::kDMotors);

m_motor2PIDController.SetP(ShooterConstant::kPMotors);
m_motor2PIDController.SetI(ShooterConstant::kIMotors);
m_motor2PIDController.SetD(ShooterConstant::kDMotors);

m_motor1Encoder.SetPositionConversionFactor((1 / 42)); // 42 counts per revolution
m_motor1Encoder.SetVelocityConversionFactor((1 / 42) / 60);

m_motor2Encoder.SetPositionConversionFactor((1 / 42)); // 42 counts per revolution
m_motor2Encoder.SetVelocityConversionFactor((1 / 42) / 60);
}

void Shooter::Periodic() {
frc::SmartDashboard::PutNumber("rightMotorRotations", m_motor1Encoder.GetPosition());
frc::SmartDashboard::PutNumber("rightMotorVelocity", m_motor1Encoder.GetVelocity());

frc::SmartDashboard::PutNumber("leftMotorRotations", m_motor2Encoder.GetPosition());
frc::SmartDashboard::PutNumber("leftMotorVelocity", m_motor2Encoder.GetVelocity());
}

void Shooter::RunShooterManually() {
m_motor1PIDController.SetReference(ShooterConstant::RMPManuallyControl, rev::ControlType::kVelocity);
m_motor2PIDController.SetReference(ShooterConstant::RMPManuallyControl, rev::ControlType::kVelocity);
}
void Shooter::StopShooter() {
m_motor1PIDController.SetReference(0, rev::ControlType::kVelocity);
m_motor2PIDController.SetReference(0, rev::ControlType::kVelocity);
}

frc2::CommandPtr Shooter::RunShooterManuallyCommand() {
    return RunEnd(
        [this] {RunShooterManually();},
        [this] {StopShooter();}
    );
}
