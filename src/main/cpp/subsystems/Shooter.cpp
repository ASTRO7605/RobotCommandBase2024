#include <subsystems/Shooter.h>

Shooter::Shooter() : 
m_motor1{ShooterConstant::Motor1ID, rev::CANSparkMax::MotorType::kBrushless},
m_motor2{ShooterConstant::Motor2ID, rev::CANSparkMax::MotorType::kBrushless},
m_motor1PIDController{m_motor1.GetPIDController()},
m_motor2PIDController{m_motor2.GetPIDController()}
{
m_motor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
m_motor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

m_motor2.SetInverted(true);
m_motor1PIDController.SetP(ShooterConstant::kPMotors);
m_motor1PIDController.SetI(ShooterConstant::kIMotors);
m_motor1PIDController.SetD(ShooterConstant::kDMotors);
m_motor1PIDController.SetOutputRange(ShooterConstant::kMinInput, ShooterConstant::kMaxInput);

m_motor2PIDController.SetP(ShooterConstant::kPMotors);
m_motor2PIDController.SetI(ShooterConstant::kIMotors);
m_motor2PIDController.SetD(ShooterConstant::kDMotors);
m_motor2PIDController.SetOutputRange(ShooterConstant::kMinInput, ShooterConstant::kMaxInput);


}

void Shooter::Periodic() {

}

void Shooter::RunShooterManually() {
m_motor1PIDController.SetReference(ShooterConstant::speed, rev::ControlType::kVelocity);
m_motor2PIDController.SetReference(ShooterConstant::speed, rev::ControlType::kVelocity);
}
void Shooter::StopShooter() {
m_motor1PIDController.SetReference(0, rev::ControlType::kVelocity);
m_motor2PIDController.SetReference(0, rev::ControlType::kVelocity);
}
