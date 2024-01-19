#include <subsystems/Shooter.h>

Shooter::Shooter() : 
m_motor1{ShooterConstant::Motor1ID, rev::CANSparkMax::MotorType::kBrushless},
m_motor2{ShooterConstant::Motor2ID, rev::CANSparkMax::MotorType::kBrushless},
m_motor1Controller{m_motor1.GetPIDController()},
m_motor2Controller{m_motor2.GetPIDController()}
{
m_motor2.SetInverted(true);
}

void Shooter::Periodic() {

}

void Shooter::RunShooter() {

}
void Shooter::Launch() {

}
void Shooter::Take() {

}
