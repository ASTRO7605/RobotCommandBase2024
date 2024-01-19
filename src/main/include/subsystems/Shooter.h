#pragma once

#include <Constants.h>

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
//#include <rev/SparkRelativeEncoder.h>

class Shooter : frc2::SubsystemBase {
public:
Shooter();
void Periodic() override;
void RunShooter();
void Launch();
void Take();


private:
rev::CANSparkMax m_motor1;
rev::CANSparkMax m_motor2;
// First motor -> right side
// Second motor -> left side

rev::SparkPIDController m_motor1PIDController;
rev::SparkPIDController m_motor2PIDController;

};