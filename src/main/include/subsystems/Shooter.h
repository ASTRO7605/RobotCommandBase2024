#pragma once

#include <Constants.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>

class Shooter : frc2::SubsystemBase {
public:
Shooter();
void Periodic() override;
void RunShooterManually();
void StopShooter();
frc2::CommandPtr RunShooterManuallyCommand();

private:

// First motor -> right side
// Second motor -> left side
rev::CANSparkMax m_motor1;
rev::CANSparkMax m_motor2;

rev::SparkPIDController m_motor1PIDController;
rev::SparkPIDController m_motor2PIDController;

rev::SparkRelativeEncoder m_motor1Encoder;
rev::SparkRelativeEncoder m_motor2Encoder;
};