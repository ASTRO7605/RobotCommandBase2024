// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <wpi/MemoryBuffer.h>

void Robot::RobotInit() {
    // frc::CameraServer::StartAutomaticCapture();

    // wpi::PortForwarder::GetInstance().Add(5800, "limelight.local", 5800);
    // wpi::PortForwarder::GetInstance().Add(5801, "limelight.local", 5801);
    // wpi::PortForwarder::GetInstance().Add(5805, "limelight.local", 5805);
    frc::SmartDashboard::PutNumber("Gyro Angle", 0);
    frc::SmartDashboard::PutNumber("leftShooterMotorVelocity", 0);
    frc::SmartDashboard::PutNumber("rightShooterMotorVelocity", 0);

    frc::Preferences::InitDouble("kPFlywheel", ShooterConstant::kPFlywheel);
    frc::Preferences::InitDouble("kIFlywheel", ShooterConstant::kIFlywheel);
    frc::Preferences::InitDouble("kDFlywheel", ShooterConstant::kDFlywheel);
    frc::Preferences::InitDouble("kFFFlywheel", ShooterConstant::kFFFlywheel);
    frc::Preferences::InitDouble("kPPositionAngleLanceur", ShooterConstant::kPPositionAngle);
    frc::Preferences::InitDouble("kIPositionAngleLanceur", ShooterConstant::kIPositionAngle);
    frc::Preferences::InitDouble("kDPositionAngleLanceur", ShooterConstant::kDPositionAngle);
    frc::Preferences::InitDouble("kFPositionAngleLanceur", ShooterConstant::kFPositionAngle);
    frc::Preferences::InitDouble("kPVitesseAngleLanceur", ShooterConstant::kPVitesseAngle);
    frc::Preferences::InitDouble("kIVitesseAngleLanceur", ShooterConstant::kIVitesseAngle);
    frc::Preferences::InitDouble("kDVitesseAngleLanceur", ShooterConstant::kDVitesseAngle);
    frc::Preferences::InitDouble("kFVitesseAngleLanceur", ShooterConstant::kFVitesseAngle);
    frc::Preferences::InitDouble("kVitesseAngleLanceur", ShooterConstant::kVitesseAngle);

} /** * This function is called every 20 ms, no matter the mode. Use * this for
   * items like diagnostics that you want to run during disabled, * autonomous,
   * teleoperated and test. * <p> This runs after the mode specific periodic
   * functions, but before LiveWindow and SmartDashboard integrated updating.
   */

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    if (m_autonomousCommand.get() != nullptr) {
        m_autonomousCommand.Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand.get() != nullptr) {
        m_autonomousCommand.Cancel();
    }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
