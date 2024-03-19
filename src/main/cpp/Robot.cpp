// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <wpi/MemoryBuffer.h>

void Robot::RobotInit() {

    m_Container.SeedEncoders();

    wpi::PortForwarder::GetInstance().Add(5800, "limelight.local", 5800);
    wpi::PortForwarder::GetInstance().Add(5801, "limelight.local", 5801);
    wpi::PortForwarder::GetInstance().Add(5802, "limelight.local", 5802);
    wpi::PortForwarder::GetInstance().Add(5803, "limelight.local", 5803);
    wpi::PortForwarder::GetInstance().Add(5804, "limelight.local", 5804);
    wpi::PortForwarder::GetInstance().Add(5805, "limelight.local", 5805);

    wpi::PortForwarder::GetInstance().Add(5806, "photonvision-a.local", 5800);
    wpi::PortForwarder::GetInstance().Add(5807, "photonvision-b.local", 5800);

    wpi::PortForwarder::GetInstance().Add(1181, "photonvision-a.local", 1181);
    wpi::PortForwarder::GetInstance().Add(1281, "photonvision-b.local", 1181);

    wpi::PortForwarder::GetInstance().Add(1182, "photonvision-a.local", 1182);
    wpi::PortForwarder::GetInstance().Add(1282, "photonvision-b.local", 1182);

    frc::Preferences::InitDouble("TestShooterAngle", 0);
    frc::Preferences::InitDouble("TestShooterSpeeds", 0);

    hasInitHooksBeenScheduled = false;
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
void Robot::DisabledInit() {
    m_Container.SetShooterAngleToNeutral();
    m_Container.SetLedForDisabled();
    m_Container.SetIdleModeSwerve(DriveConstant::IdleMode::Brake);
    m_TimerDisabled.Restart();
    haveWheelsBeenSetToBrake = false;
}

void Robot::DisabledPeriodic() {
    if (m_TimerDisabled.Get() >= DriveConstant::kTimeBeforeBrake && !haveWheelsBeenSetToBrake) {
        m_Container.SetIdleModeSwerve(DriveConstant::IdleMode::Coast);
        haveWheelsBeenSetToBrake = true;
    }
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    m_autonomousCommand = m_Container.GetAutonomousCommand();
    m_Container.SetIdleModeSwerve(DriveConstant::IdleMode::Coast);
    m_Container.ResetGyroOffsetFromAuto();
    if (m_autonomousCommand.get() != nullptr) {
        m_autonomousCommand.Schedule();
    }
    m_Container.SetLedForEnabled();
}

void Robot::AutonomousPeriodic() {
    if (!hasInitHooksBeenScheduled) {
        m_Container.SetInitHooksScheduled();
        hasInitHooksBeenScheduled = true;
    }
}

void Robot::TeleopInit() {
    m_Container.ResetRobotOffsetFromField();
    m_Container.SetLedForEnabled();
    m_Container.SetIdleModeSwerve(DriveConstant::IdleMode::Brake);
    if (m_autonomousCommand.get() != nullptr) {
        m_autonomousCommand.Cancel();
    }
    frc2::PrintCommand("Fais des points caliss").Schedule();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
    if (!hasInitHooksBeenScheduled) {
        m_Container.SetInitHooksScheduled();
        hasInitHooksBeenScheduled = true;
    }
}

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
