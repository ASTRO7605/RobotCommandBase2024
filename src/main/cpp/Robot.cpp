// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <wpi/MemoryBuffer.h>

void Robot::RobotInit() {

    m_Container.SeedEncoders();

    wpi::PortForwarder::GetInstance().Add(5800, "photonvision-limelight.local", 5800);
    wpi::PortForwarder::GetInstance().Add(5801, "photonvision-a.local", 5800);
    wpi::PortForwarder::GetInstance().Add(5802, "photonvision-b.local", 5800);

    wpi::PortForwarder::GetInstance().Add(1181, "photonvision-limelight.local", 1181);
    wpi::PortForwarder::GetInstance().Add(1281, "photonvision-a.local", 1181);
    wpi::PortForwarder::GetInstance().Add(1381, "photonvision-b.local", 1181);

    wpi::PortForwarder::GetInstance().Add(1182, "photonvision-limelight.local", 1182);
    wpi::PortForwarder::GetInstance().Add(1282, "photonvision-a.local", 1182);
    wpi::PortForwarder::GetInstance().Add(1382, "photonvision-b.local", 1182);

    frc::Preferences::InitDouble("flywheelSpeedsAmpRPM", ShooterConstant::flywheelsSpeedAmp);
    frc::Preferences::InitDouble("flywheelSpeedsTrapRPM", ShooterConstant::flywheelsSpeedTrap);

    frc::Preferences::InitDouble("testAngleShooter", 0);
    frc::Preferences::InitDouble("angleShooterAmp", ShooterConstant::kAngleShooterAmp);
    frc::Preferences::InitDouble("angleShooterTrap", ShooterConstant::kAngleShooterTrap);

    frc::Preferences::InitDouble("kPPositionAngleLanceur", ShooterConstant::kPPositionAngle);
    frc::Preferences::InitDouble("kIPositionAngleLanceur", ShooterConstant::kIPositionAngle);
    frc::Preferences::InitDouble("kDPositionAngleLanceur", ShooterConstant::kDPositionAngle);
    frc::Preferences::InitDouble("kFPositionAngleLanceur", ShooterConstant::kFPositionAngle);

    frc::Preferences::InitDouble("kVitesseAngle", ShooterConstant::kVitesseAngle);
    frc::Preferences::InitDouble("kAccelerationAngle", ShooterConstant::kAccelerationAngle);

    frc::Preferences::InitDouble("kPourcentageManual1erJoint",
                                 BarreConstant::kPourcentageManual1erJoint);
    frc::Preferences::InitDouble("kPourcentageManual2eJoint",
                                 BarreConstant::kPourcentageManual2eJoint);

    frc::Preferences::InitDouble("kVitesse1erJoint", BarreConstant::kVitesse1erJoint);
    frc::Preferences::InitDouble("kAcceleration1erJoint", BarreConstant::kAcceleration1erJoint);

    frc::Preferences::InitDouble("kVitesse2eJoint", BarreConstant::kVitesse2eJoint);
    frc::Preferences::InitDouble("kAcceleration2eJoint", BarreConstant::kAcceleration2eJoint);

    frc::Preferences::InitDouble("k1erJointAngleTrapApproach",
                                 BarreConstant::k1erJointAngleTrapApproach);
    frc::Preferences::InitDouble("k1erJointAngleTrapIntermediaire",
                                 BarreConstant::k1erJointAngleTrapIntermediaire);
    frc::Preferences::InitDouble("k1erJointAngleTrapFinal", BarreConstant::k1erJointAngleTrapFinal);
    frc::Preferences::InitDouble("k1erJointAngleAmp", BarreConstant::k1erJointAngleAmp);
    frc::Preferences::InitDouble("k2eJointAngleTrapApproach",
                                 BarreConstant::k2eJointAngleTrapApproach);
    frc::Preferences::InitDouble("k2eJointAngleTrapFinal", BarreConstant::k2eJointAngleTrapFinal);
    frc::Preferences::InitDouble("k2eJointAngleAmpApproach",
                                 BarreConstant::k2eJointAngleAmpApproach);
    frc::Preferences::InitDouble("k2eJointAngleAmpFinal", BarreConstant::k2eJointAngleAmpFinal);
    frc::Preferences::InitDouble("k1erJointStartPosition", BarreConstant::k1erJointStartPosition);
    frc::Preferences::InitDouble("k2eJointStartPosition", BarreConstant::k2eJointStartPosition);

    frc::Preferences::InitDouble("kPHooksPosition", ClimberConstant::kPHooksPosition);
    frc::Preferences::InitDouble("kIHooksPosition", ClimberConstant::kIHooksPosition);
    frc::Preferences::InitDouble("kDHooksPosition", ClimberConstant::kDHooksPosition);
    frc::Preferences::InitDouble("kFFHooksPosition", ClimberConstant::kFFHooksPosition);

    frc::Preferences::InitDouble("kPourcentageManualHooks", ClimberConstant::kPourcentageHooks);

    frc::Preferences::InitDouble("kVitesseExtensionHooks", ClimberConstant::kVitesseExtensionHooks);
    frc::Preferences::InitDouble("kAccelerationExtensionHooks",
                                 ClimberConstant::kAccelerationExtensionHooks);
    frc::Preferences::InitDouble("kVitesseRetractionHooks",
                                 ClimberConstant::kVitesseRetractionHooks);
    frc::Preferences::InitDouble("kAccelerationRetractionHooks",
                                 ClimberConstant::kAccelerationRetractionHooks);

    frc::Preferences::InitDouble("kPThetaRobot", DriveConstant::kPThetaRobot);
    frc::Preferences::InitDouble("kIThetaRobot", DriveConstant::kIThetaRobot);
    frc::Preferences::InitDouble("kDThetaRobot", DriveConstant::kDThetaRobot);

    frc::Preferences::InitDouble("kPXYRobot", DriveConstant::kPXYRobot);
    frc::Preferences::InitDouble("kIXYRobot", DriveConstant::kIXYRobot);
    frc::Preferences::InitDouble("kDXYRobot", DriveConstant::kDXYRobot);

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
    m_Container.SetIdleModeSwerve(DriveConstant::IdleMode::Brake);
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
