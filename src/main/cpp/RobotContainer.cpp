// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/PrintCommand.h>

RobotContainer::RobotContainer()
    : m_ThrottleStick{OIConstant::ThrottleStickID}, m_TurnStick{OIConstant::TurnStickID},
      m_CoPilotController{OIConstant::CoPilotControllerID} {

    // Initialize all of your commands and subsystems here
    // Configure the button bindings
    ConfigureBindings();

    m_Base.SetDefaultCommand(frc2::RunCommand(
        [this] {
            double dir_x = m_ThrottleStick.GetX();
            double dir_y = m_ThrottleStick.GetY();

            // Convert cartesian vector to polar for circular deadband
            double dir_r = std::sqrt(dir_x * dir_x + dir_y * dir_y); // norm of vector
            double dir_theta = std::atan2(dir_y, dir_x);             // direction of vector (rad)

            // Cap norm and add deadband
            if (dir_r < DriveConstant::kControllerMovementDeadband) {
                dir_r = 0.0;
            } else if (dir_r > 1.0) {
                dir_r = 1.0;
            } else {
                dir_r = (dir_r - DriveConstant::kControllerMovementDeadband) /
                        (1 - DriveConstant::kControllerMovementDeadband);
            }

            dir_r *= (dir_r * dir_r);

            double turn =
                frc::ApplyDeadband(m_TurnStick.GetX(), DriveConstant::kControllerRotationDeadband);

            turn *= (turn * turn);

            frc::SmartDashboard::PutNumber("joy_r", dir_r);
            frc::SmartDashboard::PutNumber("joy_theta",
                                           dir_theta *
                                               (180 / std::numbers::pi)); // needs to be degrees
            frc::SmartDashboard::PutNumber("joy_turn", turn);
            // Drive by reconverting polar vector to cartesian
            m_Base.Drive(-units::meters_per_second_t{dir_r * std::sin(dir_theta)}, // forward
                         -units::meters_per_second_t{dir_r * std::cos(dir_theta)}, // sideways
                         -units::radians_per_second_t{turn}, true);
        },
        {&m_Base}));

    ConfigurePathfind();
}

void RobotContainer::ConfigureBindings() {
    // Configure your trigger bindings here
    m_ThrottleStick.Button(8).OnTrue(
        frc2::InstantCommand([this]() { m_Base.SwitchRobotDrivingMode(); }).ToPtr());
    // bouton pouce
    m_ThrottleStick.Button(2).OnTrue(
        frc2::InstantCommand([this]() { m_Base.ResetGyroTeleopOffset(); }).ToPtr());
    // m_CoPilotController.LeftBumper().OnTrue(
    //     frc2::InstantCommand{[this] { pathfindingCommand.Schedule(); }, {&m_Base}}.ToPtr());
    m_CoPilotController.LeftBumper().OnTrue(
        frc2::InstantCommand(
            [this]() {
                m_ShooterWheels.ManualToggleStartWheels(
                    frc::Preferences::GetDouble("flywheelSpeedsSpeakerRPM"));
            },
            {&m_ShooterWheels})
            .ToPtr());
    m_TurnStick.Button(7).WhileTrue(
        ShooterAngleManual(&m_ShooterAngle,
                           frc::Preferences::GetDouble("kPourcentageManualAngleLanceur"))
            .ToPtr());
    m_TurnStick.Button(8).WhileTrue(
        ShooterAngleManual(&m_ShooterAngle,
                           -frc::Preferences::GetDouble("kPourcentageManualAngleLanceur"))
            .ToPtr());
    m_TurnStick.Button(9).WhileTrue(
        PremierJointManual(&m_Barre, frc::Preferences::GetDouble("kPourcentageManual1erJoint"))
            .ToPtr());
    m_TurnStick.Button(10).WhileTrue(
        PremierJointManual(&m_Barre, -frc::Preferences::GetDouble("kPourcentageManual1erJoint"))
            .ToPtr());
    m_TurnStick.Button(11).WhileTrue(
        DeuxiemeJointManual(&m_Barre, frc::Preferences::GetDouble("kPourcentageManual2eJoint"))
            .ToPtr());
    m_TurnStick.Button(12).WhileTrue(
        DeuxiemeJointManual(&m_Barre, -frc::Preferences::GetDouble("kPourcentageManual2eJoint"))
            .ToPtr());
    m_ThrottleStick.Button(7).WhileTrue(ShooterPositionTest(&m_ShooterAngle, 230).ToPtr());
    m_ThrottleStick.Button(8).WhileTrue(ShooterPositionTest(&m_ShooterAngle, 700).ToPtr());
    m_ThrottleStick.Button(9).WhileTrue(PremierJointPositionTest(&m_Barre, 120).ToPtr());
    // m_ThrottleStick.Button(10).WhileTrue(PremierJointPositionTest(&m_Barre, 1030).ToPtr());
    m_ThrottleStick.Button(10).WhileTrue(DeuxiemeJointPositionTest(&m_Barre, 300).ToPtr());
    m_ThrottleStick.Button(11).WhileTrue(DeuxiemeJointPositionTest(&m_Barre, 1700).ToPtr());
    m_ThrottleStick.Button(12).WhileTrue(DeuxiemeJointPositionTest(&m_Barre, 2100).ToPtr());

    m_CoPilotController.A().WhileTrue(IntakeCommand(&m_Intake, false).ToPtr());
    m_CoPilotController.B().WhileTrue(IntakeCommand(&m_Intake, true).ToPtr());
    m_CoPilotController.X().OnTrue(
        ShootNote(&m_Base, &m_ShooterAngle, &m_ShooterWheels, &m_Intake, &m_Barre,
                  frc::Preferences::GetDouble("flywheelSpeedsSpeakerRPM"),
                  frc::Preferences::GetDouble("testAngleShooter"), ScoringPositions::speaker)
            .ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // auto path = pathplanner::PathPlannerPath::fromPathFile("1m devant");
    // return pathplanner::AutoBuilder::followPath(path);

    return pathplanner::AutoBuilder::buildAuto("amp side");
}

void RobotContainer::ConfigurePathfind() {
    auto path = pathplanner::PathPlannerPath::fromPathFile("approach amp");

    pathplanner::PathConstraints constraints =
        pathplanner::PathConstraints(2.0_mps, 2.0_mps_sq, 540_deg_per_s, 540_deg_per_s_sq);

    pathfindingCommand = pathplanner::AutoBuilder::pathfindThenFollowPath(path, constraints);
}

void RobotContainer::SeedEncoders() {
    m_Base.SeedSwerveEncoders();
    m_Barre.SeedEncoder1erJoint();
    m_Barre.SeedEncoder2eJoint();
    m_ShooterAngle.SeedEncoder();
}
