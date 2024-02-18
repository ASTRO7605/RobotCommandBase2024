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

            double turn{};

            if (m_Base.IsRotationBeingControlled()) {
                turn = -units::radians_per_second_t{m_Base.GetPIDControlledRotationSpeedToSpeaker()}
                            .value();
                if (turn > 1) {
                    turn = 1;
                } else if (turn < -1) {
                    turn = -1;
                }
            } else {
                turn = frc::ApplyDeadband(m_TurnStick.GetX(),
                                          DriveConstant::kControllerRotationDeadband);

                turn *= (turn * turn);
            }
            // Drive by reconverting polar vector to cartesian
            m_Base.Drive(-units::meters_per_second_t{dir_r * std::sin(dir_theta)}, // forward
                         -units::meters_per_second_t{dir_r * std::cos(dir_theta)}, // sideways
                         -units::radians_per_second_t{turn}, true);
        },
        {&m_Base}));

    ConfigurePathfind();
}

void RobotContainer::Periodic() {
    frc::SmartDashboard::PutNumber("interpolatedAngle", m_ShooterAngle.GetInterpolatedShooterAngle(
                                                            m_Base.GetDistanceToSpeaker().value()));
    frc::SmartDashboard::PutNumber("interpolatedSpeed", m_ShooterWheels.GetInterpolatedWheelSpeeds(
                                                            m_Base.GetDistanceToSpeaker().value()));
}

void RobotContainer::Periodic() {
    frc::SmartDashboard::PutNumber("interpolatedAngle", m_ShooterAngle.GetInterpolatedShooterAngle(
                                                            m_Base.GetDistanceToSpeaker().value()));
    frc::SmartDashboard::PutNumber("interpolatedSpeed", m_ShooterWheels.GetInterpolatedWheelSpeeds(
                                                            m_Base.GetDistanceToSpeaker().value()));
}

void RobotContainer::ConfigureBindings() {
    // Configure your trigger bindings here
    // m_ThrottleStick.Button(8).OnTrue(
    //     frc2::InstantCommand([this]() { m_Base.SwitchRobotDrivingMode(); }).ToPtr());
    // // bouton pouce
    m_ThrottleStick.Button(2).OnTrue(
        frc2::InstantCommand([this]() { m_Base.ResetGyroTeleopOffset(); }).ToPtr());
    m_TurnStick.Button(2).OnTrue(
        frc2::InstantCommand([this]() { m_Base.SeedSwerveEncoders(); }).ToPtr());
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
    m_CoPilotController.RightBumper().OnTrue(
        frc2::InstantCommand([this]() { m_Base.SetRotationBeingControlledFlag(true); }, {})
            .ToPtr());
    m_CoPilotController.RightBumper().OnFalse(
        frc2::InstantCommand([this]() { m_Base.SetRotationBeingControlledFlag(false); }, {})
            .ToPtr());

    m_TurnStick.Button(7).WhileTrue(
        LeftHookManual(&m_LeftHook, frc::Preferences::GetDouble("kPourcentageManualHooks"))
            .ToPtr());
    m_TurnStick.Button(8).WhileTrue(
        LeftHookManual(&m_LeftHook, -frc::Preferences::GetDouble("kPourcentageManualHooks"))
            .ToPtr());
    m_TurnStick.Button(9).WhileTrue(
        RightHookManual(&m_RightHook, frc::Preferences::GetDouble("kPourcentageManualHooks"))
            .ToPtr());
    m_TurnStick.Button(10).WhileTrue(
        RightHookManual(&m_RightHook, -frc::Preferences::GetDouble("kPourcentageManualHooks"))
            .ToPtr());
    m_TurnStick.Button(11).WhileTrue(
        PremierJointManual(&m_Barre, frc::Preferences::GetDouble("kPourcentageManual1erJoint"))
            .ToPtr());
    m_TurnStick.Button(12).WhileTrue(
        PremierJointManual(&m_Barre, -frc::Preferences::GetDouble("kPourcentageManual1erJoint"))
            .ToPtr());

    m_ThrottleStick.Button(7).OnTrue(
        BarrePositionTest(&m_Barre, frc::Preferences::GetDouble("k1erJointAngleTrapApproach"),
                          frc::Preferences::GetDouble("k2eJointStartPosition"))
            .ToPtr());
    m_ThrottleStick.Button(8).OnTrue(
        BarrePositionTest(&m_Barre, frc::Preferences::GetDouble("k1erJointAngleTrapFinal"),
                          frc::Preferences::GetDouble("k2eJointStartPosition"))
            .ToPtr());
    m_ThrottleStick.Button(9).OnTrue(
        BarrePositionTest(&m_Barre, frc::Preferences::GetDouble("k1erJointAngleTrapFinal"),
                          frc::Preferences::GetDouble("k2eJointAngleTrap"))
            .ToPtr());
    m_ThrottleStick.Button(10).OnTrue(RedescendreBarre(&m_Barre, false, false).ToPtr());
    m_ThrottleStick.Button(11).OnTrue(ShooterPosition(&m_ShooterAngle, 350).ToPtr());
    m_ThrottleStick.Button(12).OnTrue(ShooterPosition(&m_ShooterAngle, 730).ToPtr());
    // m_TurnStick.Button(11).WhileTrue(
    //     DeuxiemeJointManual(&m_Barre, frc::Preferences::GetDouble("kPourcentageManual2eJoint"))
    //         .ToPtr());
    // m_TurnStick.Button(12).WhileTrue(
    //     DeuxiemeJointManual(&m_Barre, -frc::Preferences::GetDouble("kPourcentageManual2eJoint"))
    // .ToPtr());
    // m_ThrottleStick.Button(7).OnTrue(
    //     frc2::InstantCommand([this]() { m_LeftHook.SetLeftHookPosition(70); }, {&m_LeftHook})
    //         .ToPtr());
    // m_ThrottleStick.Button(8).OnTrue(
    //     frc2::InstantCommand([this]() { m_LeftHook.SetLeftHookPosition(150); }, {&m_LeftHook})
    //         .ToPtr());
    // m_ThrottleStick.Button(7).OnTrue(
    //     LeftHookPositionTest(&m_LeftHook, 70, frc::Preferences::GetDouble("kVitesseHooks"),
    //                          frc::Preferences::GetDouble("kAccelerationHooks"))
    //         .ToPtr());
    // m_ThrottleStick.Button(8).OnTrue(
    //     LeftHookPositionTest(&m_LeftHook, 150, frc::Preferences::GetDouble("kVitesseHooks"),
    //                          frc::Preferences::GetDouble("kAccelerationHooks"))
    //         .ToPtr());
    m_CoPilotController.A().WhileTrue(IntakeCommand(&m_Intake, false).ToPtr());
    (m_CoPilotController.A() && m_CoPilotController.RightTrigger(OIConstant::axisThreshold))
        .WhileTrue(IntakeCommand(&m_Intake, true).ToPtr());
    m_CoPilotController.Y().OnTrue(
        ShootNote(&m_Base, &m_ShooterAngle, &m_ShooterWheels, &m_Intake, &m_Barre, &m_LeftHook,
                  &m_RightHook, frc::Preferences::GetDouble("flywheelSpeedsTrapRPM"),
                  frc::Preferences::GetDouble("angleShooterTrap"), ScoringPositions::trap)
            .ToPtr());
    // m_CoPilotController.X().OnTrue(
    //     ShootNote(&m_Base, &m_ShooterAngle, &m_ShooterWheels, &m_Intake, &m_Barre, &m_LeftHook,
    //               &m_RightHook, frc::Preferences::GetDouble("flywheelSpeedsSpeakerRPM"),
    //               frc::Preferences::GetDouble("testAngleShooter"), ScoringPositions::speaker)
    //         .ToPtr());
    m_CoPilotController.X().OnTrue(frc2::SequentialCommandGroup{
        AlignWithSpeaker(&m_Base),
        ShootNote(&m_Base, &m_ShooterAngle, &m_ShooterWheels, &m_Intake, &m_Barre, &m_LeftHook,
                  &m_RightHook, frc::Preferences::GetDouble("flywheelSpeedsSpeakerRPM"),
                  frc::Preferences::GetDouble("testAngleShooter"), ScoringPositions::speaker)}
                                       .ToPtr());
    m_CoPilotController.B().OnTrue(
        ShootNote(&m_Base, &m_ShooterAngle, &m_ShooterWheels, &m_Intake, &m_Barre, &m_LeftHook,
                  &m_RightHook, frc::Preferences::GetDouble("flywheelSpeedsAmpRPM"),
                  frc::Preferences::GetDouble("angleShooterAmp"), ScoringPositions::amp)
            .ToPtr());

    frc2::Trigger([this] {
        return m_RightHook.IsInitScheduled();
    }).OnTrue(InitRightHook(&m_RightHook).ToPtr());

    frc2::Trigger([this] {
        return m_LeftHook.IsInitScheduled();
    }).OnTrue(InitLeftHook(&m_LeftHook).ToPtr());

    frc2::Trigger([this] {
        return m_ShooterAngle.IsShooterAngleAtInitPose();
    }).OnTrue(ShooterPosition(&m_ShooterAngle, ShooterConstant::kIntermediateAngleShooter).ToPtr());

    frc2::Trigger([this] {
        return (m_RightHook.IsInitDone() && (m_LeftHook.IsInitDone()));
    }).OnTrue(RedescendreBarre(&m_Barre, false, false).ToPtr());
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
    // m_Base.SeedSwerveEncoders();
    m_Barre.SeedEncoder1erJoint();
    m_Barre.SeedEncoder2eJoint();
    m_ShooterAngle.SeedEncoder();
}

void RobotContainer::SetIdleModeSwerve(DriveConstant::IdleMode idleMode) {
    m_Base.SetIdleMode(idleMode);
}

void RobotContainer::SetInitHooksScheduled() {
    m_RightHook.SetInitScheduled();
    m_LeftHook.SetInitScheduled();
}

bool RobotContainer::IsInitHooksDone() {
    return (m_LeftHook.IsInitDone() && m_RightHook.IsInitDone());
}

void RobotContainer::SetShooterAngleToInitPose() { m_ShooterAngle.SetShooterAngleAtInitPoseFlag(); }