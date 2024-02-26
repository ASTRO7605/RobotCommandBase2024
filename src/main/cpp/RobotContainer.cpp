// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>

RobotContainer::RobotContainer()
    : m_ThrottleStick{OIConstant::ThrottleStickID}, m_TurnStick{OIConstant::TurnStickID},
      m_CoPilotController{OIConstant::CoPilotControllerID} {

    // Initialize all of your commands and subsystems here
    // Configure the button bindings
    ConfigureNamedCommands();
    ConfigureAmpPathfind();
    ConfigureStagePathfind();
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

            double turn = 0;

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

    m_ShooterAngle.SetDefaultCommand(frc2::RunCommand(
        [this] {
            m_ShooterAngle.SetShooterAngle(
                m_ShooterAngle.GetInterpolatedShooterAngle(m_Base.GetDistanceToSpeaker().value()));
        },
        {&m_ShooterAngle}));

    m_StartingPlaceChooser.AddOption("Source", "source_");
    m_StartingPlaceChooser.AddOption("Middle", "middle_");
    m_StartingPlaceChooser.AddOption("Amp", "amp_");

    m_AutoChooser.SetDefaultOption("2 notes", "2_notes");
    m_AutoChooser.AddOption("3 notes", "3_notes");
    m_AutoChooser.AddOption("4 notes", "4_notes");

    frc::SmartDashboard::PutData("startingPlaceChooser", &m_StartingPlaceChooser);
    frc::SmartDashboard::PutData("autoChooser", &m_AutoChooser);
}

void RobotContainer::Periodic() {
    m_Led.SetNoteInIntake(m_Intake.IsObjectInIntake());
    m_Led.SetRobotInRange(m_Base.IsRobotInRangeToShoot());
    frc::SmartDashboard::PutString("chosen auto", m_StartingPlaceChooser.GetSelected() +
                                                      m_AutoChooser.GetSelected());
}

void RobotContainer::ConfigureBindings() {
    // Configure your trigger bindings here
    m_TurnStick.Button(2).OnTrue(
        frc2::InstantCommand([this]() { m_Base.ResetGyroTeleopOffset(); }).ToPtr());
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
    // m_TurnStick.Button(11).WhileTrue(
    //     PremierJointManual(&m_Barre, frc::Preferences::GetDouble("kPourcentageManual1erJoint"))
    //         .ToPtr());
    // m_TurnStick.Button(12).WhileTrue(
    //     PremierJointManual(&m_Barre, -frc::Preferences::GetDouble("kPourcentageManual1erJoint"))
    //         .ToPtr());
    m_TurnStick.Button(11).OnTrue(ShooterPosition(&m_ShooterAngle, 300, true).ToPtr());
    m_TurnStick.Button(12).OnTrue(ShooterPosition(&m_ShooterAngle, 700, true).ToPtr());

    m_ThrottleStick.Button(2).OnTrue(
        frc2::InstantCommand([this]() { m_Base.ResetGyroTeleopOffsetPoseEstimator(); }).ToPtr());
    m_ThrottleStick.Button(5).OnTrue(
        frc2::InstantCommand([this]() { m_Base.SwitchRobotDrivingMode(); }).ToPtr());
    m_ThrottleStick.Button(7).OnTrue(
        BarrePosition(&m_Barre, frc::Preferences::GetDouble("k1erJointAngleTrapApproach"),
                      frc::Preferences::GetDouble("k2eJointStartPosition"))
            .ToPtr());
    m_ThrottleStick.Button(8).OnTrue(
        BarrePosition(&m_Barre, frc::Preferences::GetDouble("k1erJointAngleTrapIntermediaire"),
                      frc::Preferences::GetDouble("k2eJointStartPosition"))
            .ToPtr());
    m_ThrottleStick.Button(9).OnTrue(
        BarrePosition(&m_Barre, frc::Preferences::GetDouble("k1erJointAngleTrapFinal"),
                      frc::Preferences::GetDouble("k2eJointStartPosition"))
            .ToPtr());
    m_ThrottleStick.Button(10).OnTrue(RedescendreBarre(&m_Barre, false, false).ToPtr());
    m_ThrottleStick.Button(11).OnTrue(frc2::cmd::Parallel(
        RightHookPositionTest(&m_RightHook, ClimberConstant::kPositionExtendedTrap,
                              frc::Preferences::GetDouble("kVitesseExtensionHooks"),
                              frc::Preferences::GetDouble("kAccelerationExtensionHooks"))
            .ToPtr(),
        LeftHookPositionTest(&m_LeftHook, ClimberConstant::kPositionExtendedTrap,
                             frc::Preferences::GetDouble("kVitesseExtensionHooks"),
                             frc::Preferences::GetDouble("kAccelerationExtensionHooks"))
            .ToPtr()));
    m_ThrottleStick.Button(12).OnTrue(frc2::cmd::Parallel(
        RightHookPositionTest(&m_RightHook, ClimberConstant::kPositionRetracted,
                              frc::Preferences::GetDouble("kVitesseRetractionHooks"),
                              frc::Preferences::GetDouble("kAccelerationRetractionHooks"))
            .ToPtr(),
        LeftHookPositionTest(&m_LeftHook, ClimberConstant::kPositionRetracted,
                             frc::Preferences::GetDouble("kVitesseRetractionHooks"),
                             frc::Preferences::GetDouble("kAccelerationRetractionHooks"))
            .ToPtr()));

    m_CoPilotController.RightBumper().OnTrue(
        frc2::InstantCommand([this]() { m_Base.SetRotationBeingControlledFlag(true); }, {})
            .ToPtr());
    m_CoPilotController.RightBumper().WhileTrue(
        StartShooterWheels(&m_ShooterWheels, &m_Base, true).ToPtr());
    m_CoPilotController.RightBumper().OnFalse(
        frc2::InstantCommand([this]() { m_Base.SetRotationBeingControlledFlag(false); }, {})
            .ToPtr());
    m_CoPilotController.RightBumper().OnFalse(StopShooterWheels(&m_ShooterWheels).ToPtr());

    (m_CoPilotController.A() && !m_CoPilotController.RightTrigger(OIConstant::axisThreshold))
        .WhileTrue(IntakeCommand(&m_Intake, false).ToPtr());
    (m_CoPilotController.A() && m_CoPilotController.RightTrigger(OIConstant::axisThreshold))
        .WhileTrue(IntakeCommand(&m_Intake, true).ToPtr());

    (m_CoPilotController.Y() && !m_CoPilotController.LeftTrigger(OIConstant::axisThreshold) &&
     m_CoPilotController.Back())
        .OnTrue(frc2::InstantCommand([this]() { ChooseCorrectStageCommand(); }).ToPtr());

    (m_CoPilotController.Y() && m_CoPilotController.LeftTrigger(OIConstant::axisThreshold) &&
     !m_CoPilotController.Back())
        .OnTrue(ShootNote(
                    &m_Base, &m_ShooterAngle, &m_ShooterWheels, &m_Intake, &m_Barre,
                    &m_CoPilotController, frc::Preferences::GetDouble("flywheelSpeedsTrapRPM"),
                    frc::Preferences::GetDouble("angleShooterTrap"), false, ScoringPositions::trap)
                    .ToPtr());

    (m_CoPilotController.X() && !m_CoPilotController.LeftTrigger(OIConstant::axisThreshold))
        .OnTrue(frc2::SequentialCommandGroup{AlignWithSpeaker(&m_Base),
                                             ShootNote(&m_Base, &m_ShooterAngle, &m_ShooterWheels,
                                                       &m_Intake, &m_Barre, &m_CoPilotController, 0,
                                                       0, true, ScoringPositions::speaker)}
                    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming));

    (m_CoPilotController.X() && m_CoPilotController.LeftTrigger(OIConstant::axisThreshold))
        .OnTrue(ShootNote(&m_Base, &m_ShooterAngle, &m_ShooterWheels, &m_Intake, &m_Barre,
                          &m_CoPilotController,
                          frc::Preferences::GetDouble("flywheelSpeedsSpeakerRPM"),
                          frc::Preferences::GetDouble("testAngleShooter"), false,
                          ScoringPositions::speaker)
                    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming));

    (m_CoPilotController.B() && !m_CoPilotController.LeftTrigger(OIConstant::axisThreshold))
        .WhileTrue(std::move(pathfindingAmpCommand));
    (m_CoPilotController.B() && !m_CoPilotController.LeftTrigger(OIConstant::axisThreshold))
        .WhileTrue(
            ShooterPosition(&m_ShooterAngle, frc::Preferences::GetDouble("angleShooterAmp"), false)
                .ToPtr());
    (m_CoPilotController.B() && !m_CoPilotController.LeftTrigger(OIConstant::axisThreshold))
        .WhileTrue(StartShooterWheels(&m_ShooterWheels, &m_Base, false,
                                      frc::Preferences::GetDouble("flywheelSpeedsAmpRPM"))
                       .ToPtr());
    (m_CoPilotController.B() && !m_CoPilotController.LeftTrigger(OIConstant::axisThreshold))
        .OnFalse(RedescendreBarre(&m_Barre, false, false).ToPtr());
    (m_CoPilotController.B() && !m_CoPilotController.LeftTrigger(OIConstant::axisThreshold))
        .OnFalse(StopShooterWheels(&m_ShooterWheels).ToPtr());
    // manual
    (m_CoPilotController.B() && m_CoPilotController.LeftTrigger(OIConstant::axisThreshold))
        .OnTrue(ShootNote(&m_Base, &m_ShooterAngle, &m_ShooterWheels, &m_Intake, &m_Barre,
                          &m_CoPilotController, frc::Preferences::GetDouble("flywheelSpeedsAmpRPM"),
                          frc::Preferences::GetDouble("angleShooterAmp"), false,
                          ScoringPositions::amp)
                    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming));

    frc2::Trigger([this] {
        return m_RightHook.IsInitScheduled();
    }).OnTrue(InitRightHook(&m_RightHook).ToPtr());

    frc2::Trigger([this] {
        return m_LeftHook.IsInitScheduled();
    }).OnTrue(InitLeftHook(&m_LeftHook).ToPtr());

    frc2::Trigger([this] {
        return (m_RightHook.IsInitDone() && (m_LeftHook.IsInitDone()));
    }).OnTrue(RedescendreBarre(&m_Barre, false, false).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    std::string currentAutonomous =
        m_StartingPlaceChooser.GetSelected() + m_AutoChooser.GetSelected();
    if (currentAutonomous != "") {
        return pathplanner::AutoBuilder::buildAuto(currentAutonomous);
    } else {
        return frc2::InstantCommand([]() {}).ToPtr();
    }
}

void RobotContainer::ConfigureAmpPathfind() {
    pathfindingAmpCommand = pathplanner::AutoBuilder::buildAuto("approach amp auto");
}

void RobotContainer::ConfigureStagePathfind() {
    pathfindingStageCommand = pathplanner::AutoBuilder::buildAuto("stage speaker side");
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

void RobotContainer::SetShooterAngleToNeutral() { m_ShooterAngle.SetMotorNeutral(); }

void RobotContainer::ConfigureNamedCommands() {
    pathplanner::NamedCommands::registerCommand(
        "barre approach amp",
        BarrePosition(&m_Barre, frc::Preferences::GetDouble("k1erJointAngleAmp"),
                      frc::Preferences::GetDouble("k2eJointAngleAmpApproach"))
            .ToPtr());

    pathplanner::NamedCommands::registerCommand(
        "shoot amp", frc2::InstantCommand([this]() { shootAmp.Schedule(); }).ToPtr());

    pathplanner::NamedCommands::registerCommand(
        "shoot trap", frc2::InstantCommand([this]() { shootTrap.Schedule(); }).ToPtr());

    pathplanner::NamedCommands::registerCommand(
        "barre approach trap",
        frc2::cmd::Parallel(BarrePosition(&m_Barre,
                                          frc::Preferences::GetDouble("k1erJointAngleTrapApproach"),
                                          frc::Preferences::GetDouble("k2eJointStartPosition"))
                                .ToPtr()));

    pathplanner::NamedCommands::registerCommand(
        "barre intermediaire trap",
        frc2::cmd::Parallel(
            BarrePosition(&m_Barre, frc::Preferences::GetDouble("k1erJointAngleTrapIntermediaire"),
                          frc::Preferences::GetDouble("k2eJointStartPosition"))
                .ToPtr()));

    pathplanner::NamedCommands::registerCommand(
        "barre final and shoot trap",
        frc2::SequentialCommandGroup(
            frc2::PrintCommand("before barre"),
            frc2::ParallelDeadlineGroup(
                frc2::WaitCommand(0.5_s),
                BarrePosition(&m_Barre, frc::Preferences::GetDouble("k1erJointAngleTrapFinal"),
                              frc::Preferences::GetDouble("k2eJointStartPosition"))),
            frc2::PrintCommand("after wait"),
            frc2::InstantCommand([this]() { shootTrap.Schedule(); }),
            frc2::PrintCommand("after shoot"))
            .ToPtr());
    pathplanner::NamedCommands::registerCommand(
        "extend crochets trap",
        frc2::cmd::Parallel(
            RightHookPositionTest(&m_RightHook, ClimberConstant::kPositionExtendedTrap,
                                  frc::Preferences::GetDouble("kVitesseExtensionHooks"),
                                  frc::Preferences::GetDouble("kAccelerationExtensionHooks"))
                .ToPtr(),
            LeftHookPositionTest(&m_LeftHook, ClimberConstant::kPositionExtendedTrap,
                                 frc::Preferences::GetDouble("kVitesseExtensionHooks"),
                                 frc::Preferences::GetDouble("kAccelerationExtensionHooks"))
                .ToPtr()));
    pathplanner::NamedCommands::registerCommand(
        "retract crochets trap",
        frc2::cmd::Parallel(
            RightHookPositionTest(&m_RightHook, ClimberConstant::kPositionRetracted,
                                  frc::Preferences::GetDouble("kVitesseRetractionHooks"),
                                  frc::Preferences::GetDouble("kAccelerationRetractionHooks"))
                .ToPtr(),
            LeftHookPositionTest(&m_LeftHook, ClimberConstant::kPositionRetracted,
                                 frc::Preferences::GetDouble("kVitesseRetractionHooks"),
                                 frc::Preferences::GetDouble("kAccelerationRetractionHooks"))
                .ToPtr()));
    pathplanner::NamedCommands::registerCommand("start intake",
                                                IntakeCommand(&m_Intake, false).ToPtr());
    pathplanner::NamedCommands::registerCommand(
        "align and shoot",
        frc2::SequentialCommandGroup{AlignWithSpeaker(&m_Base),
                                     ShootNote(&m_Base, &m_ShooterAngle, &m_ShooterWheels,
                                               &m_Intake, &m_Barre, &m_CoPilotController, 0, 0,
                                               true, ScoringPositions::speaker)}
            .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming));
    pathplanner::NamedCommands::registerCommand(
        "start shooter wheels", StartShooterWheels(&m_ShooterWheels, &m_Base, true, 0).ToPtr());
    pathplanner::NamedCommands::registerCommand(
        "adjust shooter angle",
        frc2::InstantCommand(
            [this] {
                m_ShooterAngle.SetShooterAngle(m_ShooterAngle.GetInterpolatedShooterAngle(
                    m_Base.GetDistanceToSpeaker().value()));
            },
            {&m_ShooterAngle})
            .Repeatedly());
}

void RobotContainer::ChooseCorrectStageCommand() {
    int leftCameraAprilTagID{m_Base.GetLeftCameraAprilTagID()};
    int rightCameraAprilTagID{m_Base.GetRightCameraAprilTagID()};

    if (leftCameraAprilTagID != rightCameraAprilTagID) {
        return;
    }
    auto allianceColor = frc::DriverStation::GetAlliance();
    std::string desiredCommand{""};
    if (allianceColor.has_value()) {
        if (allianceColor == frc::DriverStation::Alliance::kBlue) {
            if (leftCameraAprilTagID == VisionConstant::StageAprilTagIDs::blueSourceSide) {
                desiredCommand = "stage source side";
            } else if (leftCameraAprilTagID == VisionConstant::StageAprilTagIDs::blueMiddleSide) {
                desiredCommand = "stage middle side";
            } else if (leftCameraAprilTagID == VisionConstant::StageAprilTagIDs::blueSpeakerSide) {
                desiredCommand = "stage speaker side";
            }
        } else {
            if (leftCameraAprilTagID == VisionConstant::StageAprilTagIDs::redSourceSide) {
                desiredCommand = "stage source side";
            } else if (leftCameraAprilTagID == VisionConstant::StageAprilTagIDs::redMiddleSide) {
                desiredCommand = "stage middle side";
            } else if (leftCameraAprilTagID == VisionConstant::StageAprilTagIDs::redSpeakerSide) {
                desiredCommand = "stage speaker side";
            }
        }
        if (desiredCommand != "") {
            pathfindingStageCommand = pathplanner::AutoBuilder::buildAuto(desiredCommand);
            pathfindingStageCommand.Schedule();
        }
    }
}

// InPosition RobotContainer::IsRobotInRightPoseForAuto() {
//     std::optional<frc::Pose2d> currentPose{m_Base.GetAveragePoseFromCameras()};
//     if (!currentPose.has_value()) {
//         return InPosition{.correct_xy = false, .correct_angle = false};
//     }
//     frc::Pose2d autoStartPose{
//         pathplanner::PathPlannerAuto::getStartingPoseFromAutoFile(currentAutonomous)};

//     return InPosition{
//         .correct_xy = currentPose.value().Translation().Distance(autoStartPose.Translation()) <
//                       VisionConstant::kThresholdAutoLEDXY,
//         .correct_angle =
//             frc::InputModulus(
//                 (currentPose.value().Rotation() - autoStartPose.Rotation()).Degrees().value(),
//                 -180.0, 180.0) < VisionConstant::kThresholdAutoLEDAngle};
// }

// void RobotContainer::UpdateDisabledLed(InPosition in_position) {
//     m_Led.SetIsInStartingPositionXY(in_position.correct_xy);
//     m_Led.SetIsInStartingPositionAngle(in_position.correct_angle);
// }

void RobotContainer::ResetRobotOffsetFromField() { m_Base.ResetGyroTeleopOffsetPoseEstimator(); }