// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include "commands/AlignWithSpeaker.h"
#include "commands/BarrePositionTest.h"
#include "commands/DeuxiemeJointManual.h"
#include "commands/InitLeftHook.h"
#include "commands/InitRightHook.h"
#include "commands/IntakeCommand.h"
#include "commands/LeftHookManual.h"
#include "commands/LeftHookPositionTest.h"
#include "commands/PremierJointManual.h"
#include "commands/RedescendreBarre.h"
#include "commands/RightHookManual.h"
#include "commands/RightHookPositionTest.h"
#include "commands/ShootNote.h"
#include "commands/ShooterAngleManual.h"
#include "commands/ShooterPosition.h"
#include "commands/TestAmp.h"

#include "subsystems/Barre.h"
#include "subsystems/Base.h"
#include "subsystems/Intake.h"
#include "subsystems/Led.h"
#include "subsystems/LeftHook.h"
#include "subsystems/RightHook.h"
#include "subsystems/ShooterAngle.h"
#include "subsystems/ShooterWheels.h"
#include "subsystems/Vision.h"

#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <wpi/MemoryBuffer.h>
#include <wpi/fs.h>

#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/SubsystemBase.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/PathPlannerTrajectory.h>
#include <pathplanner/lib/path/PathPoint.h>
#include <pathplanner/lib/util/PIDConstants.h>

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <networktables/NetworkTableInstance.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/Trigger.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer : public frc2::SubsystemBase {
  public:
    RobotContainer();
    void Periodic() override;
    frc2::CommandJoystick m_ThrottleStick;
    frc2::CommandJoystick m_TurnStick;
    frc2::CommandXboxController m_CoPilotController;

    frc2::CommandPtr GetAutonomousCommand();

    void SeedEncoders();
    void BringBarreDown();
    void SetInitHooksScheduled();
    bool IsInitHooksDone();
    void SetIdleModeSwerve(DriveConstant::IdleMode);
    void SetShooterAngleToInitPose();

  private:
    void ConfigureBindings();
    void ConfigurePathfind();
    // The robot's subsystems are defined here...
    Base m_Base;
    Barre m_Barre;
    ShooterAngle m_ShooterAngle;
    ShooterWheels m_ShooterWheels;
    Intake m_Intake;
    LeftHook m_LeftHook;
    RightHook m_RightHook;
    LED m_LED;
    frc2::CommandPtr pathfindingCommand{frc2::RunCommand([]() {})};
};
