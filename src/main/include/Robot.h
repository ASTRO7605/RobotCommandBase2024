// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include "RobotContainer.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "subsystems/Base.h"
#include <cameraserver/CameraServer.h>
#include <frc/Preferences.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <wpinet/PortForwarder.h>

#include "Constants.h"
#include <span>

class Robot : public frc::TimedRobot {
  public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

  private:
    // Have it empty by default so that if testing teleop it
    // doesn't have undefined behavior and potentially crash.
    RobotContainer m_Container;
    /* WARNING : Please, initialize an object before calling a member function */
    frc2::CommandPtr m_autonomousCommand{m_Container.GetAutonomousCommand()};
    bool hasInitHooksBeenScheduled;
};
