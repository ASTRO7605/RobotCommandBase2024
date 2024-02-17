#pragma once

#include "subsystems/LeftHook.h"
#include <frc/Timer.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class LeftHookPositionTest : public frc2::CommandHelper<frc2::Command, LeftHookPositionTest> {
  private:
    LeftHook *m_pLeftHook;
    double position;
    double maxSpeed;
    double maxAcceleration;
    frc::Timer m_Timer;
    frc::TrapezoidProfile<units::meters> profile;
    frc::TrapezoidProfile<units::meters>::State setpoint;
    frc::TrapezoidProfile<units::meters>::State startState;
    frc::TrapezoidProfile<units::meters>::State endGoal;

  public:
    explicit LeftHookPositionTest(LeftHook *p_LeftHook, double position, double maxSpeed,
                                  double maxAcceleration);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};