#pragma once

#include "commands/TimedLed.h"
#include "subsystems/Intake.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class IntakeCommand : public frc2::CommandHelper<frc2::Command, IntakeCommand> {
  private:
    Intake *m_pIntake;
    bool isReversed;
    TimedLed timed_led_command;

  public:
    explicit IntakeCommand(Intake *p_Intake, bool reversed, TimedLed timed_led_command);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};