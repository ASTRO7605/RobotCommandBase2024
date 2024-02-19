#pragma once

#include "subsystems/Intake.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class IntakeCommand : public frc2::CommandHelper<frc2::Command, IntakeCommand>
{
  private:
    Intake *m_pIntake;
    bool isReversed;

  public:
    explicit IntakeCommand(Intake *p_Intake, bool reversed);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};