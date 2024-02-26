#pragma once

#include "subsystems/Intake.h"
#include "subsystems/Base.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class AutomaticIntake : public frc2::CommandHelper<frc2::Command, AutomaticIntake>
{
  private:
    Intake *m_pIntake;
    Base *m_pBase;

  public:
    explicit AutomaticIntake(Intake *p_Intake, Base *p_Base);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};