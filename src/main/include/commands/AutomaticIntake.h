#pragma once

#include "subsystems/Base.h"
#include "subsystems/Intake.h"
#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class AutomaticIntake : public frc2::CommandHelper<frc2::Command, AutomaticIntake> {
  private:
    Intake *m_pIntake;
    Base *m_pBase;
    frc::Timer m_TimerWithoutNoteSeen;
    bool isFinished;

  public:
    explicit AutomaticIntake(Intake *p_Intake, Base *p_Base);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};