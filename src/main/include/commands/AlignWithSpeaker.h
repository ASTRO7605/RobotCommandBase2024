#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Base.h"

class AlignWithSpeaker : public frc2::CommandHelper<frc2::Command, AlignWithSpeaker> {
  private:
    Base *m_pBase;
    double turn;
    frc::Timer m_Timer;

  public:
    explicit AlignWithSpeaker(Base *p_Base);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};