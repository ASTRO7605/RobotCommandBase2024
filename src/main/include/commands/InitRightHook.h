#pragma once

#include "subsystems/RightHook.h"
#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class InitRightHook : public frc2::CommandHelper<frc2::Command, InitRightHook> {
  private:
    RightHook *m_pRightHook;
    frc::Timer m_Timer;

  public:
    explicit InitRightHook(RightHook *p_RightHook);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};