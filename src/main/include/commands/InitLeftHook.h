#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/LeftHook.h"

class InitLeftHook : public frc2::CommandHelper<frc2::Command, InitLeftHook> {
  private:
    LeftHook *m_pLeftHook;
    frc::Timer m_Timer;

  public:
    explicit InitLeftHook(LeftHook *p_LeftHook);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};