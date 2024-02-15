#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/RightHook.h"

class RightHookManual : public frc2::CommandHelper<frc2::Command, RightHookManual> {
  private:
    RightHook *m_pRightHook;
    double percent;

  public:
    explicit RightHookManual(RightHook *p_RightHook, double percent);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};