#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/LeftHook.h"

class LeftHookManual : public frc2::CommandHelper<frc2::Command, LeftHookManual> {
  private:
    LeftHook *m_pLeftHook;
    double percent;

  public:
    explicit LeftHookManual(LeftHook *p_LeftHook, double percent);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};