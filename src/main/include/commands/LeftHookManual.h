#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Climber.h>

class LeftHookManual : public frc2::CommandHelper<frc2::Command, LeftHookManual> {
  private:
    Climber *m_pClimber;
    double percent;

  public:
    explicit LeftHookManual(Climber *p_Climber, double percent);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};