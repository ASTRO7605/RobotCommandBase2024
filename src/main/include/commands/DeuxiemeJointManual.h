#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Barre.h>

class DeuxiemeJointManual : public frc2::CommandHelper<frc2::Command, DeuxiemeJointManual> {
  private:
    Barre *m_pBarre;
    double percent;

  public:
    explicit DeuxiemeJointManual(Barre *p_Barre, double percent);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};