#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Barre.h"

class PremierJointManual : public frc2::CommandHelper<frc2::Command, PremierJointManual> {
  private:
    Barre *m_pBarre;
    double percent;

  public:
    explicit PremierJointManual(Barre *p_Barre, double percent);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};