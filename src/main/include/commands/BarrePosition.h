#pragma once

#include "subsystems/Barre.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class BarrePosition : public frc2::CommandHelper<frc2::Command, BarrePosition> {
  private:
    Barre *m_pBarre;
    double angle1erJoint;
    double angle2eJoint;

  public:
    explicit BarrePosition(Barre *p_Barre, double angle1erJoint, double angle2eJoint);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};