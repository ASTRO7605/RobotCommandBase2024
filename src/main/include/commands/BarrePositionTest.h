#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Barre.h"

class BarrePositionTest : public frc2::CommandHelper<frc2::Command, BarrePositionTest> {
  private:
    Barre *m_pBarre;
    double angle1erJoint;
    double angle2eJoint;

  public:
    explicit BarrePositionTest(Barre *p_Barre, double angle1erJoint, double angle2eJoint);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};