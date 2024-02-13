#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Barre.h"

class DeuxiemeJointPositionTest : public frc2::CommandHelper<frc2::Command, DeuxiemeJointPositionTest> {
  private:
    Barre *m_pBarre;
    double angle;

  public:
    explicit DeuxiemeJointPositionTest(Barre *p_Barre, double angle);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};