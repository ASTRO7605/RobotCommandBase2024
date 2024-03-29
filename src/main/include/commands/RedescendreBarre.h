#pragma once

#include "subsystems/Barre.h"
#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class RedescendreBarre : public frc2::CommandHelper<frc2::Command, RedescendreBarre> {
  private:
    Barre *m_pBarre;
    double premierJointTarget;
    double deuxiemeJointTarget;
    bool needForTimer;
    bool onlyBringBack2eJoint;
    frc::Timer m_Timer;

  public:
    explicit RedescendreBarre(Barre *p_Barre, bool needForTimer, bool onlyBringBack2eJoint);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};