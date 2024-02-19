#pragma once

#include "subsystems/ShooterWheels.h"
#include "subsystems/Base.h"
#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class StartShooterWheels : public frc2::CommandHelper<frc2::Command, StartShooterWheels> {
  private:
    ShooterWheels *m_pShooterWheels;
    Base *m_pBase;
  public:
    explicit StartShooterWheels(ShooterWheels *p_ShooterWheels, Base *p_Base);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};