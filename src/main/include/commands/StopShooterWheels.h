#pragma once

#include "subsystems/ShooterWheels.h"
#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class StopShooterWheels : public frc2::CommandHelper<frc2::Command, StopShooterWheels> {
  private:
    ShooterWheels *m_pShooterWheels;

  public:
    explicit StopShooterWheels(ShooterWheels *p_ShooterWheels);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};