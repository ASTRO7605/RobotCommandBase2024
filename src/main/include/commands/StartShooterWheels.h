#pragma once

#include "subsystems/Base.h"
#include "subsystems/ShooterWheels.h"
#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class StartShooterWheels : public frc2::CommandHelper<frc2::Command, StartShooterWheels> {
  private:
    ShooterWheels *m_pShooterWheels;
    Base *m_pBase;
    bool spin;
    double desiredWheelSpeeds;

  public:
    explicit StartShooterWheels(ShooterWheels *p_ShooterWheels, Base *p_Base, bool spin,
                                double desiredWheelsSpeeds = 0);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};