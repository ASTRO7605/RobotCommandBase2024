#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/ShooterAngle.h>

class ShooterPosition : public frc2::CommandHelper<frc2::Command, ShooterPosition> {
  private:
    ShooterAngle *m_pShooterAngle;
    double angle;
    bool endAutomatically;

  public:
    explicit ShooterPosition(ShooterAngle *p_ShooterAngle, double angle, bool endAutomatically);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};