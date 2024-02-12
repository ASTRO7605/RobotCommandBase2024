#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/ShooterAngle.h>

class ShooterAngleManual : public frc2::CommandHelper<frc2::Command, ShooterAngleManual> {
  private:
    ShooterAngle *m_pShooterAngle;
    double percent;

  public:
    explicit ShooterAngleManual(ShooterAngle *p_Intake, double percent);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};