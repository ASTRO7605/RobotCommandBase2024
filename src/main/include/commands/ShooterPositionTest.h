#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/ShooterAngle.h>

class ShooterPositionTest : public frc2::CommandHelper<frc2::Command, ShooterPositionTest> {
  private:
    ShooterAngle *m_pShooterAngle;
    double angle;

  public:
    explicit ShooterPositionTest(ShooterAngle *p_ShooterAngle, double angle);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};