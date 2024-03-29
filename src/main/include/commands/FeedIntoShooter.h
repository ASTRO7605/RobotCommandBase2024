#pragma once

#include "subsystems/Intake.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class FeedIntoShooter : public frc2::CommandHelper<frc2::Command, FeedIntoShooter> {
  private:
    Intake *m_pIntake;

  public:
    explicit FeedIntoShooter(Intake *p_Intake);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};