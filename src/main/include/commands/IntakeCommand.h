#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Intake.h>

class IntakeCommand : public frc2::CommandHelper<frc2::Command, IntakeCommand> {
  private:
    Intake *m_pIntake;

  public:
    explicit IntakeCommand(Intake *p_Intake);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};