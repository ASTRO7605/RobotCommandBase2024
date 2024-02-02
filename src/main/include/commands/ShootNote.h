#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Intake.h>
#include <subsystems/Shooter.h>

class ShootNote : public frc2::CommandHelper<frc2::Command, ShootNote> {
  private:
    Shooter *m_pShooter;
    Intake *m_pIntake;
    bool hasNoteGoneThroughShooter;
    bool noNote;

  public:
    explicit ShootNote(Shooter *p_Shooter, Intake *p_Intake);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};