#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Intake.h>
#include <subsystems/ShooterAngle.h>
#include <subsystems/ShooterWheels.h>

class ShootNote : public frc2::CommandHelper<frc2::Command, ShootNote> {
  private:
    ShooterAngle *m_pShooterAngle;
    ShooterWheels *m_pShooterWheels;
    Intake *m_pIntake;
    bool hasNoteGoneThroughShooter;
    bool noNote;

  public:
    explicit ShootNote(ShooterAngle *p_ShooterAngle, ShooterWheels *p_ShooterWheels,
                       Intake *p_Intake);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};