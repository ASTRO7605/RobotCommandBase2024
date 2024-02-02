#pragma once

#include "subsystems/Intake.h"
#include "subsystems/ShooterAngle.h"
#include "subsystems/ShooterWheels.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ShootNote : public frc2::CommandHelper<frc2::Command, ShootNote> {
  private:
    ShooterAngle *m_pShooterAngle;
    ShooterWheels *m_pShooterWheels;
    Intake *m_pIntake;
    bool hasNoteGoneThroughShooter;
    bool noNote;
    double speeds;
    bool areWheelsReadyToShoot;

  public:
    explicit ShootNote(ShooterAngle *p_ShooterAngle, ShooterWheels *p_ShooterWheels,
                       Intake *p_Intake, double wheelSpeeds);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};