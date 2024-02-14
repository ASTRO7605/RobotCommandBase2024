#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Barre.h"
#include "subsystems/Intake.h"
#include "commands/RedescendreBarre.h"


class TestAmp : public frc2::CommandHelper<frc2::Command, TestAmp> {
  private:
    Barre *m_pBarre;
    Intake *m_pIntake;
    double target2eJoint;
    bool hasNoteBeenHit;
    frc2::CommandPtr m_RedescendreBarre;


  public:
    explicit TestAmp(Barre *p_Barre, Intake *p_Intake);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};