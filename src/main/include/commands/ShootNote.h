#pragma once

#include "subsystems/Base.h"
#include "subsystems/Intake.h"
#include "subsystems/ShooterAngle.h"
#include "subsystems/ShooterWheels.h"
#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ShootNote : public frc2::CommandHelper<frc2::Command, ShootNote> {
  private:
    Base *m_pBase;
    ShooterAngle *m_pShooterAngle;
    ShooterWheels *m_pShooterWheels;
    Intake *m_pIntake;
    bool hasNoteGoneThroughShooter;
    bool noNote;
    double targetSpeeds;
    double targetAngle;
    bool areWheelsReadyToShoot;
    bool isShooterAngledRight;
    frc::Timer timer;
    ShooterConstant::ShooterState m_State;

  public:
    /// @brief
    /// @param p_Base
    /// @param p_ShooterAngle
    /// @param p_ShooterWheels
    /// @param p_Intake
    /// @param wheelSpeeds RPM
    /// @param shooterAngle angle of the shooter (1/10 degree)
    explicit ShootNote(Base *p_Base, ShooterAngle *p_ShooterAngle,
                              ShooterWheels *p_ShooterWheels, Intake *p_Intake, double wheelSpeeds,
                              double shooterAngle);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};