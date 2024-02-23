#pragma once

#include "commands/RedescendreBarre.h"
#include "subsystems/Barre.h"
#include "subsystems/Base.h"
#include "subsystems/Intake.h"
#include "subsystems/ShooterAngle.h"
#include "subsystems/ShooterWheels.h"
#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

class ShootNote : public frc2::CommandHelper<frc2::Command, ShootNote> {
  private:
    Base *m_pBase;
    ShooterAngle *m_pShooterAngle;
    ShooterWheels *m_pShooterWheels;
    Intake *m_pIntake;
    Barre *m_pBarre;
    frc2::CommandXboxController *m_pCoPilotController;
    double targetSpeeds;
    double currentShooterTargetAngle;
    double finalShooterTargetAngle;
    double targetPremierJoint;
    double targetDeuxiemeJoint;
    bool areWheelsAtRightSpeed;
    bool isShooterAngledRight;
    bool isPremierJointAngledRight;
    bool isDeuxiemeJointAngledRight;
    bool readyToShoot;
    frc::Timer timer;
    ShooterConstant::ShooterState m_State;
    ScoringPositions scoringPlace;
    frc2::CommandPtr m_RedescendreBarre;

  public:
    /// @brief
    /// @param p_Base
    /// @param p_ShooterAngle
    /// @param p_ShooterWheels
    /// @param p_Intake
    /// @param p_Barre
    /// @param wheelSpeeds
    /// @param shooterAngle
    /// @param scoringPlace
    explicit ShootNote(Base *p_Base, ShooterAngle *p_ShooterAngle, ShooterWheels *p_ShooterWheels,
                       Intake *p_Intake, Barre *p_Barre,
                       frc2::CommandXboxController *p_CoPilotController, double wheelSpeeds,
                       double shooterAngle, ScoringPositions scoringPlace);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};