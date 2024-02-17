#pragma once

#include "commands/RedescendreBarre.h"
#include "subsystems/Barre.h"
#include "subsystems/Base.h"
#include "subsystems/Intake.h"
#include "subsystems/LeftHook.h"
#include "subsystems/RightHook.h"
#include "subsystems/ShooterAngle.h"
#include "subsystems/ShooterWheels.h"
#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>

class ShootNote : public frc2::CommandHelper<frc2::Command, ShootNote> {
  private:
    Base *m_pBase;
    ShooterAngle *m_pShooterAngle;
    ShooterWheels *m_pShooterWheels;
    Intake *m_pIntake;
    Barre *m_pBarre;
    LeftHook *m_pLeftHook;
    RightHook *m_pRightHook;
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
    bool isLeftHookAtRightPose;
    bool isRightHookAtRightPose;
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
    /// @param p_LeftHook
    /// @param p_RightHook
    /// @param wheelSpeeds
    /// @param shooterAngle
    /// @param scoringPlace
    explicit ShootNote(Base *p_Base, ShooterAngle *p_ShooterAngle, ShooterWheels *p_ShooterWheels,
                       Intake *p_Intake, Barre *p_Barre, LeftHook *p_LeftHook,
                       RightHook *p_RightHook, double wheelSpeeds, double shooterAngle,
                       ScoringPositions scoringPlace);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};