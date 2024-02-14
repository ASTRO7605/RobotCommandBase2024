#include "commands/ShootNote.h"

ShootNote::ShootNote(Base *p_Base, ShooterAngle *p_ShooterAngle, ShooterWheels *p_ShooterWheels,
                     Intake *p_Intake, Barre *p_Barre, double wheelSpeeds, double shooterAngle,
                     ScoringPositions scoringPlace)
    : m_pBase{p_Base}, m_pShooterAngle{p_ShooterAngle}, m_pShooterWheels{p_ShooterWheels},
      m_pIntake{p_Intake}, m_pBarre{p_Barre}, targetSpeeds{wheelSpeeds}, targetAngle{shooterAngle},
      scoringPlace{scoringPlace}, m_RedescendreBarre{RedescendreBarre(m_pBarre)} {
    AddRequirements({m_pShooterAngle, m_pShooterWheels, m_pIntake, m_pBase});
}

void ShootNote::Initialize() {
    if (!m_pIntake->IsObjectInIntake()) { // check if empty
        m_State = ShooterConstant::ShooterState::noNote;
    } else {
        m_State = ShooterConstant::ShooterState::init;
    }
}

void ShootNote::Execute() {
    switch (m_State) {
    case (ShooterConstant::ShooterState::init):
        m_pShooterWheels->SetWheelSpeeds(targetSpeeds);
        m_pBase->SetWheelsInXFormation();
        areWheelsReadyToShoot = false;
        isShooterAngledRight = false;
        isPremierJointAngledRight = false;
        isDeuxiemeJointAngledRight = false;
        timer.Stop();
        timer.Reset();
        if (scoringPlace == ScoringPositions::amp) {
            targetPremierJoint = frc::Preferences::GetDouble("k1erJointAngleAmp");
            targetDeuxiemeJoint = frc::Preferences::GetDouble("k2eJointAngleAmpApproach");
        } else if (scoringPlace == ScoringPositions::trap) {
            targetPremierJoint = frc::Preferences::GetDouble("k1erJointAngleTrap");
            targetDeuxiemeJoint = frc::Preferences::GetDouble("k2eJointAngleTrapApproach");
        }
        m_State = ShooterConstant::ShooterState::waitingForSubsystems;
        break;
    case (ShooterConstant::ShooterState::waitingForSubsystems):
        if (m_pShooterWheels->AreWheelsDoneAccelerating(
                targetSpeeds)) { // did wheels reach their target
            areWheelsReadyToShoot = true;
        }
        if (m_pShooterAngle->IsShooterAtTargetAngle(targetAngle)) { // is shooter at right angle
            isShooterAngledRight = true;
        }
        if (scoringPlace != ScoringPositions::speaker){
            if (m_pBarre->Is1erJointAtTargetAngle(targetPremierJoint)) {
                isPremierJointAngledRight = true;
            }
            if (m_pBarre->Is2eJointAtTargetAngle(targetDeuxiemeJoint)) {
                isDeuxiemeJointAngledRight = true;
            }
        }
        if (areWheelsReadyToShoot && isShooterAngledRight && ((isPremierJointAngledRight &&
            isDeuxiemeJointAngledRight) || scoringPlace == ScoringPositions::speaker)) {
            m_State = ShooterConstant::ShooterState::moveNoteInShooter;
        }
        break;
    case (ShooterConstant::ShooterState::moveNoteInShooter):
        m_pIntake->SetIntake(true, false);
        m_State = ShooterConstant::ShooterState::waitingForNoteToEnter;
        break;
    case (ShooterConstant::ShooterState::waitingForNoteToEnter):
        if (m_pShooterWheels->IsObjectInShooter()) {
            m_State = ShooterConstant::ShooterState::waitingForNoteToExit;
        }
        break;
    case (ShooterConstant::ShooterState::waitingForNoteToExit):
        if (!m_pShooterWheels->IsObjectInShooter()) {
            timer.Restart();
            if (scoringPlace == ScoringPositions::amp) {
                targetDeuxiemeJoint = frc::Preferences::GetDouble("k2eJointAngleAmpFinal");
            } else if (scoringPlace == ScoringPositions::trap) {
                targetDeuxiemeJoint = frc::Preferences::GetDouble("k2eJointAngleTrapFinal");
            }
            isDeuxiemeJointAngledRight = false;
            m_State = ShooterConstant::ShooterState::waitingForEnd;
        }
        break;
    case (ShooterConstant::ShooterState::waitingForEnd):
        if (m_pBarre->Is2eJointAtTargetAngle(targetDeuxiemeJoint)){
            isDeuxiemeJointAngledRight = true;
        }
        if (timer.Get() >= ShooterConstant::timeThreshold && (isDeuxiemeJointAngledRight || scoringPlace == ScoringPositions::speaker)) {
            m_State = ShooterConstant::ShooterState::complete;
        }
        break;
    case (ShooterConstant::ShooterState::complete):
        break;
    case (ShooterConstant::ShooterState::noNote):
        break;
    }
    
    if (m_State != ShooterConstant::ShooterState::noNote) {
        m_pShooterAngle->SetShooterAngle(targetAngle); // to update kAF continuously
        if (scoringPlace != ScoringPositions::speaker) {
            m_pBarre->Set1erJointAngle(targetPremierJoint); // to update kAF continuously
            m_pBarre->Set2eJointAngle(targetDeuxiemeJoint); // to update kAF continuously
        }
    }
    
}

bool ShootNote::IsFinished() {
    if (m_State == ShooterConstant::ShooterState::complete || m_State == ShooterConstant::ShooterState::noNote) {
        return true;
    }
    return false;
}

void ShootNote::End(bool interrupted) {
    m_pShooterWheels->StopWheels();
    m_pIntake->SetIntake(false, false);
    m_pShooterAngle->KeepCurrentAngle();
    if ((scoringPlace != ScoringPositions::speaker) && (m_State != ShooterConstant::ShooterState::noNote)){
        m_RedescendreBarre.Schedule();
    }
}
