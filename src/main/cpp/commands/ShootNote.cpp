#include "commands/ShootNote.h"

ShootNote::ShootNote(Base *p_Base, ShooterAngle *p_ShooterAngle,
                                   ShooterWheels *p_ShooterWheels, Intake *p_Intake,
                                   double wheelSpeeds, double shooterAngle)
    : m_pBase{p_Base}, m_pShooterAngle{p_ShooterAngle}, m_pShooterWheels{p_ShooterWheels},
      m_pIntake{p_Intake}, targetSpeeds{wheelSpeeds}, targetAngle{shooterAngle} {
    AddRequirements({m_pShooterAngle, m_pShooterWheels, m_pIntake, m_pBase});
}

void ShootNote::Initialize() {
    if (!m_pIntake->IsObjectInIntake()) { // check if empty
        m_State = ShooterConstant::ShooterState::complete;
    } else {
        m_State = ShooterConstant::ShooterState::init;
    }
}

void ShootNote::Execute() {
    if (m_State != ShooterConstant::ShooterState::complete){
        m_pShooterAngle->SetShooterAngle(targetAngle); // to update kAF continuously
    }
    switch (m_State) {
    case (ShooterConstant::ShooterState::init):
        m_pShooterWheels->SetWheelSpeeds(targetSpeeds);
        m_pBase->SetWheelsInXFormation();
        areWheelsReadyToShoot = false;
        isShooterAngledRight = false;
        hasNoteGoneThroughShooter = false;
        timer.Stop();
        timer.Reset();
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
        if (areWheelsReadyToShoot && isShooterAngledRight) {
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
            m_State = ShooterConstant::ShooterState::waitingForEnd;
        }
        break;
    case (ShooterConstant::ShooterState::waitingForEnd):
        if (m_pShooterWheels->IsObjectInShooter()) {
            m_State = ShooterConstant::ShooterState::waitingForNoteToExit;
        } else if (timer.Get() >= ShooterConstant::timeThreshold) {
            m_State = ShooterConstant::ShooterState::complete;
        }
        break;
    case (ShooterConstant::ShooterState::complete):
        break;
    }
}

bool ShootNote::IsFinished() {
    if (m_State == ShooterConstant::ShooterState::complete) {
        return true;
    }
    return false;
}

void ShootNote::End(bool interrupted) {
    m_pShooterWheels->StopWheels();
    m_pIntake->SetIntake(false, false);
    m_pShooterAngle->KeepCurrentAngle();
}
