#include "commands/ShootNoteSpeaker.h"

ShootNoteSpeaker::ShootNoteSpeaker(Base *p_Base, ShooterAngle *p_ShooterAngle,
                                   ShooterWheels *p_ShooterWheels, Intake *p_Intake,
                                   double wheelSpeeds, double shooterAngle)
    : m_pBase{p_Base}, m_pShooterAngle{p_ShooterAngle}, m_pShooterWheels{p_ShooterWheels},
      m_pIntake{p_Intake}, targetSpeeds{wheelSpeeds}, targetAngle{shooterAngle} {
    AddRequirements({m_pShooterAngle, m_pShooterWheels, m_pIntake, m_pBase});
}

void ShootNoteSpeaker::Initialize() {
    if (!m_pIntake->IsObjectInIntake()) { // check if empty
        m_State = ShooterConstant::ShooterState::complete;
    } else {
        m_State = ShooterConstant::ShooterState::init;
    }
}

void ShootNoteSpeaker::Execute() {
    switch (m_State) {
    case (ShooterConstant::ShooterState::init):
        m_pShooterWheels->SetWheelSpeeds(targetSpeeds);
        m_pShooterAngle->SetShooterAngle(targetAngle);
        m_pBase->SetWheelsInXFormation();
        areWheelsReadyToShoot = false;
        isShooterAngledRight = false;
        hasNoteGoneThroughShooter = false;
        timer.Stop();
        timer.Reset();
        frc::SmartDashboard::PutBoolean("ready to shoot", false);
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
    // if (areWheelsReadyToShoot &&
    //     isShooterAngledRight) { // if both are true, move note into the shooter
    //     m_pIntake->SetIntake(true, false);
    // }
    // if (!hasNoteGoneThroughShooter &&
    //     m_pShooterWheels
    //         ->IsObjectInShooter()) { // if seeing object for first time, note is in shooter
    //     hasNoteGoneThroughShooter = true;
    // }
    // if (hasNoteGoneThroughShooter &&
    //     !m_pShooterWheels->IsObjectInShooter()) { // if has seen object but doesn't anymore
    //     timer.Start();
    // }
}

bool ShootNoteSpeaker::IsFinished() {
    if (m_State == ShooterConstant::ShooterState::complete) {
        return true;
    }
    return false;
}

void ShootNoteSpeaker::End(bool interrupted) {
    m_pShooterWheels->StopWheels();
    m_pIntake->SetIntake(false, false);
} // CONVERSION BARRE NE PAS OUBLIER
