#include "commands/ShootNote.h"

ShootNote::ShootNote(Base *p_Base, ShooterAngle *p_ShooterAngle, ShooterWheels *p_ShooterWheels,
                     Intake *p_Intake, double wheelSpeeds, double shooterAngle)
    : m_pBase{p_Base}, m_pShooterAngle{p_ShooterAngle}, m_pShooterWheels{p_ShooterWheels},
      m_pIntake{p_Intake}, targetSpeeds{wheelSpeeds}, targetAngle{shooterAngle} {
    AddRequirements({m_pShooterAngle, m_pShooterWheels, m_pIntake});
}

void ShootNote::Initialize() {
    noNote = !m_pIntake->IsObjectInIntake(); // check if empty
    if (noNote) {
        return;
    }
    m_pShooterWheels->SetWheelSpeeds(targetSpeeds);
    m_pShooterAngle->SetShooterAngle(targetAngle);
    m_pBase->SetWheelsInXFormation();
    areWheelsReadyToShoot = false;
    isShooterAngledRight = false;
    hasNoteGoneThroughShooter = false;
}

void ShootNote::Execute() {
    frc::SmartDashboard::PutBoolean("areWheelsReadyToShoot", areWheelsReadyToShoot);
    frc::SmartDashboard::PutBoolean("isShooterAngledRight", isShooterAngledRight);
    frc::SmartDashboard::PutBoolean("hasNoteGoneThroughShooter", hasNoteGoneThroughShooter); // debug
    if (noNote) {
        return;
    }
    if (!areWheelsReadyToShoot && m_pShooterWheels->AreWheelsDoneAccelerating(
                                      targetSpeeds)) { // did wheels reach their target
        areWheelsReadyToShoot = true;
    }
    if (!isShooterAngledRight &&
        m_pShooterAngle->IsShooterAtTargetAngle(targetAngle)) { // is shooter at right angle
        isShooterAngledRight = true;
    }
    if (areWheelsReadyToShoot &&
        isShooterAngledRight) { // if both are true, move note into the shooter
        m_pIntake->SetIntake(true, false);
    }
    if (!hasNoteGoneThroughShooter &&
        m_pShooterWheels
            ->IsObjectInShooter()) { // if seeing object for first time, note is in shooter
        hasNoteGoneThroughShooter = true;
    }
}

bool ShootNote::IsFinished() {
    if ((hasNoteGoneThroughShooter && !m_pShooterWheels->IsObjectInShooter()) ||
        noNote) { // if has seen object but doesn't anymore
        return true;
    }
    return false;
}

void ShootNote::End(bool interrupted) {
    m_pShooterWheels->StopWheels();
    m_pIntake->SetIntake(false, false);
}
