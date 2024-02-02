#include "commands/ShootNote.h"

ShootNote::ShootNote(ShooterAngle *p_ShooterAngle, ShooterWheels *p_ShooterWheels, Intake *p_Intake,
                     double wheelSpeeds)
    : m_pShooterAngle{p_ShooterAngle}, m_pShooterWheels{p_ShooterWheels}, m_pIntake{p_Intake},
      speeds{wheelSpeeds} {
    AddRequirements({m_pShooterAngle, m_pShooterWheels, m_pIntake});
}

void ShootNote::Initialize() {
    noNote = !m_pIntake->IsObjectInIntake(); // check if empty
    if (noNote) {
        return;
    }
    m_pShooterWheels->SetWheelSpeeds(speeds);
    areWheelsReadyToShoot = false;
    hasNoteGoneThroughShooter = false;
}

void ShootNote::Execute() {
    if (noNote) {
        return;
    }
    if (!areWheelsReadyToShoot &&
        m_pShooterWheels->AreWheelsDoneAccelerating(speeds)) { // did wheels reach their target
        areWheelsReadyToShoot = true;
    }
    if (areWheelsReadyToShoot) {
        m_pIntake->SetIntake(true, false);
    }
    if (!hasNoteGoneThroughShooter && !m_pShooterWheels->IsObjectInShooter()) {
        hasNoteGoneThroughShooter = true;
    }
}

bool ShootNote::IsFinished() {
    if ((hasNoteGoneThroughShooter && !m_pShooterWheels->IsObjectInShooter()) || noNote) {
        return true;
    }
    return false;
}

void ShootNote::End(bool interrupted) {
    m_pShooterWheels->StopWheels();
    m_pIntake->SetIntake(false, false);
}
