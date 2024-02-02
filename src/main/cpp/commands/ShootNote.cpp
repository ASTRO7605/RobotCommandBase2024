#include "commands/ShootNote.h"

ShootNote::ShootNote(ShooterAngle *p_ShooterAngle, ShooterWheels *p_ShooterWheels, Intake *p_Intake)
    : m_pShooterAngle{p_ShooterAngle}, m_pShooterWheels{p_ShooterWheels}, m_pIntake{p_Intake} {
    AddRequirements({m_pShooterAngle, m_pShooterWheels});
    hasNoteGoneThroughShooter = false;
    noNote = !m_pIntake->IsObjectInIntake(); // check if empty
}

void ShootNote::Initialize() {
    if (noNote) {
        return;
    }
    m_pShooterWheels->SetWheelSpeeds(frc::Preferences::GetDouble("flywheelSpeedsRPM"));
}

void ShootNote::Execute() {
    if (noNote) {
        return;
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

void ShootNote::End(bool interrupted) { m_pShooterWheels->SetWheelSpeeds(0); }
