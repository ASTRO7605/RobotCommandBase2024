#include "commands/ShootNote.h"

ShootNote::ShootNote(Shooter *p_Shooter, Intake *p_Intake)
    : m_pShooter{p_Shooter}, m_pIntake{p_Intake} {
    AddRequirements({m_pShooter});
    hasNoteGoneThroughShooter = false;
    noNote = !m_pIntake->IsObjectInIntake(); // check if empty
}

void ShootNote::Initialize() {
    if (noNote) {
        return;
    }
    m_pShooter->SetWheelSpeeds(frc::Preferences::GetDouble("flywheelSpeedsRPM"));
}

void ShootNote::Execute() {
    if (noNote) {
        return;
    }
    if (!hasNoteGoneThroughShooter && !m_pShooter->IsObjectInShooter()) {
        hasNoteGoneThroughShooter = true;
    }
}

bool ShootNote::IsFinished() {
    if ((hasNoteGoneThroughShooter && !m_pShooter->IsObjectInShooter()) || noNote) {
        return true;
    }
    return false;
}

void ShootNote::End(bool interrupted) { m_pShooter->SetWheelSpeeds(0); }
