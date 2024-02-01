#include "commands/ShootNote.h"

ShootNote::ShootNote(Shooter *p_Shooter) : m_pShooter{p_Shooter} {
    AddRequirements({m_pShooter});
    hasNoteGoneThroughShooter = false;
}

void ShootNote::Initialize() { m_pShooter->SetWheelSpeeds(50); }

void ShootNote::Execute() {
    if (!hasNoteGoneThroughShooter && m_pShooter->IsObjectInShooter()) {
        hasNoteGoneThroughShooter = true;
    }
}

bool ShootNote::IsFinished() {
    if (hasNoteGoneThroughShooter && !m_pShooter->IsObjectInShooter()) {
        return true;
    }
    return false;
}

void ShootNote::End(bool interrupted) { m_pShooter->SetWheelSpeeds(0); }
