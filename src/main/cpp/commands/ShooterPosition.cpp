#include "commands/ShooterPosition.h"

ShooterPosition::ShooterPosition(ShooterAngle *p_ShooterAngle, double angle, bool endAutomatically)
    : m_pShooterAngle{p_ShooterAngle}, angle{angle}, endAutomatically{endAutomatically} {
    AddRequirements({m_pShooterAngle});
}

void ShooterPosition::Initialize() {}

void ShooterPosition::Execute() { m_pShooterAngle->SetShooterAngle(angle); }

bool ShooterPosition::IsFinished() {
    return (m_pShooterAngle->IsShooterAtTargetAngle(angle) && endAutomatically);
}

void ShooterPosition::End(bool) {}