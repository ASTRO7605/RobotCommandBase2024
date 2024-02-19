#include "commands/ShooterPosition.h"

ShooterPosition::ShooterPosition(ShooterAngle *p_ShooterAngle, double angle)
    : m_pShooterAngle{p_ShooterAngle}, angle{angle} {
    AddRequirements({m_pShooterAngle});
}

void ShooterPosition::Initialize() {}

void ShooterPosition::Execute() { m_pShooterAngle->SetShooterAngle(angle); }

bool ShooterPosition::IsFinished() {
    return (m_pShooterAngle->IsShooterAtTargetAngle(angle));
}

void ShooterPosition::End(bool) {}