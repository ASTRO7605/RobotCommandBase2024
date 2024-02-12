#include "commands/ShooterAngleManual.h"

ShooterAngleManual::ShooterAngleManual(ShooterAngle *p_ShooterAngle, double percent)
    : m_pShooterAngle{p_ShooterAngle}, percent{percent} {
    AddRequirements({m_pShooterAngle});
}

void ShooterAngleManual::Initialize() {}

void ShooterAngleManual::Execute() { m_pShooterAngle->ManualShooterAngle(percent); }

bool ShooterAngleManual::IsFinished() { return false; }

void ShooterAngleManual::End(bool) { m_pShooterAngle->ManualShooterAngle(0); }