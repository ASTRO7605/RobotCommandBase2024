#include "commands/ShooterPositionTest.h"

ShooterPositionTest::ShooterPositionTest(ShooterAngle *p_ShooterAngle, double angle)
    : m_pShooterAngle{p_ShooterAngle}, angle{angle} {
    AddRequirements({m_pShooterAngle});
}

void ShooterPositionTest::Initialize() {}

void ShooterPositionTest::Execute() { m_pShooterAngle->SetShooterAngle(angle); }

bool ShooterPositionTest::IsFinished() { return false; }

void ShooterPositionTest::End(bool) {}