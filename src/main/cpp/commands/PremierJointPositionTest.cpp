#include "commands/PremierJointPositionTest.h"

PremierJointPositionTest::PremierJointPositionTest(Barre *p_Barre, double angle)
    : m_pBarre{p_Barre}, angle{angle} {
    AddRequirements({m_pBarre});
}

void PremierJointPositionTest::Initialize() {}

void PremierJointPositionTest::Execute() { m_pBarre->Set1erJointAngle(angle); }

bool PremierJointPositionTest::IsFinished() { return false; }

void PremierJointPositionTest::End(bool) {}