#include "commands/DeuxiemeJointPositionTest.h"

DeuxiemeJointPositionTest::DeuxiemeJointPositionTest(Barre *p_Barre, double angle)
    : m_pBarre{p_Barre}, angle{angle} {
    AddRequirements({m_pBarre});
}

void DeuxiemeJointPositionTest::Initialize() {}

void DeuxiemeJointPositionTest::Execute() { m_pBarre->Set2eJointAngle(angle); }

bool DeuxiemeJointPositionTest::IsFinished() { return false; }

void DeuxiemeJointPositionTest::End(bool) {}