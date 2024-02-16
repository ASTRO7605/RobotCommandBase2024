#include "commands/BarrePositionTest.h"

BarrePositionTest::BarrePositionTest(Barre *p_Barre, double angle1erJoint, double angle2eJoint)
    : m_pBarre{p_Barre}, angle1erJoint{angle1erJoint}, angle2eJoint{angle2eJoint} {
    AddRequirements({m_pBarre});
}

void BarrePositionTest::Initialize() {}

void BarrePositionTest::Execute() { 
    m_pBarre->Set1erJointAngle(angle1erJoint);
    m_pBarre->Set2eJointAngle(angle2eJoint);
}

bool BarrePositionTest::IsFinished() { 
    if (m_pBarre->Is1erJointAtTargetAngle(angle1erJoint) && m_pBarre->Is2eJointAtTargetAngle(angle2eJoint)){
        return true;
    } 
    return false;
}

void BarrePositionTest::End(bool) {}