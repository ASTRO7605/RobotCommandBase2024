#include "commands/TestAmp.h"

TestAmp::TestAmp(Barre *p_Barre, Intake *p_Intake)
    : m_pBarre{p_Barre}, m_pIntake{p_Intake},
      m_RedescendreBarre{RedescendreBarre(m_pBarre, true, false)} {
    AddRequirements({m_pBarre});
}

void TestAmp::Initialize() {
    target2eJoint = BarreConstant::k2eJointAngleAmpApproach;
    hasNoteBeenHit = false;
}

void TestAmp::Execute() {
    if (m_pBarre->Is1erJointAtTargetAngle(BarreConstant::k1erJointAngleAmp) &&
        m_pBarre->Is2eJointAtTargetAngle(BarreConstant::k2eJointAngleAmpApproach) &&
        m_pIntake->IsObjectInIntake()) {
        hasNoteBeenHit = true;
        target2eJoint = BarreConstant::k2eJointAngleAmpFinal;
    }
    m_pBarre->Set1erJointAngle(BarreConstant::k1erJointAngleAmp);
    m_pBarre->Set2eJointAngle(target2eJoint);
}

bool TestAmp::IsFinished() {
    if (hasNoteBeenHit && m_pBarre->Is2eJointAtTargetAngle(BarreConstant::k2eJointAngleAmpFinal)) {
        return true;
    }
    return false;
}

void TestAmp::End(bool) { m_RedescendreBarre.Schedule(); }