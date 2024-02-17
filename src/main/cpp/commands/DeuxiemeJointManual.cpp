#include "commands/DeuxiemeJointManual.h"

DeuxiemeJointManual::DeuxiemeJointManual(Barre *p_Barre, double percent)
    : m_pBarre{p_Barre}, percent{percent} {
    AddRequirements({m_pBarre});
}

void DeuxiemeJointManual::Initialize() {}

void DeuxiemeJointManual::Execute() { 
    m_pBarre->Manual2eJoint(percent); 
    // m_pBarre->Manual2eJoint(100 * percent);
}

bool DeuxiemeJointManual::IsFinished() { return false; }

void DeuxiemeJointManual::End(bool) { m_pBarre->KeepCurrentAngle2eJoint(); }