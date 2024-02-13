#include "commands/PremierJointManual.h"

PremierJointManual::PremierJointManual(Barre *p_Barre, double percent)
    : m_pBarre{p_Barre}, percent{percent} {
    AddRequirements({m_pBarre});
}

void PremierJointManual::Initialize() {}

void PremierJointManual::Execute() { 
    m_pBarre->Manual1erJoint(percent); 
    // m_pBarre->Manual1erJoint(100 * percent);
}

bool PremierJointManual::IsFinished() { return false; }

void PremierJointManual::End(bool) { m_pBarre->Manual1erJoint(0); }