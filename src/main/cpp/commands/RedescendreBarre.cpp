#include "commands/RedescendreBarre.h"

RedescendreBarre::RedescendreBarre(Barre *p_Barre, bool needForTimer) : m_pBarre{p_Barre}, needForTimer{needForTimer} {
    AddRequirements({m_pBarre});
}

void RedescendreBarre::Initialize() {
    premierJointTarget = frc::Preferences::GetDouble("k1erJointStartPosition");
    deuxiemeJointTarget = frc::Preferences::GetDouble("k2eJointStartPosition");
    m_Timer.Restart();
}

void RedescendreBarre::Execute() {
    if (m_Timer.Get() >= BarreConstant::kTimerThreshold || !needForTimer){
        m_pBarre->Set1erJointAngle(premierJointTarget);
        m_pBarre->Set2eJointAngle(deuxiemeJointTarget);
    }
}

bool RedescendreBarre::IsFinished() {
    if (m_pBarre->Is1erJointAtTargetAngle(premierJointTarget) &&
        m_pBarre->Is2eJointAtTargetAngle(deuxiemeJointTarget)) {
        return true;
    }
    return false;
}

void RedescendreBarre::End(bool) {}