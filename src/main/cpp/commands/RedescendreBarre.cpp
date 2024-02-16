#include "commands/RedescendreBarre.h"

RedescendreBarre::RedescendreBarre(Barre *p_Barre, bool needForTimer, bool onlyBringBack2eJoint)
    : m_pBarre{p_Barre}, needForTimer{needForTimer}, onlyBringBack2eJoint{onlyBringBack2eJoint} {
    AddRequirements({m_pBarre});
}

void RedescendreBarre::Initialize() {
    premierJointTarget = frc::Preferences::GetDouble("k1erJointStartPosition");
    deuxiemeJointTarget = frc::Preferences::GetDouble("k2eJointStartPosition");
    m_Timer.Restart();
}

void RedescendreBarre::Execute() {
    if (m_Timer.Get() >= BarreConstant::kTimerThreshold || !needForTimer) {
        m_pBarre->Set2eJointAngle(deuxiemeJointTarget);
        if (!onlyBringBack2eJoint) {
            m_pBarre->Set1erJointAngle(premierJointTarget);
        }
    }
}

bool RedescendreBarre::IsFinished() {
    if (m_pBarre->Is2eJointAtTargetAngle(deuxiemeJointTarget) &&
        (m_pBarre->Is1erJointAtTargetAngle(premierJointTarget) || onlyBringBack2eJoint)) {
        return true;
    }
    return false;
}

void RedescendreBarre::End(bool) {}