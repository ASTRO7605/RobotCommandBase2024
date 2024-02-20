#include "commands/StartShooterWheels.h"

StartShooterWheels::StartShooterWheels(ShooterWheels *p_ShooterWheels, Base *p_Base, bool start)
    : m_pShooterWheels{p_ShooterWheels}, m_pBase{p_Base} {

    AddRequirements({m_pShooterWheels});
}

void StartShooterWheels::Initialize() {}

void StartShooterWheels::Execute() {
    if (start) {
        m_pShooterWheels->SetWheelSpeeds(
            m_pShooterWheels->GetInterpolatedWheelSpeeds(m_pBase->GetDistanceToSpeaker().value()),
            true);
    } else {
        m_pShooterWheels->SetWheelSpeeds(0, false);
    }
}

bool StartShooterWheels::IsFinished() {
    if (!start) {
        return true;
    }
    return false;
}

void StartShooterWheels::End(bool interrupted) {}