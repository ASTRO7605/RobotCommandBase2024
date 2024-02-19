#include "commands/StartShooterWheels.h"

StartShooterWheels::StartShooterWheels(ShooterWheels *p_ShooterWheels, Base *p_Base) : m_pShooterWheels{p_ShooterWheels}, m_pBase{p_Base} {
    AddRequirements({m_pShooterWheels});
}

void StartShooterWheels::Initialize() { }

void StartShooterWheels::Execute() {
    m_pShooterWheels->SetWheelSpeeds(m_pShooterWheels->GetInterpolatedWheelSpeeds(m_pBase->GetDistanceToSpeaker().value()), true);
}

bool StartShooterWheels::IsFinished() {
    return false;
}

void StartShooterWheels::End(bool interrupted) {
        m_pShooterWheels->StopWheels();
}
