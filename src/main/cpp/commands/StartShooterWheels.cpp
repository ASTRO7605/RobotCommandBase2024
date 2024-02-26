#include "commands/StartShooterWheels.h"

StartShooterWheels::StartShooterWheels(ShooterWheels *p_ShooterWheels, Base *p_Base,
                                       bool shootInSpeaker, double desiredWheelsSpeeds)
    : m_pShooterWheels{p_ShooterWheels}, m_pBase{p_Base}, shootInSpeaker{shootInSpeaker},
      desiredWheelSpeeds{desiredWheelsSpeeds} {

    AddRequirements({m_pShooterWheels});
}

void StartShooterWheels::Initialize() {}

void StartShooterWheels::Execute() {
    if (shootInSpeaker) {
        m_pShooterWheels->SetWheelSpeeds(
            m_pShooterWheels->GetInterpolatedWheelSpeeds(m_pBase->GetDistanceToSpeaker().value()),
            true);
    } else {
        m_pShooterWheels->SetWheelSpeeds(desiredWheelSpeeds, false);
    }
}

bool StartShooterWheels::IsFinished() { return false; }

void StartShooterWheels::End(bool interrupted) {}