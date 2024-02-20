#include "commands/StopShooterWheels.h"

StopShooterWheels::StopShooterWheels(ShooterWheels *p_ShooterWheels)
    : m_pShooterWheels{p_ShooterWheels} {

    AddRequirements({m_pShooterWheels});
}

void StopShooterWheels::Initialize() {}

void StopShooterWheels::Execute() { m_pShooterWheels->StopWheels(); }

bool StopShooterWheels::IsFinished() { return true; }

void StopShooterWheels::End(bool interrupted) {}