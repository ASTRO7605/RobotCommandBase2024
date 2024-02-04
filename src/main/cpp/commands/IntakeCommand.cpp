#include "commands/IntakeCommand.h"

IntakeCommand::IntakeCommand(Intake *p_Intake, bool reversed)
    : m_pIntake{p_Intake}, isReversed{reversed} {
    AddRequirements({m_pIntake});
}

void IntakeCommand::Initialize() { m_pIntake->SetIntake(true, isReversed); }

void IntakeCommand::Execute() {}

bool IntakeCommand::IsFinished() {
    if (m_pIntake->IsObjectInIntake()) {
        return true;
    }
    return false;
}

void IntakeCommand::End(bool interrupted) { m_pIntake->SetIntake(false, false); }
