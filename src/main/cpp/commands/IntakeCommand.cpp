#include "commands/IntakeCommand.h"

IntakeCommand::IntakeCommand(Intake *p_Intake) : m_pIntake{p_Intake} {
    AddRequirements({m_pIntake});
}

void IntakeCommand::Initialize() { m_pIntake->SetIntake(true, false); }

void IntakeCommand::Execute() {}

bool IntakeCommand::IsFinished() {
    if (m_pIntake->IsObjectInIntake()) {
        return true;
    }
    return false;
}

void IntakeCommand::End(bool interrupted) { m_pIntake->SetIntake(false, false); }
