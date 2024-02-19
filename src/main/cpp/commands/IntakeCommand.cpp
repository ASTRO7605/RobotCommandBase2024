#include "commands/IntakeCommand.h"

IntakeCommand::IntakeCommand(Intake *p_Intake, bool reversed)
    : m_pIntake{p_Intake}, isReversed{reversed} {
    AddRequirements({m_pIntake});
}

void IntakeCommand::Initialize() { m_pIntake->SetIntake(true, isReversed, false); }

void IntakeCommand::Execute() {}

bool IntakeCommand::IsFinished() {
    return (m_pIntake->IsObjectInIntake() && !isReversed);
}

void IntakeCommand::End(bool interrupted) { m_pIntake->SetIntake(false, false, false); }
