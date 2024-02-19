#include "commands/IntakeCommand.h"

IntakeCommand::IntakeCommand(Intake *p_Intake, bool reversed, TimedLed timed_led_command)
    : m_pIntake{p_Intake}, isReversed{reversed}, timed_led_command{timed_led_command} {
    AddRequirements({m_pIntake});
}

void IntakeCommand::Initialize() { m_pIntake->SetIntake(true, isReversed, false); }

void IntakeCommand::Execute() {}

bool IntakeCommand::IsFinished() {
    if (m_pIntake->IsObjectInIntake() && !isReversed) {
        timed_led_command.Schedule();
        return true;
    }
    return false;
}

void IntakeCommand::End(bool interrupted) { m_pIntake->SetIntake(false, false, false); }
