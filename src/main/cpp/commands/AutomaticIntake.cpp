#include "commands/AutomaticIntake.h"

AutomaticIntake::AutomaticIntake(Intake *p_Intake, Base *p_Base)
    : m_pIntake{p_Intake}, m_pBase{p_Base} {
    AddRequirements({m_pIntake, m_pBase});
}

void AutomaticIntake::Initialize() {}

void AutomaticIntake::Execute() {}

bool AutomaticIntake::IsFinished() {
    if (m_pIntake->IsObjectInIntake()){
        return true;
    }
    return false;
}

void AutomaticIntake::End(bool interrupted) {}
