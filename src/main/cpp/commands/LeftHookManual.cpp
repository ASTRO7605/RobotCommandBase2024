#include "commands/LeftHookManual.h"

LeftHookManual::LeftHookManual(Climber *p_Climber, double percent)
    : m_pClimber{p_Climber}, percent{percent} {
    AddRequirements({m_pClimber});
}

void LeftHookManual::Initialize() {}

void LeftHookManual::Execute() { 
    m_pClimber->ManualLeftHook(percent); 
}

bool LeftHookManual::IsFinished() { return false; }

void LeftHookManual::End(bool) { m_pClimber->ManualLeftHook(0); }