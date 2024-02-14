#include "commands/RightHookManual.h"

RightHookManual::RightHookManual(Climber *p_Climber, double percent)
    : m_pClimber{p_Climber}, percent{percent} {
    AddRequirements({m_pClimber});
}

void RightHookManual::Initialize() {}

void RightHookManual::Execute() { 
    m_pClimber->ManualRightHook(percent); 
}

bool RightHookManual::IsFinished() { return false; }

void RightHookManual::End(bool) { m_pClimber->ManualRightHook(0); }