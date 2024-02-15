#include "commands/RightHookManual.h"

RightHookManual::RightHookManual(RightHook *p_RightHook, double percent)
    : m_pRightHook{p_RightHook}, percent{percent} {
    AddRequirements({m_pRightHook});
}

void RightHookManual::Initialize() {}

void RightHookManual::Execute() { 
    m_pRightHook->ManualRightHook(percent); 
}

bool RightHookManual::IsFinished() { return false; }

void RightHookManual::End(bool) { m_pRightHook->ManualRightHook(0); }