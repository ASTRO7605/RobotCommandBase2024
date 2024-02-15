#include "commands/LeftHookManual.h"

LeftHookManual::LeftHookManual(LeftHook *p_LeftHook, double percent)
    : m_pLeftHook{p_LeftHook}, percent{percent} {
    AddRequirements({m_pLeftHook});
}

void LeftHookManual::Initialize() {}

void LeftHookManual::Execute() { m_pLeftHook->ManualLeftHook(percent); }

bool LeftHookManual::IsFinished() { return false; }

void LeftHookManual::End(bool) {
    m_pLeftHook->ManualLeftHook(0);
    m_pLeftHook->KeepLeftHookPosition();
}