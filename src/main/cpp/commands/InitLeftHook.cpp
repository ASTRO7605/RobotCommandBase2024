#include "commands/InitLeftHook.h"

InitLeftHook::InitLeftHook(LeftHook *p_LeftHook) : m_pLeftHook{p_LeftHook} {
    AddRequirements({m_pLeftHook});
}

void InitLeftHook::Initialize() {
    m_Timer.Restart();
    frc2::PrintCommand("init").Schedule();
}

void InitLeftHook::Execute() {
    m_pLeftHook->ManualLeftHook(ClimberConstant::kPourcentageInitHooks);
}

bool InitLeftHook::IsFinished() {
    if (m_pLeftHook->IsLeftHookStopped() && (m_Timer.Get() >= ClimberConstant::kTimeDelayForInit)) {
        return true;
    }
    return false;
}

void InitLeftHook::End(bool interrupted) {
    m_pLeftHook->SetLeftHookEncoderPosition(ClimberConstant::kPositionInitReset);
    m_pLeftHook->SetLeftHookPosition(ClimberConstant::kPositionAfterInit);
    m_pLeftHook->SetInitDone();
}
