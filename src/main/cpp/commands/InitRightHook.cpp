#include "commands/InitRightHook.h"

InitRightHook::InitRightHook(RightHook *p_RightHook) : m_pRightHook{p_RightHook} {
    AddRequirements({m_pRightHook});
}

void InitRightHook::Initialize() { m_Timer.Restart(); }

void InitRightHook::Execute() {
    m_pRightHook->ManualRightHook(ClimberConstant::kPourcentageInitHooks);
}

bool InitRightHook::IsFinished() {
    if (m_pRightHook->IsRightHookStopped() &&
        (m_Timer.Get() >= ClimberConstant::kTimeDelayForInit)) {
        return true;
    }
    return false;
}

void InitRightHook::End(bool interrupted) {
    m_pRightHook->SetRightHookEncoderPosition(ClimberConstant::kPositionInitReset);
    m_pRightHook->SetRightHookPosition(ClimberConstant::kPositionAfterInit);
    m_pRightHook->SetInitDone();
}
