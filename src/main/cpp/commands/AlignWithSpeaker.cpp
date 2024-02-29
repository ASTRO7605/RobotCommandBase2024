#include "commands/AlignWithSpeaker.h"

AlignWithSpeaker::AlignWithSpeaker(Base *p_Base) : m_pBase{p_Base} { AddRequirements({m_pBase}); }

void AlignWithSpeaker::Initialize() {
    m_Timer.Stop();
    m_Timer.Reset();
}

void AlignWithSpeaker::Execute() {
    turn = -units::radians_per_second_t{m_pBase->GetPIDControlledRotationSpeedToSpeaker()}.value();
    if (turn > 1) {
        turn = 1;
    } else if (turn < -1) {
        turn = -1;
    }

    m_pBase->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0},
                   -units::radians_per_second_t{turn}, true);
    frc::SmartDashboard::PutNumber("rotationError", m_pBase->GetRotationPIDError() * 180);

    if (fabs(m_pBase->GetRotationPIDError() * 180) <= DriveConstant::kThresholdRobotAngle) {
        m_Timer.Start();
    } else {
        m_Timer.Stop();
        m_Timer.Reset();
    }
}

bool AlignWithSpeaker::IsFinished() {
    if (m_Timer.Get() >= DriveConstant::kThresholdTimer) {
        return true;
    }
    return false;
}

void AlignWithSpeaker::End(bool) {}