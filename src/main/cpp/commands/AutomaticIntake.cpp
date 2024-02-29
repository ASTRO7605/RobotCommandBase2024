#include "commands/AutomaticIntake.h"

AutomaticIntake::AutomaticIntake(Intake *p_Intake, Base *p_Base)
    : m_pIntake{p_Intake}, m_pBase{p_Base} {
    AddRequirements({m_pIntake, m_pBase});
}

void AutomaticIntake::Initialize() {
    m_pBase->SetRobotDrivingMode(false);
    m_pIntake->SetIntake(true, false, false);
    isFinished = false;
    if (!m_pBase->GetLatestLimelightTarget().has_value()) {
        isFinished = true;
    }
}

void AutomaticIntake::Execute() {
    std::optional<photon::PhotonTrackedTarget> latestTarget{m_pBase->GetLatestLimelightTarget()};
    if (latestTarget.has_value()) {
        double yawError{latestTarget.value().GetYaw()};
        double pitchError{-latestTarget.value().GetPitch()};

        units::meters_per_second_t xSpeed{pitchError * DriveConstant::kPXYRobot};
        units::meters_per_second_t ySpeed{yawError * DriveConstant::kPXYRobot};

        if (ySpeed.value() > DriveConstant::kMaxAutoAlignSpeedY) {
            ySpeed = units::meters_per_second_t(DriveConstant::kMaxAutoAlignSpeedY);
        }

        if (xSpeed.value() > DriveConstant::kMaxAutoAlignSpeedX) {
            xSpeed = units::meters_per_second_t(DriveConstant::kMaxAutoAlignSpeedX);
        }

        m_pBase->Drive(xSpeed, ySpeed, 0_rad_per_s, false);
    }
    if (m_pIntake->IsObjectInIntake()) {
        isFinished = true;
    }
}

bool AutomaticIntake::IsFinished() {
    if (isFinished) {
        return true;
    }
    return false;
}

void AutomaticIntake::End(bool interrupted) {
    m_pBase->SetRobotDrivingMode(true);
    m_pIntake->SetIntake(false, false, false);
    m_pBase->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0},
                   units::radians_per_second_t{0}, false);
}
