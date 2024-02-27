#include "subsystems/Limelight.h"

Limelight::Limelight(std::string_view cameraName) : camera{cameraName} {}

void Limelight::Periodic() {
    if (camera.GetLatestResult().HasTargets()) {
        frc::SmartDashboard::PutNumber("target yaw",
                                       camera.GetLatestResult().GetBestTarget().GetYaw());
        frc::SmartDashboard::PutNumber("target pitch",
                                       camera.GetLatestResult().GetBestTarget().GetPitch());
    }
}

std::optional<photon::PhotonTrackedTarget> Limelight::GetLatestTarget() {
    if (camera.GetLatestResult().HasTargets()) {
        return camera.GetLatestResult().GetBestTarget();
    } else {
        return {};
    }
}