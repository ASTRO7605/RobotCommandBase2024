#include "Constants.h"

const frc::Transform3d VisionConstant::rightCameraTransform{frc::Transform3d(
    frc::Translation3d(-0.292_m, -0.176_m, 0.683_m), frc::Rotation3d(0_deg, -32_deg, -24.6_deg))};

const frc::Transform3d VisionConstant::leftCameraTransform{frc::Transform3d(
    frc::Translation3d(-0.292_m, 0.189_m, 0.683_m), frc::Rotation3d(0_deg, -32_deg, 24.6_deg))};

const frc::TrapezoidProfile<units::radians>::Constraints DriveConstant::kThetaControllerConstraints{
    DriveConstant::kMaxAutoAngularSpeed, DriveConstant::kMaxAutoAngularAcceleration};