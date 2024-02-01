#include "Constants.h"

const frc::Transform3d VisionConstant::frontCameraTransform{frc::Transform3d(
    frc::Translation3d(0.375_m, 0_m, 0.309_m), frc::Rotation3d(0_deg, 22_deg, 0_deg))};

const frc::Transform3d VisionConstant::backCameraTransform{frc::Transform3d(
    frc::Translation3d(-0.375_m, 0_m, 0.295_m), frc::Rotation3d(-0.2_deg, 22_deg, 180_deg))};

const frc::TrapezoidProfile<units::radians>::Constraints DriveConstant::kThetaControllerConstraints{
    DriveConstant::kMaxAutoAngularSpeed, DriveConstant::kMaxAutoAngularAcceleration};