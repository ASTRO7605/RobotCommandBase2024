#include "Constants.h"

const frc::Transform3d VisionConstant::rightCameraTransform{frc::Transform3d(
    frc::Translation3d(-0.292_m, -0.176_m, 0.683_m), frc::Rotation3d(0_deg, -32_deg, -24.6_deg))};

const frc::Transform3d VisionConstant::leftCameraTransform{frc::Transform3d(
    frc::Translation3d(-0.292_m, 0.189_m, 0.683_m), frc::Rotation3d(0_deg, -32_deg, 24.6_deg))};

const frc::TrapezoidProfile<units::radians>::Constraints DriveConstant::kThetaControllerConstraints{
    DriveConstant::kMaxAutoAngularSpeed, DriveConstant::kMaxAutoAngularAcceleration};

const std::vector<std::pair<units::meter_t, double>>
    ShooterConstant::wheelSpeedsAccordingToDistance{
        ShooterConstant::firstDistanceWheelSpeedsCouple,
        ShooterConstant::secondDistanceWheelSpeedsCouple,
        ShooterConstant::thirdDistanceWheelSpeedsCouple,
        ShooterConstant::fourthDistanceWheelSpeedsCouple,
        ShooterConstant::fifthDistanceWheelSpeedsCouple,
        ShooterConstant::sixthDistanceWheelSpeedsCouple,
        ShooterConstant::seventhDistanceWheelSpeedsCouple,
        ShooterConstant::eighthDistanceWheelSpeedsCouple,
    };

const std::vector<std::pair<units::meter_t, double>>
    ShooterConstant::shooterAngleAccordingToDistance{
        ShooterConstant::firstDistanceShooterAngleCouple,
        ShooterConstant::secondDistanceShooterAngleCouple,
        ShooterConstant::thirdDistanceShooterAngleCouple,
        ShooterConstant::fourthDistanceShooterAngleCouple,
        ShooterConstant::fifthDistanceShooterAngleCouple,
        ShooterConstant::sixthDistanceShooterAngleCouple,
        ShooterConstant::seventhDistanceShooterAngleCouple,
        ShooterConstant::eighthDistanceShooterAngleCouple,
    };