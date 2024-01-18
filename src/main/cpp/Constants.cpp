#include "Constants.h"

const frc::TrapezoidProfile<units::radians>::Constraints DriveConstant::kThetaControllerConstraints{DriveConstant::kMaxAutoAngularSpeed, DriveConstant::kMaxAutoAngularAcceleration};