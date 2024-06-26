#include "subsystems/Base.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

Base::Base()
    : pidControllerThetaSpeaker{DriveConstant::kPThetaRobot, DriveConstant::kIThetaRobot,
                                DriveConstant::kDThetaRobot},
      m_PoseEstimator{kDriveKinematics,
                      m_Gyro.GetRotation2d().Radians(),
                      {m_FrontRightModule.GetPosition(), m_FrontLeftModule.GetPosition(),
                       m_RearLeftModule.GetPosition(), m_RearRightModule.GetPosition()},
                      frc::Pose2d{0_m, 0_m, 0_rad},

                      // State and vision standard deviations for Kalman filter.
                      PoseEstimationConstant::kStateStdDevs,
                      PoseEstimationConstant::kVisionStdDevsDefault}
// important to follow kinematics
// construction order
{
    // Implementation of subsystem constructor goes here.
    m_Gyro.Calibrate();
    m_Gyro.Reset();

    pidControllerThetaSpeaker.EnableContinuousInput(-1, 1);

    pathplanner::AutoBuilder::configureHolonomic(
        [this]() { return GetPose(); },                    // robot pose supplier
        [this](frc::Pose2d pose) { ResetOdometry(pose); }, // to reset odometry

        // ChassisSpeeds supplier/setter * MUST BE ROBOT RELATIVE *
        [this]() { return GetRobotRelativeSpeeds(); },
        [this](frc::ChassisSpeeds speeds) { DriveRobotRelativeChassisSpeeds(speeds); },
        pathplanner::HolonomicPathFollowerConfig(
            pathplanner::PIDConstants(DriveConstant::kPAutoMovementController, // P
                                      0.0,                                     // I
                                      0.0                                      // D
                                      ),                                       // PID translation

            pathplanner::PIDConstants(DriveConstant::kPAutoThetaController, // P
                                      0.0,                                  // I
                                      0.0                                   // D
                                      ),                                    // PID translation

            DriveConstant::kMaxAutoSpeed, DriveConstant::kChassisRadius,
            pathplanner::ReplanningConfig(true, true) // can be tweaked for further control
            ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the
            // red alliance This will flip the path being followed to the red side
            // of the field. THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // reference to this subsystem to add requirements
    );

    frc::SmartDashboard::PutData("VisionMeasurementLeft", &m_VisionFieldLeft);
    frc::SmartDashboard::PutData("VisionMeasurementRight", &m_VisionFieldRight);
    frc::SmartDashboard::PutData("RobotMeasurement", &m_RobotField);

    m_PoseEstimator.ResetPosition(m_Gyro.GetRotation2d().Radians(),
                                  {m_FrontRightModule.GetPosition(),
                                   m_FrontLeftModule.GetPosition(), m_RearLeftModule.GetPosition(),
                                   m_RearRightModule.GetPosition()},
                                  frc::Pose2d{0_m, 0_m, 0_rad});

    m_DrivingInFieldRelative = true;
    isRotationBeingControlled = false;
    m_TimerEncoder.Restart();
}

void Base::Periodic() {
    frc::SmartDashboard::PutBoolean("isRotationBeingControlled", isRotationBeingControlled);
    if (std::isnan(m_PoseEstimator.GetEstimatedPosition().X().value()) ||
        std::isnan(m_PoseEstimator.GetEstimatedPosition().Y().value()) ||
        std::isnan(m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees().value())) {
        ResetOdometry(lastPoseEstimate);
    }
    lastPoseEstimate = m_PoseEstimator.GetEstimatedPosition();

    frc::SmartDashboard::PutNumber("leftAprilTagID", GetLeftCameraAprilTagID());
    frc::SmartDashboard::PutNumber("rightAprilTagID", GetRightCameraAprilTagID());
    if (frc::DriverStation::GetAlliance().has_value()) {
        allianceColor = frc::DriverStation::GetAlliance().value();
    }

    frc::SmartDashboard::PutNumber("distanceToSpeaker", GetDistanceToSpeaker().value());
    frc::SmartDashboard::PutNumber("desiredRotationToSpeaker",
                                   GetDesiredRotationToSpeaker().value());
    frc::SmartDashboard::PutNumber("rotationError", GetRotationPIDError());
    frc::SmartDashboard::PutNumber(
        "Gyro Angle",
        m_Gyro.GetRotation2d().Degrees().value()); // test on smart dashboard

    frc::SmartDashboard::PutNumber("RobotPoseX",
                                   m_PoseEstimator.GetEstimatedPosition().X().value());

    frc::SmartDashboard::PutNumber("RobotPoseY",
                                   m_PoseEstimator.GetEstimatedPosition().Y().value());

    frc::SmartDashboard::PutNumber(
        "RobotOrientation", m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees().value());

    m_PoseEstimator.Update(m_Gyro.GetRotation2d().Radians(),
                           {m_FrontRightModule.GetPosition(), m_FrontLeftModule.GetPosition(),
                            m_RearLeftModule.GetPosition(),
                            m_RearRightModule.GetPosition()}); // important to follow kinematics
                                                               // construction order

    SetRobotPoseVisionEstimateLeft();
    SetRobotPoseVisionEstimateRight();

    // for SmartDashboard
    m_RobotField.SetRobotPose(m_PoseEstimator.GetEstimatedPosition());
    frc::SmartDashboard::PutNumber("fieldXSpeed", GetFieldRelativeSpeeds().vx());
    frc::SmartDashboard::PutNumber("fieldYSpeed", GetFieldRelativeSpeeds().vy());
}

void Base::SetIdleMode(DriveConstant::IdleMode idleMode) {
    m_FrontRightModule.SetIdleMode(idleMode);
    m_FrontLeftModule.SetIdleMode(idleMode);
    m_RearLeftModule.SetIdleMode(idleMode);
    m_RearRightModule.SetIdleMode(idleMode);
}
void Base::ResetEncoders() {
    m_FrontRightModule.ResetEncoders();
    m_FrontLeftModule.ResetEncoders();
    m_RearLeftModule.ResetEncoders();
    m_RearRightModule.ResetEncoders();
}

// void Base::SeedSwerveEncoders() {
//     m_FrontRightModule.SeedSparkMaxEncoder();
//     m_FrontLeftModule.SeedSparkMaxEncoder();
//     m_RearLeftModule.SeedSparkMaxEncoder();
//     m_RearRightModule.SeedSparkMaxEncoder();
// }

void Base::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                 units::radians_per_second_t rotationSpeed, bool rateLimiting) {
    // code utilise vient de MAXSwerve, regarder en ligne
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimiting) {
        // Convert XY to polar for rate limiting
        double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
        double inputTranslationMag = sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

        // Calculate the direction slew rate based on an estimate of the lateral
        // acceleration
        double directionSlewRate;
        if (m_CurrentTranslationMag != 0.0) {
            directionSlewRate = abs(DriveConstant::kDirectionSlewRate / m_CurrentTranslationMag);
        } else {
            directionSlewRate = 500.0; // some high number that means the slew rate
                                       // is effectively instantaneous
        }

        double currentTime = wpi::Now() * 1e-6; // us to s
        double elapsedTime = currentTime - m_prevTime;
        double angleDif =
            SwerveUtils::AngleDifference(inputTranslationDir, m_CurrentTranslationDir);
        if (angleDif < 0.45 * std::numbers::pi) {
            m_CurrentTranslationDir = SwerveUtils::StepTowardsCircular(
                m_CurrentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            m_CurrentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
        } else if (angleDif > 0.85 * std::numbers::pi) {
            if (m_CurrentTranslationMag > 1e-4) { // some small number to avoid floating-point
                                                  // errors with equality checking
                // keep currentTranslationDir unchanged
                m_CurrentTranslationMag = m_magLimiter.Calculate(0.0);
            } else {
                m_CurrentTranslationDir =
                    SwerveUtils::WrapAngle(m_CurrentTranslationDir + std::numbers::pi);
                m_CurrentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
            }
        } else {
            m_CurrentTranslationDir = SwerveUtils::StepTowardsCircular(
                m_CurrentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            m_CurrentTranslationMag = m_magLimiter.Calculate(0.0);
        }
        m_prevTime = currentTime;

        xSpeedCommanded = m_CurrentTranslationMag * cos(m_CurrentTranslationDir);
        ySpeedCommanded = m_CurrentTranslationMag * sin(m_CurrentTranslationDir);
        if (!IsRotationBeingControlled()) {
            m_CurrentRotation = m_rotLimiter.Calculate(rotationSpeed.value());
        } else {
            m_CurrentRotation = rotationSpeed.value();
        }

    } else {
        xSpeedCommanded = xSpeed.value();
        ySpeedCommanded = ySpeed.value();
        m_CurrentRotation = rotationSpeed.value();
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    units::meters_per_second_t xSpeedDelivered = xSpeedCommanded * DriveConstant::kMaxTeleopSpeed;
    units::meters_per_second_t ySpeedDelivered = ySpeedCommanded * DriveConstant::kMaxTeleopSpeed;
    units::radians_per_second_t rotDelivered =
        m_CurrentRotation * DriveConstant::kMaxTeleopAngularSpeed;

    auto states = kDriveKinematics.ToSwerveModuleStates(
        m_DrivingInFieldRelative
            ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                  xSpeedDelivered, ySpeedDelivered, rotDelivered,
                  m_Gyro.GetRotation2d().Radians() - m_GyroOffset)
            : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

    kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstant::kMaxTeleopSpeed);
    auto [FrontRight, FrontLeft, RearLeft, RearRight] = states;

    m_FrontRightModule.SetDesiredState(FrontRight);
    m_FrontLeftModule.SetDesiredState(FrontLeft);
    m_RearLeftModule.SetDesiredState(RearLeft);
    m_RearRightModule.SetDesiredState(RearRight);
}

void Base::DriveRobotRelativeChassisSpeeds(frc::ChassisSpeeds speeds) {
    auto states = kDriveKinematics.ToSwerveModuleStates(speeds);
    auto [FrontRight, FrontLeft, RearLeft, RearRight] = states;

    m_FrontRightModule.SetDesiredState(FrontRight);
    m_FrontLeftModule.SetDesiredState(FrontLeft);
    m_RearLeftModule.SetDesiredState(RearLeft);
    m_RearRightModule.SetDesiredState(RearRight);
}

void Base::SetWheelsFacingForward() {
    m_FrontRightModule.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)});
    m_FrontLeftModule.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)});
    m_RearLeftModule.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)});
    m_RearRightModule.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)});
}

void Base::SetWheelsFacingSideways() {
    units::radian_t SidewaysPosition{std::numbers::pi * 0.5};
    m_FrontRightModule.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d(SidewaysPosition)});
    m_FrontLeftModule.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d(SidewaysPosition)});
    m_RearLeftModule.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d(SidewaysPosition)});
    m_RearRightModule.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d(SidewaysPosition)});
}

void Base::SetWheelsInXFormation() {
    m_FrontRightModule.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d(-45_deg)});
    m_FrontLeftModule.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d(45_deg)});
    m_RearLeftModule.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d(-45_deg)});
    m_RearRightModule.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d(45_deg)});
}

units::angle::degree_t Base::GetHeadingDegrees() { return m_Gyro.GetRotation2d().Degrees(); }

frc::Pose2d Base::GetPose() { return m_PoseEstimator.GetEstimatedPosition(); }

frc::ChassisSpeeds Base::GetRobotRelativeSpeeds() {
    return kDriveKinematics.ToChassisSpeeds(
        m_FrontRightModule.GetState(), m_FrontLeftModule.GetState(), m_RearLeftModule.GetState(),
        m_RearRightModule.GetState());
}

frc::ChassisSpeeds Base::GetFieldRelativeSpeeds() {
    frc::ChassisSpeeds robotRelativeSpeeds{GetRobotRelativeSpeeds()};

    return frc::ChassisSpeeds::FromRobotRelativeSpeeds(
        robotRelativeSpeeds,
        m_Gyro.GetRotation2d().Radians() - m_GyroOffset -
            ((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 180_deg
                                                                                       : 0_deg));
}

void Base::SetRobotPoseVisionEstimateLeft() {
    if (!m_VisionLeft.SeesValidTarget()) {
        // hide robot if no target in view
        m_VisionFieldLeft.SetRobotPose(100_m, 100_m, 0_rad);
        return;
    }

    std::optional<PoseMeasurement> estimate = m_VisionLeft.GetRobotPoseEstimate();
    if (!estimate.has_value()) {
        return;
    }

    frc::Pose2d measurement2d{estimate->pose.ToPose2d()};

    if (estimate->ambiguity == 0) {
        estimate->ambiguity = 0.01;
    }

    auto std_devs = PoseEstimationConstant::kVisionStdDevsPerAmbiguityPerMeter;
    std_devs[0] =
        estimate->ambiguity * std_devs[0] + PoseEstimationConstant::kVisionStdDevsPerMeterBase[0];
    std_devs[1] =
        estimate->ambiguity * std_devs[1] + PoseEstimationConstant::kVisionStdDevsPerMeterBase[1];
    std_devs[2] =
        estimate->ambiguity * std_devs[2] + PoseEstimationConstant::kVisionStdDevsPerMeterBase[2];

    auto dst = estimate->distance.value();
    std_devs[0] *= dst; // scale based on distance
    std_devs[1] *= dst;
    std_devs[2] *= dst;

    frc::SmartDashboard::PutNumber("april_distance_Left", estimate->distance.value());
    frc::SmartDashboard::PutNumber("ambiguity_Left", estimate->ambiguity);

    m_PoseEstimator.AddVisionMeasurement(measurement2d, (estimate->timestamp), std_devs);

    // Update SmartDashboard
    m_VisionFieldLeft.SetRobotPose(measurement2d);
}

void Base::SetRobotPoseVisionEstimateRight() {

    if (!m_VisionRight.SeesValidTarget()) {
        // hide robot if no target in view
        m_VisionFieldRight.SetRobotPose(100_m, 100_m, 0_rad);
        return;
    }

    std::optional<PoseMeasurement> estimate = m_VisionRight.GetRobotPoseEstimate();
    if (!estimate.has_value()) {
        return;
    }

    frc::Pose2d measurement2d{estimate->pose.ToPose2d()};

    if (estimate->ambiguity == 0) {
        estimate->ambiguity = 0.001;
    }

    auto std_devs = PoseEstimationConstant::kVisionStdDevsPerAmbiguityPerMeter;
    std_devs[0] =
        estimate->ambiguity * std_devs[0] + PoseEstimationConstant::kVisionStdDevsPerMeterBase[0];
    std_devs[1] =
        estimate->ambiguity * std_devs[1] + PoseEstimationConstant::kVisionStdDevsPerMeterBase[1];
    std_devs[2] =
        estimate->ambiguity * std_devs[2] + PoseEstimationConstant::kVisionStdDevsPerMeterBase[2];

    auto dst = estimate->distance.value();
    std_devs[0] *= dst; // scale based on distance
    std_devs[1] *= dst;
    std_devs[2] *= dst;

    frc::SmartDashboard::PutNumber("april_distance_Right", estimate->distance.value());
    frc::SmartDashboard::PutNumber("ambiguity_Right", estimate->ambiguity);

    m_PoseEstimator.AddVisionMeasurement(measurement2d, (estimate->timestamp), std_devs);

    // Update SmartDashboard
    m_VisionFieldRight.SetRobotPose(measurement2d);
}

void Base::ResetOdometry(frc::Pose2d desiredPose) {
    m_PoseEstimator.ResetPosition(GetHeadingDegrees(),
                                  {m_FrontRightModule.GetPosition(),
                                   m_FrontLeftModule.GetPosition(), m_RearLeftModule.GetPosition(),
                                   m_RearRightModule.GetPosition()},
                                  desiredPose);
}

void Base::SwitchRobotDrivingMode() { m_DrivingInFieldRelative = !m_DrivingInFieldRelative; }

void Base::ResetGyroTeleopOffset() { m_GyroOffset = m_Gyro.GetRotation2d().Radians(); }

void Base::ResetGyroTeleopOffsetPoseEstimator() {
    units::degree_t current_heading = m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees();
    units::degree_t current_gyro = m_Gyro.GetRotation2d().Degrees();

    units::degree_t target_heading =
        (allianceColor == frc::DriverStation::Alliance::kBlue) ? 0_deg : 180_deg;

    units::degree_t error = target_heading - current_heading;

    m_GyroOffset = current_gyro + error;
}

bool Base::IsRotationBeingControlled() { return isRotationBeingControlled; }

void Base::SetRotationBeingControlledFlag(bool yesOrNo) { isRotationBeingControlled = yesOrNo; }

units::meter_t Base::GetDistanceToSpeaker() {
    if (allianceColor == frc::DriverStation::Alliance::kBlue) {
        currentColorSpeakerPose = PoseEstimationConstant::blueSpeakerPoseMeters;
    } else if (allianceColor == frc::DriverStation::Alliance::kRed) {
        currentColorSpeakerPose = PoseEstimationConstant::redSpeakerPoseMeters;
    }
    frc::Translation2d currentPose{m_PoseEstimator.GetEstimatedPosition().Translation()};
    currentPose = currentPose + GetProjectedPositionOffset(true);
    auto poseToSpeaker{currentPose - currentColorSpeakerPose};
    return units::meter_t{std::sqrt(poseToSpeaker.X().value() * poseToSpeaker.X().value() +
                                    poseToSpeaker.Y().value() * poseToSpeaker.Y().value())};
}

units::degree_t Base::GetDesiredRotationToSpeaker() {
    if (allianceColor == frc::DriverStation::Alliance::kBlue) {
        currentColorSpeakerPose = PoseEstimationConstant::blueSpeakerPoseMeters;
    } else if (allianceColor == frc::DriverStation::Alliance::kRed) {
        currentColorSpeakerPose = PoseEstimationConstant::redSpeakerPoseMeters;
    }
    frc::Translation2d currentPose{m_PoseEstimator.GetEstimatedPosition().Translation()};
    currentPose = currentPose + GetProjectedPositionOffset(false);
    auto poseToSpeaker{currentPose - currentColorSpeakerPose};
    frc::SmartDashboard::PutNumber("poseToSpeakerX", poseToSpeaker.X().value());
    frc::SmartDashboard::PutNumber("poseToSpeakerY", poseToSpeaker.Y().value());
    double desiredRotation = std::atan2(poseToSpeaker.Y().value(), poseToSpeaker.X().value()) /
                             DriveConstant::DegreesToRad;
    if (desiredRotation < 0) {
        desiredRotation = 180 + desiredRotation;
    } else {
        desiredRotation = -180 + desiredRotation;
    }
    return units::degree_t{desiredRotation};
}

units::degrees_per_second_t Base::GetPIDControlledRotationSpeed(bool alignToSpeaker) {
    if (alignToSpeaker) {
        return units::degrees_per_second_t{pidControllerThetaSpeaker.Calculate(
            m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees().value() / 180,
            GetDesiredRotationToSpeaker().value() / 180)};
    } else {
        double sourceAngleTarget{};
        if (allianceColor == frc::DriverStation::Alliance::kBlue) {
            sourceAngleTarget = DriveConstant::kBlueSourceApproachAngle.value();
        } else if (allianceColor == frc::DriverStation::Alliance::kRed) {
            sourceAngleTarget = DriveConstant::kRedSourceApproachAngle.value();
        }
        return units::degrees_per_second_t{pidControllerThetaSpeaker.Calculate(
            m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees().value() / 180,
            sourceAngleTarget / 180)};
    }
}

bool Base::IsRobotInRangeToShoot() {
    if (GetDistanceToSpeaker() <= DriveConstant::kThresholdSpeakerInRangeToShoot) {
        return true;
    }
    return false;
}

bool Base::IsRobotInRangeToStartWheels() {
    if (GetDistanceToSpeaker() <= DriveConstant::kThresholdSpeakerInRangeToStartWheels) {
        return true;
    }
    return false;
}

double Base::GetRotationPIDError() { return pidControllerThetaSpeaker.GetPositionError(); }

int Base::GetLeftCameraAprilTagID() { return m_VisionLeft.GetAprilTagIDInView(); }

int Base::GetRightCameraAprilTagID() { return m_VisionRight.GetAprilTagIDInView(); }

std::optional<frc::Pose2d> Base::GetAveragePoseFromCameras() {
    std::optional<PoseMeasurement> leftEstimate = m_VisionLeft.GetRobotPoseEstimate();
    std::optional<PoseMeasurement> rightEstimate = m_VisionRight.GetRobotPoseEstimate();
    if (leftEstimate.has_value() && rightEstimate.has_value()) {
        auto left_pose = leftEstimate->pose.ToPose2d();
        auto right_pose = rightEstimate->pose.ToPose2d();
        auto translation = (left_pose.Translation() + right_pose.Translation()) / 2;
        auto rotation = (left_pose.Rotation() + right_pose.Rotation()) / 2;

        return frc::Pose2d{translation, rotation};
    } else if (leftEstimate.has_value()) {
        return leftEstimate->pose.ToPose2d();
    } else if (rightEstimate.has_value()) {
        return rightEstimate->pose.ToPose2d();
    } else {
        return {};
    }
}

void Base::SetRobotDrivingMode(bool fieldRelative) { m_DrivingInFieldRelative = fieldRelative; }

bool Base::IsRobotAlignedToShoot() {
    return fabs((m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees() -
                 GetDesiredRotationToSpeaker())
                    .value()) <= DriveConstant::kThresholdRobotAngle;
}

frc::Translation2d Base::GetProjectedPositionOffset(bool usedForDistance) {
    frc::ChassisSpeeds currentRobotSpeeds{GetFieldRelativeSpeeds()};
    double timeForProjectionInFuture{};
    if (usedForDistance) {
        timeForProjectionInFuture = DriveConstant::kTimeForProjectionInFutureDistance;
    } else {
        timeForProjectionInFuture = DriveConstant::kTimeForProjectionInFutureRotation;
    }
    return frc::Translation2d{units::meter_t{currentRobotSpeeds.vx() * timeForProjectionInFuture},
                              units::meter_t{currentRobotSpeeds.vy() * timeForProjectionInFuture}};
}

void Base::ResetGyroOffsetFromValue(units::radian_t offset) { m_GyroOffset = offset; }