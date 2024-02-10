#include "subsystems/Base.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

Base::Base()
    : m_PoseEstimator{kDriveKinematics,
                      m_Gyro.GetRotation2d().Radians(),
                      {m_FrontRightModule.GetPosition(), m_FrontLeftModule.GetPosition(),
                       m_RearLeftModule.GetPosition(), m_RearRightModule.GetPosition()},
                      frc::Pose2d{0_m, 0_m, 0_rad},

                      // State and vision standard deviations for Kalman filter.
                      PoseEstimationConstant::kStateStdDevs,
                      PoseEstimationConstant::kVisionStdDevs} // important to follow kinematics
                                                              // construction order
{
    // Implementation of subsystem constructor goes here.
    m_Gyro.Calibrate();
    m_Gyro.Reset();
    /*
    m_pOdometry.reset(new frc::SwerveDriveOdometry<4>{kDriveKinematics,
              m_Gyro.GetRotation2d().Radians(),
              {m_FrontRightModule.GetPosition(), m_FrontLeftModule.GetPosition(),
              m_RearLeftModule.GetPosition(), m_RearRightModule.GetPosition()},
    frc::Pose2d{}}); //important to follow kinematics construction order
    */

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

    //frc::SmartDashboard::PutData("VisionMeasurementFront", &m_VisionFieldFront);
    //frc::SmartDashboard::PutData("VisionMeasurementBack", &m_VisionFieldBack);
    frc::SmartDashboard::PutData("RobotMeasurement", &m_RobotField);

    m_PoseEstimator.ResetPosition(m_Gyro.GetRotation2d().Radians(),
                                  {m_FrontRightModule.GetPosition(),
                                   m_FrontLeftModule.GetPosition(), m_RearLeftModule.GetPosition(),
                                   m_RearRightModule.GetPosition()},
                                  frc::Pose2d{0_m, 0_m, 0_rad});

    m_DrivingInFieldRelative = true;
}

void Base::Periodic() {
    frc::SmartDashboard::PutNumber(
        "Gyro Angle",
        m_Gyro.GetRotation2d().Degrees().value()); // test on smart dashboard

    frc::SmartDashboard::PutNumber("RobotPoseX",
                                   m_PoseEstimator.GetEstimatedPosition().X().value());

    frc::SmartDashboard::PutNumber("RobotPoseY",
                                   m_PoseEstimator.GetEstimatedPosition().Y().value());

    frc::SmartDashboard::PutNumber(
        "RobotOrientation", m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees().value());

    // m_pOdometry->Update(m_Gyro.GetRotation2d().Radians(),
    m_PoseEstimator.Update(m_Gyro.GetRotation2d().Radians(),
                           {m_FrontRightModule.GetPosition(), m_FrontLeftModule.GetPosition(),
                            m_RearLeftModule.GetPosition(),
                            m_RearRightModule.GetPosition()}); // important to follow kinematics
                                                               // construction order

    SetRobotPoseVisionEstimateFront();
    SetRobotPoseVisionEstimateBack();

    // for SmartDashboard
    m_RobotField.SetRobotPose(m_PoseEstimator.GetEstimatedPosition());
}

void Base::ResetEncoders() {
    m_FrontRightModule.ResetEncoders();
    m_FrontLeftModule.ResetEncoders();
    m_RearLeftModule.ResetEncoders();
    m_RearRightModule.ResetEncoders();
}

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
        m_CurrentRotation = m_rotLimiter.Calculate(rotationSpeed.value());

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

frc::Pose2d Base::GetPose() {
    // return m_pOdometry->GetPose();
    return m_PoseEstimator.GetEstimatedPosition();
}

frc::ChassisSpeeds Base::GetRobotRelativeSpeeds() {
    return kDriveKinematics.ToChassisSpeeds(
        m_FrontRightModule.GetState(), m_FrontLeftModule.GetState(), m_RearLeftModule.GetState(),
        m_RearRightModule.GetState());
}

void Base::SetRobotPoseVisionEstimateFront() {
    // if (!m_VisionFront.SeesValidTarget()) {
    //     // hide robot if no target in view
    //     m_VisionFieldFront.SetRobotPose(100_m, 100_m, 0_rad);
    //     return;
    // }

    // std::optional<PoseMeasurement> estimate = m_VisionFront.GetRobotPoseEstimate();
    // if (!estimate.has_value()) {
    //     return;
    // }

    // frc::Pose2d measurement2d{estimate->pose.ToPose2d()};

    // auto std_devs = PoseEstimationConstant::kVisionStdDevs_XYPerMeterSquared_Front;
    // auto dst_sq = estimate->distance.value() * estimate->distance.value();
    // std_devs[0] *= dst_sq; // scale based on distance
    // std_devs[1] *= dst_sq;

    // frc::SmartDashboard::PutNumber("april_distance_front", estimate->distance.value());

    // m_PoseEstimator.AddVisionMeasurement(
    //     measurement2d,
    //     // estimated original time of data capture (now - latency in ms)
    //     (estimate->timestamp), std_devs);

    // // Update SmartDashboard
    // m_VisionFieldFront.SetRobotPose(measurement2d);
}

void Base::SetRobotPoseVisionEstimateBack() {
    // if (!m_VisionBack.SeesValidTarget()) {
    //     // hide robot if no target in view
    //     m_VisionFieldBack.SetRobotPose(100_m, 100_m, 0_rad);
    //     return;
    // }

    // std::optional<PoseMeasurement> estimate = m_VisionBack.GetRobotPoseEstimate();
    // if (!estimate.has_value()) {
    //     return;
    // }

    // frc::Pose2d measurement2d{estimate->pose.ToPose2d()};

    // auto std_devs = PoseEstimationConstant::kVisionStdDevs_XYPerMeterSquared_Back;
    // auto dst_sq = estimate->distance.value() * estimate->distance.value();
    // std_devs[0] *= dst_sq; // scale based on distance
    // std_devs[1] *= dst_sq;

    // frc::SmartDashboard::PutNumber("april_distance_back", estimate->distance.value());

    // m_PoseEstimator.AddVisionMeasurement(
    //     measurement2d,
    //     // estimated original time of data capture (now - latency in ms)
    //     (estimate->timestamp), std_devs);

    // // Update SmartDashboard
    // m_VisionFieldBack.SetRobotPose(measurement2d);
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