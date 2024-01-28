// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include "subsystems/ModuleSwerve.h"
#include "subsystems/Vision.h"

#include "utils/SwerveUtils.h"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>

#include <AHRS.h>
#include <rev/CANSparkMax.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/SPI.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include <vector>

class Base : public frc2::SubsystemBase {
  public:
    Base();

    void Periodic() override;

    void ResetEncoders();

    void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
               units::radians_per_second_t rotationSpeed, bool rateLimiting);

    void DriveRobotRelativeChassisSpeeds(frc::ChassisSpeeds speeds);

    void SetWheelsFacingForward();

    void SetWheelsFacingSideways();

    frc::Pose2d GetPose();

    frc::ChassisSpeeds GetRobotRelativeSpeeds();

    void ResetOdometry(frc::Pose2d desiredPose);

    units::angle::degree_t GetHeadingDegrees();

    /**
     * switch le driving mode de field relative à robot relative et vice versa
     */
    void SwitchRobotDrivingMode();

    /**
     * Realigne l'avant du field oriented sur l'orientation du robot
     */
    void ResetGyroTeleopOffset();

    /**
     * important to follow order: FrontRight, FrontLeft, RearLeft, RearRight
     */
    frc::SwerveDriveKinematics<4> kDriveKinematics{
        frc::Translation2d{DriveConstant::kDistanceFrontAndRearWheels / 2,
                           -DriveConstant::kDistanceRightAndLeftWheels / 2}, // front right
        frc::Translation2d{DriveConstant::kDistanceFrontAndRearWheels / 2,
                           DriveConstant::kDistanceRightAndLeftWheels / 2}, // front left
        frc::Translation2d{-DriveConstant::kDistanceFrontAndRearWheels / 2,
                           DriveConstant::kDistanceRightAndLeftWheels / 2}, // rear left
        frc::Translation2d{-DriveConstant::kDistanceFrontAndRearWheels / 2,
                           -DriveConstant::kDistanceRightAndLeftWheels / 2}, // rear right
    };

    /// @brief Add a pose estimate from front vision to pose estimator.
    void SetRobotPoseVisionEstimateFront(PoseMeasurement estimate);

    /// @brief Add a pose estimate from front vision to pose estimator.
    void SetRobotPoseVisionEstimateBack(PoseMeasurement estimate);

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.

    AHRS m_Gyro{frc::SPI::Port::kMXP};
    ModuleSwerve m_FrontRightModule{DriveConstant::FrontRightTurningID,
                                    DriveConstant::FrontRightDrivingID,
                                    DriveConstant::FrontRightCANcoderID};
    ModuleSwerve m_FrontLeftModule{DriveConstant::FrontLeftTurningID,
                                   DriveConstant::FrontLeftDrivingID,
                                   DriveConstant::FrontLeftCANcoderID};
    ModuleSwerve m_RearLeftModule{DriveConstant::RearLeftTurningID,
                                  DriveConstant::RearLeftDrivingID,
                                  DriveConstant::RearLeftCANcoderID};
    ModuleSwerve m_RearRightModule{DriveConstant::RearRightTurningID,
                                   DriveConstant::RearRightDrivingID,
                                   DriveConstant::RearRightCANcoderID};

    // std::shared_ptr<frc::SwerveDriveOdometry<4>> m_pOdometry;
    frc::SwerveDrivePoseEstimator<4> m_PoseEstimator;

    double m_CurrentRotation = 0.0;
    double m_CurrentTranslationDir = 0.0;
    double m_CurrentTranslationMag = 0.0;
    bool m_DrivingInFieldRelative;

    units::radian_t m_GyroOffset = 0_rad;

    frc::SlewRateLimiter<units::scalar> m_magLimiter{DriveConstant::kMagnitudeSlewRate / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{DriveConstant::kRotationalSlewRate / 1_s};
    double m_prevTime = wpi::Now() * 1e-6;

    // Used for SmartDashboard display of pose estimation.
    frc::Field2d m_VisionFieldFront;
    frc::Field2d m_VisionFieldBack;
    frc::Field2d m_RobotField;
};