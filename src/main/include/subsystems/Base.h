// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include "subsystems/ModuleSwerve.h"
#include "subsystems/Vision.h"
#include "utils/PoseMeasurement.h"

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
#include <frc/SPI.h>
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include <vector>

class Base : public frc2::SubsystemBase {
  public:
    Base();

    void Periodic() override;

    void SeedSwerveEncoders();

    void SetIdleMode(DriveConstant::IdleMode);

    void ResetEncoders();

    void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
               units::radians_per_second_t rotationSpeed, bool rateLimiting);

    void DriveRobotRelativeChassisSpeeds(frc::ChassisSpeeds speeds);

    void SetWheelsFacingForward();

    void SetWheelsFacingSideways();

    void SetWheelsInXFormation();

    frc::Pose2d GetPose();

    frc::ChassisSpeeds GetRobotRelativeSpeeds();

    frc::ChassisSpeeds GetFieldRelativeSpeeds();

    frc::Translation2d GetProjectedPositionOffset(bool usedForDistance);

    void ResetOdometry(frc::Pose2d desiredPose);

    units::angle::degree_t GetHeadingDegrees();

    void SetRobotDrivingMode(bool fieldRelative);

    /**
     * switch le driving mode de field relative à robot relative et vice versa
     */
    void SwitchRobotDrivingMode();

    /**
     * Realigne l'avant du field oriented sur l'orientation du robot
     */
    void ResetGyroTeleopOffset();

    /**
     * Realigne l'avant du field oriented sur l'orientation du robot avec PoseEstimator
     */
    void ResetGyroTeleopOffsetPoseEstimator();

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
    void SetRobotPoseVisionEstimateLeft();

    /// @brief Add a pose estimate from front vision to pose estimator.
    void SetRobotPoseVisionEstimateRight();

    bool IsRotationBeingControlled();

    void SetRotationBeingControlledFlag(bool yesOrNo);

    units::meter_t GetDistanceToSpeaker();

    units::degree_t GetDesiredRotationToSpeaker();

    units::degrees_per_second_t GetPIDControlledRotationSpeed(bool alignToSpeaker);

    bool IsRobotInRangeToShoot();
    bool IsRobotAlignedToShoot();

    bool IsRobotInRangeToStartWheels();

    double GetRotationPIDError();

    int GetLeftCameraAprilTagID();

    int GetRightCameraAprilTagID();

    std::optional<frc::Pose2d> GetAveragePoseFromCameras();

    void ResetGyroOffsetFromValue(units::radian_t offset);

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    frc::PIDController pidControllerThetaSpeaker;
    frc::Timer m_TimerEncoder;
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

    frc::SwerveDrivePoseEstimator<4> m_PoseEstimator;

    double m_CurrentRotation = 0.0;
    double m_CurrentTranslationDir = 0.0;
    double m_CurrentTranslationMag = 0.0;
    bool m_DrivingInFieldRelative;
    bool isRotationBeingControlled;
    frc::Translation2d currentColorSpeakerPose;

    // garbage default value
    frc::DriverStation::Alliance allianceColor = frc::DriverStation::Alliance::kRed;

    units::radian_t m_GyroOffset = 0_rad;

    frc::SlewRateLimiter<units::scalar> m_magLimiter{DriveConstant::kMagnitudeSlewRate / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{DriveConstant::kRotationalSlewRate / 1_s};
    double m_prevTime = wpi::Now() * 1e-6;

    // Used for SmartDashboard display of pose estimation.
    frc::Field2d m_VisionFieldLeft;
    frc::Field2d m_VisionFieldRight;
    frc::Field2d m_RobotField;

    Vision m_VisionLeft{VisionConstant::TableNameLeft, VisionConstant::leftCameraTransform};
    Vision m_VisionRight{VisionConstant::TableNameRight, VisionConstant::rightCameraTransform};

    frc::Pose2d lastPoseEstimate{};
};