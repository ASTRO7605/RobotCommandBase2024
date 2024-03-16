#pragma once
#include <array>
#include <cstdint>
#include <frc/geometry/Transform3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <iostream>
#include <numbers>
#include <string>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

enum ScoringPositions { speaker, amp, trap };
namespace DriveConstant {
constexpr auto delayBeforeSeedEncoders = 2.5_s;
enum IdleMode { Coast, Brake };
constexpr double kDrivingGearRatio = 6.75;
constexpr double kTurningGearRatio = 150.0 / 7.0;
constexpr double kWheelDiameterM = 0.09689;
constexpr auto kWheelCirconfM = (kWheelDiameterM * std::numbers::pi);

constexpr units::meter_t kDistanceRightAndLeftWheels = 0.57785_m;
constexpr units::meter_t kDistanceFrontAndRearWheels = 0.66675_m;
// Distance between centre of chassis and any of the swerve pods.
// sqrt((front to rear/2)^2 + (left to right/2)^2) thanks to Pythagoras
constexpr units::meter_t kChassisRadius = 0.44_m;

constexpr bool kGyroReversed = false;
constexpr double currentLimit = 50; // Amperes

// constexpr auto ks = 0.17638_V;
// constexpr auto kv = 2.86 * 1_V * 1_s / 1_m;
// constexpr auto ka = 0.321 * 1_V * 1_s * 1_s / 1_m;
constexpr auto kMaxTeleopSpeed = 4.25_mps;
constexpr units::radians_per_second_t kMaxTeleopAngularSpeed{std::numbers::pi * 1.5};

constexpr auto kMaxAutoSpeed = 4.25_mps;
constexpr auto kMaxAutoAcceleration = 4.25_mps_sq;
constexpr units::radians_per_second_t kMaxAutoAngularSpeed{std::numbers::pi * 1.5};
constexpr units::radians_per_second_squared_t kMaxAutoAngularAcceleration{std::numbers::pi * 1.5};

constexpr double kDirectionSlewRate = 1.5;  // radians per second *valeurs à tester et changer
constexpr double kMagnitudeSlewRate = 8.0;  // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 8.0; // percent per second (1 = 100%)
constexpr double kControllerMovementDeadband = 0.15; // valeur minimum qu'on recoit des controllers
constexpr double kControllerRotationDeadband = 0.15;
// constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
// constexpr auto kRamseteZeta = 0.7 / 1_rad;
constexpr double kPDriving = 0.1;
constexpr double kIDriving = 0.0005;
constexpr double kDDriving = 0;
constexpr double kFFDriving = 0.15;
constexpr double kDrivingMinInput = -1.0;
constexpr double kDrivingMaxInput = 1.0;

constexpr double kPTurning = 1.65;
constexpr double kITurning = 0;
constexpr double kDTurning = 0;
constexpr double kFFTurning = 0;
constexpr double kTurningMinInput = -1.0;
constexpr double kTurningMaxInput = 1.0;

constexpr double kPAutoMovementController = 5.5;
constexpr double kIAutoMovementController = 0;
constexpr double kDAutoMovementController = 0;
constexpr double kPAutoThetaController = 3.25;

extern const frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints;
constexpr double kVoltageCompensation = 10;             // volts
constexpr double DegreesToRad = std::numbers::pi / 180; // degrees * conversion factor => rad
constexpr int PowerDistributionHubID = 1;
constexpr int FrontRightTurningID = 48;
constexpr int FrontRightDrivingID = 3;
constexpr int FrontRightCANcoderID = 4;
constexpr int FrontLeftTurningID = 5;
constexpr int FrontLeftDrivingID = 6;
constexpr int FrontLeftCANcoderID = 7;
constexpr int RearLeftTurningID = 8;
constexpr int RearLeftDrivingID = 9;
constexpr int RearLeftCANcoderID = 10;
constexpr int RearRightTurningID = 11;
constexpr int RearRightDrivingID = 12;
constexpr int RearRightCANcoderID = 13;

constexpr double kPThetaRobot = 525;
constexpr double kIThetaRobot = 0.01;
constexpr double kDThetaRobot = 20;

constexpr double kPXYRobot = 0.0115;
constexpr double kIXYRobot = 0;
constexpr double kDXYRobot = 0;

constexpr double kMaxAutoAlignSpeedY = 0.2;
constexpr double kMaxAutoAlignSpeedX = 0.1;

constexpr double kThresholdRobotAngle = 1.5;
constexpr auto kThresholdTimer = 0.1_s;

constexpr auto kThresholdSpeakerInRangeToShoot = 3.9_m;
constexpr auto kThresholdSpeakerInRangeToStartWheels = 6.0_m;
constexpr auto kTimeBeforeBrake = 1_s;

constexpr double kTimeForProjectionInFutureDistance = 0.35;
constexpr double kTimeForProjectionInFutureRotation = 0.55;
} // namespace DriveConstant

namespace VisionConstant {
constexpr int Pipeline = 0; // only one pipeline (AprilTags)
constexpr std::string_view TableNameRight = "photonvision-a";
constexpr std::string_view TableNameLeft = "photonvision-b";
constexpr std::string_view TableNameLimelight = "limelight";
constexpr double ambiguityThreshold = 0.4;
extern const frc::Transform3d rightCameraTransform;
extern const frc::Transform3d leftCameraTransform;

enum StageAprilTagIDs {
    redSourceSide = 11,
    redSpeakerSide = 12,
    redMiddleSide = 13,
    blueMiddleSide = 14,
    blueSpeakerSide = 15,
    blueSourceSide = 16
};

enum class LedMode : int { Off = 1, Flash = 2, On = 3 };
} // namespace VisionConstant
namespace PoseEstimationConstant {
// x(m), y(m), theta(rad)
constexpr std::array<double, 3> kStateStdDevs{0.12, 0.12, 0.008};
constexpr std::array<double, 3> kVisionStdDevsDefault{0.8, 0.8, 0.99};
constexpr std::array<double, 3> kVisionStdDevsPerMeterBase{0.4, 0.4, 0.95};
constexpr std::array<double, 3> kVisionStdDevsPerAmbiguityPerMeter{
    10.0, 10.0, 500.0}; // ambiguity is very small, so this number is quite big.
constexpr frc::Translation2d blueSpeakerPoseMeters =
    frc::Translation2d{units::inch_t{-1.5}, units::inch_t{218.42}};
constexpr frc::Translation2d redSpeakerPoseMeters =
    frc::Translation2d{units::inch_t{652.73}, units::inch_t{218.42}};
} // namespace PoseEstimationConstant

namespace OIConstant {
constexpr int TurnStickID = 0;
constexpr int ThrottleStickID = 1;
constexpr int CoPilotControllerID = 2;
constexpr int CoPilot_A_Button = 1;
constexpr int CoPilot_B_Button = 2;
constexpr int CoPilot_X_Button = 3;
constexpr int CoPilot_Y_Button = 4;
constexpr int CoPilot_LB_Button = 5;
constexpr int CoPilot_RB_Button = 6;
constexpr int CoPilot_Back_Button = 7;
constexpr int CoPilot_Start_Button = 8;
constexpr int CoPilot_LPush_Button = 9;
constexpr int CoPilot_RPush_Button = 10;
constexpr double axisThreshold = 0.5;
} // namespace OIConstant

namespace ShooterConstant {
enum ShooterState {
    init,
    waitingForSubsystems,
    // waitingForDeuxiemeJointTrap,
    moveNoteInShooter,
    waitingForNoteToEnter,
    waitingForNoteToExit,
    waitingForEnd,
    complete,
    noNote,
};
constexpr int leftMotorID = 16;
constexpr int rightMotorID = 17;
constexpr int angleMotorID = 18;
constexpr int capteurID = 1;
constexpr double absoluteEncoderOffset = -2078.12;   // 1/10 degre
constexpr double flywheelsSpeedManualSpeaker = 3000; // RPM
constexpr double manualSpeakerAngle = 650;
constexpr double flywheelsSpeedAmp = 550;   // RPM
constexpr double flywheelsSpeedTrap = 1650; // RPM
constexpr double speedThreshold = 100;      // RPM
constexpr auto timeThreshold = 0.2_s;
constexpr double kPLeftFlywheel = 0.00025;
constexpr double kILeftFlywheel = 0.000001;
constexpr double kDLeftFlywheel = 0.01;
constexpr double kFFLeftFlywheel = 0;
constexpr double kPRightFlywheel = 0.00025;
constexpr double kIRightFlywheel = 0.000001;
constexpr double kDRightFlywheel = 0.01;
constexpr double kFFRightFlywheel = 0;
constexpr double FConversionFactorWheels = 1.0 / 42.0;             // ticks * F -> wheel rotations
constexpr double FConversionFactorPositionAngle = 3600.0 / 4096.0; // ticks * F -> 0.1 degres
constexpr double FConversionFactorVelocityAngle = // ticks/100ms * F -> 0.1 degres/s
    36000.0 / 4096.0;
constexpr double FConversionFactorAccelerationAngle = // ticks/100ms * F -> 0.1 degres/s^2
    36000.0 / 4096.0;
constexpr int currentLimitFlywheels = 50;   // amperes
constexpr double kVoltageCompensation = 10; // Volts
constexpr int kTimeoutMs = 30;
constexpr double kNominalOutputForward = 0;
constexpr double kNominalOutputReverse = 0;
constexpr double kPeakOutputForward = 1;
constexpr double kPeakOutputReverse = -1;
constexpr double kPPositionAngle = 7.25;
constexpr double kIPositionAngle = 0.015;
constexpr double kDPositionAngle = 25.0;
constexpr double kFPositionAngle = 0;
constexpr double kVitesseAngle = 1000;      // 1/10 degre par seconde
constexpr double kAccelerationAngle = 4000; // 1/10 degre par seconde^2
constexpr double kPercentOutputAngle = 0.1; // dixieme de degre par seconde pour mode manuel
constexpr double angleThreshold = 7.5;      // dixieme de degre
constexpr double kMaxAF = 0 /*.0445*/;
constexpr double FDegToRad = std::numbers::pi / 180;
constexpr double kPeakCurrentLimit = 20;   // amperes
constexpr double kPeakCurrentDuration = 0; // ms
constexpr double kContinuousCurrent = 20;  // amperes
constexpr double kForwardSoftLimit = 785;  // 1/10 degre
constexpr double kReverseSoftLimit = 200;  // 1/10 degre
constexpr double kAngleShooterAmp = 600;
constexpr double kAngleShooterTrap = 770;
constexpr double kIntermediateAngleShooter = 450;
constexpr double kRPMDifferenceSpin = 500;
constexpr std::pair<units::meter_t, double> firstDistanceWheelSpeedsCouple =
    std::make_pair(1.35_m, 3000);
constexpr std::pair<units::meter_t, double> firstDistanceShooterAngleCouple =
    std::make_pair(1.35_m, 650);
constexpr std::pair<units::meter_t, double> secondDistanceWheelSpeedsCouple =
    std::make_pair(1.60_m, 3000);
constexpr std::pair<units::meter_t, double> secondDistanceShooterAngleCouple =
    std::make_pair(1.60_m, 600);
constexpr std::pair<units::meter_t, double> thirdDistanceWheelSpeedsCouple =
    std::make_pair(1.90_m, 3000);
constexpr std::pair<units::meter_t, double> thirdDistanceShooterAngleCouple =
    std::make_pair(1.90_m, 545);
constexpr std::pair<units::meter_t, double> fourthDistanceWheelSpeedsCouple =
    std::make_pair(2.20_m, 3000);
constexpr std::pair<units::meter_t, double> fourthDistanceShooterAngleCouple =
    std::make_pair(2.20_m, 485);
constexpr std::pair<units::meter_t, double> fifthDistanceWheelSpeedsCouple =
    std::make_pair(2.52_m, 3000);
constexpr std::pair<units::meter_t, double> fifthDistanceShooterAngleCouple =
    std::make_pair(2.52_m, 440);
constexpr std::pair<units::meter_t, double> sixthDistanceWheelSpeedsCouple =
    std::make_pair(2.81_m, 4000);
constexpr std::pair<units::meter_t, double> sixthDistanceShooterAngleCouple =
    std::make_pair(2.81_m, 405);
constexpr std::pair<units::meter_t, double> seventhDistanceWheelSpeedsCouple =
    std::make_pair(3.11_m, 4500);
constexpr std::pair<units::meter_t, double> seventhDistanceShooterAngleCouple =
    std::make_pair(3.11_m, 395);
constexpr std::pair<units::meter_t, double> eighthDistanceWheelSpeedsCouple =
    std::make_pair(3.40_m, 4500);
constexpr std::pair<units::meter_t, double> eighthDistanceShooterAngleCouple =
    std::make_pair(3.40_m, 380);
constexpr std::pair<units::meter_t, double> ninthDistanceWheelSpeedsCouple =
    std::make_pair(3.90_m, 4500);
constexpr std::pair<units::meter_t, double> ninthDistanceShooterAngleCouple =
    std::make_pair(3.90_m, 360);
extern const std::vector<std::pair<units::meter_t, double>> wheelSpeedsAccordingToDistance;
extern const std::vector<std::pair<units::meter_t, double>> shooterAngleAccordingToDistance;
constexpr double kStandByWheelRPM = 2000;
} // namespace ShooterConstant

namespace IntakeConstant {
constexpr int topMotorID = 14;
constexpr int bottomMotorID = 15;
constexpr int capteurID = 0;
constexpr double kVoltageCompensation = 10; // volts
constexpr double kCurrentLimit = 50;        // amperes
constexpr double kVoltageIntakeShot = 10;   // volts
constexpr double kVoltageIntakeCommand = 6.5;
} // namespace IntakeConstant

namespace BarreConstant {
constexpr int moteurPremierJointID = 19;
constexpr int moteurDeuxiemeJointID = 20;
constexpr int kTimeoutMs = 50;
constexpr double kNominalOutputForward = 0;
constexpr double kNominalOutputReverse = 0;
constexpr double kPeakOutputForward = 1;
constexpr double kPeakOutputReverse = -1;
constexpr double kVoltageCompensation = 10;
constexpr double kPeakCurrentLimit = 9;    // amperes
constexpr double kPeakCurrentDuration = 0; // ms
constexpr double kContinuousCurrent = 9;   // amperes
constexpr double kMaxAF1erJoint = 0.065;
constexpr double FDegToRad = std::numbers::pi / 180;
constexpr double absoluteEncoderOffset1erJoint = -940.1;              // 1/10 degre
constexpr double absoluteEncoderOffset2eJoint = 806;                  // 1/10 degre
constexpr double FConversionFactorPosition1erJoint = 1800.0 / 4096.0; // ticks * F -> 0.1 degres
constexpr double FConversionFactorVelocity1erJoint =
    18000.0 / 4096.0; // ticks/100ms * F -> 0.1 degres/s
constexpr double FConversionFactorAcceleration1erJoint =
    18000.0 / 4096.0; // ticks/100ms/s * F -> 0.1 degres/ s^2
constexpr double FConversionFactorPosition2eJoint = 3600.0 / 4096.0; // ticks * F -> 0.1 degres
constexpr double FConversionFactorVelocity2eJoint =
    36000.0 / 4096.0; // ticks/100ms * F -> 0.1 degres/s
constexpr double FConversionFactorAcceleration2eJoint =
    36000.0 / 4096.0; // ticks/100ms/s * F -> 0.1 degres/ s^2
constexpr double kPMotion1erJoint = 1.75;
constexpr double kIMotion1erJoint = 0;
constexpr double kDMotion1erJoint = 20;
constexpr double kFMotion1erJoint = 0;
constexpr double kPMotion2eJoint = 4;
constexpr double kIMotion2eJoint = 0;
constexpr double kDMotion2eJoint = 50;
constexpr double kFMotion2eJoint = 0;
constexpr double kVitesse1erJoint = 2750;
constexpr double kAcceleration1erJoint = 12000;
constexpr double kVitesse2eJoint = 6500;
constexpr double kAcceleration2eJoint = 150000;
constexpr double angleThreshold = 15;              // 1/10th degree
constexpr double kForwardSoftLimit1erJoint = 1200; // 1/10 degre
constexpr double kReverseSoftLimit1erJoint = 80;   // 1/10 degre
constexpr double kForwardSoftLimit2eJoint = 3500;  // 1/10 degre
constexpr double kReverseSoftLimit2eJoint = 100;   // 1/10 degre
constexpr double kPourcentageManual1erJoint = 0.15;
constexpr double kPourcentageManual2eJoint = 0.16;
constexpr double k1erJointAngleTrapApproach = 350;
constexpr double k1erJointAngleTrapIntermediaire = 900;
constexpr double k1erJointAngleTrapFinal = 1110;
constexpr double k1erJointAngleAmp = 1030;
constexpr double k2eJointAngleTrapApproach = 1900;
constexpr double k2eJointAngleTrapFinal = 2660;
constexpr double k2eJointAngleAmpApproach = 1600;
constexpr double k2eJointAngleAmpFinal = 560;
constexpr double k1erJointStartPosition = 80;
constexpr double k2eJointStartPosition = 900;
constexpr auto kTimerThreshold = 0.5_s;
} // namespace BarreConstant

namespace ClimberConstant {
constexpr int leftHookMotorID = 21;
constexpr int rightHookMotorID = 22;
constexpr double FConversionFactorPosition =
    (std::numbers::pi * 12.5) /
    15.6; // ticks * F -> 0.1 pouce ***native neo units: tours de moteurs
constexpr double FConversionFactorVelocity =
    (FConversionFactorPosition) / 60; // RPM * F -> 0.1 pouce / s
constexpr double kPHooksPosition = 0.115;
constexpr double kIHooksPosition = 0;
constexpr double kDHooksPosition = 0;
constexpr double kFFHooksPosition = 0;
constexpr double kAFHooks = -0.125;         // motor output voltage
constexpr double kVoltageCompensation = 10; // volts
constexpr double currentLimit = 50;         // amperes,
constexpr int positionPIDSlotID = 0;
constexpr int velocityPIDSlotID = 1;
constexpr double positionThreshold = 5; // 1/10 pouce
constexpr double kPourcentageManualHooks = 0.25;
constexpr double kPourcentageInitHooks = -0.075;
constexpr double kThresholdMotorStopped = 1;
constexpr auto kTimeDelayForInit = 0.1_s;
constexpr double FConversionTenthInchToMeter = 0.00254;
constexpr double FConversionTenthInchPerSecondToMeterPerSecond = 0.00254;
constexpr double FConversionTenthInchPerSecondSquaredToMeterPerSecondSquared = 0.00254;
constexpr double kVitesseExtensionHooks = 200;       // 1/10 inch per second
constexpr double kAccelerationExtensionHooks = 500;  // 1/10 inch per second squared
constexpr double kVitesseRetractionHooks = 75;       // 1/10 inch per second
constexpr double kAccelerationRetractionHooks = 500; // 1/10 inch per second squared
constexpr double kPositionInitReset = 0;
constexpr double kPositionAfterInit = 10;
constexpr double kForwardSoftLimit = 182.5;
constexpr double kPositionRetracted = -25;
constexpr double kPositionExtended = 210;
constexpr double kPositionExtendedTrap = 175;
} // namespace ClimberConstant

namespace LedConstants {
constexpr int kLedChannel = 9;
constexpr int kNumLeds = 12;

// moves by 1 LED each n 20-millis periods
constexpr int kSweepPrescale = 6;

// switches LEDs each n 20-millis periods
constexpr int kAlternatePrescale = 15;

// switches LEDs each n 20-millis periods
constexpr int kFlashPrescale = 5;

constexpr int kNumSweepFullOnLeds = 8;

// number of LEDs for 'in position' starting indicator
constexpr int kNumIndicatorLeds = 2;

// 8-bit R, G, B
struct Color {
    uint8_t red;
    uint8_t grn;
    uint8_t blu;
};

namespace Colors {
constexpr Color Off{0, 0, 0};
constexpr Color RedAlliance{255, 0, 0};
constexpr Color BlueAlliance{0, 0, 255};
constexpr Color NoteInIntake{255, 15, 0};
constexpr Color NoteSeen{255, 10, 150};
constexpr Color RobotInRange{0, 255, 0};
constexpr Color RobotInStartingPositionXY{0, 255, 0};
constexpr Color RobotInStartingPositionAngle{255, 105, 180};
} // namespace Colors

enum Animation {
    ALLIANCE,
    SPLIT,
};
} // namespace LedConstants
