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

enum ScoringPositions {
    speaker,
    amp,
    trap
};
namespace DriveConstant {
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
enum IdleMode { COAST = 0, BRAKE = 1 };
constexpr double kVoltageCompensation = 10;             // volts
constexpr double DegreesToRad = std::numbers::pi / 180; // degrees * conversion factor => rad
constexpr int PowerDistributionHubID = 1;
constexpr int FrontRightTurningID = 2;
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
} // namespace DriveConstant

namespace VisionConstant {
constexpr int Pipeline = 0; // only one pipeline (AprilTags)
constexpr std::string_view TableNameFront = "limelight-front";
constexpr std::string_view TableNameBack = "limelight-back";
constexpr double ambiguityThreshold = 0.2;
extern const frc::Transform3d frontCameraTransform;
extern const frc::Transform3d backCameraTransform;

enum class LedMode : int { Off = 1, Flash = 2, On = 3 };
} // namespace VisionConstant
namespace PoseEstimationConstant {
// x(m), y(m), theta(rad)
constexpr std::array<double, 3> kStateStdDevs{0.1, 0.1, 0.001};
constexpr std::array<double, 3> kVisionStdDevs{0.9, 0.9, 0.995};
constexpr std::array<double, 3> kVisionStdDevs_XYPerMeterSquared_Front{2.0, 2.0, 0.999};
constexpr std::array<double, 3> kVisionStdDevs_XYPerMeterSquared_Back{2.5, 2.5, 0.999};
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
} // namespace OIConstant

namespace ShooterConstant {
enum ShooterState {
    init,
    waitingForSubsystems,
    moveNoteInShooter,
    waitingForNoteToEnter,
    waitingForNoteToExit,
    waitingForEnd,
    complete
};
constexpr int leftMotorID = 16;
constexpr int rightMotorID = 17;
constexpr int angleMotorID = 18;
constexpr int capteurID = 1;
constexpr double absoluteEncoderOffset = -2078.12; // 1/10 degre
constexpr double flywheelsSpeedSpeaker = 4500;     // RPM
constexpr double flywheelsSpeedAmp = 0;            // RPM
constexpr double flywheelsSpeedTrap = 0;           // RPM
constexpr double speedThreshold = 50;              // RPM
constexpr auto timeThreshold = 0.1_s;
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
constexpr double kPPositionAngle = 9.5;
constexpr double kIPositionAngle = 0;
constexpr double kDPositionAngle = 10.0;
constexpr double kFPositionAngle = 4.5;
constexpr double kPVitesseAngle = 0;
constexpr double kIVitesseAngle = 0;
constexpr double kDVitesseAngle = 0;
constexpr double kFVitesseAngle = 0;
constexpr double kVitesseAngle = 2000;      // 1/10 degre par seconde
constexpr double kAccelerationAngle = 6000; // 1/10 degre par seconde^2
constexpr double kPercentOutputAngle = 0.1; // dixieme de degre par seconde pour mode manuel
constexpr double angleThreshold = 5;        // dixieme de degre
constexpr double kMaxAF = 0.0445;
constexpr double FDegToRad = M_PI / 180;
constexpr double kPeakCurrentLimit = 9;    // amperes
constexpr double kPeakCurrentDuration = 0; // ms
constexpr double kContinuousCurrent = 9;   // amperes
constexpr double kForwardSoftLimit = 740;  // 1/10 degre
constexpr double kReverseSoftLimit = 200;  // 1/10 degre
} // namespace ShooterConstant

namespace IntakeConstant {
constexpr int topMotorID = 14;
constexpr int bottomMotorID = 15;
constexpr int capteurID = 0;
constexpr double kVoltageCompensation = 10; // volts
constexpr double kCurrentLimit = 50;        // amperes
constexpr double kVoltageIntake = 7.5;      // volts
} // namespace IntakeConstant

namespace BarreConstant {
constexpr int moteurPremierJointID = 19;
constexpr int moteurDeuxiemeJointID = 20;
constexpr int kTimeoutMs = 30;
constexpr double kNominalOutputForward = 0;
constexpr double kNominalOutputReverse = 0;
constexpr double kPeakOutputForward = 1;
constexpr double kPeakOutputReverse = -1;
constexpr double kVoltageCompensation = 10;
constexpr double kPeakCurrentLimit = 9;    // amperes
constexpr double kPeakCurrentDuration = 0; // ms
constexpr double kContinuousCurrent = 9;   // amperes
constexpr double kMaxAF1erJoint = 0;
constexpr double kMaxAF2eJoint = 0;
constexpr double FDegToRad = M_PI / 180;
constexpr double kCdMOffset1erJoint = 0;
constexpr double kCdMOffset2eJoint = 0;
constexpr double absoluteEncoderOffset1erJoint = -940.1;              // 1/10 degre
constexpr double absoluteEncoderOffset2eJoint = -3490.14;             // 1/10 degre
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
constexpr double kPMotion1erJoint = 0;
constexpr double kIMotion1erJoint = 0;
constexpr double kDMotion1erJoint = 0;
constexpr double kFMotion1erJoint = 0;
constexpr double kPMotion2eJoint = 0;
constexpr double kIMotion2eJoint = 0;
constexpr double kDMotion2eJoint = 0;
constexpr double kFMotion2eJoint = 0;
constexpr double kVitesse1erJoint = 0;
constexpr double kAcceleration1erJoint = 0;
constexpr double kVitesse2eJoint = 0;
constexpr double kAcceleration2eJoint = 0;
constexpr double angleThreshold = 5; // 1/10th degree
constexpr double kForwardSoftLimit1erJoint = 0; // 1/10 degre
constexpr double kReverseSoftLimit1erJoint = 0; // 1/10 degre
constexpr double kForwardSoftLimit2eJoint = 0;  // 1/10 degre
constexpr double kReverseSoftLimit2eJoint = 0;  // 1/10 degre
constexpr double kPourcentageManual1erJoint = 0;
constexpr double kPourcentageManual2eJoint = 0;
constexpr double k1erJointAngleTrap = 0;
constexpr double k1erJointAngleAmp = 0;
constexpr double k2eJointAngleTrapApproach = 0;
constexpr double k2eJointAngleTrapFinal = 0;
constexpr double k2eJointAngleAmpApproach = 0;
constexpr double k2eJointAngleAmpFinal = 0;

} // namespace BarreConstant

namespace ClimberConstant {
constexpr int leftHookMotorID = 21;
constexpr int rightHookMotorID = 22;
constexpr double FConversionFactorPosition = M_PI * 10 / 655.2; // ticks * F -> 0.1 pouce
constexpr double FConversionFactorVelocity =
    (FConversionFactorPosition * 42) / 60; // RPM * F -> 0.1 pouce / s
constexpr double kPHooksPosition = 0;
constexpr double kIHooksPosition = 0;
constexpr double kDHooksPosition = 0;
constexpr double kFFHooksPosition = 0;
constexpr double kPHooksVelocity = 0;
constexpr double kIHooksVelocity = 0;
constexpr double kDHooksVelocity = 0;
constexpr double kFFHooksVelocity = 0;
constexpr double kAFHooks = 0;              // motor output voltage
constexpr double kVoltageCompensation = 10; // volts
constexpr double currentLimit = 50;         // amperes,
constexpr int positionPIDSlotID = 0;
constexpr int velocityPIDSlotID = 1;
constexpr double positionThreshold = 5; // 1/10 pouce
} // namespace ClimberConstant