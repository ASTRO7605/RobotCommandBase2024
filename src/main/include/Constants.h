#pragma once
#include <array>
#include <cstdint>
#include <frc/trajectory/TrapezoidProfile.h>
#include <iostream>
#include <numbers>
#include <string>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

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

enum class LedMode : int { Off = 1, Flash = 2, On = 3 };
} // namespace VisionConstant
namespace PoseEstimationConstant {
// x(m), y(m), theta(rad)
constexpr std::array<double, 3> kStateStdDevs{0.1, 0.1, 0.005};
constexpr std::array<double, 3> kVisionStdDevs{0.9, 0.9, 0.995};
constexpr std::array<double, 3> kVisionStdDevs_XYPerMeterSquared_Front{0.5, 0.5, 0.999};
constexpr std::array<double, 3> kVisionStdDevs_XYPerMeterSquared_Back{0.5, 0.5, 0.999};
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
    constexpr int Motor1ID = 0;
    constexpr int Motor2ID = 0;
    // voir les valeurs PID
    constexpr double kPMotors = 0.0;
    constexpr double kIMotors = 0.0;
    constexpr double kDMotors = 0.0;
    constexpr double kMinInput = 0;
    constexpr double kMaxInput = 5500; // un peu moins que la capacité de rmp d'un moteur NEO rev
    constexpr double speed = 5;
} // namespace Shooter