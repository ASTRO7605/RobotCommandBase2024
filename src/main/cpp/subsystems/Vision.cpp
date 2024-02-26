#include "subsystems/Vision.h"

#include <algorithm>
#include <numbers>
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

Vision::Vision(std::string_view table_name, frc::Transform3d cameraPose)
    : /*m_Vision{nt::NetworkTableInstance::GetDefault().GetTable(table_name)}*/
      robotToCam{cameraPose},
      m_PhotonPoseEstimator{frc::AprilTagFieldLayout{
                                frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo)},
                            photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
                            photon::PhotonCamera{table_name}, robotToCam} {
    // SetPipeline(VisionConstant::Pipeline);
    m_PhotonPoseEstimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);
    camera = (m_PhotonPoseEstimator.GetCamera());
}

void Vision::Periodic() {
    // photon::MultiTargetPNPResult multiResult = camera.GetLatestResult().MultiTagResult();
    // // photon::PhotonTrackedTarget singleResult = m_Vision.GetLatestResult().GetBestTarget();
    // if (((multiResult.result.isPresent) &&
    //      (0 < multiResult.result.ambiguity) && (multiResult.result.ambiguity <
    //      VisionConstant::ambiguityThreshold)) /*|| ((!multiResult.result.isPresent) && (0 <
    //      singleResult.GetPoseAmbiguity()) && (singleResult.GetPoseAmbiguity() <
    //      VisionConstant::ambiguityThreshold))*/) {
    //     auto result = m_PhotonPoseEstimator.Update();
    //     SendRobotPoseEstimate(result, multiResult);
    // }
    isLatestMultiResultValid = false;
    isLatestSingleResultValid = false;
    if (camera->GetLatestResult().HasTargets()) {
        lastAprilTagSeen = camera->GetLatestResult().GetBestTarget().GetFiducialId();
        latestMultiResult = camera->GetLatestResult().MultiTagResult();
        latestSingleResult = camera->GetLatestResult().GetBestTarget();
        if (camera->GetLatestResult().MultiTagResult().result.isPresent) {
            if (camera->GetLatestResult().MultiTagResult().result.ambiguity <
                VisionConstant::ambiguityThreshold) {
                isLatestMultiResultValid = true;
            }
        } else if (camera->GetLatestResult().GetBestTarget().poseAmbiguity <
                   VisionConstant::ambiguityThreshold) {
            isLatestSingleResultValid = true;
        }
    }
}

// double Vision::XError() { return m_Vision->GetNumber("tx", 0.0); }

// double Vision::YError() { return m_Vision->GetNumber("ty", 0.0); }

bool Vision::SeesValidTarget() { return (isLatestMultiResultValid || isLatestSingleResultValid); }

// int Vision::ViewTagID() {
//     // returns a double, but needs to be an int
//     // return static_cast<int>(m_Vision->GetNumber("tid", 0.0));
// }

// void Vision::SetPipeline(int pipeline) { m_Vision->PutNumber("pipeline", pipeline); }

// PoseMeasurement Vision::GetRobotPoseEstimate() {
//     // position in Pathweaver's coord system
//     // X, Y, Z, roll, pitch, yaw, latency(ms)
//     std::vector<double> coords =
//         m_Vision->GetNumberArray("botpose_wpiblue", std::array<double, 7>{});

//     std::vector<double> target_coords =
//         m_Vision->GetNumberArray("camerapose_targetspace", std::array<double, 6>{});

//     double target_distance = std::sqrt(target_coords[0] * target_coords[0] + // X
//                                        target_coords[1] * target_coords[1] + // Y
//                                        target_coords[2] * target_coords[2]   // Z
//     );

//     PoseMeasurement return_val{
//         frc::Pose3d{frc::Translation3d{units::meter_t{coords[0]}, units::meter_t{coords[1]},
//                                        units::meter_t{coords[2]}},

//                     frc::Rotation3d{units::degree_t{coords[3]}, units::degree_t{coords[4]},
//                                     units::degree_t{coords[5]}}},

//         units::millisecond_t{coords[6]},

//         units::meter_t{target_distance}};

//     return return_val;
// }

std::optional<PoseMeasurement> Vision::GetRobotPoseEstimate() {
    auto poseEstimate = m_PhotonPoseEstimator.Update();
    if (!poseEstimate.has_value())
        return {};

    if (isLatestMultiResultValid) {
        target_distance = GetAprilTagDistanceMeters(latestMultiResult.result.best.X().value(),
                                                    latestMultiResult.result.best.Y().value(),
                                                    latestMultiResult.result.best.Z().value());
        target_ambiguity = latestMultiResult.result.ambiguity;
    } else {
        target_distance =
            GetAprilTagDistanceMeters(latestSingleResult.bestCameraToTarget.X().value(),
                                      latestSingleResult.bestCameraToTarget.Y().value(),
                                      latestSingleResult.bestCameraToTarget.Z().value());
        target_ambiguity = latestSingleResult.poseAmbiguity;
    }

    PoseMeasurement return_val{poseEstimate->estimatedPose, poseEstimate->timestamp,
                               units::meter_t{target_distance}, target_ambiguity};
    if (std::isnan(return_val.pose.X().value()) || std::isnan(return_val.pose.Y().value()) ||
        std::isnan(return_val.pose.Z().value()) ||
        std::isnan(return_val.pose.Rotation().ToRotation2d().Degrees().value())) {
        return {};
    }
    return return_val;
}

double Vision::GetAprilTagDistanceMeters(double XDistance, double YDistance, double ZDistance) {
    return std::sqrt(XDistance * XDistance + YDistance * YDistance + ZDistance * ZDistance);
}

int Vision::GetAprilTagIDInView() { return lastAprilTagSeen; }