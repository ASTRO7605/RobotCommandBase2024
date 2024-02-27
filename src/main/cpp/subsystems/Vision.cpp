#include "subsystems/Vision.h"

#include <algorithm>
#include <numbers>
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

Vision::Vision(std::string_view table_name, frc::Transform3d cameraPose)
    : 
      robotToCam{cameraPose},
      m_PhotonPoseEstimator{frc::AprilTagFieldLayout{
                                frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo)},
                            photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
                            photon::PhotonCamera{table_name}, robotToCam} {
    m_PhotonPoseEstimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);
    camera = (m_PhotonPoseEstimator.GetCamera());
}

void Vision::Periodic() {
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

bool Vision::SeesValidTarget() { return (isLatestMultiResultValid || isLatestSingleResultValid); }

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