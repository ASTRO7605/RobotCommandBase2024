#include "subsystems/Vision.h"

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
    // Invalid if not initialized
    photon::MultiTargetPNPResult latestMultiResult;
    photon::PhotonTrackedTarget latestSingleResult;

    bool isLatestMultiResultValid = false;
    bool isLatestSingleResultValid = true;

    if (camera->GetLatestResult().HasTargets()) {
        latestMultiResult = camera->GetLatestResult().MultiTagResult();
        latestSingleResult = camera->GetLatestResult().GetBestTarget();
        lastAprilTagSeen = latestSingleResult.GetFiducialId();

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

    auto poseEstimate = m_PhotonPoseEstimator.Update();
    if (!poseEstimate.has_value()) {
        latestMeasurement = {};
        return;
    }

    if (isLatestMultiResultValid) {
        target_distance = PythagorasXYZ(latestMultiResult.result.best.X().value(),
                                        latestMultiResult.result.best.Y().value(),
                                        latestMultiResult.result.best.Z().value());
        target_ambiguity = latestMultiResult.result.ambiguity;
        latestMeasurement = {poseEstimate->estimatedPose, poseEstimate->timestamp,
                             units::meter_t{target_distance}, target_ambiguity};

    } else if (isLatestSingleResultValid) {
        target_distance = PythagorasXYZ(latestSingleResult.bestCameraToTarget.X().value(),
                                        latestSingleResult.bestCameraToTarget.Y().value(),
                                        latestSingleResult.bestCameraToTarget.Z().value());
        target_ambiguity = latestSingleResult.poseAmbiguity;
        latestMeasurement = {poseEstimate->estimatedPose, poseEstimate->timestamp,
                             units::meter_t{target_distance}, target_ambiguity};

    } else {
        latestMeasurement = {};
    }
}

std::optional<PoseMeasurement> Vision::GetRobotPoseEstimate() {
    // quick check for NaNs
    if (latestMeasurement.has_value()) {
        if (std::isnan(latestMeasurement->pose.X().value()) ||
            std::isnan(latestMeasurement->pose.Y().value()) ||
            std::isnan(latestMeasurement->pose.Z().value())) {
            return {};
        }
        return latestMeasurement;
    }
    return {};
}

double Vision::PythagorasXYZ(double XDistance, double YDistance, double ZDistance) {
    return std::sqrt(XDistance * XDistance + YDistance * YDistance + ZDistance * ZDistance);
}