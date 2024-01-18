#include "subsystems/Vision.h"

#include <algorithm>
#include <numbers>
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

Vision::Vision(std::string_view table_name)
    : m_Vision{nt::NetworkTableInstance::GetDefault().GetTable(table_name)} {
    SetPipeline(VisionConstant::Pipeline);
}

void Vision::Periodic() {}

double Vision::XError() { return m_Vision->GetNumber("tx", 0.0); }

double Vision::YError() { return m_Vision->GetNumber("ty", 0.0); }

bool Vision::SeesValidTarget() { return static_cast<bool>(m_Vision->GetNumber("tv", 0.0)); }

int Vision::ViewTagID() {
    // returns a double, but needs to be an int
    return static_cast<int>(m_Vision->GetNumber("tid", 0.0));
}

void Vision::SetPipeline(int pipeline) { m_Vision->PutNumber("pipeline", pipeline); }

PoseMeasurement Vision::GetRobotPoseEstimate() {
    // position in Pathweaver's coord system
    // X, Y, Z, roll, pitch, yaw, latency(ms)
    std::vector<double> coords =
        m_Vision->GetNumberArray("botpose_wpiblue", std::array<double, 7>{});

    std::vector<double> target_coords =
        m_Vision->GetNumberArray("camerapose_targetspace", std::array<double, 6>{});

    double target_distance = std::sqrt(target_coords[0] * target_coords[0] + // X
                                       target_coords[1] * target_coords[1] + // Y
                                       target_coords[2] * target_coords[2]   // Z
    );

    PoseMeasurement return_val{
        frc::Pose3d{frc::Translation3d{units::meter_t{coords[0]}, units::meter_t{coords[1]},
                                       units::meter_t{coords[2]}},

                    frc::Rotation3d{units::degree_t{coords[3]}, units::degree_t{coords[4]},
                                    units::degree_t{coords[5]}}},

        units::millisecond_t{coords[6]},

        units::meter_t{target_distance}};

    return return_val;
}
