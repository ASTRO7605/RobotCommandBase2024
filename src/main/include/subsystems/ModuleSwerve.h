// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Preferences.h>

class ModuleSwerve {
 public:
	ModuleSwerve(int TurningMotorID, int DrivingMotorID, int CANcoderID);
	double GetCANcoderAbsolutePosition();
	double GetCANcoderPosition();
	double GetTurningSparkMaxPosition();
	double GetDrivingPosition();
	frc::SwerveModuleState GetState();
	frc::SwerveModulePosition GetPosition();
	void SetDesiredState(frc::SwerveModuleState);
	void ResetEncoders();
	double GetDrivingVelocity();
 private:

  	rev::CANSparkMax m_TurningMotor;
	rev::CANSparkMax m_DrivingMotor;

	ctre::phoenix6::hardware::CANcoder m_TurningCANcoder;
	rev::SparkRelativeEncoder m_TurningSparkMaxEncoder;
	rev::SparkRelativeEncoder m_DrivingEncoder;

	rev::SparkPIDController m_TurningPIDController;
	rev::SparkPIDController m_DrivingPIDController;

	frc::SwerveModuleState m_DesiredState;

	
};