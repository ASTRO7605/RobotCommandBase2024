#include "commands/ShootNote.h"

ShootNote::ShootNote(Base *p_Base, ShooterAngle *p_ShooterAngle, ShooterWheels *p_ShooterWheels,
                     Intake *p_Intake, Barre *p_Barre, LeftHook *p_LeftHook, RightHook *p_RightHook,
                     double wheelSpeeds, double shooterAngle, ScoringPositions scoringPlace)
    : m_pBase{p_Base}, m_pShooterAngle{p_ShooterAngle}, m_pShooterWheels{p_ShooterWheels},
      m_pIntake{p_Intake}, m_pBarre{p_Barre}, m_pLeftHook{p_LeftHook}, m_pRightHook{p_RightHook},
      targetSpeeds{wheelSpeeds}, finalShooterTargetAngle{shooterAngle}, scoringPlace{scoringPlace},
      m_RedescendreBarre{
          RedescendreBarre(m_pBarre, true, (scoringPlace == ScoringPositions::trap))} {
    AddRequirements({m_pShooterAngle, m_pShooterWheels, m_pIntake, m_pBase});
}

void ShootNote::Initialize() {
    if (!m_pIntake->IsObjectInIntake()) { // check if empty
        m_State = ShooterConstant::ShooterState::noNote;
    } else {
        m_State = ShooterConstant::ShooterState::init;
    }
}

void ShootNote::Execute() {
    switch (m_State) {
    case (ShooterConstant::ShooterState::init):
        if (scoringPlace == ScoringPositions::speaker) {
            if (finalShooterTargetAngle >=
                ShooterConstant::kIntermediateAngleShooter) { // check if need to flick shooter
                currentShooterTargetAngle = finalShooterTargetAngle;
            } else {
                currentShooterTargetAngle = ShooterConstant::kIntermediateAngleShooter;
            }
            // currentShooterTargetAngle = finalShooterTargetAngle;
            m_pShooterWheels->SetWheelSpeeds(targetSpeeds, true);
        } else {
            if (scoringPlace == ScoringPositions::amp) {
                targetPremierJoint = frc::Preferences::GetDouble("k1erJointAngleAmp");
                targetDeuxiemeJoint = frc::Preferences::GetDouble("k2eJointAngleAmpApproach");
            } else if (scoringPlace == ScoringPositions::trap) {
                targetPremierJoint = frc::Preferences::GetDouble("k1erJointAngleTrapFinal");
                targetDeuxiemeJoint = frc::Preferences::GetDouble("k2eJointStartPosition");
            }
            currentShooterTargetAngle = finalShooterTargetAngle;
            m_pShooterWheels->SetWheelSpeeds(targetSpeeds, false);
        }
        m_pBase->SetWheelsInXFormation();
        areWheelsAtRightSpeed = false;
        isShooterAngledRight = false;
        isPremierJointAngledRight = false;
        isDeuxiemeJointAngledRight = false;
        isLeftHookAtRightPose = false;
        isRightHookAtRightPose = false;
        readyToShoot = false;
        timer.Stop();
        timer.Reset();
        m_State = ShooterConstant::ShooterState::waitingForSubsystems;
        break;
    case (ShooterConstant::ShooterState::waitingForSubsystems):
        if (m_pShooterAngle->IsShooterAtTargetAngle(
                currentShooterTargetAngle)) { // is shooter at right angle
            isShooterAngledRight = true;
        }
        if (scoringPlace == ScoringPositions::speaker) {
            if (m_pShooterWheels->AreWheelsDoneAccelerating(
                    targetSpeeds, true)) { // did wheels reach their target
                areWheelsAtRightSpeed = true;
            }
            if (isShooterAngledRight && areWheelsAtRightSpeed) {
                readyToShoot = true;
            }
        } else if (scoringPlace == ScoringPositions::amp) {
            if (m_pBarre->Is1erJointAtTargetAngle(targetPremierJoint)) {
                isPremierJointAngledRight = true;
            }
            if (m_pBarre->Is2eJointAtTargetAngle(targetDeuxiemeJoint)) {
                isDeuxiemeJointAngledRight = true;
            }
            if (m_pShooterWheels->AreWheelsDoneAccelerating(
                    targetSpeeds, false)) { // did wheels reach their target
                areWheelsAtRightSpeed = true;
            }
            if (isShooterAngledRight && isPremierJointAngledRight && isDeuxiemeJointAngledRight &&
                areWheelsAtRightSpeed) {
                readyToShoot = true;
            }
        } else if (scoringPlace == ScoringPositions::trap) {
            if (m_pShooterWheels->AreWheelsDoneAccelerating(
                    targetSpeeds, false)) { // did wheels reach their target
                areWheelsAtRightSpeed = true;
            }
            if (m_pLeftHook->IsLeftHookAtTargetPosition(ClimberConstant::kPositionRetracted)) {
                isLeftHookAtRightPose = true;
            }
            if (m_pRightHook->IsRightHookAtTargetPosition(ClimberConstant::kPositionRetracted)) {
                isRightHookAtRightPose = true;
            }
            if (isShooterAngledRight && areWheelsAtRightSpeed && isLeftHookAtRightPose &&
                isRightHookAtRightPose) {
                readyToShoot = true;
            }
        }
        if (readyToShoot) {
            if (scoringPlace == ScoringPositions::trap) {
                targetDeuxiemeJoint = frc::Preferences::GetDouble("k2eJointAngleTrap");
                m_State = ShooterConstant::ShooterState::waitingForDeuxiemeJointTrap;
            } else {
                m_State = ShooterConstant::ShooterState::moveNoteInShooter;
            }
        }
        break;
    case (ShooterConstant::ShooterState::waitingForDeuxiemeJointTrap):
        if (m_pBarre->Is2eJointAtTargetAngle(frc::Preferences::GetDouble("k2eJointAngleTrap"))) {
            m_State = ShooterConstant::ShooterState::moveNoteInShooter;
        }
        break;
    case (ShooterConstant::ShooterState::moveNoteInShooter):
        m_pIntake->SetIntake(true, false, true);
        if (scoringPlace == ScoringPositions::speaker) { // to do the flick
            currentShooterTargetAngle = finalShooterTargetAngle;
        }
        m_State = ShooterConstant::ShooterState::waitingForNoteToEnter;
        break;
    case (ShooterConstant::ShooterState::waitingForNoteToEnter):
        if (m_pShooterWheels->IsObjectInShooter()) {
            if (scoringPlace == ScoringPositions::amp) {
                targetDeuxiemeJoint = frc::Preferences::GetDouble("k2eJointAngleAmpFinal");
            }
            isDeuxiemeJointAngledRight = false;
            m_State = ShooterConstant::ShooterState::waitingForNoteToExit;
        }
        break;
    case (ShooterConstant::ShooterState::waitingForNoteToExit):
        if (!m_pShooterWheels->IsObjectInShooter()) {
            timer.Restart();
            m_State = ShooterConstant::ShooterState::waitingForEnd;
        }
        break;
    case (ShooterConstant::ShooterState::waitingForEnd):
        if (m_pBarre->Is2eJointAtTargetAngle(targetDeuxiemeJoint)) {
            isDeuxiemeJointAngledRight = true;
        }
        if (timer.Get() >= ShooterConstant::timeThreshold &&
            (isDeuxiemeJointAngledRight || scoringPlace == ScoringPositions::speaker)) {
            m_State = ShooterConstant::ShooterState::complete;
        }
        break;
    case (ShooterConstant::ShooterState::complete):
        break;
    case (ShooterConstant::ShooterState::noNote):
        break;
    }
    frc::SmartDashboard::PutBoolean("areWheelsReadyToShoot", areWheelsAtRightSpeed);
    frc::SmartDashboard::PutBoolean("isShooterAngledRight", isShooterAngledRight);
    frc::SmartDashboard::PutBoolean("isPremierJointAngledRight", isPremierJointAngledRight);
    frc::SmartDashboard::PutBoolean("isDeuxiemeJointAngledRight", isDeuxiemeJointAngledRight);
    frc::SmartDashboard::PutBoolean("isLeftHookAtRightPose", isLeftHookAtRightPose);
    frc::SmartDashboard::PutBoolean("isRightHookAtRightPose", isRightHookAtRightPose);

    if (m_State != ShooterConstant::ShooterState::noNote) {
        m_pShooterAngle->SetShooterAngle(currentShooterTargetAngle); // to update kAF continuously
        if (scoringPlace != ScoringPositions::speaker) {
            m_pBarre->Set1erJointAngle(targetPremierJoint); // to update kAF continuously
            m_pBarre->Set2eJointAngle(targetDeuxiemeJoint); // to update kAF continuously
        }
    }
}

bool ShootNote::IsFinished() {
    if (m_State == ShooterConstant::ShooterState::complete ||
        m_State == ShooterConstant::ShooterState::noNote) {
        return true;
    }
    return false;
}

void ShootNote::End(bool interrupted) {
    m_pShooterWheels->StopWheels();
    m_pIntake->SetIntake(false, false, false);
    if (((scoringPlace == ScoringPositions::amp) || (scoringPlace == ScoringPositions::trap)) &&
        (m_State != ShooterConstant::ShooterState::noNote)) {
        m_RedescendreBarre.Schedule();
    }
}
