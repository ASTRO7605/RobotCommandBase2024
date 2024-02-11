#include "commands/ShootNoteSpeaker.h"

ShootNoteSpeaker::ShootNoteSpeaker(Base *p_Base, ShooterAngle *p_ShooterAngle, ShooterWheels *p_ShooterWheels,
                     Intake *p_Intake, double wheelSpeeds, double shooterAngle)
    : m_pBase{p_Base}, m_pShooterAngle{p_ShooterAngle}, m_pShooterWheels{p_ShooterWheels},
      m_pIntake{p_Intake}, targetSpeeds{wheelSpeeds}, targetAngle{shooterAngle} {
    AddRequirements({m_pShooterAngle, m_pShooterWheels, m_pIntake});
}

void ShootNoteSpeaker::Initialize() {
    noNote = !m_pIntake->IsObjectInIntake(); // check if empty
    if (noNote) {
        return;
    }
    m_pShooterWheels->SetWheelSpeeds(targetSpeeds);
    m_pShooterAngle->SetShooterAngle(targetAngle);
    m_pBase->SetWheelsInXFormation();
    areWheelsReadyToShoot = false;
    isShooterAngledRight = false;
    hasNoteGoneThroughShooter = false;
    timer.Stop();
    timer.Reset();
}

void ShootNoteSpeaker::Execute() {
    frc::SmartDashboard::PutBoolean("areWheelsReadyToShoot", areWheelsReadyToShoot);
    frc::SmartDashboard::PutBoolean("isShooterAngledRight", isShooterAngledRight);
    frc::SmartDashboard::PutBoolean("hasNoteGoneThroughShooter", hasNoteGoneThroughShooter); // debug
    if (noNote) {
        return;
    }
    if (!areWheelsReadyToShoot && m_pShooterWheels->AreWheelsDoneAccelerating(
                                      targetSpeeds)) { // did wheels reach their target
        areWheelsReadyToShoot = true;
    }
    if (!isShooterAngledRight &&
        m_pShooterAngle->IsShooterAtTargetAngle(targetAngle)) { // is shooter at right angle
        isShooterAngledRight = true;
    }
    if (areWheelsReadyToShoot &&
        isShooterAngledRight) { // if both are true, move note into the shooter
        m_pIntake->SetIntake(true, false);
    }
    if (!hasNoteGoneThroughShooter &&
        m_pShooterWheels
            ->IsObjectInShooter()) { // if seeing object for first time, note is in shooter
        hasNoteGoneThroughShooter = true;
    }
    if (hasNoteGoneThroughShooter && !m_pShooterWheels->IsObjectInShooter()) { // if has seen object but doesn't anymore
        timer.Start();
    }
}

bool ShootNoteSpeaker::IsFinished() {
    if ((timer.Get() >= ShooterConstant::timeThreshold) || noNote){
        return true;
    }
    return false;
}

void ShootNoteSpeaker::End(bool interrupted) {
    m_pShooterWheels->StopWheels();
    m_pIntake->SetIntake(false, false);
}
