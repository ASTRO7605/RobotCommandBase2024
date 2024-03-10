#include "commands/FeedIntoShooter.h"

FeedIntoShooter::FeedIntoShooter(Intake *p_Intake) : m_pIntake{p_Intake} {
    AddRequirements({m_pIntake});
}

void FeedIntoShooter::Initialize() { m_pIntake->SetIntake(true, false, true); }

void FeedIntoShooter::Execute() {}

bool FeedIntoShooter::IsFinished() { return (!m_pIntake->IsObjectInIntake()); }

void FeedIntoShooter::End(bool interrupted) { m_pIntake->SetIntake(false, false, false); }
