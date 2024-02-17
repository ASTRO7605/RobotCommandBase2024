#include "commands/RightHookPositionTest.h"

RightHookPositionTest::RightHookPositionTest(RightHook *p_RightHook, double position,
                                             double maxSpeed, double maxAcceleration)
    : m_pRightHook{p_RightHook}, position{position}, maxSpeed{maxSpeed},
      maxAcceleration{maxAcceleration},
      profile{frc::TrapezoidProfile<units::meters>::Constraints{
          units::meters_per_second_t{
              maxSpeed * ClimberConstant::FConversionTenthInchPerSecondToMeterPerSecond},
          units::meters_per_second_squared_t{
              maxAcceleration *
              ClimberConstant::FConversionTenthInchPerSecondSquaredToMeterPerSecondSquared}}} {
    AddRequirements({m_pRightHook});
}

void RightHookPositionTest::Initialize() {
    m_Timer.Restart();
    endGoal = frc::TrapezoidProfile<units::meters>::State{
        units::meter_t{position * ClimberConstant::FConversionTenthInchToMeter}, 0_mps};
    startState = m_pRightHook->GetRightHookState();
}

void RightHookPositionTest::Execute() {
    setpoint = profile.Calculate(m_Timer.Get(), startState, endGoal);
    frc::SmartDashboard::PutNumber("setpointPosition",
                                   setpoint.position.value() /
                                       ClimberConstant::FConversionTenthInchToMeter);
    frc::SmartDashboard::PutNumber(
        "setpointVelocity",
        setpoint.velocity.value() / ClimberConstant::FConversionTenthInchPerSecondToMeterPerSecond);
    m_pRightHook->SetRightHookPosition(setpoint.position.value() /
                                       ClimberConstant::FConversionTenthInchToMeter);
}

bool RightHookPositionTest::IsFinished() {
    if (profile.IsFinished(m_Timer.Get())) {
        return true;
    }
    return false;
}

void RightHookPositionTest::End(bool) { /*m_pRightHook->ManualRightHook(0);*/
}