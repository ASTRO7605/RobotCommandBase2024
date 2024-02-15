#include "commands/LeftHookPositionTest.h"

LeftHookPositionTest::LeftHookPositionTest(LeftHook *p_LeftHook, double position, double maxSpeed,
                                           double maxAcceleration)
    : m_pLeftHook{p_LeftHook}, position{position}, maxSpeed{maxSpeed},
      maxAcceleration{maxAcceleration},
      profile{frc::TrapezoidProfile<units::meters>::Constraints{
          units::meters_per_second_t{
              maxSpeed * ClimberConstant::FConversionTenthInchPerSecondToMeterPerSecond},
          units::meters_per_second_squared_t{
              maxAcceleration *
              ClimberConstant::FConversionTenthInchPerSecondSquaredToMeterPerSecondSquared}}} {
    AddRequirements({m_pLeftHook});
}

void LeftHookPositionTest::Initialize() {
    m_Timer.Restart();
    endGoal = frc::TrapezoidProfile<units::meters>::State{
        units::meter_t{position * ClimberConstant::FConversionTenthInchToMeter}, 0_mps};
    startState = m_pLeftHook->GetLeftHookState();
}

void LeftHookPositionTest::Execute() {
    setpoint = profile.Calculate(m_Timer.Get(), startState, endGoal);
    frc::SmartDashboard::PutNumber("setpointPosition",
                                   setpoint.position.value() /
                                       ClimberConstant::FConversionTenthInchToMeter);
    frc::SmartDashboard::PutNumber(
        "setpointVelocity",
        setpoint.velocity.value() / ClimberConstant::FConversionTenthInchPerSecondToMeterPerSecond);
    m_pLeftHook->SetLeftHookPosition(setpoint.position.value() /
                                     ClimberConstant::FConversionTenthInchToMeter);
}

bool LeftHookPositionTest::IsFinished() {
    if (profile.IsFinished(m_Timer.Get())) {
        return true;
    }
    return false;
}

void LeftHookPositionTest::End(bool) { /*m_pLeftHook->ManualLeftHook(0);*/
}