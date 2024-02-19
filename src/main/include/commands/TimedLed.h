#pragma once

#include <frc/Timer.h>
#include <frc2/Command/CommandBase.h>
#include <frc2/Command/CommandHelper.h>

#include "Constants.h"
#include "subsystems/Led.h"

class TimedLed : public frc2::CommandHelper<frc2::Command, TimedLed> {
  public:
    explicit TimedLed(Led &led, LedConstants::Animation anim, units::second_t duration);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

  private:
    Led &m_Led;
    LedConstants::Animation m_anim, m_end_anim;
    units::second_t m_duration;
    frc::Timer m_timer;
};