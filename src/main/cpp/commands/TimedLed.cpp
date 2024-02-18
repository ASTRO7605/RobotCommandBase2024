#include "commands/TimedLed.h"

TimedLed::TimedLed(Led &led, LedConstants::Animation anim, units::second_t duration)
    : m_Led(led), m_anim(start_anim), m_duration(duration) {
    AddRequirements({&m_Led});
}

void TimedLed::Initialize() {
    m_Led.SetAnimation(m_anim);
    m_end_anim = m_Led.GetAnimation();

    m_timer.Reset();
    m_timer.Start();
}

void TimedLed::Execute() {}

bool TimedLed::IsFinished() { return m_timer.Get() >= m_duration; }

// Restore initial animation in case we're interrupted to gracefully reset LED state
void TimedLed::End(bool interrupted) { m_Led.SetAnimation(m_end_anim); }