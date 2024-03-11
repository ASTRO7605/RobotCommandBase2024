#pragma once

#include "Constants.h"
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>

#include <array>

class Led : public frc2::SubsystemBase
{
  public:
    Led();
    void Periodic() override;

    void SetAnimation(LedConstants::Animation animation) { m_currentAnim = animation; };
    LedConstants::Animation GetAnimation() { return m_currentAnim; };

    void SetNoteInIntake(bool in_intake) { note_in_intake = in_intake; };
    void SetNoteSeen(bool seen) { note_seen = seen; };
    void SetRobotInRange(bool in_range) { robot_in_range = in_range; };
    void SetRobotAligned(bool aligned) { robot_aligned = aligned; };

  private:
    void color_sweep(LedConstants::Color color);
    void alternate(LedConstants::Color color1, LedConstants::Color color2);
    void split_with_bottom_blink(LedConstants::Color top, LedConstants::Color bottom,
                                 bool bottom_blink);

    frc::AddressableLED m_led{LedConstants::kLedChannel};
    std::array<frc::AddressableLED::LEDData, LedConstants::kNumLeds> m_buffer;
    LedConstants::Animation m_currentAnim;
    bool note_in_intake;
    bool note_seen;
    bool robot_in_range;
    bool robot_aligned;
};