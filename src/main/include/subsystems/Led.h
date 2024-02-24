#pragma once

#include "Constants.h"
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>

#include <array>

class Led : public frc2::SubsystemBase {
  public:
    Led();
    void Periodic() override;

    void SetAnimation(LedConstants::Animation animation) { m_currentAnim = animation; };
    LedConstants::Animation GetAnimation() { return m_currentAnim; };

    void SetNoteInIntake(bool in_intake) { note_in_intake = in_intake; };
    void SetRobotInRange(bool in_range) { robot_in_range = in_range; };
    void SetInStartingPosition(bool in_position) { in_starting_position = in_position; };

  private:
    void rainbow();
    void color_sweep_with_indicator(LedConstants::Color main, LedConstants::Color indicator,
                                    bool indicator_on);
    void alternate(LedConstants::Color color1, LedConstants::Color color2);
    void split(LedConstants::Color top, LedConstants::Color bottom);

    frc::AddressableLED m_led{LedConstants::kLedChannel};
    std::array<frc::AddressableLED::LEDData, LedConstants::kNumLeds> m_buffer;
    LedConstants::Animation m_currentAnim;
    bool note_in_intake;
    bool robot_in_range;
    bool in_starting_position;
};