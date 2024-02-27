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
    void SetNoteSeen(bool seen) { note_seen = seen; };
    void SetRobotInRange(bool in_range) { robot_in_range = in_range; };
    void SetIsInStartingPositionXY(bool in_position) { in_starting_position_xy = in_position; };
    void SetIsInStartingPositionAngle(bool in_position) {
        in_starting_position_angle = in_position;
    };

  private:
    void color_sweep_with_indicators(LedConstants::Color main, LedConstants::Color indicator1,
                                     bool indicator1_on, LedConstants::Color indicator2,
                                     bool indicator2_on);
    void alternate(LedConstants::Color color1, LedConstants::Color color2);
    void split(LedConstants::Color top, LedConstants::Color bottom);

    frc::AddressableLED m_led{LedConstants::kLedChannel};
    std::array<frc::AddressableLED::LEDData, LedConstants::kNumLeds> m_buffer;
    LedConstants::Animation m_currentAnim;
    bool note_in_intake;
    bool note_seen;
    bool robot_in_range;
    bool in_starting_position_xy;
    bool in_starting_position_angle;
};