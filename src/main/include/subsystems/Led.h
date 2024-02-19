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

  private:
    void rainbow();
    void color_sweep(LedConstants::Color color);
    void color_flash(LedConstants::Color color);
    void solid_color(LedConstants::Color color);
    void alternate(LedConstants::Color color1, LedConstants::Color color2);

    frc::AddressableLED m_led{LedConstants::kLedChannel};
    std::array<frc::AddressableLED::LEDData, LedConstants::kNumLeds> m_buffer;
    LedConstants::Animation m_currentAnim;
};