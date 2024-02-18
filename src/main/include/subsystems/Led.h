#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include "Constants.h"

#include <array>

class LED : public frc2::SubsystemBase
{
public:
    LED();
    void Periodic() override;

    void SetAnimation(LEDConstants::Animation animation);

private:
    void rainbow();
    void color_sweep(LEDConstants::Color color);
    void color_flash(LEDConstants::Color color);
    void solid_color(LEDConstants::Color color);
    void alternate(LEDConstants::Color color1, LEDConstants::Color color2);

    frc::AddressableLED m_led{LEDConstants::kLEDChannel};
    std::array<frc::AddressableLED::LEDData, LEDConstants::kNumLeds> m_buffer;
    LEDConstants::Animation m_currentAnim;
};