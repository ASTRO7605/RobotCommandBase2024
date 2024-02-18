// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Led.h"

#include <cstdint>

static uint8_t const rainbow_red[LEDConstants::kNumRainbowColors] = // hsv, 100% s, 100% v,
                                                                    // [0;360[ deg h by 7.5deg
                                                                    // increments
    {255, 255, 255, 255, 255, 255, 255, 255, 255, 223, 191, 159, 128, 96,  64,  32,
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     0,   32,  64,  96,  128, 159, 191, 223, 255, 255, 255, 255, 255, 255, 255, 255};

static uint8_t const rainbow_grn[LEDConstants::kNumRainbowColors] = {
    0,   32,  64,  96,  128, 159, 191, 223, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 223, 191, 159, 128, 96,  64,  32,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0};

static uint8_t const rainbow_blu[LEDConstants::kNumRainbowColors] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   32,  64,  96,  128, 159, 191, 223, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 223, 191, 159, 128, 96,  64,  32};

LED::LED() : m_currentAnim(LEDConstants::Animation::ALLIANCE) {
    m_led.SetLength(LEDConstants::kNumLeds);
    m_led.SetData(m_buffer);
    m_led.Start();
}

void LED::Periodic() {
    switch (m_currentAnim) {
    case LEDConstants::Animation::INTAKE_DONE:
        solid_color(LEDConstants::Colors::IntakeDone);
        break;

    case LEDConstants::Animation::TARGET_ACQUIRED:
        solid_color(LEDConstants::Colors::TargetAcquired);
        break;

    case LEDConstants::Animation::ALLIANCE: {
        auto alli = frc::DriverStation::GetAlliance();
        if (alli && alli.value() == frc::DriverStation::Alliance::kRed) {
            color_sweep(LEDConstants::Colors::RED_ALLIANCE);
        } else if (alli && alli.value() == frc::DriverStation::Alliance::kBlue) {
            color_sweep(LEDConstants::Colors::BLUE_ALLIANCE);
        } else {
            alternate(LEDConstants::Colors::RED_ALLIANCE, LEDConstants::Colors::BLUE_ALLIANCE);
        }
        break;
    }

    case LEDConstants::Animation::ERROR:
        color_flash(LEDConstants::Colors::Error);
        break;

    default:
        break;
    }
}

void LED::SetAnimation(LEDConstants::Animation animation) { m_currentAnim = animation; }

void LED::color_sweep(LEDConstants::Color color) {
    static double prescale_counter = 0.0;
    static int anim_counter = 0;

    prescale_counter += (1.0 / LEDConstants::kSweepPrescale);

    // calculate value to compare against as being prescale_counter after
    // kRequestedPrescale increments (may not be equal to 1)
    if (prescale_counter >= (LEDConstants::kSweepPrescale * (1.0 / LEDConstants::kSweepPrescale))) {
        // reset prescale and increment animation
        prescale_counter = 0.0;
        ++anim_counter;

        if (anim_counter >= LEDConstants::kNumLeds)
            anim_counter = 0;
    }

    // sequence comprised of 1 'fade out' LED, 8 'full on' LEDs, 1 'fade in' LED

    // current LED to update, incremented after setting a LED
    int led = anim_counter;

    // 'fade out' LED at beginning of sequence
    // fade out as prescale increases
    m_buffer[led].SetRGB(color.red * (1.0 - prescale_counter), color.grn * (1.0 - prescale_counter),
                         color.blu * (1.0 - prescale_counter));

    // next LED
    ++led;
    if (led >= LEDConstants::kNumLeds)
        led = 0;

    // turn on LEDs in middle of sequence
    for (int count = 0; count < LEDConstants::kNumRequestedFullOnLeds; ++count) {

        m_buffer[led].SetRGB(color.red, color.grn, color.blu);

        // next LED
        ++led;
        if (led >= LEDConstants::kNumLeds)
            led = 0;
    }

    // 'fade in' LED at end of sequence
    // fade in as prescale increases
    m_buffer[led].SetRGB(color.red * prescale_counter, color.grn * prescale_counter,
                         color.blu * prescale_counter);

    // next LED
    ++led;
    if (led >= LEDConstants::kNumLeds)
        led = 0;

    // turn off LEDs past sequence
    for (int count = 0;
         count < (LEDConstants::kNumLeds - LEDConstants::kNumRequestedFullOnLeds - 2); ++count) {
        m_buffer[led].SetRGB(LEDConstants::Colors::Off.red, LEDConstants::Colors::Off.grn,
                             LEDConstants::Colors::Off.blu);

        // next LED
        ++led;
        if (led >= LEDConstants::kNumLeds)
            led = 0;
    }

    m_led.SetData(m_buffer);
}

void LED::color_flash(LEDConstants::Color color) {
    static int prescale_counter = 0;
    static bool leds_on = true;

    ++prescale_counter;

    if (prescale_counter < LEDConstants::kFlashPrescale) {
        // prescale counter not reached yet
        return;
    }

    prescale_counter = 0;
    leds_on = !leds_on;

    if (leds_on) {
        for (int led = 0; led < LEDConstants::kNumLeds; ++led) {
            m_buffer[led].SetRGB(color.red, color.grn, color.blu);
        }
    } else {
        for (int led = 0; led < LEDConstants::kNumLeds; ++led) {
            m_buffer[led].SetRGB(LEDConstants::Colors::Off.red, LEDConstants::Colors::Off.grn,
                                 LEDConstants::Colors::Off.blu);
        }
    }

    m_led.SetData(m_buffer);
}

void LED::alternate(LEDConstants::Color color1, LEDConstants::Color color2) {
    static int prescale_counter = 0;
    static bool is1 = true;

    ++prescale_counter;

    if (prescale_counter < LEDConstants::kAlternatePrescale) {
        // prescale counter not reached yet
        return;
    }

    prescale_counter = 0;
    is1 = !is1;

    LEDConstants::Color current = is1 ? color1 : color2;

    for (int led = 0; led < LEDConstants::kNumLeds; ++led) {
        m_buffer[led].SetRGB(current.red, current.blu, current.grn);
    }

    m_led.SetData(m_buffer);
}

void LED::solid_color(LEDConstants::Color color) {
    for (int led = 0; led < (LEDConstants::kNumLeds); ++led) {
        m_buffer[led].SetRGB(color.red, color.grn, color.blu);
    }

    m_led.SetData(m_buffer);
}