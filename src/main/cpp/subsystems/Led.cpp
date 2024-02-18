// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Led.h"

#include <cstdint>

static uint8_t const rainbow_red[LedConstants::kNumRainbowColors] = // hsv, 100% s, 100% v,
                                                                    // [0;360[ deg h by 7.5deg
                                                                    // increments
    {255, 255, 255, 255, 255, 255, 255, 255, 255, 223, 191, 159, 128, 96,  64,  32,
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     0,   32,  64,  96,  128, 159, 191, 223, 255, 255, 255, 255, 255, 255, 255, 255};

static uint8_t const rainbow_grn[LedConstants::kNumRainbowColors] = {
    0,   32,  64,  96,  128, 159, 191, 223, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 223, 191, 159, 128, 96,  64,  32,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0};

static uint8_t const rainbow_blu[LedConstants::kNumRainbowColors] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   32,  64,  96,  128, 159, 191, 223, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 223, 191, 159, 128, 96,  64,  32};

LED::LED() : m_currentAnim(LEDConstants::Animation::ALLIANCE) {
    m_led.SetLength(LedConstants::kNumLeds);
    m_led.SetData(m_buffer);
    m_led.Start();
}

void LED::Periodic() {
    switch (m_currentAnim) {
    case LedConstants::Animation::INTAKE_DONE:
        solid_color(LedConstants::Colors::IntakeDone);
        break;

    case LedConstants::Animation::TARGET_ACQUIRED:
        solid_color(LedConstants::Colors::TargetAcquired);
        break;

    case LedConstants::Animation::ALLIANCE: {
        auto alli = frc::DriverStation::GetAlliance();
        if (alli && alli.value() == frc::DriverStation::Alliance::kRed) {
            color_sweep(LedConstants::Colors::RED_ALLIANCE);
        } else if (alli && alli.value() == frc::DriverStation::Alliance::kBlue) {
            color_sweep(LedConstants::Colors::BLUE_ALLIANCE);
        } else {
            alternate(LedConstants::Colors::RED_ALLIANCE, LedConstants::Colors::BLUE_ALLIANCE);
        }
        break;
    }

    case LedConstants::Animation::ERROR:
        color_flash(LedConstants::Colors::Error);
        break;

    default:
        break;
    }
}

void LED::SetAnimation(LedConstants::Animation animation) { m_currentAnim = animation; }

LedConstants::Animation LED::GetAnimation(LedConstants::Animation animation) {
    return m_currentAnim;
}

void LED::color_sweep(LedConstants::Color color) {
    static double prescale_counter = 0.0;
    static int anim_counter = 0;

    prescale_counter += (1.0 / LedConstants::kSweepPrescale);

    // calculate value to compare against as being prescale_counter after
    // kRequestedPrescale increments (may not be equal to 1)
    if (prescale_counter >= (LedConstants::kSweepPrescale * (1.0 / LedConstants::kSweepPrescale))) {
        // reset prescale and increment animation
        prescale_counter = 0.0;
        ++anim_counter;

        if (anim_counter >= LedConstants::kNumLeds)
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
    if (led >= LedConstants::kNumLeds)
        led = 0;

    // turn on LEDs in middle of sequence
    for (int count = 0; count < LedConstants::kNumRequestedFullOnLeds; ++count) {

        m_buffer[led].SetRGB(color.red, color.grn, color.blu);

        // next LED
        ++led;
        if (led >= LedConstants::kNumLeds)
            led = 0;
    }

    // 'fade in' LED at end of sequence
    // fade in as prescale increases
    m_buffer[led].SetRGB(color.red * prescale_counter, color.grn * prescale_counter,
                         color.blu * prescale_counter);

    // next LED
    ++led;
    if (led >= LedConstants::kNumLeds)
        led = 0;

    // turn off LEDs past sequence
    for (int count = 0;
         count < (LedConstants::kNumLeds - LedConstants::kNumRequestedFullOnLeds - 2); ++count) {
        m_buffer[led].SetRGB(LedConstants::Colors::Off.red, LedConstants::Colors::Off.grn,
                             LedConstants::Colors::Off.blu);

        // next LED
        ++led;
        if (led >= LedConstants::kNumLeds)
            led = 0;
    }

    m_led.SetData(m_buffer);
}

void LED::color_flash(LedConstants::Color color) {
    static int prescale_counter = 0;
    static bool leds_on = true;

    ++prescale_counter;

    if (prescale_counter < LedConstants::kFlashPrescale) {
        // prescale counter not reached yet
        return;
    }

    prescale_counter = 0;
    leds_on = !leds_on;

    if (leds_on) {
        for (int led = 0; led < LedConstants::kNumLeds; ++led) {
            m_buffer[led].SetRGB(color.red, color.grn, color.blu);
        }
    } else {
        for (int led = 0; led < LedConstants::kNumLeds; ++led) {
            m_buffer[led].SetRGB(LedConstants::Colors::Off.red, LedConstants::Colors::Off.grn,
                                 LedConstants::Colors::Off.blu);
        }
    }

    m_led.SetData(m_buffer);
}

void LED::alternate(LedConstants::Color color1, LedConstants::Color color2) {
    static int prescale_counter = 0;
    static bool is1 = true;

    ++prescale_counter;

    if (prescale_counter < LedConstants::kAlternatePrescale) {
        // prescale counter not reached yet
        return;
    }

    prescale_counter = 0;
    is1 = !is1;

    LedConstants::Color current = is1 ? color1 : color2;

    for (int led = 0; led < LedConstants::kNumLeds; ++led) {
        m_buffer[led].SetRGB(current.red, current.blu, current.grn);
    }

    m_led.SetData(m_buffer);
}

void LED::solid_color(LedConstants::Color color) {
    for (int led = 0; led < (LedConstants::kNumLeds); ++led) {
        m_buffer[led].SetRGB(color.red, color.grn, color.blu);
    }

    m_led.SetData(m_buffer);
}