// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Led.h"

Led::Led()
    : m_currentAnim{LedConstants::Animation::ALLIANCE}, note_in_intake{false}, robot_in_range{true},
      robot_aligned{false} {
    m_led.SetLength(LedConstants::kNumLeds);
    m_led.SetData(m_buffer);
    m_led.Start();
}

void Led::Periodic() {
    switch (m_currentAnim) {
    case LedConstants::Animation::ALLIANCE: {
        // dereferencing a null std::optional is UB!
        // check if none
        auto alli = frc::DriverStation::GetAlliance();
        if (alli && alli.value() == frc::DriverStation::Alliance::kRed) {
            color_sweep(LedConstants::Colors::RedAlliance);
        } else if (alli && alli.value() == frc::DriverStation::Alliance::kBlue) {
            color_sweep(LedConstants::Colors::BlueAlliance);
        } else {
            alternate(LedConstants::Colors::RedAlliance, LedConstants::Colors::BlueAlliance);
        }
        break;
    }

    case LedConstants::Animation::SPLIT: {
        LedConstants::Color top = LedConstants::Colors::Off;
        if (note_in_intake) {
            top = LedConstants::Colors::NoteInIntake;
        } else if (note_seen) {
            top = LedConstants::Colors::NoteSeen;
        }
        auto bottom =
            robot_in_range ? LedConstants::Colors::RobotInRange : LedConstants::Colors::Off;
        split_with_bottom_blink(top, bottom, robot_aligned);

        break;
    }
    default:
        break;
    }
}

void Led::alternate(LedConstants::Color color1, LedConstants::Color color2) {
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
        m_buffer[led].SetRGB(current.red, current.grn, current.blu);
    }

    m_led.SetData(m_buffer);
}

void Led::color_sweep(LedConstants::Color color) {
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
    for (int count = 0; count < LedConstants::kNumSweepFullOnLeds; ++count) {

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
    for (int count = 0; count < (LedConstants::kNumLeds - LedConstants::kNumSweepFullOnLeds - 2);
         ++count) {
        m_buffer[led].SetRGB(LedConstants::Colors::Off.red, LedConstants::Colors::Off.grn,
                             LedConstants::Colors::Off.blu);

        // next LED
        ++led;
        if (led >= LedConstants::kNumLeds)
            led = 0;
    }

    m_led.SetData(m_buffer);
}

void Led::split_with_bottom_blink(LedConstants::Color top, LedConstants::Color bottom,
                                  bool bottom_blink) {
    static int prescale_counter = 0;
    static bool bottom_on = true;

    ++prescale_counter;

    if (prescale_counter >= LedConstants::kFlashPrescale) {
        prescale_counter = 0;
        bottom_on = !bottom_on;
    }

    if (bottom_blink && !bottom_on) {
        bottom = LedConstants::Colors::Off;
    }

    for (int led = 0; led < (LedConstants::kNumLeds / 2); ++led) {
        m_buffer[led].SetRGB(bottom.red, bottom.grn, bottom.blu);
    }

    for (int led = (LedConstants::kNumLeds / 2); led < (LedConstants::kNumLeds); ++led) {
        m_buffer[led].SetRGB(top.red, top.grn, top.blu);
    }

    m_led.SetData(m_buffer);
}