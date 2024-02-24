// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Led.h"

Led::Led()
    : m_currentAnim{LedConstants::Animation::ALLIANCE}, note_in_intake{false},
      robot_in_range{true} {
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
            color_sweep_with_indicator(LedConstants::Colors::RedAlliance,
                                       LedConstants::Colors::RobotInRange, in_starting_position);
        } else if (alli && alli.value() == frc::DriverStation::Alliance::kBlue) {
            color_sweep_with_indicator(LedConstants::Colors::BlueAlliance,
                                       LedConstants::Colors::RobotInRange, in_starting_position);
        } else {
            alternate(LedConstants::Colors::RedAlliance, LedConstants::Colors::BlueAlliance);
        }
        break;
    }

    case LedConstants::Animation::SPLIT: {
        auto top = note_in_intake ? LedConstants::Colors::NoteInIntake : LedConstants::Colors::Off;
        auto bottom =
            robot_in_range ? LedConstants::Colors::RobotInRange : LedConstants::Colors::Off;
        split(top, bottom);

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

void Led::color_sweep_with_indicator(LedConstants::Color main, LedConstants::Color indicator,
                                     bool indicator_on) {
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
    m_buffer[led].SetRGB(main.red * (1.0 - prescale_counter), main.grn * (1.0 - prescale_counter),
                         main.blu * (1.0 - prescale_counter));

    // next LED
    ++led;
    if (led >= LedConstants::kNumLeds)
        led = 0;

    // turn on LEDs in middle of sequence
    for (int count = 0; count < LedConstants::kNumSweepFullOnLeds; ++count) {

        m_buffer[led].SetRGB(main.red, main.grn, main.blu);

        // next LED
        ++led;
        if (led >= LedConstants::kNumLeds)
            led = 0;
    }

    // 'fade in' LED at end of sequence
    // fade in as prescale increases
    m_buffer[led].SetRGB(main.red * prescale_counter, main.grn * prescale_counter,
                         main.blu * prescale_counter);

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

    // overwrite last LEDs with indicator color if on
    if (indicator_on) {
        for (int i = LedConstants::kNumLeds - LedConstants::kNumIndicatorLeds;
             i < LedConstants::kNumLeds; ++i) {
            m_buffer[i].SetRGB(indicator.red, indicator.grn, indicator.blu);
        }
    }

    m_led.SetData(m_buffer);
}

void Led::split(LedConstants::Color top, LedConstants::Color bottom) {
    for (int led = 0; led < (LedConstants::kNumLeds / 2); ++led) {
        m_buffer[led].SetRGB(bottom.red, bottom.grn, bottom.blu);
    }

    for (int led = (LedConstants::kNumLeds / 2); led < (LedConstants::kNumLeds); ++led) {
        m_buffer[led].SetRGB(top.red, top.grn, top.blu);
    }

    m_led.SetData(m_buffer);
}