// config.h — Automated Coil Winder (Arduino Mega 2560 + 2x DRV8825).
// Every pin and tunable value is here. Check the values marked ASSUMED against your real parts.
#pragma once
#include <Arduino.h>

constexpr const char* FW_VERSION = "1.0.1";

// ============================ SIMULATION SWITCH ============================
// true  = testing in Wokwi (the limit switches are push buttons, which read the opposite way)
// false = the real machine (limit switches wired C -> GND, NC -> pin)
// This is the only setting to change when going back and forth.
constexpr bool SIMULATION = true;

// ============================ OPTIONAL HARDWARE ============================
constexpr bool HAS_END_SWITCH  = true;  // limit switch at the far end of the stage
constexpr bool HAS_ESTOP_SENSE = true;  // E-stop second (NO) contact wired to PIN_ESTOP_SENSE
constexpr bool HAS_DISPLAY     = true;  // Nokia 5110 + KY-040 knob. false = control from the app only

// ============================ PINS ============================
constexpr uint8_t PIN_SPINDLE_STEP  = 22;
constexpr uint8_t PIN_SPINDLE_DIR   = 23;
constexpr uint8_t PIN_DRIVERS_EN    = 24;  // both DRV8825 EN pins. LOW = enabled. 10k pull-up to 5 V.
constexpr uint8_t PIN_TRAVERSE_STEP = 25;
constexpr uint8_t PIN_TRAVERSE_DIR  = 26;
constexpr uint8_t PIN_LIMIT_HOME    = 2;   // switch C -> GND, NC -> pin
constexpr uint8_t PIN_LIMIT_END     = 3;   // switch C -> GND, NC -> pin
constexpr uint8_t PIN_ESTOP_SENSE   = 18;  // E-stop NO contact -> GND
constexpr uint8_t PIN_ENC_CLK       = 30;
constexpr uint8_t PIN_ENC_DT        = 31;
constexpr uint8_t PIN_ENC_SW        = 32;
constexpr uint8_t PIN_LCD_CE        = 53;  // Nokia 5110 on hardware SPI: DIN = 51, CLK = 52
constexpr uint8_t PIN_LCD_DC        = 49;
constexpr uint8_t PIN_LCD_RST       = 48;
constexpr uint8_t PIN_LCD_LIGHT     = 46;
constexpr uint8_t PIN_STATUS_LED    = 13;  // on-board LED

// Real NC switch: opens (reads HIGH) when pressed or if a wire breaks. Wokwi button: reads LOW when pressed.
constexpr int LIMIT_TRIGGERED_LEVEL = SIMULATION ? LOW : HIGH;
constexpr int ESTOP_ACTIVE_LEVEL    = LOW;

// ============================ MECHANICS ============================
constexpr int32_t MOTOR_FULL_STEPS       = 200;   // 1.8-degree NEMA 17
constexpr int32_t SPINDLE_MICROSTEPS     = 8;     // DRV8825 M0 = 5V, M1 = 5V, M2 = GND
constexpr int32_t TRAVERSE_MICROSTEPS    = 8;
constexpr int32_t SPINDLE_STEPS_PER_REV  = MOTOR_FULL_STEPS * SPINDLE_MICROSTEPS;                    // 1600
constexpr float   LEADSCREW_LEAD_MM      = 5.0f;  // ASSUMED: SFU1605 ball screw moves 5 mm per turn
constexpr float   TRAVERSE_STEPS_PER_MM  = (MOTOR_FULL_STEPS * TRAVERSE_MICROSTEPS) / LEADSCREW_LEAD_MM; // 320
constexpr float   TRAVERSE_MAX_TRAVEL_MM = 140.0f; // ASSUMED: 150 mm stage stroke minus a safety margin
constexpr bool    SPINDLE_INVERT_DIR     = false;  // flip if the bobbin turns the wrong way
constexpr bool    TRAVERSE_INVERT_DIR    = false;  // flip if homing moves away from the home switch

// ============================ MOTION ============================
constexpr float    SPINDLE_RPM_MIN          = 10.0f;
constexpr float    SPINDLE_RPM_MAX          = 300.0f;
constexpr float    SPINDLE_ACCEL_RPM_PER_S  = 100.0f;
constexpr float    TRAVERSE_MOVE_SPEED_MM_S = 10.0f;  // jogs and the move to the first turn
constexpr float    TRAVERSE_ACCEL_MM_S2     = 40.0f;
constexpr float    HOMING_SPEED_MM_S        = 4.0f;
constexpr float    HOMING_BACKOFF_MM        = 2.0f;
constexpr float    HOMING_MAX_SEEK_MM       = TRAVERSE_MAX_TRAVEL_MM + 20.0f;
constexpr float    MIN_FIRST_TURN_OFFSET_MM = HOMING_BACKOFF_MM + 0.5f;
constexpr float    MAX_STEP_RATE_HZ         = 12000.0f;
constexpr uint32_t DRIVER_ENABLE_SETTLE_MS  = 5;

// ============================ RECIPE DEFAULTS ============================
constexpr int32_t DEFAULT_TURNS     = 200;
constexpr float   DEFAULT_PITCH_MM  = 0.43f;  // set to your wire's outside diameter
constexpr float   DEFAULT_WIDTH_MM  = 20.0f;  // bobbin winding width
constexpr float   DEFAULT_OFFSET_MM = 10.0f;  // guide position of the first turn, from the home switch
constexpr float   DEFAULT_RPM       = 120.0f;

// ============================ DISPLAY / KNOB / USB ============================
constexpr uint8_t  LCD_CONTRAST               = 140;
constexpr uint8_t  LCD_BACKLIGHT_PWM          = 120;
constexpr uint32_t LCD_REFRESH_MS             = 200;
constexpr uint8_t  ENC_TRANSITIONS_PER_DETENT = 4;
constexpr bool     ENC_REVERSE                = false;
constexpr uint32_t BUTTON_DEBOUNCE_MS         = 30;
constexpr uint32_t LONG_PRESS_MS              = 800;
constexpr uint32_t SERIAL_BAUD                = 115200;
constexpr uint32_t TELEMETRY_PERIOD_MS        = 250;