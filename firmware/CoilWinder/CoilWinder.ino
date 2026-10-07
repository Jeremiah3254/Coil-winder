// CoilWinder.ino — Automated Coil Winder, Arduino Mega 2560 + 2x DRV8825.
// University of Toledo senior design. Upload with the Arduino IDE (board: Arduino Mega or Mega 2560).
#include <Arduino.h>
#include "Hmi.h"
#include "Machine.h"
#include "Protocol.h"
#include "config.h"

Machine g_machine;

static void updateStatusLed() {
  const uint32_t t = millis();
  bool on;
  switch (g_machine.state()) {
    case MachineState::Winding:
    case MachineState::Positioning: on = true; break;
    case MachineState::Fault:
    case MachineState::EStop: on = (t / 100) % 2; break;
    case MachineState::Paused:
    case MachineState::Pausing: on = (t / 500) % 2; break;
    default: on = (t % 1000) < 80; break;
  }
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_STATUS_LED, OUTPUT);
  g_machine.begin();
  Hmi::begin(&g_machine);
  Protocol::begin(&g_machine);
}

void loop() {
  g_machine.update();
  Protocol::update();
  Hmi::update();
  updateStatusLed();
}
