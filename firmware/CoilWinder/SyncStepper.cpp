// SyncStepper.cpp — see SyncStepper.h
#include "SyncStepper.h"
#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"

// ---- Module tunables ----
constexpr float    MIN_RATE_HZ    = 40.0f;   // slowest rate Timer1 can time with the /8 prescaler
constexpr float    STOP_MARGIN    = 1.15f;   // start decelerating a little early
constexpr uint32_t RAMP_UPDATE_MS = 4;       // how often the speed ramp is recalculated
#define STEP_PULSE_CYCLES 40                 // 2.5 us high time; DRV8825 needs >= 1.9 us

namespace {
struct Pin {
  volatile uint8_t* port;
  uint8_t mask;
};
Pin g_step[2], g_dir[2];
bool g_invert[2] = {SPINDLE_INVERT_DIR, TRAVERSE_INVERT_DIR};

volatile int32_t g_pos[2] = {0, 0};
volatile int8_t g_sign[2] = {1, 1};
volatile uint32_t g_major = 0, g_minor = 0, g_err = 0, g_remaining = 0;
volatile uint8_t g_master = 0;
volatile bool g_running = false;
volatile bool g_continuous = false;

float g_rate = 0.0f, g_target = 0.0f, g_accel = 1.0f;
bool g_stopReq = false;
bool g_enabled = false;
uint32_t g_lastRampMs = 0;

Pin makePin(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  return {portOutputRegister(digitalPinToPort(pin)), digitalPinToBitMask(pin)};
}

void setDir(uint8_t axis, bool forward) {
  const bool high = forward != g_invert[axis];
  if (high) *g_dir[axis].port |= g_dir[axis].mask;
  else *g_dir[axis].port &= ~g_dir[axis].mask;
  g_sign[axis] = forward ? 1 : -1;
}

void writeRate(float hz) {
  if (hz < MIN_RATE_HZ) hz = MIN_RATE_HZ;
  if (hz > MAX_STEP_RATE_HZ) hz = MAX_STEP_RATE_HZ;
  const uint16_t top = (uint16_t)(F_CPU / 8.0f / hz) - 1;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    OCR1A = top;
    if (TCNT1 > top) TCNT1 = 0;
  }
}

void startTimer() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCNT1 = 0;
    TIFR1 = _BV(OCF1A);
    TIMSK1 |= _BV(OCIE1A);
  }
}
}  // namespace

ISR(TIMER1_COMPA_vect) {
  if (!g_running) return;
  const uint8_t m = g_master;
  const uint8_t n = 1 - m;
  bool minorStep = false;
  if (g_minor != 0) {
    g_err += g_minor;
    if (g_err >= g_major) {
      g_err -= g_major;
      minorStep = true;
    }
  }
  *g_step[m].port |= g_step[m].mask;
  if (minorStep) *g_step[n].port |= g_step[n].mask;
  __builtin_avr_delay_cycles(STEP_PULSE_CYCLES);
  *g_step[m].port &= ~g_step[m].mask;
  if (minorStep) *g_step[n].port &= ~g_step[n].mask;
  g_pos[m] = g_pos[m] + g_sign[m];
  if (minorStep) g_pos[n] = g_pos[n] + g_sign[n];
  if (!g_continuous) {
    g_remaining = g_remaining - 1;
    if (g_remaining == 0) {
      g_running = false;
      TIMSK1 &= ~_BV(OCIE1A);
    }
  }
}

namespace SyncStepper {
void begin() {
  g_step[SPINDLE] = makePin(PIN_SPINDLE_STEP);
  g_dir[SPINDLE] = makePin(PIN_SPINDLE_DIR);
  g_step[TRAVERSE] = makePin(PIN_TRAVERSE_STEP);
  g_dir[TRAVERSE] = makePin(PIN_TRAVERSE_DIR);
  pinMode(PIN_DRIVERS_EN, OUTPUT);
  digitalWrite(PIN_DRIVERS_EN, HIGH);  // disabled
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11);      // CTC mode, clk/8 = 2 MHz
  writeRate(MIN_RATE_HZ);
}

void enable(bool on) {
  digitalWrite(PIN_DRIVERS_EN, on ? LOW : HIGH);
  if (on && !g_enabled) delay(DRIVER_ENABLE_SETTLE_MS);
  g_enabled = on;
}

bool enabled() { return g_enabled; }

bool startMove(int32_t dSp, int32_t dTr, float rateHz, float accelHz2) {
  if (g_running || (dSp == 0 && dTr == 0)) return false;
  const uint32_t a = dSp >= 0 ? (uint32_t)dSp : (uint32_t)(-dSp);
  const uint32_t b = dTr >= 0 ? (uint32_t)dTr : (uint32_t)(-dTr);
  setDir(SPINDLE, dSp >= 0);
  setDir(TRAVERSE, dTr >= 0);
  const uint32_t major = a >= b ? a : b;
  if (a > 0) {  // rate was given for the spindle; convert to the master axis
    const float scale = (float)major / (float)a;
    rateHz *= scale;
    accelHz2 *= scale;
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_master = a >= b ? SPINDLE : TRAVERSE;
    g_major = major;
    g_minor = a >= b ? b : a;
    g_err = major / 2;
    g_remaining = major;
    g_continuous = false;
  }
  g_target = rateHz > MAX_STEP_RATE_HZ ? MAX_STEP_RATE_HZ : rateHz;
  g_accel = accelHz2 > 1.0f ? accelHz2 : 1.0f;
  g_rate = MIN_RATE_HZ;
  g_stopReq = false;
  writeRate(g_rate);
  g_running = true;
  g_lastRampMs = millis();
  startTimer();
  return true;
}

bool startContinuous(uint8_t axis, int8_t dir, float rateHz, float accelHz2) {
  if (g_running) return false;
  setDir(axis, dir > 0);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_master = axis;
    g_major = 1;
    g_minor = 0;
    g_err = 0;
    g_remaining = 1;
    g_continuous = true;
  }
  g_target = rateHz;
  g_accel = accelHz2 > 1.0f ? accelHz2 : 1.0f;
  g_rate = MIN_RATE_HZ;
  g_stopReq = false;
  writeRate(g_rate);
  g_running = true;
  g_lastRampMs = millis();
  startTimer();
  return true;
}

void requestStop() { g_stopReq = true; }

void hardStop() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_running = false;
    TIMSK1 &= ~_BV(OCIE1A);
  }
  g_rate = 0.0f;
  g_stopReq = false;
}

void update() {
  if (!g_running) return;
  const uint32_t now = millis();
  if (now - g_lastRampMs < RAMP_UPDATE_MS) return;
  const float dt = (now - g_lastRampMs) / 1000.0f;
  g_lastRampMs = now;
  uint32_t remaining;
  bool continuous;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    remaining = g_remaining;
    continuous = g_continuous;
  }
  const float stopSteps = STOP_MARGIN * g_rate * g_rate / (2.0f * g_accel) + 1.0f;
  const bool decel = g_stopReq || (!continuous && (float)remaining <= stopSteps);
  if (decel) {
    g_rate -= g_accel * dt;
    if (g_rate <= MIN_RATE_HZ) {
      g_rate = MIN_RATE_HZ;
      if (g_stopReq) {
        hardStop();
        return;
      }
    }
  } else if (g_rate < g_target) {
    g_rate += g_accel * dt;
    if (g_rate > g_target) g_rate = g_target;
  } else if (g_rate > g_target) {
    g_rate -= g_accel * dt;
    if (g_rate < g_target) g_rate = g_target;
  }
  writeRate(g_rate);
}

bool busy() { return g_running; }

int32_t position(uint8_t axis) {
  int32_t p;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { p = g_pos[axis]; }
  return p;
}

void setPosition(uint8_t axis, int32_t steps) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_pos[axis] = steps; }
}

float spindleStepHz() {
  if (!g_running) return 0.0f;
  uint8_t m;
  uint32_t major, minor;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    m = g_master;
    major = g_major;
    minor = g_minor;
  }
  if (m == SPINDLE) return g_rate;
  return major ? g_rate * (float)minor / (float)major : 0.0f;
}
}  // namespace SyncStepper
