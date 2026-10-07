// Hmi.cpp — display + knob menu. Status screen: click = menu (idle/ready/done), pause (winding),
// resume (paused); hold = stop (running) or clear fault. Menu: turn to move, click to select or edit,
// hold to go back.
// Display: the real machine uses the Nokia 5110. Wokwi for VS Code has no Nokia 5110 part, so when
// SIMULATION is true the same screens are drawn on an SSD1306 OLED (I2C: SDA = pin 20, SCL = pin 21).
#include "Hmi.h"
#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include "Machine.h"
#include "config.h"

// ---- Module tunables ----
constexpr int32_t STEP_TURNS     = 10;
constexpr float   STEP_PITCH_MM  = 0.01f;
constexpr float   STEP_WIDTH_MM  = 0.5f;
constexpr float   STEP_OFFSET_MM = 0.5f;
constexpr float   STEP_RPM       = 10.0f;
constexpr float   STEP_JOG_MM    = 0.5f;
constexpr uint8_t MENU_ROWS      = 5;
constexpr uint8_t LINE_H         = 8;
constexpr uint8_t SIM_OLED_I2C_ADDR = 0x3C;   // Wokwi SSD1306 default address
constexpr uint8_t SCREEN_W       = 84;        // screens are laid out for the Nokia's 84 x 48 pixels

namespace {
U8G2_PCD8544_84X48_F_4W_HW_SPI g_nokia(U8G2_R0, PIN_LCD_CE, PIN_LCD_DC, PIN_LCD_RST);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C g_oled(U8G2_R0, U8X8_PIN_NONE);
U8G2& g_lcd = SIMULATION ? static_cast<U8G2&>(g_oled) : static_cast<U8G2&>(g_nokia);
Machine* g_m = nullptr;

// ---------------- inputs, sampled by Timer2 at 1 kHz ----------------
volatile int16_t g_detents = 0;
volatile bool g_click = false, g_hold = false;
uint8_t g_encPrev = 0;
int8_t g_encAcc = 0;
bool g_btnRaw = false, g_btnStable = false, g_holdFired = false;
uint16_t g_btnChangeMs = 0, g_btnDownMs = 0, g_ms = 0;
const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

enum class Screen : uint8_t { Status, Menu, Edit, Jog };
enum Item : uint8_t { I_HOME, I_START, I_TURNS, I_PITCH, I_WIDTH, I_OFFSET, I_RPM, I_JOG, I_SAVE, I_BACK, I_COUNT };
const char* const LABELS[I_COUNT] = {"Home axis", "Start wind", "Turns", "Pitch mm", "Width mm",
                                     "Offset mm", "RPM", "Jog guide", "Save recipe", "Back"};
Screen g_screen = Screen::Status;
uint8_t g_cursor = 0, g_top = 0;
uint32_t g_lastDrawMs = 0;
bool g_dirty = true;
}  // namespace

ISR(TIMER2_COMPA_vect) {
  g_ms++;
  const uint8_t cur = (uint8_t)((digitalRead(PIN_ENC_CLK) << 1) | digitalRead(PIN_ENC_DT));
  g_encAcc += QDEC[(g_encPrev << 2) | cur];
  g_encPrev = cur;
  if (cur == 0b11 || (ENC_TRANSITIONS_PER_DETENT == 2 && cur == 0b00)) {
    const int8_t th = ENC_TRANSITIONS_PER_DETENT / 2;
    if (g_encAcc >= th) g_detents = g_detents + (ENC_REVERSE ? -1 : 1);
    else if (g_encAcc <= -th) g_detents = g_detents + (ENC_REVERSE ? 1 : -1);
    g_encAcc = 0;
  }
  const bool pressed = digitalRead(PIN_ENC_SW) == LOW;
  if (pressed != g_btnRaw) { g_btnRaw = pressed; g_btnChangeMs = g_ms; }
  if (pressed != g_btnStable && (uint16_t)(g_ms - g_btnChangeMs) >= BUTTON_DEBOUNCE_MS) {
    g_btnStable = pressed;
    if (pressed) { g_btnDownMs = g_ms; g_holdFired = false; }
    else if (!g_holdFired) g_click = true;
  }
  if (g_btnStable && !g_holdFired && (uint16_t)(g_ms - g_btnDownMs) >= LONG_PRESS_MS) { g_holdFired = true; g_hold = true; }
}

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float snap(float v, float step) { return roundf(v / step) * step; }
bool isValue(uint8_t i) { return i >= I_TURNS && i <= I_RPM; }

void valueText(uint8_t i, char* buf, size_t n) {
  const Recipe& r = g_m->recipe();
  switch (i) {
    case I_TURNS: snprintf(buf, n, "%ld", (long)r.turns); break;
    case I_PITCH: dtostrf(r.pitchMm, 1, 2, buf); break;
    case I_WIDTH: dtostrf(r.widthMm, 1, 1, buf); break;
    case I_OFFSET: dtostrf(r.offsetMm, 1, 1, buf); break;
    case I_RPM: dtostrf(r.rpm, 1, 0, buf); break;
    default: buf[0] = '\0';
  }
}

void adjust(uint8_t i, int16_t d) {
  Recipe& r = g_m->recipe();
  const RecipeLimits L = g_m->limits();
  switch (i) {
    case I_TURNS: {
      const int32_t t = r.turns + (int32_t)d * STEP_TURNS;
      r.turns = t < 1 ? 1 : (t > WINDPLAN_MAX_TURNS ? WINDPLAN_MAX_TURNS : t);
      break;
    }
    case I_PITCH: r.pitchMm = clampf(snap(r.pitchMm + d * STEP_PITCH_MM, STEP_PITCH_MM), WINDPLAN_MIN_PITCH_MM, WINDPLAN_MAX_PITCH_MM); break;
    case I_WIDTH: r.widthMm = clampf(snap(r.widthMm + d * STEP_WIDTH_MM, STEP_WIDTH_MM), r.pitchMm, L.maxTravelMm - r.offsetMm); break;
    case I_OFFSET: r.offsetMm = clampf(snap(r.offsetMm + d * STEP_OFFSET_MM, STEP_OFFSET_MM), L.minOffsetMm, L.maxTravelMm - r.widthMm); break;
    case I_RPM: r.rpm = clampf(r.rpm + d * STEP_RPM, L.rpmMin, L.rpmMax); break;
    default: break;
  }
}

void activate(uint8_t i) {
  switch (i) {
    case I_HOME: g_m->home(); g_screen = Screen::Status; break;
    case I_START: g_m->start(); g_screen = Screen::Status; break;
    case I_JOG: g_screen = Screen::Jog; break;
    case I_SAVE: g_m->saveRecipe(); g_screen = Screen::Status; break;
    case I_BACK: g_screen = Screen::Status; break;
    default: if (isValue(i) && g_m->recipeEditable()) g_screen = Screen::Edit; break;
  }
}

void handleInputs() {
  int16_t d;
  bool click, hold;
  noInterrupts();
  d = g_detents; g_detents = 0;
  click = g_click; g_click = false;
  hold = g_hold; g_hold = false;
  interrupts();
  if (d == 0 && !click && !hold) return;
  g_dirty = true;
  const MachineState s = g_m->state();
  switch (g_screen) {
    case Screen::Status:
      if (click) {
        if (s == MachineState::Winding || s == MachineState::Positioning) g_m->pause();
        else if (s == MachineState::Paused) g_m->resume();
        else if (s == MachineState::Idle || s == MachineState::Ready || s == MachineState::Done) {
          g_screen = Screen::Menu; g_cursor = 0; g_top = 0;
        }
      }
      if (hold) {
        if (s == MachineState::Fault || s == MachineState::EStop) g_m->reset();
        else g_m->stop();
      }
      break;
    case Screen::Menu:
      if (d != 0) {
        const int16_t c = (int16_t)g_cursor + d;
        g_cursor = (uint8_t)(c < 0 ? 0 : (c >= I_COUNT ? I_COUNT - 1 : c));
        if (g_cursor < g_top) g_top = g_cursor;
        if (g_cursor >= g_top + MENU_ROWS) g_top = g_cursor - MENU_ROWS + 1;
      }
      if (click) activate(g_cursor);
      if (hold) g_screen = Screen::Status;
      break;
    case Screen::Edit:
      if (d != 0) adjust(g_cursor, d);
      if (click || hold) g_screen = Screen::Menu;
      break;
    case Screen::Jog:
      if (d != 0) g_m->jogTraverse(d * STEP_JOG_MM);
      if (click || hold) g_screen = Screen::Menu;
      break;
  }
}

void drawStatus() {
  char a[12], b[12], line[24];
  g_lcd.drawStr(0, 7, Machine::stateName(g_m->state()));
  if (!g_m->homed()) g_lcd.drawStr(66, 7, "NH");
  dtostrf(g_m->turnsDone(), 1, 1, a);
  snprintf(line, sizeof line, "T %s/%ld", a, (long)g_m->recipe().turns);
  g_lcd.drawStr(0, 15, line);
  snprintf(line, sizeof line, "Layer %ld/%ld", (long)g_m->layerNumber(), (long)g_m->layerTotal());
  g_lcd.drawStr(0, 23, line);
  dtostrf(g_m->traverseMm(), 1, 2, a);
  dtostrf(g_m->rpm(), 1, 0, b);
  snprintf(line, sizeof line, "X%s R%s", a, b);
  g_lcd.drawStr(0, 31, line);
  const char* msg = g_m->message();
  strncpy(line, msg, 16); line[16] = '\0';
  g_lcd.drawStr(0, 39, line);
  if (strlen(msg) > 16) { strncpy(line, msg + 16, 16); line[16] = '\0'; g_lcd.drawStr(0, 47, line); }
}

void drawMenu() {
  char val[10], line[24];
  if (g_screen == Screen::Jog) {
    g_lcd.drawStr(0, 7, "Jog: turn knob");
    dtostrf(g_m->traverseMm(), 1, 2, val);
    snprintf(line, sizeof line, "X = %s mm", val);
    g_lcd.drawStr(0, 23, line);
    g_lcd.drawStr(0, 39, "click = done");
    return;
  }
  g_lcd.drawStr(0, 7, g_screen == Screen::Edit ? "Edit value" : "Menu");
  for (uint8_t row = 0; row < MENU_ROWS && g_top + row < I_COUNT; row++) {
    const uint8_t i = g_top + row;
    const int y = 15 + row * LINE_H;
    if (i == g_cursor) { g_lcd.drawBox(0, y - 7, SCREEN_W, LINE_H); g_lcd.setDrawColor(0); }
    g_lcd.drawStr(1, y, LABELS[i]);
    valueText(i, val, sizeof val);
    if (val[0]) g_lcd.drawStr(SCREEN_W - 1 - g_lcd.getStrWidth(val), y, val);
    g_lcd.setDrawColor(1);
  }
}
}  // namespace

namespace Hmi {
void begin(Machine* machine) {
  g_m = machine;
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  g_encPrev = (uint8_t)((digitalRead(PIN_ENC_CLK) << 1) | digitalRead(PIN_ENC_DT));
  if (HAS_DISPLAY) {
    pinMode(PIN_LCD_LIGHT, OUTPUT);
    analogWrite(PIN_LCD_LIGHT, LCD_BACKLIGHT_PWM);
    if (SIMULATION) g_oled.setI2CAddress(SIM_OLED_I2C_ADDR * 2);  // U8g2 wants the 8-bit address
    g_lcd.begin();
    if (!SIMULATION) g_lcd.setContrast(LCD_CONTRAST);
    g_lcd.setFont(u8g2_font_5x8_tf);
  }
  // Timer2: CTC, clk/64, 1 kHz input sampling
  noInterrupts();
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22);
  OCR2A = 249;
  TIMSK2 = _BV(OCIE2A);
  interrupts();
}

void update() {
  if (g_m == nullptr) return;
  handleInputs();
  if (!HAS_DISPLAY) return;
  const uint32_t now = millis();
  if (!g_dirty && now - g_lastDrawMs < LCD_REFRESH_MS) return;
  g_lcd.clearBuffer();
  if (g_screen == Screen::Status) drawStatus(); else drawMenu();
  g_lcd.sendBuffer();
  g_lastDrawMs = now;
  g_dirty = false;
}
}  // namespace Hmi