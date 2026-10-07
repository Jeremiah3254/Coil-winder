// Protocol.cpp — USB serial, one JSON object per line. Commands are listed in the README.
#include "Protocol.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>
#include "Machine.h"
#include "config.h"

// ---- Module tunables ----
constexpr size_t PROTO_LINE_MAX = 160;

namespace {
Machine* g_m = nullptr;
char g_line[PROTO_LINE_MAX];
size_t g_len = 0;
bool g_overflow = false;
uint32_t g_lastTelMs = 0;
MachineState g_lastState = MachineState::Idle;
FaultCode g_lastFault = FaultCode::None;
bool g_sentState = false;

float r3(float v) { return roundf(v * 1000.0f) / 1000.0f; }
void send(JsonDocument& d) { serializeJson(d, Serial); Serial.println(); }

void ack(const char* cmd, bool ok, const char* msg) {
  JsonDocument d;
  d["type"] = "ack"; d["cmd"] = cmd; d["ok"] = ok; d["msg"] = msg;
  send(d);
}

void sendRecipe() {
  const Recipe& r = g_m->recipe();
  JsonDocument d;
  d["type"] = "recipe"; d["turns"] = r.turns; d["pitch"] = r3(r.pitchMm); d["width"] = r3(r.widthMm);
  d["offset"] = r3(r.offsetMm); d["rpm"] = r3(r.rpm); d["fw"] = FW_VERSION;
  d["max_travel"] = TRAVERSE_MAX_TRAVEL_MM; d["min_offset"] = MIN_FIRST_TURN_OFFSET_MM;
  d["rpm_min"] = SPINDLE_RPM_MIN; d["rpm_max"] = SPINDLE_RPM_MAX;
  send(d);
}

void sendTelemetry() {
  JsonDocument d;
  d["type"] = "tel"; d["ms"] = millis();
  d["state"] = Machine::stateName(g_m->state()); d["fault"] = Machine::faultName(g_m->fault());
  d["turns"] = r3(g_m->turnsDone()); d["target"] = g_m->recipe().turns;
  d["layer"] = g_m->layerNumber(); d["layers"] = g_m->layerTotal();
  d["x"] = r3(g_m->traverseMm()); d["rpm"] = r3(g_m->rpm()); d["homed"] = g_m->homed();
  send(d);
}

void sendStateIfChanged() {
  const MachineState s = g_m->state();
  const FaultCode f = g_m->fault();
  if (g_sentState && s == g_lastState && f == g_lastFault) return;
  g_sentState = true; g_lastState = s; g_lastFault = f;
  JsonDocument d;
  d["type"] = "state"; d["state"] = Machine::stateName(s); d["fault"] = Machine::faultName(f); d["msg"] = g_m->message();
  send(d);
}

void sendSummaryIfReady() {
  RunSummary s;
  if (!g_m->takeSummary(s)) return;
  JsonDocument d;
  d["type"] = "done"; d["completed"] = s.completed; d["target"] = s.targetTurns;
  d["turns"] = r3(s.turnsWound); d["layers"] = s.layers; d["dur_ms"] = s.durationMs;
  send(d);
}

template <typename T>
void readField(JsonDocument& d, const char* key, T& out) {
  JsonVariant v = d[key];
  if (!v.isNull()) out = v.as<T>();
}

bool applyRecipe(JsonDocument& d) {
  if (!g_m->recipeEditable()) return false;
  Recipe& r = g_m->recipe();
  readField(d, "turns", r.turns); readField(d, "pitch", r.pitchMm); readField(d, "width", r.widthMm);
  readField(d, "offset", r.offsetMm); readField(d, "rpm", r.rpm);
  return true;
}

void handleLine(char* line) {
  JsonDocument d;
  if (deserializeJson(d, line)) { ack("?", false, "bad JSON"); return; }
  const char* cmd = d["cmd"] | "";
  bool ok;
  if (!strcmp(cmd, "ping")) ok = true;
  else if (!strcmp(cmd, "get")) { sendRecipe(); ok = true; }
  else if (!strcmp(cmd, "set")) { ok = applyRecipe(d); if (ok) sendRecipe(); }
  else if (!strcmp(cmd, "start")) ok = applyRecipe(d) && g_m->start();
  else if (!strcmp(cmd, "home")) ok = g_m->home();
  else if (!strcmp(cmd, "pause")) ok = g_m->pause();
  else if (!strcmp(cmd, "resume")) ok = g_m->resume();
  else if (!strcmp(cmd, "stop")) ok = g_m->stop();
  else if (!strcmp(cmd, "reset")) ok = g_m->reset();
  else if (!strcmp(cmd, "save")) ok = g_m->saveRecipe();
  else if (!strcmp(cmd, "jog")) ok = g_m->jogTraverse(d["mm"] | 0.0f);
  else { ack(cmd, false, "unknown command"); return; }
  ack(cmd, ok, g_m->message());
}
}  // namespace

namespace Protocol {
void begin(Machine* machine) {
  g_m = machine;
  JsonDocument d;
  d["type"] = "hello"; d["fw"] = FW_VERSION;
  send(d);
  sendRecipe();
}

void update() {
  if (g_m == nullptr) return;
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_line[g_len] = '\0';
      if (!g_overflow && g_len > 0) handleLine(g_line);
      g_len = 0; g_overflow = false;
    } else if (g_len < PROTO_LINE_MAX - 1) {
      g_line[g_len++] = c;
    } else {
      g_overflow = true;
    }
  }
  sendStateIfChanged();
  sendSummaryIfReady();
  const uint32_t now = millis();
  if (now - g_lastTelMs >= TELEMETRY_PERIOD_MS) { g_lastTelMs = now; sendTelemetry(); }
}
}  // namespace Protocol
