// Machine.cpp — see Machine.h
#include "Machine.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include "SyncStepper.h"
#include "config.h"

// ---- Module tunables ----
constexpr uint16_t RECIPE_MAGIC        = 0xC0E2;  // change if the Recipe struct changes
constexpr int      EEPROM_ADDR         = 0;
constexpr uint8_t  ESTOP_CONFIRM_READS = 2;
constexpr float    HOMING_CLEAR_MM     = 5.0f;

namespace {
int32_t mmToSteps(float mm) { return (int32_t)lroundf(mm * TRAVERSE_STEPS_PER_MM); }
float spindleHz(float rpm) { return rpm / 60.0f * SPINDLE_STEPS_PER_REV; }
float traverseHz(float mmS) { return mmS * TRAVERSE_STEPS_PER_MM; }
}  // namespace

void Machine::begin() {
  pinMode(PIN_LIMIT_HOME, INPUT_PULLUP);
  pinMode(PIN_LIMIT_END, INPUT_PULLUP);
  pinMode(PIN_ESTOP_SENSE, INPUT_PULLUP);
  SyncStepper::begin();
  loadRecipe();
  state_ = MachineState::Idle;
  setMessage("Press: Home axis");
}

void Machine::setMessage(const char* m) {
  strncpy(message_, m, sizeof(message_) - 1);
  message_[sizeof(message_) - 1] = '\0';
}

RecipeLimits Machine::limits() const {
  RecipeLimits l;
  l.maxTravelMm = TRAVERSE_MAX_TRAVEL_MM;
  l.minOffsetMm = MIN_FIRST_TURN_OFFSET_MM;
  l.rpmMin = SPINDLE_RPM_MIN;
  l.rpmMax = SPINDLE_RPM_MAX;
  return l;
}

bool Machine::homeSwitch() const { return digitalRead(PIN_LIMIT_HOME) == LIMIT_TRIGGERED_LEVEL; }
bool Machine::endSwitch() const { return HAS_END_SWITCH && digitalRead(PIN_LIMIT_END) == LIMIT_TRIGGERED_LEVEL; }

bool Machine::estopActive() {
  if (!HAS_ESTOP_SENSE) return false;
  if (digitalRead(PIN_ESTOP_SENSE) == ESTOP_ACTIVE_LEVEL) {
    if (estopCount_ < ESTOP_CONFIRM_READS) estopCount_++;
  } else {
    estopCount_ = 0;
  }
  return estopCount_ >= ESTOP_CONFIRM_READS;
}

void Machine::hardFault(FaultCode f, const char* msg) {
  SyncStepper::hardStop();
  SyncStepper::enable(false);
  homed_ = false;
  if (runActive_) finishRun(false);
  fault_ = f;
  setMessage(msg);
  state_ = MachineState::Fault;
}

void Machine::checkLimits() {
  if (endSwitch()) hardFault(FaultCode::LimitHit, "END limit hit");
  else if (homeSwitch()) hardFault(FaultCode::LimitHit, "HOME limit hit");
}

void Machine::update() {
  SyncStepper::update();

  if (estopActive()) {
    if (state_ != MachineState::EStop) {
      SyncStepper::hardStop();
      SyncStepper::enable(false);
      homed_ = false;
      if (runActive_) finishRun(false);
      fault_ = FaultCode::EStop;
      setMessage("E-stop pressed");
      state_ = MachineState::EStop;
    }
    return;
  }

  switch (state_) {
    case MachineState::Homing:
      updateHoming();
      break;
    case MachineState::Ready:
    case MachineState::Done:
      if (SyncStepper::busy()) checkLimits();  // jogging
      break;
    case MachineState::Positioning:
      checkLimits();
      if (state_ == MachineState::Positioning && !SyncStepper::busy()) {
        if (startLayer(0)) state_ = MachineState::Winding;
        else hardFault(FaultCode::LimitHit, "Could not start");
      }
      break;
    case MachineState::Winding:
      checkLimits();
      if (state_ != MachineState::Winding || SyncStepper::busy()) break;
      if (SyncStepper::position(SyncStepper::SPINDLE) == spindleTarget_ &&
          SyncStepper::position(SyncStepper::TRAVERSE) == traverseTarget_) {
        if (layerIndex_ + 1 < plan_.layerCount()) {
          startLayer(layerIndex_ + 1);
        } else {
          finishRun(true);
          state_ = MachineState::Done;
          setMessage("Coil complete");
        }
      } else {
        continueLayer();
      }
      break;
    case MachineState::Pausing:
      checkLimits();
      if (state_ == MachineState::Pausing && !SyncStepper::busy()) {
        state_ = MachineState::Paused;
        setMessage("Paused");
      }
      break;
    case MachineState::Stopping:
      if (!SyncStepper::busy()) {
        if (runActive_) finishRun(false);
        state_ = homed_ ? MachineState::Ready : MachineState::Idle;
        setMessage(homed_ ? "Stopped" : "Stopped. Home again");
      }
      break;
    default:
      break;
  }
}

void Machine::updateHoming() {
  const float hz = traverseHz(HOMING_SPEED_MM_S);
  const float acc = traverseHz(TRAVERSE_ACCEL_MM_S2);
  switch (homePhase_) {
    case HomePhase::Clear:
      if (!SyncStepper::busy()) {
        if (homeSwitch()) {
          hardFault(FaultCode::HomeFailed, "Home switch stuck");
        } else {
          homePhase_ = HomePhase::Seek;
          SyncStepper::setPosition(SyncStepper::TRAVERSE, 0);
          SyncStepper::startContinuous(SyncStepper::TRAVERSE, -1, hz, acc);
        }
      }
      break;
    case HomePhase::Seek:
      if (homeSwitch()) {
        SyncStepper::hardStop();
        SyncStepper::setPosition(SyncStepper::TRAVERSE, 0);  // trip point is 0 mm
        homePhase_ = HomePhase::Backoff;
        SyncStepper::startMove(0, mmToSteps(HOMING_BACKOFF_MM), hz, acc);
      } else if (endSwitch()) {
        hardFault(FaultCode::HomeFailed, "Wrong way: flip TRAVERSE_INVERT_DIR");
      } else if (SyncStepper::position(SyncStepper::TRAVERSE) < -mmToSteps(HOMING_MAX_SEEK_MM)) {
        hardFault(FaultCode::HomeFailed, "Home switch not found");
      }
      break;
    case HomePhase::Backoff:
      if (!SyncStepper::busy()) {
        if (homeSwitch()) {
          hardFault(FaultCode::HomeFailed, "Switch did not release");
        } else {
          homed_ = true;
          state_ = MachineState::Ready;
          setMessage("Homed. Ready");
        }
      }
      break;
  }
}

bool Machine::moveTraverseTo(float mm) {
  if (mm < HOMING_BACKOFF_MM) mm = HOMING_BACKOFF_MM;
  if (mm > TRAVERSE_MAX_TRAVEL_MM) mm = TRAVERSE_MAX_TRAVEL_MM;
  const int32_t d = mmToSteps(mm) - SyncStepper::position(SyncStepper::TRAVERSE);
  if (d == 0) return true;
  return SyncStepper::startMove(0, d, traverseHz(TRAVERSE_MOVE_SPEED_MM_S), traverseHz(TRAVERSE_ACCEL_MM_S2));
}

bool Machine::startLayer(int32_t index) {
  LayerMove lm;
  if (!plan_.layer(index, lm)) return false;
  layerIndex_ = index;
  spindleTarget_ = (plan_.turnsBeforeLayer(index) + lm.turns) * SPINDLE_STEPS_PER_REV;
  traverseTarget_ = lm.traverseEndSteps;
  return continueLayer();
}

bool Machine::continueLayer() {
  const int32_t dSp = spindleTarget_ - SyncStepper::position(SyncStepper::SPINDLE);
  const int32_t dTr = traverseTarget_ - SyncStepper::position(SyncStepper::TRAVERSE);
  return SyncStepper::startMove(dSp, dTr, spindleHz(recipe_.rpm), spindleHz(SPINDLE_ACCEL_RPM_PER_S));
}

void Machine::finishRun(bool completed) {
  summary_.completed = completed;
  summary_.targetTurns = recipe_.turns;
  summary_.turnsWound = turnsDone();
  summary_.layers = layerIndex_ + 1;
  summary_.durationMs = millis() - runStartMs_;
  summaryReady_ = true;
  runActive_ = false;
}

bool Machine::takeSummary(RunSummary& out) {
  if (!summaryReady_) return false;
  out = summary_;
  summaryReady_ = false;
  return true;
}

// ------------------------------- commands -------------------------------
bool Machine::home() {
  if (!(state_ == MachineState::Idle || state_ == MachineState::Ready || state_ == MachineState::Done)) {
    setMessage("Can't home now");
    return false;
  }
  fault_ = FaultCode::None;
  homed_ = false;
  SyncStepper::enable(true);
  SyncStepper::setPosition(SyncStepper::TRAVERSE, 0);
  const float hz = traverseHz(HOMING_SPEED_MM_S), acc = traverseHz(TRAVERSE_ACCEL_MM_S2);
  if (homeSwitch()) {
    homePhase_ = HomePhase::Clear;
    SyncStepper::startMove(0, mmToSteps(HOMING_CLEAR_MM), hz, acc);
  } else {
    homePhase_ = HomePhase::Seek;
    SyncStepper::startContinuous(SyncStepper::TRAVERSE, -1, hz, acc);
  }
  state_ = MachineState::Homing;
  setMessage("Homing...");
  return true;
}

bool Machine::start() {
  if (!homed_ || !(state_ == MachineState::Ready || state_ == MachineState::Done)) {
    setMessage("Home first");
    return false;
  }
  const RecipeError err = WindPlan::validate(recipe_, limits());
  if (err != RecipeError::Ok) {
    setMessage(WindPlan::errorText(err));
    return false;
  }
  plan_.begin(recipe_, TRAVERSE_STEPS_PER_MM);
  LayerMove first;
  plan_.layer(0, first);
  SyncStepper::setPosition(SyncStepper::SPINDLE, 0);
  layerIndex_ = 0;
  spindleTarget_ = 0;
  traverseTarget_ = first.traverseStartSteps;
  runActive_ = true;
  hasRun_ = true;
  runStartMs_ = millis();
  summaryReady_ = false;
  fault_ = FaultCode::None;
  moveTraverseTo(first.traverseStartMm);
  state_ = MachineState::Positioning;
  setMessage("Winding");
  return true;
}

bool Machine::pause() {
  if (!(state_ == MachineState::Winding || state_ == MachineState::Positioning)) {
    setMessage("Nothing to pause");
    return false;
  }
  pausedFrom_ = state_;
  SyncStepper::requestStop();
  state_ = MachineState::Pausing;
  setMessage("Pausing...");
  return true;
}

bool Machine::resume() {
  if (state_ != MachineState::Paused) {
    setMessage("Not paused");
    return false;
  }
  if (pausedFrom_ == MachineState::Positioning) {
    LayerMove first;
    plan_.layer(0, first);
    moveTraverseTo(first.traverseStartMm);
    state_ = MachineState::Positioning;
  } else {
    continueLayer();
    state_ = MachineState::Winding;
  }
  setMessage("Winding");
  return true;
}

bool Machine::stop() {
  if (!busy() && state_ != MachineState::Paused) {
    setMessage("Nothing to stop");
    return false;
  }
  if (state_ == MachineState::Homing) homed_ = false;
  SyncStepper::requestStop();
  state_ = MachineState::Stopping;
  setMessage("Stopping...");
  return true;
}

bool Machine::reset() {
  if (state_ == MachineState::EStop && estopActive()) {
    setMessage("Release E-stop first");
    return false;
  }
  if (!(state_ == MachineState::Fault || state_ == MachineState::EStop)) {
    setMessage("No fault to reset");
    return false;
  }
  fault_ = FaultCode::None;
  state_ = MachineState::Idle;
  setMessage("Press: Home axis");
  return true;
}

bool Machine::jogTraverse(float mm) {
  if (!homed_ || !(state_ == MachineState::Ready || state_ == MachineState::Done) || SyncStepper::busy()) return false;
  return moveTraverseTo(traverseMm() + mm);
}

bool Machine::saveRecipe() {
  EEPROM.put(EEPROM_ADDR, RECIPE_MAGIC);
  EEPROM.put(EEPROM_ADDR + (int)sizeof(RECIPE_MAGIC), recipe_);
  setMessage("Recipe saved");
  return true;
}

void Machine::loadRecipe() {
  recipe_ = {DEFAULT_TURNS, DEFAULT_PITCH_MM, DEFAULT_WIDTH_MM, DEFAULT_OFFSET_MM, DEFAULT_RPM};
  uint16_t magic = 0;
  EEPROM.get(EEPROM_ADDR, magic);
  if (magic != RECIPE_MAGIC) return;
  Recipe stored;
  EEPROM.get(EEPROM_ADDR + (int)sizeof(RECIPE_MAGIC), stored);
  if (WindPlan::validate(stored, limits()) == RecipeError::Ok) recipe_ = stored;
}

// ------------------------------- status -------------------------------
bool Machine::recipeEditable() const { return !busy() && state_ != MachineState::Paused; }

bool Machine::busy() const {
  return state_ == MachineState::Homing || state_ == MachineState::Positioning || state_ == MachineState::Winding ||
         state_ == MachineState::Pausing || state_ == MachineState::Stopping;
}

float Machine::turnsDone() const {
  return hasRun_ ? (float)SyncStepper::position(SyncStepper::SPINDLE) / SPINDLE_STEPS_PER_REV : 0.0f;
}

int32_t Machine::layerNumber() const { return (runActive_ || state_ == MachineState::Done) ? layerIndex_ + 1 : 0; }
int32_t Machine::layerTotal() const { return hasRun_ ? plan_.layerCount() : 0; }
float Machine::traverseMm() const { return SyncStepper::position(SyncStepper::TRAVERSE) / TRAVERSE_STEPS_PER_MM; }
float Machine::rpm() const { return SyncStepper::spindleStepHz() * 60.0f / SPINDLE_STEPS_PER_REV; }

const char* Machine::stateName(MachineState s) {
  switch (s) {
    case MachineState::Idle: return "IDLE";
    case MachineState::Homing: return "HOMING";
    case MachineState::Ready: return "READY";
    case MachineState::Positioning: return "POSITIONING";
    case MachineState::Winding: return "WINDING";
    case MachineState::Pausing: return "PAUSING";
    case MachineState::Paused: return "PAUSED";
    case MachineState::Stopping: return "STOPPING";
    case MachineState::Done: return "DONE";
    case MachineState::Fault: return "FAULT";
    case MachineState::EStop: return "E-STOP";
  }
  return "?";
}

const char* Machine::faultName(FaultCode f) {
  switch (f) {
    case FaultCode::None: return "NONE";
    case FaultCode::HomeFailed: return "HOME_FAILED";
    case FaultCode::LimitHit: return "LIMIT_HIT";
    case FaultCode::EStop: return "ESTOP";
  }
  return "?";
}
