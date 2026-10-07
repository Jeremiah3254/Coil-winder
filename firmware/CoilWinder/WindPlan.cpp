// WindPlan.cpp — see WindPlan.h. Layer 0 runs away from home, layer 1 back, and so on.
#include "WindPlan.h"
#include <math.h>

// ---- Module tunables ----
constexpr float WINDPLAN_FLOOR_EPSILON = 1e-4f;  // so 30.0 / 0.3 counts as exactly 100 turns per layer

RecipeError WindPlan::validate(const Recipe& r, const RecipeLimits& lim) {
  if (r.turns < 1 || r.turns > WINDPLAN_MAX_TURNS) return RecipeError::Turns;
  if (!(r.pitchMm >= WINDPLAN_MIN_PITCH_MM && r.pitchMm <= WINDPLAN_MAX_PITCH_MM)) return RecipeError::Pitch;
  if (!(r.widthMm >= r.pitchMm)) return RecipeError::Width;
  if (!(r.offsetMm >= lim.minOffsetMm)) return RecipeError::Offset;
  if (!(r.offsetMm + r.widthMm <= lim.maxTravelMm)) return RecipeError::Travel;
  if (!(r.rpm >= lim.rpmMin && r.rpm <= lim.rpmMax)) return RecipeError::Rpm;
  return RecipeError::Ok;
}

const char* WindPlan::errorText(RecipeError e) {
  switch (e) {
    case RecipeError::Ok: return "OK";
    case RecipeError::Turns: return "Turns out of range";
    case RecipeError::Pitch: return "Pitch out of range";
    case RecipeError::Width: return "Width must be >= pitch";
    case RecipeError::Offset: return "Offset too close to home";
    case RecipeError::Travel: return "Offset + width too long";
    case RecipeError::Rpm: return "RPM out of range";
  }
  return "Recipe error";
}

void WindPlan::begin(const Recipe& r, float traverseStepsPerMm) {
  recipe_ = r;
  stepsPerMm_ = traverseStepsPerMm;
  const int32_t tpl = (int32_t)floorf(r.widthMm / r.pitchMm + WINDPLAN_FLOOR_EPSILON);
  turnsPerLayer_ = tpl < 1 ? 1 : tpl;
  layerCount_ = (r.turns + turnsPerLayer_ - 1) / turnsPerLayer_;
}

int32_t WindPlan::turnsBeforeLayer(int32_t index) const {
  if (index <= 0) return 0;
  if (index >= layerCount_) return recipe_.turns;
  return index * turnsPerLayer_;
}

bool WindPlan::layer(int32_t index, LayerMove& out) const {
  if (index < 0 || index >= layerCount_) return false;
  const int32_t remaining = recipe_.turns - index * turnsPerLayer_;
  const int32_t turns = remaining < turnsPerLayer_ ? remaining : turnsPerLayer_;
  const bool outbound = (index % 2) == 0;
  const float startMm = outbound ? recipe_.offsetMm : recipe_.offsetMm + turnsPerLayer_ * recipe_.pitchMm;
  const float endMm = outbound ? startMm + turns * recipe_.pitchMm : startMm - turns * recipe_.pitchMm;
  out.index = index;
  out.turns = turns;
  out.traverseStartMm = startMm;
  out.traverseEndMm = endMm;
  out.traverseStartSteps = (int32_t)lroundf(startMm * stepsPerMm_);  // rounded from mm each time: no drift
  out.traverseEndSteps = (int32_t)lroundf(endMm * stepsPerMm_);
  return true;
}
