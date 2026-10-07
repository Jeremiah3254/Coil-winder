// WindPlan.h — winding math: turns per layer and where each layer starts and ends.
#pragma once
#include <stdint.h>

// ---- Module tunables ----
constexpr int32_t WINDPLAN_MAX_TURNS    = 100000;
constexpr float   WINDPLAN_MIN_PITCH_MM = 0.02f;
constexpr float   WINDPLAN_MAX_PITCH_MM = 5.0f;

struct Recipe {
  int32_t turns;    // total turns
  float   pitchMm;  // turn-to-turn spacing = wire outside diameter
  float   widthMm;  // usable bobbin width
  float   offsetMm; // guide position of the first turn, measured from the home switch
  float   rpm;      // spindle speed
};

struct RecipeLimits {
  float maxTravelMm;
  float minOffsetMm;
  float rpmMin, rpmMax;
};

enum class RecipeError : uint8_t { Ok, Turns, Pitch, Width, Offset, Travel, Rpm };

struct LayerMove {
  int32_t index;
  int32_t turns;
  int32_t traverseStartSteps;
  int32_t traverseEndSteps;
  float   traverseStartMm;
  float   traverseEndMm;
};

class WindPlan {
 public:
  static RecipeError validate(const Recipe& r, const RecipeLimits& lim);
  static const char* errorText(RecipeError e);
  void begin(const Recipe& r, float traverseStepsPerMm);
  int32_t layerCount() const { return layerCount_; }
  int32_t turnsBeforeLayer(int32_t index) const;
  bool layer(int32_t index, LayerMove& out) const;

 private:
  Recipe  recipe_{};
  float   stepsPerMm_    = 1.0f;
  int32_t turnsPerLayer_ = 1;
  int32_t layerCount_    = 0;
};
