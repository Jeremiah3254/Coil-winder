// windplan_unit_test.cpp — Unit test WP-UT-01 for firmware/CoilWinder/WindPlan.cpp (runs on a PC, no hardware).

//Steps to use: 
// 1.) Download latest version of w64devkit at https://github.com/skeeto/w64devkit/releases
// 2.) Change to the top Coil-winder folder using cd "C:\.."
// 3.) Enter: "g++ -std=c++17 -I firmware/CoilWinder test/windplan_unit_test.cpp firmware/CoilWinder/WindPlan.cpp -o windplan_unit_test"
// 4.) Enter: "windplan_unit_test"
#include <cmath>
#include <cstdio>
#include "WindPlan.h"

// ---- Test constants (same values as config.h) ----
constexpr float  TEST_STEPS_PER_MM   = 320.0f;  // TRAVERSE_STEPS_PER_MM
constexpr float  TEST_MAX_TRAVEL_MM  = 140.0f;  // TRAVERSE_MAX_TRAVEL_MM
constexpr float  TEST_MIN_OFFSET_MM  = 2.5f;    // MIN_FIRST_TURN_OFFSET_MM
constexpr float  TEST_RPM_MIN        = 10.0f;   // SPINDLE_RPM_MIN
constexpr float  TEST_RPM_MAX        = 300.0f;  // SPINDLE_RPM_MAX
constexpr float  TEST_MM_TOLERANCE   = 0.001f;
constexpr Recipe TEST_DEFAULT_RECIPE = {200, 0.43f, 20.0f, 10.0f, 120.0f};  // turns, pitch, width, offset, rpm

static const RecipeLimits LIMITS = {TEST_MAX_TRAVEL_MM, TEST_MIN_OFFSET_MM, TEST_RPM_MIN, TEST_RPM_MAX};
static int g_passed = 0;
static int g_total = 0;

static void report(int n, const char* name, bool pass) {
  g_total++;
  if (pass) g_passed++;
  std::printf("Test %d: %-42s %s\n", n, name, pass ? "PASS" : "FAIL");
}

static bool near(float a, float b) { return std::fabs(a - b) <= TEST_MM_TOLERANCE; }

int main() {
  const Recipe r = TEST_DEFAULT_RECIPE;

  report(1, "Default recipe is accepted", WindPlan::validate(r, LIMITS) == RecipeError::Ok);

  Recipe bad = r;
  bad.turns = 0;
  report(2, "Turns = 0 is rejected", WindPlan::validate(bad, LIMITS) == RecipeError::Turns);

  bad = r;
  bad.offsetMm = 120.0f;
  bad.widthMm = 25.0f;
  report(3, "Offset + width over 140 mm is rejected", WindPlan::validate(bad, LIMITS) == RecipeError::Travel);

  WindPlan plan;
  plan.begin(r, TEST_STEPS_PER_MM);
  LayerMove l0{}, l1{}, l4{};
  plan.layer(0, l0);
  plan.layer(1, l1);
  plan.layer(4, l4);
  report(4, "46 turns per layer, 5 layers, last has 16", l0.turns == 46 && plan.layerCount() == 5 && l4.turns == 16);

  const bool l0ok = near(l0.traverseStartMm, 10.0f) && l0.traverseStartSteps == 3200 &&
                    near(l0.traverseEndMm, 29.78f) && l0.traverseEndSteps == 9530;
  const bool l1ok = near(l1.traverseStartMm, 29.78f) && l1.traverseStartSteps == 9530 &&
                    near(l1.traverseEndMm, 10.0f) && l1.traverseEndSteps == 3200;
  report(5, "Layer 1 winds back the opposite way", l0ok && l1ok);

  std::printf("Overall: %d of %d passed -> %s\n", g_passed, g_total, g_passed == g_total ? "PASS" : "FAIL");
  return g_passed == g_total ? 0 : 1;
}
