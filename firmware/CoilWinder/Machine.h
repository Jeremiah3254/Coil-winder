// Machine.h — state machine: homing, layer-by-layer winding, pause/resume/stop, limits, E-stop.
#pragma once
#include <stdint.h>
#include "WindPlan.h"

enum class MachineState : uint8_t { Idle, Homing, Ready, Positioning, Winding, Pausing, Paused, Stopping, Done, Fault, EStop };
enum class FaultCode : uint8_t { None, HomeFailed, LimitHit, EStop };

struct RunSummary {
  bool completed;
  int32_t targetTurns;
  float turnsWound;
  int32_t layers;
  uint32_t durationMs;
};

class Machine {
 public:
  void begin();
  void update();  // call every loop()

  bool home();
  bool start();
  bool pause();
  bool resume();
  bool stop();
  bool reset();
  bool jogTraverse(float mm);
  bool saveRecipe();

  Recipe& recipe() { return recipe_; }
  RecipeLimits limits() const;
  bool recipeEditable() const;

  MachineState state() const { return state_; }
  FaultCode fault() const { return fault_; }
  static const char* stateName(MachineState s);
  static const char* faultName(FaultCode f);
  const char* message() const { return message_; }

  float turnsDone() const;
  int32_t layerNumber() const;
  int32_t layerTotal() const;
  float traverseMm() const;
  float rpm() const;
  bool homed() const { return homed_; }
  bool busy() const;
  bool takeSummary(RunSummary& out);

 private:
  enum class HomePhase : uint8_t { Clear, Seek, Backoff };
  void setMessage(const char* m);
  void hardFault(FaultCode f, const char* msg);
  bool homeSwitch() const;
  bool endSwitch() const;
  bool estopActive();
  void checkLimits();
  void updateHoming();
  bool moveTraverseTo(float mm);
  bool startLayer(int32_t index);
  bool continueLayer();
  void finishRun(bool completed);
  void loadRecipe();

  WindPlan plan_;
  Recipe recipe_{};
  MachineState state_ = MachineState::Idle;
  MachineState pausedFrom_ = MachineState::Winding;
  FaultCode fault_ = FaultCode::None;
  HomePhase homePhase_ = HomePhase::Seek;
  bool homed_ = false;
  char message_[40] = "";
  int32_t layerIndex_ = 0;
  int32_t spindleTarget_ = 0;
  int32_t traverseTarget_ = 0;
  bool runActive_ = false;
  bool hasRun_ = false;
  uint32_t runStartMs_ = 0;
  uint8_t estopCount_ = 0;
  RunSummary summary_{};
  bool summaryReady_ = false;
};
