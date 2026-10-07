// SyncStepper.h — drives both steppers from one Timer1 interrupt.
// Moves are straight lines in (spindle steps, traverse steps): the axis with more steps sets the
// pace and the other follows with Bresenham, so the traverse advances exactly one pitch per spindle
// turn at every instant, including while speeding up and slowing down. Step pins can be any pins.
#pragma once
#include <stdint.h>

namespace SyncStepper {
constexpr uint8_t SPINDLE = 0;
constexpr uint8_t TRAVERSE = 1;

void begin();
void enable(bool on);
bool enabled();
// Relative move. rateHz / accelHz2 are for the spindle if it moves, otherwise for the traverse.
bool startMove(int32_t dSpindle, int32_t dTraverse, float rateHz, float accelHz2);
// Runs one axis until stopped (homing).
bool startContinuous(uint8_t axis, int8_t dir, float rateHz, float accelHz2);
void requestStop();   // decelerate to a stop
void hardStop();      // stop now
void update();        // call every loop(): updates the speed ramp
bool busy();
int32_t position(uint8_t axis);
void setPosition(uint8_t axis, int32_t steps);
float spindleStepHz();
}
