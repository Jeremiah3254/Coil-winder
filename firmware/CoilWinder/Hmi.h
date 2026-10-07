// Hmi.h — Nokia 5110 (PCD8544) screen + KY-040 rotary encoder.
#pragma once
class Machine;
namespace Hmi {
void begin(Machine* machine);
void update();  // call every loop()
}
