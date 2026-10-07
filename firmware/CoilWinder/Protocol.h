// Protocol.h — USB serial, one JSON object per line (same format the desktop app in /app speaks).
#pragma once
class Machine;
namespace Protocol {
void begin(Machine* machine);
void update();  // call every loop()
}
