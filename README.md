# Automated Coil Winder

University of Toledo senior design. An Arduino Mega 2560 winds magnet wire onto a bobbin. One stepper
spins the bobbin (spindle); a second moves the wire guide along a ball-screw stage (traverse) exactly one
wire diameter per turn, reversing at each end to build layers. Settings are entered in a laptop app over
USB, or on the machine's Nokia 5110 display and knob.

```
firmware/CoilWinder/   the program that goes on the Arduino Mega
app/                   the control app that runs on the laptop
```

## Parts this code assumes
| # | Part | Used for | Code setting that depends on it |
|---|---|---|---|
| 1 | Arduino Mega 2560 + USB A-B cable | controller | |
| 2 | 150 mm ball-screw stage (SFU1605, 5 mm lead) with NEMA 17 | traverse | `LEADSCREW_LEAD_MM`, `TRAVERSE_MAX_TRAVEL_MM` |
| 3 | NEMA 17 stepper, about 1.5 A | spindle | |
| 4 | 2x DRV8825 driver | one per motor | microstep jumpers (below) |
| 5 | 2x KP08 pillow block + 5-to-8 mm coupler | spindle bearings | |
| 6 | M8 threaded rod, 300 mm (mandrel) + M8 wing nuts | holds the bobbin | |
| 7 | Bobbins with a bore of at least 8 mm | | |
| 8 | Nokia 5110 display + level shifter (or 10k resistors) | on-machine display | `HAS_DISPLAY` |
| 9 | KY-040 rotary encoder | on-machine knob | `ENC_REVERSE` |
| 10 | 2x limit switch (microswitch with C / NO / NC) | home and end of travel | `HAS_END_SWITCH` |
| 11 | E-stop, latching, 1NO + 1NC | cuts motor power, tells the Mega | `HAS_ESTOP_SENSE` |
| 12 | 24 V power supply, 3 A or more, with fuse and power switch | motor power | |
| 13 | Buck converter set to 9 V | powers the Mega from 24 V | |
| 14 | 2x 100 µF 35 V (or higher) electrolytic capacitor | one across each driver's motor power | |
| 15 | 10 kΩ resistor | keeps the drivers off while the Mega boots | |
| 16 | Wire tensioner (felt pads) and supply spool holder | wire feed | |
| 17 | 2020 frame, 3D-printed mounts, wire, connectors, fasteners, magnet wire | build | |

## Wiring
| Mega pin | Connects to |
|---|---|
| 22 / 23 | Spindle DRV8825 STEP / DIR |
| 25 / 26 | Traverse DRV8825 STEP / DIR |
| 24 | EN on both DRV8825s, plus 10 kΩ to 5 V |
| 2 | Home limit switch: C to GND, NC to pin 2 |
| 3 | End limit switch: C to GND, NC to pin 3 |
| 18 | E-stop NO contact to pin 18, other side to GND |
| 30 / 31 / 32 | KY-040 CLK / DT / SW (KY-040 + to 5 V, GND to GND) |
| 51 / 52 / 53 / 49 / 48 | Nokia DIN / CLK / CE / DC / RST, each through the level shifter |
| 46 | Nokia LIGHT through 330 Ω. Nokia VCC to the Mega's 3.3 V pin |

**Power:** 24 V supply → fuse → power switch, then two branches:
* **Motors:** E-stop NC contact → VMOT on both DRV8825s, with a 100 µF capacitor at each driver.
* **Mega:** buck converter set to **9 V** with a multimeter **before** connecting → Mega VIN (or the barrel jack).

All grounds (supply, buck, Mega, both drivers) connect together.

**Each DRV8825:**
* **Jumpers:** M0 → 5 V, M1 → 5 V, M2 → GND (1/8 microstep); RESET and SLEEP → 5 V.
* **Current:** set Vref to about 0.6 V before connecting the motor.
* **Coils:** the motor wire pairs that read a few ohms between them are one coil; one pair goes to A1/A2, the other to B1/B2.
* **Never plug or unplug a motor with power on.**

## Upload the firmware to the Mega (Arduino IDE)
1. Install the Arduino IDE (arduino.cc/en/software).
2. File → Open → `firmware/CoilWinder/CoilWinder.ino` (the other files open as tabs).
3. Tools → Manage Libraries → install **U8g2** and **ArduinoJson**.
4. Tools → Board → **Arduino Mega or Mega 2560**; Tools → Port → the Mega's COM port.
5. Click **Upload**. Close the Arduino IDE afterwards so it releases the port.

## Run the control app
```
python -m venv .venv
.venv\Scripts\activate.bat          (Mac: source .venv/bin/activate)
pip install -r app/requirements.txt
python app/winder_app.py
```
Pick the Mega's COM port → **Connect** → **Home axis** → enter the recipe → **Start winding**.
Runs are logged to `app/runs/` (one CSV per run plus `summary.csv`).

## First run checklist
1. Power up with no bobbin. Click **Home axis**. If the guide moves away from the home switch, set
   `TRAVERSE_INVERT_DIR = true` in `config.h` and upload again.
2. Measure the real stage: fix `LEADSCREW_LEAD_MM` and `TRAVERSE_MAX_TRAVEL_MM` if they differ.
3. Tape a paper tube on the mandrel, put a marker in the wire guide, wind 50 turns and measure the
   spiral spacing. It should match the pitch you entered.
4. Set pitch to your wire's measured outside diameter and wind a real coil.

## On the machine (display + knob)
* **Status screen:** click to open the menu; while winding, click to pause, click again to resume, or hold to stop.
* **Faults and E-stop:** hold the knob to clear.
* **Menu:** turn to move, click to select or edit, hold to go back.

## USB commands (what the app sends)
`{"cmd":"home"}` · `{"cmd":"start","turns":500,"pitch":0.43,"width":20,"offset":10,"rpm":150}` ·
`{"cmd":"pause"}` · `{"cmd":"resume"}` · `{"cmd":"stop"}` · `{"cmd":"reset"}` · `{"cmd":"jog","mm":1}` ·
`{"cmd":"set",...}` then `{"cmd":"save"}` (stores the recipe in the Mega) · `{"cmd":"get"}`.
The Mega replies with `tel` (4x per second), `state`, `ack` and `done` messages.
