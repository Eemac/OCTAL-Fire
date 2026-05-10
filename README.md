# OCTAL-Fire
The largest collegiate-built drone in the world!
[OCTAL Website!](https://octalfire.com/)

<img width="100%" alt="THE DRONE" src="https://github.com/user-attachments/assets/7bd08627-0a60-4f48-a0ff-376e49a01b19" />

## What/Why is this project?
OCTAL is engineered to respond to wildfires faster--and safer--than humans can. Satellites can already detect hotspots with a resolution of ~500ft. Our system will autonomously deploy to the detected hotspot, perform an analysis with 6 visible and infrared-spectrum cameras, and release retardant before the fire has time to get large, saving wildlife, property, and human lives. With four 66Ah, 60V solid-state LiPo battery packs, a 120kg payload, and a projected 40-minute flight time, OCTAL is built smart, tough, and versatile.

## Commands for power-up on the CANable:
- sudo slcand -f -o -s6 -S 115200 -t hw /dev/ttyACMX can0
- sudo ip link set can0 up
- candump can0
- cansniffer -t 0 can0 (-t 0 will display all messages)
- cansend can0

## CANable Web Flasher:
[Link Here](https://canable.io/updater/canable2.html)

## Directory Structure:
### Root

| Path | Description |
|---|---|
| `Firmware/` | Embedded firmware for flight systems, BMS, charger, and CAN communication. |
| `Hardware/` | KiCad PCB designs, fabrication outputs, and mechanical files. |
| `libraries/` | Shared KiCad symbols, footprints, and 3D models. |
| `Software (Sensing)/` | Placeholder for sensing-side software. |
| `Software (Base Station)/` | Placeholder for base station software. |
| `README.md` | Main project overview. |

---

### Firmware/
Main embedded software tree.

| Path | Description |
|---|---|
| `Core/` | Main flight controller firmware. |
| `BMS/` | Battery Management System firmware. |
| `Charger/` | Charger controller firmware. |
| `can_api/` | Python CAN code-generation tools. |
| `libraries/` | Embedded driver libraries. |
| `can_lib.dbc` | CAN database definitions. |
| `x11.dbc` | Additional CAN message definitions. |
| `requirements.txt` | Python dependencies. |
| `run.sh` | Build script. for CAN message functions |

### Firmware/can_api/

Python tooling for generating CAN interfaces.

| File | Description |
|---|---|
| `c_generator.py` | Generates C CAN interface code. |
| `dbc_generator.py` | Generates DBC files. |
| `yaml_handler.py` | Parses YAML definitions. |
| `utils.py` | Shared helper functions. |
| `tests/` | Unit tests for generators. |
| `files/c_templates/` | Templates for generated C code. |

---

### Hardware/

PCB and electrical hardware designs.

| PCB / Project | Description |
|---|---|
| `Core/flight-core` | Main flight controller and avionics motherboard integrating STM32 control, CAN routing, power conversion, Cube interface logic, and system interconnects. |
| `BMS/` | Battery Management System board handling cell monitoring, balancing, thermistor sensing, fan control, and high-voltage battery telemetry. |
| `Power Board/` | High-current power distribution board for routing battery power to vehicle subsystems and monitoring thermal/load conditions. |
| `PowerFet/` | High-side MOSFET switching board for controllable power distribution and subsystem shutdown protection. |
| `Charger/` | Dedicated battery charger controller board with onboard regulators, USB-C input handling, and programmable charge control circuitry. |
| `GroundRing/` | Ground-distribution PCB intended to centralize system grounding and reduce high-current return path issues. |
| `Cube-Flipper/` | Adapter/interface board for Cube flight controller mounting or signal rerouting. |
| `Power Board/Thermistors/` | Small thermistor sensing daughterboard used for distributed temperature monitoring. |
| `Coaster/` | Small auxiliary/demo PCB project included in the repository. |
---

### libraries/

Shared KiCad libraries.

| Path | Description |
|---|---|
| `3DModels/` | STEP/STP component models. |
| `OlinBAD.pretty/` | Custom PCB footprints. |
| `parts/` | Symbols, footprints, datasheets, and docs. |
| `OlinBAD.kicad_sym` | KiCad symbol library. |

