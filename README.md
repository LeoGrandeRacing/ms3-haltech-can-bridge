# MS3 Pro → Haltech UC-10 CAN Bridge

An Arduino-based CAN bus bridge that translates MegaSquirt MS3 Pro Ultimate broadcast data into Haltech V2 CAN protocol, allowing you to run a Haltech UC-10 (or compatible) dash on a MegaSquirt ECU.

**Total parts cost: approximately $40-55** depending on enclosure choice and what you already have on hand.

---

> ⚠️ **Important Disclaimer — Please Read Before Building**
>
> This project has only been briefly tested in my own car at the time of release. While it has been verified to work — all major channels display correctly on the dash — I have not yet put significant runtime on it. Long-term reliability, edge cases, and bugs that may only appear under specific conditions (extreme temps, sustained high RPM, prolonged use, certain sensor combinations, etc.) are unknown at this point.
>
> Use this firmware at your own risk. I make no guarantees about its behavior in your specific setup, and I am not responsible for any damage to your ECU, dash, sensors, vehicle, or anything else that may result from using this code.
>
> If you build one and find issues or improvements, please open an issue or submit a pull request so the project can improve over time. This is a community-driven release — your testing and feedback are what will make it solid.

---

## What It Does

Reads the MS3 "Advanced Real-Time Broadcast" at 500 kbps, repackages every supported channel into Haltech V2 CAN format, and rebroadcasts at 1 Mbps to the Haltech dash. The dash thinks it's talking to a native Haltech ECU.

## Channels Working

- RPM, MAP, TPS, Battery Voltage
- Coolant Temp, Intake Air Temp
- Oil Temp, Fuel Temp
- Oil Pressure, Fuel Pressure, Coolant Pressure
- Wideband O2 / Lambda (with full Haltech WB1 controller emulation — no "Device Time Out")
- Ignition Advance, Barometric Pressure
- Gear Position, Wheel Speed, Ethanol %, EGT (when configured in MS3)

## Hardware Required

| Item | Qty | Approximate Cost (Amazon) | Notes |
|------|-----|---------------------------|-------|
| Arduino Nano clone (ATmega328P, 16MHz, 5V) | 1 | $8-12 for 3-pack | Cheap clones work fine. Avoid Nano Every (different chip, different timing). |
| MCP2515 CAN module with TJA1050 or TJA1051 transceiver | 2 | $9-13 for 2-pack | TJA1051 preferred for better automotive EMC. 8 MHz crystal. |
| Nano screw terminal adapter | 1 | $9-13 for 3-pack | Optional but highly recommended — makes wiring much easier. |
| IP65 project enclosure (100×68×50mm or larger) | 1 | $7-15 | A 100×68×50mm box fits the build but is tight. Larger boxes (120×120×75mm) make wiring easier. |
| Hookup wire (22-24 AWG), heat shrink, fuse holder, inline 1A fuse | — | $8-12 | If you don't already have these in your tool kit. |
| DTM-4 connector pigtail (for Haltech side) | 1 | $5-10 | Optional if you splice directly into Haltech harness. |

**Note:** Component multipacks mean your true incremental cost per build is lower if you already have parts on hand or build multiple bridges.

## Wiring

### Arduino Nano → MCP2515 Modules

| Nano Pin | CAN-A Module (MS3 side) | CAN-B Module (UC-10 side) |
|----------|-------------------------|----------------------------|
| D13      | SCK                     | SCK                        |
| D12      | SO (MISO)               | SO (MISO)                  |
| D11      | SI (MOSI)               | SI (MOSI)                  |
| D9       | CS                      | —                          |
| D10      | —                       | CS                         |
| D2       | INT                     | —                          |
| D3       | —                       | INT                        |
| 5V       | VCC                     | VCC                        |
| GND      | GND                     | GND                        |

**Note:** D13, D12, D11, 5V, and GND are shared between both modules (two wires from one Nano pin to both module pins). D9/D10 and D2/D3 are dedicated to one module each.

### CAN Bus Connections

| Module | Bitrate  | Connects To                                                |
|--------|----------|------------------------------------------------------------|
| CAN-A  | 500 kbps | MS3 white connector: Pin 34 (CAN-H, Tan), Pin 33 (CAN-L, Tan) |
| CAN-B  | 1 Mbps   | Haltech UC-10 DTM-4 connector: White (CAN-H), Blue (CAN-L) |

### Power

| Source                    | Connects To  | Notes                                              |
|---------------------------|--------------|----------------------------------------------------|
| 12V switched (1A fused)   | Nano VIN     | Permanent install power source                     |
| Vehicle/MS3 ground        | Nano GND     | Always connected                                   |
| USB                       | Nano USB     | Bench testing / programming only                   |

⚠️ **CRITICAL:** Never connect USB and 12V VIN simultaneously. Backfeed through the AMS1117 regulator on clone Nanos can destroy the board.

### Termination

CAN bus requires 120Ω termination at both ends of each bus.
- **CAN-A bus:** MS3 has internal termination at one end. Enable J1 jumper on MCP2515 #1 for the other end.
- **CAN-B bus:** UC-10 has internal termination (enable in NSP settings). Enable J1 jumper on MCP2515 #2 for the bridge end.

Proper termination on both ends of each bus is recommended for clean signals and reliable operation.

## MS3 TunerStudio Configuration

Navigate to **CAN-Bus/Testmodes → CAN Realtime Data Broadcasting** and configure:

- **Enable:** On
- **Base ID:** 1520

Enable these broadcast groups across the 4 pages:

**Page 1:**
- Groups 00, 01, 02, 03 at **50 Hz**
- Groups 08, 13, 14, 15 at **10 Hz**

**Page 2:**
- Groups 16, 17, 22, 23 at **10 Hz**
- Group 31 at **10 Hz** (AFR per cyl — required for wideband)

**Page 3:**
- Group 33 at **5 Hz** (gear)
- Group 42 at **10 Hz** (VSS)
- Group 47 at **2 Hz** (ethanol)

**Page 4:**
- Group 53 at **10 Hz** (fuel press/temp)
- Group 54 at **5 Hz**

**Disable** all OEM dash broadcasting (BMW E46, Alfa, Lotus, etc.) to avoid CAN bus conflicts.

## Haltech NSP Configuration

In Haltech NSP (Nexus Software Platform):

1. Configure dash to use **Haltech V2 CAN protocol** at **1 Mbps**
2. Enable **Termination Resistor** on the dash CAN channel
3. Set pressure gauges to display **"Absolute pressure"** mode (gauge mode causes -14.7 PSI offset issues)
4. Configure wideband display to use **WB1** input

## Sensor Mapping

The firmware is configured for Bosch PST-F1 combo sensors (10 bar / 145 PSI). The MS3 should be calibrated to output PSI for pressures and °F for temperatures. The firmware handles unit conversion to the Haltech protocol's expected format (bar × 1000).

To change sensor assignments, edit these lines in the firmware:

```cpp
constexpr int8_t FUELP_SENSOR_INDEX  = 0;   // Generic sensor 1 (0-indexed)
constexpr int8_t OILP_SENSOR_INDEX   = 1;   // Generic sensor 2
constexpr int8_t OILT_SENSOR_INDEX   = 2;   // Generic sensor 3
constexpr int8_t COOLP_SENSOR_INDEX  = 3;   // Generic sensor 4
constexpr int8_t FUELT_SENSOR_INDEX  = 5;   // Generic sensor 6
```

Set to `-1` for any sensor you don't have wired.

## Build Photos

### Bridge installed in the car
The complete bridge in a 100×68×50mm IP65 enclosure, installed in the car.

![Bridge installed](images/bridge_installed.jpg)

### Dash working — BMW theme
Live data displaying on the Haltech UC-10 using a BMW-themed gauge layout. Fuel pressure (43 PSI), battery voltage (12.56V), and other channels reading correctly.

![Dash BMW theme](images/dash_bmw.jpg)

### Dash working — Haltech default theme
Same setup running the stock Haltech gauge theme. All channels live and updating.

![Dash Haltech theme](images/dash_haltech.jpg)

## Installation

1. Download the firmware (`firmware/ms3_haltech_can_bridge.ino`)
2. Install the [Arduino IDE](https://www.arduino.cc/en/software)
3. Install the **mcp_can** library by Cory Fowler (Tools → Manage Libraries → search "mcp_can")
4. Open the .ino file, select Board: "Arduino Nano", Processor: "ATmega328P (Old Bootloader)" (for most clones)
5. Upload to Nano
6. Wire up per the diagram above
7. Configure MS3 broadcast groups in TunerStudio
8. Configure UC-10 in NSP
9. Power on and verify data on dash

## Compatibility

- **Tested on:** Haltech UC-10 + MS3 Pro Ultimate
- **Should work with:** IC-7, IC-7 Color, and other Haltech dashes using V2 CAN at 1 Mbps
- **MS3 variants:** Pro Ultimate confirmed. Other MS3 variants likely compatible if they support the Advanced Real-Time Broadcast format.

## Adapting for Other Dashes

The hardware design (Arduino Nano + 2× MCP2515) is universal. To adapt this for other CAN-based dashes (Holley, AEM CD7, MoTeC, Racepak, etc.):

1. Update the destination CAN bitrate if needed
2. Rewrite the broadcast functions (`send50()`, `send20()`, `send10()`, `send5()`) with the target dash's CAN message IDs and data layouts
3. Add any device announcement/heartbeat frames the target dash requires

The MS3 read side and overall framework remain unchanged. PRs welcome if you build variants for other dashes — happy to host them in this repo or link out to forks.

## Credits

- **PT Motorsport** — WB1 CAN sniffing article that provided the 0x2B1 protocol information
- **blacksheepinc/Haltech-wideband-emulator** — confirmed the 7-byte DLC requirement for WB1 frames
- **MS3 community on msextra.com** — documentation on the Advanced Real-Time Broadcast format
- **Claude (Anthropic's AI assistant)** — helped write and debug the firmware code. I drove the hardware design, reverse engineering, testing on my actual setup, and troubleshooting; the actual C++ code was developed with AI assistance since I'm not a programmer. Mentioning this for full transparency. The code has been thoroughly tested on my own car and verified to work, but if anyone wants to audit or improve it, that's encouraged.

## License

MIT License — see [LICENSE](LICENSE) file. Free to use, modify, distribute, and sell. Attribution appreciated but not required.

## Support Disclaimer

I'm happy to help where I can, but I'm releasing this for free in my spare time and can't promise quick or comprehensive support for everyone who builds one. Please:

1. **Read this README thoroughly** before asking questions
2. **Check the [Issues tab](../../issues)** to see if your question has been asked
3. If you find something the docs don't cover, **open an issue** and I'll get to it when I can

The community will hopefully help fill in the gaps over time. Contributions, improvements, and pull requests are welcome.

## Contributing

If you build this and find issues, improvements, or extensions:

- **Bug reports:** Open an issue with details about your setup and the problem
- **Improvements:** Submit a pull request
- **New dash support:** Fork the repo, build your variant, and submit a PR or link to your fork

This started as my personal project but I'd love to see it grow into a community resource.

---

*Built for my E30 turbo track/drift car. If this helps you get your project running, that's the whole point. Happy tuning.*

**Follow the build:**
- Instagram: [@frank_mopar](https://instagram.com/frank_mopar)
- Instagram: [@leogrande_racing](https://instagram.com/leogrande_racing)
