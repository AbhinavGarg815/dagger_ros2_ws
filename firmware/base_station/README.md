# RTK base station: the Blue Pill firmware

The base is a u-blox F9P and an RFD868 UX, joined by a Blue Pill (STM32F103C8). The
Blue Pill:
- sets the F9P up: survey-in, or fixed on a site's coordinates;
- sends its RTCM3 corrections to the rover inside MAVLink `GPS_RTCM_DATA` (the
  Cube uses them directly; RTK keeps working even when the Jetson is down);
- says what it is doing;
- takes the rover's commands, which come from the app's **Base station** screen.

How the base and the rover talk: DaggerGCS `docs/proposals/base-station.md`. The rover side:
`dagger_bringup/base_station` (`base_link.py`).

## Wiring

| Blue Pill | to |
|---|---|
| PA9 (USART1 TX) | F9P UART1 RX |
| PA10 (USART1 RX) | F9P UART1 TX |
| PA2 (USART2 TX) | RFD868 RX |
| PA3 (USART2 RX) | RFD868 TX |
| GND | GND of the F9P and the RFD |
| 5 V or 3.3 V | power, as the board allows |

All logic is 3.3 V. **Power the RFD868 from a 5 V supply that can give 1 A**, not from the
Blue Pill: it draws up to about 0.8 A when sending. LED on PC13: **steady** = fixed (sending
corrections), **slow blink** = surveying, **fast blink** = waiting for a GPS fix.

## Build and flash

```
cd firmware/base_station
pio run                          # build
pio run -t upload                # flash with an ST-Link (SWDIO, SWCLK, GND, 3.3 V)
pio run -e serial -t upload      # or a USB-serial adapter on PA9/PA10, BOOT0 jumper = 1, reset; BOOT0 = 0 after
g++ -std=c++17 -Wall -I include test/codec_test.cpp -o /tmp/codec_test && /tmp/codec_test   # its MAVLink/UBX/RTCM, on a PC
```

## The F9P: nothing to set

The Blue Pill sets the F9P at every power-up, in RAM. It doesn't matter what is
saved in the F9P (factory settings are fine), or whether a save in u-center stuck.
If the F9P goes quiet for 5 s (it restarted on its own, or lost power and forgot the
RAM settings), the Blue Pill sends it all again, every 5 s until it answers. It sends:
- **UART1:** 115200 baud, UBX in, UBX + RTCM3 out, no NMEA. It sends this at
  each usual speed (38400, 9600, 57600, 115200, 230400, 460800), since it can't
  know the current one, then talks at 115200;
- **the receiver:** stationary dynamic model, one solution a second;
- **the corrections on UART1:** RTCM 1005 every 5 s; 1074, 1084, 1094 and 1124
  every second; 1230 every 5 s. Plus `UBX-NAV-PVT` and `UBX-NAV-SVIN` every
  second;
- **the mode and position:** survey-in, or fixed on a site (`CFG-TMODE-*`),
  from the last command.

The keys and frames are checked against pyubx2 and pymavlink (`test/codec_test.cpp`).
Receiver firmware: HPG 1.13 or newer. Leave the F9P's USB free for u-center if you
want to watch it.

## The two RFD868 radios: the same settings on both

Set with RFD Tools or Mission Planner's SiK radio page: **load, change, save on both**.

| Setting | Value |
|---|---|
| Serial speed | 57 (57600) |
| Air speed | 64 |
| Net ID | the same on both, not the default 25 (e.g. 47) |
| MAVLink | 1 (MAVLink framing) |
| ECC | 0 (turn on only if the link drops packets at range) |
| Frequencies / power | the India (IND) firmware's band, 865–867 MHz, within the legal power |

## The Cube (Mission Planner, Full Parameter List)

`n` is the TELEM port the rover's RFD868 is on (TELEM1 = SERIAL1, TELEM2 = SERIAL2).

| Parameter | Value | Why |
|---|---|---|
| `SERIALn_PROTOCOL` | 2 (MAVLink 2) | |
| `SERIALn_BAUD` | 57 | the radio's serial speed |
| `SRn_RAW_SENS`, `SRn_EXT_STAT`, `SRn_RC_CHAN`, `SRn_POSITION`, `SRn_EXTRA1`, `SRn_EXTRA2`, `SRn_EXTRA3`, `SRn_PARAMS` | 0 | the base needs no telemetry: the radio stays free for corrections (Mission Planner on that radio asks for what it needs) |
| `GPS_INJECT_TO` | 127 (all) | the default: corrections to the C-RTK 2HP |
| `GPS1_TYPE` | 9 (DroneCAN) | as now |
| `GPS_POS1_X`, `_Y`, `_Z` | the antenna's offset from the rover's centre, m | centimetre RTK needs it |

Check `SERIALn_OPTIONS` does not have "Don't forward MAVLink to/from" set (default: not set),
and `SYSID_ENFORCE` is 0 (the default): at 1 the Cube ignores the base (system 71).

## On the rover, the first time: does it take the corrections?

Corrections only help once the base is **fixed** (on a site, or its survey done): only
then does the F9P send RTCM 1005, the base's position, and without it the rover can't
use the rest. While the base surveys, the rover stays at its ordinary GPS fix.

1. Base fixed: its LED steady; the app's Base station screen says it is sending
   corrections.
2. Rover outdoors, open sky, a minute or two. Watch, read-only, any one of:
   - Mission Planner (USB to the Cube): GPS status **RTK Float**, then **RTK Fixed**;
   - the app's status strip (RTK float / fixed);
   - on the Jetson: `ros2 topic echo /mavros/gpsstatus/gps1/raw --field fix_type`:
     5 = RTK float, 6 = RTK fixed (3 = plain 3D fix: no corrections used).
3. Still at 3 after 2 minutes, base fixed:
   - Mission Planner, MAVLink Inspector, the Cube's link: `GPS_RTCM_DATA` from system
     71 arriving about 4–5 a second? None: the radios (Net ID, speeds), TELEM2 wiring,
     `SERIAL2_*`;
   - arriving, still 3: `GPS_INJECT_TO` (127), the C-RTK 2HP's CAN, its firmware.
4. Don't use Mission Planner's own "Inject GPS" at the same time: two sources of
   corrections mix their pieces and the Cube throws both away.

Base fixed on a site whose coordinates are wrong by some metres: the rover still gets
RTK fixed, but every position it reports is shifted by that error. Survey a site once
at a clear spot (or take known coordinates) and save it.

## Bench test, before the rover

A USB-serial adapter in place of the RFD868 (adapter RX ← PA2, adapter TX → PA3, GND), on a PC
or the Jetson:

```
python3 tools/base_bench.py /dev/ttyUSB0                  # watch it: heard, mode, survey, RTCM bytes/s
python3 tools/base_bench.py /dev/ttyUSB0 survey 120 3.0   # a short survey-in
python3 tools/base_bench.py /dev/ttyUSB0 fixed 26.7478123 75.8554012 418.237
```

Expect *heard* and *satellites N* (*F9P not heard* means the F9P's UART1 isn't answering:
check its wiring and power), then *surveying* with its accuracy falling, then *fixed* with its position;
RTCM about 300–1000 bytes a second once it has satellites. Power-cycle it: it comes back in
the same mode.
