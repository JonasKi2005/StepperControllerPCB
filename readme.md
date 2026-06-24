# 6-Axis Robot Arm — Firmware (v1: ESP32-S3 + TMC2209)

Firmware for a custom 3D-printed 6-axis robotic arm (~50 cm reach, 2 kg payload).
This is the **first-generation control board**, built around an **ESP32-S3** carrier
PCB driving **six TMC2209** stepper drivers, with a WiFi web UI for manual control
and bring-up.

> **Status:** This is the original ESP32-S3 board and is **retired**. It served as the
> first working bring-up of the arm, but the fabricated PCB had a UART addressing
> limitation (the driver address-select pins were not broken out as intended, so the
> address-multiplexing scheme below could not be used as designed). Development has
> since moved to a custom STM32G484 + TMC5240 board. This repo is kept as a reference
> for the working step-generation and driver code.

---

## Hardware

| Component       | Part                                            |
| --------------- | ----------------------------------------------- |
| MCU             | ESP32-S3 (DevKitC-class carrier PCB)            |
| Stepper drivers | 6× TMC2209 (BTT V1.2 modules, R_sense = 0.11 Ω) |
| Motors J1–J3    | NEMA 17 `17HS19-2004S1` (2 A)                   |
| Motors J4–J6    | NEMA 17 `17HS13-0404S` (0.4 A, short body)      |
| Power           | 24 V supply → drivers; logic rails on-board     |

### Driver bus layout

The six drivers are split across **two UART buses**, three drivers per bus, using
the TMC2209's single-wire half-duplex UART with **address multiplexing** (each driver
on a bus is assigned address 0, 1, or 2):

```
UART1 ──┬── J1 (addr 0)      UART2 ──┬── J4 (addr 0)
        ├── J2 (addr 1)              ├── J5 (addr 1)
        └── J3 (addr 2)              └── J6 (addr 2)
```

All six drivers share a single **EN line**. The arm boots **disarmed** (EN held HIGH,
motors freewheeling) and only energizes after a POST to `/arm`.

---

## Firmware architecture

Built on **ESP-IDF (FreeRTOS)**. Four main translation units:

### `main.cpp` — entry point

Boot sequence: NVS init → construct one `TMC2209` per axis (UART + GPIO setup only) →
optionally push register config to each driver → start the motion controller →
bring up WiFi (the web server starts automatically once an IP is assigned). All pin
assignments and per-joint parameters (current, microsteps) live here in one place.

### `TMC2209.cpp` — driver class

Low-level UART register access with CRC, plus higher-level config:

- **`write_reg` / `read_reg`** — 8-byte datagrams, CRC-checked, with echo handling for
  single-wire half-duplex.
- **`init()`** — pushes GCONF/CHOPCONF/IHOLD_IRUN/PWMCONF and **verifies writes landed**
  by checking that `IFCNT` incremented by the expected count (your UART-wiring smoke test).
- **`set_current()`** — computes IRUN/IHOLD from a requested RMS current, automatically
  selecting `vsense` for best resolution.
- **`set_microsteps()`** — maps microstep count to the CHOPCONF MRES field.
- **`print_drv_status()`** — decodes DRV_STATUS flags (overtemp, short, open-load, etc.).

### `MotorController.cpp` — motion engine

The ESP32-S3 has only 4 hardware GPTimers, so **one timer per axis is impossible**.
Instead a **single shared timer fires at 20 kHz**, and the ISR uses **Bresenham-style
step accumulation** (the same pattern Marlin/Klipper use): each active axis accumulates
its `velocity_sps` per tick and emits a step pulse when the accumulator crosses the tick
rate. Supports per-axis jogging, absolute `move_to` targets, and clean stop/disarm.
Commands flow through a FreeRTOS queue to a dedicated motion task pinned to core 1.

### `Server.cpp` — WiFi + web UI

Connects as a WiFi station and serves a self-contained HTML control panel (no external
assets). The UI polls `/status` for live position and arm state, and exposes jog,
move-to, arming, current, and microstep controls.

#### HTTP API

| Method | Endpoint      | Params                | Purpose                                             |
| ------ | ------------- | --------------------- | --------------------------------------------------- |
| GET    | `/`           | —                     | Web control UI                                      |
| GET    | `/status`     | —                     | JSON: arm state, per-axis step counts, moving flags |
| POST   | `/arm`        | —                     | Energize drivers (EN low)                           |
| POST   | `/disarm`     | —                     | De-energize drivers (EN high)                       |
| POST   | `/stop_all`   | —                     | Stop all axes                                       |
| POST   | `/jog`        | `axis`, `dir`, `v`    | Continuous jog until stop                           |
| POST   | `/stop`       | `axis`                | Stop one axis                                       |
| POST   | `/move_to`    | `axis`, `target`, `v` | Move one axis to a step target                      |
| POST   | `/current`    | `axis`, `ma`          | Set RMS current (50–2000 mA)                        |
| POST   | `/microsteps` | `axis`, `us`          | Set microstepping (1…256)                           |

---

## Building & flashing

Requires the **ESP-IDF** toolchain. From the firmware directory:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

On macOS the ESP32-S3 enumerates via the **Silicon Labs CP210x** driver; `<PORT>` is
typically `/dev/cu.SLAB_USBtoUART` or similar.

Set your WiFi credentials (`WIFI_SSID` / `WIFI_PASS`) before building. On boot the
serial monitor prints the assigned IP address — open it in a browser to reach the UI.

> **Bring-up note:** Driver `init()` is commented out in `main.cpp` by default so the
> board can boot and run the web UI without working UART to the drivers. Re-enable it
> once the UART wiring is verified — `init()` will report via `IFCNT` whether register
> writes are actually reaching each driver.

---

## Known limitations of this board

- **UART addressing:** the address-multiplexing scheme above depends on each driver's
  address-select pins being individually set. On the fabricated board this wasn't the
  case and has to be fixed in the kicad files.
- No closed-loop feedback — purely open-loop step counting.
- This is my first PCB so there probably will be some other errors, since the PCB wasn't tested so far.

---

## A note on the code

**The code in this repository is handwritten.** AI was used only to polish and comment
it — the architecture, logic, and implementation decisions are my own.
