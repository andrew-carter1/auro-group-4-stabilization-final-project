# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This directory contains Python scripts for controlling a **BaseCam SimpleBGC 8-bit board running firmware 2.2b** over serial (USB-UART) using the SimpleBGC Serial API.

An older copy of `BGC_PowerControl.py` lives at `hardware/BGC_PowerControl.py` — use the scripts here instead.

| Script | Purpose |
|---|---|
| `bgc_connect.py` | **Start here** — minimal connection tester, auto-scans baud rates |
| `bgc_read_params.py` | Reads and prints all 105 params, sets PWM_FREQ to HIGH, writes back |
| `bgc_servo_test.py` | Sends power to the PITCH servo with configurable hold time |
| `BGC_PowerControl.py` | Full experiment: zeros PID gains and steps through POWER levels [50,100,150,200] |
| `bgc_servo_test_prev.py` | Archived previous iteration of bgc_servo_test — kept for comparison only |

## Step 1 — Find your device

**WSL2:** USB-serial devices do not appear as `/dev/ttyS*`. Forward via `usbipd` each session:

```powershell
# PowerShell (Admin) on Windows
usbipd list                          # find the busid for the SimpleBGC (almost always COM6)
usbipd attach --wsl --busid <busid>
```

```bash
sudo chmod a+rw /dev/ttyACM0   # device appears as ttyACM0, not ttyS6
```

## Step 2 — Verify the connection

```bash
python3 bgc_connect.py /dev/ttyACM0
```

Auto-scans baud rates and prints firmware version. **Do not proceed until this passes.**

## Step 3 — Read params and set PWM_FREQ to HIGH

```bash
python3 bgc_read_params.py /dev/ttyACM0
```

Sends `CMD_READ_PARAMS` with `[0xFF]` payload (current active profile), prints all 105 fields, sets `PWM_FREQ=1` (HIGH/silent), and writes back.

## Step 4 — Single servo test

Modifies PITCH only (ROLL/YAW untouched), turns motors on, holds, then shuts off via watchdog + POWER=0.

```bash
python3 bgc_servo_test.py /dev/ttyACM0          # power=100, hold=5s
python3 bgc_servo_test.py /dev/ttyACM0 150 8    # power=150, hold=8s
```

Motors-off sequence: stop sending `CMD_CONTROL`, wait 2s for board watchdog to release serial control mode, then `CMD_WRITE_PARAMS` with all axis POWER=0. `CMD_MOTORS_OFF` (0x6D) is silently ignored by firmware 2.22.

## Step 5 — Full power sweep

Only run once Step 4 is confirmed working.

```bash
python3 BGC_PowerControl.py /dev/ttyACM0 115200
python3 BGC_PowerControl.py /dev/ttyACM0 57600 --sniff   # raw-byte dump
python3 BGC_PowerControl.py /dev/ttyACM0 57600 --listen  # passive capture
python3 BGC_PowerControl.py /dev/ttyACM0 57600 --scan    # try all baud rates
python3 BGC_PowerControl.py /dev/ttyACM0 57600 --probe   # fire all command IDs
```

## Protocol

- Packet: `0x3E | CMD | SIZE | (CMD+SIZE)%256 | [payload] | sum(payload)%256`
- Baud: 115200, no parity
- 20ms delay after each command to avoid buffer overflow
- `CMD_READ_PARAMS` (0x52): two modes depending on caller:
  - With `[0xFF]` payload → board prepends PROFILE_ID byte; response is 49–105 bytes
  - No payload → no PROFILE_ID prefix; `bgc_servo_test.py` uses this, expects 48 bytes
- `CMD_WRITE_PARAMS` (0x57): board responds with `CMD_CONFIRM` (0x43) on success
- `CMD_MOTORS_OFF` (0x6D): silently ignored by firmware 2.22 — use POWER=0 via WRITE_PARAMS instead

## Params layout

### `bgc_servo_test.py` — no payload, no PROFILE_ID prefix (48 bytes min)

| Bytes | Field |
|---|---|
| 0–5, 6–11, 12–17 | ROLL / PITCH / YAW axis: P, I, D, POWER, INVERT, POLES |
| 18 | ACC_LIMITER |
| 19–20 | EXT_FC_GAIN_ROLL, EXT_FC_GAIN_PITCH |
| 21–44 | RC params × 3 axes (8 bytes each: MIN, MAX, MODE, LPF, SPEED, FOLLOW) |
| 45 | GYRO_TRUST |
| 46 | USE_MODEL |
| 47 | PWM_FREQ (0=LOW, 1=HIGH, 2=ULTRA_HIGH) |

### `bgc_read_params.py` — `[0xFF]` payload, PROFILE_ID prefixed (49–105 bytes)

Byte 0 is PROFILE_ID. All axis bytes shift by +1 relative to the no-payload layout. Full 105-byte layout defined by `PARAMS_FMT` struct in `bgc_read_params.py`.

## Dependencies

```bash
pip install pyserial
```

## Known Failure Modes

- **Port open fails on WSL2**: use `usbipd attach` then connect to `/dev/ttyACM0`, not `/dev/ttyS6`
- **Port open fails (permissions)**: `sudo chmod a+rw /dev/ttyACM0`
- **CMD_BOARD_INFO times out**: baud mismatch; try 115200; close SimpleBGC GUI on Windows (it holds the port)
- **CMD_READ_PARAMS wrong size**: firmware mismatch — adjust `PARAMS_BODY_SIZE` (48 for 2.2b no-payload, 49+ with `[0xFF]` payload)
- **CMD_CONTROL does nothing**: if P=I=D=0, POWER has no effect — use P=40, D=10
- **Motors won't turn off with CMD_MOTORS_OFF**: expected on firmware 2.22 — use watchdog + WRITE_PARAMS POWER=0 (implemented in `bgc_servo_test.py`)
