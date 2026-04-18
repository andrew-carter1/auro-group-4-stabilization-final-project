# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This repository contains Python scripts that communicate with a BaseCam SimpleBGC 8-bit gimbal controller over serial (USB-UART) using the SimpleBGC 8-bit Serial API.

| Script | Purpose |
|---|---|
| `BGC_PowerControl.py` | Zeros PID gains and steps through motor POWER levels [50,100,150,200] with 5-second holds |
| `bgc_servo_test.py` | Sends a single power level to the PITCH servo with configurable hold time |
| `bgc_connect.py` | Minimal connection tester — auto-scans baud rates and reports board info |

## Running the Scripts

```bash
# BGC_PowerControl.py — power sweep experiment
python BGC_PowerControl.py COM6
python BGC_PowerControl.py /dev/ttyS6          # WSL2
python BGC_PowerControl.py COM6 115200         # custom baud (default: 57600)
python BGC_PowerControl.py COM6 57600 --sniff   # raw-byte dump, no control commands
python BGC_PowerControl.py COM6 57600 --listen  # passive capture, no TX
python BGC_PowerControl.py COM6 57600 --scan    # try all common baud rates
python BGC_PowerControl.py COM6 57600 --probe   # fire all command IDs and report responses

# bgc_servo_test.py — single power level test
python bgc_servo_test.py               # COM6, power=100, hold=5s
python bgc_servo_test.py COM6 150 8    # COM6, power=150, hold=8s
# Baud is hardcoded to 115200 (confirmed working)

# bgc_connect.py — connection diagnostics
python bgc_connect.py                  # auto-scan baud rates on COM6
python bgc_connect.py COM3             # different port
python bgc_connect.py COM6 57600       # force a single baud rate
```

## Dependencies

```bash
pip install pyserial
```

## Architecture

### BGC_PowerControl.py

**Packet layer** (`build_packet`, `read_response`, `read_response_from_bytes`):
- Wire format: `0x3E | CMD | SIZE | (CMD+SIZE)%256 | [body] | sum(body)%256`
- `read_response_from_bytes` parses from an already-captured buffer (no I/O)
- `read_response` does live I/O with a timeout, scanning for the first matching `expected_cmd`

**High-level commands** (`cmd_board_info`, `read_params`, `write_params`, `motors_on`, `motors_off`, `send_control`):
- Each wraps a single Serial API command and handles basic error/timeout reporting
- `send_control` sends `CMD_CONTROL` for PITCH only; ROLL and YAW are left in `MODE_NO_CONTROL`

**Experiment loop** (`main`):
1. Handshake via `CMD_BOARD_INFO`
2. Read current params via `CMD_READ_PARAMS` (63-byte body)
3. Zero P/I/D on all axes; set POWER to first step
4. Enable motors
5. For each POWER step: update params, then send `CMD_CONTROL` at 10 Hz for 5 seconds
6. Turn motors off on exit (including `KeyboardInterrupt`)

### bgc_servo_test.py

Single-shot PITCH servo test. Sets PITCH P=40, I=0, D=10, POWER=`<arg>` (ROLL/YAW untouched), writes params, turns motors on, holds, then turns motors off. Baud hardcoded to 115200. Axis layout: ROLL at offset 0, PITCH at 6, YAW at 12 (6 bytes each: P, I, D, POWER, INVERT, POLES).

### bgc_connect.py

Minimal diagnostic tool. Sends `CMD_BOARD_INFO` and captures raw bytes. With no baud argument, auto-scans [9600, 19200, 38400, 57600, 115200, 230400, 256000] and stops on the first valid SimpleBGC packet. Prints both hex and ASCII of the raw RX buffer for debugging garbled responses.

## Key Constants

| Constant | Value | Notes |
|---|---|---|
| `PARAMS_BODY_SIZE` | 63 | Adjust if firmware returns 48 or 64 bytes |
| `PITCH_TARGET_DEG` | 30.0 | Target pitch angle in degrees |
| `POWER_STEPS` | [50,100,150,200] | Motor power levels (0–255) |
| `CONTROL_RATE_HZ` | 10 | CMD_CONTROL send frequency |
| Default baud | 57600 | Try 115200 if handshake times out |

## Known Failure Modes

- **Port open fails**: wrong port name or missing permissions (`sudo chmod a+rw /dev/ttyS6`)
- **CMD_BOARD_INFO times out**: baud rate mismatch; try 115200; close SimpleBGC GUI on Windows (it holds the port)
- **CMD_READ_PARAMS wrong size**: firmware version mismatch — adjust `PARAMS_BODY_SIZE`
- **CMD_CONTROL does nothing**: POWER caps PID output, so if P=I=D=0 no current flows. Try setting P=40, D=10 and varying POWER instead of zeroing all gains
