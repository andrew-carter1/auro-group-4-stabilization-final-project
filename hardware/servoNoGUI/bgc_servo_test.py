#!/usr/bin/env python3
"""
bgc_servo_test.py
Send power to the PITCH servo on a SimpleBGC 8-bit board.

Steps:
  1. Handshake (CMD_BOARD_INFO)
  2. Read current params (CMD_READ_PARAMS, no payload → no PROFILE_ID prefix)
  3. Set PITCH: P=40, D=10, POWER=<power_arg>, PWM_FREQ=HIGH  (other axes untouched)
  4. Write params back (CMD_WRITE_PARAMS), wait for CMD_CONFIRM
  5. Motors ON
  6. Hold for <hold_seconds>, pumping CMD_CONTROL at 10 Hz (PITCH MODE_ANGLE @ 0°)
  7. Motors OFF: send CMD_CONTROL MODE_NO_CONTROL, then CMD_MOTORS_OFF

Usage:
    python bgc_servo_test.py              # /dev/ttyACM0, power=100, hold=5s
    python bgc_servo_test.py /dev/ttyACM0 150 8
"""

import sys
import time
import struct
import serial

# ---------------------------------------------------------------------------
# Command IDs (8-bit firmware 2.2b)
# ---------------------------------------------------------------------------
CMD_BOARD_INFO   = 0x56  # 86
CMD_READ_PARAMS  = 0x52  # 82
CMD_WRITE_PARAMS = 0x57  # 87  <-- NOT 0x77 (119); that's the 32-bit variant
CMD_MOTORS_ON    = 0x4D  # 77
CMD_MOTORS_OFF   = 0x6D  # 109
CMD_CONFIRM      = 0x43  # 67 — board→host: acknowledge a write
CMD_CONTROL      = 0x43  # 67 — host→board: gimbal control (same byte, opposite direction)

# ---------------------------------------------------------------------------
# Params body layout — firmware 2.2b 8-bit, read with no payload (no PROFILE_ID prefix)
# Bytes  0–17: axis config, 6 bytes each [P, I, D, POWER, INVERT, POLES]
# Byte  18:    ACC_LIMITER
# Bytes 19–20: EXT_FC_GAIN_ROLL, EXT_FC_GAIN_PITCH
# Bytes 21–44: RC params, 8 bytes per axis [MIN, MAX, MODE, LPF, SPEED, FOLLOW]
# Byte  45:    GYRO_TRUST
# Byte  46:    USE_MODEL
# Byte  47:    PWM_FREQ  (0=LOW, 1=HIGH, 2=ULTRA_HIGH)
# (Firmware 2.22 returns 94 bytes; bytes 48+ are extended fields — left untouched)
# ---------------------------------------------------------------------------
PARAMS_BODY_SIZE = 48
ROLL_OFFSET      = 0
PITCH_OFFSET     = 6
YAW_OFFSET       = 12
IDX_P, IDX_I, IDX_D, IDX_POWER = 0, 1, 2, 3
PWM_FREQ_OFFSET  = 47

# CMD_CONTROL helpers
MODE_NO_CONTROL = 0
MODE_ANGLE      = 2
CONTROL_SPEED   = 200  # slew rate (arbitrary units)

BAUD = 115200  # confirmed working
# BAUD = 57600
# BAUD = 38400
# BAUD  = 19200
# BAUD = 9600
# ---------------------------------------------------------------------------
# Packet helpers
# ---------------------------------------------------------------------------

def build_packet(cmd: int, body: bytes = b'') -> bytes:
    size = len(body)
    hdr_crc  = (cmd + size) & 0xFF
    body_crc = sum(body) & 0xFF
    return bytes([0x3E, cmd, size, hdr_crc]) + body + bytes([body_crc])

def read_response(ser: serial.Serial, expected_cmd: int, timeout: float = 2.0) -> bytes | None:
    deadline = time.time() + timeout
    buf = b''
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        else:
            time.sleep(0.02)
        tmp = bytearray(buf)
        while len(tmp) >= 5:
            if tmp[0] != 0x3E:
                tmp = tmp[1:]; continue
            cmd_id, data_len, hdr_crc = tmp[1], tmp[2], tmp[3]
            if (cmd_id + data_len) & 0xFF != hdr_crc:
                tmp = tmp[1:]; continue
            total = 4 + data_len + 1
            if len(tmp) < total:
                break
            body = bytes(tmp[4: 4 + data_len])
            if sum(body) & 0xFF != tmp[4 + data_len]:
                tmp = tmp[1:]; continue
            if cmd_id == expected_cmd:
                return body
            tmp = tmp[total:]
        buf = bytes(tmp)
    return None

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    port         = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    power        = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    hold_seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

    print(f"Opening {port} @ {BAUD} baud...")
    try:
        ser = serial.Serial(port, baudrate=BAUD, timeout=2.0, dsrdtr=False, rtscts=False)
    except serial.SerialException as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
    time.sleep(1.0)
    ser.reset_input_buffer()

    # --- 1. Handshake ---
    print("\n[1] CMD_BOARD_INFO...")
    ser.write(build_packet(CMD_BOARD_INFO))
    time.sleep(0.02)
    body = read_response(ser, CMD_BOARD_INFO) 
    if body is None:
        print("  [FAIL] No response.")
        ser.close(); sys.exit(1)
    if len(body) >= 3:
        print(f"  OK  firmware={body[2]}.{body[1]}  board_type={body[0]}")
    else:
        print(f"  OK  ({len(body)} bytes): {body.hex()}")

    # --- 2. Read params ---
    print("\n[2] CMD_READ_PARAMS (current active)...")
    ser.reset_input_buffer()
    ser.write(build_packet(CMD_READ_PARAMS))
    time.sleep(0.02)
    body = read_response(ser, CMD_READ_PARAMS, timeout=3.0)
    if body is None:
        print("  [FAIL] No response.")
        ser.close(); sys.exit(1)
    if len(body) < PARAMS_BODY_SIZE:
        print(f"  [FAIL] Expected >={PARAMS_BODY_SIZE} bytes, got {len(body)}: {body.hex()}")
        ser.close(); sys.exit(1)
    params = bytearray(body)
    print(f"  OK  ({len(body)} bytes)")

    # --- 3. Modify PITCH only + set PWM_FREQ to HIGH ---
    print(f"\n[3] Setting PITCH: P=40, I=0, D=10, POWER={power}  PWM_FREQ=HIGH  (ROLL/YAW unchanged)")
    params[PITCH_OFFSET + IDX_P]     = 40
    params[PITCH_OFFSET + IDX_I]     = 0
    params[PITCH_OFFSET + IDX_D]     = 10
    params[PITCH_OFFSET + IDX_POWER] = power
    params[PWM_FREQ_OFFSET]          = 1  # HIGH

    # --- 4. Write params, wait for confirm ---
    print("\n[4] CMD_WRITE_PARAMS (0x57)...")
    ser.reset_input_buffer()
    ser.write(build_packet(CMD_WRITE_PARAMS, bytes(params)))
    time.sleep(0.02)
    confirm = read_response(ser, CMD_CONFIRM, timeout=2.0)
    if confirm is not None:
        print("  CMD_CONFIRM received.")
    else:
        print("  [WARN] No CMD_CONFIRM — continuing anyway.")

    # --- 5. Motors ON ---
    print("\n[5] CMD_MOTORS_ON...")
    ser.write(build_packet(CMD_MOTORS_ON))
    time.sleep(0.02)
    on_resp = read_response(ser, CMD_CONFIRM, timeout=1.0)
    if on_resp is not None:
        print("  CMD_CONFIRM received — motors armed.")
    else:
        print("  Motors enabled (no confirm).")
    time.sleep(0.5)

    # --- 6. Hold — pump CMD_CONTROL at 10 Hz so board stays engaged ---
    print(f"\n[6] Holding for {hold_seconds:.0f}s  (Ctrl+C to stop early)...")

    def send_control_pitch(pitch_deg: float) -> None:
        def axis(mode, spd, ang):
            return struct.pack('<BHh', mode, spd, ang)
        body = (
            axis(MODE_NO_CONTROL, 0, 0) +
            axis(MODE_ANGLE, CONTROL_SPEED, int(pitch_deg * 10)) +
            axis(MODE_NO_CONTROL, 0, 0)
        )
        ser.write(build_packet(CMD_CONTROL, body))

    interval = 0.1  # 10 Hz
    deadline = time.time() + hold_seconds
    try:
        while time.time() < deadline:
            t0 = time.time()
            send_control_pitch(0.0)
            elapsed = time.time() - t0
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("  Interrupted.")

    # --- 7. Motors OFF ---
    # CMD_MOTORS_OFF (0x6D) is silently ignored by firmware 2.22.
    # Shutdown path: stop CMD_CONTROL, wait for board watchdog, then write POWER=0.
    print("\n[7] Motors OFF (watchdog + WRITE_PARAMS POWER=0)...")

    print("  Waiting 2s for board watchdog to release serial control mode...")
    time.sleep(2.0)

    for off in [ROLL_OFFSET, PITCH_OFFSET, YAW_OFFSET]:
        params[off + IDX_P]     = 0
        params[off + IDX_I]     = 0
        params[off + IDX_D]     = 0
        params[off + IDX_POWER] = 0
    ser.reset_input_buffer()
    ser.write(build_packet(CMD_WRITE_PARAMS, bytes(params)))
    confirm = read_response(ser, CMD_CONFIRM, timeout=1.5)
    if confirm is not None:
        print(f"  CMD_CONFIRM received (body=0x{confirm.hex()}) — POWER=0 applied, motors off.")
    else:
        print("  [WARN] No CMD_CONFIRM for WRITE_PARAMS — params may not have applied.")

    ser.close()
    print("Done.")

if __name__ == '__main__':
    main()
