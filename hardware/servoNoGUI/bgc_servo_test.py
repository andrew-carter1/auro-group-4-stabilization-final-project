#!/usr/bin/env python3
"""
bgc_servo_test.py
Send power to the PITCH servo on a SimpleBGC 8-bit board.

Steps:
  1. Handshake (CMD_BOARD_INFO)
  2. Read current params (CMD_READ_PARAMS)
  3. Set PITCH: P=40, D=10, POWER=<power_arg>  (other axes untouched)
  4. Write params back (CMD_WRITE_PARAMS)
  5. Motors ON
  6. Hold for <hold_seconds>
  7. Motors OFF

Usage:
    python bgc_servo_test.py              # COM6, power=100, hold=5s
    python bgc_servo_test.py COM6 150 8   # COM6, power=150, hold=8s
"""

import sys
import time
import struct
import serial

# ---------------------------------------------------------------------------
# Command IDs (8-bit firmware)
# ---------------------------------------------------------------------------
CMD_BOARD_INFO   = 0x56  # 86
CMD_READ_PARAMS  = 0x52  # 82
CMD_WRITE_PARAMS = 0x57  # 87  <-- NOT 0x77 (119); that's the 32-bit variant
CMD_MOTORS_ON    = 0x4D  # 77
CMD_MOTORS_OFF   = 0x6D  # 109

# ---------------------------------------------------------------------------
# Params body layout  (63 bytes for firmware 2.x 8-bit)
# Each axis block is 6 bytes: P, I, D, POWER, INVERT, POLES
# ---------------------------------------------------------------------------
PARAMS_BODY_SIZE = 63
ROLL_OFFSET  = 0
PITCH_OFFSET = 6
YAW_OFFSET   = 12
IDX_P, IDX_I, IDX_D, IDX_POWER = 0, 1, 2, 3

BAUD = 115200  # confirmed working

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
    port         = sys.argv[1] if len(sys.argv) > 1 else 'COM6'
    power        = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    hold_seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

    print(f"Opening {port} @ {BAUD} baud...")
    try:
        ser = serial.Serial(port, baudrate=BAUD, timeout=2.0)
    except serial.SerialException as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
    time.sleep(1.0)
    ser.reset_input_buffer()

    # --- 1. Handshake ---
    print("\n[1] CMD_BOARD_INFO...")
    ser.write(build_packet(CMD_BOARD_INFO))
    body = read_response(ser, CMD_BOARD_INFO)
    if body is None:
        print("  [FAIL] No response.")
        ser.close(); sys.exit(1)
    if len(body) >= 3:
        print(f"  OK  firmware={body[2]}.{body[1]}  board_type={body[0]}")
    else:
        print(f"  OK  ({len(body)} bytes): {body.hex()}")

    # --- 2. Read params ---
    print("\n[2] CMD_READ_PARAMS...")
    ser.reset_input_buffer()
    ser.write(build_packet(CMD_READ_PARAMS))
    body = read_response(ser, CMD_READ_PARAMS, timeout=3.0)
    if body is None:
        print("  [FAIL] No response.")
        ser.close(); sys.exit(1)
    if len(body) < PARAMS_BODY_SIZE:
        print(f"  [FAIL] Expected {PARAMS_BODY_SIZE} bytes, got {len(body)}: {body.hex()}")
        ser.close(); sys.exit(1)
    params = bytearray(body)
    print(f"  OK  ({len(body)} bytes)")
    for name, off in [("ROLL", ROLL_OFFSET), ("PITCH", PITCH_OFFSET), ("YAW", YAW_OFFSET)]:
        print(f"  {name}: P={params[off+IDX_P]}  I={params[off+IDX_I]}  "
              f"D={params[off+IDX_D]}  POWER={params[off+IDX_POWER]}")

    # --- 3. Modify PITCH only ---
    print(f"\n[3] Setting PITCH: P=40, I=0, D=10, POWER={power}  (ROLL/YAW unchanged)")
    params[PITCH_OFFSET + IDX_P]     = 40
    params[PITCH_OFFSET + IDX_I]     = 0
    params[PITCH_OFFSET + IDX_D]     = 10
    params[PITCH_OFFSET + IDX_POWER] = power

    # --- 4. Write params ---
    print("\n[4] CMD_WRITE_PARAMS (0x57)...")
    ser.reset_input_buffer()
    ser.write(build_packet(CMD_WRITE_PARAMS, bytes(params)))
    time.sleep(0.3)  # 8-bit firmware may not send a confirmation

    # --- 5. Motors ON ---
    print("\n[5] CMD_MOTORS_ON...")
    ser.write(build_packet(CMD_MOTORS_ON))
    time.sleep(0.5)
    print("  Motors enabled.")

    # --- 6. Hold ---
    print(f"\n[6] Holding for {hold_seconds:.0f}s  (Ctrl+C to stop early)...")
    try:
        time.sleep(hold_seconds)
    except KeyboardInterrupt:
        print("  Interrupted.")

    # --- 7. Motors OFF ---
    print("\n[7] CMD_MOTORS_OFF...")
    ser.write(build_packet(CMD_MOTORS_OFF))
    time.sleep(0.3)
    ser.close()
    print("Done.")

if __name__ == '__main__':
    main()
