#!/usr/bin/env python3
"""
bgc_connect.py
Minimal SimpleBGC UART connection test.

Sends CMD_BOARD_INFO and reports what comes back.
Tries both 57600 and 115200 baud automatically.

Usage:
    python bgc_connect.py            # defaults to COM6
    python bgc_connect.py COM3
    python bgc_connect.py COM6 57600 # force a single baud rate
"""

import sys
import time
import serial

CMD_BOARD_INFO = 0x56  # 86

def build_packet(cmd: int, body: bytes = b'') -> bytes:
    size = len(body)
    header_crc = (cmd + size) & 0xFF
    body_crc = sum(body) & 0xFF
    return bytes([0x3E, cmd, size, header_crc]) + body + bytes([body_crc])

def try_connect(port: str, baud: int) -> bool:
    """
    Open port, send CMD_BOARD_INFO, print raw bytes and parse result.
    Returns True if a valid response was received.
    """
    print(f"\n--- Trying {port} @ {baud} baud ---")
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=2.0)
    except serial.SerialException as e:
        print(f"  [FAIL] Could not open port: {e}")
        return False

    time.sleep(1.0)  # let USB-UART chip settle
    ser.reset_input_buffer()

    pkt = build_packet(CMD_BOARD_INFO)
    print(f"  TX: {pkt.hex(' ')}")
    ser.write(pkt)

    # Collect bytes for 2 seconds
    deadline = time.time() + 2.0
    buf = b''
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        else:
            time.sleep(0.02)
    ser.close()

    if not buf:
        print("  RX: (nothing)")
        return False

    print(f"  RX ({len(buf)} bytes): {buf.hex(' ')}")
    ascii_rep = ''.join(chr(b) if 32 <= b < 127 else '.' for b in buf)
    print(f"  ASCII: {ascii_rep}")

    # Try to parse a valid SimpleBGC packet
    tmp = bytearray(buf)
    while len(tmp) >= 5:
        if tmp[0] != 0x3E:
            tmp = tmp[1:]
            continue
        cmd_id, data_len, hdr_crc = tmp[1], tmp[2], tmp[3]
        if (cmd_id + data_len) & 0xFF != hdr_crc:
            tmp = tmp[1:]
            continue
        total = 4 + data_len + 1
        if len(tmp) < total:
            break
        body = bytes(tmp[4: 4 + data_len])
        if sum(body) & 0xFF != tmp[4 + data_len]:
            tmp = tmp[1:]
            continue
        # Valid packet
        print(f"  Parsed packet: cmd=0x{cmd_id:02X} ({cmd_id}), body({data_len}B): {body.hex()}")
        if cmd_id == CMD_BOARD_INFO and len(body) >= 3:
            print(f"  --> Firmware {body[2]}.{body[1]}, board type byte={body[0]}")
            print(f"  [OK] Connected at {baud} baud!")
            return True
        tmp = tmp[total:]

    print("  [WARN] Got bytes but no valid SimpleBGC packet.")
    print("         (May be wrong baud rate or board is echoing garbage)")
    return False


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM6'

    # If a specific baud was given, try only that one
    if len(sys.argv) > 2:
        baud = int(sys.argv[2])
        try_connect(port, baud)
        return

    # Auto-scan all common baud rates
    for baud in [9600, 19200, 38400, 57600, 115200, 230400, 256000]:
        if try_connect(port, baud):
            print(f"\nSuccess: use '{port}' at {baud} baud.")
            return

    print("\n[FAIL] Could not establish a connection at any baud rate.")
    print("Tips:")
    print("  - Make sure SimpleBGC GUI is closed (it holds the port)")
    print("  - Check the COM port number in Device Manager")
    print("  - Try: python bgc_connect.py COM3")

if __name__ == '__main__':
    main()
