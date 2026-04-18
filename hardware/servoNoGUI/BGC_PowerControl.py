#!/usr/bin/env python3
"""
BGC_PowerControl.py
SimpleBGC 8-bit Serial API controller.

Goal: zero PID gains so POWER is the only motor driver, hold pitch at 30°,
step through POWER levels [50, 100, 150, 200] with 5-second holds each.

Hardware: BaseCam SimpleBGC 8-bit, PC via Mini-B USB (onboard UART chip)

Usage:
  Windows:  python BGC_PowerControl.py COM6
  WSL2:     python BGC_PowerControl.py /dev/ttyS6
            (COM6 maps to /dev/ttyS6 in WSL2 — you may need to enable it:
             sudo chmod a+rw /dev/ttyS6)

IMPORTANT NOTE ON PID=0 AND POWER:
  The POWER field caps the maximum PID output, it is NOT a direct motor drive
  value. If P=I=D=0, the PID output is always zero and no current flows to the
  motors regardless of POWER. If CMD_CONTROL does not move the motor, try
  setting P=40, D=10 and varying POWER instead of zeroing all gains.
  That said, we implement the zeroed-PID approach first and will debug from there.

Expected failure points:
  1. Port open fails      → wrong port name or permissions
  2. CMD_BOARD_INFO times out → baud rate mismatch (try 115200 or 115200)
  3. CMD_READ_PARAMS body wrong size → firmware version mismatch (try 64 or 48 bytes)
  4. CMD_CONTROL does nothing → PID=0 issue above, or angle encoding wrong
"""

import serial
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Command IDs (8-bit firmware 2.3 / 2.4)
# ---------------------------------------------------------------------------
CMD_BOARD_INFO   = 86   # 0x56
CMD_READ_PARAMS  = 82   # 0x52
CMD_WRITE_PARAMS = 87   # 0x57
CMD_MOTORS_ON    = 77   # 0x4D
CMD_MOTORS_OFF   = 109  # 0x6D
CMD_CONTROL      = 67   # 0x43

# ---------------------------------------------------------------------------
# CMD_CONTROL modes
# ---------------------------------------------------------------------------
MODE_NO_CONTROL = 0
MODE_SPEED      = 1
MODE_ANGLE      = 2

# ---------------------------------------------------------------------------
# PARAMS body layout (63 bytes total)
# [P, I, D, POWER, INVERT, POLES] × 3 axes
# ---------------------------------------------------------------------------
PARAMS_BODY_SIZE = 63

ROLL_OFFSET  = 0
PITCH_OFFSET = 6
YAW_OFFSET   = 12

IDX_P      = 0
IDX_I      = 1
IDX_D      = 2
IDX_POWER  = 3
IDX_INVERT = 4
IDX_POLES  = 5

# ---------------------------------------------------------------------------
# Experiment config
# ---------------------------------------------------------------------------
PITCH_TARGET_DEG = 30.0
POWER_STEPS      = [50, 100, 150, 200]
STEP_DURATION_S  = 5.0
CONTROL_RATE_HZ  = 10
CONTROL_SPEED    = 200   # slew rate toward target angle (arbitrary units)


# ---------------------------------------------------------------------------
# Packet helpers
# ---------------------------------------------------------------------------

def build_packet(cmd: int, body: bytes = b'') -> bytes:
    """
    Packet format: 0x3E | CMD | SIZE | (CMD+SIZE)%256 | [body] | sum(body)%256
    """
    size = len(body)
    header_crc = (cmd + size) & 0xFF
    body_crc = sum(body) & 0xFF
    return bytes([0x3E, cmd, size, header_crc]) + body + bytes([body_crc])


def read_response_from_bytes(buf: bytes, expected_cmd: int) -> bytes | None:
    """Parse a response packet from an already-received byte buffer (no I/O)."""
    buf = bytearray(buf)
    while len(buf) >= 5:
        if buf[0] != 0x3E:
            buf = buf[1:]
            continue
        cmd_id, data_len, hdr_crc = buf[1], buf[2], buf[3]
        if (cmd_id + data_len) & 0xFF != hdr_crc:
            buf = buf[1:]
            continue
        total_needed = 4 + data_len + 1
        if len(buf) < total_needed:
            break
        body = bytes(buf[4: 4 + data_len])
        body_crc = buf[4 + data_len]
        if sum(body) & 0xFF != body_crc:
            buf = buf[1:]
            continue
        if cmd_id == expected_cmd:
            return body
        buf = buf[total_needed:]
    return None


def read_response(ser: serial.Serial, expected_cmd: int, timeout: float = 2.0) -> bytes | None:
    """
    Scan incoming bytes for a valid response packet matching expected_cmd.
    Returns body bytes on success, None on timeout or error.
    """
    deadline = time.time() + timeout
    buf = b''

    while time.time() < deadline:
        waiting = ser.in_waiting
        if waiting:
            buf += ser.read(waiting)
        else:
            time.sleep(0.01)

        # Scan for a valid packet start
        while len(buf) >= 4:
            if buf[0] != 0x3E:
                buf = buf[1:]
                continue

            cmd_id   = buf[1]
            data_len = buf[2]
            hdr_crc  = buf[3]

            if (cmd_id + data_len) & 0xFF != hdr_crc:
                buf = buf[1:]
                continue

            # Valid header found — wait for full body + body_crc byte
            total_needed = 4 + data_len + 1
            if len(buf) < total_needed:
                break  # need more bytes, keep reading

            body     = buf[4: 4 + data_len]
            body_crc = buf[4 + data_len]

            if sum(body) & 0xFF != body_crc:
                print(f"[WARN] Body CRC mismatch on cmd=0x{cmd_id:02X} — skipping")
                buf = buf[1:]
                continue

            # Consume this packet from buffer
            buf = buf[total_needed:]

            if cmd_id != expected_cmd:
                print(f"[INFO] Unexpected cmd=0x{cmd_id:02X} (wanted 0x{expected_cmd:02X}), ignoring")
                continue

            return body

    return None  # timed out


def drain(ser: serial.Serial, delay: float = 0.1):
    """Discard any pending bytes from the receive buffer."""
    time.sleep(delay)
    ser.reset_input_buffer()


def _collect_raw(ser: serial.Serial, duration: float) -> bytes:
    """Read all available bytes for `duration` seconds."""
    deadline = time.time() + duration
    buf = b''
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        else:
            time.sleep(0.02)
    return buf


def _dump_raw(buf: bytes):
    if buf:
        print(f"Received {len(buf)} bytes:")
        print(" ".join(f"{b:02X}" for b in buf))
        ascii_rep = ''.join(chr(b) if 32 <= b < 127 else '.' for b in buf)
        print(f"ASCII: {ascii_rep}")
        # Try to parse any valid SimpleBGC packets in the buffer
        tmp = bytearray(buf)
        found = []
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
            if sum(body) & 0xFF == tmp[4 + data_len]:
                found.append((cmd_id, body))
                tmp = tmp[total:]
            else:
                tmp = tmp[1:]
        if found:
            print(f"\nParsed {len(found)} valid SimpleBGC packet(s):")
            for cmd_id, body in found:
                print(f"  cmd=0x{cmd_id:02X} ({cmd_id})  body({len(body)}B): {body.hex()}")
        else:
            print("\nNo valid SimpleBGC packets found in received bytes.")
            print("Likely wrong baud rate or different protocol.")
    else:
        print("Received NOTHING.")
        print("Possible causes:")
        print("  - SimpleBGC GUI is open on Windows (close it)")
        print("  - Board is off or not responding")
        print("  - Wrong baud rate")


def sniff_raw(ser: serial.Serial, duration: float = 4.0):
    """
    Send CMD_BOARD_INFO then dump every raw byte received for `duration` seconds.
    Use --sniff to diagnose what the board is actually sending back.
    """
    pkt = build_packet(CMD_BOARD_INFO)
    print(f"Sending CMD_BOARD_INFO: {pkt.hex(' ')}")
    ser.write(pkt)
    print(f"Listening for {duration}s...\n")
    buf = _collect_raw(ser, duration)
    _dump_raw(buf)


def listen_passive(ser: serial.Serial, duration: float = 5.0):
    """
    --listen: do NOT send anything; just capture whatever the board broadcasts.
    Useful to catch power-on telemetry or unsolicited status packets.
    """
    print(f"Passive listen for {duration}s (no TX)...\n")
    buf = _collect_raw(ser, duration)
    _dump_raw(buf)


def baud_scan(port: str, duration: float = 2.0):
    """
    --scan: open the port at each common baud rate, send CMD_BOARD_INFO,
    and report which rate(s) produce a valid response.
    """
    rates = [9600, 19200, 38400, 57600, 115200, 230400, 256000]
    print(f"Scanning {len(rates)} baud rates on {port}...\n")
    for baud in rates:
        print(f"  [{baud:>6}] ", end='', flush=True)
        try:
            ser = serial.Serial(port, baudrate=baud, timeout=1.0)
        except serial.SerialException as e:
            print(f"open failed: {e}")
            continue
        time.sleep(1.0)
        ser.reset_input_buffer()
        ser.write(build_packet(CMD_BOARD_INFO))
        buf = _collect_raw(ser, duration)
        ser.close()
        if not buf:
            print("no response")
            continue
        body = read_response_from_bytes(buf, CMD_BOARD_INFO)
        if body is not None:
            print(f"VALID RESPONSE  body({len(body)}B): {body.hex()}")
            if len(body) >= 3:
                print(f"           --> Firmware {body[2]}.{body[1]}, board byte={body[0]}")
        else:
            raw_hex = " ".join(f"{b:02X}" for b in buf[:24])
            print(f"got {len(buf)} bytes but no valid packet  [{raw_hex}{'...' if len(buf) > 24 else ''}]")
    print("\nScan complete.")


def probe_commands(ser: serial.Serial):
    """
    --probe: fire a range of command IDs at the current baud rate and report
    any valid responses. Covers the full 8-bit SimpleBGC command space.
    """
    KNOWN = {
        67:  "CMD_CONTROL",
        77:  "CMD_MOTORS_ON",
        82:  "CMD_READ_PARAMS",
        86:  "CMD_BOARD_INFO",
        87:  "CMD_WRITE_PARAMS",
        109: "CMD_MOTORS_OFF",
        20:  "CMD_GET_ANGLES",
        73:  "CMD_REALTIME_DATA",
        104: "CMD_BOARD_INFO_3",
    }
    candidates = sorted(set(list(KNOWN.keys()) + list(range(0x40, 0x80))))
    print(f"Probing {len(candidates)} command IDs...\n")
    for cmd_id in candidates:
        drain(ser, delay=0.05)
        ser.write(build_packet(cmd_id))
        time.sleep(0.3)
        raw = ser.read(ser.in_waiting) if ser.in_waiting else b''
        if not raw:
            continue
        label = KNOWN.get(cmd_id, "")
        body = read_response_from_bytes(raw, cmd_id)
        if body is not None:
            print(f"  cmd=0x{cmd_id:02X} ({cmd_id:3d}) {label:<25} RESPONSE  body({len(body)}B): {body.hex()}")
        else:
            raw_hex = " ".join(f"{b:02X}" for b in raw[:16])
            print(f"  cmd=0x{cmd_id:02X} ({cmd_id:3d}) {label:<25} got {len(raw)}B no valid pkt  [{raw_hex}]")
    print("\nProbe complete.")


# ---------------------------------------------------------------------------
# High-level commands
# ---------------------------------------------------------------------------

def cmd_board_info(ser: serial.Serial) -> bool:
    print("Sending CMD_BOARD_INFO (handshake)...")
    drain(ser)
    pkt = build_packet(CMD_BOARD_INFO)
    print(f"  TX: {pkt.hex(' ')}")
    ser.write(pkt)
    # Grab a raw snapshot first so we can show it on failure
    time.sleep(0.5)
    raw = ser.read(ser.in_waiting) if ser.in_waiting else b''
    if raw:
        print(f"  RX raw: {raw.hex(' ')}")
    # Feed raw into a fresh response parse
    ser.reset_input_buffer()
    body = read_response_from_bytes(raw, CMD_BOARD_INFO)
    if body is None and raw:
        # Try reading more
        body = read_response(ser, CMD_BOARD_INFO, timeout=2.5)
    elif body is None:
        body = read_response(ser, CMD_BOARD_INFO, timeout=2.5)
    if body is None:
        if not raw:
            print("[ERROR] No bytes received at all.")
            print("        → Close SimpleBGC GUI if open on Windows.")
            print("        → Try: python3 BGC_PowerControl.py /dev/ttyACM0 115200 --sniff")
        else:
            print("[ERROR] Bytes received but could not parse a valid packet.")
            print("        → Run with --sniff to see raw output.")
        return False
    if len(body) >= 3:
        fw_minor = body[1]
        fw_major = body[2]
        print(f"  Board version byte: {body[0]}, Firmware: {fw_major}.{fw_minor}")
    else:
        print(f"  Board info ({len(body)} bytes): {body.hex()}")
    return True


def read_params(ser: serial.Serial) -> bytearray | None:
    print("Sending CMD_READ_PARAMS...")
    drain(ser)
    ser.write(build_packet(CMD_READ_PARAMS))
    body = read_response(ser, CMD_READ_PARAMS, timeout=3.0)
    if body is None:
        print("[ERROR] No response to CMD_READ_PARAMS.")
        return None
    if len(body) < PARAMS_BODY_SIZE:
        print(f"[ERROR] Params body is {len(body)} bytes, expected {PARAMS_BODY_SIZE}.")
        print(f"        Raw hex: {body.hex()}")
        print("        Try adjusting PARAMS_BODY_SIZE to match your firmware version.")
        return None
    print(f"  Read {len(body)} param bytes OK.")
    return bytearray(body)


def write_params(ser: serial.Serial, params: bytearray) -> None:
    print("Sending CMD_WRITE_PARAMS...")
    ser.write(build_packet(CMD_WRITE_PARAMS, bytes(params)))
    # 8-bit firmware may or may not send a confirmation — don't block on it
    time.sleep(0.3)
    drain(ser, delay=0.05)


def motors_on(ser: serial.Serial) -> None:
    print("CMD_MOTORS_ON")
    ser.write(build_packet(CMD_MOTORS_ON))
    time.sleep(0.5)


def motors_off(ser: serial.Serial) -> None:
    print("CMD_MOTORS_OFF")
    ser.write(build_packet(CMD_MOTORS_OFF))


def send_control(ser: serial.Serial, pitch_deg: float, speed: int = CONTROL_SPEED) -> None:
    """
    CMD_CONTROL body: 3 × [MODE(1), SPEED(2 uint16), ANGLE(2 int16)]
    for axes ROLL / PITCH / YAW.
    Angle units: degrees × 10  →  30° = 300
    ROLL and YAW left in MODE_NO_CONTROL (0) so only pitch is driven.
    """
    angle_units = int(pitch_deg * 10)

    def axis_bytes(mode, spd, ang):
        return struct.pack('<BHh', mode, spd, ang)

    body = (
        axis_bytes(MODE_NO_CONTROL, 0, 0) +           # ROLL  — ignore
        axis_bytes(MODE_ANGLE, speed, angle_units) +  # PITCH — hold target
        axis_bytes(MODE_NO_CONTROL, 0, 0)             # YAW   — ignore
    )
    ser.write(build_packet(CMD_CONTROL, body))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM6'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 57600

    sniff   = '--sniff'   in sys.argv
    listen  = '--listen'  in sys.argv
    scan    = '--scan'    in sys.argv
    probe   = '--probe'   in sys.argv

    # --scan opens its own serial ports per baud rate; no shared ser needed
    if scan:
        baud_scan(port)
        return

    print(f"Opening {port} at {baud} baud...")
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"[ERROR] Could not open port: {e}")
        sys.exit(1)

    time.sleep(2.0)  # let USB-UART chip settle after open

    if sniff:
        sniff_raw(ser)
        ser.close()
        return

    if listen:
        listen_passive(ser)
        ser.close()
        return

    if probe:
        probe_commands(ser)
        ser.close()
        return

    try:
        # --- 1. Handshake ---
        if not cmd_board_info(ser):
            return

        # --- 2. Read current params ---
        params = read_params(ser)
        if params is None:
            return

        # --- 3. Print current axis config ---
        print("\nCurrent axis params (before modification):")
        for name, off in [("ROLL", ROLL_OFFSET), ("PITCH", PITCH_OFFSET), ("YAW", YAW_OFFSET)]:
            p     = params[off + IDX_P]
            i_val = params[off + IDX_I]
            d     = params[off + IDX_D]
            pwr   = params[off + IDX_POWER]
            print(f"  {name:5s}: P={p:3d}  I={i_val:3d}  D={d:3d}  POWER={pwr:3d}")

        # --- 4. Zero PID, set initial power ---
        print(f"\nZeroing PID on all axes, setting POWER={POWER_STEPS[0]}...")
        for off in [ROLL_OFFSET, PITCH_OFFSET, YAW_OFFSET]:
            params[off + IDX_P] = 0
            params[off + IDX_I] = 0
            params[off + IDX_D] = 0
            params[off + IDX_POWER] = POWER_STEPS[0]

        write_params(ser, params)

        # --- 5. Motors on ---
        motors_on(ser)

        # --- 6. Step through power levels ---
        interval = 1.0 / CONTROL_RATE_HZ

        for power in POWER_STEPS:
            print(f"\n--- POWER = {power} / 255  ({STEP_DURATION_S:.0f}s) ---")
            for off in [ROLL_OFFSET, PITCH_OFFSET, YAW_OFFSET]:
                params[off + IDX_POWER] = power
            write_params(ser, params)

            step_end = time.time() + STEP_DURATION_S
            tick = 0
            while time.time() < step_end:
                t0 = time.time()
                send_control(ser, PITCH_TARGET_DEG)
                tick += 1
                if tick % CONTROL_RATE_HZ == 0:
                    print(f"  Holding pitch={PITCH_TARGET_DEG}° @ POWER={power}  "
                          f"(t={STEP_DURATION_S - (step_end - time.time()):.1f}s)")
                elapsed = time.time() - t0
                sleep_for = interval - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)

        print("\nDone. Turning motors off.")

    except KeyboardInterrupt:
        print("\n[Ctrl+C] Interrupted.")

    finally:
        motors_off(ser)
        time.sleep(0.3)
        ser.close()
        print("Port closed.")


if __name__ == '__main__':
    main()
