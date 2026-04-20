#!/usr/bin/env python3
"""
bgc_pid_control.py — External closed-loop PID stabilization via MODE_SPEED

Architecture (two nested loops):
  Our Python PID (outer, 10 Hz):
    CMD_REALTIME_DATA → angle → PID → speed command → CMD_CONTROL(MODE_SPEED)
  Board's inner loop (fast, ~1 kHz):
    speed command → motor current

The position loop lives entirely in Python. The board acts as a velocity follower.
The board's PID gains must remain non-zero — they drive motor current; our loop
only supplies the rate setpoint.

═══════════════════════════════════════════════════════════════════════════
RECOMMENDED TEST SEQUENCE (run in order):
═══════════════════════════════════════════════════════════════════════════

  Step 1 — Confirm IMU byte offsets (no motors, safe):
    python3 bgc_pid_control.py /dev/ttyACM0 --validate-imu
    Tilt the gimbal by hand. Roll° should change left/right, Pitch° front/back.
    If values are swapped or backwards, adjust REALTIME_ROLL_OFFSET /
    REALTIME_PITCH_OFFSET / ANGLE_SCALE at the top of this file.

  Step 2 — Measure speed units (motors engage, gimbal will move):
    python3 bgc_pid_control.py /dev/ttyACM0 --calibrate
    Sends speed=50 for 2s, reports °/(unit·s). Update SPEED_SCALE below.
    Re-run with --calibrate --axis roll if testing roll axis.

  Step 3 — PID hold test (hold level for 30s, watch error decay):
    python3 bgc_pid_control.py /dev/ttyACM0 pitch 0 30
    Logs angle / error / speed_cmd every tick. Push gimbal by hand and
    confirm it corrects back to 0°.

  Step 4 — Step response test (validates kp, ki, kd tuning):
    python3 bgc_pid_control.py /dev/ttyACM0 pitch 0 30 --step 15
    Holds at 0° for 15s, then commands 15° step. Watch rise time and overshoot
    in the printed log to judge whether gains need adjustment.

═══════════════════════════════════════════════════════════════════════════

Usage:
    python3 bgc_pid_control.py [port] [axis] [target_deg] [hold_s] [options]

    port        serial port            default: /dev/ttyACM0
    axis        pitch | roll           default: pitch
    target_deg  target angle (°)       default: 0.0
    hold_s      run duration (s)       default: 30

Options:
    --validate-imu      print IMU angles without engaging motors
    --calibrate         measure speed-unit scale factor empirically
    --axis AXIS         axis for --calibrate (default: pitch)
    --power N           motor POWER param 0-255  (default: 100)
    --kp F              proportional gain        (default: 5.0)
    --ki F              integral gain            (default: 0.5)
    --kd F              derivative gain          (default: 0.1)
    --step DEG          inject target step-change at t=hold_s/2
"""

import sys
import time
import struct
import argparse
import serial

# ---------------------------------------------------------------------------
# Command IDs (8-bit firmware 2.2b)
# ---------------------------------------------------------------------------
CMD_BOARD_INFO    = 0x56
CMD_READ_PARAMS   = 0x52
CMD_WRITE_PARAMS  = 0x57
CMD_MOTORS_ON     = 0x4D
CMD_MOTORS_OFF    = 0x6D
CMD_CONFIRM       = 0x43   # board→host: ack
CMD_CONTROL       = 0x43   # host→board: gimbal control (same byte, opposite direction)
CMD_REALTIME_DATA = 0x44

# ---------------------------------------------------------------------------
# Params layout — firmware 2.2b 8-bit, no-payload READ_PARAMS (no PROFILE_ID prefix)
# Offsets confirmed against working bgc_servo_test.py (shifted +1 from API docs)
# ---------------------------------------------------------------------------
PARAMS_BODY_SIZE = 48
ROLL_OFFSET      = 1
PITCH_OFFSET     = 7
YAW_OFFSET       = 13
IDX_P, IDX_I, IDX_D, IDX_POWER = 0, 1, 2, 3
PWM_FREQ_OFFSET  = 47

# ---------------------------------------------------------------------------
# CMD_CONTROL modes
# ---------------------------------------------------------------------------
MODE_NO_CONTROL = 0
MODE_SPEED      = 1   # ang field = speed command (int16); spd field = accel limit
MODE_ANGLE      = 2
ACCEL_LIMIT     = 100  # acceleration limit passed in spd field for MODE_SPEED
MAX_SPEED       = 200  # output clamp — tighten after calibration

# ---------------------------------------------------------------------------
# REALTIME_DATA angle fields
# Byte offsets in response body — validated by gimbal_raw_reader.py
# NOTE: verify with --validate-imu before first run
# ---------------------------------------------------------------------------
REALTIME_ROLL_OFFSET  = 0   # bytes 0-1, int16
REALTIME_PITCH_OFFSET = 6   # bytes 6-7, int16
ANGLE_SCALE = 0.18          # degrees per raw unit (project CLAUDE.md)

# ---------------------------------------------------------------------------
# Speed scale: degrees per (speed_unit · second)
# Placeholder — run --calibrate and update this value before tuning PID gains.
# ---------------------------------------------------------------------------
SPEED_SCALE = 0.10

BAUD        = 115200
TICK_HZ     = 10
TICK_PERIOD = 1.0 / TICK_HZ

# ---------------------------------------------------------------------------
# Packet helpers  (identical pattern to bgc_servo_test.py)
# ---------------------------------------------------------------------------

def build_packet(cmd: int, body: bytes = b'') -> bytes:
    size    = len(body)
    hdr_crc = (cmd + size) & 0xFF
    bdy_crc = sum(body) & 0xFF
    return bytes([0x3E, cmd, size, hdr_crc]) + body + bytes([bdy_crc])

def read_raw_byte(ser: serial.Serial, expected: int, timeout: float = 1.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            if ser.read(1)[0] == expected:
                return True
        else:
            time.sleep(0.02)
    return False

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
# IMU read
# ---------------------------------------------------------------------------

def read_imu_angle(ser: serial.Serial, axis_offset: int) -> float | None:
    """Request REALTIME_DATA, return axis angle in degrees, or None on timeout."""
    ser.reset_input_buffer()
    ser.write(build_packet(CMD_REALTIME_DATA))
    body = read_response(ser, CMD_REALTIME_DATA, timeout=1.0)
    if body is None or len(body) < axis_offset + 2:
        return None
    raw = struct.unpack_from('<h', body, axis_offset)[0]
    return raw * ANGLE_SCALE

# ---------------------------------------------------------------------------
# CMD_CONTROL — MODE_SPEED
# ---------------------------------------------------------------------------

def send_speed(ser: serial.Serial, axis: str, speed_cmd: float) -> None:
    """Send a signed speed command on the selected axis; other axes get NO_CONTROL."""
    speed_int = int(max(-MAX_SPEED, min(MAX_SPEED, speed_cmd)))

    def ax(mode, spd, ang):
        return struct.pack('<BHh', mode, spd, ang)

    if axis == 'roll':
        body = (ax(MODE_SPEED,      ACCEL_LIMIT, speed_int) +
                ax(MODE_NO_CONTROL, 0,           0        ) +
                ax(MODE_NO_CONTROL, 0,           0        ))
    else:  # pitch
        body = (ax(MODE_NO_CONTROL, 0,           0        ) +
                ax(MODE_SPEED,      ACCEL_LIMIT, speed_int) +
                ax(MODE_NO_CONTROL, 0,           0        ))

    ser.write(build_packet(CMD_CONTROL, body))

# ---------------------------------------------------------------------------
# Board lifecycle helpers
# ---------------------------------------------------------------------------

def connect(port: str) -> serial.Serial:
    print(f"Opening {port} @ {BAUD} baud...")
    try:
        ser = serial.Serial(port, baudrate=BAUD, timeout=2.0, dsrdtr=False, rtscts=False)
    except serial.SerialException as e:
        print(f"[ERROR] {e}"); sys.exit(1)
    time.sleep(1.0)
    ser.reset_input_buffer()
    return ser

def handshake(ser: serial.Serial) -> None:
    print("\n[1] CMD_BOARD_INFO...")
    ser.write(build_packet(CMD_BOARD_INFO))
    time.sleep(0.02)
    body = read_response(ser, CMD_BOARD_INFO)
    if body is None:
        print("  [FAIL] No response."); ser.close(); sys.exit(1)
    if len(body) >= 3:
        print(f"  OK  firmware={body[2]}.{body[1]}  board_type={body[0]}")
    else:
        print(f"  OK  ({len(body)} bytes): {body.hex()}")

def read_params(ser: serial.Serial) -> bytearray:
    print("\n[2] CMD_READ_PARAMS...")
    ser.reset_input_buffer()
    ser.write(build_packet(CMD_READ_PARAMS))
    time.sleep(0.02)
    body = read_response(ser, CMD_READ_PARAMS, timeout=3.0)
    if body is None:
        print("  [FAIL] No response."); ser.close(); sys.exit(1)
    if len(body) < PARAMS_BODY_SIZE:
        print(f"  [FAIL] Expected >={PARAMS_BODY_SIZE} bytes, got {len(body)}")
        ser.close(); sys.exit(1)
    params = bytearray(body)
    print(f"  OK  ({len(body)} bytes)")
    for name, off in [("ROLL", ROLL_OFFSET), ("PITCH", PITCH_OFFSET), ("YAW", YAW_OFFSET)]:
        print(f"  {name}: P={params[off+IDX_P]}  I={params[off+IDX_I]}  "
              f"D={params[off+IDX_D]}  POWER={params[off+IDX_POWER]}")
    return params

def arm(ser: serial.Serial, params: bytearray, axis: str, power: int) -> bytearray:
    """Write PID+POWER for the selected axis, then motors on."""
    axis_off = ROLL_OFFSET if axis == 'roll' else PITCH_OFFSET
    print(f"\n[3] Setting {axis.upper()}: P=3, I=1, D=2, POWER={power}  PWM_FREQ=HIGH")
    params[axis_off + IDX_P]     = 3
    params[axis_off + IDX_I]     = 1
    params[axis_off + IDX_D]     = 2
    params[axis_off + IDX_POWER] = power
    params[PWM_FREQ_OFFSET]      = 1

    print("\n[4] CMD_WRITE_PARAMS...")
    ser.reset_input_buffer()
    ser.write(build_packet(CMD_WRITE_PARAMS, bytes(params)))
    time.sleep(0.02)
    confirm = read_response(ser, CMD_CONFIRM, timeout=2.0)
    print("  CMD_CONFIRM received." if confirm else "  [WARN] No CMD_CONFIRM — continuing.")

    print("\n[5] CMD_MOTORS_ON...")
    ser.write(build_packet(CMD_MOTORS_ON))
    if read_raw_byte(ser, ord('M'), timeout=1.0):
        print("  'M' received — motors armed.")
    else:
        print("  No 'M' — board may still respond to CMD_CONTROL.")
    time.sleep(0.5)
    return params

def shutdown(ser: serial.Serial, params: bytearray) -> None:
    """Stop CMD_CONTROL, wait for watchdog, write POWER=0 on all axes."""
    print("\n[SHUTDOWN] Waiting 2s for board watchdog to release serial control...")
    time.sleep(2.0)
    for off in [ROLL_OFFSET, PITCH_OFFSET, YAW_OFFSET]:
        params[off + IDX_P]     = 0
        params[off + IDX_I]     = 0
        params[off + IDX_D]     = 0
        params[off + IDX_POWER] = 0
    ser.reset_input_buffer()
    ser.write(build_packet(CMD_WRITE_PARAMS, bytes(params)))
    confirm = read_response(ser, CMD_CONFIRM, timeout=1.5)
    if confirm:
        print("  CMD_CONFIRM — POWER=0 applied, motors off.")
    else:
        print("  [WARN] No CMD_CONFIRM for shutdown WRITE_PARAMS.")
    ser.close()
    print("Done.")

# ---------------------------------------------------------------------------
# TEST 1 — IMU Validation  (no motors engaged)
# ---------------------------------------------------------------------------

def validate_imu(ser: serial.Serial, duration: float = 8.0) -> None:
    """
    Print live IMU angles without engaging motors.
    Tilt the gimbal by hand to confirm REALTIME_ROLL_OFFSET, REALTIME_PITCH_OFFSET,
    and ANGLE_SCALE are correct before running any motor test.
    """
    print(f"\n{'='*65}")
    print("TEST 1 — IMU VALIDATION  (motors NOT engaged)")
    print(f"{'='*65}")
    print("Tilt the gimbal by hand:")
    print("  Roll  should change when you tilt LEFT / RIGHT")
    print("  Pitch should change when you tilt FORWARD / BACK")
    print(f"\n  If values are swapped or zero, update the REALTIME_*_OFFSET")
    print(f"  constants at the top of this file.")
    print(f"\n  Reading for {duration:.0f}s — press Ctrl+C to stop early.")
    print(f"{'='*65}")
    print(f"  {'t(s)':>6}  {'Roll(°)':>9}  {'Pitch(°)':>9}  "
          f"{'Roll raw':>9}  {'Pitch raw':>9}  {'Body len':>8}")
    print(f"  {'-'*60}")

    deadline = time.time() + duration
    try:
        while time.time() < deadline:
            ser.reset_input_buffer()
            ser.write(build_packet(CMD_REALTIME_DATA))
            body = read_response(ser, CMD_REALTIME_DATA, timeout=1.0)
            if body and len(body) >= 8:
                roll_raw  = struct.unpack_from('<h', body, REALTIME_ROLL_OFFSET)[0]
                pitch_raw = struct.unpack_from('<h', body, REALTIME_PITCH_OFFSET)[0]
                print(f"  {time.time() % 100:6.2f}  "
                      f"{roll_raw * ANGLE_SCALE:9.2f}  "
                      f"{pitch_raw * ANGLE_SCALE:9.2f}  "
                      f"{roll_raw:9d}  {pitch_raw:9d}  {len(body):8d}")
            else:
                print("  [WARN] No REALTIME_DATA response — check connection")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n  Stopped early.")

    print(f"\n  Current offsets: ROLL={REALTIME_ROLL_OFFSET}, "
          f"PITCH={REALTIME_PITCH_OFFSET}, SCALE={ANGLE_SCALE}°/unit")

# ---------------------------------------------------------------------------
# TEST 2 — Speed Calibration
# ---------------------------------------------------------------------------

def calibrate_speed(ser: serial.Serial, params: bytearray,
                    axis: str, power: int,
                    test_speed: int = 50, test_duration: float = 2.0) -> None:
    """
    Engage motors, send a fixed speed command for test_duration seconds,
    measure how many degrees the gimbal moved, and report SPEED_SCALE.

    Update SPEED_SCALE at the top of this file with the reported value
    before tuning PID gains — without it kp/ki/kd are in arbitrary units.
    """
    axis_offset = REALTIME_ROLL_OFFSET if axis == 'roll' else REALTIME_PITCH_OFFSET

    print(f"\n{'='*65}")
    print("TEST 2 — SPEED CALIBRATION  (gimbal will move)")
    print(f"{'='*65}")
    print(f"  Axis: {axis}  Speed command: {test_speed} units  Duration: {test_duration:.1f}s")
    print(f"  The gimbal will move under power. Press Ctrl+C to abort.\n")

    params = arm(ser, params, axis, power)
    time.sleep(1.0)

    angle_start = read_imu_angle(ser, axis_offset)
    if angle_start is None:
        print("  [FAIL] Cannot read start angle."); shutdown(ser, params); return

    print(f"\n  Start angle : {angle_start:.2f}°")
    print(f"  Sending speed={test_speed} for {test_duration:.1f}s ...")

    t_start = time.time()
    try:
        while time.time() - t_start < test_duration:
            send_speed(ser, axis, test_speed)
            time.sleep(TICK_PERIOD)
    except KeyboardInterrupt:
        print("  Aborted early.")

    elapsed     = time.time() - t_start
    angle_end   = read_imu_angle(ser, axis_offset)
    if angle_end is None:
        print("  [FAIL] Cannot read end angle."); shutdown(ser, params); return

    delta = angle_end - angle_start
    scale = delta / (test_speed * elapsed) if (test_speed * elapsed) != 0 else 0.0

    print(f"  End angle   : {angle_end:.2f}°")
    print(f"  Delta       : {delta:+.2f}°  over {elapsed:.2f}s")
    print()
    print(f"  ┌──────────────────────────────────────────────────────┐")
    print(f"  │  SPEED_SCALE = {scale:.5f}  °/(unit·s)                │")
    print(f"  │  speed=50  ≈ { 50*scale:6.2f} °/s                           │")
    print(f"  │  speed=100 ≈ {100*scale:6.2f} °/s                           │")
    print(f"  │  speed=200 ≈ {200*scale:6.2f} °/s                           │")
    print(f"  │                                                      │")
    print(f"  │  Update SPEED_SCALE at the top of bgc_pid_control.py │")
    print(f"  └──────────────────────────────────────────────────────┘")

    if delta < 0:
        print(f"\n  NOTE: negative delta means speed={test_speed} moves {axis} in the")
        print(f"        negative direction. Negate test_speed or flip sign in send_speed.")

    shutdown(ser, params)

# ---------------------------------------------------------------------------
# TEST 3 / Normal — PID control loop
# ---------------------------------------------------------------------------

def run_pid(ser: serial.Serial, params: bytearray,
            axis: str, target_angle: float, hold_seconds: float,
            kp: float, ki: float, kd: float, power: int,
            step_angle: float | None = None) -> None:
    """
    Outer PID loop: reads angle via CMD_REALTIME_DATA, outputs speed command
    via CMD_CONTROL(MODE_SPEED).

    Logs every tick (10 Hz). If --step DEG is given, changes target at
    hold_seconds/2 to test step response (rise time, overshoot, settling).

    Gain tuning guide:
      kp  — start at 2-5; increase until responsive, back off before oscillation
      ki  — start at 0; add slowly to eliminate steady-state offset
      kd  — start at 0; add if overshoot is large; too much → jitter
    """
    axis_offset    = REALTIME_ROLL_OFFSET if axis == 'roll' else REALTIME_PITCH_OFFSET
    INTEGRAL_LIMIT = MAX_SPEED * 2.0

    params = arm(ser, params, axis, power)

    integral      = 0.0
    prev_error    = 0.0
    prev_time     = time.time()
    current_target = target_angle
    step_fired    = False

    print(f"\n{'='*70}")
    print(f"PID LOOP  axis={axis}  target={target_angle}°  "
          f"hold={hold_seconds:.0f}s  POWER={power}")
    print(f"Gains: kp={kp}  ki={ki}  kd={kd}")
    if step_angle is not None:
        print(f"Step test: target → {step_angle}° at t={hold_seconds/2:.0f}s")
    print(f"SPEED_SCALE = {SPEED_SCALE} °/(unit·s)  "
          f"(update after --calibrate if still placeholder)")
    print(f"{'='*70}")
    print(f"  {'t(s)':>6}  {'angle(°)':>9}  {'target(°)':>9}  "
          f"{'error(°)':>8}  {'speed':>7}  {'integral':>9}")
    print(f"  {'-'*60}")

    deadline = time.time() + hold_seconds

    try:
        while time.time() < deadline:
            t_now = time.time()
            dt    = max(t_now - prev_time, 1e-6)
            prev_time = t_now

            # Inject step change at halfway point
            if (step_angle is not None and not step_fired
                    and (deadline - t_now) <= hold_seconds / 2):
                current_target = step_angle
                step_fired = True
                print(f"\n  *** STEP: target → {step_angle}° ***\n")

            # 1. Read IMU
            angle = read_imu_angle(ser, axis_offset)
            if angle is None:
                print("  [WARN] IMU read failed — skipping tick")
                time.sleep(TICK_PERIOD)
                continue

            # 2. PID  (output = speed command in board speed units)
            error      = current_target - angle
            integral   = max(-INTEGRAL_LIMIT,
                             min(INTEGRAL_LIMIT, integral + error * dt))
            derivative = (error - prev_error) / dt
            prev_error = error

            speed_cmd = kp * error + ki * integral + kd * derivative

            # 3. Send speed command to board
            send_speed(ser, axis, speed_cmd)

            print(f"  {t_now % 100:6.2f}  {angle:9.2f}  {current_target:9.2f}  "
                  f"{error:+8.2f}  {speed_cmd:+7.1f}  {integral:+9.2f}")

            remaining = TICK_PERIOD - (time.time() - t_now)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\n  Interrupted.")

    shutdown(ser, params)

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="External PID gimbal control via CMD_CONTROL MODE_SPEED",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('port',   nargs='?', default='/dev/ttyACM0')
    parser.add_argument('axis',   nargs='?', default='pitch', choices=['pitch', 'roll'])
    parser.add_argument('target', nargs='?', type=float, default=0.0,
                        help='Target angle in degrees')
    parser.add_argument('hold',   nargs='?', type=float, default=30.0,
                        help='Run duration in seconds')

    parser.add_argument('--validate-imu', action='store_true',
                        help='Print IMU angles without engaging motors (run first)')
    parser.add_argument('--calibrate',    action='store_true',
                        help='Measure speed-unit scale (run before PID)')
    parser.add_argument('--axis',  dest='cal_axis', default=None,
                        choices=['pitch', 'roll'],
                        help='Axis for --calibrate (overrides positional axis)')

    parser.add_argument('--power', type=int,   default=100)
    parser.add_argument('--kp',    type=float, default=5.0)
    parser.add_argument('--ki',    type=float, default=0.5)
    parser.add_argument('--kd',    type=float, default=0.1)
    parser.add_argument('--step',  type=float, default=None,
                        metavar='DEG',
                        help='Step response: change target to DEG at t=hold/2')

    args = parser.parse_args()

    ser = connect(args.port)
    handshake(ser)

    # TEST 1 — IMU validation (no motors needed)
    if args.validate_imu:
        validate_imu(ser)
        ser.close()
        return

    params = read_params(ser)

    # TEST 2 — speed calibration
    if args.calibrate:
        cal_axis = args.cal_axis or args.axis
        calibrate_speed(ser, params, cal_axis, args.power)
        return

    # TEST 3 / Normal — PID loop
    run_pid(ser, params, args.axis, args.target, args.hold,
            args.kp, args.ki, args.kd, args.power,
            step_angle=args.step)


if __name__ == '__main__':
    main()
