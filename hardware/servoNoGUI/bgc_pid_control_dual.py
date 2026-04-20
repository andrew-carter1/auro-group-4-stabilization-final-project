#!/usr/bin/env python3
"""
bgc_pid_control_dual.py — External closed-loop PID stabilization for BOTH ROLL and PITCH

Architecture (two nested loops, one per axis):
  Our Python PID (outer, 10 Hz):
    CMD_REALTIME_DATA → roll_angle, pitch_angle
    → roll PID  → roll_speed_cmd  ─┐
    → pitch PID → pitch_speed_cmd ─┴→ CMD_CONTROL(MODE_SPEED) [single packet, both axes]
  Board's inner loop (fast, ~1 kHz):
    speed commands → motor currents

Both axes share one set of PID gains (kp/ki/kd). The board's PID gains must remain
non-zero — they drive motor current; our loop only supplies the rate setpoints.

═══════════════════════════════════════════════════════════════════════════
RECOMMENDED TEST SEQUENCE (run in order):
═══════════════════════════════════════════════════════════════════════════

  Step 1 — Confirm IMU byte offsets (no motors, safe):
    python3 bgc_pid_control_dual.py /dev/ttyACM0 --validate-imu
    Tilt the gimbal by hand. Roll° should change left/right, Pitch° front/back.

  Step 2 — Calibrate each axis independently (motors engage, gimbal will move):
    python3 bgc_pid_control_dual.py /dev/ttyACM0 --calibrate --cal-axis pitch
    python3 bgc_pid_control_dual.py /dev/ttyACM0 --calibrate --cal-axis roll
    Update SPEED_SCALE below with the reported value.

  Step 3 — Dual-axis PID hold test (both axes level for 30s):
    python3 bgc_pid_control_dual.py /dev/ttyACM0 0 0 30
    Push gimbal by hand; confirm both axes correct back to 0°.

  Step 4 — Step response test (validate tuning):
    python3 bgc_pid_control_dual.py /dev/ttyACM0 0 0 30 --step-roll 15 --step-pitch 10
    Commands a step on both axes at t=hold/2.

═══════════════════════════════════════════════════════════════════════════

Usage:
    python3 bgc_pid_control_dual.py [port] [roll_target] [pitch_target] [hold_s] [options]

    port          serial port                default: /dev/ttyACM0
    roll_target   roll target angle (°)      default: 0.0
    pitch_target  pitch target angle (°)     default: 0.0
    hold_s        run duration (s)           default: 30

Options:
    --validate-imu          print IMU angles without engaging motors
    --calibrate             measure speed-unit scale factor empirically
    --cal-axis AXIS         axis for --calibrate: pitch | roll  (default: pitch)
    --power N               motor POWER for both axes 0-255    (default: 70)
    --kp F                  proportional gain                  (default: 15.0)
    --ki F                  integral gain                      (default: 1.5)
    --kd F                  derivative gain                    (default: 5.0)
    --step-roll DEG         inject roll step-change at t=hold/2
    --step-pitch DEG        inject pitch step-change at t=hold/2
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
ACCEL_LIMIT     = 100
MAX_SPEED       = 200

# ---------------------------------------------------------------------------
# REALTIME_DATA angle field offsets — validated by gimbal_raw_reader.py
# NOTE: verify with --validate-imu before first run
# ---------------------------------------------------------------------------
REALTIME_ROLL_OFFSET  = 0   # bytes 0-1, int16
REALTIME_PITCH_OFFSET = 6   # bytes 6-7, int16
ANGLE_SCALE = 0.18          # degrees per raw unit

# ---------------------------------------------------------------------------
# Speed scale: degrees per (speed_unit · second)
# Placeholder — run --calibrate and update this value before tuning PID gains.
# ---------------------------------------------------------------------------
SPEED_SCALE = 0.10

BAUD        = 115200
TICK_HZ     = 10
TICK_PERIOD = 1.0 / TICK_HZ

# ---------------------------------------------------------------------------
# Packet helpers
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
# IMU read — returns (roll_deg, pitch_deg) or (None, None)
# ---------------------------------------------------------------------------

def read_imu_angles(ser: serial.Serial) -> tuple[float | None, float | None]:
    ser.reset_input_buffer()
    ser.write(build_packet(CMD_REALTIME_DATA))
    body = read_response(ser, CMD_REALTIME_DATA, timeout=1.0)
    if body is None or len(body) < REALTIME_PITCH_OFFSET + 2:
        return None, None
    roll_raw  = struct.unpack_from('<h', body, REALTIME_ROLL_OFFSET)[0]
    pitch_raw = struct.unpack_from('<h', body, REALTIME_PITCH_OFFSET)[0]
    return roll_raw * ANGLE_SCALE, pitch_raw * ANGLE_SCALE

# ---------------------------------------------------------------------------
# CMD_CONTROL — sends ROLL and PITCH speed commands in a single packet
# ---------------------------------------------------------------------------

def send_speeds(ser: serial.Serial, roll_speed: float, pitch_speed: float) -> None:
    """Send signed speed commands for both ROLL and PITCH simultaneously."""
    r = int(max(-MAX_SPEED, min(MAX_SPEED, roll_speed)))
    p = int(max(-MAX_SPEED, min(MAX_SPEED, pitch_speed)))

    def ax(mode, spd, ang):
        return struct.pack('<BHh', mode, spd, ang)

    body = (ax(MODE_SPEED,      ACCEL_LIMIT, r) +
            ax(MODE_SPEED,      ACCEL_LIMIT, p) +
            ax(MODE_NO_CONTROL, 0,           0))
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

def arm_dual(ser: serial.Serial, params: bytearray, power: int) -> bytearray:
    """Write PID+POWER for both ROLL and PITCH axes, then motors on."""
    print(f"\n[3] Setting ROLL and PITCH: P=10, I=15, D=10, POWER={power}  PWM_FREQ=HIGH")
    for off in [ROLL_OFFSET, PITCH_OFFSET]:
        params[off + IDX_P]     = 10
        params[off + IDX_I]     = 15
        params[off + IDX_D]     = 10
        params[off + IDX_POWER] = power
    params[PWM_FREQ_OFFSET] = 1

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
    print(f"\n{'='*65}")
    print("TEST 1 — IMU VALIDATION  (motors NOT engaged)")
    print(f"{'='*65}")
    print("Tilt the gimbal by hand:")
    print("  Roll  should change when you tilt LEFT / RIGHT")
    print("  Pitch should change when you tilt FORWARD / BACK")
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
            if body and len(body) >= REALTIME_PITCH_OFFSET + 2:
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
# TEST 2 — Speed Calibration (single axis at a time)
# ---------------------------------------------------------------------------

def calibrate_speed(ser: serial.Serial, params: bytearray,
                    axis: str, power: int,
                    test_speed: int = 50, test_duration: float = 2.0) -> None:
    rt_offset = REALTIME_ROLL_OFFSET if axis == 'roll' else REALTIME_PITCH_OFFSET

    print(f"\n{'='*65}")
    print("TEST 2 — SPEED CALIBRATION  (gimbal will move)")
    print(f"{'='*65}")
    print(f"  Axis: {axis}  Speed command: {test_speed} units  Duration: {test_duration:.1f}s")
    print(f"  The gimbal will move under power. Press Ctrl+C to abort.\n")

    params = arm_dual(ser, params, power)
    time.sleep(1.0)

    roll_start, pitch_start = read_imu_angles(ser)
    angle_start = roll_start if axis == 'roll' else pitch_start
    if angle_start is None:
        print("  [FAIL] Cannot read start angle."); shutdown(ser, params); return

    print(f"\n  Start angle : {angle_start:.2f}°")
    print(f"  Sending speed={test_speed} on {axis} for {test_duration:.1f}s ...")

    t_start = time.time()
    try:
        while time.time() - t_start < test_duration:
            if axis == 'roll':
                send_speeds(ser, test_speed, 0)
            else:
                send_speeds(ser, 0, test_speed)
            time.sleep(TICK_PERIOD)
    except KeyboardInterrupt:
        print("  Aborted early.")

    elapsed = time.time() - t_start
    roll_end, pitch_end = read_imu_angles(ser)
    angle_end = roll_end if axis == 'roll' else pitch_end
    if angle_end is None:
        print("  [FAIL] Cannot read end angle."); shutdown(ser, params); return

    delta = angle_end - angle_start
    scale = delta / (test_speed * elapsed) if (test_speed * elapsed) != 0 else 0.0

    print(f"  End angle   : {angle_end:.2f}°")
    print(f"  Delta       : {delta:+.2f}°  over {elapsed:.2f}s")
    print()
    print(f"  ┌──────────────────────────────────────────────────────────┐")
    print(f"  │  SPEED_SCALE = {scale:.5f}  °/(unit·s)                  │")
    print(f"  │  speed=50  ≈ { 50*scale:6.2f} °/s                             │")
    print(f"  │  speed=100 ≈ {100*scale:6.2f} °/s                             │")
    print(f"  │  speed=200 ≈ {200*scale:6.2f} °/s                             │")
    print(f"  │                                                          │")
    print(f"  │  Update SPEED_SCALE at the top of bgc_pid_control_dual.py│")
    print(f"  └──────────────────────────────────────────────────────────┘")

    shutdown(ser, params)

# ---------------------------------------------------------------------------
# TEST 3 / Normal — Dual-axis PID control loop
# ---------------------------------------------------------------------------

def run_pid_dual(ser: serial.Serial, params: bytearray,
                 roll_target: float, pitch_target: float, hold_seconds: float,
                 kp: float, ki: float, kd: float, power: int,
                 step_roll: float | None = None,
                 step_pitch: float | None = None) -> None:
    """
    Runs independent PID loops for ROLL and PITCH simultaneously.
    Both axes are sampled from a single CMD_REALTIME_DATA request per tick,
    and both speed commands are sent in a single CMD_CONTROL packet.
    """
    INTEGRAL_LIMIT = MAX_SPEED * 2.0

    params = arm_dual(ser, params, power)

    # Per-axis PID state
    roll_integral  = pitch_integral  = 0.0
    roll_prev_err  = pitch_prev_err  = 0.0
    roll_cur_tgt   = roll_target
    pitch_cur_tgt  = pitch_target
    step_fired     = False
    prev_time      = time.time()

    print(f"\n{'='*78}")
    print(f"DUAL-AXIS PID LOOP  roll_target={roll_target}°  pitch_target={pitch_target}°  "
          f"hold={hold_seconds:.0f}s  POWER={power}")
    print(f"Gains: kp={kp}  ki={ki}  kd={kd}")
    if step_roll is not None or step_pitch is not None:
        print(f"Step test at t={hold_seconds/2:.0f}s: "
              + (f"roll→{step_roll}°  " if step_roll is not None else "")
              + (f"pitch→{step_pitch}°" if step_pitch is not None else ""))
    print(f"SPEED_SCALE = {SPEED_SCALE} °/(unit·s)")
    print(f"{'='*78}")
    print(f"  {'t(s)':>6}  {'Roll(°)':>8}  {'RTgt(°)':>7}  {'RErr':>7}  {'RSpd':>7}"
          f"  {'Pitch(°)':>8}  {'PTgt(°)':>7}  {'PErr':>7}  {'PSpd':>7}")
    print(f"  {'-'*75}")

    deadline = time.time() + hold_seconds

    try:
        while time.time() < deadline:
            t_now = time.time()
            dt    = max(t_now - prev_time, 1e-6)
            prev_time = t_now

            # Inject step changes at the halfway point
            if (not step_fired
                    and (step_roll is not None or step_pitch is not None)
                    and (deadline - t_now) <= hold_seconds / 2):
                if step_roll  is not None: roll_cur_tgt  = step_roll
                if step_pitch is not None: pitch_cur_tgt = step_pitch
                step_fired = True
                print(f"\n  *** STEP: roll→{roll_cur_tgt}°  pitch→{pitch_cur_tgt}° ***\n")

            # 1. Read both IMU angles in one request
            roll_angle, pitch_angle = read_imu_angles(ser)
            if roll_angle is None or pitch_angle is None:
                print("  [WARN] IMU read failed — skipping tick")
                time.sleep(TICK_PERIOD)
                continue

            # 2. Roll PID
            roll_err        = roll_cur_tgt - roll_angle
            roll_integral   = max(-INTEGRAL_LIMIT,
                                  min(INTEGRAL_LIMIT, roll_integral + roll_err * dt))
            roll_deriv      = (roll_err - roll_prev_err) / dt
            roll_prev_err   = roll_err
            roll_speed      = kp * roll_err + ki * roll_integral + kd * roll_deriv

            # 3. Pitch PID
            pitch_err       = pitch_cur_tgt - pitch_angle
            pitch_integral  = max(-INTEGRAL_LIMIT,
                                  min(INTEGRAL_LIMIT, pitch_integral + pitch_err * dt))
            pitch_deriv     = (pitch_err - pitch_prev_err) / dt
            pitch_prev_err  = pitch_err
            pitch_speed     = kp * pitch_err + ki * pitch_integral + kd * pitch_deriv

            # 4. Send both speed commands in one packet
            send_speeds(ser, roll_speed, pitch_speed)

            print(f"  {t_now % 100:6.2f}"
                  f"  {roll_angle:8.2f}  {roll_cur_tgt:7.2f}  {roll_err:+7.2f}  {roll_speed:+7.1f}"
                  f"  {pitch_angle:8.2f}  {pitch_cur_tgt:7.2f}  {pitch_err:+7.2f}  {pitch_speed:+7.1f}")

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
        description="Dual-axis external PID gimbal control (ROLL + PITCH) via CMD_CONTROL MODE_SPEED",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('port',         nargs='?', default='/dev/ttyACM0')
    parser.add_argument('roll_target',  nargs='?', type=float, default=0.0,
                        help='Roll target angle in degrees')
    parser.add_argument('pitch_target', nargs='?', type=float, default=0.0,
                        help='Pitch target angle in degrees')
    parser.add_argument('hold',         nargs='?', type=float, default=30.0,
                        help='Run duration in seconds')

    parser.add_argument('--validate-imu', action='store_true',
                        help='Print both IMU angles without engaging motors (run first)')
    parser.add_argument('--calibrate',    action='store_true',
                        help='Measure speed-unit scale for one axis (run before PID)')
    parser.add_argument('--cal-axis',     default='pitch', choices=['pitch', 'roll'],
                        help='Axis to calibrate (default: pitch)')

    parser.add_argument('--power',      type=int,   default=70)
    parser.add_argument('--kp',         type=float, default=5.0)
    parser.add_argument('--ki',         type=float, default=0.1)
    parser.add_argument('--kd',         type=float, default=3.0)
    parser.add_argument('--step-roll',  type=float, default=None, metavar='DEG',
                        help='Step roll target to DEG at t=hold/2')
    parser.add_argument('--step-pitch', type=float, default=None, metavar='DEG',
                        help='Step pitch target to DEG at t=hold/2')

    args = parser.parse_args()

    ser = connect(args.port)
    handshake(ser)

    if args.validate_imu:
        validate_imu(ser)
        ser.close()
        return

    params = read_params(ser)

    if args.calibrate:
        calibrate_speed(ser, params, args.cal_axis, args.power)
        return

    run_pid_dual(ser, params,
                 args.roll_target, args.pitch_target, args.hold,
                 args.kp, args.ki, args.kd, args.power,
                 step_roll=args.step_roll,
                 step_pitch=args.step_pitch)


if __name__ == '__main__':
    main()
