# Storm BCG Gimbal Reader

Simple Python script to read angles and gyroscope data from Storm BCG 8-bit gimbal over serial.

## What It Does

Reads real-time data from the gimbal and displays:
- **Roll angle** (degrees)
- **Pitch angle** (degrees)  
- **Yaw angle** (degrees, continuous - can exceed 360°)
- **Roll gyroscope** (raw angular velocity)
- **Pitch gyroscope** (raw angular velocity)

Updates at ~10Hz in your terminal.

## Setup

### 1. Install Python Dependencies

```bash
sudo apt install python3-serial
# or
pip3 install pyserial
```

### 2. Fix Serial Port Permissions (Linux)

Add your user to the `dialout` group so you can access `/dev/ttyACM0` without sudo:

```bash
sudo usermod -a -G dialout $USER
```

**Important:** Log out and log back in (or reboot) for this to take effect.

### 3. Connect Gimbal

- Plug gimbal into computer via USB
- Power on gimbal
- Verify it shows up: `ls /dev/ttyACM*`

## Usage

### Auto-detect port (uses /dev/ttyACM0)
```bash
python3 gimbal_reader.py
```

### Specify port manually
```bash
python3 gimbal_reader.py /dev/ttyACM1
```

### Example Output

```
✓ Connected to /dev/ttyACM0

Reading gimbal data (Ctrl+C to stop)...

    Time |     Roll |    Pitch |       Yaw | RollGyro | PitchGyro
---------------------------------------------------------------------------
19:45:12 |    2.34° |   -1.08° |    180.5° |       12 |       -45
19:45:13 |    2.52° |   -0.90° |    181.2° |      156 |       234
19:45:14 |   15.48° |   -0.72° |    182.8° |     4521 |       89
```

## Understanding the Values

### Angles
- **Roll**: ±90° (limited by gimbal mechanics)
- **Pitch**: ±180° (full rotation possible)
- **Yaw**: Continuous (0° → 360° → 720° → ... if rotating clockwise)

### Gyroscopes
- **Raw units** representing angular velocity
- Larger values = faster rotation
- Positive/negative indicates direction
- Typical range: ±6000 for rapid hand movements

### Notes
- Roll/pitch angles may overshoot during fast movements (accelerometer artifacts)
- Yaw is absolute and accumulates beyond 360°
- Press `Ctrl+C` to stop

## Troubleshooting

**"Permission denied" error:**
```bash
sudo usermod -a -G dialout $USER
# Then log out and back in
```

**"No such file or directory" - port not found:**
```bash
ls /dev/ttyACM*  # Check which port your gimbal is on
python3 gimbal_reader.py /dev/ttyACM1  # Try different port
```

**No data received:**
- Check gimbal is powered on
- Try unplugging and replugging USB
- Close SimpleBGC GUI if it's running

## Protocol Details

Uses SimpleBGC Serial API v2:
- Command: `CMD_REALTIME_DATA` (0x44)
- Baudrate: 115200
- Packet structure: `[0x3E][CMD][SIZE][CRC][DATA...][CRC]`

### Data Layout (from response packet)

| Bytes  | Type   | Value                     | Scale        |
|--------|--------|---------------------------|--------------|
| 0-1    | int16  | Roll angle                | 0.18°/unit   |
| 2-3    | int16  | Roll gyroscope            | raw          |
| 6-7    | int16  | Pitch angle               | 0.18°/unit   |
| 8-9    | int16  | Pitch gyroscope           | raw          |
| 42-43  | int16  | Yaw angle                 | 0.1°/unit    |

All values are signed 16-bit integers in little-endian format.
