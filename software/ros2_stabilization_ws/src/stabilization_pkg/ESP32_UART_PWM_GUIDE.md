# ESP32 UART → PWM Gimbal Servo Control

This guide provides instructions for implementing the ESP32 firmware that receives UART commands from the ROS2 system and drives the gimbal pitch servo via PWM.

---

## Overview

### Protocol
- **UART Connection:** `/dev/ttyUSB0` (or configurable in ROS2 launch)
- **Baud Rate:** 115200 bps
- **Data Format:** ASCII text, `"P:{duty:.1f}\n"`
- **Example Commands:**
  - `P:0.0\n` → Pitch servo fully up (-45°)
  - `P:50.0\n` → Pitch servo neutral (0°)
  - `P:100.0\n` → Pitch servo fully down (+45°)

### Mapping
```
Duty Cycle (%)  ↔  Gimbal Angle (degrees)
    0%          ↔      -45°  (camera up)
   50%          ↔       0°   (level)
  100%          ↔      +45°  (camera down)
```

### PWM Output
- **Frequency:** 50 Hz (standard RC servo frequency)
- **Pulse Width Range:** 1000–2000 μs (typical servo range)
  - 0% duty   → 1000 μs (fully up)
  - 50% duty  → 1500 μs (neutral)
  - 100% duty → 2000 μs (fully down)

---

## Hardware Setup

### Wiring
```
ROS2 PC                        ESP32
   ↓                             ↓
(USB serial adapter)         (CH340/CP2102 etc.)
   ↓                             ↓
RX/TX lines ←────────────→ GPIO 16 (RX) / GPIO 17 (TX)
                           GPIO 25 (PWM output → servo signal pin)

Gimbal Servo                 ESP32
   ↓                          ↓
GND ←─────────────────────── GND (common ground)
5V  ←─────────────────────── 5V power
SIG ←─────────────────────── GPIO 25 (via optoisolator or 3.3V → 5V buffer)
```

**⚠️ Important:** Servo signal lines often require 5V logic. Use a level shifter or optoisolator between ESP32's 3.3V GPIO and servo's 5V signal line to avoid damage.

---

## Implementation (Arduino IDE / MicroPython)

### Option A: Arduino IDE (C++)

```cpp
// ESP32 UART to PWM Gimbal Servo
// Reads UART commands, outputs PWM to servo

#include <HardwareSerial.h>

// Pin definitions
const int PWM_PIN = 25;           // GPIO 25 for PWM output
const int RX_PIN = 16;            // GPIO 16 for UART RX
const int TX_PIN = 17;            // GPIO 17 for UART TX (may not use)
const int UART_BAUD = 115200;
const int PWM_FREQ = 50;          // Hz (standard servo frequency)
const int PWM_RESOLUTION = 16;    // 16-bit resolution

// Calculate PWM counts for pulse widths
// Duty cycle = pulse_width / period
// period = 1 / 50 Hz = 20ms = 20000 μs
// At 16-bit resolution (65535 counts): 1 count ≈ 20000/65535 ≈ 0.305 μs
// 1000 μs (0%)   ≈ 3277 counts
// 1500 μs (50%)  ≈ 4915 counts
// 2000 μs (100%) ≈ 6553 counts
const int MIN_PULSE = 1000;       // μs (0%)
const int MID_PULSE = 1500;       // μs (50%)
const int MAX_PULSE = 2000;       // μs (100%)

// Tracking
String command_buffer = "";
float last_duty = 50.0;           // Start at neutral

void setup() {
    // Initialize PWM
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);  // Channel 0, 50Hz, 16-bit
    ledcAttachPin(PWM_PIN, 0);
    
    // Start at neutral (50% = 1500 μs)
    setPWM(50.0);
    
    // Initialize UART (use Serial2 for RX on GPIO 16, TX on GPIO 17)
    Serial2.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    
    // Logging (Serial0 for debug, optional)
    Serial.begin(115200);
    Serial.println("ESP32 Gimbal Servo ready");
}

void loop() {
    // Check for incoming UART data
    while (Serial2.available()) {
        char c = Serial2.read();
        
        if (c == '\n') {
            // End of command — parse and execute
            if (command_buffer.startsWith("P:")) {
                String duty_str = command_buffer.substring(2);  // Extract "XX.X"
                float duty = duty_str.toFloat();
                
                // Clamp to valid range [0, 100]
                duty = constrain(duty, 0.0, 100.0);
                last_duty = duty;
                
                // Send PWM
                setPWM(duty);
                
                // Optional: log for debug
                // Serial.printf("CMD: P:%.1f → PWM\n", duty);
            }
            command_buffer = "";
        } else if (c != '\r') {
            // Accumulate characters (ignore CR)
            command_buffer += c;
        }
    }
}

/**
 * Set servo PWM based on duty cycle (0–100%).
 * Converts duty → pulse width (1000–2000 μs) → ledc count
 */
void setPWM(float duty) {
    // Linear interpolation: 0% → 1000 μs, 100% → 2000 μs
    float pulse_us = MIN_PULSE + (duty / 100.0) * (MAX_PULSE - MIN_PULSE);
    
    // Convert pulse width to ledc count
    // PWM period = 1 / 50 Hz = 20000 μs
    // ledc max count = 2^16 - 1 = 65535
    // 1 count ≈ 20000 / 65535 ≈ 0.305 μs
    // count = pulse_us / (20000 / 65535) = pulse_us * 65535 / 20000
    float period_us = 20000.0;  // 50 Hz
    int ledc_count = (int)(pulse_us * 65535.0 / period_us);
    
    // Clamp to valid range
    ledc_count = constrain(ledc_count, 0, 65535);
    
    // Write to PWM channel 0
    ledcWrite(0, ledc_count);
}
```

**To upload:**
1. Install ESP32 boards in Arduino IDE: `Tools → Board Manager → search "esp32" → install Espressif`
2. Select board: `Tools → Board → esp32 → ESP32 Dev Module` (or your variant)
3. Set port: `Tools → Port → /dev/ttyUSB0` (or COM port on Windows)
4. Paste code above into sketch
5. Verify (`Ctrl+/`) and upload (`Ctrl+U`)

---

### Option B: MicroPython

```python
# ESP32 UART to PWM Gimbal Servo (MicroPython)

from machine import UART, PWM, Pin
import math

# Configuration
RX_PIN = 16
TX_PIN = 17
PWM_PIN = 25
UART_BAUD = 115200
PWM_FREQ = 50  # Hz

# PWM pulse range (microseconds)
MIN_PULSE = 1000   # 0% (fully up)
MID_PULSE = 1500   # 50% (neutral)
MAX_PULSE = 2000   # 100% (fully down)

# Initialize UART
uart = UART(1, UART_BAUD, tx=Pin(TX_PIN), rx=Pin(RX_PIN))
uart.init(bits=8, parity=None, stop=1)

# Initialize PWM
pwm = PWM(Pin(PWM_PIN))
pwm.freq(PWM_FREQ)

# Start at neutral
last_duty = 50.0
set_pwm(last_duty)

def set_pwm(duty):
    """
    Set servo position based on duty cycle (0–100%).
    duty: 0 = fully up, 50 = neutral, 100 = fully down
    """
    # Clamp to valid range
    duty = max(0.0, min(100.0, duty))
    
    # Linear interpolation: duty [0, 100] → pulse [1000, 2000] μs
    pulse_us = 1000 + (duty / 100.0) * (2000 - 1000)
    
    # Convert to duty cycle for ledc (0–1023 for 10-bit resolution)
    # Period = 1 / 50 Hz = 20 ms = 20000 μs
    # duty_cycle = pulse_us / period_us = pulse_us / 20000
    period_us = 20000.0
    duty_cycle = int((pulse_us / period_us) * 1023)  # 10-bit: 0–1023
    
    pwm.duty(duty_cycle)

def process_command(cmd_str):
    """Parse 'P:XX.X' command and set PWM."""
    global last_duty
    try:
        if cmd_str.startswith('P:'):
            duty_str = cmd_str[2:]
            duty = float(duty_str)
            last_duty = duty
            set_pwm(duty)
            # Optional: print(f"Set duty to {duty:.1f}%")
    except Exception as e:
        print(f"Parse error: {e}")

# Main loop
command_buffer = ""
while True:
    if uart.any():
        byte = uart.read(1)
        if byte:
            char = byte.decode('utf-8')
            
            if char == '\n':
                # End of command
                if command_buffer.strip():
                    process_command(command_buffer.strip())
                command_buffer = ""
            elif char != '\r':
                command_buffer += char
```

**To upload:**
1. Install MicroPython on ESP32: follow [micropython.org](https://micropython.org)
2. Use `ampy` or `mpfshell` to upload:
   ```bash
   ampy --port /dev/ttyUSB0 put gimbal_servo.py
   # or: mpfshell -o /dev/ttyUSB0
   ```
3. Reboot ESP32; script runs automatically on boot

---

## Testing

### Step 1: Verify UART Connection
```bash
# On ROS2 PC, test serial communication
python3 -c "
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.write(b'P:50.0\n')  # Send neutral command
print('Sent P:50.0')
ser.close()
"
```

### Step 2: Servo Movement Test
```bash
# Send commands and observe servo movement
python3 << 'EOF'
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

commands = [
    ('P:0.0\n',   'Up (-45°)'),
    ('P:50.0\n',  'Neutral (0°)'),
    ('P:100.0\n', 'Down (+45°)'),
    ('P:25.0\n',  'Mid-up'),
    ('P:75.0\n',  'Mid-down'),
    ('P:50.0\n',  'Neutral'),
]

for cmd, desc in commands:
    ser.write(cmd.encode())
    print(f"Sent: {desc}")
    time.sleep(1.5)  # Let servo settle

ser.close()
EOF
```

### Step 3: ROS2 Integration Test
```bash
# Terminal 1: launch face tracking
cd software/ros2_stabilization_ws
source install/setup.bash
ros2 launch stabilization_pkg face_tracking_test_launch.py

# Terminal 2: publish test pitch commands
ros2 topic pub /face_tracker/pitch_cmd std_msgs/Float32 'data: 0.0'
ros2 topic pub /face_tracker/pitch_cmd std_msgs/Float32 'data: 20.0'
ros2 topic pub /face_tracker/pitch_cmd std_msgs/Float32 'data: -20.0'

# Watch gimbal servo move!
```

---

## Future: Roll Command Extension

When ready to add roll control (head tilt), extend the protocol:

### Extended Command Format
```
P:{pitch_duty:.1f} R:{roll_duty:.1f}\n
```

**Example:**
- `P:50.0 R:50.0\n` → Pitch neutral, roll neutral
- `P:40.0 R:65.0\n` → Pitch up slightly, roll right

### ROS2 Side (face_tracker node)
Currently publishes only `/face_tracker/pitch_cmd` (Float32).
To add roll, extend `uart_gimbal_servo.py`:

```python
# Add new subscription
self.create_subscription(Float32, '/face_tracker/roll_cmd', self._roll_cmd_cb, 10)

# Store roll command
self._roll_cmd = 50.0  # neutral

# Modify callback to send both pitch and roll
def _pitch_cmd_cb(self, msg: Float32):
    if self.ser and self.ser.is_open:
        pitch_duty = self._deg_to_duty(msg.data)
        roll_duty = self._roll_cmd  # current roll
        cmd = f"P:{pitch_duty:.1f} R:{roll_duty:.1f}\n"
        self.ser.write(cmd.encode())
```

### ESP32 Side
Parse extended command:

```cpp
void setPWM(float pitch_duty, float roll_duty) {
    // Pitch on GPIO 25
    ledcWrite(0, calcLedc(pitch_duty));
    
    // Roll on GPIO 26 (new)
    ledcWrite(1, calcLedc(roll_duty));
}

void loop() {
    while (Serial2.available()) {
        char c = Serial2.read();
        
        if (c == '\n') {
            // Parse "P:XX.X R:YY.Y"
            float pitch = 50.0, roll = 50.0;
            int p_idx = command_buffer.indexOf("P:");
            int r_idx = command_buffer.indexOf("R:");
            
            if (p_idx >= 0) {
                pitch = command_buffer.substring(p_idx + 2).toFloat();
            }
            if (r_idx >= 0) {
                roll = command_buffer.substring(r_idx + 2).toFloat();
            }
            
            setPWM(pitch, roll);
            command_buffer = "";
        } else if (c != '\r') {
            command_buffer += c;
        }
    }
}
```

---

## Debugging Tips

### UART Issues
- **No data received:** Check wiring (RX ↔ TX crossed?), baud rate, USB cable
- **Garbage characters:** Baud rate mismatch; verify both sides are 115200
- **Intermittent connection:** Loose USB cable, try another port/adapter

### PWM Issues
- **Servo doesn't move:** 
  - Check power supply (servo draws significant current)
  - Verify PWM frequency is 50 Hz
  - Check pulse range (should be 1000–2000 μs)
- **Servo moves wrong direction:**
  - Swap MIN/MAX_PULSE values
  - Or reverse duty mapping: `duty = 100.0 - received_duty`

### ROS2 Integration
```bash
# Monitor UART commands being published
ros2 topic echo /face_tracker/pitch_cmd

# Monitor gimbal servo responses
ros2 topic echo /gimbal/angles

# Check node logs
ros2 run stabilization_pkg uart_gimbal_servo --ros-args --log-level DEBUG
```

---

## Files & References

- **ROS2 Node:** `stabilization_pkg/uart_gimbal_servo.py` — handles pitch_cmd → UART
- **Launch Script:** `launch/face_tracking_test_launch.py` — full face tracking pipeline
- **Protocol Spec:** This document (ESP32_UART_PWM_GUIDE.md)
- **Gimbal Servo Datasheet:** Storm BCG (±45° range, typical 1500 μs neutral)

---

## Contact / Questions

If you encounter issues:
1. Check the ROS2 logs: `ros2 topic echo /face_tracker/pitch_cmd` (should see commands)
2. Test UART directly: `cat < /dev/ttyUSB0` (should print commands if ESP32 loops them back)
3. Verify servo movement: Apply known PWM (1500 μs) and confirm it moves to neutral

Good luck! 🚀
