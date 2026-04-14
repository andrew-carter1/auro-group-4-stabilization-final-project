# SimpleBGC Control Packet Reference

## Control Command (CMD_CONTROL = 0x69)

### Packet Structure
```
[HEADER] [CMD] [SIZE] [HEADER_CRC] [DATA...] [DATA_CRC]
  0x3E    0x69   N      (cmd+size)   payload    sum(payload)
```

### Current Implementation (Untested)
**Payload:** 4 bytes minimum
- Bytes [0:2]: Pitch as int16_le (units: 0.02° per unit)
- Bytes [2:4]: Roll as int16_le (units: 0.02° per unit)

Example conversion (Python):
```python
pitch_deg = 45.0
roll_deg = -30.0

pitch_units = int(pitch_deg / 0.02)  # Convert degrees to SimpleBGC units
roll_units = int(roll_deg / 0.02)

control_data = struct.pack('<hh', pitch_units, roll_units)
```

### Packet Building Function
```python
def _build_packet(cmd: int, data: bytes = b'') -> bytes:
    """Build SimpleBGC v2 packet with checksums."""
    size = len(data)
    header_crc = (cmd + size) & 0xFF
    data_crc = sum(data) & 0xFF
    
    packet = bytes([0x3E, cmd, size, header_crc]) + data + bytes([data_crc])
    return packet
```

### Send Control Function
```python
def send_control(self, pitch: float, roll: float) -> bool:
    """
    Send pitch and roll control values to gimbal.
    
    Args:
        pitch: Pitch angle in degrees
        roll: Roll angle in degrees
    
    Returns:
        True if sent successfully
    
    NOTE: Control payload structure not yet validated against gimbal response.
    """
    if not self.connected:
        return False
    
    try:
        pitch_units = int(pitch / 0.02)
        roll_units = int(roll / 0.02)
        
        control_data = struct.pack('<hh', pitch_units, roll_units)
        packet = self._build_packet(0x69, control_data)
        self.ser.write(packet)
        
        print(f"→ Sent control: pitch={pitch}°, roll={roll}°")
        return True
    
    except Exception as e:
        print(f"✗ Failed to send control: {e}")
        return False
```

## Known Issues / Validation Needed

1. **Payload structure unconfirmed** — only pitch/roll as int16s tested in theory, not against gimbal
2. **Scale factor (0.02°)** — matches REALTIME_DATA but may differ for control
3. **Yaw handling** — gimbal has no yaw motor; control payload may need yaw=0 or different structure
4. **Response validation** — no ACK/NACK handling after sending control command
5. **Blocking behavior** — unclear if gimbal accepts rapid successive commands or needs inter-command delays

## Testing Checklist

- [ ] Send control with pitch=0, roll=0 (no-op, verify no crash)
- [ ] Send pitch=45, roll=0 (verify gimbal moves to pitch position)
- [ ] Send pitch=0, roll=45 (verify gimbal moves to roll position)
- [ ] Send pitch=45, roll=45 (verify combined movement)
- [ ] Rapid commands (test if gimbal keeps up or needs delays)
- [ ] Read REALTIME_DATA after control to verify gimbal reached setpoint
- [ ] Check for any response/ACK packet from gimbal (may need read_timeout increase)

## Alternative Payload Structures to Test

If simple (pitch, roll) doesn't work, try:

**Extended format (6 bytes):**
```python
# Pitch, Roll, Yaw (no-op)
control_data = struct.pack('<hhh', pitch_units, roll_units, 0)
```

**With flags (8+ bytes):**
```python
# pitch, roll, yaw, control_mode, reserved, etc.
# Requires reverse-engineering from GUI captures
```

## SimpleBGC Command Reference (Partial)

Known working:
- `0x44` (REALTIME_DATA) — read gimbal state ✓ validated
- `0x56` (BOARD_INFO) — read board version (unreliable)

To test:
- `0x69` (CONTROL) — set pitch/roll setpoints ⚠️ untested
- Others in SimpleBGC spec (if docs found)
