#!/usr/bin/env python3
"""
Storm BCG 8-bit Gimbal Reader
Communicates with gimbal via SimpleBGC v2 protocol over serial.
Polls REALTIME DATA every second and prints pitch/roll/yaw values.
"""

import serial
import time
import struct
import sys
from pathlib import Path


class SimpleBGCGimbal:
    """Handles SimpleBGC protocol communication with gimbal."""
    
    # SimpleBGC v2 protocol constants
    HEADER = 0x3E
    CMD_REALTIME_DATA = 0x44
    CMD_CONTROL = 0x69
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize gimbal connection.
        
        Args:
            port: Serial port (e.g., '/dev/ttyACM0')
            baudrate: Baud rate (default 115200)
            timeout: Serial read timeout in seconds
        """
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.connected = False
    
    def connect(self) -> bool:
        """Open serial connection to gimbal."""
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                rtscts=False,
                dsrdtr=True
            )
            time.sleep(2)  # Give gimbal time to initialize
            self.connected = True
            print(f"✓ Connected to {self.port_name} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {self.port_name}: {e}")
            print("\nTroubleshooting:")
            print("  1. Check gimbal is powered on and connected via USB")
            print("  2. Verify port with: ls -la /dev/ttyACM*")
            print("  3. Check permissions (see SETUP.md)")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connected = False
            print("✓ Disconnected")
    
    def _build_packet(self, cmd: int, data: bytes = b'') -> bytes:
        """
        Build a SimpleBGC v2 packet.
        
        Format:
            [0x3E] [CMD] [SIZE] [HEADER_CRC] [DATA...] [DATA_CRC]
        
        Args:
            cmd: Command byte
            data: Command data (optional)
        
        Returns:
            Complete packet as bytes
        """
        size = len(data)
        
        # Header checksum: (cmd + size) & 0xFF
        header_crc = (cmd + size) & 0xFF
        
        # Data checksum: sum of all data bytes
        data_crc = sum(data) & 0xFF
        
        packet = bytes([self.HEADER, cmd, size, header_crc]) + data + bytes([data_crc])
        return packet
    
    def _parse_realtime_data(self, data: bytes) -> dict:
        """
        Parse REALTIME DATA (0x44) response.
        
        Pitch/Roll validated experimentally. Magnetometer estimated.
        SimpleBGC uses 0.02° per unit for angles.
        """
        if len(data) < 42:
            return None
        
        try:
            # Bytes [0:2] - PITCH angle
            pitch_raw = struct.unpack('<h', data[0:2])[0]
            pitch = pitch_raw * 0.02
            
            # Bytes [6:8] - ROLL angle
            roll_raw = struct.unpack('<h', data[6:8])[0]
            roll = roll_raw * 0.02
            
            # Bytes [37:39] - MAGNETOMETER (estimated)
            mag_raw = struct.unpack('<h', data[37:39])[0]
            
            result = {
                'pitch_deg': pitch,
                'roll_deg': roll,
                'pitch_raw': pitch_raw,
                'roll_raw': roll_raw,
                'magnetometer_raw': mag_raw,
            }
            return result
        except struct.error as e:
            print(f"✗ Failed to parse realtime data: {e}")
            return None
    
    def request_realtime_data(self) -> dict:
        """
        Request and read REALTIME DATA from gimbal.
        
        Returns:
            dict with angle values, or None on error
        """
        if not self.connected:
            return None
        
        try:
            # Build and send request packet
            packet = self._build_packet(self.CMD_REALTIME_DATA)
            self.ser.write(packet)
            
            # Read response (collect data with small delays)
            response = b''
            for _ in range(30):  # Try up to 30 reads
                if self.ser.in_waiting > 0:
                    response += self.ser.read(self.ser.in_waiting)
                time.sleep(0.01)  # Small delay between reads
            
            if len(response) == 0:
                print("✗ No response from gimbal")
                return None
            
            # Validate response header
            if response[0] != self.HEADER:
                print(f"✗ Invalid response header: {response[0]:02x}")
                return None
            
            if len(response) < 4:
                print(f"✗ Response too short: {len(response)} bytes")
                return None
            
            cmd = response[1]
            size = response[2]
            
            # Extract data payload (skip header + cmd + size + header_crc)
            payload = response[4:4+size]
            
            if len(payload) < size:
                print(f"✗ Incomplete payload: expected {size}, got {len(payload)}")
                return None
            
            # Parse the data
            angles = self._parse_realtime_data(payload)
            return angles
        
        except Exception as e:
            print(f"✗ Error reading data: {e}")
            return None
    
    def send_control(self, pitch: float, roll: float) -> bool:
        """
        Send pitch and roll control values to gimbal.
        
        Args:
            pitch: Pitch angle in degrees
            roll: Roll angle in degrees
        
        Returns:
            True if sent successfully
        """
        if not self.connected:
            return False
        
        try:
            # Convert degrees to SimpleBGC units (0.02° per unit)
            pitch_units = int(pitch / 0.02)
            roll_units = int(roll / 0.02)
            
            # Pack control data (pitch, roll as int16_t)
            control_data = struct.pack('<hh', pitch_units, roll_units)
            
            packet = self._build_packet(self.CMD_CONTROL, control_data)
            self.ser.write(packet)
            
            print(f"→ Sent control: pitch={pitch}°, roll={roll}°")
            return True
        
        except Exception as e:
            print(f"✗ Failed to send control: {e}")
            return False


def find_gimbal_port() -> str:
    """
    Auto-detect gimbal on /dev/ttyACM* ports.
    
    Returns:
        Port name (e.g., '/dev/ttyACM0') or None if not found
    """
    import glob
    ports = sorted(glob.glob('/dev/ttyACM*'))
    if ports:
        return ports[0]  # Return first one
    return None


def main():
    """Main demo: poll gimbal every second."""
    
    # Auto-detect or use first argument
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_gimbal_port()
        if not port:
            print("✗ No /dev/ttyACM* ports found.")
            print("  Try: python3 gimbal_reader.py /dev/ttyACM0")
            sys.exit(1)
        print(f"ℹ Auto-detected gimbal on {port}")
    
    gimbal = SimpleBGCGimbal(port)
    
    if not gimbal.connect():
        sys.exit(1)
    
    try:
        print("\nPolling gimbal every 1 second (Ctrl+C to stop)...\n")
        
        while True:
            angles = gimbal.request_realtime_data()
            
            if angles:
                print(f"Pitch={angles['pitch_deg']:7.2f}°  "
                      f"Roll={angles['roll_deg']:7.2f}°  "
                      f"Mag={angles['magnetometer_raw']:6d}  "
                      f"(raw: pitch={angles['pitch_raw']:6d}, roll={angles['roll_raw']:6d})")
            
            time.sleep(1.0)
    
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        gimbal.disconnect()


if __name__ == '__main__':
    main()
