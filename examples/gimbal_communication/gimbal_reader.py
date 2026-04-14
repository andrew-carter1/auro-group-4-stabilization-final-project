#!/usr/bin/env python3
"""
Storm BCG Gimbal Reader
Reads pitch, roll, yaw angles and gyroscope rates from gimbal via serial.
"""
import serial
import time
import struct
import sys

class StormGimbal:
    HEADER = 0x3E
    CMD_REALTIME_DATA = 0x44
    
    def __init__(self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1.0,
            dsrdtr=True
        )
        time.sleep(2)  # Wait for gimbal to initialize
        print(f"✓ Connected to {port}")
    
    def _build_packet(self, cmd: int) -> bytes:
        """Build SimpleBGC command packet."""
        size = 0
        header_crc = (cmd + size) & 0xFF
        data_crc = 0
        return bytes([self.HEADER, cmd, size, header_crc, data_crc])
    
    def read_data(self) -> dict:
        """
        Request and parse gimbal data.
        
        Returns dict with:
        - roll_deg: Roll angle in degrees
        - pitch_deg: Pitch angle in degrees  
        - yaw_deg: Yaw angle in degrees (continuous, can exceed 360°)
        - roll_gyro: Roll angular velocity (raw units)
        - pitch_gyro: Pitch angular velocity (raw units)
        """
        # Send request
        self.ser.write(self._build_packet(self.CMD_REALTIME_DATA))
        
        # Read response
        response = b''
        for _ in range(30):
            if self.ser.in_waiting > 0:
                response += self.ser.read(self.ser.in_waiting)
            time.sleep(0.01)
        
        if len(response) < 48:  # Need at least 44 bytes of data + 4 byte header
            return None
        
        # Extract payload (skip 4-byte header)
        payload = response[4:]
        
        # Parse values
        # [0-1]: Roll angle (signed int16, 0.18° per unit)
        roll_raw = struct.unpack('<h', payload[0:2])[0]
        roll_deg = roll_raw * 0.18
        
        # [2-3]: Roll gyroscope (signed int16, raw angular velocity)
        roll_gyro = struct.unpack('<h', payload[2:4])[0]
        
        # [6-7]: Pitch angle (signed int16, 0.18° per unit)
        pitch_raw = struct.unpack('<h', payload[6:8])[0]
        pitch_deg = pitch_raw * 0.18
        
        # [8-9]: Pitch gyroscope (signed int16, raw angular velocity)
        pitch_gyro = struct.unpack('<h', payload[8:10])[0]
        
        # [42-43]: Yaw angle (signed int16, 0.1° per unit)
        yaw_raw = struct.unpack('<h', payload[42:44])[0]
        yaw_deg = yaw_raw * 0.1
        
        return {
            'roll_deg': roll_deg,
            'pitch_deg': pitch_deg,
            'yaw_deg': yaw_deg,
            'roll_gyro': roll_gyro,
            'pitch_gyro': pitch_gyro,
            'roll_raw': roll_raw,
            'pitch_raw': pitch_raw,
            'yaw_raw': yaw_raw
        }
    
    def close(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Disconnected")


def main():
    # Get port from command line or use default
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    
    gimbal = StormGimbal(port)
    
    try:
        print("\nReading gimbal data (Ctrl+C to stop)...\n")
        print(f"{'Time':>8} | {'Roll':>8} | {'Pitch':>8} | {'Yaw':>9} | {'RollGyro':>9} | {'PitchGyro':>10}")
        print("-" * 75)
        
        while True:
            data = gimbal.read_data()
            
            if data:
                timestamp = time.strftime("%H:%M:%S")
                print(f"{timestamp} | "
                      f"{data['roll_deg']:7.2f}° | "
                      f"{data['pitch_deg']:7.2f}° | "
                      f"{data['yaw_deg']:8.1f}° | "
                      f"{data['roll_gyro']:8d} | "
                      f"{data['pitch_gyro']:9d}")
            else:
                print("No data received")
            
            time.sleep(0.1)  # Read at ~10Hz
    
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        gimbal.close()


if __name__ == '__main__':
    main()
