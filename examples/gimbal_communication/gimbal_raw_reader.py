#!/usr/bin/env python3
"""
Storm BCG Gimbal - Raw Value Inspector
Minimal script to display all int16 values from REALTIME_DATA packet.
"""
import serial
import time
import struct
import sys

class GimbalReader:
    HEADER = 0x3E
    CMD_REALTIME_DATA = 0x44
    
    def __init__(self, port: str):
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=1.0,
            dsrdtr=True
        )
        time.sleep(2)
    
    def _build_packet(self, cmd: int) -> bytes:
        size = 0
        header_crc = (cmd + size) & 0xFF
        data_crc = 0
        return bytes([self.HEADER, cmd, size, header_crc, data_crc])
    
    def read_raw(self) -> dict:
        """Request data and return all int16 values unpacked."""
        self.ser.write(self._build_packet(self.CMD_REALTIME_DATA))
        
        # Collect response
        response = b''
        for _ in range(30):
            if self.ser.in_waiting > 0:
                response += self.ser.read(self.ser.in_waiting)
            time.sleep(0.01)
        
        if len(response) < 4:
            return None
        
        # Skip header: [0x3E][CMD][SIZE][HEADER_CRC]
        payload = response[4:]
        
        # Extract size from packet
        size = response[2]
        payload = payload[:size]  # Only take declared data
        
        # Unpack all int16 values (little-endian)
        num_values = len(payload) // 2
        values = struct.unpack(f'<{num_values}h', payload[:num_values*2])
        
        # Also get as unsigned for comparison
        unsigned = struct.unpack(f'<{num_values}H', payload[:num_values*2])
        
        return {
            'signed': values,
            'unsigned': unsigned,
            'raw_bytes': payload
        }
    
    def close(self):
        self.ser.close()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    
    print(f"Connecting to {port}...")
    gimbal = GimbalReader(port)
    print("Connected. Press Ctrl+C to stop.\n")
    
    try:
        while True:
            data = gimbal.read_raw()
            if not data:
                print("No data")
                time.sleep(1)
                continue
            
            signed = data['signed']
            unsigned = data['unsigned']
            
            print("\n" + "="*80)
            print(f"Timestamp: {time.time():.2f}")
            print("="*80)
            print(f"{'Byte':>5} | {'Signed (int16)':>15} | {'Unsigned (uint16)':>17} | {'As degrees (*0.18)':>18}")
            print("-"*80)
            
            for i in range(len(signed)):
                byte_offset = i * 2
                deg = signed[i] * 0.18
                print(f"[{byte_offset:2d}-{byte_offset+1:2d}] | {signed[i]:15d} | {unsigned[i]:17d} | {deg:18.2f}")
            
            # Highlight the ones you mentioned
            print("\n" + "-"*80)
            print("KEY OBSERVATIONS:")
            if len(signed) > 0:
                print(f"  [0-1]   (your 'roll'):     {signed[0]:8d}  ({signed[0]*0.18:7.2f}°)")
            if len(signed) > 3:
                print(f"  [6-7]   (your 'pitch'):    {signed[3]:8d}  ({signed[3]*0.18:7.2f}°)")
            if len(signed) > 18:
                print(f"  [36-37] :                  {signed[18]:8d}  ({signed[18]*0.18:7.2f}°)")
            if len(signed) > 19:
                print(f"  [38-39] (mystery jumper):  {signed[19]:8d}  ({signed[19]*0.18:7.2f}°)")
            
            time.sleep(1.0)
    
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        gimbal.close()


if __name__ == '__main__':
    main()
