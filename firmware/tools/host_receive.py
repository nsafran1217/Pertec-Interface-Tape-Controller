#!/usr/bin/env python3
"""
Simple host receiver script to capture the streamed tape image from the MCU
over a USB CDC / serial device and save to a file.

Usage:
  python3 host_receive.py /dev/ttyACM1 output.img

Requires pyserial: pip install pyserial
"""
import sys
import serial

if len(sys.argv) < 3:
    print("Usage: {} <serial-device> <output-file>".format(sys.argv[0]))
    sys.exit(2)

port = sys.argv[1]
outfile = sys.argv[2]

ser = serial.Serial(port, 115200, timeout=1)
print(f"Opened {port} @115200, writing to {outfile}")

with open(outfile, 'wb') as f:
    try:
        while True:
            data = ser.read(4096)
            if not data:
                # No data received within timeout; continue polling
                continue
            f.write(data)
            f.flush()
    except KeyboardInterrupt:
        print('\nInterrupted by user, closing.')
    finally:
        ser.close()
