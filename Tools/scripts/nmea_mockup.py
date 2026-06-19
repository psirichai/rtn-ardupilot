#!/usr/bin/env python3
"""
NMEA-0183 Mockup Generator

This script generates mock NMEA-0183 sentences and sends them over a specified serial port.
It is designed to test the AP_NMEAMux driver in ArduPilot.

It generates:
- White-listed: MWV, MWD, MDA, XDR, DPT, DBT, VHW
- Black-listed: MTW, GPGGA
"""

import serial
import time
import random
import argparse

def calculate_checksum(sentence):
    """Calculate the NMEA checksum for a given sentence."""
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def generate_mwv():
    """Wind Speed and Angle: $WIMWV,angle,R/T,speed,N/K,A*cs"""
    angle = round(random.uniform(0.0, 360.0), 1)
    ref = random.choice(['R', 'T'])
    speed = round(random.uniform(0.0, 30.0), 1)
    unit = random.choice(['N', 'K'])
    sentence = f"WIMWV,{angle},{ref},{speed},{unit},A"
    return f"${sentence}*{calculate_checksum(sentence)}\r\n"

def generate_mwd():
    """Wind Direction and Speed: $WIMWD,dirT,T,dirM,M,speedN,N,speedM,M*cs"""
    dir_t = round(random.uniform(0.0, 360.0), 1)
    dir_m = round(random.uniform(0.0, 360.0), 1)
    speed_n = round(random.uniform(0.0, 30.0), 1)
    speed_m = round(speed_n * 0.514444, 1)
    sentence = f"WIMWD,{dir_t},T,{dir_m},M,{speed_n},N,{speed_m},M"
    return f"${sentence}*{calculate_checksum(sentence)}\r\n"

def generate_mda():
    """Meteorological Composite: $WIMDA,press_inHg,I,press_bar,B,air_temp,C,..."""
    press_bar = round(random.uniform(0.95, 1.05), 4) # ~1 bar
    press_inHg = round(press_bar * 29.53, 2)
    air_temp = round(random.uniform(-10.0, 40.0), 1)
    # Just generating the required fields for our parser, padding rest
    sentence = f"WIMDA,{press_inHg},I,{press_bar},B,{air_temp},C,,,,,,,,,,"
    return f"${sentence}*{calculate_checksum(sentence)}\r\n"

def generate_xdr():
    """Transducer Measurements: $WIXDR,type,data,unit,id,..."""
    # Generate Pressure in bar and Temp in C
    press_bar = round(random.uniform(0.95, 1.05), 4)
    temp_c = round(random.uniform(-10.0, 40.0), 1)
    sentence = f"WIXDR,P,{press_bar},B,BARO,C,{temp_c},C,TEMP"
    return f"${sentence}*{calculate_checksum(sentence)}\r\n"

def generate_dpt():
    """Depth: $SDDPT,depth_m,offset_m,range_max*cs"""
    depth = round(random.uniform(1.0, 100.0), 1)
    offset = round(random.uniform(0.0, 2.0), 1)
    sentence = f"SDDPT,{depth},{offset},100"
    return f"${sentence}*{calculate_checksum(sentence)}\r\n"

def generate_dbt():
    """Depth Below Transducer: $SDDBT,feet,f,meters,M,fathoms,F*cs"""
    depth_m = round(random.uniform(1.0, 100.0), 1)
    depth_f = round(depth_m * 3.28084, 1)
    depth_fa = round(depth_m * 0.546807, 1)
    sentence = f"SDDBT,{depth_f},f,{depth_m},M,{depth_fa},F"
    return f"${sentence}*{calculate_checksum(sentence)}\r\n"

def generate_vhw():
    """Water Speed and Heading: $VDVHW,degT,T,degM,M,knots,N,kmph,K*cs"""
    deg_t = round(random.uniform(0.0, 360.0), 1)
    deg_m = round(random.uniform(0.0, 360.0), 1)
    knots = round(random.uniform(0.0, 20.0), 1)
    kmph = round(knots * 1.852, 1)
    sentence = f"VDVHW,{deg_t},T,{deg_m},M,{knots},N,{kmph},K"
    return f"${sentence}*{calculate_checksum(sentence)}\r\n"

def generate_mtw():
    """Water Temperature (Blacklisted): $INMTW,temp,C*cs"""
    temp = round(random.uniform(0.0, 30.0), 1)
    sentence = f"INMTW,{temp},C"
    return f"${sentence}*{calculate_checksum(sentence)}\r\n"

def generate_gga():
    """GPS Fix Data (Blacklisted): $GPGGA,..."""
    sentence = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
    return f"${sentence}*{calculate_checksum(sentence)}\r\n"

def main():
    parser = argparse.ArgumentParser(description='Generate mock NMEA-0183 messages.')
    parser.add_argument('--port', type=str, required=True, help='Serial port to send data to (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--rate', type=float, default=2.0, help='Output rate in Hz (default: 2.0)')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baudrate)
        print(f"Connected to {args.port} at {args.baudrate} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    generators = [
        generate_mwv,
        generate_mwd,
        generate_mda,
        generate_xdr,
        generate_dpt,
        generate_dbt,
        generate_vhw,
        generate_mtw,
        generate_gga
    ]

    period = 1.0 / args.rate
    print(f"Sending messages at {args.rate} Hz. Press Ctrl+C to stop.")

    try:
        while True:
            for _ in range(3): # Send a few messages per tick
                gen_func = random.choice(generators)
                sentence = gen_func()
                ser.write(sentence.encode('ascii'))
                print(f"Sent: {sentence.strip()}")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        if ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
