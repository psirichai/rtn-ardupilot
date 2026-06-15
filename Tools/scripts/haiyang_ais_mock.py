import serial
import time
import random
import argparse

def encode_aivdm(mmsi, lat, lon, heading, speed):
    def int_to_bin(val, bits):
        if val < 0:
            val = (1 << bits) + val
        return format(val, f'0{bits}b')

    lon_val = int(lon * 600000)
    lat_val = int(lat * 600000)

    bin_str = (
        int_to_bin(1, 6) +
        int_to_bin(0, 2) +
        int_to_bin(mmsi, 30) +
        int_to_bin(0, 4) +
        int_to_bin(0, 8) +
        int_to_bin(int(speed * 10), 10) +
        int_to_bin(0, 1) +
        int_to_bin(lon_val, 28) +
        int_to_bin(lat_val, 27) +
        int_to_bin(int(heading * 10), 12) +
        int_to_bin(heading, 9) +
        int_to_bin(0, 6) +
        int_to_bin(0, 1) +
        int_to_bin(0, 19)
    )

    ascii_payload = ""
    for i in range(0, len(bin_str), 6):
        chunk = bin_str[i:i+6]
        if len(chunk) < 6:
            chunk = chunk.ljust(6, '0')
        val = int(chunk, 2)
        if val < 40:
            val += 48
        else:
            val += 56
        ascii_payload += chr(val)

    sentence = f"!AIVDM,1,1,,A,{ascii_payload},0"
    checksum = 0
    for char in sentence[1:]:
        checksum ^= ord(char)

    return f"{sentence}*{checksum:02X}\r\n".encode('ascii')

def main():
    parser = argparse.ArgumentParser(description='Mock HAIYANG HIS-50A AIS Device')
    parser.add_argument('--port', required=True, help='Serial port')
    parser.add_argument('--baud', type=int, default=38400, help='Baud rate')
    parser.add_argument('--lat', type=float, default=47.3667, help='Base Latitude')
    parser.add_argument('--lon', type=float, default=8.5500, help='Base Longitude')
    parser.add_argument('--rate', type=float, default=1.0, help='Message rate (Hz)')

    args = parser.parse_args()

    print(f"Starting AIS mock on {args.port} at {args.baud} baud...")

    try:
        ser = serial.Serial(args.port, args.baud)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        while True:
            mmsi = 123456789
            lat = args.lat + random.uniform(-0.01, 0.01)
            lon = args.lon + random.uniform(-0.01, 0.01)
            heading = random.randint(0, 359)
            speed = random.uniform(5.0, 15.0)

            msg = encode_aivdm(mmsi, lat, lon, heading, speed)

            decoded_msg = msg.decode('ascii').strip()
            print(f"Sending: {decoded_msg}")

            try:
                ser.write(msg)
                ser.flush()
            except serial.SerialException as e:
                print(f"Error writing to serial port: {e}")
                break

            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print("\nStopping mock.")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
