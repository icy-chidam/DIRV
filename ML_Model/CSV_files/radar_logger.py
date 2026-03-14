import serial
import time
import struct
import numpy as np
import csv
import csv
import datetime
import sys

# ============================================================================
# USER CONFIGURATION - ADJUST THESE TO MATCH YOUR SYSTEM
# ============================================================================
CLI_PORT = 'COM13'          # User/CLI port (sending configurastion)
DATA_PORT = 'COM14'         # Data port (receiving point cloud)
CONFIG_FILE = 'ISK_6m_default.cfg'   # Path to your .cfg file
CSV_OUTPUT = 'pointcloud_pedestrian_2PEOP_.csv'      # Output CSV file name
MAX_FRAMES = 2000         # Stop after this many frames
# ============================================================================

# ----------------------------------------------------------------------------
# Helper functions (byte to number conversions)
# ----------------------------------------------------------------------------
def uint32_le(data):
    return data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24)

def uint16_le(data):
    return data[0] + (data[1] << 8)

def int8_t(data):
    return struct.unpack('<b', bytes([data]))[0]

def int16_t(data0, data1):
    return struct.unpack('<h', bytes([data0, data1]))[0]

# ----------------------------------------------------------------------------
# Send configuration to the sensor via CLI port
# ----------------------------------------------------------------------------
def send_config(cli_port, config_file):
    print(f"Sending configuration from {config_file}...")
    with serial.Serial(cli_port, 115200, timeout=1) as ser:
        time.sleep(2)
        ser.reset_input_buffer()
        with open(config_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('%'):
                    print(f"  > {line}")
                    ser.write((line + '\n').encode())
                    time.sleep(0.05)   # small delay between commands
        print("Configuration sent successfully.\n")

# ----------------------------------------------------------------------------
# Parse one frame of data from the byte buffer (3D People Tracking format)
# Returns: (success, frame_number, points_list, bytes_consumed)
#   points_list: list of dicts with keys: x, y, z, azimuth, snr
# ----------------------------------------------------------------------------
def parse_frame(buffer):
    MAGIC = bytes([2, 1, 4, 3, 6, 5, 8, 7])
    HEADER_SIZE = 40
    TLV_TYPE_POINTCLOUD = 1020

    # Look for magic word
    magic_idx = buffer.find(MAGIC)
    if magic_idx == -1:
        return False, 0, [], 0   # no magic word found

    if len(buffer) < magic_idx + HEADER_SIZE:
        return False, 0, [], 0   # not enough data for header

    # Parse frame header
    total_packet_len = uint32_le(buffer[magic_idx+12:magic_idx+16])
    frame_number = uint32_le(buffer[magic_idx+20:magic_idx+24])
    num_tlvs = uint32_le(buffer[magic_idx+32:magic_idx+36])

    # Check if we have the whole packet
    if len(buffer) < magic_idx + total_packet_len:
        return False, 0, [], 0   # incomplete packet

    # Now parse TLVs starting after header
    ptr = magic_idx + HEADER_SIZE
    points = []

    for tlv_idx in range(num_tlvs):
        if ptr + 8 > magic_idx + total_packet_len:
            break   # malformed
        tlv_type = uint32_le(buffer[ptr:ptr+4])
        tlv_len = uint32_le(buffer[ptr+4:ptr+8])   # length in bytes
        ptr += 8
        tlv_end = ptr + tlv_len

        if tlv_type == TLV_TYPE_POINTCLOUD:
            # Need at least 20 bytes for the unit factors
            if ptr + 20 > tlv_end:
                break
            # Read unit scaling factors (floats)
            elev_unit = struct.unpack('<f', buffer[ptr:ptr+4])[0]
            az_unit   = struct.unpack('<f', buffer[ptr+4:ptr+8])[0]
            dopp_unit = struct.unpack('<f', buffer[ptr+8:ptr+12])[0]
            range_unit= struct.unpack('<f', buffer[ptr+12:ptr+16])[0]
            snr_unit  = struct.unpack('<f', buffer[ptr+16:ptr+20])[0]
            ptr += 20

            # Each point is 8 bytes: elevation(int8), azimuth(int8), doppler(int16), range(int16), snr(int16)
            point_size = 8
            num_points = (tlv_len - 20) // point_size

            for i in range(num_points):
                if ptr + point_size > tlv_end:
                    break
                elev_raw = int8_t(buffer[ptr])
                az_raw   = int8_t(buffer[ptr+1])
                dopp_raw = int16_t(buffer[ptr+2], buffer[ptr+3])
                range_raw= int16_t(buffer[ptr+4], buffer[ptr+5])
                snr_raw  = int16_t(buffer[ptr+6], buffer[ptr+7])
                ptr += point_size

                # Apply scaling
                elevation = elev_raw * elev_unit          # radians
                azimuth   = az_raw * az_unit              # radians
                doppler   = dopp_raw * dopp_unit          # m/s (not used here)
                rng       = range_raw * range_unit        # meters
                snr       = snr_raw * snr_unit            # ratio (linear)

                # Convert spherical coordinates to Cartesian (sensor coordinates)
                # Sensor: X = left/right, Y = forward, Z = up
                cos_elev = np.cos(elevation)
                x = rng * cos_elev * np.sin(azimuth)
                y = rng * cos_elev * np.cos(azimuth)
                z = rng * np.sin(elevation)

                points.append({
                    'x': x,
                    'y': y,
                    'z': z,
                    'azimuth': azimuth,       # radians
                    'snr': snr
                })

            # We have parsed the point cloud TLV; ignore remaining TLVs in this frame
            break   # exit TLV loop after processing point cloud

        else:
            # Skip other TLV types
            ptr = tlv_end

    # Success: return points and the number of bytes consumed (the whole packet)
    return True, frame_number, points, total_packet_len

# ----------------------------------------------------------------------------
# Main data acquisition loop
# ----------------------------------------------------------------------------
def main():
    # Send configuration
    send_config(CLI_PORT, CONFIG_FILE)

    # Open data port
    print(f"Opening data port {DATA_PORT} at 921600 baud...")
    data_ser = serial.Serial(DATA_PORT, 921600, timeout=1)
    time.sleep(2)
    data_ser.reset_input_buffer()

    # Prepare CSV file with header
    csv_file = open(CSV_OUTPUT, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    # Write header row
    csv_writer.writerow(['timestamp', 'frame', 'point_id', 'x_m', 'y_m', 'z_m', 'azimuth_rad', 'snr'])

    buffer = bytearray()
    frame_count = 0

    print("\nListening for data... Press Ctrl+C to stop.\n")
    try:
        while frame_count < MAX_FRAMES:
            # Read available bytes
            if data_ser.in_waiting:
                buffer.extend(data_ser.read(data_ser.in_waiting))

            # Try to parse a frame from the buffer
            success, frame_num, points, bytes_consumed = parse_frame(buffer)
            if success:
                # Remove the processed frame from buffer
                buffer = buffer[bytes_consumed:]

                # Write points to CSV
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                for pid, pt in enumerate(points):
                    csv_writer.writerow([
                        timestamp, frame_num, pid,
                        f"{pt['x']:.6f}", f"{pt['y']:.6f}", f"{pt['z']:.6f}",
                        f"{pt['azimuth']:.6f}", f"{pt['snr']:.2f}"
                    ])
                csv_file.flush()
                frame_count += 1
                print(f"Frame {frame_num} ({frame_count}/{MAX_FRAMES}): {len(points)} points")

            else:
                # No complete frame yet, wait a bit
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    finally:
        # Cleanup
        data_ser.close()
        csv_file.close()
        print(f"\nData saved to {CSV_OUTPUT}")
        print(f"Total frames captured: {frame_count}")

if __name__ == '__main__':
    main()