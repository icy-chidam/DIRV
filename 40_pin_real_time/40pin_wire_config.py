import serial
import time
import struct
import numpy as np
import csv
import datetime

# ============================================================
# UART CONFIGURATION
# ============================================================

COM_PORT = 'COM13'

CLI_BAUD = 115200
DATA_BAUD = 921600

CSV_OUTPUT = 'radar_output.csv'

MAX_FRAMES = 100000

# ============================================================
# MAGIC WORD
# ============================================================

MAGIC_WORD = bytes([2,1,4,3,6,5,8,7])

# ============================================================
# HELPER FUNCTIONS
# ============================================================

def uint32_le(data):
    return data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24)

def int8_t(data):
    return struct.unpack('<b', bytes([data]))[0]

def int16_t(data0, data1):
    return struct.unpack('<h', bytes([data0, data1]))[0]

# ============================================================
# SEND CONFIG
# ============================================================

def send_config():

    print("\n====================================")
    print(" OPENING CLI UART ")
    print("====================================\n")

    ser = serial.Serial(
        COM_PORT,
        CLI_BAUD,
        timeout=1
    )

    print("Waiting for radar boot...\n")

    time.sleep(8)

    ser.reset_input_buffer()

    config = [

        "sensorStop",

        "flushCfg",

        "dfeDataOutputMode 1",

        "channelCfg 15 7 0",

        "adcCfg 2 1",

        "adcbufCfg -1 0 1 1 1",

        "lowPower 0 0",

        "profileCfg 0 60.75 30.00 25.00 59.10 657930 0 54.71 1 96 2950.00 2 1 36",

        "chirpCfg 0 0 0 0 0 0 0 1",

        "chirpCfg 1 1 0 0 0 0 0 2",

        "chirpCfg 2 2 0 0 0 0 0 4",

        "frameCfg 0 2 96 0 55.00 1 0",

        "dynamicRACfarCfg -1 4 4 2 2 8 12 4 8 5.00 8.00 0.40 1 1",

        "staticRACfarCfg -1 6 2 2 2 8 8 6 4 8.00 15.00 0.30 0 0",

        "dynamicRangeAngleCfg -1 0.75 0.0010 1 0",

        "dynamic2DAngleCfg -1 1.5 0.0300 1 0 1 0.30 0.85 8.00",

        "staticRangeAngleCfg -1 0 8 8",

        "antGeometry0 0 -1 -2 -3 -2 -3 -4 -5 -4 -5 -6 -7",

        "antGeometry1 -1 -1 -1 -1 0 0 0 0 -1 -1 -1 -1",

        "antPhaseRot 1 1 1 1 1 1 1 1 1 1 1 1",

        "fovCfg -1 70.0 20.0",

        "compRangeBiasAndRxChanPhase 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0",

        "staticBoundaryBox -3 3 0.5 7.5 0 3",

        "boundaryBox -4 4 0 8 0 3",

        "sensorPosition 2 0 15",

        "gatingParam 3 2 2 2 4",

        "stateParam 3 3 12 500 5 6000",

        "allocationParam 40 100 0.1 20 0.5 20",

        "maxAcceleration 0.1 0.1 0.1",

        "trackingCfg 1 2 800 30 46 96 55",

        "presenceBoundaryBox -3 3 0.5 7.5 0 3",

        "sensorStart"
    ]

    print("\nSending Configuration...\n")

    for line in config:

        print("CMD >", line)

        ser.write((line + '\r\n').encode())

        time.sleep(0.3)

        response = ser.readline()

        print("RADAR <", response)

    print("\n====================================")
    print(" CONFIGURATION SENT ")
    print("====================================\n")

    ser.close()

# ============================================================
# PARSE TLV FRAME
# ============================================================

def parse_frame(buffer):

    HEADER_SIZE = 40

    TLV_TYPE_POINTCLOUD = 1020

    magic_idx = buffer.find(MAGIC_WORD)

    if magic_idx == -1:
        return False, 0, [], 0

    if len(buffer) < magic_idx + HEADER_SIZE:
        return False, 0, [], 0

    total_packet_len = uint32_le(
        buffer[magic_idx+12:magic_idx+16]
    )

    frame_number = uint32_le(
        buffer[magic_idx+20:magic_idx+24]
    )

    num_tlvs = uint32_le(
        buffer[magic_idx+32:magic_idx+36]
    )

    if len(buffer) < magic_idx + total_packet_len:
        return False, 0, [], 0

    ptr = magic_idx + HEADER_SIZE

    points = []

    for tlv_idx in range(num_tlvs):

        if ptr + 8 > magic_idx + total_packet_len:
            break

        tlv_type = uint32_le(buffer[ptr:ptr+4])

        tlv_len = uint32_le(buffer[ptr+4:ptr+8])

        ptr += 8

        tlv_end = ptr + tlv_len

        if tlv_type == TLV_TYPE_POINTCLOUD:

            elev_unit = struct.unpack(
                '<f',
                buffer[ptr:ptr+4]
            )[0]

            az_unit = struct.unpack(
                '<f',
                buffer[ptr+4:ptr+8]
            )[0]

            dopp_unit = struct.unpack(
                '<f',
                buffer[ptr+8:ptr+12]
            )[0]

            range_unit = struct.unpack(
                '<f',
                buffer[ptr+12:ptr+16]
            )[0]

            snr_unit = struct.unpack(
                '<f',
                buffer[ptr+16:ptr+20]
            )[0]

            ptr += 20

            point_size = 8

            num_points = (tlv_len - 20) // point_size

            for i in range(num_points):

                if ptr + point_size > tlv_end:
                    break

                elev_raw = int8_t(buffer[ptr])

                az_raw = int8_t(buffer[ptr+1])

                dopp_raw = int16_t(
                    buffer[ptr+2],
                    buffer[ptr+3]
                )

                range_raw = int16_t(
                    buffer[ptr+4],
                    buffer[ptr+5]
                )

                snr_raw = int16_t(
                    buffer[ptr+6],
                    buffer[ptr+7]
                )

                ptr += point_size

                elevation = elev_raw * elev_unit

                azimuth = az_raw * az_unit

                doppler = dopp_raw * dopp_unit

                rng = range_raw * range_unit

                snr = snr_raw * snr_unit

                cos_elev = np.cos(elevation)

                x = rng * cos_elev * np.sin(azimuth)

                y = rng * cos_elev * np.cos(azimuth)

                z = rng * np.sin(elevation)

                points.append({

                    'x': x,
                    'y': y,
                    'z': z,
                    'azimuth': azimuth,
                    'snr': snr,
                    'doppler': doppler
                })

            break

        else:

            ptr = tlv_end

    return True, frame_number, points, total_packet_len

# ============================================================
# MSS LOGGER PARSER
# ============================================================

def parse_tlv():

    print("\n====================================")
    print(" MOVE RX WIRE ")
    print(" J5.5 ---> J6.9 MSSLOGGER ")
    print("====================================\n")

    input("Press ENTER after moving wire...")

    data_ser = serial.Serial(
        COM_PORT,
        DATA_BAUD,
        timeout=1
    )

    time.sleep(2)

    data_ser.reset_input_buffer()

    csv_file = open(
        CSV_OUTPUT,
        'w',
        newline=''
    )

    csv_writer = csv.writer(csv_file)

    csv_writer.writerow([

        'timestamp',
        'frame',
        'point_id',
        'x_m',
        'y_m',
        'z_m',
        'azimuth_rad',
        'snr',
        'doppler'
    ])

    buffer = bytearray()

    frame_count = 0

    print("\nListening for TLV data...\n")

    try:

        while frame_count < MAX_FRAMES:

            if data_ser.in_waiting:

                buffer.extend(
                    data_ser.read(
                        data_ser.in_waiting
                    )
                )

            success, frame_num, points, bytes_consumed = parse_frame(buffer)

            if success:

                buffer = buffer[bytes_consumed:]

                timestamp = datetime.datetime.now().strftime(
                    "%Y-%m-%d %H:%M:%S.%f"
                )[:-3]

                for pid, pt in enumerate(points):

                    csv_writer.writerow([

                        timestamp,
                        frame_num,
                        pid,

                        f"{pt['x']:.6f}",
                        f"{pt['y']:.6f}",
                        f"{pt['z']:.6f}",

                        f"{pt['azimuth']:.6f}",

                        f"{pt['snr']:.2f}",

                        f"{pt['doppler']:.3f}"
                    ])

                csv_file.flush()

                frame_count += 1

                print(
                    f"Frame {frame_num} | Points: {len(points)}"
                )

            else:

                time.sleep(0.01)

    except KeyboardInterrupt:

        print("\nStopped By User")

    finally:

        data_ser.close()

        csv_file.close()

        print(f"\nCSV Saved: {CSV_OUTPUT}")

# ============================================================
# MAIN
# ============================================================

if __name__ == '__main__':

    send_config()

    parse_tlv()