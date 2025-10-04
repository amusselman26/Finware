#!/usr/bin/env python3
import argparse
import csv
import os
import struct

# Little-endian layout that matches the *natural* C layout on STM32F4 (Cortex-M4, 32-bit).
# Types:
#   Q = uint64_t, I = uint32_t, i = int32_t (C long on STM32F4), f = float, B = uint8_t, x = pad byte
#
# SensorsSnapshot {
#   uint64_t t_us;
#   IMU_Sample imu {
#     uint64_t t_us;
#     float q[4];
#     uint8_t calib;
#     uint8_t pad[3];
#     uint32_t seq;
#   };
#   BARO_Sample baro {
#     uint64_t t_us;
#     float pressure_hPa, temperature_C, altitude_m;
#     uint32_t seq;
#   };
#   GNSS_Sample gnss {
#     uint64_t t_us;
#     int32_t lat, lon;     // degrees * 1e7
#     float alt_m, speed_mps, heading_deg;
#     uint8_t sats_used;
#     uint8_t pad[3];
#     uint32_t seq;
#   };
#   uint8_t pad[4];         // tail padding so size is multiple of 8
# }

RECORD_FMT = (
    "<"      # little-endian, standard sizes, NO implicit alignment padding
    "Q"      # snapshot.t_us
    # IMU_Sample
    "Q"      # imu.t_us
    "4f"     # imu.q[0..3]
    "B"      # imu.calib
    "3x"     # padding to align imu.seq (C adds this; we model it explicitly)
    "I"      # imu.seq
    # BARO_Sample
    "Q"      # baro.t_us
    "f"      # baro.pressure_hPa
    "f"      # baro.temperature_C
    "f"      # baro.altitude_m
    "I"      # baro.seq
    # GNSS_Sample
    "Q"      # gnss.t_us
    "i"      # gnss.lat (int32)
    "i"      # gnss.lon (int32)
    "f"      # gnss.alt_m
    "f"      # gnss.speed_mps
    "f"      # gnss.heading_deg
    "B"      # gnss.sats_used
    "3x"     # padding before gnss.seq
    "I"      # gnss.seq
    # Tail pad to 8-byte multiple (C often does this for the whole struct)
    "4x"
)

RECORD_SIZE = struct.calcsize(RECORD_FMT)  # should be 104

FIELDS = [
    # snapshot
    "t_us",
    # imu
    "imu_t_us",
    "imu_q0", "imu_q1", "imu_q2", "imu_q3",
    "imu_calib",
    "imu_seq",
    # baro
    "baro_t_us",
    "baro_pressure_hPa",
    "baro_temperature_C",
    "baro_altitude_m",
    "baro_seq",
    # gnss
    "gnss_t_us",
    "gnss_lat_e7",
    "gnss_lon_e7",
    "gnss_alt_m",
    "gnss_speed_mps",
    "gnss_heading_deg",
    "gnss_sats_used",
    "gnss_seq",
]

def unpack_record(chunk):
    v = struct.unpack(RECORD_FMT, chunk)
    # Map tuple to dict for clarity (optional)
    return {
        "t_us": v[0],
        "imu_t_us": v[1],
        "imu_q0": v[2], "imu_q1": v[3], "imu_q2": v[4], "imu_q3": v[5],
        "imu_calib": v[6],
        "imu_seq": v[7],
        "baro_t_us": v[8],
        "baro_pressure_hPa": v[9],
        "baro_temperature_C": v[10],
        "baro_altitude_m": v[11],
        "baro_seq": v[12],
        "gnss_t_us": v[13],
        "gnss_lat_e7": v[14],
        "gnss_lon_e7": v[15],
        "gnss_alt_m": v[16],
        "gnss_speed_mps": v[17],
        "gnss_heading_deg": v[18],
        "gnss_sats_used": v[19],
        "gnss_seq": v[20],
    }

def decode_file(bin_path: str, csv_path: str):
    size = os.path.getsize(bin_path)
    if size % RECORD_SIZE != 0:
        print(f"[WARN] File size ({size} B) not multiple of record size "
              f"({RECORD_SIZE} B). Last partial record will be ignored.")
    n = size // RECORD_SIZE

    with open(bin_path, "rb") as fbin, open(csv_path, "w", newline="") as fcsv:
        w = csv.writer(fcsv)
        w.writerow(FIELDS)
        count = 0
        for _ in range(n):
            chunk = fbin.read(RECORD_SIZE)
            if len(chunk) < RECORD_SIZE:
                break
            r = unpack_record(chunk)
            w.writerow([r[k] for k in FIELDS])
            count += 1

    print(f"[OK] Parsed {count} records → {csv_path}")
    print(f"[i] RECORD_SIZE = {RECORD_SIZE} bytes")

def main():
    ap = argparse.ArgumentParser(description="Decode SensorsSnapshot BIN to CSV")
    ap.add_argument("bin", help="Path to LOGxxxx.BIN")
    ap.add_argument("-o", "--out", help="CSV output path (default: same name with .csv)")
    args = ap.parse_args()

    bin_path = args.bin
    csv_path = args.out or (os.path.splitext(bin_path)[0] + ".csv")
    decode_file(bin_path, csv_path)

if __name__ == "__main__":
    main()
