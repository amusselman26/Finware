#!/usr/bin/env python3
import argparse, csv, os, struct

# Little-endian, explicit padding to match natural C layout on Cortex-M (STM32F4).
# Types: Q=uint64, I=uint32, i=int32, f=float, B=uint8, x=pad byte

# C structs (summary):
# SensorsSnapshot {
#   uint64_t t_us;
#   SystemState state;     // enum class -> 4-byte int
#   float     batteryVoltage;
#   IMU_Sample imu {
#     uint64_t t_us;
#     float q[4];
#     uint8_t calib; uint8_t pad[3];
#     float ax, ay, az;
#     float gx, gy, gz;
#     uint32_t seq;
#   };                   // 56 B
#   BARO_Sample baro {   // 24 B
#     uint64_t t_us;
#     float pressure_hPa, temperature_C, altitude_m;
#     uint32_t seq;
#   };
#   GNSS_Sample gnss {   // 36 B
#     uint64_t t_us;
#     int32_t lat, lon;  // deg*1e7
#     float alt_m, speed_mps, heading_deg;
#     uint8_t sats_used; uint8_t pad[3];
#     uint32_t seq;
#   };
#   uint8_t pad[4];      // tail pad to 8-byte multiple
# }; // total = 136 B

RECORD_FMT = (
    "<"
    # snapshot
    "Q"      # t_us
    "I"      # state (SystemState underlying int32/uint32)
    "f"      # batteryVoltage
    # IMU
    "Q"      # imu.t_us
    "4f"     # imu.q[4]
    "B"      # imu.calib
    "3x"     # pad
    "3f"     # imu.ax, imu.ay, imu.az
    "3f"     # imu.gx, imu.gy, imu.gz
    "I"      # imu.seq
    # BARO
    "Q"      # baro.t_us
    "f"      # baro.pressure_hPa
    "f"      # baro.temperature_C
    "f"      # baro.altitude_m
    "I"      # baro.seq
    # GNSS
    "Q"      # gnss.t_us
    "i"      # gnss.lat_e7
    "i"      # gnss.lon_e7
    "f"      # gnss.alt_m
    "f"      # gnss.speed_mps
    "f"      # gnss.heading_deg
    "B"      # gnss.sats_used
    "3x"     # pad before seq
    "I"      # gnss.seq
    # tail pad to 8-byte boundary
    "4x"
)

RECORD_SIZE = struct.calcsize(RECORD_FMT)  # should be 136 with added SystemState

FIELDS = [
    # snapshot
    "t_us",
    # system state
    "state",
    "batteryVoltage",
    # imu
    "imu_t_us",
    "imu_q0","imu_q1","imu_q2","imu_q3",
    "imu_calib",
    "imu_ax","imu_ay","imu_az",
    "imu_gx","imu_gy","imu_gz",
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

def decode_file(bin_path: str, csv_path: str):
    size = os.path.getsize(bin_path)
    if size % RECORD_SIZE != 0:
        print(f"[WARN] File size {size} B is not a multiple of record size "
              f"{RECORD_SIZE} B. Trailing bytes will be ignored.")
    n_records = size // RECORD_SIZE

    with open(bin_path, "rb") as fbin, open(csv_path, "w", newline="") as fcsv:
        w = csv.writer(fcsv)
        w.writerow(FIELDS)
        count = 0
        for _ in range(n_records):
            chunk = fbin.read(RECORD_SIZE)
            if len(chunk) < RECORD_SIZE:
                break
            vals = struct.unpack(RECORD_FMT, chunk)
            w.writerow(vals)
            count += 1

    print(f"[OK] Wrote {count} records to {csv_path} (RECORD_SIZE={RECORD_SIZE} B)")

def main():
    ap = argparse.ArgumentParser(description="Decode SensorsSnapshot BIN → CSV")
    ap.add_argument("bin", help="Path to LOGxxxx.BIN")
    ap.add_argument("-o", "--out", help="Output CSV path (default: same name with .csv)")
    args = ap.parse_args()

    bin_path = args.bin
    csv_path = args.out or (os.path.splitext(bin_path)[0] + ".csv")
    decode_file(bin_path, csv_path)

if __name__ == "__main__":
    main()
