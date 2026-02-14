#!/usr/bin/env python3
import argparse, csv, os, struct

# FlightRecord = SensorsSnapshot (136 B) + float fin_cmd_rad (4 B) = 140 B

RECORD_FMT = (
    "<"
    # ---- SensorsSnapshot ----
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
    # tail pad to 8-byte boundary inside SensorsSnapshot
    "4x"
    # ---- FlightRecord extra ----
    "f"      # fin_cmd_rad
)

RECORD_SIZE = struct.calcsize(RECORD_FMT)  # should be 140

FIELDS = [
    # snapshot
    "t_us",
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
    # flight record extras
    "fin_cmd_rad",
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
    ap = argparse.ArgumentParser(description="Decode FlightRecord BIN → CSV")
    ap.add_argument("bin", help="Path to LOGxxxx.BIN")
    ap.add_argument("-o", "--out", help="Output CSV path (default: same name with .csv)")
    args = ap.parse_args()

    bin_path = args.bin
    csv_path = args.out or (os.path.splitext(bin_path)[0] + ".csv")
    decode_file(bin_path, csv_path)

if __name__ == "__main__":
    main()
