# LOGFORMAT.md — Finware (Flight Computer + Fin Control)  
**Version:** v0.1 (2025‑08‑20) • **Owner:** Finware Team • **Status:** Draft for ground + bench testing

> This document defines the on‑device log format and the matching telemetry schema used by Finware during development and testing. It is intentionally verbose to make post‑test analysis easy and unambiguous. All fields and units are **SI unless noted**.

---

## 1) File Types & Naming

- **On‑device binary** (optional): `*.bin` raw block logs for maximum throughput.  
- **Primary text logs** (recommended for testing):  
  - **CSV**: `YYYYMMDD_hhmmssZ_FINWARE_<testtag>_v<schema>.csv` (one row per tick, 10–500 Hz)  
  - **JSONL**: `YYYYMMDD_hhmmssZ_FINWARE_<testtag>_v<schema>.jsonl` (one object per record/event, mixed rates)  
- **Rotation:** start a new file when size > 100 MB or on reboot.  
- **Timebase:** first record must include `utc_time`, `monotonic_s=0.0`, and `seq=0`.

> **Example:** `20250820_1812Z_FINWARE_static-bench_v0.1.csv`

---

## 2) Coordinate Frames & Conventions

- **Body frame (b):** +x forward, +y right, +z down.  
- **Navigation frame (NED):** +North, +East, +Down.  
- **Quaternions:** `[w, x, y, z]` (Hamilton), body→world unless suffixed `_wb` (world→body).  
- **Angles:** radians. Output deg fields (e.g., `_deg`) only for convenience.  
- **Pressure alt:** derived via ISA (sea‑level 101325 Pa, 288.15 K) unless `p0_pa` supplied.  
- **GPS:** WGS‑84.  
- **NaN handling:** if a value is invalid/unavailable, write `NaN` (CSV) or omit field (JSONL).  
- **Booleans:** `0/1` in CSV, `true/false` in JSONL.

---

## 3) Sampling Rates (guidance for tests)

| Channel                   | Typical Rate |
|--------------------------|--------------|
| IMU (BNO085)             | 200–400 Hz   |
| Baro (LPS22HB)           | 25–75 Hz     |
| GNSS (u-blox)            | 5–20 Hz      |
| State Estimator (EKF)    | 100–200 Hz   |
| Control Loop             | 100–200 Hz   |
| Radio Telemetry (LoRa)   | 1–20 Hz      |

---

## 4) CSV Schema (row = tick)

> **Delimiter:** comma. **Header row present.** Fields may be a superset; consumers should tolerate unknown columns.

```
# Minimal header (first 8 columns are mandatory)
utc_time, seq, monotonic_s, dt_s, schema_ver, run_id, fw_git, vehicle_id,
mode, flight_phase,
ax_mps2, ay_mps2, az_mps2, gx_rps, gy_rps, gz_rps, mx_uT, my_uT, mz_uT,
baro_pres_pa, baro_temp_C, baro_alt_m, p0_pa,
gps_fix, gps_sats, gps_lat_deg, gps_lon_deg, gps_h_m, gps_vn_mps, gps_ve_mps, gps_vd_mps, gps_hdop, gps_vdop,
q_w, q_x, q_y, q_z, roll_deg, pitch_deg, yaw_deg,
ned_n_m, ned_e_m, ned_d_m,
v_n_mps, v_e_mps, v_d_mps, speed_mps,
fin1_cmd_rad, fin2_cmd_rad, fin3_cmd_rad, fin4_cmd_rad,
fin1_pos_rad, fin2_pos_rad, fin3_pos_rad, fin4_pos_rad,
battery_v, battery_i_a, board_temp_C,
event_code, event_note,
rc_link, rc_rssi_dbm, lora_rssi_dbm, lora_snr_db,
sd_ok, faults,
test_tag
```

### 4.1 Column Dictionary

**Core timing & metadata**  
- `utc_time` (ISO8601, e.g., `2025-08-20T18:12:34.567Z`)  
- `seq` (uint32) — sequential record index starting at 0  
- `monotonic_s` (float) — starts at 0.0 on boot  
- `dt_s` (float) — elapsed since previous row  
- `schema_ver` (string) — e.g., `v0.1`  
- `run_id` (string) — UUIDv4 or short hash for the test run  
- `fw_git` (string) — firmware commit (short SHA) + dirty flag `-dirty`  
- `vehicle_id` (string) — e.g., `DEV1`, `FLT001`  
- `test_tag` (string) — operator tag like `static-bench`, `drop-test-1`

**Modes & phases**  
- `mode` (enum): `BOOT, IDLE, TEST, ARM, FLIGHT, ABORT`  
- `flight_phase` (enum): `PAD, BOOST, COAST, APOGEE, DESCENT, FLARE, TOUCHDOWN`

**Raw sensors**  
- `ax_mps2, ay_mps2, az_mps2` — accel in body frame  
- `gx_rps, gy_rps, gz_rps` — gyro in body frame  
- `mx_uT, my_uT, mz_uT` — magnetometer in body frame (if available)
- `baro_pres_pa, baro_temp_C` — LPS22HB pressure & temperature  
- `p0_pa` — sea‑level reference pressure used for `baro_alt_m` (optional per row)

**GNSS**  
- `gps_fix` (enum): `0=no, 2=2D, 3=3D`  
- `gps_sats` (uint)  
- `gps_lat_deg, gps_lon_deg` (float)  
- `gps_h_m` (altitude MSL)  
- `gps_vn_mps, gps_ve_mps, gps_vd_mps` (NED velocities)  
- `gps_hdop, gps_vdop`

**Attitude / state estimate**  
- `q_w, q_x, q_y, q_z` — attitude quaternion (b→NED)  
- `roll_deg, pitch_deg, yaw_deg` — Euler from quaternion (for convenience)  
- `ned_n_m, ned_e_m, ned_d_m` — position vs launch NED origin  
- `v_n_mps, v_e_mps, v_d_mps` — EKF velocity NED  
- `speed_mps` — |v|

**Control**  
- `fin{1..4}_cmd_rad` — commanded deflection (rad)  
- `fin{1..4}_pos_rad` — measured deflection (rad), NaN if not instrumented

**Power & health**  
- `battery_v, battery_i_a, board_temp_C`  
- `rc_link` (0/1), `rc_rssi_dbm`  
- `lora_rssi_dbm, lora_snr_db`  
- `sd_ok` (0/1) — streaming to SD healthy  
- `faults` (bitfield as hex, e.g., `0x0004`) — see §7

**Events**  
- `event_code` (enum int; `0=NONE, 10=ARMED, 11=LAUNCH_DET, 12=BURNOUT, 13=APOGEE, 14=DROGUE, 15=MAIN, 16=TOUCHDOWN, 90=ABORT`)  
- `event_note` (string, short) — operator or auto‑annotated message

---

## 5) JSONL Records (events, sparse updates)

Each line is a compact JSON object. Use this for **asynchronous** events, mode changes, health updates, or lower‑rate subsystems.

### 5.1 Common envelope
```json
{
  "schema": "finware.v0.1",
  "t_utc": "2025-08-20T18:12:34.567Z",
  "t_mono_s": 12.345,
  "seq": 1234,
  "type": "event|health|gnss|state|control|config",
  "run_id": "b2f9…",
  "vehicle_id": "DEV1"
}
```

### 5.2 Record types

**event**  
```json
{
  "schema":"finware.v0.1","type":"event","t_utc":"2025-08-20T18:12:34.567Z","t_mono_s":42.000,"seq":4200,
  "code":11,"name":"LAUNCH_DET","note":">20 m/s"
}
```

**health**  
```json
{
  "schema":"finware.v0.1","type":"health","t_utc":"2025-08-20T18:12:35.100Z","t_mono_s":42.533,"seq":4206,
  "battery_v":11.9,"battery_i_a":1.2,"board_temp_C":35.0,"sd_ok":true,"faults":"0x0000"
}
```

**gnss**  
```json
{
  "schema":"finware.v0.1","type":"gnss","t_utc":"2025-08-20T18:12:35.200Z","t_mono_s":42.633,"seq":4210,
  "fix":3,"sats":18,"lat_deg":25.7172,"lon_deg":-80.2773,"h_m":8.4,
  "vn_mps":-0.2,"ve_mps":0.1,"vd_mps":0.0,"hdop":0.6,"vdop":0.9
}
```

**state**  
```json
{
  "schema":"finware.v0.1","type":"state","t_utc":"2025-08-20T18:12:35.300Z","t_mono_s":42.733,"seq":4215,
  "q":[0.99,0.01,-0.02,0.00],"ned":[1.2,0.5,-0.1],
  "v_ned_mps":[12.1,0.4,-0.3],"speed_mps":12.1,"mode":"FLIGHT","phase":"BOOST"
}
```

**control**  
```json
{
  "schema":"finware.v0.1","type":"control","t_utc":"2025-08-20T18:12:35.350Z","t_mono_s":42.783,"seq":4218,
  "fin_cmd_rad":[0.05,0.05,0.05,0.05],"fin_pos_rad":[0.05,0.05,0.05,0.05]
}
```

**config** (logged once per run and on changes)  
```json
{
  "schema":"finware.v0.1","type":"config","t_utc":"2025-08-20T18:12:34.000Z","t_mono_s":0.000,"seq":0,
  "fw_git":"a1b2c3d4","params":{
    "imu_rate_hz":400,"ekf_rate_hz":200,"ctrl_rate_hz":200,
    "launch_speed_mps":20.0,"p0_pa":101325.0
  }
}
```

---

## 6) Test Metadata Block (first rows / first JSONL objects)

Log the following at the start of every run (**CSV as repeated columns; JSONL as a `config` record**):

- `operator` (string initials), `location` (string), `notes`  
- `p0_pa` (ambient reference), `temp_C` (ambient)  
- `mount` (enum): `PAD, BENCH, TOWER, RIG`  
- `sim_mode` (0/1) and `hw_mask` (bitfield) if any sensors are simulated

---

## 7) Faults Bitfield (hex in logs)

| Bit | Meaning                        |
|-----|--------------------------------|
| 0   | IMU fault                      |
| 1   | Baro fault                     |
| 2   | GNSS fault                     |
| 3   | Estimator divergence           |
| 4   | Control saturation             |
| 5   | Battery low                    |
| 6   | SD write error                 |
| 7   | Overtemp                       |
| 8   | Radio link lost                |
| 9   | Config invalid                 |

---

## 8) Derived Quantities (recommended to log)

- `alt_agl_m` — altitude above launch (from NED D)  
- `drag_est_N` — from model or estimator  
- `acc_norm_g` — |a| / 9.80665  
- `mach` — requires air temp; optional

---

## 9) Bench & Ground Test Checkpoints

1. **Static bench:** verify IMU noise levels, gravity vector, gyro bias, `baro_alt_m` stability.  
2. **Tilt test:** 90° rotations, confirm quaternion continuity and Euler sanity.  
3. **GNSS sky test:** open sky, > 10 sats, position drift < 2 m (static).  
4. **Control loop dry‑run:** command step ±5° fins; ensure `pos` ≈ `cmd`, track latency.  
5. **Logging soak:** 10 minutes at full rate; no `sd_ok=0`, no `faults` bits.  
6. **Radio telemetry:** verify downlink at target rate with `seq` continuity.

---

## 10) Minimal CSV Example

```csv
utc_time,seq,monotonic_s,dt_s,schema_ver,run_id,fw_git,vehicle_id,mode,flight_phase,ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps,mx_uT,my_uT,mz_uT,baro_pres_pa,baro_temp_C,baro_alt_m,p0_pa,gps_fix,gps_sats,gps_lat_deg,gps_lon_deg,gps_h_m,gps_vn_mps,gps_ve_mps,gps_vd_mps,gps_hdop,gps_vdop,q_w,q_x,q_y,q_z,roll_deg,pitch_deg,yaw_deg,ned_n_m,ned_e_m,ned_d_m,v_n_mps,v_e_mps,v_d_mps,speed_mps,fin1_cmd_rad,fin2_cmd_rad,fin3_cmd_rad,fin4_cmd_rad,fin1_pos_rad,fin2_pos_rad,fin3_pos_rad,fin4_pos_rad,battery_v,battery_i_a,board_temp_C,event_code,event_note,rc_link,rc_rssi_dbm,lora_rssi_dbm,lora_snr_db,sd_ok,faults,test_tag
2025-08-20T18:12:34.000Z,0,0.000,0.000,v0.1,run-abc123,a1b2c3d,DEV1,IDLE,PAD,0.02,-0.01,9.80,0.001,0.000,0.000,10.2,-5.3,39.0,101200.0,31.2,1.1,101200.0,0,0,,, ,,, ,,, ,,,1.000,0.000,0.000,0.000,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,12.5,0.2,32.0,0,,1,-60,-95,9.0,1,0x0000,static-bench
```

---

## 11) Validation Rules (quick sanity checks)

- `abs(|q| - 1) < 1e-3`  
- `|roll|,|pitch| ≤ 90°` (during ground tests)  
- `dt_s` within ±20% of target period; if not, flag timing anomaly  
- If `gps_fix < 3`, NED position should be 0/NaN unless fused with baro/IMU  
- `fin_pos_rad` must remain within hardware limits; flag saturation

---

## 12) Backward Compatibility

- New fields append to the end in CSV; JSONL remains schematized by keys.  
- Breaking changes bump `schema_ver` (e.g., `v0.2`) and **must** be reflected in filename.

---

## 13) Implementation Notes (firmware)

- Log **CSV** on every control tick (e.g., 200 Hz) or decimate to 100 Hz if bandwidth constrained.  
- Burst‑write in blocks (≥ 1 kB) to SD to reduce wear.  
- Prewrite header row once; enforce consistent column order.  
- Emit `config` JSONL entry at start and on parameter changes.

---

## 14) Appendix — Enumerations

```
mode: BOOT=0, IDLE=1, TEST=2, ARM=3, FLIGHT=4, ABORT=5
flight_phase: PAD=0, BOOST=1, COAST=2, APOGEE=3, DESCENT=4, FLARE=5, TOUCHDOWN=6
event_code: NONE=0, ARMED=10, LAUNCH_DET=11, BURNOUT=12, APOGEE=13, DROGUE=14, MAIN=15, TOUCHDOWN=16, ABORT=90
```

---

## 15) To‑Do for v0.2

- Add pyro channel currents & continuity (`pyro1_cont, pyro1_i_a, …`)  
- Add environmental (`humid_rel_pct`, `wind_est_mps`)  
- Add thrust stand force (for static motor tests)
