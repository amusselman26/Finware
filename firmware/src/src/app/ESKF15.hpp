#pragma once
#include <stdint.h>
#include <math.h>

// =======================
// 15-state Error-State EKF (ESKF) for rocket FC
// Nominal: p,v,q,bg,ba
// Error-state: dp,dv,dtheta,dbg,dba  (15x1)
// =======================

namespace finware {

struct Vec3 {
  float x, y, z;
};

struct Quat {
  // scalar-first: w, x, y, z
  float w, x, y, z;
};

struct ESKF15Config {
  // Gravity magnitude (positive), NED frame uses +Down, so gravity is +g in Down axis.
  float g = 9.80665f;

  // -------- Process noise (continuous-time intensities) --------
  // These are "power spectral density" style knobs (tune).
  float sigma_accel = 0.8f;      // m/s^2 / sqrt(Hz)  (IMU accel noise)
  float sigma_gyro  = 0.03f;     // rad/s / sqrt(Hz)  (IMU gyro noise)

  float sigma_bg_rw = 0.002f;    // rad/s^2 / sqrt(Hz)  gyro bias random walk
  float sigma_ba_rw = 0.05f;     // m/s^3 / sqrt(Hz)    accel bias random walk

  // -------- Measurement noise (discrete) --------
  float sigma_gps_pos = 2.5f;    // m (1-sigma)
  float sigma_gps_vel = 0.4f;    // m/s
  float sigma_baro_z  = 1.5f;    // m

  // Quaternion measurement noise -> mapped to small-angle (rad)
  float sigma_att_meas = 0.12f;  // rad (about ~7 deg). Make smaller if you trust BNO085 more.

  // Optional: reject outliers by normalized innovation
  float nis_gate_gps_pos = 25.0f;  // ~ chi2 gate (3 dof) conservative
  float nis_gate_gps_vel = 25.0f;  // (3 dof)
  float nis_gate_baro    = 16.0f;  // (1 dof)
  float nis_gate_att     = 25.0f;  // (3 dof)
};

struct GPSMeas {
  bool valid = false;
  Vec3 p_ned_m;  // position in NED meters
  Vec3 v_ned_mps;
};

struct BaroMeas {
  bool valid = false;
  float z_down_m;  // NED down position (positive down). If you use altitude-up, convert.
};

struct AttMeas {
  bool valid = false;
  Quat q_nb; // measured body->nav quaternion (scalar-first)
};

class ESKF15 {
public:
  explicit ESKF15(const ESKF15Config& cfg = ESKF15Config());

  void reset(const Vec3& p_ned, const Vec3& v_ned, const Quat& q_nb,
             const Vec3& bg, const Vec3& ba);

  // Predict using IMU samples in BODY frame:
  // gyro_radps: body rates (p,q,r) in rad/s
  // accel_mps2: specific force in m/s^2 (what IMU gives as "linear accel" is NOT ideal;
  //            best is raw accel including gravity, but you can still run with linear accel;
  //            see notes below).
  void predict(float dt, const Vec3& gyro_radps, const Vec3& accel_mps2);

  // Updates
  bool updateGPS(const GPSMeas& gps);
  bool updateBaro(const BaroMeas& baro);
  bool updateAttitude(const AttMeas& att); // uses q_meas as a measurement of nominal attitude

  // Accessors
  Vec3 positionNED() const { return _p; }
  Vec3 velocityNED() const { return _v; }
  Quat attitude_q_nb() const { return _q; }
  Vec3 gyroBias() const { return _bg; }
  Vec3 accelBias() const { return _ba; }

  // Covariance access (15x15)
  const float* Pdata() const { return &Pcov[0][0]; }

private:
  ESKF15Config _cfg;

  // Nominal state
  Vec3 _p{};
  Vec3 _v{};
  Quat _q{1,0,0,0};
  Vec3 _bg{};
  Vec3 _ba{};

  // Error covariance
  float Pcov[15][15]{};

  // Helpers: math
  static inline Vec3 vadd(const Vec3& a, const Vec3& b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
  static inline Vec3 vsub(const Vec3& a, const Vec3& b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
  static inline Vec3 vmul(const Vec3& a, float s) { return {a.x*s, a.y*s, a.z*s}; }

  static inline float dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
  static inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
  }

  static Quat qmul(const Quat& a, const Quat& b);
  static Quat qconj(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }
  static Quat qnorm(const Quat& q);
  static Vec3 quatRotate(const Quat& q_nb, const Vec3& v_b); // v_n = R_nb * v_b
  static void quatToDCM(const Quat& q, float R[3][3]);

  static Quat smallAngleQuat(const Vec3& dtheta); // exp(0.5*dtheta)
  static void skew(const Vec3& a, float S[3][3]);

  // Core EKF update for H picking rows (general small dims)
  // z = h(x) + v, residual r = z - h(x_nom), H is (m x 15), R is (m x m)
  // returns true if accepted (passes gate)
  bool updateGeneric(const float* H, int m, const float* r, const float* R, float nis_gate);

  // Apply error-state correction to nominal state, then reset error-state to zero and adjust P
  void injectAndReset(const float dx[15]);

  // Matrix utilities (small, fixed)
  static void matIdentity(float* A, int n);
  static void matZero(float* A, int r, int c);

  static void matMul(const float* A, int Ar, int Ac,
                     const float* B, int Br, int Bc,
                     float* C); // C = A*B

  static void matMulT_B(const float* A, int Ar, int Ac,
                        const float* B, int Br, int Bc,
                        float* C); // C = A * B^T

  static void matMulT_A(const float* A, int Ar, int Ac,
                        const float* B, int Br, int Bc,
                        float* C); // C = A^T * B

  static bool matInv(float* A, int n); // in-place invert (Gauss-Jordan), returns success
};

} // namespace finware