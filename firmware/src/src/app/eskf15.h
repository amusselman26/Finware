#pragma once
#include <stdint.h>
#include <math.h>

// Minimal 15-state ESKF:
// Nominal: p(3), v(3), q(4), bg(3), ba(3)
// Error:   dp(3), dv(3), dtheta(3), dbg(3), dba(3)

namespace eskf {

// ------------------------ Small math types ------------------------

struct Vec3 {
  float x, y, z;

  Vec3() : x(0), y(0), z(0) {}
  Vec3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}

  Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
  Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
  Vec3 operator*(float s) const { return Vec3(x*s, y*s, z*s); }

  Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
  Vec3& operator-=(const Vec3& o){ x-=o.x; y-=o.y; z-=o.z; return *this; }
};

inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
inline float dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline float norm(const Vec3& v) { return sqrtf(dot(v,v)); }

struct Quat {
  // scalar-first (w, x, y, z)
  float w, x, y, z;

  Quat() : w(1), x(0), y(0), z(0) {}
  Quat(float W, float X, float Y, float Z) : w(W), x(X), y(Y), z(Z) {}

  static Quat identity() { return Quat(1,0,0,0); }

  Quat operator*(const Quat& q) const;
  void normalize();

  // Rotate a vector from body to nav: v_nav = q * v_body * q^{-1}
  Vec3 rotate(const Vec3& v_body) const;

  // Convert raw IMU quaternion (ENU body->nav) to NED body->nav.
  static Quat enuBodyToNavToNedBodyToNav(const Quat& q_enu_bn);

  // Exponential map for small angle (rad): Exp(dtheta) ~ [1, 0.5*dtheta]
  static Quat expSmall(const Vec3& dtheta);
};

// ------------------------ Fixed-size matrices ------------------------

template<int R, int C>
struct Mat {
  float a[R][C];

  void setZero() {
    for (int i=0;i<R;i++) for (int j=0;j<C;j++) a[i][j]=0.0f;
  }

  static Mat<R,C> Zero() { Mat<R,C> m; m.setZero(); return m; }

  static Mat<R,C> Identity() {
    Mat<R,C> m; m.setZero();
    const int n = (R < C) ? R : C;
    for (int i=0;i<n;i++) m.a[i][i]=1.0f;
    return m;
  }
};

template<int R, int C>
Mat<R,C> operator+(const Mat<R,C>& A, const Mat<R,C>& B) {
  Mat<R,C> M;
  for(int i=0;i<R;i++) for(int j=0;j<C;j++) M.a[i][j] = A.a[i][j] + B.a[i][j];
  return M;
}

template<int R, int C>
Mat<R,C> operator-(const Mat<R,C>& A, const Mat<R,C>& B) {
  Mat<R,C> M;
  for(int i=0;i<R;i++) for(int j=0;j<C;j++) M.a[i][j] = A.a[i][j] - B.a[i][j];
  return M;
}

template<int R, int C>
Mat<R,C> operator*(float s, const Mat<R,C>& A) {
  Mat<R,C> M;
  for(int i=0;i<R;i++) for(int j=0;j<C;j++) M.a[i][j] = s * A.a[i][j];
  return M;
}

template<int R, int K, int C>
Mat<R,C> mul(const Mat<R,K>& A, const Mat<K,C>& B) {
  Mat<R,C> M; M.setZero();
  for(int i=0;i<R;i++){
    for(int k=0;k<K;k++){
      const float aik = A.a[i][k];
      for(int j=0;j<C;j++){
        M.a[i][j] += aik * B.a[k][j];
      }
    }
  }
  return M;
}

template<int R, int C>
Mat<C,R> transpose(const Mat<R,C>& A) {
  Mat<C,R> M;
  for(int i=0;i<R;i++) for(int j=0;j<C;j++) M.a[j][i] = A.a[i][j];
  return M;
}

// Invert small NxN (N <= 6) with Gauss-Jordan. Returns false if singular.
template<int N>
bool invertSmall(const Mat<N,N>& A, Mat<N,N>& Ainv);

// ------------------------ ESKF 15 ------------------------

class ESKF15 {
public:
  enum UpdateMask : uint16_t {
    UPD_DP  = 1 << 0,
    UPD_DV  = 1 << 1,
    UPD_DTH = 1 << 2,
    UPD_DBG = 1 << 3,
    UPD_DBA = 1 << 4,
    UPD_ALL = UPD_DP | UPD_DV | UPD_DTH | UPD_DBG | UPD_DBA
  };

  // Nominal state (NED frame recommended; gravity set accordingly)
  Vec3 p;     // position
  Vec3 v;     // velocity
  Quat q;     // attitude (body->nav)
  Vec3 bg;    // gyro bias
  Vec3 ba;    // accel bias

  // Error covariance
  Mat<15,15> P;

  // Noise params (continuous-time std dev)
  float sigma_gyr;    // rad/s / sqrt(Hz)
  float sigma_acc;    // m/s^2 / sqrt(Hz)
  float sigma_bg_rw;  // rad/s^2 / sqrt(Hz) (gyro bias random walk)
  float sigma_ba_rw;  // m/s^3 / sqrt(Hz) (accel bias random walk)

  // Gravity in nav frame (NED default: +9.80665 in +Down)
  Vec3 g;

  bool heading_observable = false;  // set true when you have mag or GPS course while moving

  ESKF15();

  void reset();

  // Predict with IMU measurements (body frame)
  void predict(float dt, const Vec3& gyr_meas_radps, const Vec3& acc_meas_mps2);

  // Generic measurement update: residual r = z - h(x), H = dh/dx_err
  // r is mx1, H is mx15, R is mxm
  template<int M>
  void update(const Mat<M,15>& H, const Mat<M,1>& r, const Mat<M,M>& R);

  template<int M>
  void updateMasked(const Mat<M,15>& H, const Mat<M,1>& r, const Mat<M,M>& R,
                    uint16_t mask);

  // Helpers
  void updateGPSPosVel(const Vec3& pos_meas, const Vec3& vel_meas,
                       float sigma_pos, float sigma_vel);

  // Baro altitude (Up-positive). Assumes p.z is Down (NED).
  void updateBaroAlt(float alt_m_up, float sigma_alt);

private:
  static Mat<3,3> skew(const Vec3& w);
  Mat<3,3> Rnb() const; // rotation matrix body->nav

  void inject_(const Mat<15,1>& dx);
  void resetErrorState_(const Vec3& dtheta);
};

} // namespace eskf