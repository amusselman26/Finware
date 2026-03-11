#include "eskf15.h"

namespace eskf {

// ------------------------ Quat ------------------------

Quat Quat::operator*(const Quat& q2) const {
  // Hamilton product
  return Quat(
    w*q2.w - x*q2.x - y*q2.y - z*q2.z,
    w*q2.x + x*q2.w + y*q2.z - z*q2.y,
    w*q2.y - x*q2.z + y*q2.w + z*q2.x,
    w*q2.z + x*q2.y - y*q2.x + z*q2.w
  );
}

void Quat::normalize() {
  const float n = sqrtf(w*w + x*x + y*y + z*z);
  if (n > 0.0f) { w/=n; x/=n; y/=n; z/=n; }
  else { w=1; x=y=z=0; }
}

Vec3 Quat::rotate(const Vec3& v) const {
  // q * [0,v] * q^{-1}
  const Quat qv(0, v.x, v.y, v.z);
  const Quat qi(w, -x, -y, -z); // inverse for unit quat
  const Quat r = (*this) * qv * qi;
  return Vec3(r.x, r.y, r.z);
}

Quat Quat::expSmall(const Vec3& dtheta) {
  // For small angles: Exp(dtheta) ~ [cos(|d|/2), sin(|d|/2) * d/|d|]
  const float a = norm(dtheta);
  if (a < 1e-6f) {
    return Quat(1.0f, 0.5f*dtheta.x, 0.5f*dtheta.y, 0.5f*dtheta.z);
  }
  const float half = 0.5f * a;
  const float s = sinf(half) / a;
  return Quat(cosf(half), s*dtheta.x, s*dtheta.y, s*dtheta.z);
}

// ------------------------ Matrix inversion ------------------------

template<int N>
bool invertSmall(const Mat<N,N>& A, Mat<N,N>& Ainv) {
  // Augment [A | I] and Gauss-Jordan
  Mat<N,2*N> aug; aug.setZero();
  for(int i=0;i<N;i++){
    for(int j=0;j<N;j++) aug.a[i][j] = A.a[i][j];
    aug.a[i][N+i] = 1.0f;
  }

  for(int col=0; col<N; col++){
    // pivot
    int pivot = col;
    float best = fabsf(aug.a[col][col]);
    for(int r=col+1;r<N;r++){
      float v = fabsf(aug.a[r][col]);
      if (v > best) { best=v; pivot=r; }
    }
    if (best < 1e-9f) return false;

    // swap
    if (pivot != col){
      for(int j=0;j<2*N;j++){
        float tmp = aug.a[col][j];
        aug.a[col][j] = aug.a[pivot][j];
        aug.a[pivot][j] = tmp;
      }
    }

    // normalize row
    const float invp = 1.0f / aug.a[col][col];
    for(int j=0;j<2*N;j++) aug.a[col][j] *= invp;

    // eliminate others
    for(int r=0;r<N;r++){
      if (r==col) continue;
      const float f = aug.a[r][col];
      if (f == 0.0f) continue;
      for(int j=0;j<2*N;j++){
        aug.a[r][j] -= f * aug.a[col][j];
      }
    }
  }

  // extract
  for(int i=0;i<N;i++) for(int j=0;j<N;j++) Ainv.a[i][j] = aug.a[i][N+j];
  return true;
}

// Explicit instantiations used (1,3,6)
template bool invertSmall<1>(const Mat<1,1>&, Mat<1,1>&);
template bool invertSmall<3>(const Mat<3,3>&, Mat<3,3>&);
template bool invertSmall<6>(const Mat<6,6>&, Mat<6,6>&);

// ------------------------ ESKF15 ------------------------

ESKF15::ESKF15() {
  reset();
}

void ESKF15::reset() {
  p = Vec3(0,0,0);
  v = Vec3(0,0,0);
  q = Quat::identity();
  bg = Vec3(0,0,0);
  ba = Vec3(0,0,0);

  P = Mat<15,15>::Identity();

  // Reasonable defaults (tune these!)
  sigma_gyr   = 0.02f;   // rad/s/sqrt(Hz)
  sigma_acc   = 0.20f;   // m/s^2/sqrt(Hz)
  sigma_bg_rw = 1e-5f;   // rad/s^2/sqrt(Hz)
  sigma_ba_rw = 0.01f;   // m/s^3/sqrt(Hz)

  // NED: +Down gravity
  g = Vec3(0,0,9.80665f);
}

Mat<3,3> ESKF15::skew(const Vec3& w) {
  Mat<3,3> S; S.setZero();
  S.a[0][1] = -w.z; S.a[0][2] =  w.y;
  S.a[1][0] =  w.z; S.a[1][2] = -w.x;
  S.a[2][0] = -w.y; S.a[2][1] =  w.x;
  return S;
}

Mat<3,3> ESKF15::Rnb() const {
  // Rotation matrix from body to nav given unit quaternion (w,x,y,z)
  const float w = q.w, x = q.x, y = q.y, z = q.z;
  Mat<3,3> R; R.setZero();

  R.a[0][0] = 1 - 2*(y*y + z*z);
  R.a[0][1] = 2*(x*y - w*z);
  R.a[0][2] = 2*(x*z + w*y);

  R.a[1][0] = 2*(x*y + w*z);
  R.a[1][1] = 1 - 2*(x*x + z*z);
  R.a[1][2] = 2*(y*z - w*x);

  R.a[2][0] = 2*(x*z - w*y);
  R.a[2][1] = 2*(y*z + w*x);
  R.a[2][2] = 1 - 2*(x*x + y*y);

  return R;
}

void ESKF15::predict(float dt, const Vec3& gyr_meas, const Vec3& acc_meas) {
  // --- Nominal propagation ---
  const Vec3 omega = gyr_meas - bg;   // rad/s
  const Vec3 f_b   = acc_meas - ba;   // m/s^2 (specific force in body)

  // Attitude: q <- q * Exp(omega*dt)
  const Vec3 dtheta = omega * dt;
  q = q * Quat::expSmall(dtheta);
  q.normalize();

  // Accel to nav + gravity
  const Vec3 a_n = q.rotate(f_b) + g;

  // Integrate p,v
  p += v*dt + a_n*(0.5f*dt*dt);
  v += a_n*dt;

  // --- Covariance propagation (first-order discretization) ---
  // Error-state ordering: [dp(0:2), dv(3:5), dth(6:8), dbg(9:11), dba(12:14)]

  Mat<15,15> F = Mat<15,15>::Zero();

  // dpdot = dv
  for(int i=0;i<3;i++) F.a[i][3+i] = 1.0f;

  // dvdot = -R * skew(f_b) * dtheta - R*dba
  const Mat<3,3> R = Rnb();
  const Mat<3,3> Sf = skew(f_b);
  const Mat<3,3> R_Sf = mul(R, Sf);
  for(int r=0;r<3;r++){
    for(int c=0;c<3;c++){
      F.a[3+r][6+c]  = -R_Sf.a[r][c];  // v,theta
      F.a[3+r][12+c] = -R.a[r][c];     // v,ba
    }
  }

  // dthetadot = -skew(omega)*dtheta - dbg
  const Mat<3,3> Sw = skew(omega);
  for(int r=0;r<3;r++){
    for(int c=0;c<3;c++){
      F.a[6+r][6+c] = -Sw.a[r][c]; // theta,theta
    }
    F.a[6+r][9+r] = -1.0f;        // theta,bg
  }

  // Noise mapping G (15x12): [ng(3), na(3), nbg(3), nba(3)]
  Mat<15,12> G; G.setZero();

  // theta <- -ng
  for(int i=0;i<3;i++) G.a[6+i][i] = -1.0f;

  // v <- -R*na
  for(int r=0;r<3;r++) for(int c=0;c<3;c++) G.a[3+r][3+c] = -R.a[r][c];

  // bg <- nbg
  for(int i=0;i<3;i++) G.a[9+i][6+i] = 1.0f;

  // ba <- nba
  for(int i=0;i<3;i++) G.a[12+i][9+i] = 1.0f;

  // Continuous noise covariance Qc (12x12 diagonal)
  Mat<12,12> Qc = Mat<12,12>::Zero();
  const float sg2  = sigma_gyr*sigma_gyr;
  const float sa2  = sigma_acc*sigma_acc;
  const float sbg2 = sigma_bg_rw*sigma_bg_rw;
  const float sba2 = sigma_ba_rw*sigma_ba_rw;
  for(int i=0;i<3;i++){
    Qc.a[i][i]     = sg2;
    Qc.a[3+i][3+i] = sa2;
    Qc.a[6+i][6+i] = sbg2;
    Qc.a[9+i][9+i] = sba2;
  }

  // Discretize: Phi ≈ I + F dt, Qd ≈ G Qc G^T dt
  Mat<15,15> Phi = Mat<15,15>::Identity();
  for(int i=0;i<15;i++) for(int j=0;j<15;j++) Phi.a[i][j] += F.a[i][j]*dt;

  const Mat<15,12> GQc = mul(G, Qc);
  Mat<15,15> Qd  = mul(GQc, transpose(G));
  for(int i=0;i<15;i++) for(int j=0;j<15;j++) Qd.a[i][j] *= dt;

  const Mat<15,15> P1  = mul(Phi, mul(P, transpose(Phi)));
  P = P1 + Qd;

  // Symmetrize (numerical hygiene)
  for(int i=0;i<15;i++){
    for(int j=i+1;j<15;j++){
      const float s = 0.5f*(P.a[i][j] + P.a[j][i]);
      P.a[i][j] = s; P.a[j][i] = s;
    }
  }
}

template<int M>
void ESKF15::update(const Mat<M,15>& H, const Mat<M,1>& r, const Mat<M,M>& Rm) {
  // S = H P H^T + R
  const Mat<M,15> HP  = mul(H, P);
  Mat<M,M> S = mul(HP, transpose(H)) + Rm;

  Mat<M,M> Sinv;
  static_assert(M <= 6, "invertSmall supports M<=6");
  if (!invertSmall<M>(S, Sinv)) return;

  // K = P H^T S^-1
  const Mat<15,M> PHt = mul(P, transpose(H));
  const Mat<15,M> K   = mul(PHt, Sinv);

  // dx = K r
  const Mat<15,1> dx = mul(K, r);

  // Inject into nominal
  inject_(dx);

  // Joseph form: P = (I-KH)P(I-KH)^T + K R K^T
  const Mat<15,15> I = Mat<15,15>::Identity();
  const Mat<15,15> KH = mul(K, H);
  const Mat<15,15> A  = I - KH;

  const Mat<15,15> APAT = mul(A, mul(P, transpose(A)));
  const Mat<15,15> KRKt = mul(K, mul(Rm, transpose(K)));
  P = APAT + KRKt;

  // Symmetrize
  for(int i=0;i<15;i++){
    for(int j=i+1;j<15;j++){
      const float s = 0.5f*(P.a[i][j] + P.a[j][i]);
      P.a[i][j] = s; P.a[j][i] = s;
    }
  }
}

template<int M>
void ESKF15::updateMasked(const Mat<M,15>& H, const Mat<M,1>& r, const Mat<M,M>& Rm,
                          uint16_t mask) {
  // S = HPH^T + R
  const Mat<M,15> HP  = mul(H, P);
  Mat<M,M> S = mul(HP, transpose(H)) + Rm;

  Mat<M,M> Sinv;
  static_assert(M <= 6, "invertSmall supports M<=6");
  if (!invertSmall<M>(S, Sinv)) return;

  // K = P H^T S^-1
  const Mat<15,M> PHt = mul(P, transpose(H));
  Mat<15,M> K = mul(PHt, Sinv);

  // Mask out state blocks we don't want to correct
  auto zero_rows = [&](int r0, int r1){
    for(int rr=r0; rr<=r1; rr++)
      for(int c=0;c<M;c++) K.a[rr][c] = 0.0f;
  };

  if (!(mask & UPD_DP))  zero_rows(0,2);
  if (!(mask & UPD_DV))  zero_rows(3,5);
  if (!(mask & UPD_DTH)) zero_rows(6,8);
  if (!(mask & UPD_DBG)) zero_rows(9,11);
  if (!(mask & UPD_DBA)) zero_rows(12,14);

  // dx = K r
  const Mat<15,1> dx = mul(K, r);

  // Inject correction
  inject_(dx);

  // Joseph form covariance update
  const Mat<15,15> I = Mat<15,15>::Identity();
  const Mat<15,15> KH = mul(K, H);
  const Mat<15,15> A  = I - KH;

  const Mat<15,15> APAT = mul(A, mul(P, transpose(A)));
  const Mat<15,15> KRKt = mul(K, mul(Rm, transpose(K)));
  P = APAT + KRKt;

  // Symmetrize
  for(int i=0;i<15;i++){
    for(int j=i+1;j<15;j++){
      const float s = 0.5f*(P.a[i][j] + P.a[j][i]);
      P.a[i][j] = s; P.a[j][i] = s;
    }
  }
}

// Explicit instantiations for common measurement sizes
template void ESKF15::update<1>(const Mat<1,15>&, const Mat<1,1>&, const Mat<1,1>&);
template void ESKF15::update<3>(const Mat<3,15>&, const Mat<3,1>&, const Mat<3,3>&);
template void ESKF15::update<6>(const Mat<6,15>&, const Mat<6,1>&, const Mat<6,6>&);
template void ESKF15::updateMasked<1>(const Mat<1,15>&, const Mat<1,1>&, const Mat<1,1>&, uint16_t);
template void ESKF15::updateMasked<3>(const Mat<3,15>&, const Mat<3,1>&, const Mat<3,3>&, uint16_t);
template void ESKF15::updateMasked<6>(const Mat<6,15>&, const Mat<6,1>&, const Mat<6,6>&, uint16_t);

void ESKF15::inject_(const Mat<15,1>& dx) {
  // Extract pieces (make non-const so we can clamp)
  Vec3 dp (dx.a[0][0],  dx.a[1][0],  dx.a[2][0]);
  Vec3 dv (dx.a[3][0],  dx.a[4][0],  dx.a[5][0]);
  Vec3 dth(dx.a[6][0],  dx.a[7][0],  dx.a[8][0]);
  Vec3 dbg(dx.a[9][0],  dx.a[10][0], dx.a[11][0]);
  Vec3 dba(dx.a[12][0], dx.a[13][0], dx.a[14][0]);

  // If we don't have a heading reference (mag or GPS course while moving),
  // yaw and bg.z are not observable. Do NOT let the filter change them.
  if (!heading_observable) {
    dth.z = 0.0f;  // block yaw correction
    dbg.z = 0.0f;  // block z gyro-bias correction
  }

  // Inject into nominal state
  p += dp;
  v += dv;

  q = q * Quat::expSmall(dth);
  q.normalize();

  bg += dbg;
  ba += dba;

  // Error-state reset (needs the SAME dth we injected)
  resetErrorState_(dth);
}

void ESKF15::resetErrorState_(const Vec3& dtheta) {
  // ESKF "reset" for theta: G_theta = I - 0.5*skew(dtheta)
  Mat<15,15> G = Mat<15,15>::Identity();
  const Mat<3,3> S = skew(dtheta);

  for(int r=0;r<3;r++){
    for(int c=0;c<3;c++){
      G.a[6+r][6+c] = (r==c ? 1.0f : 0.0f) - 0.5f * S.a[r][c];
    }
  }

  P = mul(G, mul(P, transpose(G)));
}

void ESKF15::updateGPSPosVel(const Vec3& pos_meas, const Vec3& vel_meas,
                            float sigma_pos, float sigma_vel) {
  // z = [p; v], h = [p; v]
  Mat<6,15> H = Mat<6,15>::Zero();
  for(int i=0;i<3;i++){
    H.a[i][i]     = 1.0f; // pos wrt dp
    H.a[3+i][3+i] = 1.0f; // vel wrt dv
  }

  Mat<6,1> r; r.setZero();
  r.a[0][0] = pos_meas.x - p.x;
  r.a[1][0] = pos_meas.y - p.y;
  r.a[2][0] = pos_meas.z - p.z;
  r.a[3][0] = vel_meas.x - v.x;
  r.a[4][0] = vel_meas.y - v.y;
  r.a[5][0] = vel_meas.z - v.z;

  Mat<6,6> Rm = Mat<6,6>::Zero();
  const float sp2 = sigma_pos*sigma_pos;
  const float sv2 = sigma_vel*sigma_vel;
  for(int i=0;i<3;i++) {
    Rm.a[i][i]     = sp2;
    Rm.a[3+i][3+i] = sv2;
  }

  // GPS pos/vel should not affect attitude or gyro bias unless heading is observable.
  updateMasked<6>(H, r, Rm, UPD_DP | UPD_DV);
}

void ESKF15::updateBaroAlt(float alt_m_up, float sigma_alt) {
  // h = -p.z (since p.z is Down in NED); z is Up-positive altitude
  Mat<1,15> H = Mat<1,15>::Zero();
  H.a[0][2] = -1.0f; // d(-p.z)/d(dpz) = -1

  Mat<1,1> r; r.setZero();
  const float h = -p.z;
  r.a[0][0] = alt_m_up - h;

  Mat<1,1> Rm; Rm.setZero();
  Rm.a[0][0] = sigma_alt*sigma_alt;

  // Baro should NOT correct attitude or gyro bias. Only allow position/velocity corrections.
  updateMasked<1>(H, r, Rm, UPD_DP | UPD_DV);
}

} // namespace eskf