#include "ESKF15.hpp"

namespace finware {

// ---------------- Quaternion ops ----------------
Quat ESKF15::qmul(const Quat& a, const Quat& b) {
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

Quat ESKF15::qnorm(const Quat& q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n <= 1e-9f) return {1,0,0,0};
  float inv = 1.0f / n;
  return {q.w*inv, q.x*inv, q.y*inv, q.z*inv};
}

Quat ESKF15::smallAngleQuat(const Vec3& dtheta) {
  // For small angles: q = [1, 0.5*dtheta]
  return qnorm({1.0f, 0.5f*dtheta.x, 0.5f*dtheta.y, 0.5f*dtheta.z});
}

void ESKF15::quatToDCM(const Quat& q_in, float R[3][3]) {
  Quat q = qnorm(q_in);
  const float w=q.w, x=q.x, y=q.y, z=q.z;

  R[0][0] = 1.0f - 2.0f*(y*y + z*z);
  R[0][1] = 2.0f*(x*y - w*z);
  R[0][2] = 2.0f*(x*z + w*y);

  R[1][0] = 2.0f*(x*y + w*z);
  R[1][1] = 1.0f - 2.0f*(x*x + z*z);
  R[1][2] = 2.0f*(y*z - w*x);

  R[2][0] = 2.0f*(x*z - w*y);
  R[2][1] = 2.0f*(y*z + w*x);
  R[2][2] = 1.0f - 2.0f*(x*x + y*y);
}

Vec3 ESKF15::quatRotate(const Quat& q_nb, const Vec3& v_b) {
  // v_n = q * [0,v] * q_conj
  Quat q = qnorm(q_nb);
  Quat vq{0, v_b.x, v_b.y, v_b.z};
  Quat out = qmul(qmul(q, vq), qconj(q));
  return {out.x, out.y, out.z};
}

void ESKF15::skew(const Vec3& a, float S[3][3]) {
  S[0][0]=0;    S[0][1]=-a.z; S[0][2]= a.y;
  S[1][0]= a.z; S[1][1]=0;    S[1][2]=-a.x;
  S[2][0]=-a.y; S[2][1]= a.x; S[2][2]=0;
}

// ---------------- Matrix utils ----------------
void ESKF15::matIdentity(float* A, int n) {
  for (int i=0;i<n;i++){
    for (int j=0;j<n;j++) A[i*n+j] = (i==j)?1.0f:0.0f;
  }
}
void ESKF15::matZero(float* A, int r, int c) {
  for (int i=0;i<r*c;i++) A[i]=0.0f;
}

void ESKF15::matMul(const float* A, int Ar, int Ac,
                    const float* B, int Br, int Bc,
                    float* C) {
  (void)Br;
  for (int i=0;i<Ar;i++){
    for (int j=0;j<Bc;j++){
      float s=0;
      for (int k=0;k<Ac;k++) s += A[i*Ac+k]*B[k*Bc+j];
      C[i*Bc+j]=s;
    }
  }
}

void ESKF15::matMulT_B(const float* A, int Ar, int Ac,
                       const float* B, int Br, int Bc,
                       float* C) {
  // C = A * B^T, so B is (Br x Bc), B^T is (Bc x Br)
  (void)Bc;
  for (int i=0;i<Ar;i++){
    for (int j=0;j<Br;j++){
      float s=0;
      for (int k=0;k<Ac;k++) s += A[i*Ac+k]*B[j*Bc+k];
      C[i*Br+j]=s;
    }
  }
}

void ESKF15::matMulT_A(const float* A, int Ar, int Ac,
                       const float* B, int Br, int Bc,
                       float* C) {
  // C = A^T * B, A^T is (Ac x Ar)
  (void)Br;
  for (int i=0;i<Ac;i++){
    for (int j=0;j<Bc;j++){
      float s=0;
      for (int k=0;k<Ar;k++) s += A[k*Ac+i]*B[k*Bc+j];
      C[i*Bc+j]=s;
    }
  }
}

bool ESKF15::matInv(float* A, int n) {
  // Gauss-Jordan with partial pivoting (small n)
  float I[16*16]; // enough for up to 16
  if (n > 16) return false;
  matIdentity(I, n);

  for (int col=0; col<n; col++) {
    // pivot
    int piv = col;
    float maxv = fabsf(A[col*n + col]);
    for (int r=col+1;r<n;r++){
      float v = fabsf(A[r*n + col]);
      if (v > maxv){ maxv=v; piv=r; }
    }
    if (maxv < 1e-9f) return false;

    // swap rows
    if (piv != col){
      for (int c=0;c<n;c++){
        float tmp = A[col*n+c]; A[col*n+c] = A[piv*n+c]; A[piv*n+c]=tmp;
        tmp = I[col*n+c]; I[col*n+c] = I[piv*n+c]; I[piv*n+c]=tmp;
      }
    }

    float diag = A[col*n + col];
    float invd = 1.0f / diag;

    // normalize row
    for (int c=0;c<n;c++){
      A[col*n+c] *= invd;
      I[col*n+c] *= invd;
    }

    // eliminate others
    for (int r=0;r<n;r++){
      if (r==col) continue;
      float f = A[r*n + col];
      if (fabsf(f) < 1e-12f) continue;
      for (int c=0;c<n;c++){
        A[r*n+c] -= f * A[col*n+c];
        I[r*n+c] -= f * I[col*n+c];
      }
    }
  }

  // copy inverse back
  for (int i=0;i<n*n;i++) A[i]=I[i];
  return true;
}

// ---------------- ESKF core ----------------
ESKF15::ESKF15(const ESKF15Config& cfg) : _cfg(cfg) {
  // Initialize P to something reasonable
  // dp ~ 10m, dv ~ 3m/s, dtheta ~ 10deg, biases moderate
  for (int i=0;i<15;i++) for (int j=0;j<15;j++) Pcov[i][j]=0.0f;

  const float dp2 = 10.0f*10.0f;
  const float dv2 = 3.0f*3.0f;
  const float da2 = (0.17f)*(0.17f);     // 10 deg ~ 0.17 rad
  const float dbg2 = (0.05f)*(0.05f);    // rad/s
  const float dba2 = (0.5f)*(0.5f);      // m/s^2

  for (int i=0;i<3;i++){
    Pcov[i][i]       = dp2;
    Pcov[i+3][i+3]   = dv2;
    Pcov[i+6][i+6]   = da2;
    Pcov[i+9][i+9]   = dbg2;
    Pcov[i+12][i+12] = dba2;
  }
}

void ESKF15::reset(const Vec3& p_ned, const Vec3& v_ned, const Quat& q_nb,
                   const Vec3& bg, const Vec3& ba) {
  _p = p_ned;
  _v = v_ned;
  _q = qnorm(q_nb);
  _bg = bg;
  _ba = ba;
}

void ESKF15::predict(float dt, const Vec3& gyro_radps, const Vec3& accel_mps2) {
  if (dt <= 0) return;

  // 1) Nominal propagate attitude using gyro - bg
  Vec3 w = vsub(gyro_radps, _bg); // body rates
  // small-angle integration: q_{k+1} = q * exp(0.5*w*dt)
  Vec3 dth = vmul(w, dt);
  Quat dq = smallAngleQuat(dth);
  _q = qnorm(qmul(_q, dq));

  // 2) Nominal propagate velocity/position using accel - ba, rotated to nav, add gravity
  Vec3 f_b = vsub(accel_mps2, _ba);       // specific force estimate in body
  Vec3 f_n = quatRotate(_q, f_b);         // to nav
  // add gravity in NED (+Down)
  Vec3 g_n{0.0f, 0.0f, _cfg.g};
  Vec3 a_n = vadd(f_n, g_n);

  _v = vadd(_v, vmul(a_n, dt));
  _p = vadd(_p, vadd(vmul(_v, dt), vmul(a_n, 0.5f*dt*dt))); // semi-implicit-ish

  // 3) Build continuous-time F and G, then discretize: Phi ≈ I + F dt, Qd ≈ (G Qc G^T) dt
  // State ordering: [dp(0:2), dv(3:5), dtheta(6:8), dbg(9:11), dba(12:14)]
  float F[15][15]; // continuous
  for (int i=0;i<15;i++) for (int j=0;j<15;j++) F[i][j]=0.0f;

  // dp_dot = dv
  for (int i=0;i<3;i++) F[i][i+3] = 1.0f;

  // dv_dot ≈ -R_nb * skew(f_b) * dtheta  - R_nb*dba
  float Rnb[3][3]; quatToDCM(_q, Rnb);
  float Sfb[3][3]; skew(f_b, Sfb);

  // -Rnb * Sfb
  float A[3][3];
  for (int i=0;i<3;i++){
    for (int j=0;j<3;j++){
      float s=0;
      for (int k=0;k<3;k++) s += Rnb[i][k]*Sfb[k][j];
      A[i][j] = -s;
    }
  }
  // place into F[dv, dtheta]
  for (int r=0;r<3;r++) for (int c=0;c<3;c++) F[r+3][c+6] = A[r][c];

  // dv wrt accel bias: -Rnb
  for (int r=0;r<3;r++) for (int c=0;c<3;c++) F[r+3][c+12] = -Rnb[r][c];

  // dtheta_dot = -dbg  (simplified)
  for (int i=0;i<3;i++) F[i+6][i+9] = -1.0f;

  // biases random walk -> modeled in Q, F zeros

  // Discrete Phi ≈ I + F dt
  float Phi[15][15];
  for (int i=0;i<15;i++){
    for (int j=0;j<15;j++){
      Phi[i][j] = (i==j)?1.0f:0.0f;
      Phi[i][j] += F[i][j]*dt;
    }
  }

  // Process noise Qd (15x15)
  // We inject noise into dv from accel noise, into dtheta from gyro noise,
  // and into dbg/dba from their random walks.
  float Qd[15][15]; for (int i=0;i<15;i++) for (int j=0;j<15;j++) Qd[i][j]=0.0f;

  const float sa2  = _cfg.sigma_accel * _cfg.sigma_accel;
  const float sg2  = _cfg.sigma_gyro  * _cfg.sigma_gyro;
  const float sbg2 = _cfg.sigma_bg_rw * _cfg.sigma_bg_rw;
  const float sba2 = _cfg.sigma_ba_rw * _cfg.sigma_ba_rw;

  // dv noise: Rnb * accel_noise
  // Approx: Q_dv = (Rnb * sa2*I * Rnb^T) * dt = sa2 * I * dt  (since R is orthonormal)
  for (int i=0;i<3;i++) Qd[i+3][i+3] += sa2 * dt;

  // dtheta noise from gyro measurement noise (integrated): sg2 * dt
  for (int i=0;i<3;i++) Qd[i+6][i+6] += sg2 * dt;

  // bias random walk
  for (int i=0;i<3;i++) Qd[i+9][i+9]   += sbg2 * dt;
  for (int i=0;i<3;i++) Qd[i+12][i+12] += sba2 * dt;

  // P = Phi P Phi^T + Qd
  float Ptmp[15][15]; for (int i=0;i<15;i++) for (int j=0;j<15;j++) Ptmp[i][j]=0.0f;
  // Ptmp = Phi*P
  for (int i=0;i<15;i++){
    for (int j=0;j<15;j++){
      float s=0;
      for (int k=0;k<15;k++) s += Phi[i][k]*Pcov[k][j];
      Ptmp[i][j]=s;
    }
  }
  float Pnew[15][15]; for (int i=0;i<15;i++) for (int j=0;j<15;j++) Pnew[i][j]=0.0f;
  // Pnew = Ptmp * Phi^T
  for (int i=0;i<15;i++){
    for (int j=0;j<15;j++){
      float s=0;
      for (int k=0;k<15;k++) s += Ptmp[i][k]*Phi[j][k];
      Pnew[i][j]=s + Qd[i][j];
    }
  }
  // copy back
  for (int i=0;i<15;i++) for (int j=0;j<15;j++) Pcov[i][j]=Pnew[i][j];
}

bool ESKF15::updateGPS(const GPSMeas& gps) {
  if (!gps.valid) return false;

  // --- position update (3) ---
  {
    float r[3] = {
      gps.p_ned_m.x - _p.x,
      gps.p_ned_m.y - _p.y,
      gps.p_ned_m.z - _p.z
    };

    float H[3*15]; // row-major (m x 15)
    for (int i=0;i<3*15;i++) H[i]=0.0f;
    // residual depends on dp directly: h = p, so r = z - p => H = [I3, 0]
    for (int i=0;i<3;i++) H[i*15 + i] = 1.0f;

    float Rm[3*3]; for (int i=0;i<9;i++) Rm[i]=0.0f;
    for (int i=0;i<3;i++) Rm[i*3+i] = _cfg.sigma_gps_pos * _cfg.sigma_gps_pos;

    if (!updateGeneric(H, 3, r, Rm, _cfg.nis_gate_gps_pos)) {
      // reject pos update if gated
      // still try vel update below
    }
  }

  // --- velocity update (3) ---
  {
    float r[3] = {
      gps.v_ned_mps.x - _v.x,
      gps.v_ned_mps.y - _v.y,
      gps.v_ned_mps.z - _v.z
    };

    float H[3*15];
    for (int i=0;i<3*15;i++) H[i]=0.0f;
    for (int i=0;i<3;i++) H[i*15 + (i+3)] = 1.0f;

    float Rm[3*3]; for (int i=0;i<9;i++) Rm[i]=0.0f;
    for (int i=0;i<3;i++) Rm[i*3+i] = _cfg.sigma_gps_vel * _cfg.sigma_gps_vel;

    return updateGeneric(H, 3, r, Rm, _cfg.nis_gate_gps_vel);
  }
}

bool ESKF15::updateBaro(const BaroMeas& baro) {
  if (!baro.valid) return false;

  // measurement: z_down = p.z (Down)
  float r = baro.z_down_m - _p.z;

  float H[1*15]; for (int i=0;i<15;i++) H[i]=0.0f;
  H[2] = 1.0f; // dp_z

  float Rm = _cfg.sigma_baro_z * _cfg.sigma_baro_z;
  return updateGeneric(H, 1, &r, &Rm, _cfg.nis_gate_baro);
}

bool ESKF15::updateAttitude(const AttMeas& att) {
  if (!att.valid) return false;

  // We treat measured q_nb as an attitude measurement:
  // q_err = q_meas * conj(q_nom)
  // For small angles: q_err ≈ [1, 0.5*dtheta]
  Quat q_meas = qnorm(att.q_nb);
  Quat q_nom  = qnorm(_q);

  Quat q_err = qmul(q_meas, qconj(q_nom));
  q_err = qnorm(q_err);

  // Map quaternion error to small angle: dtheta ≈ 2 * vec(q_err) * sign(w)
  float sgn = (q_err.w >= 0.0f) ? 1.0f : -1.0f;
  Vec3 dtheta_meas{ 2.0f*sgn*q_err.x, 2.0f*sgn*q_err.y, 2.0f*sgn*q_err.z };

  float r[3] = { dtheta_meas.x, dtheta_meas.y, dtheta_meas.z };

  float H[3*15]; for (int i=0;i<3*15;i++) H[i]=0.0f;
  // residual is directly dtheta error state
  for (int i=0;i<3;i++) H[i*15 + (i+6)] = 1.0f;

  float Rm[3*3]; for (int i=0;i<9;i++) Rm[i]=0.0f;
  for (int i=0;i<3;i++) Rm[i*3+i] = _cfg.sigma_att_meas * _cfg.sigma_att_meas;

  return updateGeneric(H, 3, r, Rm, _cfg.nis_gate_att);
}

bool ESKF15::updateGeneric(const float* H, int m, const float* r, const float* R, float nis_gate) {
  // Compute S = H P H^T + R   (m x m)
  float HP[16*15]; // m x 15, max m=3 here but allow up to 16
  if (m > 16) return false;

  // HP = H * P
  for (int i=0;i<m;i++){
    for (int j=0;j<15;j++){
      float s=0;
      for (int k=0;k<15;k++) s += H[i*15+k]*Pcov[k][j];
      HP[i*15+j]=s;
    }
  }

  float S[16*16]; matZero(S, m, m);

  // S = HP * H^T + R
  for (int i=0;i<m;i++){
    for (int j=0;j<m;j++){
      float s=0;
      for (int k=0;k<15;k++) s += HP[i*15+k]*H[j*15+k];
      S[i*m+j]=s + R[i*m+j];
    }
  }

  // Gate by NIS = r^T S^{-1} r
  float Sinv[16*16];
  for (int i=0;i<m*m;i++) Sinv[i]=S[i];
  if (!matInv(Sinv, m)) return false;

  float nis = 0.0f;
  // tmp = Sinv * r
  float tmp[16]; for (int i=0;i<m;i++){
    float s=0; for (int j=0;j<m;j++) s += Sinv[i*m+j]*r[j];
    tmp[i]=s;
  }
  for (int i=0;i<m;i++) nis += r[i]*tmp[i];

  if (nis > nis_gate) {
    return false; // reject
  }

  // Kalman gain: K = P H^T S^{-1}  (15 x m)
  // First PHt = P H^T = (15 x m)
  float PHt[15*16]; // 15 x m
  for (int i=0;i<15;i++){
    for (int j=0;j<m;j++){
      float s=0;
      for (int k=0;k<15;k++) s += Pcov[i][k]*H[j*15+k];
      PHt[i*m+j]=s;
    }
  }

  float K[15*16]; // 15 x m
  for (int i=0;i<15;i++){
    for (int j=0;j<m;j++){
      float s=0;
      for (int k=0;k<m;k++) s += PHt[i*m+k]*Sinv[k*m+j];
      K[i*m+j]=s;
    }
  }

  // dx = K * r
  float dx[15]; for (int i=0;i<15;i++){
    float s=0; for (int j=0;j<m;j++) s += K[i*m+j]*r[j];
    dx[i]=s;
  }

  // Joseph form covariance update:
  // P = (I-KH) P (I-KH)^T + K R K^T
  float I[15*15]; matIdentity(I, 15);

  float KH[15*15]; matZero(KH, 15, 15);
  for (int i=0;i<15;i++){
    for (int j=0;j<15;j++){
      float s=0;
      for (int k=0;k<m;k++) s += K[i*m+k]*H[k*15+j];
      KH[i*15+j]=s;
    }
  }

  float IKH[15*15];
  for (int i=0;i<15;i++) for (int j=0;j<15;j++) IKH[i*15+j] = I[i*15+j] - KH[i*15+j];

  // temp = (I-KH) P
  float Pflat[15*15];
  for (int i=0;i<15;i++) for (int j=0;j<15;j++) Pflat[i*15+j] = Pcov[i][j];

  float temp[15*15];
  matMul(IKH, 15, 15, Pflat, 15, 15, temp);

  // P1 = temp (I-KH)^T
  float P1[15*15];
  matMulT_B(temp, 15, 15, IKH, 15, 15, P1);

  // KRKt = K R K^T
  float KR[15*16]; matZero(KR, 15, m);
  // KR = K*R
  for (int i=0;i<15;i++){
    for (int j=0;j<m;j++){
      float s=0; for (int k=0;k<m;k++) s += K[i*m+k]*R[k*m+j];
      KR[i*m+j]=s;
    }
  }
  float KRKt[15*15]; matZero(KRKt, 15, 15);
  // KRKt = KR * K^T
  for (int i=0;i<15;i++){
    for (int j=0;j<15;j++){
      float s=0; for (int k=0;k<m;k++) s += KR[i*m+k]*K[j*m+k];
      KRKt[i*15+j]=s;
    }
  }

  // Pnew = P1 + KRKt
  for (int i=0;i<15;i++){
    for (int j=0;j<15;j++){
      Pcov[i][j] = P1[i*15+j] + KRKt[i*15+j];
    }
  }

  // Inject correction into nominal + reset
  injectAndReset(dx);
  return true;
}

void ESKF15::injectAndReset(const float dx[15]) {
  // dx: [dp,dv,dtheta,dbg,dba]
  Vec3 dp{dx[0],dx[1],dx[2]};
  Vec3 dv{dx[3],dx[4],dx[5]};
  Vec3 dth{dx[6],dx[7],dx[8]};
  Vec3 dbg{dx[9],dx[10],dx[11]};
  Vec3 dba{dx[12],dx[13],dx[14]};

  _p = vadd(_p, dp);
  _v = vadd(_v, dv);

  // attitude injection: q <- q * exp(0.5*dtheta)
  Quat dq = smallAngleQuat(dth);
  _q = qnorm(qmul(_q, dq));

  _bg = vadd(_bg, dbg);
  _ba = vadd(_ba, dba);

  // Covariance reset for ESKF (first-order):
  // P <- G P G^T, where G adjusts attitude error after injection:
  // G = I, except dtheta block: (I - 0.5*skew(dtheta))
  float G[15][15];
  for (int i=0;i<15;i++) for (int j=0;j<15;j++) G[i][j] = (i==j)?1.0f:0.0f;

  float Sd[3][3]; skew(dth, Sd);
  // (I - 0.5*Sd)
  for (int r=0;r<3;r++){
    for (int c=0;c<3;c++){
      float val = (r==c)?1.0f:0.0f;
      val -= 0.5f * Sd[r][c];
      G[r+6][c+6] = val;
    }
  }

  // P = G P G^T
  float Ptmp[15][15]; for (int i=0;i<15;i++) for (int j=0;j<15;j++) Ptmp[i][j]=0.0f;
  for (int i=0;i<15;i++){
    for (int j=0;j<15;j++){
      float s=0;
      for (int k=0;k<15;k++) s += G[i][k]*Pcov[k][j];
      Ptmp[i][j]=s;
    }
  }
  float Pnew[15][15]; for (int i=0;i<15;i++) for (int j=0;j<15;j++) Pnew[i][j]=0.0f;
  for (int i=0;i<15;i++){
    for (int j=0;j<15;j++){
      float s=0;
      for (int k=0;k<15;k++) s += Ptmp[i][k]*G[j][k];
      Pnew[i][j]=s;
    }
  }
  for (int i=0;i<15;i++) for (int j=0;j<15;j++) Pcov[i][j]=Pnew[i][j];
}

} // namespace finware