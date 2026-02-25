// #include "KalmanFilter.h"
// #include <iostream>
// #include <vector>
// #include <math.h>

// // Filter follows step-wise implementation as outlined in ardupilot EKF1: https://ardupilot.org/dev/docs/extended-kalman-filter.html

// // 15 state filter with states: [px, py, pz, vx, vy, vz, dtheta_x, dtheta_y, dtheta_z, gyro_bias_x, gyro_bias_y, gyro_bias_z, accel_bias_x, accel_bias_y, accel_bias_z]
// // NED frame for world and IMU axises for body
// // Quaternion storage order begins with scalar term
// // Quaternion maps body to NED: q = [qw, qx, qy, qz]

// // Helper function
// void matrixMultiply(float A[3][3], float B[3][3], float result[3][3]) {
//     for (int i = 0; i < 3; ++i) {
//         for (int j = 0; j < 3; ++j) {
//             result[i][j] = 0.0f;
//             for (int k = 0; k < 3; ++k) {
//                 result[i][j] += A[i][k] * B[k][j];
//             }
//         }
//     }
// }


// // Step 1
// void KalmanFilter::integrateIMU(float gx, float gy, float gz, float dt, float gyro_bias_x, float gyro_bias_y, float gyro_bias_z) {
//     // Integrate gyroscope data to update orientation estimate
//     std::vector<float> omega = {gx - gyro_bias_x, gy - gyro_bias_y, gz - gyro_bias_z}; // Corrected angular velocity
//     dtheta = {omega[0] * dt, omega[1] * dt, omega[2] * dt};
//     mag_dtheta = sqrtf(dtheta[0]*dtheta[0] + dtheta[1]*dtheta[1] + dtheta[2]*dtheta[2]);

//     // Convert dtheta to quaternion representation (small angle approximation)
//     dquat = { 1.0f - (mag_dtheta * mag_dtheta) / 8.0f,
//               dtheta[0] / 2.0f,
//               dtheta[1] / 2.0f,
//               dtheta[2] / 2.0f };
    
//     // Quaternion multiplication to update current attitude
//     quat_attitude = {
//         quat_attitude[0] * dquat[0] - quat_attitude[1] * dquat[1] - quat_attitude[2] * dquat[2] - quat_attitude[3] * dquat[3],
//         quat_attitude[0] * dquat[1] + quat_attitude[1] * dquat[0] + quat_attitude[2] * dquat[3] - quat_attitude[3] * dquat[2],
//         quat_attitude[0] * dquat[2] - quat_attitude[1] * dquat[3] + quat_attitude[2] * dquat[0] + quat_attitude[3] * dquat[1],
//         quat_attitude[0] * dquat[3] + quat_attitude[1] * dquat[2] - quat_attitude[2] * dquat[1] + quat_attitude[3] * dquat[0]
//     };

//     // Normalize quaternion
//     float norm = sqrtf(quat_attitude[0]*quat_attitude[0] + quat_attitude[1]*quat_attitude[1] +
//                            quat_attitude[2]*quat_attitude[2] + quat_attitude[3]*quat_attitude[3]);
//     quat_attitude[0] /= norm;
//     quat_attitude[1] /= norm; 
//     quat_attitude[2] /= norm;
//     quat_attitude[3] /= norm;
// }


// // Step 2
// void KalmanFilter::convertAccelToNED(float ax, float ay, float az, std::vector<float>& quat_attitude, float accel_bias_x, float accel_bias_y, float accel_bias_z) {
//     // Rotation matrix from quaternion to convert body axis to NED
//     rotation_matrix = {
//         {1 - 2*(quat_attitude[2]*quat_attitude[2] + quat_attitude[3]*quat_attitude[3]), 2*(quat_attitude[1]*quat_attitude[2] - quat_attitude[0]*quat_attitude[3]), 2*(quat_attitude[1]*quat_attitude[3] + quat_attitude[0]*quat_attitude[2])},
//         {2*(quat_attitude[1]*quat_attitude[2] + quat_attitude[0]*quat_attitude[3]), 1 - 2*(quat_attitude[1]*quat_attitude[1] + quat_attitude[3]*quat_attitude[3]), 2*(quat_attitude[2]*quat_attitude[3] - quat_attitude[0]*quat_attitude[1])},
//         {2*(quat_attitude[1]*quat_attitude[3] - quat_attitude[0]*quat_attitude[2]), 2*(quat_attitude[2]*quat_attitude[3] + quat_attitude[0]*quat_attitude[1]), 1 - 2*(quat_attitude[1]*quat_attitude[1] + quat_attitude[2]*quat_attitude[2])}
//     };

//     // Subtract accelerometer bias
//     ax -= accel_bias_x;
//     ay -= accel_bias_y;
//     az -= accel_bias_z;

//     a_ned = {
//         rotation_matrix[0][0] * ax + rotation_matrix[0][1] * ay + rotation_matrix[0][2] * az,
//         rotation_matrix[1][0] * ax + rotation_matrix[1][1] * ay + rotation_matrix[1][2] * az,
//         rotation_matrix[2][0] * ax + rotation_matrix[2][1] * ay + rotation_matrix[2][2] * az
//     };

//     a_ned[2] += 9.81f; // add gravity to vertical component
// }

// // Step 3
// void KalmanFilter::integrateVelocity(float dt, std::vector<float> a_ned, std::vector<float>& vel_ned) {
//     // Integrate acceleration to update velocity estimate
//     vel_ned = {
//         vel_ned[0] + a_ned[0] * dt,
//         vel_ned[1] + a_ned[1] * dt,
//         vel_ned[2] + a_ned[2] * dt
//     };
// } 

// // Step 4
// void KalmanFilter::integratePosition(float dt, std::vector<float> vel_ned, std::vector<float>& pos_ned) {
//     // Integrate velocity to update position estimate
//     pos_ned = {
//         pos_ned[0] + vel_ned[0] * dt,
//         pos_ned[1] + vel_ned[1] * dt,
//         pos_ned[2] + vel_ned[2] * dt
//     };
// }

// // Step 5
// void buildSkewMatrix(const std::vector<float>& vec, float skew[3][3]) {
//     skew[0][0] = 0.0f;        skew[0][1] = -vec[2];   skew[0][2] = vec[1];
//     skew[1][0] = vec[2];     skew[1][1] = 0.0f;      skew[1][2] = -vec[0];
//     skew[2][0] = -vec[1];    skew[2][1] = vec[0];    skew[2][2] = 0.0f;
// }

// // Step 5
// void buldErrorStateMatrix(float rot_matrix[3][3], float ax, float ay, float az, float gx, float gy, float gz, float F[15][15]) {
//     F = {0};
//     // Build skew-symmetric matrices for gyro and accel
//     float gyro_skew[3][3];
//     buildSkewMatrix({gx, gy, gz}, gyro_skew);
//     float accel_skew[3][3];
//     buildSkewMatrix({ax, ay, az}, accel_skew);

//     float neg_gyro_skew[3][3];
//     for (int i = 0; i < 3; ++i) {
//         for (int j = 0; j < 3; ++j) {
//             neg_gyro_skew[i][j] = -gyro_skew[i][j];
//         }
//     }

//     float neg_rot_matrix[3][3];
//     for (int i = 0; i < 3; ++i) {
//         for (int j = 0; j < 3; ++j) {
//             neg_rot_matrix[i][j] = -rot_matrix[i][j];
//         }
//     }

//     float rbgx[3][3];
//     matrixMultiply(neg_rot_matrix, accel_skew, rbgx);
    
//     // Fill in F matrix
//     // Identity in [0,1]
//     F[0][3] = 1.0f; F[1][4] = 1.0f; F[2][5] = 1.0f;

//     // Cross product in [1,2]
//     F[3][6] = rbgx[0][0]; F[3][7] = rbgx[0][1]; F[3][8] = rbgx[0][2];
//     F[4][6] = rbgx[1][0]; F[4][7] = rbgx[1][1]; F[4][8] = rbgx[1][2];
//     F[5][6] = rbgx[2][0]; F[5][7] = rbgx[2][1]; F[5][8] = rbgx[2][2];

//     // Negative rotation matrix in [1,4]
//     F[3][12] = -rot_matrix[0][0]; F[3][13] = -rot_matrix[0][1]; F[3][14] = -rot_matrix[0][2];
//     F[4][12] = -rot_matrix[1][0]; F[4][13] = -rot_matrix[1][1]; F[4][14] = -rot_matrix[1][2];
//     F[5][12] = -rot_matrix[2][0]; F[5][13] = -rot_matrix[2][1]; F[5][14] = -rot_matrix[2][2];

//     // Negative gyro skew in [2,2]
//     F[6][6] = neg_gyro_skew[0][0]; F[6][7] = neg_gyro_skew[0][1]; F[6][8] = neg_gyro_skew[0][2];
//     F[7][6] = neg_gyro_skew[1][0]; F[7][7] = neg_gyro_skew[1][1]; F[7][8] = neg_gyro_skew[1][2];
//     F[8][6] = neg_gyro_skew[2][0]; F[8][7] = neg_gyro_skew[2][1]; F[8][8] = neg_gyro_skew[2][2];

//     // Negative Identity matrix in [2, 3]
//     F[6][9] = -1.0f; F[7][10] = -1.0f; F[8][11] = -1.0f;
// }

// // Step 5
// void discretizeError(float F[15][15], float dt, float Phi[15][15]) {
//     // Simple Euler discretization: Fd = I + F*dt
//     for (int i = 0; i < 15; ++i) {
//         for (int j = 0; j < 15; ++j) {
//             if (i == j) {
//                 Phi[i][j] = 1.0f + F[i][j] * dt;
//             } else {
//                 Phi[i][j] = F[i][j] * dt;
//             }
//         }
//     }
// }

// // Step 5
// // Unused currently because of the simplification in computeProcessNoise
// void buildNoiseCovariance(float EKF_GYRO_NOISE, float EKF_ACCEL_NOISE, float EKF_GYRO_BIAS_RW, float EKF_ACC_BIAS_RW, float Q[12][12]) {
//     Q = {0};
//     // Process noise covariance matrix
//     // Zero except diagonal elements
//     // Remains constant
//     Q[0][0] = EKF_GYRO_NOISE * EKF_GYRO_NOISE;
//     Q[1][1] = EKF_GYRO_NOISE * EKF_GYRO_NOISE;
//     Q[2][2] = EKF_GYRO_NOISE * EKF_GYRO_NOISE;

//     Q[3][3] = EKF_ACCEL_NOISE * EKF_ACCEL_NOISE;
//     Q[4][4] = EKF_ACCEL_NOISE * EKF_ACCEL_NOISE;
//     Q[5][5] = EKF_ACCEL_NOISE * EKF_ACCEL_NOISE;

//     Q[6][6] = EKF_GYRO_BIAS_RW * EKF_GYRO_BIAS_RW;
//     Q[7][7] = EKF_GYRO_BIAS_RW * EKF_GYRO_BIAS_RW;
//     Q[8][8] = EKF_GYRO_BIAS_RW * EKF_GYRO_BIAS_RW;

//     Q[9][9] = EKF_ACC_BIAS_RW * EKF_ACC_BIAS_RW;
//     Q[10][10] = EKF_ACC_BIAS_RW * EKF_ACC_BIAS_RW;
//     Q[11][11] = EKF_ACC_BIAS_RW * EKF_ACC_BIAS_RW;
// }


// // Step 5
// // Unused currently because of the simplification in computeProcessNoise
// void buildInjectionMatrix(float rot_matrix[3][3], float G[15][12]) {
//     float neg_rot_matrix[3][3];
//     for (int i = 0; i < 3; ++i) {
//         for (int j = 0; j < 3; ++j) {
//             neg_rot_matrix[i][j] = -rot_matrix[i][j];
//         }
//     }
//     G = {0};

//     // Fill in G matrix
//     // negative rotation matrix at [1,1]
//     G[3][3] = neg_rot_matrix[0][0]; G[3][4] = neg_rot_matrix[0][1]; G[3][5] = neg_rot_matrix[0][2];
//     G[4][3] = neg_rot_matrix[1][0]; G[4][4] = neg_rot_matrix[1][1]; G[4][5] = neg_rot_matrix[1][2];
//     G[5][3] = neg_rot_matrix[2][0]; G[5][4] = neg_rot_matrix[2][1]; G[5][5] = neg_rot_matrix[2][2];

//     // Negative identity at [2,0]
//     G[6][0] = -1.0f; G[7][1] = -1.0f; G[8][2] = -1.0f;

//     // Identity at [3,2]
//     G[9][6] = 1.0f; G[10][7] = 1.0f; G[11][8] = 1.0f;

//     //Identity at [4,3]
//     G[12][9] = 1.0f; G[13][10] = 1.0f; G[14][11] = 1.0f;
// }

// // Step 5
// void computeProcessNoise(float EKF_GYRO_NOISE, float EKF_ACCEL_NOISE, float EKF_GYRO_BIAS_RW, float EKF_ACC_BIAS_RW, float dt, float Qd[15][15]) {
//     // Simplification of Qd = G*Q*G^t*dt
//     // Simplifies to filling in diagonal elements only
//     Qd = {0};
//     Qd[3][3] = EKF_ACCEL_NOISE * EKF_ACCEL_NOISE * dt;
//     Qd[4][4] = EKF_ACCEL_NOISE * EKF_ACCEL_NOISE * dt;
//     Qd[5][5] = EKF_ACCEL_NOISE * EKF_ACCEL_NOISE * dt;

//     Qd[6][6] = EKF_GYRO_NOISE * EKF_GYRO_NOISE * dt;
//     Qd[7][7] = EKF_GYRO_NOISE * EKF_GYRO_NOISE * dt;
//     Qd[8][8] = EKF_GYRO_NOISE * EKF_GYRO_NOISE * dt;

//     Qd[9][9] = EKF_GYRO_BIAS_RW * EKF_GYRO_BIAS_RW * dt;
//     Qd[10][10] = EKF_GYRO_BIAS_RW * EKF_GYRO_BIAS_RW * dt;
//     Qd[11][11] = EKF_GYRO_BIAS_RW * EKF_GYRO_BIAS_RW * dt;

//     Qd[12][12] = EKF_ACC_BIAS_RW * EKF_ACC_BIAS_RW * dt;
//     Qd[13][13] = EKF_ACC_BIAS_RW * EKF_ACC_BIAS_RW * dt;
//     Qd[14][14] = EKF_ACC_BIAS_RW * EKF_ACC_BIAS_RW * dt;
// }

// // Step 5
// void propagateCovariance(float Phi [15][15], float P[15][15], float Qd[15][15], float P_pred[15][15]) {
//     float temp[15][15] = {0};
//     // Intermediate = Phi * P
//     for (int i = 0; i < 15; ++i) {
//         for (int j = 0; j < 15; ++j) {
//             for (int k = 0; k < 15; ++k) {
//                 temp[i][j] += Phi[i][k] * P[k][j];
//             }
//         }
//     }

//     P_pred = {0};
//     // P_pred = Intermediate * Phi^t + Qd
//     for (int i = 0; i < 15; ++i) {
//         for (int j = 0; j < 15; ++j) {
//             for (int k = 0; k < 15; ++k) {
//                 P_pred[i][j] += temp[i][k] * Phi[j][k];
//             }
//             P_pred[i][j] += Qd[i][j];
//         }
//     }

//     // Ensure symmetry
//     for (int i = 0; i < 15; ++i) {
//         for (int j = i + 1; j < 15; ++j) {
//             float s = 0.5f * (P_pred[i][j] + P_pred[j][i]);
//             P_pred[i][j] = s;
//             P_pred[j][i] = s;
//         }
//     }
// }

// // Step 6
// void GPSInnovation(float gps_N, float gps_E, std::vector<float> pos_ned, float& Ninnovation, float Einnovation) {
//     // Compute innovation between GPS position and estimated position
//     Ninnovation = gps_N - pos_ned[0];
//     Einnovation = gps_E - pos_ned[1];
// }

// // Step 6
// void BaroInnovation(float baro_z, float pos_ned_z, float& Dinnovation) {
//     // Compute innovation between barometric altitude and estimated altitude
//     Dinnovation = baro_z - pos_ned_z;
// }

// // Step 7
// void computeInnovationVariance(float P[15][15], int H_index, float& variance, float R) {
//     // Compute innovation variance: S = H*P*H^t + R
//     // Here H is a selection matrix picking the relevant state
//     variance = P[H_index][H_index] + R; // Simplified for direct state observation
// }

// // Step 7
// void computeKalmanGain(float P[15][15], int H_index, float S, float K[15]) {
//     // Compute Kalman Gain: K = P*H^t*S^-1
//     for (int i = 0; i < 15; ++i) {
//         K[i] = P[i][H_index] / S; // Simplified for direct state observation
//     }
// }

// // Step 7
// void computeErrorStateCorrection(float K[15], float innovation, float dx[15]) {
//     // Compute error state correction: dx = K * innovation
//     for (int i = 0; i < 15; ++i) {
//         dx[i] = K[i] * innovation;
//     }
// }

// // Step 7 (still)
// void injectErrorStateIntoNominalState(float dx[15], std::vector<float>& quat_attitude, std::vector<float>& vel_ned, std::vector<float>& pos_ned, float& gyro_bias_x, float& gyro_bias_y, float& gyro_bias_z, float& accel_bias_x, float& accel_bias_y, float& accel_bias_z) {
//     // Inject attitude error
//     std::vector<float> dtheta = {dx[6], dx[7], dx[8]};
//     float mag_dtheta = sqrtf(dtheta[0]*dtheta[0] + dtheta[1]*dtheta[1] + dtheta[2]*dtheta[2]);

//     std::vector<float> dquat = { 1.0f - (mag_dtheta * mag_dtheta) / 8.0f,
//                                  dtheta[0] / 2.0f,
//                                  dtheta[1] / 2.0f,
//                                  dtheta[2] / 2.0f };

//     // Quaternion multiplication to update current attitude
//     quat_attitude = {
//         quat_attitude[0] * dquat[0] - quat_attitude[1] * dquat[1] - quat_attitude[2] * dquat[2] - quat_attitude[3] * dquat[3],
//         quat_attitude[0] * dquat[1] + quat_attitude[1] * dquat[0] + quat_attitude[2] * dquat[3] - quat_attitude[3] * dquat[2],
//         quat_attitude[0] * dquat[2] - quat_attitude[1] * dquat[3] + quat_attitude[2] * dquat[0] + quat_attitude[3] * dquat[1],
//         quat_attitude[0] * dquat[3] + quat_attitude[1] * dquat[2] - quat_attitude[2] * dquat[1] + quat_attitude[3] * dquat[0]
//     };

//     // inject position error
//     pos_ned[0] += dx[0];
//     pos_ned[1] += dx[1];
//     pos_ned[2] += dx[2];

//     // inject velocity error
//     vel_ned[0] += dx[3];
//     vel_ned[1] += dx[4];
//     vel_ned[2] += dx[5];

//     //inject gyro bias error
//     gyro_bias_x += dx[9];
//     gyro_bias_y += dx[10];
//     gyro_bias_z += dx[11];

//     // Inject accel bias error
//     accel_bias_x += dx[12];
//     accel_bias_y += dx[13];
//     accel_bias_z += dx[14];

//     // Update quaternion normalization
//     float norm = sqrtf(quat_attitude[0]*quat_attitude[0] + quat_attitude[1]*quat_attitude[1] +
//                            quat_attitude[2]*quat_attitude[2] + quat_attitude[3]*quat_attitude[3]);

//     quat_attitude[0] /= norm;
//     quat_attitude[1] /= norm;
//     quat_attitude[2] /= norm;
//     quat_attitude[3] /= norm;
// }

// // Step 7
// void updateCovariance(float K[15], int H_index, float P[15][15], float P_updated[15][15]) {
//     // Update covariance: P = (I - K*H)*P
//     float temp[15][15] = {0};
//     // Compute K*H
//     for (int i = 0; i < 15; ++i) {
//         for (int j = 0; j < 15; ++j) {
//             temp[i][j] = K[i] * ((i == H_index) ? 1.0f : 0.0f);
//         }
//     }

//     float I_minus_KH[15][15] = {0};
//     // Compute I - K*H
//     for (int i = 0; i < 15; ++i) {
//         for (int j = 0; j < 15; ++j) {
//             if (i == j) {
//                 I_minus_KH[i][j] = 1.0f - temp[i][j];
//             } else {
//                 I_minus_KH[i][j] = -temp[i][j];
//             }
//         }
//     }

//     // Compute P_updated = (I - K*H)*P
//     P_updated = {0};
//     for (int i = 0; i < 15; ++i) {
//         for (int j = 0; j < 15; ++j) {
//             for (int k = 0; k < 15; ++k) {
//                 P_updated[i][j] += I_minus_KH[i][k] * P[k][j];
//             }
//         }
//     }

//     // Ensure symmetry
//     for (int i = 0; i < 15; ++i) {
//         for (int j = i + 1; j < 15; ++j) {
//             float s = 0.5f * (P_updated[i][j] + P_updated[j][i]);
//             P_updated[i][j] = s;
//             P_updated[j][i] = s;
//         }
//     }
// }

// // Fix H, GPS N = 0, GPS E = 1, Baro D = 2 with a negative
// // Update covarance is also the simple method P <- (I-KH)P
// // More accurate is P <- (I-KH)P(I-KH)^t + KRK^t but more computationally expensive

// // Step 8: Joseph covariance update for scalar measurement
// // P <- (I - K H) P (I - K H)^T + K R K^T
// //
// // Scalar H: a 1x15 row with only one nonzero entry:
// //   H[H_index] = sign  (sign is +1 or -1)
// //
// // Inputs:
// //   P        : 15x15 covariance (updated in-place)
// //   K        : 15x1 Kalman gain vector
// //   H_index  : index of observed state
// //   sign     : +1 or -1
// //   R        : measurement variance (sigma^2)
// void josephUpdateScalar(float P[15][15], const float K[15], int H_index, float sign, float R)
// {
//     // 1) Build A = (I - K H) efficiently.
//     // Since H has only one nonzero at column H_index:
//     //   A[i][j] = I[i][j] - K[i] * H[j]
//     //         = I[i][j] - K[i] * sign  if j == H_index
//     //         = I[i][j]                otherwise
//     //
//     // So A is identity except column H_index is modified.

//     float A[15][15] = {0.0f};
//     for (int i = 0; i < 15; ++i) {
//         A[i][i] = 1.0f;
//         A[i][H_index] -= K[i] * sign;  // modifies the single column
//     }

//     // 2) temp = A * P
//     float temp[15][15] = {0.0f};
//     for (int i = 0; i < 15; ++i) {
//         for (int j = 0; j < 15; ++j) {
//             float sum = 0.0f;
//             for (int k = 0; k < 15; ++k) {
//                 sum += A[i][k] * P[k][j];
//             }
//             temp[i][j] = sum;
//         }
//     }

//     // 3) P_new = temp * A^T
//     float P_new[15][15] = {0.0f};
//     for (int i = 0; i < 15; ++i) {
//         for (int j = 0; j < 15; ++j) {
//             float sum = 0.0f;
//             for (int k = 0; k < 15; ++k) {
//                 sum += temp[i][k] * A[j][k]; // A^T[k][j] = A[j][k]
//             }
//             P_new[i][j] = sum;
//         }
//     }

//     // 4) Add KRK^T term (rank-1 update since scalar R):
//     // P_new += K * R * K^T
//     for (int i = 0; i < 15; ++i) {
//         for (int j = 0; j < 15; ++j) {
//             P_new[i][j] += K[i] * R * K[j];
//         }
//     }

//     // 5) Copy back + symmetrize
//     for (int i = 0; i < 15; ++i) {
//         for (int j = 0; j < 15; ++j) {
//             P[i][j] = P_new[i][j];
//         }
//     }

//     // Enforce symmetry
//     for (int i = 0; i < 15; ++i) {
//         for (int j = i + 1; j < 15; ++j) {
//             float s = 0.5f * (P[i][j] + P[j][i]);
//             P[i][j] = s;
//             P[j][i] = s;
//         }
//     }
// }


