// // Psuedocode for now
// #pragma once
// #include <vector>
// #include <iostream>

// class KalmanFilter {
//     public:
//         void predict();
//         void update();

//     private:
//         // Integrate IMU data, gx, gy, gz are angular velocities in rad/s, dt is time step in seconds
//         void integrateIMU(float gx, float gy, float gz, float dt, float gyro_bias_x, float gyro_bias_y, float gyro_bias_z);

//         // Convert accelerometer data from body frame to earth NED frame using current orientation
//         void convertAccelToNED(float ax, float ay, float az);

//         // Attitude angles in radians
//         float theta_x = 0.0f;
//         float theta_y = 0.0f;
//         float theta_z = 0.0f;

//         // Change in angles from IMU integration
//         std::vector<float> dtheta = {0.0f, 0.0f, 0.0f};
//         float mag_dtheta = 0.0f;
//         std::vector<float> dquat = {1.0f, 0.0f, 0.0f, 0.0f}; // Quaternion representation
//         std::vector<float> quat_attitude = {1.0f, 0.0f, 0.0f, 0.0f}; // Current attitude quaternion

//         float rotation_matrix[3][3] = {
//             {1.0f, 0.0f, 0.0f},
//             {0.0f, 1.0f, 0.0f},
//             {0.0f, 0.0f, 1.0f}
//         };
//         std::vector<float> a_ned = {0.0f, 0.0f, 0.0f}; // Acceleration in NED frame
//         std::vector<float> vel_ned = {0.0f, 0.0f, 0.0f}; // Velocity in NED frame
//         std::vector<float> pos_ned = {0.0f, 0.0f, 0.0f}; // Position in NED frame

//         // Error covariance matrix
//         float dx[5][3] = {0};
//         float P[15][15] = {0};
// };