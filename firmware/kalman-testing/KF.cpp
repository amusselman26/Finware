#include <cstdint>

struct KalmanCV1D {
    // State: [position, velocity]
    float p = 0.0f;
    float v = 0.0f;

    // Covariance P (2x2)
    float P00 = 1000.0f, P01 = 0.0f;
    float P10 = 0.0f,    P11 = 1000.0f;

    // Timestep
    float dt = 0.01f;

    // Process noise covariance Q (2x2) (tuning knobs)
    float Q00 = 0.01f, Q01 = 0.0f;
    float Q10 = 0.0f,  Q11 = 0.1f;

    // Measurement noise variance R (scalar) (tuning knob)
    float R = 1.0f;

    KalmanCV1D(float dt_, float q_pos, float q_vel, float r_pos)
        : dt(dt_)
    {
        Q00 = q_pos;
        Q11 = q_vel;
        R   = r_pos;
    }

    // Predict step (constant velocity model)
    void predict() {
        // x = F x, where F = [[1, dt],[0,1]]
        p = p + dt * v;
        // v unchanged

        // P = F P F^T + Q
        //
        // Let F = [ [1, dt],
        //           [0,  1] ]
        //
        // Compute F*P first:
        // A = F*P
        const float A00 = P00 + dt * P10;
        const float A01 = P01 + dt * P11;
        const float A10 = P10;
        const float A11 = P11;

        // Then P = A * F^T + Q, where F^T = [[1,0],[dt,1]]
        const float newP00 = A00 + dt * A01 + Q00;
        const float newP01 = A01 + Q01;
        const float newP10 = A10 + dt * A11 + Q10;
        const float newP11 = A11 + Q11;

        P00 = newP00; P01 = newP01;
        P10 = newP10; P11 = newP11;
    }

    // Update step with a position measurement z
    void update(float z_pos) {
        // Measurement model: z = H x + noise, H = [1, 0]
        // Innovation: y = z - Hx = z - p
        const float y = z_pos - p;

        // Innovation covariance: S = H P H^T + R = P00 + R (scalar)
        const float S = P00 + R;

        // Kalman gain: K = P H^T S^-1
        // Since H^T = [1;0], K = [P00; P10] / S
        const float invS = 1.0f / S;
        const float K0 = P00 * invS;
        const float K1 = P10 * invS;

        // State update: x = x + K y
        p += K0 * y;
        v += K1 * y;

        // Cov update (Joseph form is more numerically stable, but this is fine for 2x2 + scalar H):
        // P = (I - K H) P
        // With H=[1 0], (I-KH) = [[1-K0, 0],[-K1, 1]]
        const float newP00 = (1.0f - K0) * P00;
        const float newP01 = (1.0f - K0) * P01;
        const float newP10 = P10 - K1 * P00;
        const float newP11 = P11 - K1 * P01;

        P00 = newP00; P01 = newP01;
        P10 = newP10; P11 = newP11;
    }
};

// --- Example usage ---
#ifdef KALMAN_DEMO_MAIN
#include <iostream>

int main() {
    KalmanCV1D kf(/*dt=*/0.1f, /*q_pos=*/0.01f, /*q_vel=*/0.1f, /*r_pos=*/2.0f);

    float measurements[] = {0.2f, 0.9f, 2.1f, 2.9f, 4.2f};

    for (float z : measurements) {
        kf.predict();
        kf.update(z);
        std::cout << "z=" << z
                  << " -> p=" << kf.p
                  << ", v=" << kf.v << "\n";
    }
    return 0;
}
#endif
