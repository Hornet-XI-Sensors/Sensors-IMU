// Simple 1D Kalman filter for angle estimation
#ifndef KALMAN_H
#define KALMAN_H

class Kalman {
public:
    Kalman();
    // Call this every step: newAngle from accel/mag, newRate from gyro, dt in seconds
    float update(float newAngle, float newRate, float dt);
    void setQangle(float q) { Q_angle = q; }
    void setQbias(float q) { Q_bias = q; }
    void setRmeasure(float r) { R_measure = r; }
private:
    float Q_angle; // process noise variance for the accelerometer
    float Q_bias;  // process noise variance for the gyro bias
    float R_measure; // measurement noise variance

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state
    float bias;  // The gyro bias calculated by the Kalman filter - part of the 2x1 state

    float P[2][2]; // Error covariance matrix
};

#endif
