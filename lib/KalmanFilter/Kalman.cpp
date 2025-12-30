#include "Kalman.h"

Kalman::Kalman() {
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f;
    bias = 0.0f;

    P[0][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
}

float Kalman::update(float newAngle, float newRate, float dt) {
    // Predict
    // Step 1: angle += dt * (newRate - bias)
    angle += dt * (newRate - bias);

    // Step 2: update estimation error covariance - Project the error covariance ahead
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    // Step 3: Calculate Kalman gain
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Step 4: Update angle and bias - Angle is corrected with measurement
    float y = newAngle - angle;
    angle += K[0] * y;
    bias  += K[1] * y;

    // Step 5: Update error covariance matrix
    float P00 = P[0][0];
    float P01 = P[0][1];
    float P10 = P[1][0];
    float P11 = P[1][1];

    P[0][0] = P00 - K[0] * P00;
    P[0][1] = P01 - K[0] * P01;
    P[1][0] = P10 - K[1] * P00;
    P[1][1] = P11 - K[1] * P01;

    return angle;
}
