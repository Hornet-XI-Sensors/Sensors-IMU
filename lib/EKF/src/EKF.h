#ifndef EKF_H
#define EKF_H

#include <array>

class EKF {
public:
    static constexpr int STATE_SIZE = 9;
    static constexpr int MEAS_SIZE = 6; // accel + magnetometer



    using StateVector = std::array<float, STATE_SIZE>;
    using StateMatrix = std::array<std::array<float, STATE_SIZE>, STATE_SIZE>;
    using MeasVector  = std::array<float, MEAS_SIZE>;
    using MeasMatrix  = std::array<std::array<float, STATE_SIZE>, MEAS_SIZE>;
    using MeasCov     = std::array<std::array<float, MEAS_SIZE>, MEAS_SIZE>;

    EKF();

    void predict(const StateVector &u, float dt);
    void update(const MeasVector &z);

    StateVector getState() const { return x; }

private:
    StateVector x;      // state vector
    StateMatrix P;      // covariance
    StateMatrix Q;      // process noise
    MeasMatrix H;       // measurement matrix
    MeasCov R;          // measurement noise

    StateMatrix matAdd(const StateMatrix &a, const StateMatrix &b) const;
};

#endif // EKF_H
